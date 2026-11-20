/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file ortho_map.cpp
 * @brief Load fixed-session photos and render Simple/Great map pixels.
 * @details Great computes seam ownership once from a bounded global overview,
 * then reuses that plan for every output tile. Global decisions avoid seams
 * changing at tile borders; full-resolution footprint fallback preserves
 * narrow source coverage that a coarse overview might miss.
 */
#include "ortho_map.h"
#include "jpeg_encoder.h"
#include "parallel_rows.h"
#include "jpeg_stream.h"
#include "usher_sources.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace pangea_survey {
namespace {
using fsPath = std::filesystem::path;
std::vector<std::string> csvFields(const std::string& line) {
    std::vector<std::string> result;
    std::string value;
    bool quoted = false, closed = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quoted) {
            if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') { value += '"'; ++i; }
            else if (c == '"') { quoted = false; closed = true; }
            else value += c;
        } else if (c == ',') { result.push_back(value); value.clear(); closed = false; }
        else if (c == '"' && value.empty() && !closed) quoted = true;
        else if (closed || c == '"') throw std::runtime_error("Malformed source CSV quoting");
        else if (c != '\r') value += c;
    }
    if (quoted) throw std::runtime_error("Unclosed source CSV quote");
    result.push_back(value);
    return result;
}
std::vector<OrthoSource> readSources(const fsPath& path, const fsPath& sourceDirectory = {}) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open orthorectification source CSV: " + path.u8string());
    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("Empty source CSV");
    const auto header = csvFields(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i)
        if (!columns.emplace(header[i], i).second) throw std::runtime_error("Duplicate source CSV column");
    const bool haveUtc = columns.count("capture_utc") != 0;
    for (const auto& group : {std::vector<const char*>{"fx_px", "fy_px", "cx_px", "cy_px"},
                             std::vector<const char*>{"lens_k1", "lens_k2", "lens_blue_scale", "lens_red_scale"}}) {
        const auto present = std::count_if(group.begin(), group.end(),
            [&](const char* key) { return columns.count(key) != 0; });
        if (present != 0 && present != static_cast<std::ptrdiff_t>(group.size()))
            throw std::runtime_error("Source CSV has incomplete optical calibration columns");
    }
    if ((columns.count("utc_absolute_seconds") != 0) != haveUtc ||
        (columns.count("utc_read_qpc_100ns") != 0) != haveUtc)
        throw std::runtime_error("Source CSV has incomplete capture UTC columns");
    if ((columns.count("tile_row") != 0) != (columns.count("tile_column") != 0))
        throw std::runtime_error("Source CSV has incomplete capture grid columns");
    std::vector<OrthoSource> sources;
    std::map<fsPath, bool> paths;
    // One fixed camera/altitude/time session keeps the intrinsic and map scale valid for every row.
    while (std::getline(input, line)) {
        const auto fields = csvFields(line);
        if (fields.size() != header.size()) throw std::runtime_error("Truncated/malformed source CSV row");
        const auto text = [&](const char* key) -> const std::string& {
            const auto it = columns.find(key);
            if (it == columns.end()) throw std::runtime_error(std::string("Source CSV lacks ") + key);
            return fields[it->second];
        };
        const auto get = [&](const char* key) { return number(text(key), key); };
        if (text("trigger") != "box_tile") continue;
        const fsPath name = std::filesystem::u8path(text("filename"));
        if (name.empty() || name.has_parent_path() || name.extension() != ".jpg")
            throw std::runtime_error("Source CSV filename must be a local JPEG basename");
        OrthoSource s;
        s.path = (sourceDirectory.empty() ? path.parent_path() : sourceDirectory) / name;
        if (!paths.emplace(s.path, true).second) throw std::runtime_error("Duplicate source photo");
        s.latitude = get("observed_latitude"); s.longitude = get("observed_longitude");
        s.altitude = get("observed_altitude_m");
        s.pitch = get("observed_pitch_deg"); s.bank = get("observed_bank_deg");
        s.heading = get("observed_heading_deg"); s.fov = get("sdk_fov_deg");
        range(s.latitude, -80, 84, "source latitude"); range(s.longitude, -180, 180, "source longitude");
        range(s.fov, 10, 120, "source vertical FOV");
        if (text("projection") != "perspective" || !text("map_epsg").empty() ||
            std::abs(s.pitch - 90) > 0.00002 || std::abs(std::remainder(s.heading - s.bank, 360.0)) > 0.001 ||
            get("commanded_pitch_deg") != 90 || get("commanded_bank_deg") != 0 ||
            get("commanded_heading_deg") != 0)
            throw std::runtime_error("Orthorectification requires unrotated, confirmed true-north nadir source photos");
        const auto integer = [&](const char* key, double low, double high) {
            const double n = get(key); range(n, low, high, key);
            if (std::floor(n) != n) throw std::runtime_error(std::string("Noninteger ") + key);
            return n;
        };
        s.width = static_cast<int>(integer("width", 64, 8192));
        s.height = static_cast<int>(integer("height", 64, 8192));
        if (columns.count("tile_row"))
            s.captureCell = std::make_pair(static_cast<int>(integer("tile_row", 0, 100000)),
                                          static_cast<int>(integer("tile_column", 0, 100000)));
        const double sw = integer("source_width", 64, 16384);
        const double sh = integer("source_height", 64, 16384);
        const double cw = get("crop_width"), ch = get("crop_height");
        const double x = get("crop_x"), y = get("crop_y");
        if (cw <= 0 || ch <= 0 || x < 0 || y < 0 || x + cw > sw + 1e-6 || y + ch > sh + 1e-6 ||
            std::abs(cw / s.width - ch / s.height) > 1e-6)
            throw std::runtime_error("Invalid source crop/intrinsics");
        s.focal = sh / (2 * std::tan(s.fov * pi / 360)) * s.width / cw;
        s.cx = (sw / 2 - x) * s.width / cw - 0.5;
        s.cy = (sh / 2 - y) * s.height / ch - 0.5;
        if (columns.count("lens_profile")) {
            const auto& profile = text("lens_profile");
            if (profile != "simconnect" && profile != "raspberry_pi_ai")
                throw std::runtime_error("Unsupported source lens profile");
            s.distorted = profile == "raspberry_pi_ai";
        }
        if (columns.count("start_ground_m") && !text("start_ground_m").empty())
            s.referencePlane = get("start_ground_m");
        if (columns.count("fx_px")) {
            const double fx = get("fx_px"), fy = get("fy_px");
            if (std::abs(fx - s.focal) > .01 || std::abs(fy - fx) > .01 ||
                std::abs(get("cx_px") - s.cx) > .01 || std::abs(get("cy_px") - s.cy) > .01)
                throw std::runtime_error("Source saved-image intrinsics disagree with capture framing");
        }
        if (columns.count("lens_k1")) {
            s.lens = {get("lens_k1"), get("lens_k2"), get("lens_blue_scale"), get("lens_red_scale")};
            range(s.lens.k1, -.05, 0, "source lens_k1"); range(s.lens.k2, 0, .01, "source lens_k2");
            range(s.lens.blueScale, .999, 1.001, "source lens_blue_scale");
            range(s.lens.redScale, .999, 1.001, "source lens_red_scale");
            if (!s.distorted && (s.lens.k1 != 0 || s.lens.k2 != 0 ||
                s.lens.blueScale != 1 || s.lens.redScale != 1))
                throw std::runtime_error("Rectilinear source has nonidentity lens effects");
        }
        const auto timestamp = [&](const char* key) {
            const auto n = integer(key, 1, 9e15);
            return static_cast<long long>(n);
        };
        s.poseTime = timestamp("pose_received_qpc_100ns"); s.frameTime = timestamp("frame_qpc_100ns");
        const auto requested = timestamp("request_qpc_100ns");
        if (s.poseTime > s.frameTime || requested > s.frameTime || s.frameTime - s.poseTime > 10000000)
            throw std::runtime_error("Source camera/frame timestamps are missing, stale or reversed");
        if (haveUtc && !text("capture_utc").empty()) {
            s.captureUtc = parseUtcDateTime(text("capture_utc"));
            s.utcAbsoluteSeconds = get("utc_absolute_seconds");
            s.utcRead100ns = timestamp("utc_read_qpc_100ns");
            if (!captureUtcSampleValid(*s.captureUtc, s.utcAbsoluteSeconds, s.utcRead100ns, s.frameTime))
                throw std::runtime_error("Source capture UTC readback is stale, reversed or outside the locked second");
        }
        if (haveUtc && !s.captureUtc &&
            (!text("utc_absolute_seconds").empty() || !text("utc_read_qpc_100ns").empty()))
            throw std::runtime_error("Source capture UTC evidence lacks its target date/time");
        if (!std::filesystem::is_regular_file(s.path)) throw std::runtime_error("Missing source JPEG: " + s.path.u8string());
        if (!sources.empty() && (std::abs(s.altitude - sources[0].altitude) > 0.01 ||
            s.width != sources[0].width || s.height != sources[0].height ||
            std::abs(s.focal / sources[0].focal - 1) > 1e-6 ||
            std::abs(s.cx - sources[0].cx) > .01 || std::abs(s.cy - sources[0].cy) > .01 ||
            s.distorted != sources[0].distorted ||
            (s.distorted && (s.lens.k1 != sources[0].lens.k1 || s.lens.k2 != sources[0].lens.k2 ||
                s.lens.blueScale != sources[0].lens.blueScale || s.lens.redScale != sources[0].lens.redScale))))
            throw std::runtime_error("Source set changed altitude, lens or viewport; use one fixed-MSL capture session");
        if (!sources.empty() && (s.captureUtc.has_value() != sources[0].captureUtc.has_value() ||
            (s.captureUtc && s.captureUtc->text != sources[0].captureUtc->text)))
            throw std::runtime_error("Source set changed capture UTC; use one fixed-time capture session");
        sources.push_back(s);
    }
    if (input.bad()) throw std::runtime_error("Cannot read source CSV");
    if (sources.empty()) throw std::runtime_error("No box photographs in source CSV");
    return sources;
}
struct RectificationCache {
    std::array<double, 9> calibration{};
    std::array<cv::Mat, 4> mapX, mapY;
    std::array<int, 4> inset{};

    bool prepare(const OrthoSource& source, std::size_t pixels) {
        constexpr std::size_t maximumBytes = 96 * 1024 * 1024;
        if (pixels > maximumBytes / (8 * sizeof(float))) {
            *this = {};
            return false;
        }
        const std::array<double, 9> current{double(source.width), double(source.height),
            source.focal, source.cx, source.cy, source.lens.k1, source.lens.k2,
            source.lens.blueScale, source.lens.redScale};
        if (current != calibration) {
            *this = {};
            calibration = current;
        }
        return true;
    }
};

struct Photo {
    OrthoSource info;
    int validInset = 0;
    double east = 0, north = 0, c = 1, s = 0, scale = 1;
    std::vector<unsigned char> bgra;
    fsPath raster;
    void load(int maxSide = 0, RectificationCache* rectification = nullptr) {
        JpegStream decoder;
        auto image = decoder.read(info.path, info.width, info.height, info.distorted ? 0 : maxSide);
        info.width = image.width; info.height = image.height;
        info.focal *= image.scale;
        info.cx = (info.cx + .5) * image.scale - .5;
        info.cy = (info.cy + .5) * image.scale - .5;
        const auto count = static_cast<std::size_t>(info.width) * info.height;
        bgra = std::move(image.pixels);
        if (info.distorted) {
            if (rectification && !rectification->prepare(info, count)) rectification = nullptr;
            cv::Mat input(info.height, info.width, CV_8UC4, bgra.data());
            std::vector<cv::Mat> channels;
            cv::split(input, channels);
            const double scales[] = {info.lens.blueScale, 1, info.lens.redScale, 1};
            const cv::Matx33d ideal(info.focal, 0, info.cx, 0, info.focal, info.cy, 0, 0, 1);
            for (int channel = 0; channel < 4; ++channel) {
                const double focal = info.focal * scales[channel];
                const cv::Matx33d sensor(focal, 0, info.cx, 0, focal, info.cy, 0, 0, 1);
                cv::Mat mx, my, corrected;
                int channelInset = 0;
                if (rectification) {
                    if (rectification->mapX[channel].empty()) {
                        for (int previous = 0; previous < channel; ++previous) {
                            if (scales[previous] == scales[channel]) {
                                rectification->mapX[channel] = rectification->mapX[previous];
                                rectification->mapY[channel] = rectification->mapY[previous];
                                rectification->inset[channel] = rectification->inset[previous];
                                break;
                            }
                        }
                    }
                    mx = rectification->mapX[channel]; my = rectification->mapY[channel];
                    channelInset = rectification->inset[channel];
                }
                if (mx.empty()) {
                    cv::initUndistortRectifyMap(sensor, cv::Vec<double, 5>(info.lens.k1, info.lens.k2, 0, 0, 0),
                        cv::noArray(), ideal, cv::Size(info.width, info.height), CV_32FC1, mx, my);
                    for (int y = 0; y < info.height; ++y) for (int x = 0; x < info.width; ++x) {
                        const double u = mx.at<float>(y, x), v = my.at<float>(y, x);
                        if (!std::isfinite(u) || !std::isfinite(v))
                            throw std::runtime_error("Nonfinite rectified camera ray");
                        if (u < 0 || v < 0 || u > info.width - 1 || v > info.height - 1)
                            channelInset = std::max(channelInset,
                                std::min({x, y, info.width - 1 - x, info.height - 1 - y}) + 1);
                    }
                    if (rectification) {
#if defined(__x86_64__)
                        cv::convertMaps(mx, my, rectification->mapX[channel], rectification->mapY[channel],
                                        CV_16SC2, false);
                        mx = rectification->mapX[channel]; my = rectification->mapY[channel];
#else
                        rectification->mapX[channel] = mx; rectification->mapY[channel] = my;
#endif
                        rectification->inset[channel] = channelInset;
                    }
                }
                validInset = std::max(validInset, channelInset);
                cv::remap(channels[channel], corrected, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                channels[channel] = corrected;
            }
            cv::Mat corrected;
            cv::merge(channels, corrected);
            if (!corrected.isContinuous() || corrected.total() * corrected.elemSize() != bgra.size())
                throw std::runtime_error("Rectified photo storage differs from the decoded image");
            for (std::size_t offset = 0; offset < bgra.size(); ++offset)
                bgra[offset] = corrected.data[offset];
            if (2 * validInset > std::min(info.width, info.height) - 64)
                throw std::runtime_error("Lens rectification leaves too few fully supported image pixels");
        }
    }
    bool project(double e, double n, double z, double& x, double& y, int border = 5) const {
        border += validInset;
        const double depth = info.altitude - z;
        if (depth <= 5) return false;
        const double de = (e - east) / scale, dn = (n - north) / scale;
        if (info.camera) {
            const auto pixel = info.camera->groundToPixel(depth) * cv::Vec3d(c * de + s * dn, -s * de + c * dn, 1);
            if (pixel[2] <= 1e-9) return false;
            x = pixel[0] / pixel[2]; y = pixel[1] / pixel[2];
            return std::isfinite(x) && std::isfinite(y) && x >= border && y >= border &&
                   x < info.width - border - 1 && y < info.height - border - 1;
        }
        x = info.cx + info.focal * (c * de + s * dn) / depth;
        y = info.cy - info.focal * (-s * de + c * dn) / depth;
        return x >= border && y >= border && x < info.width - border - 1 && y < info.height - border - 1;
    }
    void color(double x, double y, unsigned char* pixel) const {
        const int ix = static_cast<int>(x), iy = static_cast<int>(y);
        const double dx = x - ix, dy = y - iy;
        const auto i = (static_cast<std::size_t>(iy) * info.width + ix) * 4;
        for (std::size_t k = 0; k < 3; ++k)
            pixel[k] = static_cast<unsigned char>(std::clamp(std::lround((1 - dy) * ((1 - dx) * bgra[i + k] +
                dx * bgra[i + k + 4]) + dy * ((1 - dx) * bgra[i + info.width * 4 + k] +
                dx * bgra[i + info.width * 4 + k + 4])), 0L, 255L));
        pixel[3] = 255;
    }
};
using Photos = std::vector<std::shared_ptr<Photo>>;

}

OrthoLayout orthoLayout(const MapConfig& config) {
    validateJpegSize(config.jpegWidth, config.jpegHeight);
    if (config.jpegWidth > 4096 || config.jpegHeight > 4096)
        throw std::runtime_error("Orthorectified tile dimensions must be at most 4096");
    range(config.mapPixelSize, .05, 2, "map_pixel_size_m");
    const MapProjection projection((config.start.lat + config.end.lat) / 2,
                                   (config.start.lon + config.end.lon) / 2);
    const auto corner = projection.forward(config.start.lat, config.start.lon);
    OrthoLayout result{corner.first, corner.second, corner.first, corner.second, 0, 0, projection.epsg()};
    for (const auto lat : {config.start.lat, config.end.lat}) for (const auto lon : {config.start.lon, config.end.lon}) {
        const auto p = projection.forward(lat, lon);
        result.west = std::min(result.west, p.first); result.east = std::max(result.east, p.first);
        result.north = std::max(result.north, p.second); result.south = std::min(result.south, p.second);
    }
    const double columns = std::ceil((result.east - result.west) / (config.jpegWidth * config.mapPixelSize));
    const double rows = std::ceil((result.north - result.south) / (config.jpegHeight * config.mapPixelSize));
    if (!std::isfinite(columns) || !std::isfinite(rows) || columns < 1 || rows < 1 || columns * rows > 100000)
        throw std::runtime_error("Invalid/excessive orthophoto tile count");
    result.columns = static_cast<std::size_t>(columns); result.rows = static_cast<std::size_t>(rows);
    result.east = result.west + columns * config.jpegWidth * config.mapPixelSize;
    result.south = result.north - rows * config.jpegHeight * config.mapPixelSize;
    return result;
}

/** @brief Fill unsupported areas using a global push-pull colour pyramid.
 * @param premultiplied Four-channel float overview with colour weighted by alpha.
 * @param workers Worker count for row processing.
 * @param stopped Cooperative cancellation predicate.
 * @return Completed four-channel float overview with alpha set to one.
 * @details Downsample weighted colour and support together; pull coarser
 * estimates into gaps without introducing tile-local fill boundaries.
 * @throws std::runtime_error On invalid input, absent support, or cancellation. */
static cv::Mat completeMosaicPyramid(const cv::Mat& premultiplied, unsigned workers,
                                      const std::function<bool()>& stopped) {
    if (premultiplied.empty() || premultiplied.type() != CV_32FC4)
        throw std::runtime_error("Mosaic completion requires premultiplied float colour/alpha");
    // Push/pull from one global pyramid so completion crosses source gaps, not tile boundaries.
    std::vector<cv::Mat> levels{premultiplied.clone()};
    while (levels.back().cols > 1 || levels.back().rows > 1) {
        if (stopped()) throw std::runtime_error("Mosaic completion canceled");
        cv::Mat reduced;
        cv::pyrDown(levels.back(), reduced);
        levels.push_back(std::move(reduced));
    }
    auto& root = levels.back().at<cv::Vec4f>(0, 0);
    if (!std::isfinite(root[3]) || root[3] <= 0)
        throw std::runtime_error("Cannot complete a mosaic without any source colour");
    for (int channel = 0; channel < 3; ++channel) root[channel] /= root[3];
    root[3] = 1;
    for (std::size_t level = levels.size() - 1; level > 0; --level) {
        cv::Mat expanded;
        cv::pyrUp(levels[level], expanded, levels[level - 1].size());
        auto& destination = levels[level - 1];
        parallelRows(destination.rows, [&](int row) {
            if (stopped()) throw std::runtime_error("Mosaic completion canceled");
            auto* values = destination.ptr<cv::Vec4f>(row);
            const auto* coarse = expanded.ptr<cv::Vec4f>(row);
            for (int column = 0; column < destination.cols; ++column) {
                const float alpha = std::clamp(values[column][3], 0.0f, 1.0f);
                for (int channel = 0; channel < 3; ++channel)
                    values[column][channel] += (1 - alpha) * coarse[column][channel];
                values[column][3] = 1;
            }
        }, workers);
    }
    return levels.front();
}

struct Orthorectifier::Impl {
    MapConfig config;
    MapProjection projection;
    std::vector<OrthoSource> sources;
    double west = 0, north = 0;
    std::size_t cols = 0, rows = 0;
    struct CachedBlock { OrthoImage image; std::size_t touched = 0; };
    std::map<std::pair<std::size_t, std::size_t>, CachedBlock> blocks;
    std::size_t blockClock = 0;
    struct SourcePhoto { std::shared_ptr<Photo> photo; std::size_t touched; };
    std::map<std::size_t, SourcePhoto> sourcePhotos;
    std::size_t sourcePhotoBytes = 0, sourcePhotoClock = 0;
    std::shared_ptr<Photo> sourcePhoto(std::size_t index);
    struct MosaicView {
        std::size_t source;
        cv::Rect rectangle;
        cv::Mat seam;
        double gain = 1;
        cv::Rect footprint;
    };
    struct CachedPhoto { Photo photo; std::size_t touched = 0; };
    std::vector<MosaicView> mosaicViews;
    std::map<std::size_t, CachedPhoto> mosaicPhotos;
    std::size_t photoClock = 0, photoBytes = 0;
    int seamScale = 0;
    cv::Mat completedMosaic, mosaicSourceDistance;
    std::vector<std::filesystem::path> sourceRasters;
    bool mosaicPrepared = false, diskSourcesPrepared = false;
    Photo& mosaicPhoto(std::size_t index);
    void prepareDiskSources(const std::function<bool()>& stopped);
    cv::Rect mosaicRectangle(const OrthoSource& source, int scale, bool seamPlan = false) const;
    cv::Matx33d mosaicWarp(const Photo& photo, const cv::Rect& rectangle, int scale, cv::Mat& image, cv::Mat& mask) const;
    void prepareMosaic(const std::function<bool()>& stopped);
    void prepareMosaicCompletion(const std::vector<cv::UMat>& images, const std::vector<cv::UMat>& originalMasks,
                                const std::vector<std::size_t>& seamViews, const std::function<bool()>& stopped);
    OrthoImage renderMosaicRegion(const MapTile& map, int width, int height, const std::function<bool()>& stopped);
    OrthoImage renderRegion(const MapTile& map, int width, int height, const std::function<bool()>& stopped);
    /** @brief Admit a source set and establish the shared projection and grid.
     * @param c Requested configuration.
     * @param csv Source manifest used by the selected input path.
     * @details Reconcile the manifest count and choose a reference plane before
     * rendering; camera heights must leave clearance above that plane.
     * @throws std::runtime_error On invalid input, engine, or layout. */
    Impl(const MapConfig& c, const std::filesystem::path& csv) : config(c),
        projection((c.start.lat + c.end.lat) / 2, (c.start.lon + c.end.lon) / 2),
        sources(c.photoDirectory.empty() ? readSources(csv, c.sourceDirectory) : readUsherSources(c, csv)) {
        if (c.expectedSaved && sources.size() != *c.expectedSaved)
            throw std::runtime_error("Capture manifest saved count disagrees with source CSV box photographs");
        const bool mosaic = c.orthoEngine == "mosaic";
        if (c.orthoEngine != "simple" && !mosaic)
            throw std::runtime_error("Unsupported orthomap engine");
        if (mosaic) config.referencePlane = c.mosaicReferencePlane;
        if ((config.orthoEngine == "simple" || mosaic) && !std::isfinite(config.referencePlane))
            config.referencePlane = sources[0].referencePlane;
        if ((config.orthoEngine == "simple" || mosaic) && (!std::isfinite(config.referencePlane) ||
            config.referencePlane >= sources[0].altitude - 5))
            throw std::runtime_error("Mosaicking requires a valid capture reference plane or the mode's reference_plane_amsl_m");
        const auto layout = orthoLayout(c);
        west = layout.west; north = layout.north; cols = layout.columns; rows = layout.rows;
    }
};
Orthorectifier::Orthorectifier(const MapConfig& config, const std::filesystem::path& csv)
    : impl_(std::make_unique<Impl>(config, csv)) {}
Orthorectifier::~Orthorectifier() = default;
std::size_t Orthorectifier::columns() const { return impl_->cols; }
std::size_t Orthorectifier::rows() const { return impl_->rows; }
std::size_t Orthorectifier::size() const { return columns() * rows(); }
std::optional<UtcDateTime> Orthorectifier::captureUtc() const { return impl_->sources.front().captureUtc; }
std::size_t Orthorectifier::sourceCount() const { return impl_->sources.size(); }
std::string Orthorectifier::sourceFingerprint(const std::function<bool()>& stopped) const {
    std::ostringstream identity;
    for (const auto& source : impl_->sources)
        identity << std::filesystem::canonical(source.path).generic_string() << '\n'
                 << fileDigest(source.path, stopped) << '\n';
    const auto text = identity.str();
    return dataDigest(text.data(), text.size());
}

/** @brief Get a pinned decoded photo for the simple renderer.
 * @param index Index in the admitted source list.
 * @return Shared ownership of the decoded, projection-ready photo.
 * @details Evict least-recently-used entries only when the cache is their sole
 * owner; active renders must retain their pixel storage.
 * @throws std::runtime_error If the decoded-photo budget cannot be met. */
std::shared_ptr<Photo> Orthorectifier::Impl::sourcePhoto(std::size_t index) {
    auto& cache = sourcePhotos;
    auto found = cache.find(index);
    if (found == cache.end()) {
        const auto& source = sources.at(index);
        const auto bytes = static_cast<std::size_t>(source.width) * source.height * 4;
        const std::size_t budget = std::size_t{512} * 1024 * 1024;
        if (bytes > budget) throw std::runtime_error("Source photo exceeds decoded-photo cache budget");
        while (sourcePhotoBytes + bytes > budget) {
            auto oldest = cache.end();
            for (auto candidate = cache.begin(); candidate != cache.end(); ++candidate)
                if (candidate->second.photo.use_count() == 1 &&
                    (oldest == cache.end() || candidate->second.touched < oldest->second.touched))
                    oldest = candidate;
            if (oldest == cache.end()) throw std::runtime_error("Active source photos exceed decoded-photo cache budget");
            const auto& photo = *oldest->second.photo;
            sourcePhotoBytes -= photo.bgra.size();
            cache.erase(oldest);
        }
        auto photo = std::make_shared<Photo>();
        photo->info = source;
        const auto coordinate = projection.forward(source.latitude, source.longitude);
        photo->east = coordinate.first; photo->north = coordinate.second;
        const double heading = projection.convergence(source.latitude, source.longitude) * pi / 180;
        photo->c = std::cos(heading); photo->s = std::sin(heading);
        photo->scale = projection.scale(source.latitude, source.longitude);
        photo->load();
        sourcePhotoBytes += photo->bgra.size();
        found = cache.emplace(index, SourcePhoto{std::move(photo), 0}).first;
    }
    found->second.touched = ++sourcePhotoClock;
    return found->second.photo;
}

/** @brief Prepare disk-backed rasters for low-memory rendering.
 * @param stopped Cooperative cancellation predicate.
 * @details Identify each raster with its input digest so reusable prepared
 * data cannot quietly stand in for changed source imagery.
 * @throws std::runtime_error On cancellation or invalid source data or storage. */
void Orthorectifier::Impl::prepareDiskSources(const std::function<bool()>& stopped) {
    if (diskSourcesPrepared) return;
    if (config.workingDirectory.empty()) throw std::runtime_error("Low-memory processing requires a working directory");
    const auto directory = config.preparedSourceDirectory.empty() ? config.workingDirectory / "sources" :
                                                                  config.preparedSourceDirectory;
    std::filesystem::create_directories(directory);
    if (sources.size() > 4096) throw std::runtime_error("Low-memory source count exceeds 4096");
    sourceRasters.resize(sources.size());
    for (std::size_t index = 0; index < sources.size(); ++index) {
        if (stopped()) throw std::runtime_error("Source preparation canceled");
        auto& info = sources[index];
        const auto sourceSize = std::filesystem::file_size(info.path);
        if (!sourceSize || sourceSize > 128ull * 1024 * 1024)
            throw std::runtime_error("Invalid streamed JPEG file size");
        const auto sourceHash = fileDigest(info.path, stopped);
        std::ostringstream identity;
        identity.imbue(std::locale::classic());
        identity << std::setprecision(17) << sourceHash << ' ' << info.width << ' ' << info.height << ' '
                 << info.focal << ' ' << info.cx << ' ' << info.cy << ' ' << info.distorted << ' '
                 << info.lens.k1 << ' ' << info.lens.k2 << ' ' << info.lens.blueScale << ' ' << info.lens.redScale;
        if (info.camera) identity << " fy=" << info.focalY;
        const auto cacheIdentity = "PANGEA_PREPARED_SOURCE_V1\n" + identity.str();
        const auto key = config.preparedSourceDirectory.empty() ? std::to_string(index) :
            dataDigest(cacheIdentity.data(), cacheIdentity.size());
        const auto path = directory / (key + ".bgra");
        const auto metadata = path.string() + ".verified";
        int inset = 0;
        if (std::filesystem::exists(metadata)) {
            std::ifstream record(metadata);
            std::string saved, digest, trailing;
            std::getline(record, saved);
            if (saved != identity.str() || !(record >> inset >> digest) || (record >> trailing) ||
                inset < 0 || 2 * inset > std::min(info.width, info.height) - 64 || fileDigest(path, stopped) != digest)
                throw std::runtime_error("Source checkpoint identity or checksum mismatch; originals retained");
            DiskRaster checked(path, info.width - 2 * inset, info.height - 2 * inset);
        } else {
            storageHeadroom(directory, static_cast<std::uint64_t>(info.width) * info.height * (info.distorted ? 12 : 4));
            const auto decoded = info.distorted ? directory / (key + ".decoded") : path;
            { JpegStream decoder; decoder.decode(info.path, decoded, info.width, info.height, stopped); }
            if (info.distorted) {
                const DiskRaster raster(decoded, info.width, info.height);
                const auto correctedPath = directory / (key + ".corrected");
                std::ofstream output(correctedPath, std::ios::binary | std::ios::trunc);
                const double scales[] = {info.lens.blueScale, 1, info.lens.redScale, 1};
                for (int firstRow = 0; firstRow < info.height; firstRow += 32) {
                    if (stopped()) throw std::runtime_error("Strip rectification canceled");
                    const int rows = std::min(32, info.height - firstRow);
                    cv::Mat stripe(rows, info.width, CV_8UC4);
                    for (int firstColumn = 0; firstColumn < info.width; firstColumn += 256) {
                        const int columns = std::min(256, info.width - firstColumn);
                        for (int channel = 0; channel < 4; ++channel) {
                            const double focalY = info.focalY > 0 ? info.focalY : info.focal;
                            const cv::Matx33d sensor(info.focal * scales[channel], 0, info.cx,
                                                   0, focalY * scales[channel], info.cy, 0, 0, 1);
                            const cv::Matx33d ideal(info.focal, 0, info.cx - firstColumn,
                                                  0, focalY, info.cy - firstRow, 0, 0, 1);
                            cv::Mat mx, my;
                            cv::initUndistortRectifyMap(sensor, cv::Vec<double, 5>(info.lens.k1, info.lens.k2, 0, 0, 0),
                                cv::noArray(), ideal, {columns, rows}, CV_32FC1, mx, my);
                            double minX = info.width - 1, maxX = 0, minY = info.height - 1, maxY = 0;
                            for (int row = 0; row < rows; ++row) for (int column = 0; column < columns; ++column) {
                                const double sx = mx.at<float>(row, column), sy = my.at<float>(row, column);
                                if (!std::isfinite(sx) || !std::isfinite(sy))
                                    throw std::runtime_error("Nonfinite rectified camera ray");
                                if (sx < 0 || sy < 0 || sx > info.width - 1 || sy > info.height - 1)
                                    inset = std::max(inset, std::min({firstColumn + column, firstRow + row,
                                        info.width - 1 - firstColumn - column, info.height - 1 - firstRow - row}) + 1);
                                minX = std::min(minX, std::clamp(sx, 0.0, double(info.width - 1)));
                                maxX = std::max(maxX, std::clamp(sx, 0.0, double(info.width - 1)));
                                minY = std::min(minY, std::clamp(sy, 0.0, double(info.height - 1)));
                                maxY = std::max(maxY, std::clamp(sy, 0.0, double(info.height - 1)));
                            }
                            const int left = std::max(0, static_cast<int>(std::floor(minX)) - 1);
                            const int top = std::max(0, static_cast<int>(std::floor(minY)) - 1);
                            const int right = std::min(info.width, static_cast<int>(std::ceil(maxX)) + 2);
                            const int bottom = std::min(info.height, static_cast<int>(std::ceil(maxY)) + 2);
                            const auto input = raster.read({left, top, right - left, bottom - top});
                            cv::Mat plane, corrected;
                            cv::extractChannel(input, plane, channel);
                            mx -= left; my -= top;
                            cv::remap(plane, corrected, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                            for (int row = 0; row < rows; ++row) for (int column = 0; column < columns; ++column)
                                stripe.at<cv::Vec4b>(row, firstColumn + column)[channel] = corrected.at<unsigned char>(row, column);
                        }
                    }
                    output.write(reinterpret_cast<const char*>(stripe.data), static_cast<std::streamsize>(stripe.total() * 4));
                }
                output.close();
                if (!output || 2 * inset > std::min(info.width, info.height) - 64)
                    throw std::runtime_error("Rectification write failed or unsupported camera crop");
                const DiskRaster corrected(correctedPath, info.width, info.height);
                const auto temporary = path.string() + ".partial";
                std::ofstream cropped(temporary, std::ios::binary | std::ios::trunc);
                for (int row = inset; row < info.height - inset; ++row) {
                    if (stopped()) throw std::runtime_error("Source crop canceled");
                    const auto pixels = corrected.read({inset, row, info.width - 2 * inset, 1});
                    cropped.write(reinterpret_cast<const char*>(pixels.data), pixels.cols * 4);
                }
                cropped.close();
                if (!cropped) throw std::runtime_error("Cannot write rectified raster");
                publishWorkingFile(temporary, path);
                std::filesystem::remove(decoded); std::filesystem::remove(correctedPath);
            }
            if (fileDigest(info.path, stopped) != sourceHash)
                throw std::runtime_error("Photo changed during preparation; no source checkpoint committed");
            saveWorkingText(metadata, identity.str() + "\n" + std::to_string(inset) + " " + fileDigest(path, stopped) + "\n");
        }
        sourceRasters[index] = path;
        info.width -= 2 * inset; info.height -= 2 * inset; info.cx -= inset; info.cy -= inset;
        if (info.camera) info.camera->crop(inset);
        info.distorted = false;
    }
    diskSourcesPrepared = true;
}

void Orthorectifier::prepareSources(const std::function<bool()>& stopped) {
    if (impl_->config.lowMemory) impl_->prepareDiskSources(stopped);
}

/** @brief Get the mosaic's cached projection and image source.
 * @param index Index in the admitted source list.
 * @return Borrowed entry that can be invalidated by cache eviction.
 * @details Low-memory mode retains a raster path instead of decoded pixels;
 * otherwise evict by recency before allocating the next image. */
Photo& Orthorectifier::Impl::mosaicPhoto(std::size_t index) {
    auto cached = mosaicPhotos.find(index);
    if (cached == mosaicPhotos.end()) {
        const auto& info = sources.at(index);
        const auto bytes = static_cast<std::size_t>(info.width) * info.height * 4;
        const auto budget = static_cast<std::size_t>(config.mosaicPhotoCacheMiB) * 1024 * 1024;
        if (!config.lowMemory && bytes > budget) throw std::runtime_error("Mosaic source exceeds its configured photo cache budget");
        while (photoBytes + bytes > budget && !mosaicPhotos.empty()) {
            const auto oldest = std::min_element(mosaicPhotos.begin(), mosaicPhotos.end(),
                [](const auto& a, const auto& b) { return a.second.touched < b.second.touched; });
            photoBytes -= oldest->second.photo.bgra.size();
            mosaicPhotos.erase(oldest);
        }
        CachedPhoto entry; entry.photo.info = info;
        const auto coordinate = projection.forward(info.latitude, info.longitude);
        entry.photo.east = coordinate.first; entry.photo.north = coordinate.second;
        const double heading = projection.convergence(info.latitude, info.longitude) * pi / 180;
        entry.photo.c = std::cos(heading); entry.photo.s = std::sin(heading);
        entry.photo.scale = projection.scale(info.latitude, info.longitude);
        if (config.lowMemory) entry.photo.raster = sourceRasters.at(index);
        else { entry.photo.load(); photoBytes += bytes; }
        cached = mosaicPhotos.emplace(index, std::move(entry)).first;
    }
    cached->second.touched = ++photoClock;
    return cached->second.photo;
}

/** @brief Bound the projected source footprint on a mosaic grid.
 * @param source Source pose and camera geometry.
 * @param scale Number of base pixels per grid pixel.
 * @param seamPlan True when computing the seam-planning footprint.
 * @return Pixel rectangle in the selected mosaic grid.
 * @details Project image corners to the common reference plane so the
 * renderer can avoid processing unrelated regions. */
cv::Rect Orthorectifier::Impl::mosaicRectangle(const OrthoSource& source, int scale, bool seamPlan) const {
    const auto position = projection.forward(source.latitude, source.longitude);
    const double heading = projection.convergence(source.latitude, source.longitude) * pi / 180;
    const double c = std::cos(heading), s = std::sin(heading);
    const double ray = (source.altitude - config.referencePlane) / source.focal *
                       projection.scale(source.latitude, source.longitude);
    const double pixel = config.mapPixelSize * scale;
    double minX = std::numeric_limits<double>::infinity(), minY = minX;
    double maxX = -minX, maxY = -minX;
    for (const double u : {1.0, double(source.width - 2)}) for (const double v : {1.0, double(source.height - 2)}) {
        const auto local = source.camera ? source.camera->ground(u, v, source.altitude - config.referencePlane) :
            cv::Point2d((u - source.cx) * ray, -(v - source.cy) * ray);
        const double unit = source.camera ? projection.scale(source.latitude, source.longitude) : 1;
        const double x = (position.first + unit * (c * local.x - s * local.y) - west) / pixel;
        const double y = (north - position.second - unit * (s * local.x + c * local.y)) / pixel;
        minX = std::min(minX, x); maxX = std::max(maxX, x);
        minY = std::min(minY, y); maxY = std::max(maxY, y);
    }
    const int pad = (64 + scale - 1) / scale + 2;
    double endX = std::ceil(double(cols) * config.jpegWidth / scale) + pad;
    double endY = std::ceil(double(rows) * config.jpegHeight / scale) + pad;
    if (seamPlan) {
        double actualEast = west, actualSouth = north;
        for (const auto latitude : {config.start.lat, config.end.lat})
            for (const auto longitude : {config.start.lon, config.end.lon}) {
                const auto corner = projection.forward(latitude, longitude);
                actualEast = std::max(actualEast, corner.first);
                actualSouth = std::min(actualSouth, corner.second);
            }
        endX = std::ceil((actualEast - west) / pixel) + pad;
        endY = std::ceil((north - actualSouth) / pixel) + pad;
    }
    const double x0 = std::max(std::floor(minX), -double(pad)), y0 = std::max(std::floor(minY), -double(pad));
    const double x1 = std::min(std::ceil(maxX), endX), y1 = std::min(std::ceil(maxY), endY);
    if (x1 <= x0 || y1 <= y0) return {};
    if (std::max({std::abs(x0), std::abs(y0), std::abs(x1), std::abs(y1)}) > 1e8)
        throw std::runtime_error("Great mosaic extent exceeds supported grid coordinates");
    return {static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(x1 - x0), static_cast<int>(y1 - y0)};
}

/** @brief Resample one source over a mosaic rectangle.
 * @param photo Projection-ready source with decoded or disk-backed pixels.
 * @param rectangle Output window in mosaic-grid pixels.
 * @param scale Number of base pixels per grid pixel.
 * @param[out] image Warped source colour.
 * @param[out] mask Support for the warped colour.
 * @return Transform mapping local destination coordinates into the source.
 * @details Evaluate the mapping at pixel centres; use the calibrated camera
 * when present and process in strips to bound source access. */
cv::Matx33d Orthorectifier::Impl::mosaicWarp(const Photo& photo, const cv::Rect& rectangle, int scale,
                                    cv::Mat& image, cv::Mat& mask) const {
    const double pixel = config.mapPixelSize * scale;
    const double e = west + (rectangle.x + .5) * pixel, n = north - (rectangle.y + .5) * pixel;
    double u = 0, v = 0;
    if (!photo.info.camera) photo.project(e, n, config.referencePlane, u, v, 1);
    const double factor = pixel * photo.info.focal / ((photo.info.altitude - config.referencePlane) * photo.scale);
    cv::Matx33d transform(factor * photo.c, -factor * photo.s, u, factor * photo.s, factor * photo.c, v, 0, 0, 1);
    if (photo.info.camera) {
        const double de = (e - photo.east) / photo.scale, dn = (n - photo.north) / photo.scale;
        const double step = pixel / photo.scale;
        transform = photo.info.camera->groundToPixel(photo.info.altitude - config.referencePlane) *
            cv::Matx33d(step * photo.c, -step * photo.s, photo.c * de + photo.s * dn,
                        -step * photo.s, -step * photo.c, -photo.s * de + photo.c * dn, 0, 0, 1);
    }
    const cv::Mat source = photo.raster.empty() ? cv::Mat(photo.info.height, photo.info.width, CV_8UC4,
                         const_cast<unsigned char*>(photo.bgra.data())) : cv::Mat();
    cv::Mat warped;
        warped.create(rectangle.size(), CV_8UC4);
        constexpr int stripRows = 32;
        parallelRows((rectangle.height + stripRows - 1) / stripRows, [&](int strip) {
            const int first = strip * stripRows, last = std::min(rectangle.height, first + stripRows);
            auto local = transform;
            for (int axis = 0; axis < 3; ++axis) local(axis, 2) += local(axis, 1) * first;
            cv::Mat destination = warped.rowRange(first, last);
            const auto warp = [&](auto&& self, cv::Mat output, cv::Matx33d mapping) -> void {
                cv::Mat input = source;
                if (!photo.raster.empty()) {
                    double minX = photo.info.width - 1, maxX = 0, minY = photo.info.height - 1, maxY = 0;
                    double minDepth = std::numeric_limits<double>::infinity(), maxDepth = -minDepth;
                    for (int row : {0, output.rows - 1}) for (int column : {0, output.cols - 1}) {
                        const double depth = mapping(2, 0) * column + mapping(2, 1) * row + mapping(2, 2);
                        minDepth = std::min(minDepth, depth); maxDepth = std::max(maxDepth, depth);
                        const double denominator = std::abs(depth) > 1e-12 ? depth : 1e-12;
                        const double sx = std::clamp((mapping(0, 0) * column + mapping(0, 1) * row + mapping(0, 2)) / denominator, 0.0, double(photo.info.width - 1));
                        const double sy = std::clamp((mapping(1, 0) * column + mapping(1, 1) * row + mapping(1, 2)) / denominator, 0.0, double(photo.info.height - 1));
                        minX = std::min(minX, sx); maxX = std::max(maxX, sx); minY = std::min(minY, sy); maxY = std::max(maxY, sy);
                    }
                    if (maxDepth <= 1e-9) { output.setTo(cv::Scalar(0)); return; }
                    // Across the projective horizon, corner extrema cannot safely bound the source read.
                    if (minDepth <= 1e-9) { minX = minY = 0; maxX = photo.info.width - 1; maxY = photo.info.height - 1; }
                    const int left = std::max(0, static_cast<int>(std::floor(minX)) - 2);
                    const int top = std::max(0, static_cast<int>(std::floor(minY)) - 2);
                    const int right = std::min(photo.info.width, static_cast<int>(std::ceil(maxX)) + 3);
                    const int bottom = std::min(photo.info.height, static_cast<int>(std::ceil(maxY)) + 3);
                    if (static_cast<std::uint64_t>(right - left) * (bottom - top) * 4 >
                        static_cast<std::uint64_t>(config.mosaicPhotoCacheMiB) * 1024 * 1024) {
                        // Split the destination before loading either source window; keep the warp origin aligned.
                        const bool splitRows = output.rows >= output.cols && output.rows > 1;
                        const int split = (splitRows ? output.rows : output.cols) / 2;
                        if (!split) throw std::runtime_error("Single warp sample exceeds raster budget");
                        const cv::Rect first(0, 0, splitRows ? output.cols : split, splitRows ? split : output.rows);
                        const cv::Rect second(splitRows ? 0 : split, splitRows ? split : 0,
                            splitRows ? output.cols : output.cols - split, splitRows ? output.rows - split : output.rows);
                        self(self, output(first), mapping);
                        for (int axis = 0; axis < 3; ++axis)
                            mapping(axis, 2) += mapping(axis, 0) * second.x + mapping(axis, 1) * second.y;
                        self(self, output(second), mapping);
                        return;
                    }
                    input = DiskRaster(photo.raster, photo.info.width, photo.info.height).read({left, top, right - left, bottom - top});
                    for (int axis = 0; axis < 3; ++axis) {
                        mapping(0, axis) -= left * mapping(2, axis);
                        mapping(1, axis) -= top * mapping(2, axis);
                    }
                }
                if (photo.info.camera)
                    cv::warpPerspective(input, output, mapping, output.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
                                        cv::BORDER_REPLICATE);
                else
                    cv::warpAffine(input, output, cv::Matx23d(mapping(0, 0), mapping(0, 1), mapping(0, 2),
                        mapping(1, 0), mapping(1, 1), mapping(1, 2)), output.size(),
                        cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_REPLICATE);
            };
            warp(warp, destination, local);
        }, config.mosaicThreads);
    cv::cvtColor(warped, image, cv::COLOR_BGRA2BGR);
    mask.create(rectangle.size(), CV_8U);
    const int border = 1 + photo.validInset;
    parallelRows(rectangle.height, [&](int y) {
        auto* row = mask.ptr<unsigned char>(y);
        for (int x = 0; x < rectangle.width; ++x) {
            const double depth = transform(2, 0) * x + transform(2, 1) * y + transform(2, 2);
            const double sx = (transform(0, 0) * x + transform(0, 1) * y + transform(0, 2)) / depth;
            const double sy = (transform(1, 0) * x + transform(1, 1) * y + transform(1, 2)) / depth;
            row[x] = depth > 1e-9 && sx >= border && sy >= border && sx < photo.info.width - border - 1 &&
                     sy < photo.info.height - border - 1 ? 255 : 0;
        }
    }, config.mosaicThreads);
    return transform;
}

/** @brief Build the shared seam and exposure plan on first use.
 * @param stopped Cooperative cancellation predicate.
 * @details Coarsen the overview until the global planning workspace fits its
 * limits. Reuse that plan across windows so adjacent renders agree. */
void Orthorectifier::Impl::prepareMosaic(const std::function<bool()>& stopped) {
    if (mosaicPrepared) return;
    if (config.lowMemory) prepareDiskSources(stopped);
    const auto prepareStarted = std::chrono::steady_clock::now();
    seamScale = config.mosaicSeamScale;
    // Increase overview scale until source work and the global grid fit their memory budgets.
    for (;;) {
        std::size_t pixels = 0;
        for (const auto& source : sources) {
            const auto rectangle = mosaicRectangle(source, seamScale, true);
            pixels += static_cast<std::size_t>(rectangle.width) * rectangle.height;
        }
        const auto globalPixels = std::ceil(double(cols) * config.jpegWidth / seamScale) *
                                  std::ceil(double(rows) * config.jpegHeight / seamScale);
        if (pixels <= static_cast<std::size_t>(config.mosaicOverviewPixels) &&
            (!config.lowMemory || globalPixels <= config.mosaicOverviewPixels)) break;
        seamScale *= 2;
        if (seamScale > (config.lowMemory ? 8192 : 256))
            throw std::runtime_error("Mosaic overview exceeds its memory budget; split the map area");
    }
    std::cout << config.mode << ": camera-calibrated mosaic, " << config.mosaicSeamMethod << " seams at 1/" << seamScale
              << ", " << config.mosaicBlendBands << " blending bands; no stereo cost volume.\n";
    std::vector<cv::UMat> images, masks, originalMasks;
    std::vector<cv::Point> corners;
    std::vector<std::size_t> seamViews;
    std::vector<cv::Mat> overviewImages(sources.size());
    std::vector<cv::Mat> overviewMasks(sources.size());
        parallelRows(static_cast<int>(sources.size()), [&](int index) {
            if (stopped()) throw std::runtime_error("Mosaic overview canceled");
            const auto rectangle = mosaicRectangle(sources[index], seamScale, true);
            if (rectangle.empty()) return;
            Photo photo;
            photo.info = sources[index];
            const auto coordinate = projection.forward(photo.info.latitude, photo.info.longitude);
            photo.east = coordinate.first; photo.north = coordinate.second;
            const double heading = projection.convergence(photo.info.latitude, photo.info.longitude) * pi / 180;
            photo.c = std::cos(heading); photo.s = std::sin(heading);
            photo.scale = projection.scale(photo.info.latitude, photo.info.longitude);
            if (config.lowMemory) photo.raster = sourceRasters.at(index);
            else photo.load(config.mosaicOverviewSourceMaxSide);
            mosaicWarp(photo, rectangle, seamScale, overviewImages[index], overviewMasks[index]);
        }, config.mosaicThreads);
    // Seam ownership is global; per-output-tile seam finding would make adjacent tiles disagree.
    for (std::size_t i = 0; i < sources.size(); ++i) {
        if (stopped()) throw std::runtime_error("Great overview canceled");
        const auto rectangle = mosaicRectangle(sources[i], seamScale, true);
        if (mosaicRectangle(sources[i], seamScale).empty()) continue;
        if (mosaicViews.size() == 4096)
            throw std::runtime_error("Great seam plan exceeds 4096 overlapping sources; split the map area");
        // A coarse overview can miss a narrow valid footprint; retain its full-resolution colour fallback.
        mosaicViews.push_back({i, rectangle, {}, 1, mosaicRectangle(sources[i], 1)});
        if (rectangle.empty()) continue;
        cv::Mat image = overviewImages[i], mask = overviewMasks[i];
        if (cv::countNonZero(mask) == 0) continue;
        images.emplace_back(); masks.emplace_back();
        image.copyTo(images.back()); mask.copyTo(masks.back());
        if (config.mosaicFillGaps) { originalMasks.emplace_back(); mask.copyTo(originalMasks.back()); }
        corners.push_back(rectangle.tl());
        seamViews.push_back(mosaicViews.size() - 1);
    }
    if (mosaicViews.empty())
        throw std::runtime_error("Great has no source-photo coverage in the requested area");
    const auto exposureStarted = std::chrono::steady_clock::now();
    if (config.mosaicExposure && images.size() > 1) {
        cv::detail::GainCompensator exposure;
        std::vector<std::pair<cv::UMat, unsigned char>> tagged;
        for (const auto& mask : masks) tagged.emplace_back(mask, static_cast<unsigned char>(255));
        exposure.feed(corners, images, tagged);
        const auto gains = exposure.gains();
        if (gains.size() != images.size()) throw std::runtime_error("Great exposure solver returned wrong gain count");
        for (std::size_t i = 0; i < gains.size(); ++i) {
            if (!std::isfinite(gains[i]) || gains[i] <= 0)
                throw std::runtime_error("Great exposure solver returned invalid gain");
            mosaicViews[seamViews[i]].gain = std::clamp(gains[i], .8, 1.25);
        }
    }
    const auto seamsStarted = std::chrono::steady_clock::now();
    if (config.mosaicSeamMethod != "voronoi")
        for (std::size_t i = 0; i < images.size(); ++i)
            images[i].convertTo(images[i], CV_32F, mosaicViews[seamViews[i]].gain);
    if (stopped()) throw std::runtime_error("Great seam optimization canceled");
    if (!images.empty()) {
        if (config.mosaicSeamMethod == "dp") {
            cv::detail::DpSeamFinder finder(cv::detail::DpSeamFinder::COLOR_GRAD);
            finder.find(images, corners, masks);
        } else if (config.mosaicSeamMethod == "voronoi") {
            cv::detail::VoronoiSeamFinder finder;
            finder.find(images, corners, masks);
        } else {
            cv::detail::GraphCutSeamFinder finder(cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD);
            finder.find(images, corners, masks);
        }
    }
    for (std::size_t i = 0; i < masks.size(); ++i) {
        cv::dilate(masks[i], mosaicViews[seamViews[i]].seam, cv::Mat(), cv::Point(-1, -1), 1);
    }
    if (config.mosaicFillGaps) prepareMosaicCompletion(images, originalMasks, seamViews, stopped);
    if (stopped()) throw std::runtime_error("Great seam optimization canceled");
    mosaicPrepared = true;
    const auto finished = std::chrono::steady_clock::now();
    const auto elapsed = [](auto first, auto last) { return std::chrono::duration<double>(last - first).count(); };
    std::cout << "TIMING mosaic_prepare photos_seconds=" << elapsed(prepareStarted, exposureStarted)
              << " exposure_seconds=" << elapsed(exposureStarted, seamsStarted)
              << " seams_seconds=" << elapsed(seamsStarted, finished)
              << " total_seconds=" << elapsed(prepareStarted, finished) << '\n';
    std::cout << config.mode << " overview ready: " << mosaicViews.size()
              << " source footprints; global seams reused for every output tile.\n";
}

/** @brief Build a shared fill overview while retaining observed support.
 * @param images Warped colour overviews.
 * @param originalMasks Matching support masks before seam selection.
 * @param seamViews Source indices corresponding to the overview entries.
 * @param stopped Cooperative cancellation predicate.
 * @details Keep the observed mask separate from inferred fill so later
 * renders need not report extrapolated colours as measured coverage. */
void Orthorectifier::Impl::prepareMosaicCompletion(const std::vector<cv::UMat>& images,
    const std::vector<cv::UMat>& originalMasks, const std::vector<std::size_t>& seamViews,
    const std::function<bool()>& stopped) {
    double actualEast = west, actualSouth = north;
    for (const auto latitude : {config.start.lat, config.end.lat})
        for (const auto longitude : {config.start.lon, config.end.lon}) {
            const auto corner = projection.forward(latitude, longitude);
            actualEast = std::max(actualEast, corner.first); actualSouth = std::min(actualSouth, corner.second);
        }
    const int width = std::max(1, static_cast<int>(std::ceil((actualEast - west) / (config.mapPixelSize * seamScale))));
    const int height = std::max(1, static_cast<int>(std::ceil((north - actualSouth) / (config.mapPixelSize * seamScale))));
    if (static_cast<std::size_t>(width) * height > static_cast<std::size_t>(config.lowMemory ? config.mosaicOverviewPixels : 8000000))
        throw std::runtime_error("Global completion overview exceeds 8 million pixels; increase mosaic.seam_scale");
    cv::Mat premultiplied(height, width, CV_32FC4, cv::Scalar(0));
    cv::Mat scores(height, width, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));
    cv::Mat observed(height, width, CV_8U, cv::Scalar(0));
    for (std::size_t index = 0; index < images.size(); ++index) {
        if (stopped()) throw std::runtime_error("Global mosaic source fusion canceled");
        const auto& view = mosaicViews[seamViews[index]];
        const auto rectangle = view.rectangle & cv::Rect(0, 0, width, height);
        if (rectangle.empty()) continue;
        cv::Mat colour;
        images[index].convertTo(colour, CV_32F, config.mosaicSeamMethod == "voronoi" ? view.gain : 1);
        const auto original = originalMasks[index].getMat(cv::ACCESS_READ);
        for (int row = rectangle.y; row < rectangle.y + rectangle.height; ++row) {
            auto* destination = premultiplied.ptr<cv::Vec4f>(row);
            auto* best = scores.ptr<float>(row);
            auto* coverage = observed.ptr<unsigned char>(row);
            const int sourceRow = row - view.rectangle.y;
            const auto* pixels = colour.ptr<cv::Vec3f>(sourceRow);
            const auto* valid = original.ptr<unsigned char>(sourceRow);
            const auto* seam = view.seam.ptr<unsigned char>(sourceRow);
            for (int column = rectangle.x; column < rectangle.x + rectangle.width; ++column) {
                const int sourceColumn = column - view.rectangle.x;
                if (!valid[sourceColumn]) continue;
                const double dx = (sourceColumn + .5) / view.rectangle.width - .5;
                const double dy = (sourceRow + .5) / view.rectangle.height - .5;
                const float score = static_cast<float>(dx * dx + dy * dy + (seam[sourceColumn] ? 0 : 10));
                if (score >= best[column]) continue;
                best[column] = score; coverage[column] = 255;
                const auto& pixel = pixels[sourceColumn];
                destination[column] = cv::Vec4f(pixel[0], pixel[1], pixel[2], 1);
            }
        }
    }
    cv::distanceTransform(observed, mosaicSourceDistance, cv::DIST_L2, 3);
    completedMosaic = completeMosaicPyramid(premultiplied, config.mosaicThreads, stopped);
    std::cout << "Global source-derived completion ready: " << width << " x " << height
              << "; completed pixels are tracked separately from observed photo colour.\n";
}

/** @brief Render a mosaic window using the session-wide seam plan.
 * @param map Projected centre and georeference of the window.
 * @param width Output columns.
 * @param height Output rows.
 * @param stopped Cooperative cancellation predicate.
 * @return Owned pixels, confidence, and rendering provenance.
 * @details Combine planned contributions in the local window while tracking
 * fallback colour and coverage independently of multiband blending. */
OrthoImage Orthorectifier::Impl::renderMosaicRegion(const MapTile& map, int width, int height,
                                                   const std::function<bool()>& stopped) {
    prepareMosaic(stopped);
    OrthoImage result;
    result.map = map; result.width = width; result.height = height; result.mode = config.mode;
    result.referencePlane = config.referencePlane;
    result.seamScale = seamScale; result.blendBands = config.mosaicBlendBands;
    result.seamMethod = config.mosaicSeamMethod;
    result.exposureCompensated = config.mosaicExposure;
    const auto count = static_cast<std::size_t>(width) * height;
    result.bgra.assign(count * 4, 0); result.confidence.assign(count, 0);
    const cv::Rect region(static_cast<int>(std::llround((map.east - west) / config.mapPixelSize - width / 2.0)),
                          static_cast<int>(std::llround((north - map.north) / config.mapPixelSize - height / 2.0)),
                          width, height);
    cv::Mat best(height, width, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));
    cv::Mat fallback(height, width, CV_8UC3, cv::Scalar(0));
    cv::Mat coverage(height, width, CV_8U, cv::Scalar(0));
    cv::detail::MultiBandBlender blender(false, config.mosaicBlendBands);
    blender.prepare(cv::Rect(0, 0, width, height));
    bool fed = false;
    for (const auto& view : mosaicViews) {
        if (stopped()) throw std::runtime_error("Great blending canceled");
        const cv::Rect footprint = view.footprint;
        if ((footprint & region).empty()) continue;
        auto& photo = mosaicPhoto(view.source);
        cv::Mat image, mask;
        const auto transform = mosaicWarp(photo, region, 1, image, mask);
        if (cv::countNonZero(mask) == 0) continue;
        image.convertTo(image, CV_8U, view.gain);
        result.sources.push_back(photo.info);
        cv::Mat selected(height, width, CV_8U, cv::Scalar(0));
        parallelRows(height, [&](int y) {
            const auto* valid = mask.ptr<unsigned char>(y);
            const auto* colours = image.ptr<cv::Vec3b>(y);
            auto* scores = best.ptr<float>(y);
            auto* backup = fallback.ptr<cv::Vec3b>(y);
            auto* covered = coverage.ptr<unsigned char>(y);
            auto* owned = selected.ptr<unsigned char>(y);
            const double gy = (region.y + y + .5) / seamScale - .5 - view.rectangle.y;
            const int sy = view.seam.empty() ? 0 :
                std::clamp(static_cast<int>(std::lround(gy)), 0, view.seam.rows - 1);
            for (int x = 0; x < width; ++x) {
                if (!valid[x]) continue;
                covered[x] = 255;
                const double depth = transform(2, 0) * x + transform(2, 1) * y + transform(2, 2);
                const double u = (transform(0, 0) * x + transform(0, 1) * y + transform(0, 2)) / depth;
                const double v = (transform(1, 0) * x + transform(1, 1) * y + transform(1, 2)) / depth;
                const double score = std::pow((u - photo.info.cx) / photo.info.width, 2) +
                                     std::pow((v - photo.info.cy) / photo.info.height, 2);
                if (score < scores[x]) { scores[x] = static_cast<float>(score); backup[x] = colours[x]; }
                if (view.seam.empty()) continue;
                const double gx = (region.x + x + .5) / seamScale - .5 - view.rectangle.x;
                const int sx = std::clamp(static_cast<int>(std::lround(gx)), 0, view.seam.cols - 1);
                if (view.seam.at<unsigned char>(sy, sx)) owned[x] = 255;
            }
        }, config.mosaicThreads);
        if (cv::countNonZero(selected)) {
            cv::Mat signedImage; image.convertTo(signedImage, CV_16S);
            blender.feed(signedImage, selected, cv::Point(0, 0));
            fed = true;
        }
    }
    cv::Mat blended, blendedMask;
    if (fed) blender.blend(blended, blendedMask);
    std::atomic<std::size_t> reconstructed{0}, reconstructionBlend{0};
    parallelRows(height, [&](int y) {
        if (stopped()) throw std::runtime_error("Great mosaic finalization canceled");
        std::size_t rowReconstructed = 0, rowBlended = 0;
        for (int x = 0; x < width; ++x) {
            const bool observed = coverage.at<unsigned char>(y, x) != 0;
            if (!observed && completedMosaic.empty()) continue;
            const auto index = static_cast<std::size_t>(y) * width + x;
            auto* pixel = result.bgra.data() + index * 4;
            if (observed && fed && blendedMask.at<unsigned char>(y, x)) {
                const auto value = blended.at<cv::Vec3s>(y, x);
                for (int channel = 0; channel < 3; ++channel) pixel[channel] = cv::saturate_cast<unsigned char>(value[channel]);
            } else if (observed) {
                const auto value = fallback.at<cv::Vec3b>(y, x);
                for (int channel = 0; channel < 3; ++channel) pixel[channel] = value[channel];
            }
            pixel[3] = 255;
            result.confidence[index] = observed ? 64 : 16;
            if (!completedMosaic.empty()) {
                const double sourceX = std::clamp((region.x + x + .5) / seamScale - .5,
                                                 0.0, double(completedMosaic.cols - 1));
                const double sourceY = std::clamp((region.y + y + .5) / seamScale - .5,
                                                 0.0, double(completedMosaic.rows - 1));
                const int left = static_cast<int>(sourceX), top = static_cast<int>(sourceY);
                const int right = std::min(left + 1, completedMosaic.cols - 1);
                const int bottom = std::min(top + 1, completedMosaic.rows - 1);
                const double fractionX = sourceX - left, fractionY = sourceY - top;
                const auto sample = [&](const cv::Mat& image, int channel) {
                    const int channels = image.channels();
                    const auto* first = image.ptr<float>(top); const auto* last = image.ptr<float>(bottom);
                    return (1 - fractionY) * ((1 - fractionX) * first[left * channels + channel] +
                        fractionX * first[right * channels + channel]) + fractionY *
                        ((1 - fractionX) * last[left * channels + channel] + fractionX * last[right * channels + channel]);
                };
                double weight = observed ? std::clamp(sample(mosaicSourceDistance, 0) / 2, 0.0, 1.0) : 0;
                weight = weight * weight * (3 - 2 * weight);
                if (weight < 1) {
                    for (int channel = 0; channel < 3; ++channel)
                        pixel[channel] = cv::saturate_cast<unsigned char>(weight * pixel[channel] +
                            (1 - weight) * sample(completedMosaic, channel));
                    if (observed) { result.confidence[index] = 48; ++rowBlended; }
                    else ++rowReconstructed;
                }
            }
        }
        reconstructed += rowReconstructed; reconstructionBlend += rowBlended;
    }, config.mosaicThreads);
    result.validFraction = completedMosaic.empty() ? static_cast<double>(cv::countNonZero(coverage)) / count : 1;
    result.reconstructedFraction = static_cast<double>(reconstructed.load()) / count;
    result.reconstructionBlendFraction = static_cast<double>(reconstructionBlend.load()) / count;
    return result;
}

std::filesystem::path Orthorectifier::normalizeSources(const fsPath& csv, const fsPath& directory,
                                                      const std::function<bool()>& stopped,
                                                      unsigned workers, unsigned memoryMiB) {
    const auto started = std::chrono::steady_clock::now();
    const auto sources = readSources(csv);
    if (std::none_of(sources.begin(), sources.end(), [](const OrthoSource& source) { return source.distorted; }))
        return csv;
    if (std::filesystem::exists(directory)) throw std::runtime_error("Rectified working directory already exists");
    std::filesystem::create_directories(directory);
    std::ifstream input(csv);
    std::string line;
    std::getline(input, line);
    const auto header = csvFields(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) columns.emplace(header[i], i);
    const auto result = directory / "photos.csv";
    std::ofstream output(result, std::ios::binary);
    output << line << '\n';
    std::uint64_t bytesPerWorker = 1;
    for (const auto& source : sources) {
        const auto required = static_cast<std::uint64_t>(source.width) * source.height * 128 +
            std::filesystem::file_size(source.path) + 4 * 1024 * 1024;
        bytesPerWorker = std::max(bytesPerWorker, required);
    }
    const auto memoryWorkers = std::max<std::uint64_t>(1,
        static_cast<std::uint64_t>(memoryMiB) * 1024 * 1024 / bytesPerWorker);
    workers = static_cast<unsigned>(std::min<std::uint64_t>({std::max(1u, workers),
        std::max(1u, std::thread::hardware_concurrency()), sources.size(), memoryWorkers}));
    std::vector<int> insets(sources.size());
    parallelRows(static_cast<int>(workers), [&](int worker) {
        JpegEncoder encoder;
        RectificationCache rectification;
        for (std::size_t index = static_cast<unsigned>(worker); index < sources.size(); index += workers) {
            if (stopped()) throw std::runtime_error("Camera rectification canceled");
            const auto& source = sources[index];
            Photo photo; photo.info = source; photo.load(0, &rectification);
            const int inset = insets[index] = photo.validInset;
            const auto* pixels = photo.bgra.data() + (static_cast<std::size_t>(inset) * source.width + inset) * 4;
            const auto image = encoder.encode(pixels, source.width - 2 * inset, source.height - 2 * inset,
                                              source.width * 4, 100);
            JpegEncoder::save(image, directory / source.path.filename());
        }
    }, workers);
    const int inset = insets.front();
    if (!std::all_of(insets.begin(), insets.end(), [&](int value) { return value == inset; }))
        throw std::runtime_error("Source lens rectification changed valid crop");
    while (std::getline(input, line)) {
        auto fields = csvFields(line);
        if (fields.size() != header.size()) throw std::runtime_error("Source CSV changed during rectification");
        if (fields[columns.at("trigger")] != "box_tile") continue;
        if (inset > 0) {
            const auto value = [&](const char* key) { return number(fields.at(columns.at(key)), key); };
            const double width = value("width"), height = value("height");
            const double cropWidth = value("crop_width"), cropHeight = value("crop_height");
            const auto set = [&](const char* key, double numberValue) {
                std::ostringstream text;
                text.imbue(std::locale::classic()); text << std::setprecision(17) << numberValue;
                fields.at(columns.at(key)) = text.str();
            };
            set("crop_x", value("crop_x") + inset * cropWidth / width);
            set("crop_y", value("crop_y") + inset * cropHeight / height);
            set("crop_width", cropWidth * (width - 2 * inset) / width);
            set("crop_height", cropHeight * (height - 2 * inset) / height);
            set("width", width - 2 * inset); set("height", height - 2 * inset);
            if (columns.count("cx_px")) { set("cx_px", value("cx_px") - inset); set("cy_px", value("cy_px") - inset); }
        }
        fields[columns.at("lens_profile")] = "simconnect";
        for (const auto* key : {"lens_k1", "lens_k2", "lens_blue_scale", "lens_red_scale"}) {
            const auto column = columns.find(key);
            if (column != columns.end()) fields[column->second] = std::string(key).find("scale") != std::string::npos ? "1" : "0";
        }
        for (std::size_t i = 0; i < fields.size(); ++i) {
            if (i) output << ',';
            output << '"';
            for (const auto ch : fields[i]) { if (ch == '"') output << '"'; output << ch; }
            output << '"';
        }
        output << '\n';
    }
    if (input.bad()) throw std::runtime_error("Cannot read source CSV during rectification");
    output.close();
    if (!output) throw std::runtime_error("Cannot finalize rectified source CSV");
#if defined(__GLIBC__)
    if (workers > 1) malloc_trim(0);
#endif
    if (inset > 0)
        std::cout << "Rectified working images exclude " << inset
                  << " unsupported border pixels per edge; cropped intrinsics recorded.\n";
    std::cout << "Lens-aware working images saved locally; originals unchanged: " << directory.u8string() << '\n';
    std::cout << "TIMING source_prepare photos=" << sources.size() << " workers=" << workers << " wall_seconds="
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() << '\n';
    return result;
}

/** @brief Render a window with the selected orthorectification engine.
 * @param map Projected centre and georeference of the window.
 * @param outputWidth Output columns.
 * @param outputHeight Output rows.
 * @param stopped Cooperative cancellation predicate.
 * @return Owned pixels and coverage information.
 * @details The simple engine ranks sources by distance and bounds the
 * candidate set; the mosaic engine uses the shared seam plan instead. */
OrthoImage Orthorectifier::Impl::renderRegion(const MapTile& map, int outputWidth, int outputHeight,
                                             const std::function<bool()>& stopped) {
    if (config.orthoEngine == "mosaic") return renderMosaicRegion(map, outputWidth, outputHeight, stopped);
    const auto& state = *this;
    OrthoImage result;
    result.width = outputWidth; result.height = outputHeight;
    const double tileWidth = result.width * config.mapPixelSize, tileHeight = result.height * config.mapPixelSize;
    result.map = map;
    std::vector<std::pair<double, std::size_t>> nearest;
    for (std::size_t i = 0; i < state.sources.size(); ++i) {
        const auto p = state.projection.forward(state.sources[i].latitude, state.sources[i].longitude);
        nearest.emplace_back(std::hypot(p.first - result.map.east, p.second - result.map.north), i);
    }
    std::sort(nearest.begin(), nearest.end());
    Photos photos;
    for (const auto& location : nearest) {
        if (photos.size() == 12) break;
        const auto& info = state.sources[location.second];
        const double maxFootprint = (info.altitude - config.referencePlane) / info.focal * std::hypot(info.width, info.height);
        if (location.first > maxFootprint / 2 + std::hypot(tileWidth, tileHeight) / 2) break;
        const auto decoded = static_cast<std::size_t>(info.width) * info.height * 4;
        std::size_t loaded = 0;
        for (const auto& photo : photos) loaded += photo->bgra.size();
        if (loaded + decoded > 512 * 1024 * 1024)
            throw std::runtime_error("Source view memory exceeds 512 MB; reduce source viewport dimensions");
        result.sources.push_back(info);
        photos.push_back(sourcePhoto(location.second));
    }
        const auto count = static_cast<std::size_t>(result.width) * result.height;
        result.bgra.assign(count * 4, 0);
        result.confidence.assign(count, 0);
        const double left = map.east - (result.width - 1) * config.mapPixelSize / 2;
        const double top = map.north + (result.height - 1) * config.mapPixelSize / 2;
        parallelRows(result.height, [&](int y) {
            if (stopped()) throw std::runtime_error("Simple mosaic canceled");
            for (int x = 0; x < result.width; ++x) {
                const double e = left + x * config.mapPixelSize, n = top - y * config.mapPixelSize;
                double best = std::numeric_limits<double>::infinity(), weights = 0;
                std::array<double, 3> sum{};
                const auto index = static_cast<std::size_t>(y) * result.width + x;
                auto* pixel = result.bgra.data() + index * 4;
                for (const auto& owner : photos) {
                    const auto& photo = *owner;
                    double u, v;
                    if (!photo.project(e, n, config.referencePlane, u, v, 1)) continue;
                    const double score = std::pow((u - photo.info.cx) / photo.info.width, 2) +
                                         std::pow((v - photo.info.cy) / photo.info.height, 2);
                    unsigned char colour[4]{};
                    photo.color(u, v, colour);
                    if (config.blend == "feather") {
                        const double edge = std::min({u, v, photo.info.width - 1 - u, photo.info.height - 1 - v});
                        const double weight = edge * edge / (1 + score);
                        for (std::size_t channel = 0; channel < 3; ++channel) sum[channel] += colour[channel] * weight;
                        weights += weight;
                    } else if (score < best) {
                        best = score;
                        std::copy(colour, colour + 4, pixel);
                    }
                }
                if (weights > 0) {
                    for (std::size_t channel = 0; channel < 3; ++channel)
                        pixel[channel] = static_cast<unsigned char>(std::clamp(std::lround(sum[channel] / weights), 0L, 255L));
                    pixel[3] = 255;
                }
                if (pixel[3]) result.confidence[index] = 64;
            }
        });
        result.validFraction = static_cast<double>(std::count(result.confidence.begin(), result.confidence.end(), 64)) / count;
        return result;
}

OrthoImage Orthorectifier::render(std::size_t index, const std::function<bool()>& stopped,
                                const std::function<void(double)>& progress) {
    if (index >= size()) throw std::runtime_error("Orthophoto tile index out of range");
    const auto& config = impl_->config;
    return renderWindow(index % columns() * config.jpegWidth, index / columns() * config.jpegHeight,
                        config.jpegWidth, config.jpegHeight, stopped, progress);
}

OrthoImage Orthorectifier::renderWindow(std::size_t originX, std::size_t originY, int width, int height,
                                      const std::function<bool()>& stopped,
                                      const std::function<void(double)>& progress) {
    auto& state = *impl_;
    const auto& config = state.config;
    if (config.lowMemory) state.prepareDiskSources(stopped);
    if (width <= 0 || height <= 0 || width > 4096 || height > 4096 ||
        originX > columns() * config.jpegWidth || originY > rows() * config.jpegHeight ||
        static_cast<std::size_t>(width) > columns() * config.jpegWidth - originX ||
        static_cast<std::size_t>(height) > rows() * config.jpegHeight - originY)
        throw std::runtime_error("Requested render window is outside the working grid");
    OrthoImage result;
    result.width = width; result.height = height;
    result.mode = config.mode;
    result.referencePlane = state.config.referencePlane;
    result.map = {state.west + (originX + result.width / 2.0) * config.mapPixelSize,
                  state.north - (originY + result.height / 2.0) * config.mapPixelSize,
                  config.mapPixelSize, 0, state.projection.epsg()};
    const auto count = static_cast<std::size_t>(result.width) * result.height;
    result.bgra.resize(count * 4); result.confidence.resize(count);
    const std::size_t blockSize = config.orthoEngine == "mosaic" ? 512 : 500;
    constexpr std::size_t halo = 64;
    const std::size_t regionSize = blockSize + 2 * halo;
    std::size_t valid = 0, reconstructed = 0, reconstructionBlend = 0;
    const auto blocks = ((originX + result.width - 1) / blockSize - originX / blockSize + 1) *
                        ((originY + result.height - 1) / blockSize - originY / blockSize + 1);
    std::size_t completedBlocks = 0;
    if (progress) progress(0);
    for (auto by = originY / blockSize; by <= (originY + result.height - 1) / blockSize; ++by)
        for (auto bx = originX / blockSize; bx <= (originX + result.width - 1) / blockSize; ++bx) {
            if (stopped()) throw std::runtime_error("Orthorectification canceled");
            const auto key = std::make_pair(bx, by);
            auto cached = state.blocks.find(key);
            if (cached == state.blocks.end()) {
                const MapTile map{state.west + (bx * blockSize + blockSize / 2.0) * config.mapPixelSize,
                                  state.north - (by * blockSize + blockSize / 2.0) * config.mapPixelSize,
                                  config.mapPixelSize, 0, state.projection.epsg()};
                const bool webTiles = config.outputFormat == "xyz";
                const auto capacity = config.lowMemory ? std::max<std::size_t>(1,
                    static_cast<std::size_t>(config.blockCacheMiB) * 1024 * 1024 / (regionSize * regionSize * 5)) :
                    (webTiles ? 32u : 4u);
                if (state.blocks.size() >= capacity) {
                    auto oldest = state.blocks.begin();
                    if (webTiles) oldest = std::min_element(state.blocks.begin(), state.blocks.end(),
                        [](const auto& a, const auto& b) { return a.second.touched < b.second.touched; });
                    state.blocks.erase(oldest);
                }
                OrthoImage image;
                const auto blockPath = config.workingDirectory / "blocks" /
                    (std::to_string(bx) + "_" + std::to_string(by));
                const auto recordPath = blockPath.string() + ".verified";
                if (config.lowMemory && std::filesystem::exists(recordPath)) {
                    std::ifstream record(recordPath);
                    std::string digest, trailing;
                    if (!(record >> image.seamScale >> digest) || (record >> trailing) ||
                        image.seamScale < 1 || fileDigest(blockPath) != digest ||
                        std::filesystem::file_size(blockPath) != regionSize * regionSize * 5)
                        throw std::runtime_error("Working block checksum or metadata mismatch");
                    image.width = image.height = static_cast<int>(regionSize);
                    image.bgra.resize(regionSize * regionSize * 4); image.confidence.resize(regionSize * regionSize);
                    std::ifstream payload(blockPath, std::ios::binary);
                    payload.read(reinterpret_cast<char*>(image.bgra.data()), static_cast<std::streamsize>(image.bgra.size()));
                    payload.read(reinterpret_cast<char*>(image.confidence.data()), static_cast<std::streamsize>(image.confidence.size()));
                    if (!payload) throw std::runtime_error("Cannot load working block");
                    image.blendBands = config.mosaicBlendBands; image.seamMethod = config.mosaicSeamMethod;
                    image.exposureCompensated = config.mosaicExposure;
                } else {
                    image = state.renderRegion(map, static_cast<int>(regionSize), static_cast<int>(regionSize), stopped);
                    if (config.lowMemory) {
                        std::filesystem::create_directories(blockPath.parent_path());
                        storageHeadroom(blockPath.parent_path(), image.bgra.size() + image.confidence.size());
                        const auto temporary = blockPath.string() + ".partial";
                        std::ofstream payload(temporary, std::ios::binary | std::ios::trunc);
                        payload.write(reinterpret_cast<const char*>(image.bgra.data()), static_cast<std::streamsize>(image.bgra.size()));
                        payload.write(reinterpret_cast<const char*>(image.confidence.data()), static_cast<std::streamsize>(image.confidence.size()));
                        payload.close();
                        if (!payload) throw std::runtime_error("Cannot save working block");
                        publishWorkingFile(temporary, blockPath);
                        saveWorkingText(recordPath, std::to_string(image.seamScale) + " " + fileDigest(blockPath) + "\n");
                    }
                }
                Impl::CachedBlock block{std::move(image), 0};
                cached = state.blocks.emplace(key, std::move(block)).first;
            }
            cached->second.touched = ++state.blockClock;
            const auto& block = cached->second.image;
            result.seamScale = block.seamScale; result.blendBands = block.blendBands;
            result.seamMethod = block.seamMethod;
            result.exposureCompensated = block.exposureCompensated;
            for (const auto& source : block.sources)
                if (std::none_of(result.sources.begin(), result.sources.end(),
                    [&](const OrthoSource& existing) { return existing.path == source.path; }))
                    result.sources.push_back(source);
            const auto x0 = std::max(originX, bx * blockSize), x1 = std::min(originX + result.width, (bx + 1) * blockSize);
            const auto y0 = std::max(originY, by * blockSize), y1 = std::min(originY + result.height, (by + 1) * blockSize);
            for (auto y = y0; y < y1; ++y) for (auto x = x0; x < x1; ++x) {
                const auto input = (y - by * blockSize + halo) * regionSize + x - bx * blockSize + halo;
                const auto output = (y - originY) * result.width + x - originX;
                std::copy_n(block.bgra.data() + input * 4, 4, result.bgra.data() + output * 4);
                result.confidence[output] = block.confidence[input];
                if (block.confidence[input]) ++valid;
                if (config.orthoEngine == "mosaic") {
                    reconstructed += block.confidence[input] == 16;
                    reconstructionBlend += block.confidence[input] == 48;
                }
            }
            if (progress) progress(static_cast<double>(++completedBlocks) / blocks);
        }
    result.validFraction = static_cast<double>(valid) / count;
    result.reconstructedFraction = static_cast<double>(reconstructed) / count;
    result.reconstructionBlendFraction = static_cast<double>(reconstructionBlend) / count;
    result.corroboratedFraction = 0;
    std::cout << "Working region " << originX << ',' << originY << " shared-grid coverage: "
              << std::fixed << std::setprecision(3) << result.validFraction * 100 << "%.\n";
    return result;
}

/** @brief Run synthetic orthorectification regression cases.
 * @details Use controlled source geometry and imagery to check render
 * behavior independently of simulator or camera availability.
 * @throws std::runtime_error If a fixture invariant fails. */
void Orthorectifier::selfTest() {
    const auto require = [](bool value, const char* message) {
        if (!value) throw std::runtime_error(std::string("Orthorectification test: ") + message);
    };
    {
        const auto directory = std::filesystem::temp_directory_path() /
            ("PangeaRectification_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        require(std::filesystem::create_directory(directory), "Create rectification parity fixture");
        OrthoSource source;
        source.path = directory / "source.jpg";
        source.width = 96; source.height = 80; source.focal = 90;
        source.cx = 47.5; source.cy = 39.5; source.distorted = true;
        source.lens.k1 = .03; source.lens.k2 = .005;
        source.lens.blueScale = .9996; source.lens.redScale = 1.0004;
        std::vector<unsigned char> pixels(96 * 80 * 4);
        for (std::size_t offset = 0; offset < pixels.size(); ++offset)
            pixels[offset] = static_cast<unsigned char>((offset * 37 + offset / 96) % 251);
        JpegEncoder encoder;
        JpegEncoder::save(encoder.encode(pixels.data(), 96, 80, 96 * 4, 100), source.path);
        RectificationCache cache;
        for (int scenario = 0; scenario < 10; ++scenario) {
            auto camera = source;
            if (scenario == 2) camera.focal += 5;
            if (scenario == 3) camera.cx += 1;
            if (scenario == 4) camera.cy -= 1;
            if (scenario == 5) camera.lens.k1 += .01;
            if (scenario == 6) camera.lens.k2 += .001;
            if (scenario == 7) camera.lens.blueScale -= .001;
            if (scenario == 8) camera.lens.redScale += .001;
            for (int repeated = 0; repeated < 2; ++repeated) {
                Photo original, cached;
                original.info = cached.info = camera;
                original.load(); cached.load(0, &cache);
                require(original.bgra == cached.bgra && original.validInset == cached.validInset,
                        "Cached rectification preserves every BGRA byte and crop across calibration changes");
            }
        }
        require(cache.mapX[1].data == cache.mapX[3].data && cache.mapY[1].data == cache.mapY[3].data,
                "Identical green and alpha calibration shares immutable maps");
        for (const bool changeWidth : {false, true}) {
            auto camera = source;
            if (changeWidth) ++camera.width; else ++camera.height;
            require(cache.prepare(camera, 96 * 80) && cache.mapX[0].empty(),
                    "Changed dimensions invalidate cached maps");
        }
        require(!cache.prepare(source, 96 * 1024 * 1024) && cache.mapX[0].empty(),
                "Oversized calibration releases the cache and uses the uncached path");
        std::filesystem::remove(source.path);
        std::filesystem::remove(directory);
    }
    {
        cv::Mat sparse(13, 9, CV_32FC4, cv::Scalar(0));
        sparse.at<cv::Vec4f>(3, 3) = cv::Vec4f(32, 64, 128, 1);
        sparse.at<cv::Vec4f>(7, 5) = cv::Vec4f(0, 0, 0, 1);
        const auto completed = completeMosaicPyramid(sparse, 32, [] { return false; });
        for (int row = 0; row < completed.rows; ++row) for (int column = 0; column < completed.cols; ++column) {
            const auto value = completed.at<cv::Vec4f>(row, column);
            require(value[3] == 1 && std::isfinite(value[0]) && value[0] >= 0 && value[0] <= 32.001,
                    "Odd-size push-pull completion has finite opaque colour everywhere");
            if (sparse.at<cv::Vec4f>(row, column)[3])
                require(value == sparse.at<cv::Vec4f>(row, column), "Completion preserves observed colour, including genuine black");
        }
        bool emptyRejected = false, canceled = false;
        try { completeMosaicPyramid(cv::Mat::zeros(9, 13, CV_32FC4), 1, [] { return false; }); }
        catch (const std::runtime_error&) { emptyRejected = true; }
        try { completeMosaicPyramid(sparse, 1, [] { return true; }); }
        catch (const std::runtime_error&) { canceled = true; }
        require(emptyRejected && canceled, "Global completion rejects no-source images and honors cancellation");
    }
    MapConfig plan;
    plan.start = {52.00455, 4.35870, 120}; plan.end = {52.00365, 4.36065, 120};
    plan.jpegWidth = 1400; plan.jpegHeight = 1100; plan.mapPixelSize = .1;
    const auto whole = orthoLayout(plan);
    require(whole.columns == 1 && whole.rows == 1 && whole.epsg == 32631 &&
            std::abs(whole.east - whole.west - 140) < 1e-8 &&
            std::abs(whole.north - whole.south - 110) < 1e-8, "Exact final map extent");
    plan.jpegWidth = 700; plan.jpegHeight = 550;
    const auto cuts = orthoLayout(plan);
    require(cuts.columns == 2 && cuts.rows == 2 && cuts.west == whole.west &&
            cuts.north == whole.north && cuts.east == whole.east && cuts.south == whole.south,
            "Tile cuts preserve exact map origin and extent");
    plan.mapPixelSize = 2;
    require(orthoLayout(plan).columns == 1 && orthoLayout(plan).rows == 1,
            "Existing two-metre pixel-size setting remains supported");
    require(csvFields("a,\"b,c\",\"d\"\"e\"") == std::vector<std::string>{"a", "b,c", "d\"e"}, "CSV quoting");
    {
        const auto directory = std::filesystem::temp_directory_path() /
            ("PangeaUtcSourceTest_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        require(std::filesystem::create_directory(directory), "Create isolated UTC source CSV fixture");
        for (const auto* name : {"one.jpg", "two.jpg", "three.jpg"}) {
            std::ofstream file(directory / name);
            file << 'x';
            file.close();
            require(static_cast<bool>(file), "Create local source existence fixture");
        }
        const auto target = parseUtcDateTime("2026-11-01T14:56Z");
        const auto csv = directory / "capture.csv";
        for (int mode = 0; mode <= 7; ++mode) {
            std::ofstream fixture(csv);
            fixture << "filename,trigger,observed_latitude,observed_longitude,observed_altitude_m,"
                "observed_pitch_deg,observed_bank_deg,observed_heading_deg,sdk_fov_deg,projection,map_epsg,"
                "commanded_pitch_deg,commanded_bank_deg,commanded_heading_deg,width,height,source_width,"
                "source_height,crop_width,crop_height,crop_x,crop_y,pose_received_qpc_100ns,"
                "frame_qpc_100ns,request_qpc_100ns";
            if (mode) {
                fixture << ",capture_utc,utc_absolute_seconds";
                if (mode != 7) fixture << ",utc_read_qpc_100ns";
            }
            fixture << '\n';
            int index = 0;
            for (const auto* name : {"one.jpg", "two.jpg", "three.jpg"}) {
                fixture << name << ",box_tile,52,4,120,90,-90,-90,60,perspective,,90,0,0,"
                    "64,64,64,64,64,64,0,0,11000000,13000000,12000000";
                if (mode == 6) fixture << ",,,";
                else if (mode) {
                    auto utc = target;
                    if (mode == 2 && index == 2) utc = parseUtcDateTime("2026-11-01T14:57Z");
                    fixture << ',' << (mode == 5 && index == 2 ? "" : utc.text) << ','
                        << std::setprecision(17) << utc.absoluteSeconds() + (mode == 3 ? 1 : 0.125);
                    if (mode != 7) fixture << ',' << (mode == 4 ? 3000000 : 12000000);
                }
                fixture << '\n';
                ++index;
            }
            fixture.close();
            require(static_cast<bool>(fixture), "Write UTC source CSV fixture");
            bool rejected = false;
            try {
                const auto sources = readSources(csv);
                require(sources.size() == 3 &&
                        sources.front().captureUtc.has_value() == (mode != 0 && mode != 6),
                        "Legacy and fixed-time CSVs retain correct UTC verification status");
                if (mode == 1) require(sources.back().captureUtc->text == target.text,
                                      "All source photos retain the same locked UTC");
            } catch (const std::runtime_error& error) {
                rejected = std::string(error.what()).find("Source") == 0;
                if (!rejected) throw;
            }
            require(rejected == (mode >= 2 && mode != 6),
                    "Reject changed, drifting, stale, mixed or incomplete source UTC evidence");
        }
        for (const auto* name : {"capture.csv", "one.jpg", "two.jpg", "three.jpg"})
            std::filesystem::remove(directory / name);
        std::filesystem::remove(directory);
    }
    Photo p;
    p.info = {};
    p.info.latitude = 52; p.info.longitude = 4; p.info.altitude = 120;
    p.info.focal = 1000; p.info.cx = 500; p.info.cy = 500; p.info.pitch = 90;
    p.info.fov = 60; p.info.poseTime = 1; p.info.frameTime = 2;
    p.info.width = 1001; p.info.height = 1001;
    p.east = 10; p.north = 20;
    double x, y;
    require(p.project(22, 26, 0, x, y) && std::abs(x - 600) < 1e-9 && std::abs(y - 450) < 1e-9,
            "Ground pinhole projection");
    require(p.project(22, 26, 60, x, y) && std::abs(x - 700) < 1e-9 && std::abs(y - 400) < 1e-9,
            "Roof height projection");
    std::vector<int> visited(11);
    parallelRows(11, [&](int row) { visited[static_cast<std::size_t>(row)] = row + 1; });
    require(std::accumulate(visited.begin(), visited.end(), 0) == 66, "Every parallel row runs exactly once");
    std::vector<int> nestedRows(40, 0);
    parallelRows(8, [&](int outer) {
        parallelRows(5, [&](int inner) { nestedRows[static_cast<std::size_t>(outer) * 5 + inner] = 1; }, 32);
    }, 32);
    require(std::accumulate(nestedRows.begin(), nestedRows.end(), 0) == 40,
            "Nested row work completes without nested worker teams or lost rows");
    bool workerFailed = false;
    try { parallelRows(11, [](int row) { if (row == 3) throw std::runtime_error("test worker failure"); }); }
    catch (const std::runtime_error& error) { workerFailed = std::string(error.what()) == "test worker failure"; }
    require(workerFailed, "Parallel worker errors propagate after joining");
    std::cout << "PASS: orthorectification CSV, plane projection, source planning, "
                 "parallel errors, discontinuities and cancellation.\n";
}
}
