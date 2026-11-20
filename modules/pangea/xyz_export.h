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
 * @file xyz_export.h
 * @brief Resample the UTM source mosaic into complete 256-pixel XYZ tiles.
 * @details Tile pixels are transformed through geographic coordinates instead
 * of assuming UTM and Web Mercator share a grid. Every interpolation tap with a
 * nonzero weight must have source support; publishing partial or fabricated
 * pixels would break opaque-tile compatibility and misstate coverage.
 */
#ifndef PANGEA_XYZ_EXPORT_H
#define PANGEA_XYZ_EXPORT_H

#include "xyz_tiles.h"
#include "parallel_rows.h"

namespace pangea_survey {
/** @brief Colour and confidence planes for one aligned XYZ working batch. */
struct XyzPixels {
    int width = 0, height = 0;
    std::vector<unsigned char> bgra, confidence;
};

/** @brief Require every one of the tile's 256 x 256 output pixels to be supported. */
inline bool xyzCompleteCoverage(std::size_t valid) {
    return valid == 65536;
}

/**
 * @brief Resample an orthomosaic window onto one Web-Mercator pixel grid.
 * @param[in] processor Prepared source-photo mapper.
 * @param[in] config Validated survey settings and worker budget.
 * @param[in] layout UTM layout used by the mapper.
 * @param[in] zoom XYZ zoom level.
 * @param[in] originX Global XYZ pixel x coordinate of the output window's first pixel.
 * @param[in] originY Global XYZ pixel y coordinate of the output window's first pixel.
 * @param[in] width Output window width, at most 1024 pixels.
 * @param[in] height Output window height, at most 1024 pixels.
 * @param[in] stopped Cooperative cancellation predicate.
 * @return BGRA output and confidence for every resampled pixel.
 * @throws std::runtime_error If the footprint exceeds its bounded UTM window,
 * projection fails, or cancellation is requested.
 */
inline XyzPixels renderXyzPixels(Orthorectifier& processor, const MapConfig& config,
                                const OrthoLayout& layout, int zoom, std::uint64_t originX,
                                std::uint64_t originY, int width, int height,
                                const std::function<bool()>& stopped) {
    if (width <= 0 || height <= 0 || width > 1024 || height > 1024)
        throw std::runtime_error("XYZ working batch must fit 1024 x 1024 pixels");
    const MapProjection projection((config.start.lat + config.end.lat) / 2,
                                   (config.start.lon + config.end.lon) / 2);
    struct Pixel { double x, y; };
    const auto count = static_cast<std::size_t>(width) * height;
    std::vector<Pixel> positions(count);
    std::vector<double> longitude(width), latitude(height);
    for (int x = 0; x < width; ++x)
        longitude[x] = XyzTiles::pixel(originX + x, originY, zoom).second;
    for (int y = 0; y < height; ++y)
        latitude[y] = XyzTiles::pixel(originX, originY + y, zoom).first;
    parallelRows(height, [&](int y) {
        if (stopped()) throw std::runtime_error("XYZ coordinate transformation canceled");
        for (int x = 0; x < width; ++x) {
            const auto p = projection.forward(latitude[y], longitude[x]);
            positions[static_cast<std::size_t>(y) * width + x] =
                {(p.first - layout.west) / config.mapPixelSize - .5,
                 (layout.north - p.second) / config.mapPixelSize - .5};
        }
    }, config.mosaicThreads);
    double minX = positions[0].x, maxX = minX, minY = positions[0].y, maxY = minY;
    for (const auto& p : positions) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    const double left = std::floor(minX), top = std::floor(minY);
    const int windowWidth = static_cast<int>(std::floor(maxX) - left + 2);
    const int windowHeight = static_cast<int>(std::floor(maxY) - top + 2);
    if (left < 0 || top < 0 || windowWidth > 4096 || windowHeight > 4096)
        throw std::runtime_error("XYZ resampling footprint exceeds its padded working grid");
    const auto source = processor.renderWindow(static_cast<std::size_t>(left), static_cast<std::size_t>(top),
                                               windowWidth, windowHeight, stopped);
    XyzPixels result;
    result.width = width; result.height = height;
    result.bgra.assign(count * 4, 0); result.confidence.assign(count, 0);
    parallelRows(height, [&](int y) {
        if (stopped()) throw std::runtime_error("XYZ resampling canceled");
        for (int x = 0; x < width; ++x) {
            const auto i = static_cast<std::size_t>(y) * width + x;
            const auto& p = positions[i];
            const double bx = std::floor(p.x), by = std::floor(p.y);
            const double fx = p.x - bx, fy = p.y - by;
            const auto offset = static_cast<std::size_t>(by - top) * windowWidth +
                                static_cast<std::size_t>(bx - left);
            const std::size_t taps[] = {offset, offset + 1, offset + windowWidth, offset + windowWidth + 1};
            const double weights[] = {(1 - fx) * (1 - fy), fx * (1 - fy), (1 - fx) * fy, fx * fy};
            bool valid = true;
            unsigned char confidence = 255;
            double colour[3]{};
            // A bilinear result is supported only when every contributing tap is supported.
            for (int t = 0; t < 4; ++t) {
                if (weights[t] == 0) continue;
                valid = valid && source.confidence[taps[t]] != 0;
                confidence = std::min(confidence, source.confidence[taps[t]]);
                for (int c = 0; c < 3; ++c) colour[c] += source.bgra[taps[t] * 4 + c] * weights[t];
            }
            if (!valid) continue;
            result.confidence[i] = confidence;
            for (int c = 0; c < 3; ++c)
                result.bgra[i * 4 + c] = static_cast<unsigned char>(std::clamp(std::lround(colour[c]), 0L, 255L));
            result.bgra[i * 4 + 3] = 255;
        }
    }, config.mosaicThreads);
    return result;
}

/**
 * @brief Build one lower-zoom level from aligned 2 x 2 child pixels.
 * @param[in] input Complete, even-sized child pixel batch.
 * @param[in] workers Upper bound on row workers.
 * @param[in] stopped Cooperative cancellation predicate.
 * @return Half-size overview; minimum child confidence propagates missingness.
 * @throws std::runtime_error If dimensions are not aligned or cancellation is requested.
 */
inline XyzPixels xyzOverview(const XyzPixels& input, unsigned workers,
                             const std::function<bool()>& stopped) {
    if (input.width <= 0 || input.height <= 0 || input.width % 2 || input.height % 2)
        throw std::runtime_error("XYZ overview requires complete aligned child pixels");
    XyzPixels output;
    output.width = input.width / 2; output.height = input.height / 2;
    const auto count = static_cast<std::size_t>(output.width) * output.height;
    output.bgra.resize(count * 4); output.confidence.resize(count);
    parallelRows(output.height, [&](int y) {
        if (stopped()) throw std::runtime_error("XYZ overview generation canceled");
        for (int x = 0; x < output.width; ++x) {
            const auto at = static_cast<std::size_t>(2 * y) * input.width + 2 * x;
            const std::size_t taps[] = {at, at + 1, at + input.width, at + input.width + 1};
            const auto i = static_cast<std::size_t>(y) * output.width + x;
            unsigned char confidence = 255;
            for (const auto t : taps) {
                confidence = std::min(confidence, input.confidence[t]);
            }
            // A parent cannot claim stronger support than its weakest child pixel.
            output.confidence[i] = confidence;
            for (int c = 0; c < 4; ++c) {
                unsigned sum = 0;
                for (const auto t : taps) sum += input.bgra[t * 4 + c];
                output.bgra[i * 4 + c] = static_cast<unsigned char>((sum + 2) / 4);
            }
        }
    }, workers);
    return output;
}
}

/**
 * @brief Render or reuse one identity-checked XYZ batch for low-memory resume.
 * @param[in] processor Prepared orthorectifier.
 * @param[in] working Working-grid configuration and checkpoint directory.
 * @param[in] layout Projected working-grid bounds.
 * @param[in] zoom Base XYZ zoom.
 * @param[in] x Global XYZ pixel origin.
 * @param[in] y Global XYZ pixel origin.
 * @param[in] width Batch width in pixels.
 * @param[in] height Batch height in pixels.
 * @return Owned BGRA and confidence planes.
 * @throws std::runtime_error If a saved batch has mismatched dimensions/hash or rendering fails.
 */
static pangea_survey::XyzPixels xyzBatch(pangea_survey::Orthorectifier& processor, const pangea_survey::MapConfig& working,
    const pangea_survey::OrthoLayout& layout, int zoom, std::uint64_t x, std::uint64_t y, int width, int height) {
    const auto path = working.workingDirectory / "batches" / (std::to_string(x) + "_" + std::to_string(y));
    const auto recordPath = path.string() + ".verified";
    if (working.lowMemory && fs::exists(recordPath)) {
        std::ifstream record(recordPath);
        int savedWidth = 0, savedHeight = 0;
        std::string digest, trailing;
        if (!(record >> savedWidth >> savedHeight >> digest) || (record >> trailing) ||
            savedWidth != width || savedHeight != height || pangea_survey::fileDigest(path, isStopping) != digest ||
            fs::file_size(path) != static_cast<std::uint64_t>(width) * height * 5)
            throw std::runtime_error("XYZ batch checksum or dimensions mismatch");
        pangea_survey::XyzPixels pixels; pixels.width = width; pixels.height = height;
        const auto count = static_cast<std::size_t>(width) * height;
        pixels.bgra.resize(count * 4); pixels.confidence.resize(count);
        std::ifstream input(path, std::ios::binary);
        for (auto* plane : {&pixels.bgra, &pixels.confidence})
            input.read(reinterpret_cast<char*>(plane->data()), static_cast<std::streamsize>(plane->size()));
        if (!input) throw std::runtime_error("Cannot load XYZ batch");
        std::cout << "Reusing verified XYZ batch " << x << ',' << y << '\n';
        return pixels;
    }
    auto pixels = pangea_survey::renderXyzPixels(processor, working, layout, zoom, x, y, width, height, isStopping);
    if (working.lowMemory) {
        fs::create_directories(path.parent_path());
        pangea_survey::storageHeadroom(path.parent_path(), static_cast<std::uint64_t>(width) * height * 5);
        const auto temporary = path.string() + ".partial";
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        for (const auto* plane : {&pixels.bgra, &pixels.confidence})
            output.write(reinterpret_cast<const char*>(plane->data()), static_cast<std::streamsize>(plane->size()));
        output.close();
        if (!output) throw std::runtime_error("Cannot checkpoint XYZ batch");
        pangea_survey::publishWorkingFile(temporary, path);
        pangea_survey::saveWorkingText(recordPath, std::to_string(width) + " " + std::to_string(height) + " " + pangea_survey::fileDigest(path, isStopping) + "\n");
    }
    return pixels;
}

/**
 * @brief Render and publish the planned XYZ pyramid, optionally as diagnostics.
 * @param[in] config Validated XYZ settings, coverage policy, and output root.
 * @param[in] csv Fixed-session source manifest.
 * @param[in] workDirectory Private report/checkpoint directory for this run.
 * @param[in] diagnostic If true, save inspection images but leave completion false.
 * @return Zero only after the report and all required tiles are finalized.
 * @throws std::runtime_error On cancellation, incomplete coverage, write failure,
 * or mismatched checkpoint content.
 */
static int exportXyzTiles(const pangea_survey::MapConfig& config, const fs::path& csv,
                          const fs::path& workDirectory, bool diagnostic = false) {
    const pangea_survey::XyzPlan plan(config);
    if (!diagnostic && !config.lowMemory) plan.checkOutput(config.output);
    auto working = plan.workingConfig(config);
    working.output = workDirectory;
    pangea_survey::MapProgress progress(diagnostic ? "XYZ diagnostic progress" : "XYZ progress");
    const auto incomplete = "{\"complete\":false,\"format\":\"xyz\",\"mode\":" +
                            pangea_survey::PhotoExif::jsonString(config.mode) + "}";
    if (config.lowMemory && fs::exists(workDirectory / "dataset.json"))
        pangea_survey::publishWorkingFile(workDirectory / "dataset.json", workDirectory / "dataset.previous.json");
    saveBinary(workDirectory / "dataset.incomplete.json", incomplete.data(), incomplete.size());
    constexpr double first = 0;
    progress.stage("Loading XYZ sources and grid", first, first + (99 - first) * .02);
    pangea_survey::Orthorectifier processor(working, csv);
    processor.prepareSources(isStopping);
    const auto layout = pangea_survey::orthoLayout(working);
    const auto latitude = (plan.north + plan.south) / 2;
    std::cout << "Pangea " << config.mode << " XYZ: " << plan.size() << " tiles, zoom "
              << plan.minZoom << ".." << plan.base.zoom << ", 256 x 256, EPSG:3857, no EXIF/XMP.\n"
              << "Ground pixel at area centre: " << pangea_survey::XyzTiles::groundPixel(plan.base.zoom, latitude)
              << " m; working UTM pixel: " << working.mapPixelSize << " m. CPU workers: "
              << config.mosaicThreads << (config.lowMemory ? "; byte-bounded disk cache; " : "; 32-block LRU; ")
              << "no full-map intermediate JPEG.\n"
              << "Output: " << config.output.u8string() << "\nExport report: " << workDirectory.u8string() << '\n';
    if (working.mapPixelSize > pangea_survey::XyzTiles::groundPixel(plan.base.zoom, latitude) * 1.05)
        std::cout << "WARNING: Requested XYZ pixels are finer than the minimum working grid; "
                     "this resampling cannot add source detail.\n";
    const auto manifestPath = workDirectory / (config.lowMemory ? "tiles.csv.partial" : "tiles.csv");
    std::ofstream manifest(manifestPath);
    manifest.imbue(std::locale::classic()); manifest << std::setprecision(17);
    manifest << "zoom,x,y,width,height,valid_fraction,"
                "reconstructed_fraction,reconstruction_blend_fraction,published\n";
    if (!manifest) throw std::runtime_error("Cannot create XYZ tile manifest");
    progress.stage(diagnostic ? "Rendering XYZ diagnostic tiles" : "Rendering and publishing XYZ tiles",
                   first + (99 - first) * .02, 99);
    const auto started = Clock::now();
    std::size_t published = 0, omitted = 0, reconstructed = 0, reconstructionBlend = 0;
    for (auto y = plan.base.firstY; y <= plan.base.lastY;) {
        const auto endY = std::min(plan.base.lastY + 1, (y / 4 + 1) * 4);
        for (auto x = plan.base.firstX; x <= plan.base.lastX;) {
            const auto endX = std::min(plan.base.lastX + 1, (x / 4 + 1) * 4);
            if (isStopping()) throw std::runtime_error("XYZ export canceled; report remains incomplete");
            auto pixels = xyzBatch(processor, working, layout, plan.base.zoom,
                std::uint64_t{x} * 256, std::uint64_t{y} * 256,
                static_cast<int>(endX - x) * 256, static_cast<int>(endY - y) * 256);
            auto tileX = x, tileY = y;
            for (int zoom = plan.base.zoom; zoom >= plan.minZoom; --zoom) {
                const int columns = pixels.width / 256, rows = pixels.height / 256;
                struct Counts {
                    std::size_t valid = 0;
                    std::size_t reconstructed = 0, reconstructionBlend = 0;
                    bool publish = true;
                };
                std::vector<Counts> counts(static_cast<std::size_t>(columns) * rows);
                for (int ty = 0; ty < rows; ++ty) for (int tx = 0; tx < columns; ++tx) {
                    auto& c = counts[static_cast<std::size_t>(ty) * columns + tx];
                    for (int py = 0; py < 256; ++py) for (int px = 0; px < 256; ++px) {
                        const auto i = static_cast<std::size_t>(ty * 256 + py) * pixels.width + tx * 256 + px;
                        c.valid += pixels.confidence[i] != 0;
                        if (pangea_survey::mosaicMode(config.mode)) {
                            c.reconstructed += pixels.confidence[i] == 16;
                            c.reconstructionBlend += pixels.confidence[i] == 48;
                        }
                    }
                    // Missing colour blocks strict export; opt-in omissions remain explicit in the report.
                    if (!diagnostic && !pangea_survey::xyzCompleteCoverage(c.valid)) {
                        if (!config.xyzCompleteOnly)
                            throw std::runtime_error("XYZ tile " + std::to_string(zoom) + "/" +
                                std::to_string(tileX + tx) + "/" + std::to_string(tileY + ty) +
                                " lacks complete colour coverage; not published. "
                                "Acquire more source margin or reduce bounds; export report remains incomplete.");
                        c.publish = false;
                    }
                }
                pangea_survey::parallelRows(columns * rows, [&](int tile) {
                    if (isStopping()) throw std::runtime_error("XYZ JPEG publication canceled");
                    if (!counts[tile].publish) return;
                    const int tx = tile % columns, ty = tile / columns;
                    auto path = pangea_survey::XyzTiles::path(config.output, zoom, tileX + tx, tileY + ty);
                    if (diagnostic) path = workDirectory / (std::to_string(zoom) + "_" +
                        std::to_string(tileX + tx) + "_" + std::to_string(tileY + ty) + ".diagnostic.jpeg");
                    fs::create_directories(path.parent_path());
                    if (!config.lowMemory && (fs::exists(path) || fs::exists(path.string() + ".partial")))
                        throw std::runtime_error("Refusing to overwrite XYZ output: " + path.u8string());
                    JpegEncoder encoder;
                    const auto* data = pixels.bgra.data() +
                        (static_cast<std::size_t>(ty * 256) * pixels.width + tx * 256) * 4;
                    const auto jpeg = encoder.encode(data, 256, 256, pixels.width * 4, static_cast<int>(config.quality));
                    if (config.lowMemory && fs::exists(path)) {
                        if (pangea_survey::fileDigest(path, isStopping) != pangea_survey::dataDigest(jpeg.data, jpeg.size))
                            throw std::runtime_error("Existing XYZ tile differs from its working job; refusing overwrite");
                    } else JpegEncoder::save(jpeg, path, nullptr, 0, config.lowMemory);
                }, config.mosaicThreads);
                for (int tile = 0; tile < columns * rows; ++tile) {
                    const auto& c = counts[tile];
                    manifest << zoom << ',' << tileX + tile % columns << ',' << tileY + tile / columns
                             << ",256,256," << c.valid / 65536.0 << ','
                             << c.reconstructed / 65536.0 << ',' << c.reconstructionBlend / 65536.0 << ','
                             << (c.publish ? 1 : 0) << '\n';
                    if (c.publish) {
                        reconstructed += c.reconstructed;
                        reconstructionBlend += c.reconstructionBlend; ++published;
                    }
                    else ++omitted;
                }
                manifest.flush();
                if (!manifest) throw std::runtime_error("Cannot write XYZ tile manifest");
                progress.update(static_cast<double>(published + omitted) / plan.size(),
                                std::to_string(published) + " published, " + std::to_string(omitted) +
                                " omitted / " + std::to_string(plan.size()) + " tiles");
                if (zoom > plan.minZoom) {
                    // Derive each parent level once from aligned child pixels to keep cuts consistent.
                    pixels = pangea_survey::xyzOverview(pixels, config.mosaicThreads, isStopping);
                    tileX /= 2; tileY /= 2;
                }
            }
            x = endX;
        }
        y = endY;
    }
    manifest.close();
    if (!manifest || published + omitted != plan.size()) throw std::runtime_error("XYZ tile manifest is incomplete");
    if (!published && !config.incrementalPreview)
        throw std::runtime_error("No XYZ tile has complete colour coverage; report remains incomplete");
    if (config.lowMemory) pangea_survey::publishWorkingFile(manifestPath, workDirectory / "tiles.csv");
    progress.stage("Finalizing XYZ export report", 99, 99.9);
    std::ostringstream metadata;
    metadata.imbue(std::locale::classic()); metadata << std::setprecision(17);
    metadata << "{\"complete\":" << (diagnostic || omitted || config.incrementalPreview ? "false" : "true")
             << ",\"provisional\":" << (config.incrementalPreview ? "true" : "false")
             << ",\"processing_complete\":true,\"coverage_policy\":"
             << pangea_survey::PhotoExif::jsonString(config.xyzCompleteOnly ? "complete_tiles_only" : "require_all")
             << ",\"diagnostic\":" << (diagnostic ? "true" : "false")
             << ",\"application\":\"pangea\",\"format\":\"xyz\",\"epsg\":3857,\"scheme\":\"xyz\","
                "\"tile_size\":256,\"extension\":\"jpeg\",\"embedded_metadata\":false,"
                "\"cuda\":false,\"accuracy_certified\":false,\"mode\":"
             << pangea_survey::PhotoExif::jsonString(config.mode)
             << ",\"relief_corrected\":false"
             << ",\"zoom\":" << plan.base.zoom << ",\"min_zoom\":" << plan.minZoom
             << ",\"tile_count\":" << published << ",\"requested_tile_count\":" << plan.size()
             << ",\"omitted_tile_count\":" << omitted << ",\"first_x\":" << plan.base.firstX
             << ",\"last_x\":" << plan.base.lastX << ",\"first_y\":" << plan.base.firstY
             << ",\"last_y\":" << plan.base.lastY << ",\"north\":" << plan.north
             << ",\"south\":" << plan.south << ",\"west\":" << plan.west << ",\"east\":" << plan.east
             << ",\"ground_pixel_size_at_centre_m\":" << pangea_survey::XyzTiles::groundPixel(plan.base.zoom, latitude)
             << ",\"working_pixel_size_m\":" << working.mapPixelSize
             << ",\"threads\":" << config.mosaicThreads
             << ",\"runtime_profile\":" << pangea_survey::PhotoExif::jsonString(config.lowMemory ? "low_memory" : "desktop")
             << ",\"address_space_limit_mib\":" << (config.lowMemory ? config.memoryMiB : 0)
             << ",\"mosaic_seam_method\":" << pangea_survey::PhotoExif::jsonString(config.mosaicSeamMethod)
             << ",\"mosaic_blend_bands\":" << config.mosaicBlendBands
             << ",\"mosaic_photo_cache_mib\":" << config.mosaicPhotoCacheMiB
             << ",\"mosaic_overview_source_max_side\":" << config.mosaicOverviewSourceMaxSide
             << ",\"mosaic_fill_gaps\":" << (config.mosaicFillGaps ? "true" : "false")
             << ",\"gap_fill_method\":" << pangea_survey::PhotoExif::jsonString(config.mosaicFillGaps ? "global_source_push_pull" : "none")
             << ",\"reconstructed_colour_pixel_count\":" << reconstructed
             << ",\"reconstruction_blend_pixel_count\":" << reconstructionBlend
             << ",\"observed_coverage_complete\":" << (omitted || reconstructed || reconstructionBlend ? "false" : "true")
             << ",\"render_export_seconds\":" << seconds(Clock::now() - started)
             << ",\"output\":" << pangea_survey::PhotoExif::jsonString(config.output.u8string())
             << ",\"original_source_csv\":" << pangea_survey::PhotoExif::jsonString(config.originalSourceCsv.u8string())
                  << ",\"photo_directory\":" << pangea_survey::PhotoExif::jsonString(config.photoDirectory.u8string())
                  << ",\"camera_geometry\":" << pangea_survey::PhotoExif::jsonString(config.photoDirectory.empty() ?
                      "confirmed_survey_nadir" : config.allowRecordedPose ? "usher_recorded_exif_pose_perspective_unverified_timing" :
                      "usher_sensor_exposure_pose_perspective")
                  << ",\"optics_calibration\":" << pangea_survey::PhotoExif::jsonString(config.photoDirectory.empty() ?
                      "source_csv" : "raspberry_pi_ai_illustrative_radial_colour_model_not_measured")
                  << ",\"camera_profile\":" << pangea_survey::PhotoExif::jsonString(config.cameraProfile.u8string())
                  << ",\"admission_max_pose_uncertainty_m\":" << (config.allowRecordedPose ? "null" : std::to_string(config.maxPoseUncertainty))
                  << ",\"surveyed_accuracy_validated\":false"
             << ",\"source_csv\":" << pangea_survey::PhotoExif::jsonString(csv.u8string()) << '}';
    const auto text = metadata.str();
    if (config.lowMemory) pangea_survey::saveWorkingText(workDirectory / "dataset.json", text);
    else saveBinary(workDirectory / "dataset.json", text.data(), text.size());
    if (!fs::remove(workDirectory / "dataset.incomplete.json"))
        throw std::runtime_error("Cannot remove incomplete XYZ export marker");
    if (diagnostic) std::cout << "XYZ diagnostic evidence completed; no cache tiles published.\n";
    else std::cout << "XYZ export processing completed: " << published << " complete tiles published, "
                   << omitted << " unsupported tiles omitted; "
                   << (omitted ? "requested area coverage is incomplete.\n" : "full requested area covered.\n");
    progress.finish();
    return 0;
}
#endif
