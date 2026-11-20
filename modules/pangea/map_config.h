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
 * @file map_config.h
 * @brief Own the survey mapper's validated configuration and mode defaults.
 * @details CLI overrides are applied before mode-specific defaults so the
 * resulting configuration has one explicit owner and one validation path.
 * The low-memory profile further narrows that configuration to a supported
 * bounded workflow instead of pretending every option fits on a small board.
 */
#ifndef PANGEA_MAP_CONFIG_H
#define PANGEA_MAP_CONFIG_H

#include "ini_file.h"
#include <limits>
#include <thread>
#include <unistd.h>

namespace pangea_survey {
/**
 * @brief Fully parsed settings shared by orthomap and XYZ export.
 * @details Distances are metres, coordinates are WGS-84 degrees, and image
 * dimensions are pixels. `lowMemory` describes a process policy, not measured
 * whole-board headroom or a hard resident-memory limit.
 */
struct MapConfig {
    Point start, end;
    int jpegWidth = 700, jpegHeight = 1100;
    unsigned long quality = 95;
    double mapPixelSize = .1;
    double minCoverage = .999, referencePlane = std::numeric_limits<double>::quiet_NaN();
    double mosaicReferencePlane = std::numeric_limits<double>::quiet_NaN();
    int mosaicSeamScale = 8, mosaicBlendBands = 3;
    bool mosaicExposure = true;
    std::string mosaicSeamMethod = "graphcut";
    int mosaicOverviewPixels = 4000000;
    int mosaicPhotoCacheMiB = 256;
    int mosaicOverviewSourceMaxSide = 512;
    bool mosaicFillGaps = false;
    bool lowMemory = false;
    int memoryMiB = 192, reserveMiB = 192, blockCacheMiB = 8;
    std::filesystem::path workingDirectory;
    std::filesystem::path preparedSourceDirectory;
    std::filesystem::path sourceDirectory;
    std::filesystem::path photoDirectory, cameraProfile;
    double maxPoseUncertainty = 5;
    bool allowRecordedPose = false;
    bool incrementalPreview = false;
    int liveUpdateSeconds = 30, liveBatchPhotos = 4, liveCpuPercent = 20;
    std::filesystem::path livePauseFile, liveFinalFile;
    unsigned mosaicThreads = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    std::string outputFormat = "orthomap";
    int xyzZoom = 19, xyzMinZoom = 19;
    bool xyzCompleteOnly = false;
    std::string mode = "simple", orthoEngine = "simple", blend = "seam";
    std::filesystem::path sourceCsv, originalSourceCsv, captureManifest, output;
    std::optional<std::size_t> expectedSaved;
    bool allowIncomplete = false;
    std::optional<UtcDateTime> startupUtc;
    std::string weatherPreset = "unchanged";
    std::filesystem::path weatherReadback;
};

/** @brief Return whether a supported processing mode uses the Great mosaic. */
inline bool mosaicMode(const std::string& mode) {
    return mode == "great";
}

/**
 * @brief Select one of the two supported processing modes and apply its defaults.
 * @param[in,out] config Settings to update.
 * @param[in] mode Exact supported name: `simple` or `great`.
 * @throws std::runtime_error If @p mode is not supported.
 * @note Explicit INI values are loaded afterward and may override mode defaults.
 */
inline void mapMode(MapConfig& config, const std::string& mode) {
    if (mode != "simple" && mode != "great")
        throw std::runtime_error("processing.mode must be simple or great");
    config.mode = mode;
    config.orthoEngine = mosaicMode(mode) ? "mosaic" : "simple";
    if (mode == "great") {
        config.mosaicSeamMethod = "graphcut";
        config.mosaicOverviewPixels = 8000000;
        config.mosaicPhotoCacheMiB = 1024;
        config.mosaicOverviewSourceMaxSide = 512;
        config.mosaicFillGaps = true;
        config.mosaicSeamScale = 4;
        config.mosaicBlendBands = 4;
    }
}

/** @brief Give a stable descriptive name to a Great seam-and-blend renderer. */
inline std::string mosaicRenderMethod(const std::string& method) {
    return "overview_" + (method == "graphcut" ? std::string("graph_cut") : method) + "_multiband";
}

/** @brief Normalize INI paths to the host filesystem's UTF-8 path parser. */
inline std::filesystem::path configPath(std::string text) {
    for (char& ch : text) {
        if (ch == '\\') ch = '/';
    }
    return std::filesystem::u8path(text);
}

/**
 * @brief Load and validate a survey INI plus its capture manifest or source CSV.
 * @param[in] path Configuration file path; relative inputs resolve beside it.
 * @param[in] modeOverride Optional exact CLI mode override, applied before defaults.
 * @return A complete configuration whose source CSV exists and matches its contract.
 * @throws std::runtime_error For unknown settings, invalid values, inconsistent
 * capture metadata, unsupported profile combinations, or missing inputs.
 */
inline MapConfig loadMapConfig(const std::filesystem::path& path, const std::string& modeOverride = {}) {
    const IniFile ini(path);
    ini.validate({"input.capture_manifest", "input.source_csv", "input.allow_incomplete",
        "input.photo_directory", "input.camera_profile", "input.max_pose_uncertainty_m", "input.pose_policy",
        "start.lat", "start.lon", "end.lat", "end.lon", "processing.mode",
        "map.pixel_size_m", "map.tile_width", "map.tile_height",
        "simple.reference_plane_amsl_m", "simple.blend",
        "mosaic.seam_method", "mosaic.seam_scale", "mosaic.blend_bands", "mosaic.exposure_compensation",
        "mosaic.overview_pixels",
        "mosaic.photo_cache_mib", "mosaic.threads", "mosaic.reference_plane_amsl_m",
        "mosaic.overview_source_max_side",
        "mosaic.fill_gaps",
        "live.update_seconds", "live.batch_photos", "live.cpu_percent", "live.pause_file", "live.final_file",
        "runtime.profile", "runtime.memory_mib", "runtime.reserve_mib",
        "output.directory", "output.jpeg_quality", "output.format", "tiles.zoom", "tiles.min_zoom", "tiles.complete_only",
        "quality.min_colour_coverage"});
    const auto base = std::filesystem::absolute(path).parent_path();
    MapConfig config;
    mapMode(config, modeOverride.empty() ? ini.text("processing.mode", "simple") : modeOverride);
    const auto profile = ini.text("runtime.profile", "auto");
    if (profile != "auto" && profile != "desktop" && profile != "low_memory")
        throw std::runtime_error("runtime.profile must be auto, desktop or low_memory");
    const long pages = sysconf(_SC_PHYS_PAGES), pageSize = sysconf(_SC_PAGESIZE);
    config.lowMemory = profile == "low_memory" || (profile == "auto" && pages > 0 && pageSize > 0 &&
        static_cast<unsigned long long>(pages) * pageSize <= 1024ull * 1024 * 1024);
    if (config.lowMemory) {
        config.mosaicOverviewPixels = 250000;
        config.mosaicPhotoCacheMiB = 8;
        config.mosaicThreads = 1;
    }
    config.outputFormat = ini.text("output.format", "orthomap");
    if (config.outputFormat != "orthomap" && config.outputFormat != "xyz")
        throw std::runtime_error("output.format must be orthomap or xyz");
    if (config.outputFormat != "xyz" && (ini.has("tiles.zoom") || ini.has("tiles.min_zoom") ||
                                       ini.has("tiles.complete_only")))
        throw std::runtime_error("tiles settings require output.format=xyz");
    if (config.outputFormat == "xyz" && (ini.has("map.pixel_size_m") || ini.has("map.tile_width") ||
                                       ini.has("map.tile_height")))
        throw std::runtime_error("XYZ grid uses tiles.zoom and fixed 256-pixel tiles; remove map grid settings");
    config.allowIncomplete = ini.boolean("input.allow_incomplete");
    if (int(ini.has("input.capture_manifest")) + int(ini.has("input.source_csv")) +
        int(ini.has("input.photo_directory")) != 1)
        throw std::runtime_error("Specify exactly one input.capture_manifest, input.source_csv or input.photo_directory");
    if (ini.has("input.capture_manifest")) {
        config.captureManifest = base / configPath(ini.text("input.capture_manifest"));
        const IniFile manifest(config.captureManifest);
        manifest.validate({"capture.schema", "capture.status", "capture.source_csv", "capture.planned", "capture.saved",
            "start.lat", "start.lon", "start.agl", "end.lat", "end.lon", "end.agl",
            "camera.lens_profile", "camera.lock_start_altitude", "camera.tile_overlap",
            "acquisition.start_lat", "acquisition.start_lon", "acquisition.end_lat", "acquisition.end_lon",
            "simulation.weather_preset", "simulation.utc_datetime"});
        if (manifest.text("capture.schema") != "tessavia_capture_v1")
            throw std::runtime_error("Unsupported capture manifest schema");
        const auto status = manifest.text("capture.status");
        if (status == "in_progress" || (status != "complete" && status != "interrupted"))
            throw std::runtime_error("Capture is unfinished or its status is invalid");
        if (status != "complete" && !config.allowIncomplete)
            throw std::runtime_error("Interrupted capture requires input.allow_incomplete=1");
        const auto count = [&](const char* key) {
            const double value = manifest.required(key);
            range(value, 0, 1e9, key);
            if (value != std::floor(value))
                throw std::runtime_error(std::string(key) + " must be integer");
            return static_cast<std::size_t>(value);
        };
        const auto planned = count("capture.planned"), saved = count("capture.saved");
        if (saved == 0 || (planned != 0 && (saved > planned || (status == "complete" && saved != planned))))
            throw std::runtime_error("Capture manifest saved/planned counts disagree with its status");
        config.expectedSaved = saved;
        const auto csv = configPath(manifest.text("capture.source_csv"));
        if (csv.empty() || csv.has_parent_path()) throw std::runtime_error("Manifest CSV must be a local filename");
        config.sourceCsv = config.captureManifest.parent_path() / csv;
        config.start = {manifest.required("start.lat"), manifest.required("start.lon"), manifest.required("start.agl")};
        config.end = {manifest.required("end.lat"), manifest.required("end.lon"), manifest.required("end.agl")};
    } else if (ini.has("input.photo_directory")) {
        config.photoDirectory = base / configPath(ini.text("input.photo_directory"));
        config.cameraProfile = base / configPath(ini.text("input.camera_profile"));
        config.maxPoseUncertainty = ini.numeric("input.max_pose_uncertainty_m", 5);
        range(config.maxPoseUncertainty, 0, 100, "input.max_pose_uncertainty_m");
        const auto policy = ini.text("input.pose_policy", "exposure");
        if (policy != "exposure" && policy != "recorded")
            throw std::runtime_error("input.pose_policy must be exposure or recorded");
        config.allowRecordedPose = policy == "recorded";
        if (config.allowRecordedPose && ini.has("input.max_pose_uncertainty_m"))
            throw std::runtime_error("Recorded-pose input cannot certify max_pose_uncertainty_m; use exposure policy for that gate");
        if (!std::filesystem::is_directory(config.photoDirectory) || !std::filesystem::is_regular_file(config.cameraProfile))
            throw std::runtime_error("AI Camera input requires an existing photo_directory and camera_profile");
    } else {
        config.sourceCsv = base / configPath(ini.text("input.source_csv"));
    }
    const bool bounds = ini.has("start.lat") || ini.has("start.lon") || ini.has("end.lat") || ini.has("end.lon");
    if (bounds || config.captureManifest.empty()) {
        config.start.lat = ini.required("start.lat"); config.start.lon = ini.required("start.lon");
        config.end.lat = ini.required("end.lat"); config.end.lon = ini.required("end.lon");
    }
    for (const auto& point : {config.start, config.end}) {
        range(point.lat, -80, 84, "map latitude"); range(point.lon, -180, 180, "map longitude");
    }
    if (config.start.lat == config.end.lat || config.start.lon == config.end.lon ||
        std::abs(config.start.lon - config.end.lon) >= 180)
        throw std::runtime_error("Map needs nonzero bounds in one supported UTM region");
    config.mapPixelSize = ini.numeric("map.pixel_size_m", .1);
    range(config.mapPixelSize, .05, 2, "map.pixel_size_m");
    const auto integer = [&](const char* key, int fallback, int low, int high) {
        const auto value = ini.numeric(key, fallback);
        range(value, low, high, key);
        if (value != std::floor(value)) throw std::runtime_error(std::string(key) + " must be integer");
        return static_cast<int>(value);
    };
    config.jpegWidth = integer("map.tile_width", 700, 1, 4096);
    config.jpegHeight = integer("map.tile_height", 1100, 1, 4096);
    config.xyzZoom = integer("tiles.zoom", 19, 0, 22);
    config.xyzMinZoom = integer("tiles.min_zoom", config.xyzZoom, std::max(0, config.xyzZoom - 2), config.xyzZoom);
    config.xyzCompleteOnly = ini.boolean("tiles.complete_only");
    config.quality = static_cast<unsigned long>(integer("output.jpeg_quality", 95, 1, 100));
    if (ini.has("simple.reference_plane_amsl_m"))
        config.referencePlane = ini.required("simple.reference_plane_amsl_m");
    if (ini.has("mosaic.reference_plane_amsl_m")) {
        config.mosaicReferencePlane = ini.required("mosaic.reference_plane_amsl_m");
    }
    config.mosaicSeamScale = integer("mosaic.seam_scale", config.mosaicSeamScale, 4, 64);
    config.mosaicBlendBands = integer("mosaic.blend_bands", config.mosaicBlendBands, 1, 4);
    config.mosaicExposure = ini.boolean("mosaic.exposure_compensation", config.mosaicExposure);
    config.mosaicOverviewPixels = integer("mosaic.overview_pixels", config.mosaicOverviewPixels, 250000, 8000000);
    config.mosaicPhotoCacheMiB = integer("mosaic.photo_cache_mib", config.mosaicPhotoCacheMiB,
                                       config.lowMemory ? 1 : 128, config.lowMemory ? 16 : 8192);
    config.mosaicOverviewSourceMaxSide = integer("mosaic.overview_source_max_side", config.mosaicOverviewSourceMaxSide, 64, 2048);
    config.mosaicFillGaps = ini.boolean("mosaic.fill_gaps", config.mosaicFillGaps);
    if (config.mosaicFillGaps && !mosaicMode(config.mode))
        throw std::runtime_error("mosaic.fill_gaps requires Great processing");
    config.mosaicSeamMethod = ini.text("mosaic.seam_method", config.mosaicSeamMethod);
    if (config.mosaicSeamMethod != "graphcut" && config.mosaicSeamMethod != "dp" && config.mosaicSeamMethod != "voronoi")
        throw std::runtime_error("mosaic.seam_method must be graphcut, dp or voronoi");
    config.mosaicThreads = static_cast<unsigned>(integer("mosaic.threads", static_cast<int>(config.mosaicThreads), 1, 32));
    config.blend = ini.text("simple.blend", "seam");
    if (config.blend != "seam" && config.blend != "feather")
        throw std::runtime_error("simple.blend must be seam or feather");
    config.minCoverage = ini.numeric("quality.min_colour_coverage", config.outputFormat == "xyz" ? 1 : .999);
    range(config.minCoverage, .001, 1, "quality.min_colour_coverage");
    if (config.outputFormat == "xyz" && config.minCoverage != 1)
        throw std::runtime_error("Opaque XYZ JPEG tiles require quality.min_colour_coverage=1");
    config.output = base / configPath(ini.text("output.directory",
        config.outputFormat == "xyz" ? "Pictures/XYZ" : "maps"));
    config.memoryMiB = integer("runtime.memory_mib", 192, 128, 192);
    config.reserveMiB = integer("runtime.reserve_mib", 192, 192, 4096);
    config.liveUpdateSeconds = integer("live.update_seconds", 30, 1, 3600);
    config.liveBatchPhotos = integer("live.batch_photos", 4, 1, 4096);
    config.liveCpuPercent = integer("live.cpu_percent", 20, 1, 100);
    if (ini.has("live.pause_file")) config.livePauseFile = base / configPath(ini.text("live.pause_file"));
    if (ini.has("live.final_file")) config.liveFinalFile = base / configPath(ini.text("live.final_file"));
    if (config.lowMemory && (config.mode != "great" || config.outputFormat != "xyz" ||
        config.mosaicSeamMethod != "graphcut" ||
        !config.mosaicFillGaps || config.xyzCompleteOnly || config.mosaicThreads != 1 ||
        config.mosaicOverviewPixels > 250000 || config.mosaicBlendBands > 4))
        throw std::runtime_error("Low-memory profile requires Great XYZ with graphcut seams, gap filling, "
                                 "one worker and at most 250000 overview pixels; no omitted tiles");
    if (!config.photoDirectory.empty() && (!config.lowMemory || config.mode != "great" || config.outputFormat != "xyz" ||
        !std::isfinite(config.mosaicReferencePlane)))
        throw std::runtime_error("AI Camera photos require low_memory GREAT XYZ and mosaic.reference_plane_amsl_m in FC navigation datum");
    if (config.photoDirectory.empty() && (ini.has("input.camera_profile") || ini.has("input.max_pose_uncertainty_m") || ini.has("input.pose_policy")))
        throw std::runtime_error("AI Camera settings require input.photo_directory");
    if (config.photoDirectory.empty() && !std::filesystem::is_regular_file(config.sourceCsv))
        throw std::runtime_error("Missing source CSV: " + config.sourceCsv.u8string());
    return config;
}
}
#endif
