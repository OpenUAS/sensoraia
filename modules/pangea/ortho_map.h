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
 * @file ortho_map.h
 * @brief Describe the validated photo set and render projected map windows.
 * @details Rendering retains source provenance and a per-pixel confidence class
 * alongside colour so publication can distinguish observed colour from
 * completion. The API keeps rendering separate from JPEG/XYZ publication,
 * allowing both output formats to share the same map engine.
 */
#ifndef PANGEA_ORTHO_MAP_H
#define PANGEA_ORTHO_MAP_H

#include "map_config.h"
#include "map_projection.h"
#include "camera_projection.h"
#include <functional>
#include <memory>
#include <vector>

namespace pangea_survey {
/** @brief UTM-aligned extent and tile count for one requested map area. */
struct OrthoLayout {
    double west = 0, north = 0, east = 0, south = 0;
    std::size_t columns = 0, rows = 0;
    int epsg = 0;
};
/**
 * @brief Compute a north-up, non-overlapping image grid for the requested bounds.
 * @param[in] config Validated geographic bounds, pixel size, and tile dimensions.
 * @return Projected bounds, EPSG code, and whole-tile row/column counts.
 */
OrthoLayout orthoLayout(const MapConfig& config);
/**
 * @brief One fixed-session source image and its camera/pose evidence.
 * @details Coordinates are WGS-84 degrees; elevations are AMSL metres; camera
 * intrinsics are pixels; timestamps are 100 ns monotonic counts. The optional
 * capture cell lets Great use the planned flight grid without inventing pose data.
 */
struct OrthoSource {
    std::filesystem::path path;
    std::optional<CameraPlane> camera;
    double latitude = 0, longitude = 0, altitude = 0;
    double focal = 0, cx = 0, cy = 0;
    double focalY = 0, poseUncertainty = 0;
    double pitch = 0, bank = 0, heading = 0, fov = 0;
    long long poseTime = 0, frameTime = 0;
    int width = 0, height = 0;
    std::optional<UtcDateTime> captureUtc;
    double utcAbsoluteSeconds = 0;
    long long utcRead100ns = 0;
    double referencePlane = std::numeric_limits<double>::quiet_NaN();
    bool distorted = false;
    LensEffects lens;
    std::optional<std::pair<int, int>> captureCell;
};
/**
 * @brief Rendered BGRA map plus coverage, source list, and method provenance.
 * @details Confidence is independent of colour: zero means no supported colour,
 * and Great's reconstruction classes must not be reported as observed camera
 * geometry. `validFraction` is image colour coverage, not map accuracy.
 */
struct OrthoImage {
    MapTile map{};
    int width = 0, height = 0;
    std::vector<unsigned char> bgra, confidence;
    std::vector<OrthoSource> sources;
    double validFraction = 0;
    double geometryFraction = 0, corroboratedFraction = 0;
    std::string mode = "simple";
    double referencePlane = std::numeric_limits<double>::quiet_NaN();
    int seamScale = 0, blendBands = 0;
    std::string seamMethod;
    bool exposureCompensated = false;
    double reconstructedFraction = 0, reconstructionBlendFraction = 0;
};
/**
 * @brief Load a fixed-session source set and render its projected tiles/windows.
 * @details The implementation owns bounded decoded-photo and rendered-block
 * caches. Returned images own their pixel/provenance vectors; cancellation is
 * cooperative and throws before an incomplete render can be treated as complete.
 */
class Orthorectifier {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    /**
     * @brief Validate source CSV rows and derive the requested output grid.
     * @param[in] config Validated mapping configuration.
     * @param[in] csv Source manifest with one consistent fixed-session camera setup.
     * @throws std::runtime_error If any source, calibration, pose, or session contract fails.
     */
    Orthorectifier(const MapConfig& config, const std::filesystem::path& csv);
    /** @brief Release the render state and its caches.
     * @details Defined out of line where the private implementation is complete. */
    ~Orthorectifier();
    /** @brief Disallow copying the owned render state. */
    Orthorectifier(const Orthorectifier&) = delete;
    /** @brief Disallow copy assignment of the owned render state. */
    Orthorectifier& operator=(const Orthorectifier&) = delete;
    /** @brief Return output tile columns. */
    std::size_t columns() const;
    /** @brief Return output tile rows. */
    std::size_t rows() const;
    /** @brief Return total requested map tiles. */
    std::size_t size() const;
    /** @brief Return the validated common capture UTC target, when present. */
    std::optional<UtcDateTime> captureUtc() const;
    /** @brief Return the validated source count without decoding pixels. */
    std::size_t sourceCount() const;
    /**
     * @brief Hash canonical source paths and current JPEG contents in source order.
     * @param[in] stopped Cooperative cancellation predicate.
     * @return Content fingerprint used to detect changes to admitted photographs.
     */
    std::string sourceFingerprint(const std::function<bool()>& stopped) const;
    /**
     * @brief Prepare shared source rasters/mosaic state before repeated rendering.
     * @param[in] stopped Cooperative cancellation predicate.
     */
    void prepareSources(const std::function<bool()>& stopped);
    /**
     * @brief Render one configured output tile.
     * @param[in] index Row-major tile index in `[0, size())`.
     * @param[in] stopped Cooperative cancellation predicate.
     * @param[in] progress Optional render fraction callback in `[0, 1]`.
     * @return Owned image pixels, confidence classes, and source provenance.
     * @throws std::runtime_error If index is invalid, rendering fails, or canceled.
     */
    OrthoImage render(std::size_t index, const std::function<bool()>& stopped,
                     const std::function<void(double)>& progress = {});
    /**
     * @brief Render an arbitrary bounded window of the configured map grid.
     * @param[in] originX Leftmost map-grid pixel.
     * @param[in] originY Topmost map-grid pixel.
     * @param[in] width Window width in pixels.
     * @param[in] height Window height in pixels.
     * @param[in] stopped Cooperative cancellation predicate.
     * @param[in] progress Optional render fraction callback in `[0, 1]`.
     * @return Owned image and coverage metadata for the requested window.
     * @throws std::runtime_error If bounds exceed the configured area or rendering fails.
     */
    OrthoImage renderWindow(std::size_t originX, std::size_t originY, int width, int height,
                           const std::function<bool()>& stopped,
                           const std::function<void(double)>& progress = {});
    static void selfTest();
    /**
    * @brief Rectify JPEG sources into working JPEGs and write a new CSV.
     * @param[in] csv Original source CSV.
     * @param[in] directory New directory for normalized JPEGs and CSV.
     * @param[in] stopped Cooperative cancellation predicate.
    * @param[in] workers Upper bound on independent source-preparation workers.
    * @param[in] memoryMiB Working-memory budget used to reduce the worker count.
     * @return Path to the normalized source CSV.
     * @throws std::runtime_error If source decoding, rectification, or publication fails.
     */
    static std::filesystem::path normalizeSources(const std::filesystem::path& csv,
        const std::filesystem::path& directory, const std::function<bool()>& stopped,
        unsigned workers = 1, unsigned memoryMiB = 128);
};
}
#endif
