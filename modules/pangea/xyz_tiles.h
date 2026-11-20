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
 * @file xyz_tiles.h
 * @brief Plan interoperable XYZ tile ranges in Web Mercator.
 * @details XYZ uses a north-origin tile grid and the Web-Mercator latitude
 * limit. Planning snaps only floating-point noise at exact tile boundaries so
 * a request does not gain or lose an entire row of tiles from rounding error.
 */
#ifndef PANGEA_XYZ_TILES_H
#define PANGEA_XYZ_TILES_H

#include "map_config.h"
#include "map_projection.h"
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace pangea_survey {
/** @brief Inclusive rectangular tile range at one zoom level. */
struct XyzRange {
    int zoom = 19;
    std::uint32_t firstX = 0, firstY = 0, lastX = 0, lastY = 0;
    /** @brief Count columns in the ordered inclusive X range.
     * @return One column when both X endpoints are the same. */
    std::size_t columns() const { return static_cast<std::size_t>(lastX) - firstX + 1; }
    /** @brief Count rows in the ordered inclusive Y range.
     * @return One row when both Y endpoints are the same. */
    std::size_t rows() const { return static_cast<std::size_t>(lastY) - firstY + 1; }
    /** @brief Count tiles in the validated rectangular range.
     * @return Number of tiles at this zoom; both edges are included. */
    std::size_t size() const { return columns() * rows(); }
};

/**
 * @brief Convert Web-Mercator geographic coordinates to canonical XYZ addresses.
 * @details Tile pixels use a 256-pixel extent and paths are `zoom/x/y.jpeg`;
 * the output is an ordinary interoperable XYZ tile pyramid.
 */
class XyzTiles {
public:
    static constexpr int tileSize = 256;
    static constexpr double radius = 6378137;
    static constexpr double radians = 0.01745329251994329577;
    static constexpr double latitudeLimit = 85.0511287798066;
    /** @brief Number of tiles across one axis at this zoom level. */
    static double dimension(int zoom) {
        if (zoom < 0 || zoom > 22) throw std::runtime_error("XYZ zoom must be an integer from 0 to 22");
        return std::ldexp(1.0, zoom);
    }
    /** @brief Equatorial Web-Mercator metres per pixel at this zoom. */
    static double metresPerPixel(int zoom) {
        return 2 * 3.14159265358979323846 * radius / (tileSize * dimension(zoom));
    }
    /** @brief Ground metres per pixel at a latitude after Mercator scale correction. */
    static double groundPixel(int zoom, double latitude) {
        if (!std::isfinite(latitude) || std::abs(latitude) > latitudeLimit)
            throw std::runtime_error("Latitude is outside the Web-Mercator tile grid");
        return metresPerPixel(zoom) * std::cos(latitude * radians);
    }
    /**
     * @brief Convert WGS-84 latitude/longitude to fractional tile coordinates.
     * @param[in] latitude Latitude in degrees within Web-Mercator limits.
     * @param[in] longitude Longitude in degrees.
     * @param[in] zoom XYZ zoom level from 0 through 22.
     * @return Fractional tile x and y, with y increasing southward.
     */
    static std::pair<double, double> tileCoordinate(double latitude, double longitude, int zoom) {
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || std::abs(latitude) > latitudeLimit ||
            longitude < -180 || longitude > 180)
            throw std::runtime_error("Invalid Web-Mercator latitude/longitude");
        const double n = dimension(zoom);
        return {(longitude + 180) / 360 * n,
                (1 - std::asinh(std::tan(latitude * radians)) / 3.14159265358979323846) / 2 * n};
    }
    /** @brief Convert fractional tile coordinates back to latitude/longitude. */
    static std::pair<double, double> geographic(double x, double y, int zoom) {
        const double n = dimension(zoom);
        if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x > n || y > n)
            throw std::runtime_error("Invalid Web-Mercator tile coordinates");
        return {std::atan(std::sinh(3.14159265358979323846 * (1 - 2 * y / n))) / radians,
                x / n * 360 - 180};
    }
    /** @brief Return the geographic centre of one global pixel address. */
    static std::pair<double, double> pixel(std::uint64_t x, std::uint64_t y, int zoom) {
        return geographic((static_cast<double>(x) + .5) / tileSize,
                          (static_cast<double>(y) + .5) / tileSize, zoom);
    }
    /** @brief Build the canonical `zoom/x/y.jpeg` path after range validation. */
    static std::filesystem::path path(const std::filesystem::path& root, int zoom,
                                     std::uint32_t x, std::uint32_t y) {
        const double n = dimension(zoom);
        if (x >= n || y >= n) throw std::runtime_error("XYZ tile address is outside its zoom level");
        return root / std::to_string(zoom) / std::to_string(x) / (std::to_string(y) + ".jpeg");
    }
};

/**
 * @brief Derive requested XYZ bounds, map grid, and per-zoom tile ranges.
 * @details The tile rectangle is aligned in Web Mercator while the source
 * mapper renders in UTM; a small working-grid pad contains the resampling taps.
 * It is not a claim that imagery exists over every requested output pixel.
 */
struct XyzPlan {
    XyzRange base;
    int minZoom = 19;
    double north = 0, south = 0, west = 0, east = 0;
    /**
     * @brief Construct a bounded tile pyramid from the configured geographic bounds.
     * @param[in] config Validated XYZ settings and requested WGS-84 bounds.
     * @throws std::runtime_error If zooms, bounds, or requested tile count are invalid.
     */
    explicit XyzPlan(const MapConfig& config) : minZoom(config.xyzMinZoom) {
        if (minZoom < 0 || minZoom > config.xyzZoom || config.xyzZoom - minZoom > 2)
            throw std::runtime_error("XYZ min_zoom must be within two levels below zoom");
        const auto nw = XyzTiles::tileCoordinate(std::max(config.start.lat, config.end.lat),
                                                 std::min(config.start.lon, config.end.lon), minZoom);
        const auto se = XyzTiles::tileCoordinate(std::min(config.start.lat, config.end.lat),
                                                 std::max(config.start.lon, config.end.lon), minZoom);
        const auto n = static_cast<std::uint32_t>(XyzTiles::dimension(minZoom));
        const auto snap = [](double value) {
            const double integer = std::round(value);
            return std::abs(value - integer) <= 8 * std::numeric_limits<double>::epsilon() *
                   std::max(1.0, std::abs(value)) ? integer : value;
        };
        const auto first = [n, &snap](double value) {
            value = snap(value);
            return static_cast<std::uint32_t>(std::clamp(std::floor(value), 0.0, double(n - 1)));
        };
        const auto last = [n, &snap](double value) {
            value = snap(value);
            return static_cast<std::uint32_t>(std::clamp(std::ceil(value) - 1, 0.0, double(n - 1)));
        };
        const auto factor = std::uint32_t{1} << (config.xyzZoom - minZoom);
        base = {config.xyzZoom, first(nw.first) * factor, first(nw.second) * factor,
                (last(se.first) + 1) * factor - 1, (last(se.second) + 1) * factor - 1};
        if (base.firstX > base.lastX || base.firstY > base.lastY || size() > 100000)
            throw std::runtime_error("Invalid XYZ bounds or more than 100000 requested tiles");
        const auto corner = XyzTiles::geographic(base.firstX, base.firstY, base.zoom);
        const auto opposite = XyzTiles::geographic(double(base.lastX) + 1, double(base.lastY) + 1, base.zoom);
        north = corner.first; west = corner.second; south = opposite.first; east = opposite.second;
    }
    /** @brief Return the aligned tile rectangle at a zoom within this plan. */
    XyzRange range(int zoom) const {
        if (zoom < minZoom || zoom > base.zoom) throw std::runtime_error("Zoom outside the XYZ export plan");
        const auto shift = base.zoom - zoom;
        return {zoom, base.firstX >> shift, base.firstY >> shift, base.lastX >> shift, base.lastY >> shift};
    }
    /** @brief Count all requested tiles across the configured zoom pyramid. */
    std::size_t size() const {
        std::size_t count = 0;
        for (int zoom = minZoom; zoom <= base.zoom; ++zoom) count += range(zoom).size();
        return count;
    }
    /** @brief Derive the padded UTM working-grid settings needed for resampling. */
    MapConfig workingConfig(const MapConfig& original) const {
        auto result = original;
        const double latitude = (north + south) / 2, longitude = (west + east) / 2;
        const MapProjection projection(latitude, longitude);
        result.mapPixelSize = std::clamp(XyzTiles::groundPixel(base.zoom, latitude) *
                                         projection.scale(latitude, longitude), .05, 2.0);
        const double pad = 4 * result.mapPixelSize / XyzTiles::radius / XyzTiles::radians;
        result.start.lat = north + pad; result.end.lat = south - pad;
        result.start.lon = west - pad / std::cos(latitude * XyzTiles::radians);
        result.end.lon = east + pad / std::cos(latitude * XyzTiles::radians);
        result.jpegWidth = result.jpegHeight = 512;
        return result;
    }
    /**
     * @brief Reject any existing tile or partial file in the output plan.
     * @param[in] root XYZ output directory.
     * @throws std::runtime_error If any planned destination already exists.
     */
    void checkOutput(const std::filesystem::path& root) const {
        for (int zoom = minZoom; zoom <= base.zoom; ++zoom) {
            const auto r = range(zoom);
            for (auto x = r.firstX; x <= r.lastX; ++x) for (auto y = r.firstY; y <= r.lastY; ++y) {
                const auto tile = XyzTiles::path(root, zoom, x, y);
                if (std::filesystem::exists(tile) || std::filesystem::exists(tile.string() + ".partial"))
                    throw std::runtime_error("Refusing to overwrite XYZ tile: " + tile.u8string());
            }
        }
    }
};
}
#endif
