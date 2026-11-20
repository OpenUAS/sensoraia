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
 * @file map_projection.h
 * @brief Convert between WGS-84 coordinates and one local UTM grid.
 * @details Map pixels are laid out in metres in a single zone so resampling
 * and world-file spacing stay linear across a small survey. Inputs outside
 * the supported UTM latitude, hemisphere, or zone span are rejected rather
 * than silently producing a plausible-looking but invalid grid.
 */
#ifndef PANGEA_MAP_PROJECTION_H
#define PANGEA_MAP_PROJECTION_H

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pangea_survey {
/** @brief Centre and pixel geometry for a projected image tile. */
struct MapTile {
    double east, north, pixelSize, heading;
    int epsg;
};

/**
 * @brief Forward and inverse WGS-84 Transverse Mercator conversion for one UTM zone.
 * @details This projection locates simulator imagery; it does not establish
 * camera calibration, relief correction, or surveyed positional accuracy.
 */
class MapProjection {
    int zone_;
    bool north_;
    static constexpr double a = 6378137, e2 = 0.0066943799901413165, k = 0.9996;
    static constexpr double radians = 0.01745329251994329577;
    /** @brief Select a standard six-degree UTM zone.
     * @param latitude WGS-84 latitude in degrees.
     * @param longitude WGS-84 longitude in degrees.
     * @return Zone number from 1 to 60.
     * @throws std::runtime_error For nonfinite or unsupported coordinates.
     * @details Clamp the positive antimeridian endpoint to zone 60 and
     * use regular longitudinal bands rather than regional exceptions. */
    static int zone(double latitude, double longitude) {
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || latitude < -80 || latitude > 84 ||
            longitude < -180 || longitude > 180)
            throw std::runtime_error("Mapping grid requires WGS-84 latitude -80..84 and longitude -180..180");
        return std::clamp(static_cast<int>(std::floor((longitude + 180) / 6)) + 1, 1, 60);
    }
public:
    /**
     * @brief Choose the UTM zone and hemisphere from a representative coordinate.
     * @param[in] latitude WGS-84 latitude in degrees.
     * @param[in] longitude WGS-84 longitude in degrees.
     * @throws std::runtime_error If the coordinate is outside supported UTM limits.
     */
    MapProjection(double latitude, double longitude)
        : zone_(zone(latitude, longitude)), north_(latitude >= 0) {}
    /** @brief Get the CRS identifier for this WGS-84 UTM zone.
     * @return EPSG code in the northern 326xx or southern 327xx family.
     * @details Include the hemisphere so consumers can interpret false northing. */
    int epsg() const { return (north_ ? 32600 : 32700) + zone_; }
    /** @brief Return the selected zone's central meridian in radians. */
    double centralMeridian() const { return (zone_ * 6 - 183) * radians; }
    /** @brief Estimate grid convergence in degrees at a geographic position. */
    double convergence(double latitude, double longitude) const {
        return std::atan(std::tan(longitude * radians - centralMeridian()) *
                         std::sin(latitude * radians)) / radians;
    }
    /** @brief Estimate local UTM point scale at a geographic position. */
    double scale(double latitude, double longitude) const {
        const double p = latitude * radians;
        const double A = std::cos(p) * (longitude * radians - centralMeridian());
        const double C = e2 / (1 - e2) * std::pow(std::cos(p), 2);
        const double T = std::pow(std::tan(p), 2);
        return k * (1 + (1 + C) * A * A / 2 +
                    (5 - 4 * T + 42 * C + 13 * C * C - 28 * e2 / (1 - e2)) * std::pow(A, 4) / 24);
    }
    /**
     * @brief Project a geographic position into UTM easting and northing.
     * @param[in] latitude WGS-84 latitude in degrees.
     * @param[in] longitude WGS-84 longitude in degrees.
     * @return Easting and northing in metres, including standard UTM false origins.
     * @throws std::runtime_error If the coordinate crosses the selected hemisphere
     * or lies too far from the selected zone's central meridian.
     */
    std::pair<double, double> forward(double latitude, double longitude) const {
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || latitude < -80 || latitude > 84 ||
            (latitude >= 0) != north_ || std::abs(longitude * radians - centralMeridian()) > 0.105)
            throw std::runtime_error("Area crosses a hemisphere or extends too far from its UTM zone");
        const double p = latitude * radians, l = longitude * radians;
        const double s = std::sin(p), c = std::cos(p), t = std::tan(p);
        const double ep = e2 / (1 - e2), n = a / std::sqrt(1 - e2 * s * s);
        const double T = t * t, C = ep * c * c, A = c * (l - centralMeridian());
        const double e4 = e2 * e2, e6 = e4 * e2;
        const double M = a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * p -
            (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin(2 * p) +
            (15 * e4 / 256 + 45 * e6 / 1024) * std::sin(4 * p) -
            35 * e6 / 3072 * std::sin(6 * p));
        return {500000 + k * n * (A + (1 - T + C) * std::pow(A, 3) / 6 +
                    (5 - 18 * T + T * T + 72 * C - 58 * ep) * std::pow(A, 5) / 120),
                (north_ ? 0 : 10000000) + k * (M + n * t * (A * A / 2 +
                    (5 - T + 9 * C + 4 * C * C) * std::pow(A, 4) / 24 +
                    (61 - 58 * T + T * T + 600 * C - 330 * ep) * std::pow(A, 6) / 720))};
    }
    /**
     * @brief Convert UTM easting/northing back to WGS-84 latitude/longitude.
     * @param[in] east UTM easting in metres.
     * @param[in] north UTM northing in metres.
     * @return Latitude and longitude in degrees.
     * @throws std::runtime_error If either projected coordinate is nonfinite.
     */
    std::pair<double, double> inverse(double east, double north) const {
        if (!std::isfinite(east) || !std::isfinite(north))
            throw std::runtime_error("Nonfinite mapping-grid coordinates");
        const double e4 = e2 * e2, e6 = e4 * e2;
        const double mu = (north - (north_ ? 0 : 10000000)) /
            (k * a * (1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256));
        const double e1 = (1 - std::sqrt(1 - e2)) / (1 + std::sqrt(1 - e2));
        const double p = mu + (3 * e1 / 2 - 27 * std::pow(e1, 3) / 32) * std::sin(2 * mu) +
            (21 * e1 * e1 / 16 - 55 * std::pow(e1, 4) / 32) * std::sin(4 * mu) +
            151 * std::pow(e1, 3) / 96 * std::sin(6 * mu) +
            1097 * std::pow(e1, 4) / 512 * std::sin(8 * mu);
        const double s = std::sin(p), c = std::cos(p), t = std::tan(p);
        const double ep = e2 / (1 - e2), C = ep * c * c, T = t * t;
        const double n = a / std::sqrt(1 - e2 * s * s);
        const double r = a * (1 - e2) / std::pow(1 - e2 * s * s, 1.5);
        const double D = (east - 500000) / (n * k);
        return {(p - n * t / r * (D * D / 2 -
                    (5 + 3 * T + 10 * C - 4 * C * C - 9 * ep) * std::pow(D, 4) / 24 +
                    (61 + 90 * T + 298 * C + 45 * T * T - 252 * ep - 3 * C * C) *
                    std::pow(D, 6) / 720)) / radians,
                (centralMeridian() + (D - (1 + 2 * T + C) * std::pow(D, 3) / 6 +
                    (5 - 2 * C + 28 * T - 3 * C * C + 8 * ep + 24 * T * T) *
                    std::pow(D, 5) / 120) / c) / radians};
    }
};
}
#endif
