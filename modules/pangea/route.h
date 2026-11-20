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
 * @file route.h
 * @brief Parse capture-route settings and evaluate the simulated flight path.
 * @details Route interpolation is deliberately simple and deterministic: the
 * capture generator needs repeatable positions and altitude policy, not a
 * navigation controller. Terrain-relative and start-locked altitude are kept
 * separate because changing the reference mid-session would corrupt map scale.
 */
#ifndef PANGEA_ROUTE_H
#define PANGEA_ROUTE_H

#include "utc_time.h"
#include "image_framing.h"
#include "camera_optics.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace pangea_survey {
constexpr double pi = 3.14159265358979323846;
constexpr double earthRadius = 6371008.8;

/** @brief Geographic route point; latitude/longitude are degrees and AGL is metres. */
struct Point {
    double lat = 0;
    double lon = 0;
    double agl = 120;
};

/**
 * @brief Validated camera, route, simulator-startup, and output settings.
 * @details `fov` is vertical degrees; positions are WGS-84 and altitudes are
 * metres. Startup weather/time readback and capture timing remain provenance,
 * not a sensor exposure clock.
 */
struct Config {
    Point start;
    Point end;
    double speed = 20;
    double interval = 4;
    double settle = 10;
    double fov = 60;
    std::string lensProfile = "simconnect";
    LensEffects lens;
    CameraExifSettings exifSettings;
    int grab = 2;
    double tileSettle = 0.05, tileOverlap = 0.15;
    bool captureMargin = false;
    double mapCameraAgl = 0;
    int captureInset = 0;
    unsigned long quality = 92;
    int jpegWidth = 2028, jpegHeight = 1520;
    bool lockStartAltitude = true;
    bool ivyEnabled = false;
    std::string ivyAddress = "127.255.255.255:2010";
    int ivyAircraftId = -1;
    std::filesystem::path output;
    std::string startupFlight = "auto";
    std::string startupAirport = "EHRD";
    bool autoEnterFlight = true;
    bool startupMaximizeMinimized = true;
    double startupTimeout = 0;
    std::optional<UtcDateTime> startupUtc;
    std::string weatherPreset = "unchanged";
    std::filesystem::path weatherReadback;
    std::filesystem::path iniDirectory;

    /** @brief Check whether an enabled timed capture has reached its deadline.
     * @param elapsed Seconds elapsed in the session.
     * @param nextPhoto Scheduled time in the same elapsed-time domain.
     * @return True when timed capture is enabled and due. */
    bool timedPhotoDue(double elapsed, double nextPhoto) const {
        return interval > 0 && elapsed >= nextPhoto;
    }
    /** @brief Compute field of view at the selected map-camera height.
     * @return Field of view in degrees.
     * @details Preserve the starting ground footprint by scaling the tangent
     * of the half-angle, not the angle itself. */
    double cameraFov() const {
        return mapCameraAgl == 0 ? fov :
            360 / pi * std::atan(start.agl / mapCameraAgl * std::tan(fov * pi / 360));
    }
    /** @brief Resolve the camera's height above ground.
     * @param nominalAgl Route height in metres.
     * @return Configured map-camera height, or @p nominalAgl if unset. */
    double cameraAgl(double nominalAgl) const {
        return mapCameraAgl == 0 ? nominalAgl : mapCameraAgl;
    }
    /** @brief Check for the calibrated Raspberry Pi AI camera optics path.
     * @return True when the route selects that profile. */
    bool piCamera() const { return lensProfile == "raspberry_pi_ai"; }
    /** @brief Choose the simulator viewport field of view.
     * @param width Viewport columns in pixels.
     * @param height Viewport rows in pixels.
     * @return Field of view in degrees.
     * @details The Pi optics path accounts for viewport shape and capture inset;
     * the generic path uses the height-adjusted configuration. */
    double viewportFov(int width, int height) const {
        return piCamera() ? PiAiOptics::viewportFov(width, height, captureInset) : cameraFov();
    }
};

/** @brief Strip the ASCII whitespace allowed around route INI tokens.
 * @param text Original field or line.
 * @return Trimmed text, empty for an all-whitespace input. */
inline std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

/** @brief Read a complete finite numeric route field.
 * @param text Already trimmed value.
 * @param name Setting name for diagnostics.
 * @return Converted value.
 * @throws std::runtime_error If malformed, nonfinite, or only partly consumed. */
inline double number(const std::string& text, const std::string& name) {
    std::size_t used = 0;
    double value;
    try {
        value = std::stod(text, &used);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid number for " + name + ": " + text);
    }
    if (used != text.size() || !std::isfinite(value))
        throw std::runtime_error("Invalid number for " + name + ": " + text);
    return value;
}

/** @brief Enforce the inclusive policy bounds for a parsed setting.
 * @param value Already validated finite value.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @param name Setting name for diagnostics.
 * @throws std::runtime_error If outside the bounds. */
inline void range(double value, double minimum, double maximum, const std::string& name) {
    if (value < minimum || value > maximum)
        throw std::runtime_error(name + " must be between " + std::to_string(minimum) +
                                 " and " + std::to_string(maximum));
}

/** @brief Validate the syntax of an Ivy IPv4 bus endpoint.
 * @param address Numeric dotted quad followed by a port.
 * @throws std::runtime_error For malformed octets, port or delimiters.
 * @details Checks syntax, not whether the endpoint is reachable or broadcast-capable. */
inline void validateIvyAddress(const std::string& address) {
    std::istringstream input(address);
    int octet[4]{}, port = 0;
    char separator[4]{};
    if (!(input >> octet[0] >> separator[0] >> octet[1] >> separator[1] >>
          octet[2] >> separator[2] >> octet[3] >> separator[3] >> port) ||
        separator[0] != '.' || separator[1] != '.' || separator[2] != '.' ||
        separator[3] != ':' || input.peek() != std::char_traits<char>::eof() ||
        address.find_first_not_of("0123456789.:") != std::string::npos ||
        port < 1 || port > 65535 ||
        std::any_of(std::begin(octet), std::end(octet), [](int value) { return value < 0 || value > 255; }))
        throw std::runtime_error("ivy.bus_address must be a full IPv4 broadcast/multicast address:port (1-65535)");
}

/** @brief Compute the shortest signed longitude change across the antimeridian.
 * @param start Starting longitude in degrees.
 * @param end Ending longitude in degrees.
 * @return Wrapped delta in [-180, 180] degrees. */
inline double longitudeDelta(double start, double end) {
    return std::remainder(end - start, 360.0);
}

/** @brief Estimate surface distance between two route points.
 * @param a First geographic position.
 * @param b Second geographic position.
 * @return Spherical great-circle distance in metres, ignoring altitude.
 * @details Use haversine with a wrapped longitude delta. Clamp its argument
 * so roundoff near antipodal points cannot invalidate the inverse sine. */
inline double distance(const Point& a, const Point& b) {
    const double lat1 = a.lat * pi / 180;
    const double lat2 = b.lat * pi / 180;
    const double dlat = lat2 - lat1;
    const double dlon = longitudeDelta(a.lon, b.lon) * pi / 180;
    const double h = std::pow(std::sin(dlat / 2), 2) +
                     std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2), 2);
    return 2 * earthRadius * std::asin(std::sqrt(std::clamp(h, 0.0, 1.0)));
}

/**
 * @brief Parse and validate a route INI stream relative to a base directory.
 * @param[in] input Readable INI stream.
 * @param[in] base Directory used to resolve the output path.
 * @return Validated route and camera configuration.
 * @throws std::runtime_error If keys, units, bounds, or option combinations are invalid.
 */
inline Config parseConfig(std::istream& input, const std::filesystem::path& base) {
    std::map<std::string, std::string> values;
    std::string section, line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        line = trim(line.substr(0, line.find_first_of(";#")));
        if (line.empty()) continue;
        if (line.front() == '[' && line.back() == ']') {
            section = trim(line.substr(1, line.size() - 2));
            if (section != "start" && section != "end" && section != "camera" &&
                section != "ivy" && section != "simulation")
                throw std::runtime_error("Unknown INI section on line " + std::to_string(lineNumber));
            continue;
        }
        const auto equal = line.find('=');
        if (section.empty() || equal == std::string::npos)
            throw std::runtime_error("Expected section/key=value on INI line " +
                                     std::to_string(lineNumber));
        const std::string key = section + "." + trim(line.substr(0, equal));
        const std::string value = trim(line.substr(equal + 1));
        if (value.empty() || !values.emplace(key, value).second)
            throw std::runtime_error("Empty or duplicate INI key: " + key);
    }
    if (input.bad()) throw std::runtime_error("Failed while reading INI file");
    const auto required = [&](const std::string& key) {
        const auto found = values.find(key);
        if (found == values.end()) throw std::runtime_error("Missing INI key: " + key);
        return number(found->second, key);
    };
    const auto optional = [&](const std::string& key, double fallback) {
        const auto found = values.find(key);
        return found == values.end() ? fallback : number(found->second, key);
    };
    const auto boolean = [&](const std::string& key, bool fallback) {
        const double value = optional(key, fallback ? 1 : 0);
        if (value != 0 && value != 1) throw std::runtime_error(key + " must be 0 or 1");
        return value == 1;
    };
    const std::string allowed[] = {
        "start.lat", "start.lon", "start.agl", "end.lat", "end.lon", "end.agl",
        "camera.speed_mps", "camera.photo_interval_seconds", "camera.settle_seconds",
        "camera.fov_degrees", "camera.jpeg_quality", "camera.output_directory", "camera.startup_flight",
        "camera.lock_start_altitude", "camera.jpeg_width", "camera.jpeg_height",
        "camera.grab", "camera.tile_settle_seconds", "camera.tile_overlap", "camera.map_camera_agl_m",
        "camera.capture_inset_pixels", "camera.lens_profile",
        "camera.shutter_speed_seconds", "camera.iso", "camera.hardware_identifier",
        "camera.capture_margin",
        "camera.lens_k1", "camera.lens_k2", "camera.lens_blue_scale", "camera.lens_red_scale",
        "camera.auto_enter_flight", "camera.startup_timeout_seconds", "camera.startup_airport",
        "camera.startup_maximize_minimized",
        "ivy.enabled", "ivy.bus_address", "ivy.aircraft_id", "simulation.utc_datetime",
        "simulation.weather_preset"
    };
    for (const auto& entry : values)
        if (std::find(std::begin(allowed), std::end(allowed), entry.first) == std::end(allowed))
            throw std::runtime_error("Unknown INI key: " + entry.first);

    Config config;
    config.start = {required("start.lat"), required("start.lon"), required("start.agl")};
    config.end = {required("end.lat"), required("end.lon"), required("end.agl")};
    for (const auto& point : {config.start, config.end}) {
        range(point.lat, -85, 85, "latitude");
        range(point.lon, -180, 180, "longitude");
        range(point.agl, 1, 1000, "AGL in meters");
    }
    config.speed = optional("camera.speed_mps", config.speed);
    config.interval = optional("camera.photo_interval_seconds", config.interval);
    config.settle = optional("camera.settle_seconds", config.settle);
    config.fov = optional("camera.fov_degrees", config.fov);
    const auto lens = values.find("camera.lens_profile");
    if (lens != values.end()) config.lensProfile = lens->second;
    if (config.lensProfile != "simconnect" && !config.piCamera())
        throw std::runtime_error("camera.lens_profile must be simconnect or raspberry_pi_ai");
    if (config.piCamera() && values.count("camera.fov_degrees"))
        throw std::runtime_error("Pi AI lens fixes the optical FOV; omit camera.fov_degrees");
    config.exifSettings.shutterSeconds = optional("camera.shutter_speed_seconds",
        config.piCamera() ? PiAiOptics::clearDayShutterSeconds : config.exifSettings.shutterSeconds);
    const double iso = optional("camera.iso", config.exifSettings.iso);
    range(iso, 1, 65535, "camera.iso");
    if (std::floor(iso) != iso)
        throw std::runtime_error("camera.iso must be an integer");
    config.exifSettings.iso = static_cast<std::uint16_t>(iso);
    const auto hardware = values.find("camera.hardware_identifier");
    if (hardware != values.end()) config.exifSettings.hardwareIdentifier = hardware->second;
    config.exifSettings.validate();
    config.lens.k1 = optional("camera.lens_k1", config.lens.k1);
    config.lens.k2 = optional("camera.lens_k2", config.lens.k2);
    config.lens.blueScale = optional("camera.lens_blue_scale", config.lens.blueScale);
    config.lens.redScale = optional("camera.lens_red_scale", config.lens.redScale);
    range(config.lens.k1, -.05, 0, "camera.lens_k1");
    range(config.lens.k2, 0, .01, "camera.lens_k2");
    range(config.lens.blueScale, .999, 1.001, "camera.lens_blue_scale");
    range(config.lens.redScale, .999, 1.001, "camera.lens_red_scale");
    if (!config.piCamera() && (values.count("camera.lens_k1") || values.count("camera.lens_k2") ||
        values.count("camera.lens_blue_scale") || values.count("camera.lens_red_scale")))
        throw std::runtime_error("Lens effect overrides require lens_profile=raspberry_pi_ai");
    const double grab = optional("camera.grab", config.grab);
    if (grab != 2 && grab != 3) throw std::runtime_error("camera.grab must be 2 (line flight) or 3 (box tiles)");
    config.grab = static_cast<int>(grab);
    config.tileSettle = optional("camera.tile_settle_seconds", config.tileSettle);
    config.tileOverlap = optional("camera.tile_overlap", config.tileOverlap);
    config.captureMargin = boolean("camera.capture_margin", false);
    config.mapCameraAgl = optional("camera.map_camera_agl_m", 0);
    const double inset = optional("camera.capture_inset_pixels", 0);
    range(inset, 0, 256, "camera.capture_inset_pixels");
    if (std::floor(inset) != inset)
        throw std::runtime_error("camera.capture_inset_pixels must be an integer");
    config.captureInset = static_cast<int>(inset);
    if (config.mapCameraAgl != 0)
        range(config.mapCameraAgl, 1000, 100000, "camera.map_camera_agl_m");
    range(config.tileSettle, 0.05, 60, "camera.tile_settle_seconds");
    range(config.tileOverlap, 0, 0.9, "camera.tile_overlap");
    config.lockStartAltitude = boolean("camera.lock_start_altitude", config.lockStartAltitude);
    if (config.mapCameraAgl != 0 && (config.grab != 3 || !config.lockStartAltitude))
        throw std::runtime_error("map_camera_agl_m requires grab=3 and lock_start_altitude=1");
    config.autoEnterFlight = boolean("camera.auto_enter_flight", config.autoEnterFlight);
    config.startupMaximizeMinimized = boolean("camera.startup_maximize_minimized",
                                             config.startupMaximizeMinimized);
    config.startupTimeout = optional("camera.startup_timeout_seconds", config.startupTimeout);
    range(config.startupTimeout, 0, 86400, "camera.startup_timeout_seconds");
    const auto airport = values.find("camera.startup_airport");
    if (airport != values.end()) config.startupAirport = airport->second;
    if (config.startupAirport.size() != 4 ||
        !std::all_of(config.startupAirport.begin(), config.startupAirport.end(), [](unsigned char value) {
            return (value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9');
        }))
        throw std::runtime_error("camera.startup_airport must be a four-character uppercase ICAO airport code");
    config.ivyEnabled = boolean("ivy.enabled", config.ivyEnabled);
    const auto ivyAddress = values.find("ivy.bus_address");
    if (ivyAddress != values.end()) config.ivyAddress = ivyAddress->second;
    validateIvyAddress(config.ivyAddress);
    const double aircraftId = optional("ivy.aircraft_id", config.ivyAircraftId);
    range(aircraftId, -1, 255, "ivy.aircraft_id");
    if (std::floor(aircraftId) != aircraftId)
        throw std::runtime_error("ivy.aircraft_id must be an integer (-1 = any)");
    config.ivyAircraftId = static_cast<int>(aircraftId);
    const auto utc = values.find("simulation.utc_datetime");
    if (utc != values.end() && utc->second != "unchanged") {
        try {
            config.startupUtc = parseUtcDateTime(utc->second);
        } catch (const std::invalid_argument& error) {
            throw std::runtime_error(std::string("simulation.utc_datetime: ") + error.what());
        }
    }
    const auto weather = values.find("simulation.weather_preset");
    if (weather != values.end()) config.weatherPreset = weather->second;
    if (config.weatherPreset != "unchanged" && config.weatherPreset != "clear_sky")
        throw std::runtime_error("simulation.weather_preset must be unchanged or clear_sky");
    const double quality = optional("camera.jpeg_quality", config.quality);
    const double jpegWidth = optional("camera.jpeg_width", config.jpegWidth);
    const double jpegHeight = optional("camera.jpeg_height", config.jpegHeight);
    range(jpegWidth, 1, 16384, "camera.jpeg_width");
    range(jpegHeight, 1, 16384, "camera.jpeg_height");
    if (std::floor(jpegWidth) != jpegWidth || std::floor(jpegHeight) != jpegHeight)
        throw std::runtime_error("camera.jpeg_width and camera.jpeg_height must be integer pixel counts");
    config.jpegWidth = static_cast<int>(jpegWidth);
    config.jpegHeight = static_cast<int>(jpegHeight);
    validateJpegSize(config.jpegWidth, config.jpegHeight);
    if (config.piCamera() && (config.grab != 3 || config.mapCameraAgl != 0 ||
        config.jpegWidth != PiAiOptics::width || config.jpegHeight != PiAiOptics::height))
        throw std::runtime_error("Pi AI lens requires grab=3, ordinary camera height and 2028x1520 JPEGs");
    range(config.speed, 0.1, 200, "speed_mps");
    if (config.interval != -1 && config.interval <= 0)
        throw std::runtime_error("photo_interval_seconds must be -1 (disabled) or a positive number of seconds");
    range(config.settle, 1, 300, "settle_seconds");
    range(config.fov, 10, 120, "fov_degrees");
    if (config.mapCameraAgl != 0 && config.cameraFov() < 1.05)
        throw std::runtime_error("Mapping camera needs at least 1.05 degrees FOV; reduce map_camera_agl_m "
                                 "or increase start AGL/fov_degrees (MSFS clamps narrower lenses)");
    range(quality, 1, 100, "jpeg_quality");
    if (std::floor(quality) != quality) throw std::runtime_error("jpeg_quality must be an integer");
    config.quality = static_cast<unsigned long>(quality);
    const auto output = values.find("camera.output_directory");
    config.iniDirectory = base;
    const auto startup = values.find("camera.startup_flight");
    if (startup != values.end()) config.startupFlight = startup->second;
    config.output = base / (output == values.end() ? std::filesystem::path(config.piCamera() ? L"pictures" : L"mytiles") :
                           std::filesystem::u8path(output->second));
    const double length = distance(config.start, config.end);
    range(length, 1, 100000, "route length in meters");
    if (config.grab == 3 && (config.start.lat == config.end.lat ||
                            longitudeDelta(config.start.lon, config.end.lon) == 0))
        throw std::runtime_error("grab=3 needs opposite corners of a nonzero-width, nonzero-height bounding box");
    return config;
}

/**
 * @brief Open and parse a route configuration file.
 * @param[in] path INI file; output paths are resolved relative to its directory.
 * @return Validated capture configuration.
 * @throws std::runtime_error If the file cannot be read or its settings are invalid.
 */
inline Config loadConfig(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open INI file: " + path.u8string());
    return parseConfig(input, std::filesystem::absolute(path).parent_path());
}

/**
 * @brief Apply the route's chosen terrain-following or fixed-start altitude policy.
 * @details A fixed altitude is latched only after the initial camera has settled;
 * before then the provisional value follows current terrain plus configured AGL.
 */
class CameraAltitude {
    bool lockStart_;
    double startAgl_;
    std::optional<double> lockedMsl_;
public:
    /** @brief Initialize altitude policy from a validated route configuration. */
    explicit CameraAltitude(const Config& config)
        : lockStart_(config.lockStartAltitude), startAgl_(config.cameraAgl(config.start.agl)) {}

    /**
     * @brief Compute camera AMSL altitude for one terrain sample.
     * @param[in] ground Ground elevation AMSL in metres.
     * @param[in] routeAgl Requested route height above ground in metres.
     * @param[in] settled Whether the initial settled position may be latched.
     * @return Camera altitude AMSL in metres.
     * @throws std::runtime_error If either input height is nonfinite.
     */
    double at(double ground, double routeAgl, bool settled) {
        if (!std::isfinite(ground) || !std::isfinite(routeAgl))
            throw std::runtime_error("Nonfinite camera terrain/AGL height");
        if (!lockStart_) return ground + routeAgl;
        if (!lockedMsl_ && settled) lockedMsl_ = ground + startAgl_;
        return lockedMsl_.value_or(ground + startAgl_);
    }
    /** @brief Report whether fixed-start AMSL has been latched. */
    bool locked() const { return lockedMsl_.has_value(); }
};

/**
 * @brief Evaluate a back-and-forth route at elapsed time.
 * @details Geographic interpolation uses the shortest longitude delta; it does
 * not model aircraft dynamics or establish that a real aircraft flew the path.
 */
class Route {
    Config config_;
    double length_;
public:
    /** @brief Construct a route and cache its great-circle endpoint distance. */
    explicit Route(const Config& config) : config_(config), length_(distance(config.start, config.end)) {}
    /** @brief Return route length in metres. */
    double length() const { return length_; }
    /** @brief Return one-way travel duration at the configured speed. */
    double legSeconds() const { return length_ / config_.speed; }
    /**
     * @brief Return the interpolated endpoint-to-endpoint route position.
     * @param[in] seconds Nonnegative elapsed seconds; every second leg reverses direction.
     * @return Geographic position and interpolated AGL.
     * @throws std::runtime_error If elapsed time is negative or nonfinite.
     */
    Point at(double seconds) const {
        if (!std::isfinite(seconds) || seconds < 0)
            throw std::runtime_error("Route time must be finite and nonnegative");
        const double phase = std::fmod(seconds / legSeconds(), 2.0);
        const double t = phase <= 1 ? phase : 2 - phase;
        return {
            config_.start.lat + t * (config_.end.lat - config_.start.lat),
            std::remainder(config_.start.lon + t * longitudeDelta(config_.start.lon, config_.end.lon), 360.0),
            config_.start.agl + t * (config_.end.agl - config_.start.agl)
        };
    }
    /** @brief Interpolate route AGL from distance along the endpoint segment. */
    double aglAt(const Point& point) const {
        const double t = std::clamp(distance(config_.start, point) / length_, 0.0, 1.0);
        return config_.start.agl + t * (config_.end.agl - config_.start.agl);
    }
};
}
#endif
