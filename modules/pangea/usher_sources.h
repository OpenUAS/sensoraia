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
 * @file usher_sources.h
 * @brief Convert validated USHER AI Camera evidence into GREAT source records.
 * @details Inputs must be real bounded JPEGs with complete EXIF, profile-matched
 * dimensions, and qualified pose evidence. Synthetic map imagery and nominal
 * lens-only geometry are rejected rather than promoted into survey sources.
 */
#ifndef PANGEA_USHER_SOURCES_H
#define PANGEA_USHER_SOURCES_H

extern "C" {
#include "camera_profile.h"
}
#include "sensor_capture.h"
#include <charconv>
#include <sys/stat.h>

namespace pangea_survey {
/**
 * @brief Read a photo snapshot and verify each source against profile/evidence.
 * @param[in] config GREAT session settings, measured camera profile and source directory.
 * @param[in] manifest Bounded `USHER_AICAM_V1` filename snapshot.
 * @return Fully validated sources with camera geometry and capture provenance.
 * @throws std::runtime_error If any JPEG, EXIF field, profile, frame identity,
 * pose qualification, or shared-coverage check fails.
 * @details Exposure-aligned pose is required unless configuration explicitly
 * admits the recorded-pose approximation policy.
 */
inline std::vector<OrthoSource> readUsherSources(const MapConfig& config, const std::filesystem::path& manifest) {
    usher_camera_profile profile{};
    if (!usher_camera_profile_read(config.cameraProfile.c_str(), &profile) || profile.camera_id != 2)
        throw std::runtime_error("GREAT requires an AI Camera profile with filename_prefix=a");
    const auto& calibration = profile.calibration;
    std::ifstream input(manifest);
    std::string name;
    if (!std::getline(input, name) || name != "USHER_AICAM_V1")
        throw std::runtime_error("AI Camera input requires a GREAT session photo snapshot");
    std::vector<OrthoSource> result;
    std::map<std::string, bool> names;
    while (std::getline(input, name)) {
        if (result.size() == 4096 || name.empty() || name.size() > 160 || name.front() != 'a' ||
            name.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-") != std::string::npos ||
            std::filesystem::path(name).extension() != ".jpg" || !names.emplace(name, true).second)
            throw std::runtime_error("Invalid or duplicate AI Camera photo basename");
        OrthoSource source;
        source.path = (config.sourceDirectory.empty() ? config.photoDirectory : config.sourceDirectory) / name;
        struct stat status{};
        if (lstat(source.path.c_str(), &status) || !S_ISREG(status.st_mode) || status.st_size <= 0 ||
            status.st_size > 128 * 1024 * 1024)
            throw std::runtime_error("AI Camera input must be a bounded regular JPEG, not a symlink");
        std::array<unsigned char, EXIF_MAX_PAYLOAD> payload{};
        std::size_t size = 0;
        exif_view view{};
        if (exif_read_jpeg(source.path.c_str(), payload.data(), payload.size(), &size) || exif_parse(&view, payload.data(), size))
            throw std::runtime_error("AI Camera JPEG has no valid complete EXIF: " + name);
        const auto text = [&](exif_directory directory, uint16_t tag) {
            exif_value field{};
            if (!exif_get(&view, directory, tag, &field)) return std::string();
            return std::string(reinterpret_cast<const char*>(field.data), exif_text_length(&field));
        };
        const auto comment = text(EXIF_CAMERA, EXIF_USER_COMMENT);
        if (comment.find("USHER_MOCK_MAP_V1") != std::string::npos || comment.find("USHER_LENS_V1") != std::string::npos)
            throw std::runtime_error("Synthetic or lens-derived photos are not raw AI Camera inputs");
        const auto description = text(EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION);
        if (description.rfind("Paparazzi USHER ", 0) != 0)
            throw std::runtime_error("AI Camera input lacks USHER provenance");
        std::map<std::string, std::string> fields;
        std::istringstream parts(description);
        std::string part;
        std::getline(parts, part, ';');
        while (std::getline(parts, part, ';')) {
            const auto separator = part.find('=');
            if (separator == std::string::npos) throw std::runtime_error("Invalid USHER EXIF field");
            const auto trim = [](std::string value) {
                const auto first = value.find_first_not_of(" \t\r\n");
                if (first == std::string::npos) return std::string();
                return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
            };
            if (!fields.emplace(trim(part.substr(0, separator)), trim(part.substr(separator + 1))).second)
                throw std::runtime_error("Duplicate USHER EXIF field");
        }
        const auto required = [&](const char* key) -> const std::string& {
            const auto found = fields.find(key);
            if (found == fields.end() || found->second.empty()) throw std::runtime_error(std::string("USHER EXIF lacks ") + key);
            return found->second;
        };
        const auto real = [&](const char* key, double low, double high) {
            const double value = number(required(key), key);
            range(value, low, high, key);
            return value;
        };
        const auto integer = [&](const char* key) {
            const auto& value = required(key);
            uint64_t parsed = 0;
            const auto converted = std::from_chars(value.data(), value.data() + value.size(), parsed);
            if (converted.ec != std::errc() || converted.ptr != value.data() + value.size())
                throw std::runtime_error(std::string("Invalid USHER integer ") + key);
            return parsed;
        };
        bool exposurePose = true;
        for (const auto* key : {"capture_pose_synchronized", "observation_pose_aligned", "frame_identity_bound"}) {
            const auto& value = required(key);
            if (value != "true" && value != "false") throw std::runtime_error("Invalid USHER pose qualification");
            exposurePose = exposurePose && value == "true";
        }
        if (required("altitude_datum") != "flight_controller_navigation")
            throw std::runtime_error("AI Camera mapping requires FC navigation altitude");
        if (!exposurePose && !config.allowRecordedPose)
            throw std::runtime_error("AI Camera mapping requires exposure-aligned, frame-bound pose; recorded policy is explicitly approximate");
        if (exposurePose) {
            if (required("observation_time_kind") != "sensor_exposure_interval")
                throw std::runtime_error("AI Camera mapping requires sensor exposure timing");
            const auto earliest = integer("observation_earliest_monotonic_us"), latest = integer("observation_latest_monotonic_us");
            if (!earliest || latest < earliest || latest - earliest > 50000 || latest > uint64_t(LLONG_MAX / 10))
                throw std::runtime_error("AI Camera observation interval exceeds 50 ms or is invalid");
            sensor_capture capture{};
            exif_value evidence{};
            if (!exif_get(&view, EXIF_CAMERA, SENSOR_CAPTURE_TAG, &evidence) ||
                !sensor_capture_decode(&capture, evidence.data, evidence.size) ||
                !capture.sequence || !capture.sensor_timestamp_ns || capture.sequence != integer("observation_sequence") ||
                capture.sensor_timestamp_ns != integer("sensor_timestamp_ns") || capture.boot_id != required("observation_boot_id") ||
                capture.width != calibration.width || capture.height != calibration.height ||
                capture.monotonic_us < earliest || capture.monotonic_us > latest)
                throw std::runtime_error("AI Camera frame identity/dimensions disagree with its observation pose");
            source.poseTime = static_cast<long long>(earliest * 10); source.frameTime = static_cast<long long>(latest * 10);
        }
        FILE* jpeg = fopen(source.path.c_str(), "rb");
        uint32_t width = 0, height = 0;
        unsigned components = 0;
        const bool dimensions = jpeg && !exif_jpeg_dimensions(jpeg, &width, &height, &components);
        const int closed = jpeg ? fclose(jpeg) : -1;
        if (!dimensions || closed || width != calibration.width || height != calibration.height)
            throw std::runtime_error("AI Camera JPEG dimensions disagree with its profile");
        exif_value orientation{};
        uint32_t upright = 1;
        if (exif_get(&view, EXIF_PRIMARY, EXIF_ORIENTATION, &orientation) &&
            (!exif_value_unsigned(&orientation, 0, &upright) || upright != 1))
            throw std::runtime_error("AI Camera input must be upright sensor pixels");
        source.latitude = real(exposurePose ? "observation_lat_deg" : "lat_deg", -80, 84);
        source.longitude = real(exposurePose ? "observation_lon_deg" : "lon_deg", -180, 180);
        source.altitude = real(exposurePose ? "observation_alt_m" : "alt_m", -1000, 20000);
        source.bank = real(exposurePose ? "observation_roll_deg" : "roll_deg", -180, 180);
        source.pitch = real(exposurePose ? "observation_pitch_deg" : "pitch_deg", -90, 90);
        source.heading = real(exposurePose ? "observation_yaw_deg" : "yaw_deg", -360, 360);
        source.poseUncertainty = exposurePose ? real("observation_pose_uncertainty_m", 0, config.maxPoseUncertainty) :
                               std::numeric_limits<double>::quiet_NaN();
        source.width = int(calibration.width); source.height = int(calibration.height);
        source.focal = calibration.fx; source.focalY = calibration.fy;
        source.cx = calibration.cx; source.cy = calibration.cy;
        source.referencePlane = config.mosaicReferencePlane;
        source.distorted = true;
        source.lens = {PiAiOptics::k1, PiAiOptics::k2, PiAiOptics::blueScale, PiAiOptics::redScale};
        coverage_pose pose{};
        pose.latitude_deg = source.latitude; pose.longitude_deg = source.longitude;
        pose.agl_m = source.altitude - source.referencePlane;
        pose.roll_deg = source.bank; pose.pitch_deg = source.pitch; pose.yaw_deg = source.heading;
        pose.uncertainty_m = exposurePose ? source.poseUncertainty : 0;
        coverage_footprint footprint{};
        if (!coverage_project(&calibration, &pose, source.latitude, source.longitude, &footprint))
            throw std::runtime_error("AI Camera calibration/pose fails shared coverage geometry validation");
        source.camera.emplace(source.focal, source.focalY, source.cx, source.cy, source.bank, source.pitch, source.heading,
                              calibration.mount_roll_deg, calibration.mount_pitch_deg, calibration.mount_yaw_deg);
        for (const double column : {0.0, double(source.width - 1)}) for (const double row : {0.0, double(source.height - 1)})
            source.camera->ground(column, row, source.altitude - source.referencePlane);
        result.push_back(std::move(source));
    }
    if (!input.eof() || result.empty()) throw std::runtime_error("Empty or unreadable AI Camera snapshot");
    return result;
}
}
#endif