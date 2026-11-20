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
 * @file photo_exif.h
 * @brief Encode simulated-camera pose, optics, and uncertainty into EXIF.
 * @details Metadata distinguishes commanded pose from observed simulator pose,
 * and labels nominal/emulated camera settings rather than presenting them as
 * physical measurements. This is important because a complete EXIF record is
 * provenance, not proof of exposure synchronization or surveyed accuracy.
 */
#ifndef PANGEA_PHOTO_EXIF_H
#define PANGEA_PHOTO_EXIF_H

#include "exifite.h"
#include "map_projection.h"

#include "camera_optics.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace pangea_survey {
/** @brief Photo location with AMSL camera and ground heights in metres. */
struct PhotoPosition {
    double latitude, longitude, altitudeAmsl, groundAmsl;
    /** @brief Compute height above ground from the shared AMSL datum.
     * @return Camera-minus-ground height in metres. */
    double agl() const { return altitudeAmsl - groundAmsl; }
};

/**
 * @brief Camera and timing provenance required for a simulated-photo EXIF record.
 * @details Dimensions are pixels, timestamps are 100 ns monotonic counts, angles
 * are degrees except `fovRadians`, and the optional map records projected metres.
 */
struct PhotoCamera {
    double fovRadians = 0;
    double commandedHeading = 0;
    double pitch = 0, bank = 0, heading = 0;
    double observedLatitude = 0, observedLongitude = 0, observedAltitude = 0;
    long long observed100ns = 0;
    int sourceWidth = 0, sourceHeight = 0;
    double cropX = 0, cropY = 0, cropWidth = 0, cropHeight = 0;
    bool narrowAngle = false;
    bool piAiCamera = false;
    LensEffects lens;
    CameraExifSettings exifSettings;
    bool fixedStartAltitude = false;
    double startGroundMsl = 0;
    long long frame100ns = 0;
    std::optional<UtcDateTime> captureUtc;
    double utcAbsoluteSeconds = 0;
    long long utcRead100ns = 0;
    std::optional<MapTile> map;
    double imageRotationDegrees = 0, rotationBoundDegrees = 0;
    std::optional<unsigned long long> photoNumber;
};

/**
 * @brief Build bounded EXIF payloads for simulator-generated source photos.
 * @details The output buffer is owned by this object and is replaced by each
 * call to @ref build. Callers must consume or copy the payload before building
 * another record with the same instance.
 */
class PhotoExif {
protected:
    exif_document document_{};
    std::array<unsigned char, EXIF_MAX_PAYLOAD> payload_{};

    /** @brief Convert a C EXIF status into the mapper's error path.
     * @param result Zero on success, nonzero on failure.
     * @param operation Context for the error.
     * @throws std::runtime_error With the current errno diagnostic on failure. */
    static void check(int result, const char* operation) {
        if (result != 0)
            throw std::runtime_error(std::string(operation) + ": " +
                                     std::generic_category().message(errno));
    }
    /** @brief Allocate one field in the bounded EXIF document.
     * @param directory Target EXIF directory.
     * @param tag Numeric field identifier.
     * @param format Element encoding.
     * @param count Number of elements, not the byte count.
     * @return Borrowed writable field storage.
     * @throws std::runtime_error If the document cannot reserve the field. */
    unsigned char* reserve(exif_directory directory, std::uint16_t tag,
                           exif_format format, std::uint32_t count) {
        auto* value = exif_document_reserve(&document_, directory, tag, format, count);
        if (!value) check(-1, "Reserve photo EXIF field");
        return value;
    }
    /** @brief Encode a validated geographic coordinate as EXIF GPS rationals.
     * @param degrees Signed decimal degrees.
     * @param referenceTag Hemisphere tag ID.
     * @param valueTag Degrees/minutes/seconds tag ID.
     * @param positive Hemisphere for a nonnegative coordinate.
     * @param negative Hemisphere for a negative coordinate.
     * @throws std::runtime_error If either tag cannot be stored.
     * @details Round the total once so overflow from the fractional second
     * carries naturally into minutes and degrees. */
    void coordinate(double degrees, std::uint16_t referenceTag, std::uint16_t valueTag,
                    char positive, char negative) {
        const char reference[] = {degrees < 0 ? negative : positive, '\0'};
        check(exif_document_ascii(&document_, EXIF_GPS, referenceTag, reference), "Set EXIF GPS hemisphere");
        // Round once in integer microseconds, carrying 60-second/minute boundaries.
        const auto total = static_cast<std::uint64_t>(std::llround(std::abs(degrees) * 3600 * 1000000));
        auto* value = reserve(EXIF_GPS, valueTag, EXIF_RATIONAL, 3);
        exif_encode_long(value, static_cast<std::uint32_t>(total / 3600000000ULL));
        exif_encode_long(value + 4, 1);
        exif_encode_long(value + 8, static_cast<std::uint32_t>(total / 60000000ULL % 60));
        exif_encode_long(value + 12, 1);
        exif_encode_long(value + 16, static_cast<std::uint32_t>(total % 60000000ULL));
        exif_encode_long(value + 20, 1000000);
    }
    /** @brief Write simulator-observed UTC into consistent EXIF and GPS fields.
     * @param camera Capture metadata with its optional UTC field engaged.
     * @throws std::runtime_error If the time sample is unqualified or a tag cannot be written.
     * @details Check frame/readback timing before publishing a wall-clock label.
     * Derive both tag families from the same UTC sample. */
    void captureTime(const PhotoCamera& camera) {
        const auto& utc = *camera.captureUtc;
        if (!captureUtcSampleValid(utc, camera.utcAbsoluteSeconds, camera.utcRead100ns, camera.frame100ns))
            throw std::runtime_error("Invalid verified capture UTC metadata");
        const auto date = utc.text.substr(0, 10);
        std::string exifDate = date;
        std::replace(exifDate.begin(), exifDate.end(), '-', ':');
        const auto timestamp = exifDate + " " + utc.text.substr(11, 5) + ":00";
        const auto milliseconds = static_cast<std::uint32_t>(
            std::floor((camera.utcAbsoluteSeconds - utc.absoluteSeconds()) * 1000));
        std::ostringstream subsecond;
        subsecond.imbue(std::locale::classic());
        subsecond << std::setfill('0') << std::setw(3) << milliseconds;
        check(exif_document_ascii(&document_, EXIF_PRIMARY, 0x0132, timestamp.c_str()), "Set EXIF DateTime");
        for (const auto tag : {0x9003, 0x9004})
            check(exif_document_ascii(&document_, EXIF_CAMERA, static_cast<std::uint16_t>(tag), timestamp.c_str()),
                  "Set EXIF original/digitized time");
        for (const auto tag : {0x9010, 0x9011, 0x9012})
            check(exif_document_ascii(&document_, EXIF_CAMERA, static_cast<std::uint16_t>(tag), "+00:00"),
                  "Set EXIF UTC offset");
        for (const auto tag : {0x9290, 0x9291, 0x9292})
            check(exif_document_ascii(&document_, EXIF_CAMERA, static_cast<std::uint16_t>(tag), subsecond.str().c_str()),
                  "Set EXIF UTC subsecond");
        check(exif_document_ascii(&document_, EXIF_GPS, 0x001d, exifDate.c_str()), "Set GPS date");
        auto* time = reserve(EXIF_GPS, 0x0007, EXIF_RATIONAL, 3);
        exif_encode_long(time, static_cast<std::uint32_t>(utc.hour));
        exif_encode_long(time + 4, 1);
        exif_encode_long(time + 8, static_cast<std::uint32_t>(utc.minute));
        exif_encode_long(time + 12, 1);
        exif_encode_long(time + 16, milliseconds);
        exif_encode_long(time + 20, 1000);
    }
public:
    /** @brief Borrowed serialized EXIF bytes held in this object's workspace. */
    struct Payload {
        const unsigned char* data;
        std::size_t size;
    };
    /**
     * @brief Construct EXIF from validated photo position and optional camera evidence.
     * @param[in] position WGS-84 position with camera/ground AMSL heights in metres.
     * @param[in] width Encoded image width in pixels.
     * @param[in] height Encoded image height in pixels.
     * @param[in] camera Optional pose, optics, UTC-readback, and map provenance.
     * @return Serialized EXIF payload borrowed from this object.
     * @throws std::runtime_error If coordinates, optics, timestamps, or payload size
     * violate the supported metadata contract.
     */
    Payload build(const PhotoPosition& position, int width, int height,
                  const std::optional<PhotoCamera>& camera = {}) {
        if (!std::isfinite(position.latitude) || std::abs(position.latitude) > 90 ||
            !std::isfinite(position.longitude) || std::abs(position.longitude) > 180 ||
            !std::isfinite(position.altitudeAmsl) || !std::isfinite(position.groundAmsl) ||
            !std::isfinite(position.agl()) ||
            std::abs(position.altitudeAmsl) >
                static_cast<double>(std::numeric_limits<std::uint32_t>::max()) / 1000 ||
            width < 1 || height < 1 || width > 16384 || height > 16384)
            throw std::runtime_error("Invalid position, heights or dimensions for photo EXIF");
        exif_document_init(&document_);
        check(exif_document_ascii(&document_, EXIF_PRIMARY, EXIF_SOFTWARE, "Tessavia / pangea_survey"),
              "Set EXIF software");
        check(exif_document_ascii(&document_, EXIF_PRIMARY, 0x010f, "Tessavia"),
              "Set virtual camera manufacturer");
        check(exif_document_ascii(&document_, EXIF_PRIMARY, 0x0110, "MSFS 2024 add-on camera"),
              "Set virtual camera model");
        auto* version = reserve(EXIF_GPS, EXIF_GPS_VERSION, EXIF_BYTE, 4);
        version[0] = 2;
        version[1] = 3;
        coordinate(position.latitude, EXIF_GPS_LATITUDE_REF, EXIF_GPS_LATITUDE, 'N', 'S');
        coordinate(position.longitude, EXIF_GPS_LONGITUDE_REF, EXIF_GPS_LONGITUDE, 'E', 'W');
        constexpr std::uint16_t gpsMapDatum = 0x0012;
        check(exif_document_ascii(&document_, EXIF_GPS, gpsMapDatum, "WGS-84"), "Set EXIF GPS datum");
        *reserve(EXIF_GPS, EXIF_GPS_ALTITUDE_REF, EXIF_BYTE, 1) = position.altitudeAmsl < 0 ? 1 : 0;
        const auto altitudeMm = static_cast<std::uint32_t>(std::llround(std::abs(position.altitudeAmsl) * 1000));
        check(exif_document_rational(&document_, EXIF_GPS, EXIF_GPS_ALTITUDE, altitudeMm, 1000),
              "Set EXIF AMSL altitude");
        exif_encode_long(reserve(EXIF_CAMERA, EXIF_PIXEL_X_DIMENSION, EXIF_LONG, 1),
                         static_cast<std::uint32_t>(width));
        exif_encode_long(reserve(EXIF_CAMERA, EXIF_PIXEL_Y_DIMENSION, EXIF_LONG, 1),
                         static_cast<std::uint32_t>(height));

        std::ostringstream description, comment;
        description.imbue(std::locale::classic());
        comment.imbue(std::locale::classic());
        description << std::fixed << std::setprecision(6)
                    << "Pangea simulated camera; AGL=" << position.agl()
                    << (camera && camera->fixedStartAltitude ? " m (unverified estimate); AMSL=" : " m; AMSL=")
                    << position.altitudeAmsl << " m";
        comment << std::fixed << std::setprecision(10)
                << "source=pangea_survey/MSFS2024; pose="
                << (camera && camera->piAiCamera ? "simconnect_observed_camera" : "simconnect_commanded_camera")
                << "; datum=WGS-84; latitude_deg=" << position.latitude
                << "; longitude_deg=" << position.longitude << std::setprecision(6)
                << "; agl_m=" << position.agl() << "; amsl_m=" << position.altitudeAmsl
                << "; ground_amsl_m=" << position.groundAmsl;
        if (camera && (camera->piAiCamera || camera->fixedStartAltitude)) {
            if (!std::isfinite(camera->startGroundMsl))
                throw std::runtime_error("Invalid initial ground reference in camera metadata");
            comment << "; agl_verified=" << (camera->fixedStartAltitude ? "false" : "true")
                    << "; agl_basis=" << (camera->fixedStartAltitude ?
                        "unverified_cached_sdk_ground_estimate" : "current_terrain_reference");
            if (camera->fixedStartAltitude)
                comment << "; start_ground_amsl_m=" << camera->startGroundMsl
                        << "; reference_plane_height_m=" << position.altitudeAmsl - camera->startGroundMsl;
        }
        if (camera && camera->captureUtc) {
            captureTime(*camera);
            comment << "; capture_utc=" << camera->captureUtc->text
                    << "; utc_absolute_seconds=" << std::setprecision(17) << camera->utcAbsoluteSeconds
                    << "; utc_read_qpc_100ns=" << camera->utcRead100ns
                    << "; utc_hold_method=repeated_simconnect_clock_resets"
                    << "; utc_tolerance_seconds=1; utc_frame_synchronized=false"
                    << "; exif_time_basis=verified_simulator_utc_readback_not_exposure_clock"
                    << std::setprecision(6);
        }
        if (camera) {
            const auto& c = *camera;
            c.exifSettings.validate();
            const auto shutterUs = static_cast<std::uint32_t>(std::llround(c.exifSettings.shutterSeconds * 1000000));
            check(exif_document_rational(&document_, EXIF_CAMERA, EXIF_EXPOSURE_TIME, shutterUs, 1000000),
                  "Set nominal simulated exposure time");
            exif_encode_short(reserve(EXIF_CAMERA, EXIF_ISO_SPEED, EXIF_SHORT, 1), c.exifSettings.iso);
            if (!c.exifSettings.hardwareIdentifier.empty())
                check(exif_document_ascii(&document_, EXIF_CAMERA, EXIF_BODY_SERIAL_NUMBER,
                                          c.exifSettings.hardwareIdentifier.c_str()),
                      "Set INI camera hardware identifier");
            comment << "; camera.exposure_basis=nominal_emulation_not_measured"
                    << "; camera.shutter_speed_seconds=" << shutterUs / 1000000.0
                    << "; camera.iso=" << c.exifSettings.iso
                    << "; camera.hardware_identifier="
                    << (c.exifSettings.hardwareIdentifier.empty() ? "undefined" : c.exifSettings.hardwareIdentifier);
            if (!c.exifSettings.hardwareIdentifier.empty())
                comment << "; camera.hardware_identifier_basis=ini_setting";
            if (std::abs(position.groundAmsl) > static_cast<double>(std::numeric_limits<std::int32_t>::max()) / 256)
                throw std::runtime_error("Ground altitude exceeds Paparazzi EXIF fixed-point range");
            const double fields[] = {c.fovRadians, c.commandedHeading, c.pitch, c.bank, c.heading,
                c.observedLatitude, c.observedLongitude, c.observedAltitude,
                c.cropX, c.cropY, c.cropWidth, c.cropHeight, c.imageRotationDegrees, c.rotationBoundDegrees};
            if (!std::all_of(std::begin(fields), std::end(fields), [](double value) { return std::isfinite(value); }) ||
                c.fovRadians <= 0 || c.fovRadians >= 3.141592653589793 ||
                c.sourceWidth < 1 || c.sourceHeight < 1 || c.cropX < 0 || c.cropY < 0 ||
                c.cropWidth <= 0 || c.cropHeight <= 0 ||
                c.cropX + c.cropWidth > c.sourceWidth + 1e-6 ||
                c.cropY + c.cropHeight > c.sourceHeight + 1e-6 ||
                std::abs(c.observedLatitude) > 90 || std::abs(c.observedLongitude) > 180 ||
                c.observed100ns <= 0)
                throw std::runtime_error("Invalid camera lens/attitude metadata");
            const double scale = c.cropWidth / width;
            if (std::abs(c.cropHeight / height - scale) > 1e-6)
                throw std::runtime_error("Camera intrinsics require proportional image framing");
            const double radiansToDegrees = 180 / 3.14159265358979323846;
            const double sourceFocal = c.sourceHeight / (2 * std::tan(c.fovRadians / 2));
            const double focal = sourceFocal / scale;
            if (c.piAiCamera && (width != PiAiOptics::width || height != PiAiOptics::height ||
                std::abs(focal - PiAiOptics::focalPixels) > 0.01 || c.frame100ns <= 0))
                throw std::runtime_error("Pi AI camera frame does not match its nominal optical profile");
            const double equivalent35mm = 36 * focal / width;
            if (equivalent35mm > 65535)
                throw std::runtime_error("Virtual lens exceeds EXIF 35mm-equivalent range");
            check(exif_document_ascii(&document_, EXIF_PRIMARY, 0x0110, c.piAiCamera ?
                                      "Raspberry Pi AI Camera (nominal emulation)" : "MSFS 2024 add-on camera"),
                  "Set virtual camera model");
            check(exif_document_ascii(&document_, EXIF_CAMERA, EXIF_LENS_MODEL,
                                      c.piAiCamera ? "Stock 4.74 mm f/1.79 (estimated lens effects v1)" :
                                      "SimConnect rectilinear virtual lens"),
                  "Set virtual lens model");
            if (c.piAiCamera) {
                check(exif_document_ascii(&document_, EXIF_PRIMARY, 0x010f, "Raspberry Pi (emulated)"),
                      "Set simulated camera manufacturer");
                check(exif_document_rational(&document_, EXIF_CAMERA, EXIF_FOCAL_LENGTH, 474, 100),
                      "Set nominal physical lens focal length");
                check(exif_document_rational(&document_, EXIF_CAMERA, 0x829d, 179, 100),
                      "Set nominal lens aperture");
                auto* lens = reserve(EXIF_CAMERA, 0xa432, EXIF_RATIONAL, 4);
                for (int i = 0; i < 4; ++i) {
                    exif_encode_long(lens + i * 8, static_cast<std::uint32_t>(std::llround(
                        (i < 2 ? PiAiOptics::focalMm : PiAiOptics::aperture) * 100)));
                    exif_encode_long(lens + i * 8 + 4, 100);
                }
                check(exif_document_rational(&document_, EXIF_CAMERA, 0xa20e,
                    static_cast<std::uint32_t>(width * 100000), 62868), "Set nominal sensor X resolution");
                check(exif_document_rational(&document_, EXIF_CAMERA, 0xa20f,
                    static_cast<std::uint32_t>(height * 100000), 47120), "Set nominal sensor Y resolution");
                exif_encode_short(reserve(EXIF_CAMERA, 0xa210, EXIF_SHORT, 1), 3);
            }
            exif_encode_short(reserve(EXIF_CAMERA, 0xa405, EXIF_SHORT, 1),
                              static_cast<std::uint16_t>(std::llround(equivalent35mm)));
            check(exif_document_ascii(&document_, EXIF_GPS, EXIF_GPS_DIRECTION_REF, "T"),
                  "Set image direction reference");
            const double direction = std::fmod(std::fmod(c.commandedHeading + c.imageRotationDegrees, 360.0) + 360.0, 360.0);
            const double imageHeading = direction / radiansToDegrees;
            check(exif_document_rational(&document_, EXIF_GPS, EXIF_GPS_DIRECTION,
                  static_cast<std::uint32_t>(std::llround(direction * 1000000)), 1000000),
                  "Set image grid-up direction");
            check(exif_document_ascii(&document_, EXIF_GPS, EXIF_GPS_SPEED_REF, "K"), "Set GPS speed units");
            check(exif_document_rational(&document_, EXIF_GPS, EXIF_GPS_SPEED, 0, 1),
                  "Set stationary virtual capture speed");
            comment << "; camera.projection=perspective; camera.mode="
                    << (c.piAiCamera ? "raspberry_pi_ai_nominal" : (c.narrowAngle ? "narrow_angle_mapping" : "normal"))
                    << "; camera.lens_model=" << (c.piAiCamera ? "brown_conrady_estimated_v1" : "ideal_rectilinear")
                    << "; camera.physical_sensor_mm="
                    << (c.piAiCamera ? "6.2868,4.712" : "undefined")
                    << "; camera.fov_axis=vertical; camera.sdk_fov_deg=" << c.fovRadians * radiansToDegrees
                    << "; camera.image_fov_x_deg=" << 2 * std::atan(width / (2 * focal)) * radiansToDegrees
                    << "; camera.image_fov_y_deg=" << 2 * std::atan(height / (2 * focal)) * radiansToDegrees
                    << "; camera.fx_px=" << focal << "; camera.fy_px=" << focal
                    << "; camera.cx_px=" << (c.sourceWidth / 2.0 - c.cropX) / scale - 0.5
                    << "; camera.cy_px=" << (c.sourceHeight / 2.0 - c.cropY) / scale - 0.5
                    << "; camera.equivalent_35mm_focal_mm=" << equivalent35mm
                    << "; camera.source_width=" << c.sourceWidth << "; camera.source_height=" << c.sourceHeight
                    << "; camera.crop_x=" << c.cropX << "; camera.crop_y=" << c.cropY
                    << "; camera.crop_width=" << c.cropWidth << "; camera.crop_height=" << c.cropHeight
                    << "; camera.image_rotation_deg=" << c.imageRotationDegrees
                    << "; camera.framing_rotation_bound_deg=" << c.rotationBoundDegrees
                    << "; camera.intrinsics_basis=vertical_FOV_pinhole_model"
                    << "; camera.attitude_convention=SDK_degrees_positive_pitch_down"
                    << "; camera.commanded_pitch_deg=90.000000; camera.commanded_bank_deg=0.000000"
                    << "; camera.commanded_heading_deg=" << c.commandedHeading
                    << "; camera.commanded_image_to_enu_rotation="
                    << std::cos(imageHeading) << ',' << -std::sin(imageHeading) << ",0,"
                    << -std::sin(imageHeading) << ',' << -std::cos(imageHeading) << ",0,0,0,-1"
                    << "; camera.observed_pitch_deg=" << c.pitch << "; camera.observed_bank_deg=" << c.bank
                    << "; camera.observed_heading_deg=" << c.heading << std::setprecision(10)
                    << "; camera.observed_latitude_deg=" << c.observedLatitude
                    << "; camera.observed_longitude_deg=" << c.observedLongitude << std::setprecision(6)
                    << "; camera.observed_amsl_m=" << c.observedAltitude
                    << "; camera.pose_received_qpc_100ns=" << c.observed100ns
                    << "; camera.pose_frame_synchronized=false";
            if (c.piAiCamera)
                comment << "; camera.calibrated=false; camera.distortion_basis=engineering_estimate_not_measured"
                        << "; camera.distortion_coefficients=" << c.lens.k1 << ',' << c.lens.k2 << ",0,0,0"
                        << "; camera.distortion_coefficient_order=k1,k2,p1,p2,k3"
                        << "; camera.chromatic_aberration.model=lateral_channel_magnification"
                        << "; camera.chromatic_aberration.basis=engineering_estimate_not_measured"
                        << "; camera.chromatic_aberration.reference_channel=green"
                        << "; camera.chromatic_aberration.channel_focal_scales_bgr=" << c.lens.blueScale << ','
                        << PiAiOptics::greenScale << ',' << c.lens.redScale
                        << "; camera.chromatic_aberration.longitudinal_simulated=false"
                        << "; camera.render_overscan_factor=" << PiAiOptics::overscan
                        << "; camera.fov_basis=nominal_undistorted_intrinsics"
                        << "; camera.sensor_model=Sony IMX500; camera.sensor_mode=2028x1520_2x2_binned"
                        << "; camera.physical_focal_mm=4.74; camera.aperture_f_number=1.79"
                        << "; camera.render_frame_qpc_100ns=" << c.frame100ns;
            const auto yawBfp = std::llround(direction / radiansToDegrees * 4096);
            if (c.photoNumber && *c.photoNumber == 0)
                throw std::runtime_error("Photo number must be positive");
            description << "; ";
            if (c.photoNumber) description << "nr=" << *c.photoNumber << "; ";
            description << "lat_e7deg=" << std::llround(position.latitude * 10000000)
                        << "; lon_e7deg=" << std::llround(position.longitude * 10000000)
                        << "; alt_mm=" << std::llround(position.altitudeAmsl * 1000)
                        << "; phi_bfp=0; theta_bfp=0; psi_bfp=" << yawBfp
                        << "; vground_bfp=0; course_bfp=undefined; groundalt_bfp="
                        << std::llround(position.groundAmsl * 256)
                        << "; groundalt_verified=" << (c.fixedStartAltitude ? "false" : "true")
                        << "; lat_deg=" << std::setprecision(10) << position.latitude
                        << "; lon_deg=" << position.longitude << std::setprecision(6)
                        << "; alt_m=" << position.altitudeAmsl
                        << "; roll_deg=0; pitch_deg=0; yaw_deg=" << direction
                        << "; speed_m_s=0; course_deg=undefined; groundalt_m=" << position.groundAmsl
                        << (c.fixedStartAltitude ? " (unverified cached estimate)" : "")
                        << "; attitude_source=simulated_level_stationary_body; camera_pitch_deg=90";
            comment << "; autopilot.schema=pangea_fake_airframe_v1"
                    << "; autopilot.source=simulated_stationary_capture"
                    << "; autopilot.latitude_e7=" << std::llround(position.latitude * 10000000)
                    << "; autopilot.longitude_e7=" << std::llround(position.longitude * 10000000)
                    << "; autopilot.altitude_amsl_m=" << position.altitudeAmsl
                    << "; autopilot.altitude_agl_m=";
            if (c.fixedStartAltitude) comment << "undefined";
            else comment << position.agl();
            comment << "; autopilot.roll_deg=0; autopilot.pitch_deg=0; autopilot.yaw_deg=" << direction
                    << "; autopilot.vground_mps=0; autopilot.heading_deg=" << direction
                    << "; autopilot.course_deg=undefined; autopilot.course_defined=false"
                    << "; autopilot.attitude_convention=level_body_downward_camera_mount,true_north_clockwise"
                    << "; angle_bfp_scale=4096; speed_bfp_scale=524288; position_bfp_scale=256";
            if (c.map) {
                const auto& m = *c.map;
                if (!std::isfinite(m.east) || !std::isfinite(m.north) ||
                    !std::isfinite(m.pixelSize) || m.pixelSize <= 0 || !std::isfinite(m.heading) ||
                    !((m.epsg >= 32601 && m.epsg <= 32660) || (m.epsg >= 32701 && m.epsg <= 32760)))
                    throw std::runtime_error("Invalid map georeferencing metadata");
                comment << "; map.crs=EPSG:" << m.epsg
                        << "; map.model=narrow_angle_reference_plane_not_true_orthophoto"
                        << "; map.center_easting_m=" << m.east << "; map.center_northing_m=" << m.north
                        << "; map.pixel_size_m=" << m.pixelSize << "; map.grid_up_heading_deg=" << m.heading
                        << "; map.row_direction=north_to_south";
            }
        }
        const auto text = description.str() + "; " + comment.str();
        check(exif_document_ascii(&document_, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, text.c_str()),
              "Set EXIF photo description");
        const unsigned char prefix[] = {'A', 'S', 'C', 'I', 'I', 0, 0, 0};
        if (text.size() > EXIF_MAX_PAYLOAD - sizeof(prefix) - 1)
            throw std::runtime_error("Photo provenance exceeds EXIF capacity");
        auto* bytes = reserve(EXIF_CAMERA, EXIF_USER_COMMENT, EXIF_UNDEFINED,
                              static_cast<std::uint32_t>(8 + text.size() + 1));
        for (std::size_t offset = 0; offset < sizeof(prefix); ++offset) bytes[offset] = prefix[offset];
        for (std::size_t offset = 0; offset < text.size(); ++offset)
            bytes[sizeof(prefix) + offset] = static_cast<unsigned char>(text[offset]);
        bytes[sizeof(prefix) + text.size()] = 0;
        std::size_t size = 0;
        check(exif_serialize(&document_, payload_.data(), payload_.size(), &size), "Serialize photo EXIF");
        return {payload_.data(), size};
    }
    /**
     * @brief Escape a string as a JSON string literal.
     * @param[in] text UTF-8 byte string without control characters.
     * @return Quoted JSON string value.
     * @throws std::runtime_error If the input contains a control character.
     */
    static std::string jsonString(const std::string& text) {
        std::string escaped = "\"";
        for (const unsigned char ch : text) {
            if (ch < 32) throw std::runtime_error("Control character in map provenance");
            if (ch == '\\' || ch == '"') escaped += '\\';
            escaped += static_cast<char>(ch);
        }
        return escaped + '"';
    }
};
}
#endif
