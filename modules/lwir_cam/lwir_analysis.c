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

/** @file
 * @brief Serial saved-image analysis adapter, not part of the camera driver.
 *
 * @details Built only when `LWIR_ENABLE_ANALYSIS=1`. Works on an already saved
 * LWIR JPEG: extracts the embedded exact Kelvin*64 temperature plane, detects
 * hotspots, reads the observation pose from EXIF ImageDescription, optionally
 * projects each hotspot to the ground, and writes a text record into EXIF
 * UserComment. The entropy-coded display image is not decoded or recompressed.
 *
 * Memory: large buffers are file-scope statics (no heap, no large stack frames
 * apart from the 8 KiB record and EXIF payload copy in read_pose()), which is
 * why the adapter is strictly serial.
 */
#include "lwir_analysis.h"
#include "lwir_detection.h"
#include "lwir_geolocation.h"

#include "lwir_container.h"
#include "image_exif.h"
#include "lwir_frame.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

/* Static rather than stack/heap: together these exceed typical worker stacks, and
 * keeping them here makes the whole adapter single-instance (not reentrant). */
static struct exif_document geolocation_exif_document;
static unsigned char geolocation_exif_payload[EXIF_MAX_PAYLOAD];
/* Copy of the original ImageDescription, prepended to the UserComment record. */
static unsigned char geolocation_description[8192];
/* Native little-endian Kelvin*64 plane; the detector reads it, never modifies it. */
static uint8_t temperature_plane[LWIR_MAX_WIDTH * LWIR_MAX_HEIGHT * 2];
static struct lwir_detection_workspace detection_workspace;

/**
 * @brief Decode one unsigned 16-bit little-endian value.
 * @details Byte-wise assembly is independent of host endianness and alignment,
 * so the plane can be read straight from an arbitrary byte buffer.
 * @param bytes At least two readable bytes.
 * @return The decoded value.
 */
static uint16_t read_u16_le(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8);
}

/**
 * @brief Raw Kelvin*64 code of one pixel in the static temperature plane.
 * @param pixel Row-major native index (y * width + x); caller bounds-checks it.
 * @return Raw code; 0 and 65535 are sensor endpoint codes, not real temperatures.
 * @note Celsius is code / 64 - 273.15.
 */
static uint16_t temperature_at(size_t pixel)
{
    return read_u16_le(temperature_plane + pixel * 2);
}

/**
 * @brief Intermediate pose values parsed from the ImageDescription key=value list.
 * @details Angles are still in degrees here; read_pose() converts them.
 * `agl` holds height above ground from `altitude_agl_m` or `groundalt_m`.
 * `seen` is a bitmask indexed like the key table in read_pose(), used both for
 * duplicate-key rejection and required-key checks. `uncertainty` stays -1 when
 * absent, which later disqualifies `synchronized`.
 */
struct pose_fields {
    double latitude, longitude, altitude, agl, roll, pitch, yaw, uncertainty;
    unsigned seen;
    bool synchronized, exposure_interval;
};

/**
 * @brief Parse a finite decimal number from a non-NUL-terminated slice.
 * @details Trims spaces/tabs, copies the slice into a bounded local buffer so
 * strtod() has a terminator (EXIF text is not guaranteed to be NUL-terminated at
 * the slice end), and requires strtod() to consume every remaining character.
 * @param value Start of the value slice (may be unterminated).
 * @param length Slice length in bytes.
 * @param[out] number Written only on success.
 * @return True for a complete finite number; false for empty, overlong (>= 96
 * bytes after trimming), partial, out-of-range (ERANGE), NaN or infinite input.
 * @note Locale-dependent strtod(); presumably the process uses the "C" locale.
 */
static bool parse_description_number(const char *value, size_t length, double *number)
{
    char text[96];
    while (length && (*value == ' ' || *value == '\t')) { ++value; --length; }
    while (length && (value[length - 1] == ' ' || value[length - 1] == '\t')) --length;
    if (!length || length >= sizeof(text)) return false;
    for (size_t index = 0; index < length; ++index) text[index] = value[index];
    text[length] = '\0';
    errno = 0;
    char *end = NULL;
    const double parsed = strtod(text, &end);
    if (errno || end == text || *end || !isfinite(parsed)) return false;
    *number = parsed;
    return true;
}

/**
 * @brief Read the observation pose recorded in a saved image's EXIF ImageDescription.
 * @details The description is a `key=value; key=value` list written at capture
 * time. Segments end at ';' or NUL; segments without '=' and unknown keys are
 * ignored, but a duplicated known key or an unparsable number rejects the whole
 * pose rather than guessing which value is right.
 *
 * Required: `lat_deg`, `lon_deg`, `alt_m` (WGS84 ellipsoidal height, as sent by
 * the flight controller), `roll_deg`, `pitch_deg`, `yaw_deg`, and either
 * `altitude_agl_m` or `groundalt_m`. Both carry height above ground: USHER's
 * `groundalt_m` is the flight controller's `groundalt` field, which Paparazzi
 * fills with altitude above ground, not the ground's own altitude. When both are
 * present `altitude_agl_m` wins.
 *
 * Timing qualification: `synchronized` is true only when
 * `capture_pose_synchronized=true`, `observation_time_kind` names an
 * exposure-bounding interval, and a non-negative `pose_uncertainty_m` is present.
 * Any other time kind (for example a plain arrival time) leaves it false.
 * @param filename Saved JPEG to read; opened independently of the publisher.
 * @param[out] pose Written only on success; angles converted to radians.
 * @return True when a complete pose was parsed; false when EXIF or the
 * description is absent/malformed or required keys are missing. Callers treat
 * false as "missing pose", never as a zero pose.
 * @note Uses a stack copy of the EXIF payload (EXIF_MAX_PAYLOAD bytes).
 */
static bool read_pose(const char *filename, struct lwir_camera_pose *pose)
{
    unsigned char payload[EXIF_MAX_PAYLOAD];
    size_t payload_size = 0;
    struct exif_view view;
    struct exif_value description;
    if (exif_read_jpeg(filename, payload, sizeof(payload), &payload_size) != 0
        || exif_parse(&view, payload, payload_size) != 0
        || !exif_get(&view, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, &description)
        || description.format != EXIF_ASCII || description.size > 8192) return false;
    struct pose_fields fields = {.uncertainty = -1};
    size_t offset = 0;
    while (offset < description.size) {
        const size_t start = offset;
        while (offset < description.size && description.data[offset] != ';' && description.data[offset] != 0) ++offset;
        const size_t end = offset;
        if (offset < description.size) {
            if (description.data[offset] == ';') ++offset;
            else offset = description.size;
        }
        size_t separator = start;
        while (separator < end && description.data[separator] != '=') ++separator;
        if (separator == end) continue;
        size_t key_start = start, key_end = separator;
        while (key_start < key_end && (description.data[key_start] == ' ' || description.data[key_start] == '\t')) ++key_start;
        while (key_end > key_start && (description.data[key_end - 1] == ' ' || description.data[key_end - 1] == '\t')) --key_end;
        const char *key = (const char *)description.data + key_start;
        const size_t key_length = key_end - key_start;
        const char *value = (const char *)description.data + separator + 1;
        size_t value_length = end - separator - 1;
        while (value_length && (*value == ' ' || *value == '\t')) { ++value; --value_length; }
        while (value_length && (value[value_length - 1] == ' ' || value[value_length - 1] == '\t')) --value_length;
        static const char *const names[] = {"lat_deg", "lon_deg", "alt_m", "roll_deg", "pitch_deg", "yaw_deg",
            "altitude_agl_m", "groundalt_m", "pose_uncertainty_m", "capture_pose_synchronized", "observation_time_kind"};
        unsigned field = 0;
        while (field < sizeof(names) / sizeof(names[0])
            && (strlen(names[field]) != key_length || memcmp(key, names[field], key_length))) ++field;
        if (field == sizeof(names) / sizeof(names[0])) continue;
        const unsigned field_bit = 1U << field;
        if (fields.seen & field_bit) return false;
        fields.seen |= field_bit;
        if (field == 9) {
            while (value_length && (*value == ' ' || *value == '\t')) { ++value; --value_length; }
            fields.synchronized = value_length == 4 && !memcmp(value, "true", 4);
            if (!fields.synchronized && !(value_length == 5 && !memcmp(value, "false", 5))) return false;
            continue;
        }
        if (field == 10) {
            /* Command-to-download presumably qualifies because it brackets the
             * exposure, unlike an arrival time that only follows it. */
            fields.exposure_interval = (value_length == 24 && !memcmp(value, "sensor_exposure_interval", 24))
                || (value_length == 28 && !memcmp(value, "calibrated_exposure_interval", 28))
                || (value_length == sizeof("command_to_download_interval") - 1 &&
                    !memcmp(value, "command_to_download_interval", sizeof("command_to_download_interval") - 1));
            continue;
        }
        double number;
        if (!parse_description_number(value, value_length, &number)) return false;
        switch (field) {
        case 0: fields.latitude = number; break;
        case 1: fields.longitude = number; break;
        case 2: fields.altitude = number; break;
        case 3: fields.roll = number; break;
        case 4: fields.pitch = number; break;
        case 5: fields.yaw = number; break;
        case 6: fields.agl = number; break;
        /* groundalt_m is a fallback for the same quantity; never overwrite an explicit AGL. */
        case 7: if (!(fields.seen & (1U << 6))) fields.agl = number; break;
        case 8: fields.uncertainty = number; break;
        default: return false;
        }
    }
    const unsigned required = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4) | (1U << 5);
    if ((fields.seen & required) != required || !(fields.seen & ((1U << 6) | (1U << 7)))) return false;
    *pose = (struct lwir_camera_pose){
        .latitude_deg = fields.latitude, .longitude_deg = fields.longitude,
        .altitude_ellipsoid_m = fields.altitude, .agl_m = fields.agl,
        .roll_rad = fields.roll * (3.14159265358979323846 / 180),
        .pitch_rad = fields.pitch * (3.14159265358979323846 / 180),
        .yaw_rad = fields.yaw * (3.14159265358979323846 / 180),
        .uncertainty_m = fields.uncertainty,
        .synchronized = fields.synchronized && fields.exposure_interval && fields.uncertainty >= 0};
    return true;
}

/**
 * @brief Append printf-formatted text to a fixed record buffer.
 * @details Fails instead of truncating, so a published record is never silently
 * cut in the middle of a key/value pair.
 * @param record Destination buffer, NUL-terminated after each successful append.
 * @param capacity Total size of @p record in bytes.
 * @param[in,out] used Bytes already used (excluding the terminator); advanced on success.
 * @param format printf format string.
 * @param ... Format arguments.
 * @return True on success; false on encoding error or insufficient space.
 */
static bool append_record(char *record, size_t capacity, size_t *used, const char *format, ...)
{
    if (*used >= capacity) return false;
    va_list arguments;
    va_start(arguments, format);
    const int count = vsnprintf(record + *used, capacity - *used, format, arguments);
    va_end(arguments);
    if (count < 0 || (size_t)count >= capacity - *used) return false;
    *used += (size_t)count;
    return true;
}

/**
 * @brief Store the analysis record in the in-memory EXIF document's UserComment.
 * @details Layout: the 8-byte EXIF character-code prefix `ASCII\0\0\0`, the
 * original ImageDescription text (so pose evidence is visible in one field), a
 * `\nLWIR_HOTSPOTS_V1\n` version separator, the record, and a terminating NUL.
 * The description is copied out first because exif_document_reserve() can move
 * existing values that `description.data` borrows.
 * @param information NUL-terminated record, at most 8192 characters.
 * @return True when the tag was reserved and filled; false for an oversized or
 * non-ASCII-format description, an oversized record, or exhausted document
 * capacity (errno ENOSPC/EINVAL from exif_document_reserve()).
 * @note exif_document_reserve() replaces an existing UserComment, so re-running
 * analysis overwrites rather than stacks records.
 */
static bool set_hotspot_comment(const char *information)
{
    struct exif_value description = {0};
    size_t description_length = 0;
    if (exif_document_get(&geolocation_exif_document, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, &description)) {
        if (description.format != EXIF_ASCII || description.size > sizeof(geolocation_description)) return false;
        description_length = exif_text_length(&description);
        if (description_length > sizeof(geolocation_description)) return false;
        for (size_t index = 0; index < description_length; ++index)
            geolocation_description[index] = description.data[index];
    }
    const size_t information_length = strnlen(information, 8193);
    if (information_length > 8192) return false;
    static const unsigned char prefix[8] = {'A', 'S', 'C', 'I', 'I', 0, 0, 0};
    const size_t tag_length = sizeof("\nLWIR_HOTSPOTS_V1\n") - 1;
    const size_t total = sizeof(prefix) + description_length + tag_length + information_length + 1;
    if (total > UINT32_MAX) return false;
    unsigned char *comment = exif_document_reserve(&geolocation_exif_document, EXIF_CAMERA,
        EXIF_USER_COMMENT, EXIF_UNDEFINED, (uint32_t)total);
    if (!comment) return false;
    size_t offset = 0;
    for (size_t index = 0; index < sizeof(prefix); ++index) comment[offset++] = prefix[index];
    for (size_t index = 0; index < description_length; ++index) comment[offset++] = geolocation_description[index];
    static const char separator[] = "\nLWIR_HOTSPOTS_V1\n";
    for (size_t index = 0; index < tag_length; ++index) comment[offset++] = (unsigned char)separator[index];
    for (size_t index = 0; index < information_length; ++index) comment[offset++] = (unsigned char)information[index];
    comment[offset] = 0;
    return true;
}

/**
 * @brief Rewrite a JPEG's EXIF with the analysis record, keeping image data intact.
 * @details Opens with O_NOFOLLOW and requires a regular file, so a symlink cannot
 * redirect the write. Existing EXIF is imported (ENODATA means none, which is
 * acceptable); other read/parse failures abort rather than discard metadata.
 *
 * With IMAGE_EXIF_ATOMIC_REPLACE the JPEG is streamed to a sibling mkstemp()
 * file (same directory, so rename() stays atomic on one filesystem) with the
 * original permission bits, fsync()ed, and renamed over the original only if the
 * path still names the same inode (dev/ino), so a concurrently replaced file is
 * not clobbered. Otherwise the EXIF segment is updated in place.
 * @param filename Target JPEG path.
 * @param information NUL-terminated analysis record.
 * @return 0 on success, -1 on any open, stat, EXIF, write, sync or rename
 * failure (errno may reflect the failing call; not guaranteed).
 * @note The containing directory is not fsync()ed after rename(), so the rename
 * itself may not survive a sudden power loss.
 * @warning In-place mode cannot roll back a partially written update.
 */
static int publish_hotspot_comment(const char *filename, const char *information)
{
    if (!filename || !information) return -1;
    const int input_fd = open(filename, O_RDWR | O_CLOEXEC | O_NOFOLLOW);
    if (input_fd < 0) return -1;
    struct stat original;
    if (fstat(input_fd, &original) != 0 || !S_ISREG(original.st_mode)) { close(input_fd); return -1; }
    FILE *input = fdopen(input_fd, "r+b");
    if (!input) { close(input_fd); return -1; }
    size_t payload_size = 0;
    struct exif_view view;
    exif_document_init(&geolocation_exif_document);
    bool success = fseeko(input, 0, SEEK_SET) == 0;
    if (success) {
        const int read_result = exif_read_stream(input, geolocation_exif_payload,
            sizeof(geolocation_exif_payload), &payload_size);
        if (read_result == 0) {
            success = exif_parse(&view, geolocation_exif_payload, payload_size) == 0
                && image_exif_import(&geolocation_exif_document, &view) == 0;
        } else if (errno != ENODATA) {
            success = false;
        }
    }
    success = success && set_hotspot_comment(information)
        && exif_serialize(&geolocation_exif_document, geolocation_exif_payload,
                          sizeof(geolocation_exif_payload), &payload_size) == 0;
#if IMAGE_EXIF_ATOMIC_REPLACE
    char temporary[PATH_MAX];
    int temporary_length = snprintf(temporary, sizeof(temporary), "%s.lwir.XXXXXX", filename);
    int output_fd = -1;
    bool temporary_created = false;
    FILE *output = NULL;
    if (success && (temporary_length < 0 || (size_t)temporary_length >= sizeof(temporary))) success = false;
    if (success) output_fd = mkstemp(temporary);
    if (success && output_fd < 0) success = false;
    if (output_fd >= 0) temporary_created = true;
    if (success && fchmod(output_fd, original.st_mode & 0777) != 0) success = false;
    if (success) output = fdopen(output_fd, "wb");
    if (success && !output) success = false;
    /* The FILE now owns the descriptor; avoid a double close below. */
    if (output) output_fd = -1;
    if (success) success = fseeko(input, 0, SEEK_SET) == 0
        && exif_rewrite_jpeg(input, output, geolocation_exif_payload, payload_size) == 0
        && fflush(output) == 0 && fsync(fileno(output)) == 0;
    if (output && fclose(output) != 0) success = false;
    if (output_fd >= 0) close(output_fd);
    struct stat current;
    if (success) success = lstat(filename, &current) == 0
        && current.st_dev == original.st_dev && current.st_ino == original.st_ino;
    if (success) success = rename(temporary, filename) == 0;
    if (!success && temporary_created) unlink(temporary);
#else
    success = success && fseeko(input, 0, SEEK_SET) == 0
        && exif_update_jpeg(input, geolocation_exif_payload, payload_size) == 0
        && fsync(fileno(input)) == 0;
#endif
    if (fclose(input) != 0) success = false;
    return success ? 0 : -1;
}

/**
 * @brief Detect, optionally geolocate and record hotspots for one saved JPEG.
 * @details See lwir_analysis.h for the contract. Order of evaluation:
 * -# Read the embedded temperature plane: invalid -> `status=invalid_native_frame`;
 *    absent -> `status=no_temperature_plane` (both published, return value from publishing).
 * -# Read pose and calibration (each may be missing without aborting).
 * -# Detect hotspots, then derive a single status with precedence
 *    missing pose > missing calibration > invalid calibration > unverified
 *    calibration, finally overridden by `no_hotspots`.
 * -# For each hotspot, report the rounded pixel, its nearest-pixel temperature
 *    and the subpixel projection input; project only when status is `estimated`.
 * @return 0 when a record was published, -1 otherwise.
 */
int lwir_geolocate_image(const char *filename, const char *calibration_path)
{
    if (!filename || !*filename) return -1;
    unsigned width = 0, height = 0;
    bool synthetic = false;
    const int layer = lwir_read_temperature_layer(filename, temperature_plane, sizeof(temperature_plane),
        &width, &height, &synthetic);
    char record[8192];
    size_t used = 0;
    if (layer < 0)
            return publish_hotspot_comment(filename, "status=invalid_native_frame; count=0; coordinates_omitted=true");
    /* Header states the model limits next to every result: flat ground, zero GPS
     * antenna offset, apparent (uncorrected) temperatures, unknown accuracy. */
    if (!append_record(record, sizeof(record), &used,
        "coordinate_datum=WGS84; ground_model=local_horizontal_plane; gps_camera_offset_m=0; "
        "temperature_surface_correction=not_applied; absolute_accuracy_m=unknown; temperature_kind=%s; ",
        synthetic ? "synthetic" : "sensor_apparent")) return -1;
    if (layer == 0) {
        if (!append_record(record, sizeof(record), &used, "status=no_temperature_plane; count=0")) return -1;
            return publish_hotspot_comment(filename, record);
    }
    struct lwir_camera_pose pose = {0};
    const bool have_pose = read_pose(filename, &pose);
    struct lwir_camera_geometry geometry = {0};
    const bool have_calibration = calibration_path && *calibration_path
        && lwir_geolocation_load_calibration(calibration_path, width, height, &geometry);
    struct lwir_hotspot_set hotspots;
    if (!lwir_detect_hotspots(temperature_plane, (size_t)width * height * 2, width, height,
        &detection_workspace, &hotspots)) return -1;
    const char *status = "estimated";
    if (!have_pose) status = "missing_shot_pose";
    else if (!calibration_path || !*calibration_path) status = "missing_camera_calibration";
    else if (!have_calibration) status = "invalid_camera_calibration";
    /* Synthetic frames presumably come from simulation with known geometry, so an
     * unverified profile is accepted there but never for sensor data. Pose timing
     * is deliberately not gated here; it is reported for the consumer instead. */
    else if (!geometry.verified && !synthetic) status = "unverified_camera_calibration";
    if (hotspots.count == 0) status = "no_hotspots";
    if (!append_record(record, sizeof(record), &used,
        "capture_pose_synchronized=%s; capture_pose_uncertainty_m=%.6f; status=%s; calibration_verified=%s; "
        "count=%zu; generic_candidate_count=%zu; tray_candidate_count=%zu; "
        "selection_pass=%s; center_method=subpixel_temperature_weighted_centroid; "
        "temperature_sample_method=nearest_pixel; temperature_units=C\n",
        pose.synchronized ? "true" : "false", pose.uncertainty_m, status,
        have_calibration && geometry.verified ? "true" : "false", hotspots.count,
        hotspots.generic_count, hotspots.tray_count,
        hotspots.tray_pass_selected ? "fire_tray_hypothesis" : "generic_hotspot_fallback")) return -1;
    for (size_t index = 0; index < hotspots.count; ++index) {
        const struct lwir_hotspot *hotspot = &hotspots.candidates[index];
        /* The weighted centroid lies within the component's bounding box, so this
         * check is defensive. The pixel nearest the centroid may be cooler than the
         * threshold (e.g. a ring-shaped blob), hence center_above_detection_threshold. */
        const unsigned pixel_x = (unsigned)lround(hotspot->center_x);
        const unsigned pixel_y = (unsigned)lround(hotspot->center_y);
        if (pixel_x >= width || pixel_y >= height) return -1;
        const uint16_t raw = temperature_at((size_t)pixel_y * width + pixel_x);
        if (!append_record(record, sizeof(record), &used,
            "hotspot=%zu; pixel_x=%u; pixel_y=%u; center_temperature_c=%.6f; raw_kelvin_x64=%u; "
            "center_above_detection_threshold=%s; projection_pixel_x=%.6f; projection_pixel_y=%.6f",
            index + 1, pixel_x, pixel_y, raw / 64.0 - 273.15, raw,
            raw / 64.0 - 273.15 >= hotspots.threshold_celsius ? "true" : "false",
            hotspot->center_x, hotspot->center_y)) return -1;
        /* Project the subpixel centroid, not the rounded pixel, to avoid up to
         * half a pixel of extra ground error. */
        struct lwir_ground_point point;
        if (!strcmp(status, "estimated") && lwir_geolocation_project(hotspot->center_x, hotspot->center_y, &geometry, &pose, &point)) {
            if (!append_record(record, sizeof(record), &used,
                "; location_status=estimated; latitude_deg=%.9f; longitude_deg=%.9f; "
                "altitude_ellipsoid_m=%.6f; slant_range_m=%.6f\n",
                point.latitude_deg, point.longitude_deg, point.altitude_ellipsoid_m, point.slant_range_m)) return -1;
        } else {
            const char *location_status = !strcmp(status, "estimated") ? "invalid_geometry_or_horizon" : status;
            if (!append_record(record, sizeof(record), &used, "; location_status=%s\n", location_status)) return -1;
        }
    }
        return publish_hotspot_comment(filename, record);
}