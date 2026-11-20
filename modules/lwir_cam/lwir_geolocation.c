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
 * @brief Native calibration and projection; no detector, JPEG codec or flight parser.
 *
 * @details Analysis builds only (`LWIR_ENABLE_ANALYSIS=1`). Provides a bounded,
 * line-oriented calibration reader and a pixel-to-ground projection used on
 * hotspots found in saved LWIR JPEG temperature planes. Fixed-size buffers only;
 * no heap allocation. See lwir_geolocation.h for frame conventions and model limits.
 */
#include "lwir_geolocation.h"
#include "lwir_frame.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

/* Upper bound on calibration file size, keeping parsing bounded and cheap. */
enum { lwir_calibration_max = 8192 };

/** @brief Axis identities for rays and lever arms after rotation into NED. */
enum ned_axis { NED_NORTH, NED_EAST, NED_DOWN };

/**
 * @brief Parse one finite number at a cursor, skipping surrounding blanks.
 * @details Rejects an empty field or a leading list/comment delimiter before
 * calling strtod(), and requires the whole parsed value to be finite.
 * @param[in,out] cursor Advanced past the number and trailing blanks on success.
 * @param[out] value Written only on success.
 * @return False for empty, unparsable, ERANGE, NaN or infinite input.
 */
static bool parse_number(const char **cursor, double *value)
{
    while (**cursor == ' ' || **cursor == '\t') ++*cursor;
    if (**cursor == '\0' || **cursor == ',' || **cursor == ']' || **cursor == '#') return false;
    errno = 0;
    char *end = NULL;
    const double parsed = strtod(*cursor, &end);
    if (errno != 0 || end == *cursor || !isfinite(parsed)) return false;
    *cursor = end;
    while (**cursor == ' ' || **cursor == '\t') ++*cursor;
    *value = parsed;
    return true;
}

/**
 * @brief Parse a value that must be exactly one number, optionally followed by a comment.
 * @param text NUL-terminated value text after the key's colon.
 * @param[out] value Parsed number; may be written even when trailing text fails.
 * @return True when only blanks or a '#' comment follow the number.
 */
static bool parse_scalar(const char *text, double *value)
{
    const char *cursor = text;
    if (!parse_number(&cursor, value)) return false;
    if (*cursor == '#') return true;
    return *cursor == '\0';
}

/**
 * @brief Parse a nonnegative decimal integer that fits in unsigned int.
 * @details Requires a leading digit so strtoul() cannot accept a sign ("-1"
 * would otherwise wrap to a huge value).
 * @param text NUL-terminated value text.
 * @param[out] value Written only on success.
 * @return True for a complete in-range integer, optionally followed by a comment.
 */
static bool parse_unsigned_scalar(const char *text, unsigned *value)
{
    const char *cursor = text;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor < '0' || *cursor > '9') return false;
    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(cursor, &end, 10);
    if (errno != 0 || end == cursor || parsed > UINT_MAX) return false;
    while (*end == ' ' || *end == '\t') ++end;
    if (*end && *end != '#') return false;
    *value = (unsigned)parsed;
    return true;
}

/**
 * @brief Parse a single-line flow array `[a, b, ...]` with exactly @p count numbers.
 * @param text NUL-terminated value text.
 * @param[out] values Receives @p count numbers; may be partially written on failure.
 * @param count Required element count; more or fewer elements fail.
 * @return True for a well-formed array optionally followed by a comment.
 */
static bool parse_array(const char *text, double *values, size_t count)
{
    const char *cursor = text;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor++ != '[') return false;
    for (size_t index = 0; index < count; ++index) {
        if (!parse_number(&cursor, &values[index])) return false;
        if (index + 1 < count) {
            if (*cursor++ != ',') return false;
        } else if (*cursor++ != ']') {
            return false;
        }
    }
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    return *cursor == '\0' || *cursor == '#';
}

/**
 * @brief Load a calibration profile for one native image size.
 * @details Reads `key: value` lines (a YAML-like subset) into a local copy and
 * publishes it only after all checks pass, so @p geometry is never half-written.
 * A bitmask of seen keys detects duplicates and missing required keys.
 * @return True for a structurally valid profile; false for I/O errors, a
 * non-regular/empty/oversized file, syntax errors, duplicates, missing keys or
 * out-of-range values.
 */
bool lwir_geolocation_load_calibration(const char *path, unsigned width, unsigned height,
                                       struct lwir_camera_geometry *geometry)
{
    if (!path || !*path || !geometry || !width || !height ||
        width > LWIR_MAX_WIDTH || height > LWIR_MAX_HEIGHT) return false;
    FILE *input = fopen(path, "rb");
    if (!input) return false;
    /* Size is checked on the opened descriptor, so the file cannot be swapped
     * between the check and the read. */
    struct stat status;
    bool valid = fstat(fileno(input), &status) == 0 && S_ISREG(status.st_mode)
        && status.st_size > 0 && status.st_size <= lwir_calibration_max;
    struct lwir_camera_geometry parsed = {
        .camera_to_body = {0, -1, 0, 1, 0, 0, 0, 0, 1},
        .minimum_ground_cosine = 0.2,
        .image_width = width,
        .image_height = height
    };
    enum { key_width = 0, key_height, key_fx, key_fy, key_cx, key_cy,
           key_k1, key_k2, key_p1, key_p2, key_k3, key_verified,
           key_rotation, key_lever_arm, key_minimum_cosine };
    unsigned seen = 0;
    char line[512];
    size_t total = 0;
    while (valid && fgets(line, sizeof(line), input)) {
        size_t line_length = strlen(line);
        total += line_length;
        /* The running total also bounds files that grew after fstat(); a line
         * filling the buffer without a newline (and not at EOF) is too long. */
        if (total > lwir_calibration_max || (line_length && line[line_length - 1] != '\n' && !feof(input))) {
            valid = false;
            break;
        }
        while (line_length && (line[line_length - 1] == '\n' || line[line_length - 1] == '\r'))
            line[--line_length] = '\0';
        char *cursor = line;
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') ++cursor;
        /* Skip blanks, comments, YAML directives ('%') and the document marker. */
        if (!*cursor || *cursor == '#' || *cursor == '%' || !strcmp(cursor, "---")) continue;
        char *colon = strchr(cursor, ':');
        if (!colon) { valid = false; break; }
        char *key_end = colon;
        while (key_end > cursor && (key_end[-1] == ' ' || key_end[-1] == '\t')) --key_end;
        *key_end = '\0';
        const char *value = colon + 1;
        while (*value == ' ' || *value == '\t') ++value;
        unsigned field = 0;
        const char *const names[] = {"image_width", "image_height", "fx", "fy", "cx", "cy",
            "k1", "k2", "p1", "p2", "k3", "verified", "camera_to_body",
            "lever_arm_body_m", "minimum_ground_cosine"};
        while (field < sizeof(names) / sizeof(names[0]) && strcmp(cursor, names[field])) ++field;
        /* Unknown keys are tolerated, presumably so profiles can carry extra
         * metadata (e.g. from other calibration tools). */
        if (field == sizeof(names) / sizeof(names[0])) continue;
        if (seen & (1U << field)) { valid = false; break; }
        seen |= 1U << field;
        if (field == key_width || field == key_height) {
            /* Intrinsics are only valid for the resolution they were measured at. */
            unsigned dimension;
            valid = parse_unsigned_scalar(value, &dimension) &&
                dimension == (field == key_width ? width : height);
        } else if (field == key_rotation || field == key_lever_arm) {
            double values[9];
            const size_t number_count = field == key_rotation ? 9 : 3;
            valid = parse_array(value, values, number_count);
            if (valid && field == key_rotation)
                for (size_t index = 0; index < 9; ++index) parsed.camera_to_body[index] = values[index];
            if (valid && field == key_lever_arm)
                for (size_t index = 0; index < 3; ++index) parsed.lever_arm_body_m[index] = values[index];
        } else {
            double number;
            valid = parse_scalar(value, &number);
            if (!valid) break;
            switch (field) {
            case key_fx: parsed.fx = number; break;
            case key_fy: parsed.fy = number; break;
            case key_cx: parsed.cx = number; break;
            case key_cy: parsed.cy = number; break;
            case key_k1: parsed.k1 = number; break;
            case key_k2: parsed.k2 = number; break;
            case key_p1: parsed.p1 = number; break;
            case key_p2: parsed.p2 = number; break;
            case key_k3: parsed.k3 = number; break;
            case key_verified:
                if (number != 0 && number != 1) valid = false;
                else parsed.verified = number == 1;
                break;
            case key_minimum_cosine: parsed.minimum_ground_cosine = number; break;
            default: valid = false; break;
            }
        }
        if (!valid) break;
    }
    if (ferror(input)) valid = false;
    if (fclose(input) != 0) valid = false;
    /* Lever arm and minimum cosine are optional; the rotation is required even
     * though a default exists, presumably so the mount is always stated explicitly. */
    const unsigned required = (1U << key_width) | (1U << key_height) | (1U << key_fx)
        | (1U << key_fy) | (1U << key_cx) | (1U << key_cy) | (1U << key_k1)
        | (1U << key_k2) | (1U << key_p1) | (1U << key_p2) | (1U << key_k3)
        | (1U << key_verified) | (1U << key_rotation);
    if (!valid || (seen & required) != required || parsed.fx <= 0 || parsed.fy <= 0 ||
        parsed.minimum_ground_cosine < 0.05 || parsed.minimum_ground_cosine > 1) return false;
    *geometry = parsed;
    return true;
}

/**
 * @brief Multiply a row-major 3x3 matrix by a 3-vector.
 * @param matrix Row-major matrix.
 * @param vector Input vector; must not alias @p result.
 * @param[out] result matrix * vector.
 */
static void rotate_vector(const double matrix[9], const double vector[3], double result[3])
{
    for (size_t row = 0; row < 3; ++row) {
        result[row] = 0;
        for (size_t column = 0; column < 3; ++column)
            result[row] += matrix[3 * row + column] * vector[column];
    }
}

/**
 * @brief Project a native subpixel location onto the local ground plane.
 * @details Steps:
 * -# Validate every input (finite, in range, proper rotation).
 * -# Pixel -> distorted normalized coordinates via intrinsics.
 * -# Invert Brown-Conrady distortion by fixed-point iteration.
 * -# Build the camera ray (x, y, 1), rotate camera -> body -> NED.
 * -# Intersect with the plane `agl_m` below the navigation reference, starting
 *    from the lever-arm-offset camera origin.
 * -# Convert the NED offset to WGS84 latitude/longitude via ECEF.
 * @return False for invalid inputs, an improper mount rotation, nonconvergent
 * undistortion, a ray too close to the horizon, or a camera origin at/below
 * the ground plane.
 * @note Pixel coordinates are combined with cx/cy as-is, so they must follow the
 * calibration's convention (OpenCV: integer coordinates at pixel centres).
 */
bool lwir_geolocation_project(double pixel_x, double pixel_y,
                              const struct lwir_camera_geometry *camera,
                              const struct lwir_camera_pose *pose,
                              struct lwir_ground_point *point)
{
    if (!camera || !pose || !point) return false;
    const double values[] = {pixel_x, pixel_y, camera->fx, camera->fy, camera->cx, camera->cy,
        camera->k1, camera->k2, camera->p1, camera->p2, camera->k3, camera->minimum_ground_cosine,
        pose->latitude_deg, pose->longitude_deg, pose->altitude_ellipsoid_m, pose->agl_m,
        pose->roll_rad, pose->pitch_rad, pose->yaw_rad};
    for (size_t index = 0; index < sizeof(values) / sizeof(values[0]); ++index)
        if (!isfinite(values[index])) return false;
    if (!camera->image_width || !camera->image_height || camera->image_width > LWIR_MAX_WIDTH ||
        camera->image_height > LWIR_MAX_HEIGHT || pixel_x < 0 || pixel_y < 0 ||
        pixel_x >= camera->image_width || pixel_y >= camera->image_height ||
        camera->fx <= 0 || camera->fy <= 0 || pose->agl_m <= 0 ||
        fabs(pose->latitude_deg) >= 90 || fabs(pose->longitude_deg) > 180 ||
        camera->minimum_ground_cosine < 0.05 || camera->minimum_ground_cosine > 1) return false;
    for (size_t index = 0; index < 3; ++index)
        if (!isfinite(camera->lever_arm_body_m[index])) return false;
    for (size_t index = 0; index < 9; ++index)
        if (!isfinite(camera->camera_to_body[index])) return false;
    /* A reflected/skewed mount can produce plausible but wrong ground positions. */
    /* Orthonormality: R * R^T must equal I, checked row against row. */
    for (size_t row = 0; row < 3; ++row) {
        for (size_t other = 0; other < 3; ++other) {
            double product = 0;
            for (size_t column = 0; column < 3; ++column)
                product += camera->camera_to_body[3 * row + column] * camera->camera_to_body[3 * other + column];
            if (fabs(product - (row == other ? 1.0 : 0.0)) > 1e-6) return false;
        }
    }
    const double *mount = camera->camera_to_body;
    /* Orthonormal matrices have determinant +/-1; -1 is a mirror, not a mount. */
    const double determinant = mount[0] * (mount[4] * mount[8] - mount[5] * mount[7])
        - mount[1] * (mount[3] * mount[8] - mount[5] * mount[6])
        + mount[2] * (mount[3] * mount[7] - mount[4] * mount[6]);
    if (fabs(determinant - 1) > 1e-6) return false;
    /* Pinhole inverse: normalized image-plane coordinates still containing lens
     * distortion, i.e. x_d = (u - cx) / fx, y_d = (v - cy) / fy. */
    const double distorted_x = (pixel_x - camera->cx) / camera->fx;
    const double distorted_y = (pixel_y - camera->cy) / camera->fy;
    double undistorted_x = distorted_x, undistorted_y = distorted_y;
    /* Bound inverse distortion work; failed convergence must not become a coordinate. */
    /* Forward model (OpenCV): x_d = x * k_r + t_x, y_d = y * k_r + t_y, with
     *   r^2 = x^2 + y^2,  k_r = 1 + k1 r^2 + k2 r^4 + k3 r^6,
     *   t_x = 2 p1 x y + p2 (r^2 + 2 x^2),  t_y = p1 (r^2 + 2 y^2) + 2 p2 x y.
     * It has no closed-form inverse, so iterate x <- (x_d - t_x) / k_r from the
     * distorted point until the forward residual is below 1e-12 (normalized
     * units), an iterative scheme similar to cv::undistortPoints. A nonpositive k_r
     * presumably means the polynomial has folded over, so the ray is rejected. */
    bool converged = false;
    for (unsigned iteration = 0; iteration < 50; ++iteration) {
        const double radius2 = undistorted_x * undistorted_x + undistorted_y * undistorted_y;
        const double radial = 1 + radius2 * (camera->k1 + radius2 * (camera->k2 + radius2 * camera->k3));
        if (!isfinite(radial) || radial <= 0) return false;
        const double tangent_x = 2 * camera->p1 * undistorted_x * undistorted_y +
            camera->p2 * (radius2 + 2 * undistorted_x * undistorted_x);
        const double tangent_y = camera->p1 * (radius2 + 2 * undistorted_y * undistorted_y) +
            2 * camera->p2 * undistorted_x * undistorted_y;
        if (hypot(undistorted_x * radial + tangent_x - distorted_x,
                  undistorted_y * radial + tangent_y - distorted_y) < 1e-12) {
            converged = true;
            break;
        }
        undistorted_x = (distorted_x - tangent_x) / radial;
        undistorted_y = (distorted_y - tangent_y) / radial;
    }
    if (!converged) return false;
    /* Keep distortion in camera coordinates; mount and attitude rotations follow convergence. */
    /* Body-to-NED direction cosine matrix for aerospace Z-Y-X Euler angles:
     * R = Rz(yaw) * Ry(pitch) * Rx(roll), i.e. roll applied first in body axes.
     * Row-major; columns are the body forward/right/down axes expressed in NED. */
    const double roll_cos = cos(pose->roll_rad), roll_sin = sin(pose->roll_rad);
    const double pitch_cos = cos(pose->pitch_rad), pitch_sin = sin(pose->pitch_rad);
    const double yaw_cos = cos(pose->yaw_rad), yaw_sin = sin(pose->yaw_rad);
    const double body_to_ned[9] = {
        pitch_cos * yaw_cos, roll_sin * pitch_sin * yaw_cos - roll_cos * yaw_sin, roll_cos * pitch_sin * yaw_cos + roll_sin * yaw_sin,
        pitch_cos * yaw_sin, roll_sin * pitch_sin * yaw_sin + roll_cos * yaw_cos, roll_cos * pitch_sin * yaw_sin - roll_sin * yaw_cos,
        -pitch_sin, roll_sin * pitch_cos, roll_cos * pitch_cos};
    /* Unnormalized ray through the undistorted point at unit depth; only its
     * direction matters, and the norm is applied later for range and the guard. */
    const double camera_ray[3] = {undistorted_x, undistorted_y, 1};
    double body_ray[3], ned_ray[3], ned_origin[3];
    rotate_vector(camera->camera_to_body, camera_ray, body_ray);
    rotate_vector(body_to_ned, body_ray, ned_ray);
    /* Camera origin relative to the navigation reference, in NED. */
    rotate_vector(body_to_ned, camera->lever_arm_body_m, ned_origin);
    const double norm = sqrt(ned_ray[NED_NORTH] * ned_ray[NED_NORTH] +
                             ned_ray[NED_EAST] * ned_ray[NED_EAST] +
                             ned_ray[NED_DOWN] * ned_ray[NED_DOWN]);
    /* Near-horizon intersections amplify small calibration/pose errors without bound. */
    /* down/norm is the cosine between the ray and straight down. The camera
     * origin must also be above the plane, else the intersection is behind it. */
    if (!isfinite(norm) || ned_ray[NED_DOWN] / norm < camera->minimum_ground_cosine ||
        pose->agl_m <= ned_origin[NED_DOWN]) return false;
    /* Ray-plane intersection: origin_down + s * ray_down = agl (ground is +agl
     * down from the reference), so s = (agl - origin_down) / ray_down; ray_down > 0
     * is guaranteed by the cosine guard. Because the ray is unnormalized, the
     * metric range is s * |ray|. Flat-ground gotcha: any terrain height difference
     * dh moves the point by roughly dh * tan(off-nadir angle) horizontally. */
    const double scale = (pose->agl_m - ned_origin[NED_DOWN]) / ned_ray[NED_DOWN];
    struct lwir_ground_point result;
    result.north_m = ned_origin[NED_NORTH] + scale * ned_ray[NED_NORTH];
    result.east_m = ned_origin[NED_EAST] + scale * ned_ray[NED_EAST];
    result.slant_range_m = scale * norm;
    /* Apply the local NED offset in ECEF before recovering geodetic latitude/longitude. */
    /* Step 1: reference position in ECEF on the WGS84 ellipsoid (a = 6378137 m,
     * e^2 = 6.694e-3), with N the prime-vertical radius of curvature:
     *   X = (N + h) cos(lat) cos(lon), Y = (N + h) cos(lat) sin(lon),
     *   Z = (N (1 - e^2) + h) sin(lat).
     * h is the WGS84 ellipsoidal height, exactly what this formula requires.
     * Step 2: add north * n + east * e + agl * d, where the NED unit vectors are
     *   n = (-sin lat cos lon, -sin lat sin lon, cos lat),
     *   e = (-sin lon, cos lon, 0),
     *   d = (-cos lat cos lon, -cos lat sin lon, -sin lat).
     * Doing the offset in ECEF avoids a flat-earth metres-per-degree approximation. */
    const double pi = 3.14159265358979323846;
    const double radius = 6378137.0;
    const double eccentricity_squared = 6.6943799901413165e-3;
    const double latitude = pose->latitude_deg * pi / 180;
    const double longitude = pose->longitude_deg * pi / 180;
    const double latitude_sin = sin(latitude), latitude_cos = cos(latitude);
    const double longitude_sin = sin(longitude), longitude_cos = cos(longitude);
    const double prime_vertical = radius / sqrt(1 - eccentricity_squared * latitude_sin * latitude_sin);
    const double earth_x = (prime_vertical + pose->altitude_ellipsoid_m) * latitude_cos * longitude_cos
        - latitude_sin * longitude_cos * result.north_m - longitude_sin * result.east_m
        - latitude_cos * longitude_cos * pose->agl_m;
    const double earth_y = (prime_vertical + pose->altitude_ellipsoid_m) * latitude_cos * longitude_sin
        - latitude_sin * longitude_sin * result.north_m + longitude_cos * result.east_m
        - latitude_cos * longitude_sin * pose->agl_m;
    const double earth_z = (prime_vertical * (1 - eccentricity_squared) + pose->altitude_ellipsoid_m) * latitude_sin
        + latitude_cos * result.north_m - latitude_sin * pose->agl_m;
    const double horizontal = hypot(earth_x, earth_y);
    /* Step 3: ECEF -> geodetic latitude by fixed-point iteration
     *   lat <- atan2(Z + e^2 N(lat) sin(lat), p),  p = sqrt(X^2 + Y^2),
     * seeded with the h = 0 solution. A fixed ten iterations is expected to be
     * ample for near-surface points (no convergence test); longitude is closed-form. */
    double target_latitude = atan2(earth_z, horizontal * (1 - eccentricity_squared));
    for (unsigned iteration = 0; iteration < 10; ++iteration) {
        const double target_sin = sin(target_latitude);
        const double target_radius = radius / sqrt(1 - eccentricity_squared * target_sin * target_sin);
        target_latitude = atan2(earth_z + eccentricity_squared * target_radius * target_sin, horizontal);
    }
    result.latitude_deg = target_latitude * 180 / pi;
    result.longitude_deg = atan2(earth_y, earth_x) * 180 / pi;
    /* Reported altitude is the assumed plane, not a height recovered from ECEF. */
    result.altitude_ellipsoid_m = pose->altitude_ellipsoid_m - pose->agl_m;
    if (!isfinite(result.latitude_deg) || !isfinite(result.longitude_deg) || !isfinite(result.slant_range_m)) return false;
    *point = result;
    return true;
}

