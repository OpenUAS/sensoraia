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

#ifndef SENSORAIA_LWIR_GEOLOCATION_H
#define SENSORAIA_LWIR_GEOLOCATION_H

/** @file
 * @brief Optional camera calibration and native pixel-to-ground geometry only.
 *
 * @details Compiled only in analysis builds (`LWIR_ENABLE_ANALYSIS=1`). Used by
 * lwir_analysis.c to turn hotspot centroids found in the native temperature plane
 * of a saved LWIR JPEG into estimated WGS84 positions. No detection, JPEG or EXIF
 * code lives here, so the geometry can be tested with synthetic poses.
 *
 * Frames (all right-handed):
 * - camera: x right, y down, z forward along the optical axis (OpenCV);
 * - body: x forward, y right, z down;
 * - local NED at the navigation reference: north, east, down.
 *
 * Model limits callers must respect:
 * - pinhole plus Brown-Conrady distortion; strong fisheye lenses are outside it;
 * - the ground is a horizontal plane `agl_m` below the navigation reference, so
 *   terrain relief and slope translate directly into horizontal error;
 * - pose quality is the caller's responsibility: a pose stamped at frame arrival
 *   rather than over the exposure interval can be displaced by aircraft motion,
 *   so projection results must be qualified with the pose timing evidence;
 * - an unverified calibration yields plausible-looking but unqualified numbers.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Projection calibration in native coordinates, never display coordinates.
 * @details Lens coefficients follow OpenCV's pinhole/Brown-Conrady convention.
 * Keeping them in the stored native pixel frame prevents display rotation or
 * preview enlargement from silently changing the ray being projected.
 */
struct lwir_camera_geometry {
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
    double camera_to_body[9]; ///< Row-major rotation: camera right/down/forward to body forward/right/down.
    double lever_arm_body_m[3]; ///< Navigation reference to camera origin, in body forward/right/down metres.
    double minimum_ground_cosine; ///< Minimum normalized downward ray component; rejects unstable horizon intersections.
    unsigned image_width, image_height;
    bool verified; ///< Operator verification claim; loading coefficients does not establish it.
};

/** @var lwir_camera_geometry::fx
 * @brief Horizontal focal length in native pixels; must be positive.
 */
/** @var lwir_camera_geometry::fy
 * @brief Vertical focal length in native pixels; must be positive.
 */
/** @var lwir_camera_geometry::cx
 * @brief Principal-point column in native pixel coordinates.
 */
/** @var lwir_camera_geometry::cy
 * @brief Principal-point row in native pixel coordinates.
 */
/** @var lwir_camera_geometry::k1
 * @brief Dimensionless radial coefficient multiplying normalized radius squared.
 */
/** @var lwir_camera_geometry::k2
 * @brief Dimensionless radial coefficient multiplying normalized radius to the fourth power.
 */
/** @var lwir_camera_geometry::p1
 * @brief First dimensionless tangential-distortion coefficient in OpenCV order.
 */
/** @var lwir_camera_geometry::p2
 * @brief Second dimensionless tangential-distortion coefficient in OpenCV order.
 */
/** @var lwir_camera_geometry::k3
 * @brief Dimensionless radial coefficient multiplying normalized radius to the sixth power.
 */
/** @var lwir_camera_geometry::image_width
 * @brief Native stored-image column count; resizing requires adjusted intrinsics.
 */
/** @var lwir_camera_geometry::image_height
 * @brief Native stored-image row count, before display orientation changes.
 */
/** @var lwir_camera_geometry::camera_to_body
 * @details Must be a proper rotation (orthonormal, determinant +1); projection
 * rejects anything else. The loader default `{0,-1,0, 1,0,0, 0,0,1}` maps camera
 * forward to body down and camera up (-y) to body forward: a nadir camera with
 * the image top toward the nose.
 */
/** @var lwir_camera_geometry::minimum_ground_cosine
 * @details Accepted range 0.05..1; 0.2 means rays more than about 78.5 degrees
 * from straight down are rejected.
 */

/** @brief Observation pose; geographic angles are degrees, attitude angles radians. */
struct lwir_camera_pose {
    double latitude_deg, longitude_deg, altitude_ellipsoid_m, agl_m;
    double roll_rad, pitch_rad, yaw_rad;
    double uncertainty_m; ///< Observation uncertainty retained for caller qualification, not applied by projection.
    bool synchronized; ///< Timing qualification retained for caller policy, not inferred by geometry.
};

/** @var lwir_camera_pose::latitude_deg
 * @brief Navigation reference latitude in WGS84 degrees.
 */
/** @var lwir_camera_pose::longitude_deg
 * @brief Navigation reference longitude in WGS84 degrees.
 */
/** @var lwir_camera_pose::altitude_ellipsoid_m
 * @brief Navigation reference height above the WGS84 ellipsoid in metres.
 * @details USHER's `alt_m` is the flight controller's LLA altitude, which is
 * ellipsoidal, not mean-sea-level; the ECEF conversion requires exactly this.
 */
/** @var lwir_camera_pose::agl_m
 * @brief Positive height of the navigation reference above the local ground plane.
 */
/** @var lwir_camera_pose::roll_rad
 * @brief Body roll in radians for the body-to-NED attitude rotation.
 */
/** @var lwir_camera_pose::pitch_rad
 * @brief Body pitch in radians for the body-to-NED attitude rotation.
 */
/** @var lwir_camera_pose::yaw_rad
 * @brief Body yaw in radians, measured from north toward east.
 */
/** @var lwir_camera_pose::uncertainty_m
 * @details Negative means unknown; lwir_analysis.c then reports the pose as unsynchronized.
 */
/** @var lwir_camera_pose::synchronized
 * @details True only when the pose was associated with an exposure-bounding time
 * interval. An arrival-time pose is not exposure-qualified; geometry still runs,
 * so callers must decide whether such results are usable.
 */

/** @brief WGS84 estimate on a local horizontal plane, not a surveyed accuracy claim. */
struct lwir_ground_point {
    double latitude_deg, longitude_deg, altitude_ellipsoid_m;
    double north_m, east_m, slant_range_m;
};

/** @var lwir_ground_point::latitude_deg
 * @brief Estimated ground-intersection latitude in WGS84 degrees.
 */
/** @var lwir_ground_point::longitude_deg
 * @brief Estimated ground-intersection longitude in WGS84 degrees.
 */
/** @var lwir_ground_point::altitude_ellipsoid_m
 * @brief Assumed ground-plane height above the WGS84 ellipsoid: observation height minus AGL.
 */
/** @var lwir_ground_point::north_m
 * @brief Northward ground-intersection offset from the navigation reference.
 */
/** @var lwir_ground_point::east_m
 * @brief Eastward ground-intersection offset from the navigation reference.
 */
/** @var lwir_ground_point::slant_range_m
 * @brief Camera-origin to ground-intersection distance along the projected ray.
 */

/** @brief Load bounded projection calibration, requiring exact native dimensions.
 * @param[in] path Nonempty regular calibration file, at most 8192 bytes.
 * @param width Expected native image width, within the supported LWIR bounds.
 * @param height Expected native image height, within the supported LWIR bounds.
 * @param[out] geometry Result, unchanged on failure.
 * @return True for structurally valid calibration, not proof of physical accuracy.
 * @details This is a bounded profile reader, not a general YAML implementation.
 * Each scalar or bracketed numeric array must fit on one line. Required fields
 * include all lens coefficients, dimensions, verification flag and the nine-value
 * rotation; duplicate known fields fail. Optional lever arm defaults to zero and
 * minimum_ground_cosine to 0.2. Unknown keys are ignored.
 * @note An unverified profile is intentionally loadable for workshop checks.
 * Loading does not validate rotation orthonormality; projection checks it before
 * use. Neither operation establishes that a mount was physically measured.
 * @note Lines longer than 511 characters, a missing ':' on a non-comment line,
 * `verified` other than 0/1, nonpositive focal lengths, or a
 * minimum_ground_cosine outside 0.05..1 all fail. Lines starting with '#',
 * '%' or consisting of `---` are skipped. errno is not part of the failure contract.
 */
bool lwir_geolocation_load_calibration(const char *path, unsigned width, unsigned height,
                                       struct lwir_camera_geometry *geometry);
/** @brief Project a native subpixel location using distortion, mount and observation pose.
 * @param pixel_x Native column, independent of JPEG rotation.
 * @param pixel_y Native row.
 * @param[in] camera Intrinsics and proper camera-to-body rotation.
 * @param[in] pose Observation position, attitude and positive height above the ground plane.
 * @param[out] point Result, unchanged on failure.
 * @return False for invalid/nonfinite geometry, nonconvergence or near-horizon rays.
 * @details Inverse distortion is solved in camera coordinates before mount and
 * attitude rotations. A finite iteration budget rejects nonconvergent rays, and
 * the downward-cosine guard avoids arbitrarily long intersections near the
 * horizon. The intersection uses a local horizontal plane, not a terrain model.
 * The local north/east offset is converted to latitude/longitude on the WGS84
 * ellipsoid via ECEF, so no small-angle flat-earth approximation is involved.
 * @warning The caller must enforce verification, observation synchronization and
 * uncertainty policy; this geometry function does not gate on those flags.
 */
bool lwir_geolocation_project(double pixel_x, double pixel_y,
                              const struct lwir_camera_geometry *geometry,
                              const struct lwir_camera_pose *pose,
                              struct lwir_ground_point *point);
#ifdef __cplusplus
}
#endif

#endif