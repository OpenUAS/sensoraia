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

#ifndef USHER_COVERAGE_SCHEDULER_H
#define USHER_COVERAGE_SCHEDULER_H

/**
 * @file coverage_scheduler.h
 * @brief Fixed-memory geometric coverage accounting, independent of camera I/O.
 * @details This layer compares footprints rather than decoding photographs. Keep
 * predicted observations separate from committed coverage: only the capture owner
 * knows whether an original and its metadata were successfully published.
 * @note Functions do not lock. Serialize access to a shared state at the caller.
 * Invalid geometry means "unknown", never "already photographed".
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define COVERAGE_SIDE 1024       /**< Cells per axis; memory does not grow with flight duration. */
#define COVERAGE_CAMERAS 3       /**< Image-camera IDs 1..3; EARcam audio is not image coverage. */
#define COVERAGE_CANDIDATES 4    /**< Bounded lookahead set, not a queue of retained image pixels. */
#define COVERAGE_SCAN_LIMIT 4096 /**< Maximum cells per footprint scan, limiting decision work. */

/** @brief Measured pinhole calibration for the delivered image, including its mount. */
struct coverage_camera {
  unsigned width, height; /**< Delivered dimensions in pixels, after any camera crop or resize. */
  double fx, fy, cx, cy; /**< Focal lengths and principal point in delivered-image pixels. */
  double mount_roll_deg, mount_pitch_deg, mount_yaw_deg; /**< Camera-to-body rotation, degrees, Rz*Ry*Rx. */
};
/** @var coverage_camera::width
 * @brief Delivered width in pixels; must agree with the saved image, not the sensor mode alone. */
/** @var coverage_camera::fx
 * @brief Horizontal focal length in delivered-image pixels. */
/** @var coverage_camera::fy
 * @brief Vertical focal length in delivered-image pixels. */
/** @var coverage_camera::cx
 * @brief Principal-point column in delivered-image pixels. */
/** @var coverage_camera::mount_roll_deg
 * @brief Camera-to-body roll in degrees; mounting is applied before aircraft attitude. */
/** @var coverage_camera::mount_pitch_deg
 * @brief Camera-to-body pitch in degrees, using the same convention as PANGEA. */
/** @brief Pose estimate and its positional margin for a single observation. */
struct coverage_pose {
  double latitude_deg, longitude_deg, agl_m; /**< Geographic degrees and height above the local ground plane. */
  double roll_deg, pitch_deg, yaw_deg; /**< Body attitude in degrees using the USHER/PANGEA convention. */
  double uncertainty_m; /**< Nonnegative ground-position margin; not a statistical confidence level. */
};
/** @var coverage_pose::latitude_deg
 * @brief Observation latitude in geographic degrees, not FC fixed-point units. */
/** @var coverage_pose::longitude_deg
 * @brief Observation longitude in geographic degrees. */
/** @var coverage_pose::roll_deg
 * @brief Aircraft roll in degrees. */
/** @var coverage_pose::pitch_deg
 * @brief Aircraft pitch in degrees. */
/** @brief Local ground coordinates in metres, east and north of the projection origin. */
struct coverage_point { double east, north; };
/** @var coverage_point::east
 * @brief Metres east of the shared local geographic reference. */
/** @var coverage_point::north
 * @brief Metres north of the shared local geographic reference. */
/** @brief Convex ground quadrilateral with a conservative boundary margin. */
struct coverage_footprint {
  struct coverage_point corner[4]; /**< Cyclic vertices, either winding; never a crossed polygon. */
  double uncertainty_m; /**< Metres to exclude from credit and include when testing redundancy. */
};
/** @brief One feasible observation; deadlines constrain acquisition start, not saving. */
struct coverage_candidate {
  struct coverage_footprint footprint; /**< Predicted or saved footprint, according to the caller's operation. */
  uint64_t ready_us, deadline_us; /**< Earliest/latest start in the caller's monotonic microsecond domain. */
  unsigned camera_id; /**< One-based image-camera identity; coverage is never shared between sensors. */
  uint8_t quality; /**< Positive comparable detail score, 1..255; not an object-detection probability. */
};
/** @var coverage_candidate::ready_us
 * @brief Earliest acquisition start; all candidates must share a monotonic time domain. */
/** @brief Fixed coverage ledger; zero cells are unknown, not necessarily unphotographed. */
struct coverage_state {
  double origin_east, origin_north, cell_m; /**< Grid lower corner and cell edge, all in metres. */
  uint8_t saved[COVERAGE_CAMERAS][COVERAGE_SIDE * COVERAGE_SIDE]; /**< Best committed detail per camera and cell. */
};
/** @var coverage_state::origin_east
 * @brief Lower east grid edge in metres relative to the projection origin. */
/** @var coverage_state::origin_north
 * @brief Lower north grid edge in metres relative to the projection origin. */

#ifdef __cplusplus
extern "C" {
#endif
/** @brief Project delivered-image corners onto a local flat ground plane.
 * @param camera Measured intrinsics and camera-to-body mount.
 * @param pose Observation pose with AGL and uncertainty.
 * @param origin_latitude_deg Local projection reference latitude, degrees.
 * @param origin_longitude_deg Local projection reference longitude, degrees.
 * @param[out] footprint Result, written only on success.
 * @return False for invalid, near-horizon or excessively distant geometry.
 * @warning A plausible footprint does not establish terrain visibility, focus or
 * exposure synchronization. Zero mount means image-up forward and optical-axis down.
 */
bool coverage_project(const struct coverage_camera *camera, const struct coverage_pose *pose,
                      double origin_latitude_deg, double origin_longitude_deg, struct coverage_footprint *footprint);
/** @brief Project an image pixel onto the same ground plane as the coverage footprint.
 * @param camera Delivered-image calibration and mount.
 * @param pose Observation attitude and camera-to-ground height.
 * @param column Pixel column within the delivered image.
 * @param row Pixel row within the delivered image.
 * @param[out] offset Metres east/north relative to the observation position.
 * @return False for invalid pixels or geometry, leaving offset unchanged.
 * @details Reuses whole-footprint qualification and the existing mount/body rotation order. */
bool coverage_project_pixel(const struct coverage_camera *camera, const struct coverage_pose *pose,
                            double column, double row, struct coverage_point *offset);
/** @brief Clear the ledger and set its fixed spatial domain.
 * @param[out] state Caller-owned ledger; allocate off small embedded thread stacks.
 * @param east Lower grid edge in local east metres.
 * @param north Lower grid edge in local north metres.
 * @param cell_m Cell edge in metres, within 0.25..100.
 * @return True on initialization; false leaves state unchanged.
 */
bool coverage_init(struct coverage_state *state, double east, double north, double cell_m);
/** @brief Score additional detail in conservatively contained cells without mutation.
 * @param state Initialized committed ledger.
 * @param candidate Observation to evaluate.
 * @return Quality-weighted square metres, zero for no credited gain, or -1 for
 * unknown geometry. Zero alone is not permission to skip; use coverage_wholly_saved().
 */
double coverage_gain(const struct coverage_state *state, const struct coverage_candidate *candidate);
/** @brief Credit a successfully saved observation, retaining the best detail per cell.
 * @param[in,out] state Initialized committed ledger.
 * @param candidate Saved image footprint with trusted pose and detail score.
 * @return True for a valid bounded update, even if no cells improve; false for unknown geometry.
 * @pre The caller has verified publication. Never call this for a mere capture request.
 */
bool coverage_commit(struct coverage_state *state, const struct coverage_candidate *candidate);
/** @brief Check whether skipping this observation is geometrically conservative.
 * @param state Initialized committed ledger.
 * @param candidate Proposed observation.
 * @return True only when the uncertainty-expanded bounding region already has at
 * least the requested detail. Outside the grid or work budget, return false.
 */
bool coverage_wholly_saved(const struct coverage_state *state, const struct coverage_candidate *candidate);
/** @brief Estimate conservative overlap for deciding whether a delay is acceptable.
 * @param state Grid geometry used to discretize the comparison; saved values are not read.
 * @param first Candidate whose bounding-cell count is the denominator.
 * @param second Same-camera footprint to overlap with.
 * @return Conservative fraction in 0..1, or -1 if the comparison is unknown.
 * @note This is asymmetric and deliberately stricter than exact polygon overlap.
 */
double coverage_overlap(const struct coverage_state *state, const struct coverage_candidate *first,
                        const struct coverage_candidate *second);
/** @brief Choose the first observation of the best feasible one- or two-shot plan.
 * @param state Committed coverage, left unchanged.
 * @param candidates At most COVERAGE_CANDIDATES opportunities.
 * @param count Number of opportunities.
 * @param now_us Earliest permissible start, monotonic microseconds.
 * @param service_us Positive serialized acquisition interval used between choices.
 * @return First candidate index, or -1 when no positive feasible gain is known.
 * @details Union scoring avoids double-counting repeated ground in the second
 * view. Ties favor an earlier start, then greater first-shot gain. The caller still
 * enforces overlap, uncertainty and delay limits; this is not a global flight planner.
 */
int coverage_choose(const struct coverage_state *state, const struct coverage_candidate *candidates,
                    size_t count, uint64_t now_us, uint64_t service_us);
#ifdef __cplusplus
}
#endif

#endif