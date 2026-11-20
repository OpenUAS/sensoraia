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

#ifndef USHER_CAPTURE_ARBITER_H
#define USHER_CAPTURE_ARBITER_H

/**
 * @file capture_arbiter.h
 * @brief Session-scoped survey admission and saved-image evidence for USHER.
 * @details The serial owner offers intentions and takes ready work; camera workers
 * return acquisition and publication results. Public state access is mutex-protected.
 * The implementation is C11 with pthread locking and explicit resource cleanup;
 * the C linkage guards also permit existing C++ callers and regression fixtures.
 * Configure profiles before starting workers. No function here operates a camera,
 * changes FC distance rules, or assumes that a request contains recoverable pixels.
 * @note Times are MORA CLOCK_MONOTONIC microseconds unless explicitly stated.
 * Queues and coverage are session memory, not a restart-persistent mission database.
 */

#include "usher_protocol.h"
#include "coverage_scheduler.h"
#include "capture_timing.h"

#ifdef __cplusplus
extern "C" {
#endif
/** @brief Load one camera's measured profile without guessing geometry from its ID.
 * @param filename Bounded PANGEA_CAMERA_V1 flat key=value file; prefix c/a/l identifies the sensor.
 * @return True on acceptance; false with a diagnostic on invalid input or allocation failure.
 * @details The first valid profile allocates the fixed ledger and random peer session.
 * Replacing calibration is a startup operation, not a way to reinterpret existing coverage.
 * Input is a nonempty regular file of at most 1 KiB containing one key=value
 * per line. Blank lines and # comments are allowed; duplicate/unknown keys and
 * invalid decimal fields are rejected without allocation.
 * Invalid input leaves existing calibration and session state intact.
 */
bool capture_arbiter_load_camera(const char *filename);
/** @brief Enable coverage-based suppression and limited delay, or observe decisions only.
 * @param enforce True permits conservative enforcement; false is the default.
 * @note Observation still uses bounded admission and expiry, not the legacy queue policy.
 */
void capture_arbiter_set_enforce(bool enforce);
/** @brief Set the overlap floor for deliberate predictive delay, not FC shot spacing.
 * @param overlap Fraction within 0..0.9.
 * @return False for invalid input, leaving the previous setting intact.
 */
bool capture_arbiter_set_overlap(double overlap);
/** @brief Set admission from a backend's actual capacity, independent of camera ID.
 * @param camera_id One-based image-camera identity.
 * @param capacity Maximum active captures, 1..2.
 * @return False for invalid input. Lowering capacity does not cancel active captures.
 */
bool capture_arbiter_set_camera_capacity(unsigned camera_id, unsigned capacity);
/** @brief Return the peer-session token, or zero when survey mode is not configured.
 * @return Random nonzero token after successful profile initialization; not an authentication key.
 */
uint32_t capture_arbiter_session_id(void);
/** @brief Refresh survey authorization or cancel waiting work when scope is withdrawn.
 * @param scope Validated wire structure; wrong sessions and malformed values are ignored.
 * @param now_us Local receipt time, not a remote FC timestamp.
 * @details A stop or removed mask bit affects waiting intentions only. An acquired
 * original still deserves to be saved even when the survey has just ended.
 */
void capture_arbiter_set_scope(const union usher_capture_arbiter_state_union *scope, uint64_t now_us);
/** @brief Split a survey request into bounded per-camera intentions.
 * @param request Session-tagged shot and camera mask from the FC.
 * @param now_us Receipt time used for intent age and scope freshness.
 * @return True when handled, including expiry or no image-camera bits; not a saved-photo ACK.
 * False indicates an invalid request/session. Overflow replaces only the same camera's
 * oldest waiting record, so another sensor's backlog cannot be evicted by this one.
 */
bool capture_arbiter_offer(const union usher_capture_arbiter_shoot_union *request, uint64_t now_us);
/** @brief Admit the oldest ready camera intention after optional coverage evaluation.
 * @param ready_mask Backend readiness snapshot; active-capacity limits are also enforced here.
 * @param now_us Current local time for expiry and short-horizon prediction.
 * @param[out] shot Original FC request, not the predicted future exposure pose.
 * @param[out] mask Image camera admitted for this shot.
 * @return True when the caller must dispatch work and eventually call capture_arbiter_complete().
 * False means no admission; expiry or redundancy results may still have been queued.
 */
bool capture_arbiter_take(uint32_t ready_mask, uint64_t now_us, union dc_shot_union *shot, uint8_t *mask);
/** @brief Release admission and optionally credit verified saved coverage.
 * @param shot Matching request number, with exposure pose when valid.
 * @param camera_id Camera completing this observation.
 * @param saved Original and required metadata were successfully published.
 * @param exposure_pose_valid Pose is backed by frame timing, not just trigger time.
 * @param uncertainty_m Nonnegative position margin associated with that pose.
 * @param now_us Completion time; used for fallback service-time estimation.
 * @details Call on startup and metadata failures too, or that camera remains busy.
 * Unknown pose still permits a SAVED result but never trusted coverage credit.
 */
void capture_arbiter_complete(const union dc_shot_union *shot, unsigned camera_id, bool saved,
                              bool exposure_pose_valid, double uncertainty_m, uint64_t now_us);
/** @brief Pop a bounded best-effort result for UART transmission.
 * @param[out] result Result whose storage is independent of the internal queue.
 * @param ready_mask Current readiness snapshot to attach.
 * @return True if a record was popped; false when empty or result is NULL.
 * @note Retain the popped record locally if UART transmission must be retried.
 */
bool capture_arbiter_result(union usher_capture_arbiter_result_union *result, uint32_t ready_mask);
/** @brief Cancel waiting intentions, retaining active-image ownership and saved coverage. */
void capture_arbiter_stop(void);
/** @brief Check whether waiting or active survey work remains.
 * @return A synchronized snapshot, not a wait primitive or a readiness guarantee.
 */
bool capture_arbiter_pending(void);
/** @brief Identify a shot whose active survey ownership must not be superseded.
 * @param number FC shot number to look up across image cameras.
 * @return True if any active camera owns this number; not evidence of publication.
 */
bool capture_arbiter_owns(int32_t number);
/** @brief Learn acquisition latency separately from slower JPEG publication.
 * @param number Original FC shot number.
 * @param camera_id Backend that now owns the selected pixels.
 * @param now_us Acquisition-complete time in MORA's monotonic domain.
 * @details The first plausible receipt updates service time; publication still
 * owes capture_arbiter_complete() and no coverage is credited at this point.
 */
void capture_arbiter_acquired(int32_t number, unsigned camera_id, uint64_t now_us);
/** @brief Add a pose only after the caller has established bounded clock alignment.
 * @param sample FC pose/GPS evidence, including raw fixed-point shot fields.
 * @param earliest_us Earliest possible local sample time.
 * @param latest_us Latest possible local sample time.
 * @details Invalid evidence clears interpolation history rather than bridging a
 * clock discontinuity with seemingly precise coordinates.
 */
void capture_arbiter_pose_sample(const union usher_pose_sample_union *sample, uint64_t earliest_us, uint64_t latest_us);
/** @brief Forget pose history when synchronization is lost; keep saved coverage. */
void capture_arbiter_pose_invalidate(void);
/** @brief Interpolate within trusted nearby pose samples, never across a long gap.
 * @param time_us Desired observation time in MORA's monotonic domain.
 * @param time_error_us Observation-time uncertainty in microseconds.
 * @param[out] shot Interpolated raw FC pose; caller must restore the observation's shot number.
 * @param[out] uncertainty_m Conservative ground-position margin in metres.
 * @return True when all timing, motion and uncertainty checks pass. Ignore outputs on false.
 */
bool capture_arbiter_pose_at(uint64_t time_us, uint64_t time_error_us, union dc_shot_union *shot,
                             double *uncertainty_m);
/** @brief Resolve one bounded observation; wait_us is 0 on the serial thread, at most 200000 in workers. */
bool capture_arbiter_observation_pose(struct capture_observation *observation, uint64_t wait_us);
/** @brief Read embedded capture identity from a stable JPEG; true need not qualify timing.
 * @details frame_bound may be true while kind remains UNKNOWN. Profiles are optional;
 * when configured their dimensions must match. Call before the EXIF inode replacement. */
bool capture_arbiter_frame_timing(const char *filename, unsigned camera_id, struct capture_observation *observation);
/** @brief Associate embedded exposure evidence with this JPEG before geotagging.
 * @param filename Published JPEG path, exclusively owned during inspection.
 * @param camera_id Sensor identity; configured profile dimensions must match the JPEG and record.
 * @param request Original request; its number is retained in the returned exposure record.
 * @param[out] exposure Pose estimate at the frame's bounded observation time.
 * @param[out] uncertainty_m Ground-position margin carried into coverage accounting.
 * @return False when evidence is missing, stale, inconsistent or outside supported bounds.
 * @warning Identity checks prevent accidental mismatches, not hostile file replacement.
 * Do not recycle or edit capture paths concurrently. Failure must not discard the JPEG.
 */
bool capture_arbiter_frame_pose(const char *filename, unsigned camera_id, const union dc_shot_union *request,
                                union dc_shot_union *exposure, double *uncertainty_m);
#ifdef __cplusplus
}
#endif

#endif