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
 * @file ear_cam_pipe.h
 * @brief Audio/pose adapter and zero-copy snapshot API for the EAR backend.
 * @details Configuration and active-session functions belong to USHER's serial
 * owner. Only snapshot solve/number/render/release are intended for the finalizer.
 * Configure before initialization; drain detached workers before deinitialization.
 */
#ifndef USHER_EAR_CAM_PIPE_H
#define USHER_EAR_CAM_PIPE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usher_protocol.h"
#include "capture_timing.h"
#include "modules/ear_cam/earcam_core.h"

/**
 * @brief Set the startup-only pose callback, later called without waiting on the serial owner.
 * @param lookup Borrowed callback with host lifetime; NULL selects request-pose fallback.
 * @details Never invoked by the audio reader or finalizer; shared navigation state stays owner-local.
 */
void ear_cam_pipe_set_pose_lookup(bool (*lookup)(struct capture_observation *, uint64_t));
/**
 * @brief Copy a startup ALSA PCM name into bounded adapter-owned storage.
 * @param device Nonempty name; must fit including its terminating NUL.
 * @return True when copied; false for NULL/empty/oversized input or running capture threads.
 * @details No shell is involved. Every child retry receives the same owned name.
 */
bool ear_cam_pipe_set_device(const char *device);
/** @brief Copy an assignment path before the microphone supervisor starts.
 * @param settings Nonempty path forwarded to every server retry without shell expansion.
 * @return True for a complete bounded copy; false while supervision is active or input is invalid. */
bool ear_cam_pipe_set_settings(const char *settings);
/** @brief Retry bounded pending audio windows when the serial owner receives new poses. */
void ear_cam_pipe_update_pose(void);

struct ear_session_snapshot;
struct ear_loudest_spot;

/**
 * @brief Begin/reset active collection without solving or waiting for log I/O.
 * @param session_id Token retained in audit rows; backend admission validates it before this call.
 * @return False when no active or free buffer exists; true after resetting active counts.
 * @details Does not evict worker-owned snapshots. The active buffer is serial-owner-only.
 */
bool ear_cam_pipe_begin_session(uint32_t session_id);
/**
 * @brief Best-effort enqueue of a correlated result row on the serial owner.
 * @param session_id Token identifying the completed snapshot, not necessarily the current session.
 * @param result Borrowed estimate, copied into a row before return; NULL is ignored.
 * @details Only the log worker accesses its FILE. A dropped audit row is not a lost point completion.
 */
void ear_cam_pipe_log_result(uint32_t session_id, const struct ear_loudest_spot *result);
/**
 * @brief Transfer active storage to an immutable worker snapshot without copying samples.
 * @return Owned snapshot, or NULL if no active buffer exists.
 * @details Serial-owner-only. Pending pose windows are resolved/fallback-committed
 * before freezing count; active collection stays disabled until the next successful begin.
 * @warning Caller must eventually release exactly once, after all readers stop using it.
 */
struct ear_session_snapshot *ear_cam_pipe_detach_session(void);
/**
 * @brief Solve immutable detached evidence without touching current-session state.
 * @param snapshot Borrowed owned snapshot, which must remain unreleased throughout this call.
 * @param result Required estimate destination, cleared before invoking the core.
 * @return Core solver status: 0 for an estimate, -1 for invalid/insufficient evidence or arguments.
 * @details Worker-safe; shared core workspace serializes concurrent solves. No ownership transfer.
 */
int ear_cam_pipe_snapshot_solve(const struct ear_session_snapshot *snapshot, struct ear_loudest_spot *result);
/**
 * @brief Read the last request number from immutable evidence.
 * @param snapshot Borrowed unreleased snapshot, or NULL.
 * @return Newest stored request number, or zero for NULL/empty evidence.
 */
int32_t ear_cam_pipe_snapshot_number(const struct ear_session_snapshot *snapshot);
/**
 * @brief Render detached evidence using collision-safe session-tagged JPEG/CSV names.
 * @param snapshot Borrowed owned snapshot; remains immutable and unreleased during I/O.
 * @param session_id Required nonzero correlation token for file naming and CSV provenance.
 * @param result Required borrowed estimate; an invalid estimate may still produce evidence.
 * @param filename Destination path buffer; cleared by the renderer on handled creation failure.
 * @param filename_size Capacity of filename including its terminating NUL.
 * @param simulated Select the simulated-image prefix, not a different rendering algorithm.
 * @return 0 for decorated JPEG success, -1 for invalid arguments or primary rendering failure.
 * @details Worker-safe. CSV, undecorated field and .geo errors are reported separately;
 * JPEG success does not guarantee all evidence products or power-loss durability.
 */
int ear_cam_pipe_snapshot_render(const struct ear_session_snapshot *snapshot, uint32_t session_id,
                                 const struct ear_loudest_spot *result, char *filename,
                                 size_t filename_size, bool simulated);
/**
 * @brief Return detached storage to admission after all snapshot access has ended.
 * @param snapshot Previously detached owned snapshot; NULL is a no-op.
 * @warning The pointer and sample contents must not be used after release, even
 * if the backing array still exists. A new serial-owner session may overwrite it.
 */
void ear_cam_pipe_snapshot_release(struct ear_session_snapshot *snapshot);

/**
 * @brief Start EARcam sampling or its deterministic simulated-source replacement.
 * @param unused Reserved for the common camera-backend initialization signature; ignored.
 * @return 0 when supervision is started (not hardware readiness); -1 for setup failure.
 * @details A missing microphone is deliberately not fatal. The optical pipeline can continue
 * operating while an independent supervisor retries the server. Configure the
 * simulated source and frequency band before calling this function.
 */
int ear_cam_pipe_init(const char *unused);

/**
 * @brief Stop EARcam activity and release its process and descriptors.
 * @details Safe after a partially successful initialization. Call this after the final render
 * or solve because it discards unfinalized session samples.
 */
void ear_cam_pipe_deinit(void);

/**
 * @brief Report readiness of the configured live or simulated measurement source.
 * @return True after the live handshake or simulated initialization; false after teardown/lost readiness.
 * @details Ready does not guarantee a fresh sample or prove live hardware is present;
 * simulated mode intentionally has no microphone process and also reports ready.
 */
bool ear_cam_pipe_ready(void);

/**
 * @brief Replace microphone input with a virtual fixed loudspeaker for NPS or desk tests.
 * @param lat_deg Source latitude in degrees.
 * @param lon_deg Source longitude in degrees.
 * @param level_db_at_1m Sound level at one metre in decibels.
 * @details Must be called before @c ear_cam_pipe_init(). Simulation avoids ALSA and the
 * external EARcam process while preserving the same geotagging and fusion interface.
 */
void ear_cam_pipe_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m);

/**
 * @brief Set the live EARcam tone-search band.
 * @param low_cut_hz Lower cutoff in hertz, or zero to retain EARcam's default.
 * @param high_cut_hz Upper cutoff in hertz, or zero to retain EARcam's default.
 * @details Must be called before @c ear_cam_pipe_init() because the values become arguments
 * of the persistent external process. Invalid ordering is rejected by the caller's CLI path.
 */
void ear_cam_pipe_set_band(double low_cut_hz, double high_cut_hz);

/**
 * @brief Associate the newest sound window with one flight-controller shot pose.
 * @param shot Flight pose and shot number received from USHER transport.
 * @return 0 when evidence is committed or admitted to the bounded pose queue;
 * -1 for invalid input, unavailable/stale data or duplicate timed windows.
 * @details Timed windows await pose evidence in a fixed queue for at most 200 ms;
 * unresolved or legacy measurements retain their labelled request-pose fallback.
 * Only the newest live measurement is read at each request, not every audio window.
 * Capacity pressure retains the newest half of active samples, never a worker snapshot.
 * Server recovery and CSV I/O run independently. This call never waits for them.
 */
int ear_cam_pipe_record(const union dc_shot_union *shot);

/**
 * @brief Fuse the session into its final loudest-spot result and clear stored samples.
 * @param result Destination for the solved or invalid result.
 * @return Core solver status: 0 for an estimate, -1 for insufficient evidence or invalid arguments.
 * @details Finalization consumes the session so the next survey cannot accidentally mix old
 * and new flight legs. This legacy synchronous call solves committed samples and
 * joins the log writer; it is not suitable for a latency-sensitive UART callback.
 * Resolve pending poses first (ear_cam_pipe_solve() forces their fallback), and render
 * before clearing samples. Prefer detach/solve/render/release for asynchronous sessions.
 */
int ear_cam_pipe_finish(struct ear_loudest_spot *result);

/**
 * @brief Solve the accumulated session without clearing it.
 * @param result Destination for the current estimate.
 * @return Core solver status: 0 for an estimate, -1 for insufficient evidence or invalid arguments.
 * @details Use for refinement decisions. Unlike @c ear_cam_pipe_finish(), subsequent records
 * remain part of the same survey session. Serial-owner-only; pending poses are
 * forced to their bounded fallback before this synchronous solve.
 */
int ear_cam_pipe_solve(struct ear_loudest_spot *result);

/**
 * @brief Render the current session as a north-up acoustic evidence JPEG.
 * @param result Solved loudest-spot estimate to mark on the image.
 * @param filename Destination buffer receiving the JPEG path.
 * @param filename_size Capacity of @p filename.
 * @param simulated True when rendering NPS/desk simulated data.
 * @return 0 on success; -1 if inputs, session data, or output creation fail.
 * @details Must be called before @c ear_cam_pipe_finish() clears the session. The JPEG and
 * sidecar preserve the sampled field and final estimate for post-flight review.
 */
int ear_cam_pipe_render(const struct ear_loudest_spot *result, char *filename, size_t filename_size,
                        bool simulated);

/**
 * @brief Get the shot number associated with the newest stored acoustic sample.
 * @return The newest shot number, or 0 when the session is empty.
 */
int32_t ear_cam_pipe_last_shot_nr(void);

#endif
