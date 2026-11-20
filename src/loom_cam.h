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
 * @file loom_cam.h
 * @brief Bounded asynchronous saved-photo adapter for LOOM_cam.
 * @details USHER's capture owner supplies immutable, EXIF-tagged AIcam photos.
 * This adapter records outbound references or runs a saved-photo child; it does
 * not own the sensor, transmit UART or mutate waypoints. One pending request and
 * one completion mailbox prioritize freshness over processing every photograph.
 * @details Each START creates a unique reference subdirectory, retaining only
 * that sortie's links for its return phase. A per-sortie cap bounds directory
 * growth without pruning evidence. Phase tokens select collection/return modes;
 * FC photo numbers correlate individual results. Keeping those identities separate prevents old mode work
 * from looking like a fresh candidate route point after START or STOP.
 * @warning Accepted points require a declared return model and validated observation evidence.
 */
#ifndef USHER_LOOM_CAM_H
#define USHER_LOOM_CAM_H

#include "camera_backend.h"

/** @brief LOOM photographs and outbound references use 'o', not AIcam's 'a'. */
#define LOOM_IMAGE_PREFIX 'o'

/**
 * @brief Start one saved-photo worker; does not acquire another camera.
 * @param references Optional outbound-photo directory, copied before return.
 * @param model Optional native capture RPK path, copied before return.
 * @return Zero on startup, -1 for invalid paths or worker setup failure.
 * @details Starts in return mode; only the configured current-photo model vector
 * is interpreted. Outbound references are retained but are not yet compared.
 * NULL paths select empty configuration; overlong paths are rejected, not truncated.
 * @pre Lifecycle calls are serialized by the owning USHER control thread.
 */
int loom_cam_init(const char *references, const char *model);
/** @brief Select executable settings before starting the saved-photo worker.
 * @param settings Copied path or NULL to restore executable discovery.
 * @return Zero when accepted, minus one while running or for an invalid path. */
int loom_cam_set_settings(const char *settings);
/**
 * @brief Report whether this optional stream is running and not stopping.
 * @return True when phase callbacks may be admitted; false otherwise.
 * @note Uses atomic lifecycle flags; no camera acquisition or mutex wait occurs.
 */
bool loom_cam_enabled(void);
/** @brief Borrow the nonblocking completion descriptor; minus one when not initialized. */
int loom_cam_completion_fd(void);
/**
 * @brief Select outbound reference collection and discard old-phase mailbox work.
 * @param session_id Admitted nonzero FC session token, not a photo number.
 * @return True if accepted by a running worker; false for zero or shutdown.
 * @details Advances a phase generation to suppress in-flight old-phase results.
 * The original photo archive remains authoritative if reference work is dropped.
 */
bool loom_cam_start(uint32_t session_id);
/**
 * @brief Select return matching and discard old-phase mailbox work.
 * @param session_id Nonzero token matching the most recently admitted START.
 * @return True if the token is accepted; false if absent, mismatched or stopping.
 * @note Acceptance changes local mode; it does not prove a neural match exists.
 */
bool loom_cam_stop(uint32_t session_id);
/**
 * @brief Copy the newest completed AIcam photo request into bounded worker storage.
 * @param filename Immutable saved JPEG path, borrowed only for this call.
 * @param shot Completed photo pose/request; its positive number correlates the update.
 * @details Thread-safe and nonblocking; pending work is fresh-first, not an evidence archive.
 * Busy admission, invalid input, overlong paths and duplicate/reversed photo
 * numbers are dropped. The file itself must remain available to the worker.
 */
void loom_cam_image_saved(const char *filename, const union dc_shot_union *shot);
/**
 * @brief Consume a result on the serial owner; session_id carries the photo number for this stream.
 * @param result Required destination in existing point units.
 * @return True for a new matched or no-match result; false when absent/busy.
 * @details Consumes the single mailbox entry. True means a completion exists,
 * not that result->valid is true; FC navigation checks must still be applied.
 */
bool loom_cam_poll_result(struct camera_point_result *result);
/**
 * @brief Cancel pending work and join the worker before reusing adapter state.
 * @details Wakes an idle worker or causes its child watchdog to terminate/reap
 * the process. Unlike admission/polling, shutdown waits for the worker to exit.
 * @pre Lifecycle calls are serialized; do not call from the worker itself.
 */
void loom_cam_deinit(void);

#endif