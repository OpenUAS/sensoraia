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
 * @file ai_cam_pipe.h
 * @brief Parent-side contract for stock and native Raspberry Pi camera capture.
 * @details Both modes publish private JPEGs and preserve no-replay behavior;
 * only the native worker provides USHER sensor timing evidence.
 */
#ifndef USHER_AI_CAM_PIPE_H
#define USHER_AI_CAM_PIPE_H

#include <stddef.h>
#include <stdbool.h>

struct camera_backend_host;

/** @brief Select native or system-camera capture before startup.
 * @param enabled True for native photo-only capture.
 * @return Zero on success, or -1 while either mode has an owned child.
 * @details Refuse to switch modes without releasing the previous owner. */
int ai_cam_pipe_set_native(bool enabled);
/** @brief Report whether the selected AIcam mode uses native photo-only capture.
 * @return True for native mode.
 * @details Reports configuration, not whether the camera has started. */
bool ai_cam_pipe_native(void);
/** @brief Pass the shutdown flag to native worker supervision.
 * @param host Nullable backend host; its running flag must outlive the worker.
 * @details Retain only the borrowed flag, not the host structure.
 * A null host clears the flag. */
void ai_cam_pipe_configure(const struct camera_backend_host *host);

/**
 * @brief Validate storage and start the selected camera owner.
 * @param[in] unused Reserved backend argument.
 * @return Zero for usable storage, otherwise -1.
 * @details All operations require the owning backend's serialization. Native
 * startup waits for a valid frame; the system-camera path checks its child per shot. */
int ai_cam_pipe_init(const char *unused);

/**
 * @brief Select a borrowed directory before initialization.
 * @param[in] directory Caller-owned path, or NULL for the compiled default.
 * @details Retain the pointer, not a copy; keep the path valid through capture.
 */
void ai_cam_pipe_set_photo_directory(const char *directory);

/**
 * @brief Capture one ordinary JPEG into private staging for metadata completion.
 * @param[out] filename Receives the .pending path, empty on failure.
 * @param[in] filename_size Buffer capacity including NUL.
 * @param[in] image_number USHER shot number.
 * @return Zero for a nonempty regular image, otherwise -1.
 * @details A failure after triggering never replays the exposure. The system
 * camera path does not supply Sensoraia's sensor evidence. The caller completes
 * metadata before exposing the final filename.
 */
int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);

/** @brief Stop the native and system-camera subprocesses.
 * @details Attempt bounded shutdown; an unreaped system-camera child retains
 * ownership rather than allowing a second process to open the camera. */
void ai_cam_pipe_deinit(void);

#endif
