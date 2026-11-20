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

/** @file dipla_camera_backend.h
 * @brief Optional non-image camera adapter for the native Diplacam worker.
 * @details This boundary owns a local worker process and reply correlation.
 * It does not run inference, move FC waypoints or select mission blocks. */
#ifndef SENSORAIA_DIPLA_CAMERA_BACKEND_H
#define SENSORAIA_DIPLA_CAMERA_BACKEND_H
#include "camera_backend.h"
/** @brief Select the worker experiment before acquiring its process.
 * @param configuration Nonempty worker-INI path, copied into bounded storage.
 * @return False for an invalid/overlong path or an already acquired channel.
 * @details Copying prevents an argv buffer from becoming a dangling configuration
 * pointer. Setup does not open the field, initialize ONNX or acquire a sensor. */
bool dipla_camera_backend_setup(const char *configuration);
/** @brief Describe camera slot five without starting work.
 * @return Native non-image backend with process-owned point-stream callbacks.
 * @details Static callback lifetimes let USHER retain the descriptor safely;
 * camera admission remains separate from route readiness. */
struct camera_backend dipla_camera_backend_get(void);
#endif
