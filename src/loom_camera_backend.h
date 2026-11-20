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
 * @file loom_camera_backend.h
 * @brief LOOM-owned capture specialization and stream lifecycle. */
#ifndef USHER_LOOM_CAMERA_BACKEND_H
#define USHER_LOOM_CAMERA_BACKEND_H

#include "camera_backend.h"

/** @name LOOM deployment defaults shared with CLI wiring
 * @{ */
#ifndef USHER_LOOMCAM_COMMAND
  /** @brief Native LOOM capture/saved-photo executable beside the deployed tools. */
  #define USHER_LOOMCAM_COMMAND "./bin/loomcam"
#endif
#ifndef USHER_LOOMCAM_MODEL
  /** @brief Expected RPK path; the framework does not fabricate a trained model. */
  #define USHER_LOOMCAM_MODEL "models/ai_cam/nn_loom_v001.rpk"
#endif
#ifndef USHER_LOOMCAM_REFERENCE_DIR
  /** @brief Default outbound archive; use independent reference storage per sortie. */
  #define USHER_LOOMCAM_REFERENCE_DIR "photos/loom-reference"
#endif
/** @} */

/**
 * @brief Configure the LOOM plugin before selecting or initializing its owner.
 * @param references Borrowed reference-directory path, valid until initialization.
 * @param model Borrowed RPK path, valid for the capture backend lifetime.
 * @pre No LOOM owner is active while configuration changes.
 * @details Saved-photo worker initialization copies its paths; native capture
 * continues borrowing the model until shutdown. Do not shorten that lifetime
 * merely because the matcher has already started.
 */
void loom_camera_backend_configure(const char *references, const char *model);

/**
 * @brief Return LOOM's descriptor without opening a sensor or starting a thread.
 * @param mock True for saved/mock pixels rather than the native capture owner.
 * @return Immutable hook lifetimes with LOOM-owned naming and point semantics.
 * @details Native LOOM advertises image capture and finish ownership, never object
 * detection. Frame timing remains on the common capture path; matching has its
 * own saved-photo stream lifecycle and cannot acquire the sensor again.
 */
struct camera_backend loom_camera_backend_get(bool mock);
/** @brief Choose one LOOM assignment before acquiring either worker.
 * @param settings Borrowed nonempty settings path, alive until capture and analysis have stopped.
 * @return Zero if accepted; minus one when invalid or workers already own configuration. */
int loom_camera_backend_set_settings(const char *settings);

#endif