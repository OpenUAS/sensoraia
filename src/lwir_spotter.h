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

#ifndef USHER_LWIR_SPOTTER_H
#define USHER_LWIR_SPOTTER_H

/** @file
 * @brief Serialized saved-LWIR object analysis with immutable frame and exposure evidence.
 */

#include <signal.h>
#include "capture_timing.h"

/**
 * @brief Select a separate ONNX detector assignment.
 * @param settings Borrowed settings path; keep it valid while analysis is enabled.
 * @return True if the path was accepted; false for an empty path or a live worker.
 * @details Store the selection without starting a worker. The first analysis
 * request loads the assignment.
 */
bool lwir_spotter_settings(const char *settings);
/**
 * @brief Load a native LWIR intrinsics and mount profile.
 * @param profile Path of the camera profile to read.
 * @return True if an LWIR profile with 256 by 192 calibration was loaded.
 * @details Validate a candidate before replacing the current profile. A profile
 * cannot be replaced while the analysis worker is live.
 */
bool lwir_spotter_profile(const char *profile);
/**
 * @brief Set the timing and pose uncertainty limits for projection.
 * @param limits Comma-separated maximum exposure interval in microseconds
 *               and maximum pose uncertainty in metres.
 * @return True if both positive bounds were accepted.
 * @details Apply both limits together after parsing, before the worker starts.
 * They constrain geographic estimates, not pixel-space detection.
 */
bool lwir_spotter_limits(const char *limits);
/**
 * @brief Test whether saved-LWIR analysis was explicitly selected.
 * @return True if a settings path has been selected.
 * @details This reports configuration, not worker readiness or model validity.
 */
bool lwir_spotter_enabled(void);
/** @brief Analyse one immutable image with copied capture evidence; no camera is acquired.
 * @details Caller serializes calls. Geographic results require qualified exposure
 * timing and pose; an unavailable position never invalidates a good image result.
 * @param filename Path to the published LWIR JPEG with embedded thermal evidence.
 * @param shot Capture trigger carrying the photo number.
 * @param timing Frame delivery and optional exposure/pose evidence for this image.
 * @param running Cancellation flag passed to the subprocess wrapper.
 * @return Zero for disabled or successful analysis, minus one on failure.
 */
int lwir_spotter_analyze(const char *filename, const union dc_shot_union *shot,
                         const struct capture_timing *timing, const volatile sig_atomic_t *running);
/**
 * @brief Stop the exclusively owned model worker.
 * @details Call after the analysis owner has joined. Stopping the process
 * does not clear the selected settings.
 */
void lwir_spotter_stop(void);

#endif