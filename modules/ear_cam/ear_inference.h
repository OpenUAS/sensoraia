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

/** @file ear_inference.h
 * @brief Bounded PCM classification without allowing model latency to stall ALSA capture. */
#ifndef SENSORAIA_EAR_INFERENCE_H
#define SENSORAIA_EAR_INFERENCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief One classified audio window; probability and measured level are different evidence. */
struct ear_inference_result {
  uint64_t begin_us, end_us, error_us, sequence;
  double level_dbfs, score; /**< PCM RMS level and selected model probability, respectively. */
  unsigned category;
  bool alarm, clipped;
};

/** @brief Select the explicit detector and qualify an ONNX PCM contract before capture starts.
 * @param settings EARcam application settings path.
 * @param rate Exact capture sample rate; no implicit resampling is performed.
 * @param enabled Set true for ONNX, false for a successfully selected native detector.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True for an accepted native or ONNX selection; false never means native fallback.
 * @pre No inference thread or previously configured model is active. */
bool ear_inference_configure(const char *settings, unsigned rate, bool *enabled,
                              char *error, size_t capacity);
/** @brief Start the sole classifier thread after successful ONNX configuration.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True after thread creation, not proof of a classified acoustic event. */
bool ear_inference_start(char *error, size_t capacity);
/** @brief Supply capture samples without waiting for model execution or a busy model mailbox.
 * @param samples Borrowed signed mono PCM; no pointer survives this call.
 * @param count Number of samples in this continuous capture chunk.
 * @param end_us Capture end of the chunk in CLOCK_MONOTONIC microseconds, or zero when unknown.
 * @param error_us Bound on capture-window timing uncertainty, not inference duration.
 * @details Complete windows may replace pending work; overload loses evidence rather than blocking ALSA.
 * @pre One capture producer owns push/discontinuity calls until stop begins. */
void ear_inference_push(const int16_t *samples, size_t count, uint64_t end_us, uint64_t error_us);
/** @brief Consume at most one completed original-window result without blocking capture.
 * @param result Writable destination, unchanged when no result is available.
 * @return True for a newly consumed completion; its window time is not its inference completion time. */
bool ear_inference_take(struct ear_inference_result *result);
/** @brief Invalidate queued/completed evidence after capture continuity breaks, including in-flight generations. */
void ear_inference_discontinuity(void);
/** @brief Request termination, join the classifier and release its model; repeat calls are harmless.
 * @pre The capture producer has stopped supplying new chunks. */
void ear_inference_stop(void);
/** @brief Classify exact whole windows from a regular little-endian signed-16 mono PCM file.
 * @param filename Immutable replay input; no hardware or simulated exposure timing is invented.
 * @return Zero after complete replay, otherwise one for invalid input or model failure.
 * @pre The model is configured but its capture worker is not running. */
int ear_inference_file(const char *filename);

#endif