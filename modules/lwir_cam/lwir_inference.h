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

/** @file lwir_inference.h
 * @brief Optional serialized LWIR model adapter, separate from USB capture and calibration.
 * @details Built only with LWIR_ENABLE_ANALYSIS. Analyses an already saved photo and
 * writes a separate sidecar; a failed analysis never affects the photograph. */
#ifndef SENSORAIA_LWIR_INFERENCE_H
#define SENSORAIA_LWIR_INFERENCE_H

#include "inference_settings.h"

/** @brief Open one explicit LWIR classifier with a declared pixel or radiometric input domain.
 * @param settings Resolved ONNX settings owned by the caller.
 * @param threshold Inclusive probability gate in [0,1].
 * @param category Index within the model's finite probability vector.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True for a qualified classifier contract, not calibrated temperatures or detection accuracy.
 * @pre Calls are serialized by the capture/analysis owner; configure before acquiring a live stream.
 * @note @p error is overwritten even on success. Once open, later calls return true
 * without applying new @p threshold or @p category; close first to change them. */
bool lwir_inference_open(const struct inference_settings *settings, double threshold, unsigned category,
                          char *error, size_t capacity);
/** @brief Classify one immutable JPEG/temperature plane and exclusively publish its model-identified sidecar.
 * @param filename JPEG whose original compressed pixels and temperature evidence are never rewritten.
 * @param sequence Producer correlation number, not proof of exposure synchronization.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True for complete inference/publication; false never invalidates the retained photograph.
 * @details Sidecar is "<name>.lwir.ini" beside "<name>.jpg", created exclusively, so an
 * existing sidecar makes the call fail instead of being replaced. */
bool lwir_inference_image(const char *filename, unsigned long long sequence, char *error, size_t capacity);
/** @brief Release the serialized classifier; repeated closes are harmless. */
void lwir_inference_close(void);

#endif