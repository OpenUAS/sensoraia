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

/** @file inference_jpeg.h
 * @brief Bounded JPEG-to-tensor preparation without running the graph or modifying the input file. */
#ifndef SENSORAIA_INFERENCE_JPEG_H
#define SENSORAIA_INFERENCE_JPEG_H

#include "inference_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stream a JPEG into an open context's single declared image input.
 * @param context Exclusive model owner; input storage and transform are written in place.
 * @param filename Readable regular JPEG, immutable throughout decoding.
 * @param error Caller-owned diagnostic buffer, cleared on success.
 * @param capacity Available bytes in error, including its terminator.
 * @return True for complete, warning-free decoding and supported preprocessing.
 * @warning On failure discard the partially written tensor and transform; do not run inference.
 */
bool inference_jpeg_prepare(struct inference_context *context, const char *filename,
                             char *error, size_t capacity);

#ifdef __cplusplus
}
#endif
#endif