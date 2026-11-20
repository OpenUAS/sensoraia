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

/** @file inference_worker.h
 * @brief Warm saved-picture processing without acquiring a camera or interpreting CAM results. */
#ifndef SENSORAIA_INFERENCE_WORKER_H
#define SENSORAIA_INFERENCE_WORKER_H

#include "inference_jpeg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Borrowed module callbacks; the worker serializes calls and retains no per-request pixel pointers.
 * @details Writers own partial-result cleanup; false must not leave a successful-looking record.
 * Private-image mutation is allowed only when the producer explicitly owns publication. */
struct inference_image_processor {
  const char *application, *module, *adapter;
  ini_field_handler setting;
  void *context;
  bool (*write)(void *context, const struct inference_context *model,
                 const char *image, uint32_t sequence, char *error, size_t capacity);
  bool (*validate)(void *context, const struct inference_context *model, char *error, size_t capacity);
  const char *result_suffix;
};

/** @brief Dispatch one-shot analysis, model checks or the persistent ANALYZE protocol.
 * @param argc Argument count including the executable.
 * @param argv Borrowed command-line tokens; diagnostics use stderr, protocol replies stdout.
 * @param processor Module contract whose callback/context lifetimes cover this call.
 * @return Minus one when this mode was not selected, zero on success, one on failure, two on misuse.
 * @note Request numbers increase within a server lifetime to prevent stale reply association. */
int inference_worker_cli(int argc, char **argv, const struct inference_image_processor *processor);

#ifdef __cplusplus
}
#endif
#endif