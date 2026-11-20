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

/** @file onnx_runtime.h
 * @brief Process-wide runtime ownership shared by independent model sessions. */
#ifndef SENSORAIA_ONNX_RUNTIME_H
#define SENSORAIA_ONNX_RUNTIME_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct OrtApi;
struct OrtEnv;

/**
 * @brief Initialize one process-lifetime runtime/environment using explicit local search roots.
 * @param library_directory Optional final search directory; NULL/empty omits it.
 * @param error Writable diagnostic buffer, cleared on success.
 * @param capacity Available bytes in error, including its terminator.
 * @return True after initialization or when this process already owns a runtime.
 * @details The first existing candidate must work; falling back would hide broken packages.
 * The library and environment stay alive so sessions never retain unloaded API pointers.
 * @note Concurrent initialization is serialized; later calls reuse the selected runtime.
 */
bool onnx_runtime_open(const char *library_directory, char *error, size_t capacity);
/** @brief Borrow the initialized API, without transferring release ownership.
 * @return Process-lifetime API, or NULL before successful open. */
const struct OrtApi *onnx_runtime_api(void);
/** @brief Borrow the environment retained until process exit.
 * @return Shared environment, or NULL before successful open. */
struct OrtEnv *onnx_runtime_environment(void);

#ifdef __cplusplus
}
#endif
#endif