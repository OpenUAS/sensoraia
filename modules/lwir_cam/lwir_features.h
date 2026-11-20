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

#ifndef SENSORAIA_LWIR_FEATURES_H
#define SENSORAIA_LWIR_FEATURES_H
/** @file lwir_features.h
 * @brief Positive opt-ins shared by the worker and its USHER adapter.
 * @details Undefined switches mean imagery-only, including standalone compilation.
 * The root build records these values so changing profile rebuilds dependent code.
 * Defaults are 0 so a missing build flag can never silently enable optional code.
 */
#ifndef LWIR_ENABLE_ANALYSIS
/** @brief 1 builds the optional ONNX analysis of saved photos (lwir_inference). */
#define LWIR_ENABLE_ANALYSIS 0
#endif
#ifndef LWIR_ENABLE_MOCK
/** @brief 1 builds the development replay/synthetic-temperature adapter (lwir_mock). */
#define LWIR_ENABLE_MOCK 0
#endif
/* Only 0 and 1 are meaningful; fail the build early on anything else. */
#if (LWIR_ENABLE_ANALYSIS != 0 && LWIR_ENABLE_ANALYSIS != 1) || \
    (LWIR_ENABLE_MOCK != 0 && LWIR_ENABLE_MOCK != 1)
#error "LWIR feature switches must be 0 or 1"
#endif
#endif