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

#ifndef SENSORAIA_LWIR_MOCK_H
#define SENSORAIA_LWIR_MOCK_H
/** @file lwir_mock.h
 * @brief Optional development replay adapter, independent of geographic analysis.
 * @details Built only with LWIR_ENABLE_MOCK. Turns an ordinary JPEG into LWIR
 * evidence for tests and demos: an existing temperature layer is preserved,
 * otherwise temperatures are synthesized from luminance and tagged LWIRSIM1.
 */
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Preserve embedded temperatures or attach explicitly synthetic evidence.
 * @param input_path Regular source JPEG.
 * @param output_path Destination; existing unrelated files are never overwritten.
 * @param layer_only Attach evidence without recompressing display pixels.
 * @return Zero on success, otherwise -1.
 * @details Serial worker adapter. Synthesized temperatures cannot represent a
 * physical measurement and never inherit live device-state provenance.
 * The result is published atomically (link or same-inode rename) and the
 * directory is fsynced; the source must stay unchanged during processing.
 * @note With @p layer_only and a source that already holds a temperature layer,
 * nothing is written and 0 is returned, even when @p output_path differs.
 * @warning Uses one static workspace; not reentrant or thread-safe.
 */
int lwir_process_mock(const char *input_path, const char *output_path, bool layer_only);

#ifdef __cplusplus
}
#endif
#endif