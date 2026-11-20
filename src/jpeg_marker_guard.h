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

/** @file jpeg_marker_guard.h
 * @brief Reject invalid Huffman selectors before the bundled codec indexes its tables. */
#ifndef USHER_JPEG_MARKER_GUARD_H
#define USHER_JPEG_MARKER_GUARD_H

#include <stddef.h>
#include <stdio.h>
#include <jpeglib.h>
#include <jerror.h>

/** @brief Validate marker parameters from an error manager's emit_message callback. */
static inline void usher_jpeg_validate_marker(j_common_ptr state)
{
  if (!state->is_decompressor) return;
  if (state->err->msg_code == JTRC_DHT) {
    const int index = state->err->msg_parm.i[0];
    if (index < 0 || (index & ~0x10) >= NUM_HUFF_TBLS) ERREXIT1(state, JERR_DHT_INDEX, index);
  } else if (state->err->msg_code == JTRC_SOS_COMPONENT && !((j_decompress_ptr)state)->arith_code) {
    const int *selectors = state->err->msg_parm.i;
    if (selectors[1] < 0 || selectors[1] >= NUM_HUFF_TBLS ||
        selectors[2] < 0 || selectors[2] >= NUM_HUFF_TBLS) ERREXIT(state, JERR_BAD_HUFF_TABLE);
  }
}

#endif