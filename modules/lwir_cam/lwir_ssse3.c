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

/** @file
 * @brief x86-64 SSSE3 expansion of the camera's own YUYV image into JPEG Y/Cb/Cr scanlines.
 * @details This is the only translation unit built with `-mssse3` (see
 * build-support/make/lwircam.mk), so the rest of LWIRcam stays baseline x86-64.
 * lwir_render.c calls it from sensor_row() only after a cached CPUID check and only for
 * horizontal (+1/-1 stride) display rows of at least eight pixels; transposed rows,
 * the remainder tail and CPUs without SSSE3 use the scalar loop, which produces the
 * same bytes. No camera bytes are modified and no allocation is made.
 *
 * Input layout: packed YUYV, four bytes per pixel pair `Y0 U Y1 V`, so the two pixels of a
 * pair share one Cb (U) and one Cr (V). Output layout: three bytes per pixel `Y Cb Cr`,
 * ready for a libjpeg YCbCr scanline without an RGB round trip.
 */
#include "lwir_render.h"

#include <tmmintrin.h>

/**
 * @brief Expand eight YUYV pixels per iteration into interleaved Y/Cb/Cr, forwards or reversed.
 * @details One unaligned 16-byte load holds exactly four YUYV pairs (pixels 0..7 in memory).
 * Two PSHUFB masks scatter those bytes into the 24 output bytes: the `low` mask fills
 * output bytes 0..15 (five whole pixels plus the sixth pixel's Y) and the `high` mask
 * fills bytes 16..23 (the sixth pixel's Cb/Cr and the last two pixels); -128 lanes are
 * zeroed by PSHUFB and fall outside the 8-byte store.
 *
 * Forward (step > 0) masks follow memory order: pixel p takes Y at byte 2p and the
 * pair's U/V at bytes (p/2)*4+1 and (p/2)*4+3. Reversed (step < 0) rows load the block
 * that ENDS at the current pixel (`pixel - 7`) and the masks list memory pixels 7..0, so
 * mirroring costs no extra instruction.
 * @param frame Image plane start (combined frame or image-only YUYV buffer).
 * @param first Native pixel index of display column 0, from row_source().
 * @param step Native pixel stride per display column; must be +1 or -1.
 * @param width Display row width in pixels.
 * @param output Destination row with room for at least `width * 3` bytes.
 * @return Number of leading display columns written, a multiple of 8 (<= width); the
 *         caller completes the remainder with the scalar path.
 * @warning Requires SSSE3; callers must check CPU support first. Reversed rows rely on
 * `pixel - 7` being even (even native width puts the row's last pixel at an odd index),
 * so each load starts on an intact YUYV pair; arbitrary `first` values would split pairs.
 */
unsigned lwir_sensor_ssse3(const uint8_t *frame, ptrdiff_t first, ptrdiff_t step,
                          unsigned width, uint8_t *output)
{
    /* Triplets are (Y, U, V) source bytes per output pixel; both pixels of a pair repeat
     * bytes 1 and 3 because YUYV chroma is shared. Reversed masks walk pairs 3..0. */
    const __m128i low = step > 0 ? _mm_setr_epi8(0,1,3,2,1,3,4,5,7,6,5,7,8,9,11,10) :
        _mm_setr_epi8(14,13,15,12,13,15,10,9,11,8,9,11,6,5,7,4);
    const __m128i high = step > 0 ? _mm_setr_epi8(9,11,12,13,15,14,13,15,-128,-128,-128,-128,-128,-128,-128,-128) :
        _mm_setr_epi8(5,7,2,1,3,0,1,3,-128,-128,-128,-128,-128,-128,-128,-128);
    unsigned column = 0;
    for (; width - column >= 8; column += 8) {
        const ptrdiff_t pixel = first + (ptrdiff_t)column * step;
        /* Reading exactly 16 bytes (8 pixels) and writing 16 + 8 bytes stays inside both rows. */
        const __m128i packed = _mm_loadu_si128((const __m128i *)(frame + (size_t)(step < 0 ? pixel - 7 : pixel) * 2));
        _mm_storeu_si128((__m128i *)(output + column * 3), _mm_shuffle_epi8(packed, low));
        _mm_storel_epi64((__m128i *)(output + column * 3 + 16), _mm_shuffle_epi8(packed, high));
    }
    return column;
}
