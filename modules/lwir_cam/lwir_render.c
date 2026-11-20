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
 * @brief Display window resolution and per-scanline Y/Cb/Cr rendering of LWIR frames.
 * @details Input is the Tiny1-C combined frame: the first half is the YUYV image plane
 * (`Y0 U Y1 V` per pixel pair, chroma shared by the pair), the second half the thermal
 * plane of little-endian uint16 Kelvin*64 codes (raw/64 - 273.15 = degC as reported by the
 * device, calibration unverified). Codes 0 and 65535 are endpoint/invalid codes.
 * LWIR_SENSOR also accepts an image-only buffer of half that size.
 *
 * Output is one JPEG-ready scanline of interleaved full-range BT.601 (JFIF) Y/Cb/Cr
 * bytes per call. Orientation is resolved once per scanline by row_source() into a first
 * native pixel and a signed stride, so the raw thermal bytes are never reordered or copied.
 *
 * Optional profile corrections, bad-pixel replacement and denoise act on display values
 * only. The scalar code is the reference; NEON (ARM64) and SSSE3 (x86-64) cover common
 * +/-1-stride cases and must yield identical bytes, falling back to scalar for transposed
 * rows, tails, profiles, denoise, the iron palette and legend rows.
 *
 * Storage: no heap. The percentile histogram is a 256 KiB static array, chosen over the
 * stack (too large for thread stacks) and heap (project policy); this makes
 * lwir_render_prepare() non-re-entrant.
 */
#include "lwir_render.h"

#include <errno.h>
#include <stdio.h>

#if defined(__x86_64__) && !defined(LWIR_DISABLE_SSSE3)
#include <cpuid.h>
#endif

#if defined(__aarch64__) && !defined(LWIR_DISABLE_NEON)
#include <arm_neon.h>
#endif

/** Percentile workspace: one counter per possible raw code. Shared, so prepare is serial. */
static uint32_t histogram[65536];

/**
 * @brief Read one little-endian thermal code.
 * @param frame Combined frame start.
 * @param pixels Native pixel count (width * height); the thermal plane starts at pixels * 2.
 * @param pixel Native raster index.
 * @return Raw Kelvin*64 code, including endpoint codes 0/65535.
 * @note Caller guarantees bounds; byte assembly keeps it independent of host endianness and alignment.
 */
static uint16_t temperature(const uint8_t *frame, size_t pixels, size_t pixel)
{
    const size_t offset = (pixels + pixel) * 2;
    return (uint16_t)frame[offset] | (uint16_t)frame[offset + 1] << 8;
}

#if defined(__aarch64__) && !defined(LWIR_DISABLE_NEON)
/**
 * @brief Load eight consecutive thermal codes as uint16 lanes.
 * @details VLD2 de-interleaves 16 bytes into low bytes (val[0]) and high bytes (val[1]);
 * widening and `low | high << 8` rebuilds little-endian codes without unaligned u16 loads.
 * @param plane Thermal plane start.
 * @param pixel Native raster index of lane 0; pixels pixel..pixel+7 must exist.
 * @return Raw codes in memory order.
 */
static uint16x8_t thermal_block(const uint8_t *plane, size_t pixel)
{
    const uint8x8x2_t bytes = vld2_u8(plane + pixel * 2);
    return vorrq_u16(vmovl_u8(bytes.val[0]), vshlq_n_u16(vmovl_u8(bytes.val[1]), 8));
}

/**
 * @brief Lane mask of endpoint codes.
 * @param raw Eight raw codes.
 * @return 0xFFFF where the code is 0 or 65535, else 0.
 */
static uint16x8_t invalid_block(uint16x8_t raw)
{
    return vorrq_u16(vceqq_u16(raw, vdupq_n_u16(0)), vceqq_u16(raw, vdupq_n_u16(UINT16_MAX)));
}
#endif

/**
 * @brief Check pointers, native geometry and buffer size for the requested view.
 * @param frame Frame bytes.
 * @param bytes Supplied size.
 * @param width Native width.
 * @param height Native height.
 * @param options Render options (only the view is consulted here).
 * @return True for an exact combined frame, or an exact image-only half for LWIR_SENSOR.
 */
static bool render_frame_valid(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                               const struct lwir_render_options *options)
{
    const size_t combined_bytes = lwir_frame_bytes(width, height);
    return frame && combined_bytes && options &&
        (bytes == combined_bytes || (options->view == LWIR_SENSOR && bytes == combined_bytes / 2U));
}

/**
 * @brief Validate enumerations, orientation and view-specific option ranges.
 * @details Rotation must be a quarter turn 0..270. LWIR_SENSOR rejects profile and denoise
 * because the camera image has no raw codes to correct. LWIR_FIXED needs
 * 0 <= low < high <= 65535; LWIR_PERCENTILE needs low < high <= 100. Denoise is capped
 * at 640 raw (10 K).
 * @param options Options to check, may be NULL.
 * @return True when every field is supported.
 */
static bool valid_options(const struct lwir_render_options *options)
{
    return options != NULL && options->view >= LWIR_SENSOR && options->view <= LWIR_PERCENTILE &&
        options->palette >= LWIR_WHITE_HOT && options->palette <= LWIR_IRON && options->denoise_raw <= 640 &&
        options->rotation <= 270 && options->rotation % 90 == 0 &&
        (options->view != LWIR_SENSOR || (!options->profile && !options->denoise_raw)) &&
        (options->view != LWIR_FIXED || (options->low >= 0 && options->high <= 65535 && options->low < options->high)) &&
        (options->view != LWIR_PERCENTILE || (options->low_percentile < options->high_percentile && options->high_percentile <= 100));
}

/**
 * @brief Display size after orientation.
 * @details 90/270 degree rotations swap width and height; mirror and flip do not.
 * @return False for unsupported geometry, invalid options or NULL outputs (errno untouched).
 */
bool lwir_render_dimensions(unsigned width, unsigned height, const struct lwir_render_options *options,
                              unsigned *display_width, unsigned *display_height)
{
    if (!lwir_frame_bytes(width, height) || !valid_options(options) || !display_width || !display_height) return false;
    const bool transpose = options->rotation == 90 || options->rotation == 270;
    *display_width = transpose ? height : width; *display_height = transpose ? width : height;
    return true;
}

/**
 * @brief Map a display row to its first native pixel and per-column native stride.
 * @details Display = flip(mirror(rotate_cw(native))), so the inverse is applied in reverse
 * order. For display (x, row):
 * 1. Undo flip: `vertical = flip ? display_height-1-row : row`.
 * 2. Undo mirror: rotated column `x' = horizontal + direction*x`, with
 *    `horizontal = mirror ? display_width-1 : 0` and `direction = mirror ? -1 : +1`.
 * 3. Undo clockwise rotation of (x', vertical) into native (nx, ny):
 *    - 0:   nx = x',          ny = vertical         -> stride  +direction
 *    - 90:  nx = vertical,    ny = height-1-x'      -> stride  -direction*width
 *    - 180: nx = width-1-x',  ny = height-1-vertical -> stride -direction
 *    - 270: nx = width-1-vertical, ny = x'          -> stride  +direction*width
 * Native index ny*width + nx is affine in x, so the result is `first + x*step`.
 * @param width Native width.
 * @param height Native height.
 * @param options Orientation policy (validated).
 * @param display_width Oriented width.
 * @param display_height Oriented height.
 * @param row Display row, < display_height.
 * @param step Output signed native stride per display column.
 * @return Native raster index of display column 0.
 * @note With even native width, forward (+1) rows start at an even native index and
 * reversed (-1) rows start at an odd one, so `pixel - 7` is even and 8-pixel SIMD blocks
 * always begin on an intact YUYV pair.
 */
static ptrdiff_t row_source(unsigned width, unsigned height, const struct lwir_render_options *options,
                             unsigned display_width, unsigned display_height, unsigned row, ptrdiff_t *step)
{
    const unsigned vertical = options->flip ? display_height - 1 - row : row;
    const unsigned horizontal = options->mirror ? display_width - 1 : 0;
    const ptrdiff_t direction = options->mirror ? -1 : 1;
    /* One inverse transform per scanline; the hot loop uses only a signed stride. */
    switch (options->rotation) {
    case 90: *step = -direction * width; return (ptrdiff_t)(height - 1 - horizontal) * width + vertical;
    case 180: *step = -direction; return (ptrdiff_t)(height - 1 - vertical) * width + width - 1 - horizontal;
    case 270: *step = direction * width; return (ptrdiff_t)horizontal * width + width - 1 - vertical;
    default: *step = direction; return (ptrdiff_t)vertical * width + horizontal;
    }
}

/**
 * @brief Convert a display coordinate into native sensor coordinates.
 * @details Uses the same row_source() transform as rendering, so overlays and
 * measurements picked on the display address exactly the pixel that was shown.
 * @return False for invalid geometry/options, NULL outputs or out-of-range display points.
 */
bool lwir_render_native_coordinate(unsigned width, unsigned height, const struct lwir_render_options *options,
                                     unsigned display_x, unsigned display_y, unsigned *native_x, unsigned *native_y)
{
    unsigned display_width, display_height;
    if (!native_x || !native_y || !lwir_render_dimensions(width, height, options, &display_width, &display_height) ||
        display_x >= display_width || display_y >= display_height) return false;
    ptrdiff_t step;
    const ptrdiff_t first = row_source(width, height, options, display_width, display_height, display_y, &step);
    const size_t pixel = (size_t)(first + (ptrdiff_t)display_x * step);
    *native_x = (unsigned)(pixel % width); *native_y = (unsigned)(pixel / width);
    return true;
}

#if defined(__x86_64__) && !defined(LWIR_DISABLE_SSSE3)
/**
 * @brief Runtime SSSE3 detection, cached after the first query.
 * @return True when CPUID leaf 1 reports SSSE3.
 * @note The lazy cache is unsynchronized: concurrent first calls would race on the static,
 * although every caller computes the same value.
 */
static bool ssse3_available(void)
{
    static int available = -1;
    if (available < 0) {
        unsigned leaf, vendor, features, extended;
        available = __get_cpuid(1, &leaf, &vendor, &features, &extended) && (features & bit_SSSE3) != 0;
    }
    return available != 0;
}

#endif

/**
 * @brief Copy the camera's own YUYV image row into Y/Cb/Cr output (LWIR_SENSOR).
 * @details The camera's chroma is reused directly, so no colour conversion is needed:
 * each output pixel takes its own Y and its pair's shared U (Cb) and V (Cr). SSSE3 or NEON
 * handle leading 8-pixel blocks of +/-1-stride rows; the scalar loop handles the rest and
 * all transposed (+/-width stride) rows.
 * @param frame Image plane start (combined or image-only buffer).
 * @param first Native index of display column 0.
 * @param step Signed native stride per display column.
 * @param width Display width.
 * @param output Destination of width * 3 bytes.
 */
static void sensor_row(const uint8_t *frame, ptrdiff_t first, ptrdiff_t step, unsigned width, uint8_t *output)
{
    unsigned column = 0;
#if defined(__x86_64__) && !defined(LWIR_DISABLE_SSSE3)
    if ((step == 1 || step == -1) && width >= 8 && ssse3_available()) column = lwir_sensor_ssse3(frame, first, step, width, output);
#endif
#if defined(__aarch64__) && !defined(LWIR_DISABLE_NEON)
    /* ARM64 SIMD fuses eight YUY2 expansions/reversals into the JPEG scanline.
     * +/-1 rows begin at an intact YUY2 pair; transposed rows use the scalar stride.
     * Flip only selects another source row: no image copy or camera control write. */
    if (step == 1 || step == -1) {
        for (; width - column >= 8; column += 8) {
            const ptrdiff_t pixel = first + (ptrdiff_t)column * step;
            const uint8x8x2_t packed = vld2_u8(frame + (size_t)(step < 0 ? pixel - 7 : pixel) * 2);
            /* val[0] = Y0..Y7; val[1] = U0 V0 U1 V1 U2 V2 U3 V3. Unzipping that vector with
             * itself puts U0..U3 / V0..V3 in lanes 0..3 (lanes 4..7 are unused copies). */
            const uint8x8x2_t chroma = vuzp_u8(packed.val[1], packed.val[1]);
            uint8x8_t luminance = packed.val[0], cb = chroma.val[0], cr = chroma.val[1];
            /* Reverse 8 Y lanes, but only the four pair-chroma lanes: rev32 reverses each
             * 4-byte group, and only the low group is consumed below. */
            if (step < 0) { luminance = vrev64_u8(luminance); cb = vrev32_u8(cb); cr = vrev32_u8(cr); }
            /* Zipping lanes 0..3 with themselves duplicates each pair chroma for both pixels;
             * VST3 then interleaves Y/Cb/Cr straight into the JPEG scanline. */
            const uint8x8x3_t expanded = {.val = {luminance, vzip_u8(cb, cb).val[0], vzip_u8(cr, cr).val[0]}};
            vst3_u8(output + column * 3, expanded);
        }
    }
#endif
    for (; column < width; ++column) {
        /* Y sits at pixel*2; the pair's 4-byte group starts at (pixel/2)*4 with U at +1, V at +3. */
        const size_t pixel = (size_t)(first + (ptrdiff_t)column * step), pair = pixel / 2 * 4;
        output[column * 3] = frame[pixel * 2];
        output[column * 3 + 1] = frame[pair + 1]; output[column * 3 + 2] = frame[pair + 3];
    }
}

/**
 * @brief Test whether a native pixel is listed as bad in the profile.
 * @details Lower-bound binary search over the strictly ascending bad_pixels list
 * (ordering is enforced by valid_profile()); O(log n) for at most 128 entries.
 * @param profile Optional profile; NULL means no bad pixels.
 * @param pixel Native raster index.
 * @return True when the pixel is listed.
 */
static bool bad_pixel(const struct lwir_profile *profile, size_t pixel)
{
    if (!profile) return false;
    unsigned low = 0, high = profile->bad_count;
    while (low < high) {
        const unsigned middle = low + (high - low) / 2;
        if (profile->bad_pixels[middle] < pixel) low = middle + 1; else high = middle;
    }
    return low < profile->bad_count && profile->bad_pixels[low] == pixel;
}

/**
 * @brief Read an unreplaced sample usable as-is or as a neighbour.
 * @param frame Combined frame.
 * @param width Native width.
 * @param height Native height.
 * @param pixel Native raster index.
 * @param profile Optional profile supplying bad pixels and the supported raw range.
 * @param raw Output code; written unless the pixel is listed bad.
 * @return False for listed bad pixels, endpoint codes or codes outside the profile's
 *         support range (these render as invalid rather than being extrapolated).
 */
static bool original_sample(const uint8_t *frame, unsigned width, unsigned height, size_t pixel,
                             const struct lwir_profile *profile, uint16_t *raw)
{
    if (bad_pixel(profile, pixel)) return false;
    *raw = temperature(frame, (size_t)width * height, pixel);
    return *raw && *raw != UINT16_MAX && (!profile || (*raw >= profile->range_min && *raw <= profile->range_max));
}

/**
 * @brief Produce the uncorrected display value of one native pixel.
 * @details
 * 1. Ordinary pixels use original_sample().
 * 2. Listed bad pixels take the median of their usable 4-connected neighbours (bad,
 *    endpoint and out-of-support neighbours are skipped; edges have fewer candidates).
 * 3. If denoise_raw is set, average the value with those of its 8 neighbours whose
 *    original sample lies within +/-denoise_raw of it (edge-preserving: neighbours across
 *    a stronger edge are excluded), rounded to nearest.
 * @param frame Combined frame.
 * @param width Native width.
 * @param height Native height.
 * @param pixel Native raster index.
 * @param options Profile and denoise policy.
 * @param raw Output display value (still uncorrected Kelvin*64).
 * @return False when no usable value exists; the pixel renders as invalid.
 * @note Neighbourhoods read original samples only, never replaced or denoised values, so
 * the result is independent of traversal order.
 */
static bool display_sample(const uint8_t *frame, unsigned width, unsigned height, size_t pixel,
                            const struct lwir_render_options *options, uint16_t *raw)
{
    if (!bad_pixel(options->profile, pixel)) {
        if (!original_sample(frame, width, height, pixel, options->profile, raw)) return false;
    } else {
        const unsigned column = (unsigned)(pixel % width), row = (unsigned)(pixel / width);
        uint16_t neighbors[4];
        unsigned count = 0;
        /* Off-sensor directions alias to the pixel itself and are skipped below. */
        const size_t positions[4] = {column ? pixel - 1 : pixel, column + 1 < width ? pixel + 1 : pixel,
            row ? pixel - width : pixel, row + 1 < height ? pixel + width : pixel};
        for (unsigned index = 0; index < 4; ++index) {
            uint16_t candidate;
            if (positions[index] != pixel && original_sample(frame, width, height, positions[index], options->profile, &candidate)) neighbors[count++] = candidate;
        }
        if (!count) return false;
        /* Insertion sort of <=4 values; count/2 picks the UPPER median for even counts
         * (no averaging), so the replacement is always a real neighbour's code. */
        for (unsigned index = 1; index < count; ++index) {
            const uint16_t value = neighbors[index]; unsigned position = index;
            while (position && neighbors[position - 1] > value) { neighbors[position] = neighbors[position - 1]; --position; }
            neighbors[position] = value;
        }
        *raw = neighbors[count / 2];
    }
    if (options->denoise_raw) {
        const int column = (int)(pixel % width), row = (int)(pixel / width);
        unsigned sum = *raw, count = 1;
        for (int vertical = -1; vertical <= 1; ++vertical) for (int horizontal = -1; horizontal <= 1; ++horizontal) {
            if ((!vertical && !horizontal) || column + horizontal < 0 || column + horizontal >= (int)width || row + vertical < 0 || row + vertical >= (int)height) continue;
            uint16_t candidate;
            const size_t neighbor = (size_t)(row + vertical) * width + (unsigned)(column + horizontal);
            if (original_sample(frame, width, height, neighbor, options->profile, &candidate)) {
                const unsigned difference = candidate > *raw ? candidate - *raw : *raw - candidate;
                if (difference <= options->denoise_raw) { sum += candidate; ++count; }
            }
        }
        *raw = (uint16_t)((sum + count / 2) / count);
    }
    return true;
}

/**
 * @brief Apply the profile's linear display correction.
 * @details `round_half_up(raw * scale_ppm / 1e6) + trunc(offset_mk * 64 / 1000)`: one kelvin
 * is 64 raw units, so one millikelvin is 0.064 raw. 64-bit intermediates avoid overflow.
 * For the accepted positive scale the mapping is monotone non-decreasing, so ranking raw
 * values (percentiles) and correcting afterwards preserves order.
 * @param raw Raw or replaced/denoised display value.
 * @param profile Optional profile; NULL returns raw unchanged.
 * @return Corrected value in Kelvin*64 units; may fall outside 0..65535.
 */
static int32_t corrected(int32_t raw, const struct lwir_profile *profile)
{
    return profile ? (int32_t)(((int64_t)raw * profile->scale_ppm + 500000) / 1000000 + (int64_t)profile->offset_mk * 64 / 1000) : raw;
}

/**
 * @brief Check a profile against the native geometry and renderer limits.
 * @details Requires matching geometry, at most LWIR_MAX_BAD_PIXELS entries, scale
 * 0.5..1.5, offset +/-50 K, a non-empty support range excluding both endpoint codes, and
 * in-bounds strictly ascending bad pixels (required by bad_pixel()'s binary search).
 * @param profile Optional profile; NULL is valid.
 * @param width Native width.
 * @param height Native height.
 * @return True when the profile can be applied.
 */
static bool valid_profile(const struct lwir_profile *profile, unsigned width, unsigned height)
{
    if (!profile) return true;
    if (profile->width != width || profile->height != height || profile->bad_count > LWIR_MAX_BAD_PIXELS ||
        profile->scale_ppm < 500000 || profile->scale_ppm > 1500000 || profile->offset_mk < -50000 || profile->offset_mk > 50000 ||
        !profile->range_min || profile->range_min >= profile->range_max || profile->range_max == UINT16_MAX) return false;
    for (unsigned index = 0; index < profile->bad_count; ++index) if (profile->bad_pixels[index] >= (size_t)width * height ||
        (index && profile->bad_pixels[index] <= profile->bad_pixels[index - 1])) return false;
    return true;
}

/**
 * @brief Validate a render request and resolve its display window and counts.
 * @details Percentile algorithm: histogram every valid uncorrected display value, then
 * pick the values at 0-based ranks `floor((samples-1) * percentile / 100)` (lower
 * nearest-rank) by scanning cumulative counts. The window is corrected afterwards, which
 * is equivalent to ranking corrected values because correction is monotone. Afterwards
 * every pixel is classified as invalid, below or above the window.
 * @return True on success; false with errno EINVAL for invalid input or ENODATA when a
 *         percentile view has no valid sample.
 * @warning Not re-entrant: shares the static histogram.
 */
bool lwir_render_prepare(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                           const struct lwir_render_options *options, struct lwir_render_range *range)
{
    unsigned display_width, display_height;
    if (!render_frame_valid(frame, bytes, width, height, options) || !lwir_render_dimensions(width, height, options, &display_width, &display_height) ||
        !range || !valid_profile(options->profile, width, height) ||
        (options->legend && (options->view == LWIR_SENSOR || display_width < 112 || display_height < 24))) { errno = EINVAL; return false; }
    struct lwir_render_range result = {.low = options->low, .high = options->high};
    if (options->view == LWIR_SENSOR) { *range = (struct lwir_render_range){.high = 255}; return true; }
    const size_t pixels = (size_t)width * height;
    if (options->view == LWIR_PERCENTILE) {
        for (size_t index = 0; index < 65536; ++index) histogram[index] = 0;
        size_t samples = 0;
        for (size_t pixel = 0; pixel < pixels; ++pixel) {
            uint16_t raw;
            if (display_sample(frame, width, height, pixel, options, &raw)) { ++histogram[raw]; ++samples; }
        }
        if (samples == 0) { errno = ENODATA; return false; }
        /* Rank r means "the (r+1)-th smallest"; the first code whose cumulative count
         * exceeds r holds it. Integer math keeps percentile windows deterministic. */
        const size_t low_rank = (samples - 1) * options->low_percentile / 100;
        const size_t high_rank = (samples - 1) * options->high_percentile / 100;
        size_t accumulated = 0;
        bool found_low = false;
        /* Endpoint bins 0/65535 cannot be filled: display_sample() rejects them and
         * replacements/denoise only combine valid codes. */
        for (unsigned raw = 1; raw < UINT16_MAX; ++raw) {
            accumulated += histogram[raw];
            if (!found_low && accumulated > low_rank) { result.low = (int32_t)raw; found_low = true; }
            if (accumulated > high_rank) { result.high = (int32_t)raw; break; }
        }
        /* A uniform scene gives equal ranks; +/-32 raw (+/-0.5 K) keeps a usable gradient. */
        if (result.low == result.high) { result.low -= 32; result.high += 32; }
        result.low = corrected(result.low, options->profile); result.high = corrected(result.high, options->profile);
        /* Down-scaling profiles can merge adjacent raw codes; keep the display window non-empty. */
        if (result.low >= result.high) { result.low -= 32; result.high += 32; }
    }
    size_t pixel = 0;
#if defined(__aarch64__) && !defined(LWIR_DISABLE_NEON)
    /* Without profile/denoise a display value is the raw code, so lane compares match the
     * scalar classification; the window must fit uint16 for the vector constants. */
    if (!options->profile && !options->denoise_raw && result.low >= 0 && result.high <= UINT16_MAX) {
        for (; pixels - pixel >= 8; pixel += 8) {
            const uint16x8_t raw = thermal_block(frame + pixels * 2, pixel), invalid = invalid_block(raw), valid = vmvnq_u16(invalid);
            /* Compare masks are 0xFFFF per true lane; >>15 turns them into 0/1 for horizontal sums. */
            result.invalid += vaddlvq_u16(vshrq_n_u16(invalid, 15));
            result.below += vaddlvq_u16(vshrq_n_u16(vandq_u16(valid, vcltq_u16(raw, vdupq_n_u16((uint16_t)result.low))), 15));
            result.above += vaddlvq_u16(vshrq_n_u16(vandq_u16(valid, vcgtq_u16(raw, vdupq_n_u16((uint16_t)result.high))), 15));
        }
    }
#endif
    for (; pixel < pixels; ++pixel) {
        uint16_t raw;
        if (!display_sample(frame, width, height, pixel, options, &raw)) ++result.invalid;
        else { const int32_t value = corrected(raw, options->profile); result.below += value < result.low; result.above += value > result.high; }
    }
    *range = result;
    return true;
}

/**
 * @brief Map a window intensity to RGB for the selected palette.
 * @details White-hot is grey `i`, black-hot grey `255-i`. Iron splits 0..255 into five
 * equal segments between six anchors: `scaled = 5i`, segment `scaled / 255`, weight
 * `scaled % 255`, then rounded linear interpolation `(a*(255-f) + b*f + 127) / 255`.
 * At i = 255 the segment is 5 with weight 0, so the last anchor is returned exactly.
 * @param intensity Window position 0..255.
 * @param palette Palette.
 * @param red Output red 0..255.
 * @param green Output green 0..255.
 * @param blue Output blue 0..255.
 */
static void color(unsigned intensity, enum lwir_palette palette, unsigned *red, unsigned *green, unsigned *blue)
{
    if (palette != LWIR_IRON) {
        *red = *green = *blue = palette == LWIR_BLACK_HOT ? 255 - intensity : intensity;
        return;
    }
    static const uint8_t anchors[6][3] = {{0,0,0}, {40,0,100}, {120,0,100}, {200,40,20}, {255,150,0}, {255,255,150}};
    const unsigned scaled = intensity * 5;
    const unsigned segment = scaled / 255;
    const unsigned fraction = scaled % 255;
    const unsigned next = segment < 5 ? segment + 1 : 5;
    *red = (anchors[segment][0] * (255 - fraction) + anchors[next][0] * fraction + 127) / 255;
    *green = (anchors[segment][1] * (255 - fraction) + anchors[next][1] * fraction + 127) / 255;
    *blue = (anchors[segment][2] * (255 - fraction) + anchors[next][2] * fraction + 127) / 255;
}

/**
 * @brief One row of a built-in 5x7 bitmap font.
 * @param character Digit, 'C', '-', '.' or anything else (blank).
 * @param row Font row 0..6.
 * @return Five-bit row mask; bit 4 is the leftmost column.
 */
static uint8_t glyph(char character, unsigned row)
{
    static const uint8_t digits[10][7] = {
        {14,17,19,21,25,17,14}, {4,12,4,4,4,4,14}, {14,17,1,2,4,8,31}, {30,1,1,14,1,1,30},
        {2,6,10,18,31,2,2}, {31,16,16,30,1,1,30}, {14,16,16,30,17,17,14}, {31,1,2,4,8,8,8},
        {14,17,17,14,17,17,14}, {14,17,17,15,1,1,14}};
    static const uint8_t celsius[7] = {14,17,16,16,16,17,14};
    if (character >= '0' && character <= '9') return digits[(unsigned)(character - '0')][row];
    if (character == 'C') return celsius[row];
    if (character == '-') return row == 3 ? 14 : 0;
    return character == '.' && row == 6 ? 4 : 0;
}

/**
 * @brief Format a window bound as degrees Celsius with one decimal.
 * @details `raw * 1000 / 64` gives millikelvin, minus 273150 gives milli-degC, `/ 100`
 * gives tenths. Integer division truncates toward zero, so labels are truncated, not
 * rounded. The value is the device-reported (optionally profile-corrected) scale, not a
 * verified temperature.
 * @param text Output buffer of 16 bytes, always NUL-terminated by snprintf.
 * @param raw Window bound in Kelvin*64 units.
 */
static void legend_label(char text[16], int32_t raw)
{
    const int32_t tenths = (int32_t)(((int64_t)raw * 1000 / 64 - 273150) / 100);
    const unsigned magnitude = (unsigned)(tenths < 0 ? -tenths : tenths);
    snprintf(text, 16, "%s%u.%uC", tenths < 0 ? "-" : "", magnitude / 10, magnitude % 10);
}

/**
 * @brief Colour of one pixel inside the 16-row legend strip at the bottom of the display.
 * @details Strip rows from top: 7 rows of palette gradient across the full width
 * (`column * 255 / (width-1)`), one black separator row, 7 font rows, one black bottom row.
 * The low label is left-aligned in the left half, the high label right-aligned; each
 * glyph uses a 6-pixel cell (5 pixels plus a gap). Positions outside a label stay black.
 * @param row Display row within the last 16 rows.
 * @param column Display column.
 * @param width Display width (>= 112, checked by callers).
 * @param height Display height (>= 24, checked by callers).
 * @param options Palette.
 * @param range Window whose bounds are labelled.
 * @param red Output red.
 * @param green Output green.
 * @param blue Output blue.
 * @note Labels are re-formatted per legend pixel: simpler than caching across calls and
 * limited to 16 rows, at the cost of redundant snprintf work.
 */
static void legend_pixel(unsigned row, unsigned column, unsigned width, unsigned height,
                          const struct lwir_render_options *options, const struct lwir_render_range *range,
                          unsigned *red, unsigned *green, unsigned *blue)
{
    if (row < height - 9) { color(column * 255 / (width - 1), options->palette, red, green, blue); return; }
    *red = *green = *blue = 0;
    const unsigned font_row = row - (height - 8);
    if (row == height - 9 || font_row >= 7) return;
    char low[16], high[16];
    legend_label(low, range->low); legend_label(high, range->high);
    size_t high_length = 0; while (high[high_length]) ++high_length;
    const unsigned right_start = width - (unsigned)high_length * 6;
    const char *text = column < width / 2 ? low : high;
    /* Columns in the right half before the high label get position = width, which is past
     * any label and therefore blank. */
    unsigned position = column < width / 2 ? column : column >= right_start ? column - right_start : width;
    size_t length = 0; while (text[length]) ++length;
    if (position / 6 < length && position % 6 < 5 && (glyph(text[position / 6], font_row) & (16U >> (position % 6))))
        *red = *green = *blue = 255;
}

#if defined(__aarch64__) && !defined(LWIR_DISABLE_NEON)
/**
 * @brief Exact `floor(delta * 255 / span)` for four lanes without integer vector division.
 * @details NEON has no integer divide, so:
 * 1. numerator = delta * 255 (<= 16711425 < 2^24, exact in float).
 * 2. q = trunc(numerator * reciprocal); the rounded reciprocal can put q one unit off.
 * 3. If q * span > numerator, q was one too high: subtract 1 (mask >> 31 is 0 or 1).
 * 4. If (q+1) * span <= numerator, q was one too low: add 1.
 * Products stay below 2^32 because q <= 256 and span <= 65535. The result equals the
 * scalar `(value - low) * 255 / (high - low)`.
 * @param delta Clamped `raw - low`, 0..span.
 * @param span Window width high - low, 1..65535.
 * @param reciprocal 1.0f / span, computed once per row.
 * @return Intensities 0..255.
 */
static uint16x4_t window_intensity(uint16x4_t delta, unsigned span, float reciprocal)
{
    const uint32x4_t numerator = vmull_n_u16(delta, 255), denominator = vdupq_n_u32(span);
    /* Numerator <=16711425 is exactly representable in float. Reciprocal estimates
     * differ by <1 integer unit; these integer checks recover exact floor division. */
    uint32x4_t quotient = vcvtq_u32_f32(vmulq_n_f32(vcvtq_f32_u32(numerator), reciprocal));
    quotient = vsubq_u32(quotient, vshrq_n_u32(vcgtq_u32(vmulq_u32(quotient, denominator), numerator), 31));
    const uint32x4_t next = vaddq_u32(quotient, vdupq_n_u32(1));
    quotient = vaddq_u32(quotient, vshrq_n_u32(vcleq_u32(vmulq_u32(next, denominator), numerator), 31));
    return vmovn_u32(quotient);
}

/**
 * @brief NEON greyscale thermal row for +/-1 strides, eight pixels per iteration.
 * @details Loads eight codes (block ending at the current pixel when reversed, then full
 * lane reversal), clamps to the window, converts to intensity with window_intensity(),
 * applies black-hot inversion and writes grey Y/Cb/Cr (Cb = Cr = 128, which is exactly
 * what the scalar BT.601 formulas yield for R = G = B). Flag colours are then blended in
 * the scalar priority order: invalid overrides above/below, matching the scalar branch
 * where invalid is tested first.
 * @param plane Thermal plane start.
 * @param first Native index of display column 0.
 * @param step +1 or -1.
 * @param width Display width.
 * @param options Palette (white-hot or black-hot only).
 * @param range Window with 0 <= low < high <= 65535.
 * @param output Y/Cb/Cr destination of width * 3 bytes.
 * @return Columns written, a multiple of 8; the caller finishes the tail.
 * @warning Only valid without profile, denoise, iron palette or legend rows; the caller
 * enforces those conditions.
 */
static unsigned thermal_neon(const uint8_t *plane, ptrdiff_t first, ptrdiff_t step, unsigned width,
                               const struct lwir_render_options *options, const struct lwir_render_range *range, uint8_t *output)
{
    const uint16x8_t low = vdupq_n_u16((uint16_t)range->low), high = vdupq_n_u16((uint16_t)range->high);
    const unsigned span = (unsigned)(range->high - range->low);
    const float reciprocal = 1.0f / (float)span;
    /* Exact full-range YCbCr of the existing cyan, magenta and yellow RGB flags. */
    static const uint8_t below_color[3] = {134, 196, 32}, above_color[3] = {106, 213, 235}, invalid_color[3] = {226, 1, 149};
    unsigned column = 0;
    for (; width - column >= 8; column += 8) {
        const ptrdiff_t pixel = first + (ptrdiff_t)column * step;
        uint16x8_t raw = thermal_block(plane, (size_t)(step < 0 ? pixel - 7 : pixel));
        /* Full 8-lane reversal: reverse each 4-lane half, then swap the halves. */
        if (step < 0) raw = vcombine_u16(vrev64_u16(vget_high_u16(raw)), vrev64_u16(vget_low_u16(raw)));
        const uint16x8_t delta = vsubq_u16(vminq_u16(vmaxq_u16(raw, low), high), low);
        uint8x8_t intensity = vmovn_u16(vcombine_u16(window_intensity(vget_low_u16(delta), span, reciprocal), window_intensity(vget_high_u16(delta), span, reciprocal)));
        if (options->palette == LWIR_BLACK_HOT) intensity = vsub_u8(vdup_n_u8(255), intensity);
        uint8x8x3_t expanded = {.val = {intensity, vdup_n_u8(128), vdup_n_u8(128)}};
        const uint8x8_t below = vmovn_u16(vcltq_u16(raw, low)), above = vmovn_u16(vcgtq_u16(raw, high)), invalid = vmovn_u16(invalid_block(raw));
        /* Endpoint codes may also test below/above; selecting invalid last gives it priority. */
        for (unsigned channel = 0; channel < 3; ++channel) {
            expanded.val[channel] = vbsl_u8(below, vdup_n_u8(below_color[channel]), expanded.val[channel]);
            expanded.val[channel] = vbsl_u8(above, vdup_n_u8(above_color[channel]), expanded.val[channel]);
            expanded.val[channel] = vbsl_u8(invalid, vdup_n_u8(invalid_color[channel]), expanded.val[channel]);
        }
        vst3_u8(output + column * 3, expanded);
    }
    return column;
}
#endif

/**
 * @brief Render one oriented display row as interleaved Y/Cb/Cr.
 * @details Resolves the row's first native pixel and stride once, then either copies the
 * camera image (LWIR_SENSOR) or maps each display value through the window: invalid ->
 * yellow, below -> cyan-blue, above -> magenta, otherwise palette colour; legend rows are
 * overdrawn. RGB is converted with integer full-range BT.601 coefficients.
 * @return True on success; false with errno EINVAL for invalid input.
 */
bool lwir_render_row(const uint8_t *frame, size_t bytes, unsigned width, unsigned height, unsigned row,
                       const struct lwir_render_options *options, const struct lwir_render_range *range,
                       uint8_t *output, size_t capacity)
{
    unsigned display_width, display_height;
    if (!render_frame_valid(frame, bytes, width, height, options) || !lwir_render_dimensions(width, height, options, &display_width, &display_height) || !range ||
        !output || row >= display_height || capacity < (size_t)display_width * 3 || range->low >= range->high ||
        range->low < -65536 || range->high > 131070 ||
        !valid_profile(options->profile, width, height) ||
        (options->legend && (options->view == LWIR_SENSOR || display_width < 112 || display_height < 24))) { errno = EINVAL; return false; }
    ptrdiff_t step;
    ptrdiff_t source = row_source(width, height, options, display_width, display_height, row, &step);
    if (options->view == LWIR_SENSOR) { sensor_row(frame, source, step, display_width, output); return true; }
    unsigned column = 0;
#if defined(__aarch64__) && !defined(LWIR_DISABLE_NEON)
    if ((step == 1 || step == -1) && !options->profile && !options->denoise_raw && options->palette != LWIR_IRON &&
        range->low >= 0 && range->high <= UINT16_MAX && (!options->legend || row < display_height - 16)) {
        column = thermal_neon(frame + (size_t)width * height * 2, source, step, display_width, options, range, output);
        source += (ptrdiff_t)column * step;
    }
#endif
    for (; column < display_width; ++column, source += step) {
        const size_t pixel = (size_t)source;
        uint8_t *destination = output + column * 3;
        uint16_t raw = 0;
        const bool valid = display_sample(frame, width, height, pixel, options, &raw);
        const int32_t value = corrected(raw, options->profile);
        unsigned red, green, blue;
        if (!valid) { red = 255; green = 255; blue = 0; }
        else if (value < range->low) { red = 0; green = 180; blue = 255; }
        else if (value > range->high) { red = 255; green = 0; blue = 255; }
        else {
            const unsigned intensity = (unsigned)((int64_t)(value - range->low) * 255 / (range->high - range->low));
            color(intensity, options->palette, &red, &green, &blue);
        }
        if (options->legend && row >= display_height - 16) legend_pixel(row, column, display_width, display_height, options, range, &red, &green, &blue);
        /* Full-range BT.601 (JFIF) in /256 fixed point:
         *   Y  =  0.299 R + 0.587 G + 0.114 B          -> 77, 150, 29 (sum 256)
         *   Cb = -0.1687 R - 0.3313 G + 0.5 B + 128    -> -43, -85, 128 (sum 0)
         *   Cr =  0.5 R - 0.4187 G - 0.0813 B + 128    -> 128, -107, -21 (sum 0)
         * 32896 = 128*256 + 128 adds the chroma offset plus rounding. Coefficient sums make
         * grey map to Y = intensity, Cb = Cr = 128 exactly, and keep numerators >= 0 so C's
         * truncating division is a floor. NEON flag colours are these formulas' outputs. */
        destination[0] = (uint8_t)((77 * red + 150 * green + 29 * blue + 128) / 256);
        destination[1] = (uint8_t)(((int)blue * 128 - (int)red * 43 - (int)green * 85 + 32896) / 256);
        destination[2] = (uint8_t)(((int)red * 128 - (int)green * 107 - (int)blue * 21 + 32896) / 256);
    }
    return true;
}
