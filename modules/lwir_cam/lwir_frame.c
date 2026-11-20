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
 * @brief Frame geometry, structural/live qualification and native temperature measurements.
 * @details Operates on borrowed Tiny1-C combined frames: YUYV image plane (`Y0 U Y1 V` per
 * pixel pair) followed by a thermal plane of little-endian uint16 Kelvin*64 codes, where
 * 0 and 65535 are endpoint/invalid codes. Everything is in native, unrotated coordinates;
 * display orientation and correction profiles (lwir_render.c) never affect these results.
 *
 * Pipeline role: capture calls lwir_frame_capture_valid() to drop camera transition fill
 * and frames without temperatures; lwir_frame_plausible()/lwir_frame_settled() are optional
 * heuristics; lwir_measure_temperature() serves point, line and rectangle requests.
 *
 * The scalar measurement code is the reference. On little-endian ARM64 NEON handles
 * eight-pixel rectangle blocks with identical results: exact rounded integer averages,
 * first-occurrence extrema and the same coverage counts (parity-tested). No heap is used.
 */
#include "lwir_frame.h"
#include "exifite.h"
#include "lwir_render.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if defined(__aarch64__) && defined(__AARCH64EL__) && !defined(LWIR_DISABLE_NEON)
#include <arm_neon.h>
#endif

/**
 * @brief Combined frame size for supported geometry.
 * @details Two planes of 2 bytes per pixel. Width must be even because YUYV stores pixel pairs.
 * @return width * height * 4, or 0 for unsupported geometry (also the geometry check used
 *         throughout this module).
 */
size_t lwir_frame_bytes(unsigned width, unsigned height)
{
    if (width < 2 || width > LWIR_MAX_WIDTH || width % 2 != 0 ||
        height == 0 || height > LWIR_MAX_HEIGHT) {
        return 0;
    }
    return (size_t)width * height * 4;
}

/**
 * @brief Check that a non-empty rectangle lies entirely inside the image.
 * @return True when every sample is in bounds.
 * @note Extents are compared as `width <= image_width - x` so `x + width` cannot wrap.
 */
bool lwir_roi_valid(unsigned width, unsigned height, const struct lwir_roi *roi)
{
    return lwir_frame_bytes(width, height) != 0 && roi != NULL &&
        roi->width != 0 && roi->height != 0 && roi->x < width && roi->y < height &&
        roi->width <= width - roi->x && roi->height <= height - roi->y;
}

/**
 * @brief Validate a measurement request against native geometry.
 * @details Rectangles must be in bounds, or entirely zero to mean the full image (a zero
 * width with any other non-zero field is rejected as ambiguous). Points need an in-bounds
 * start; lines need in-bounds start and end.
 * @return True when the request can be measured.
 */
bool lwir_measurement_valid(unsigned width, unsigned height, const struct lwir_measurement *measurement)
{
    if (!measurement || !lwir_frame_bytes(width, height)) return false;
    if (measurement->kind == LWIR_RECTANGLE) {
        if (!measurement->roi.width) return !measurement->roi.x && !measurement->roi.y && !measurement->roi.height;
        return lwir_roi_valid(width, height, &measurement->roi);
    }
    if (measurement->kind != LWIR_POINT && measurement->kind != LWIR_LINE) return false;
    return measurement->roi.x < width && measurement->roi.y < height &&
        (measurement->kind == LWIR_POINT || (measurement->end_x < width && measurement->end_y < height));
}

/**
 * @brief Read one little-endian thermal code.
 * @param plane Thermal plane start.
 * @param width Native width.
 * @param column Native column (caller-checked).
 * @param row Native row (caller-checked).
 * @return Raw Kelvin*64 code, including endpoint codes.
 */
static uint16_t thermal_sample(const uint8_t *plane, unsigned width, unsigned column, unsigned row)
{
    const size_t offset = ((size_t)row * width + column) * 2;
    return (uint16_t)plane[offset] | (uint16_t)plane[offset + 1] << 8;
}

/**
 * @brief Measured value of one center, optionally 3x3-averaged.
 * @details Averages the valid (non-endpoint) codes of the 3x3 neighbourhood clipped at
 * the sensor edges, rounded half up: `(sum + n/2) / n`. The valid center guarantees n >= 1.
 * @param plane Thermal plane start.
 * @param width Native width.
 * @param height Native height.
 * @param column Center column.
 * @param row Center row.
 * @param average Whether averaging is requested.
 * @param center The center's raw code.
 * @return center unchanged when not averaging or when it is an endpoint code; otherwise
 *         the rounded mean, which stays within 1..65534.
 */
static uint16_t measurement_sample(const uint8_t *plane, unsigned width, unsigned height,
                                    unsigned column, unsigned row, bool average, uint16_t center)
{
    if (!average || !center || center == UINT16_MAX) return center;
    const unsigned left = column ? column - 1 : 0, top = row ? row - 1 : 0;
    const unsigned right = column + 1 < width ? column + 1 : column;
    const unsigned bottom = row + 1 < height ? row + 1 : row;
    uint32_t sum = 0; unsigned samples = 0;
    /* Clip the 3x3 neighbourhood to the sensor; never synthesize invalid centers. */
    for (unsigned neighbour_y = top; neighbour_y <= bottom; ++neighbour_y) {
        for (unsigned neighbour_x = left; neighbour_x <= right; ++neighbour_x) {
            const uint16_t raw = thermal_sample(plane, width, neighbour_x, neighbour_y);
            if (raw && raw != UINT16_MAX) { sum += raw; ++samples; }
        }
    }
    return (uint16_t)((sum + samples / 2) / samples);
}

/**
 * @brief Add one measured sample to running statistics.
 * @details Strict comparisons keep the first visited extremum, matching the NEON path.
 * Sums and extrema include endpoint codes; coverage counts exclude them and use the
 * ORIGINAL center, so averaging cannot move a pixel between coverage bins.
 * @param result Statistics being accumulated.
 * @param raw Measured sample (possibly averaged).
 * @param column Native column, recorded for extrema.
 * @param row Native row, recorded for extrema.
 * @param thresholds Optional coverage band.
 * @param center Original center code used for coverage.
 * @note Endpoint status of raw and center always agree, because measurement_sample()
 * returns endpoint centers unchanged and never averages to an endpoint code.
 */
static void accumulate_temperature(struct lwir_temperature_stats *result, uint16_t raw, unsigned column,
                                    unsigned row, const struct lwir_thresholds *thresholds, uint16_t center)
{
    if (!result->pixels || raw < result->min_raw) {
        result->min_raw = raw; result->min_x = column; result->min_y = row;
    }
    if (!result->pixels || raw > result->max_raw) {
        result->max_raw = raw; result->max_x = column; result->max_y = row;
    }
    result->sum_raw += raw;
    result->sum_squared_raw += (uint64_t)raw * raw;
    const bool endpoint = !raw || raw == UINT16_MAX;
    result->endpoint_codes += endpoint;
    ++result->pixels;
    /* Integer comparisons reuse this pass; display/profile/denoise never affect coverage. */
    if (thresholds && thresholds->enabled && !endpoint) {
        result->below += center < thresholds->low;
        result->above += center > thresholds->high;
        result->within += center >= thresholds->low && center <= thresholds->high;
    }
}

#if defined(__aarch64__) && defined(__AARCH64EL__) && !defined(LWIR_DISABLE_NEON)
/**
 * @brief Load eight consecutive thermal codes of one row.
 * @details VLD2 splits low/high bytes; widening and `low | high << 8` rebuilds the
 * little-endian codes.
 * @param plane Thermal plane start.
 * @param width Native width.
 * @param column First column; column + 7 must be in the row.
 * @param row Native row.
 * @return Raw codes for columns column..column+7.
 */
static uint16x8_t thermal_block(const uint8_t *plane, unsigned width, unsigned column, unsigned row)
{
    const uint8x8x2_t bytes = vld2_u8(plane + ((size_t)row * width + column) * 2);
    return vorrq_u16(vmovl_u8(bytes.val[0]), vshlq_n_u16(vmovl_u8(bytes.val[1]), 8));
}

/**
 * @brief Lane mask of valid (non-endpoint) codes.
 * @param raw Eight raw codes.
 * @return 0xFFFF where 0 < code < 65535, else 0.
 */
static uint16x8_t valid_block(uint16x8_t raw)
{
    return vandq_u16(vcgtq_u16(raw, vdupq_n_u16(0)), vcltq_u16(raw, vdupq_n_u16(UINT16_MAX)));
}

/**
 * @brief Lane index of the first lane equal to value.
 * @details The 0xFFFF/0 compare mask is viewed as two 64-bit halves; the count of trailing
 * zero bits divided by 16 is the lane index within a half, so the lowest (first-in-raster)
 * matching lane wins, as in the scalar path.
 * @param samples Eight samples.
 * @param value A value known to occur in samples (a vminv/vmaxv result).
 * @return Lane 0..7.
 * @warning ctz of zero is undefined; callers must guarantee a match exists.
 */
static unsigned first_equal(uint16x8_t samples, uint16_t value)
{
    const uint16x8_t equal = vceqq_u16(samples, vdupq_n_u16(value));
    const uint64_t low = vget_lane_u64(vreinterpret_u64_u16(vget_low_u16(equal)), 0);
    const uint64_t high = vget_lane_u64(vreinterpret_u64_u16(vget_high_u16(equal)), 0);
    return low ? (unsigned)__builtin_ctzll(low) / 16 : 4U + (unsigned)__builtin_ctzll(high) / 16;
}

/**
 * @brief Add eight measured samples to running statistics, matching accumulate_temperature().
 * @details Horizontal min/max with first_equal() preserve first-occurrence coordinates;
 * strict comparisons keep earlier blocks on ties. Squares use widening u16 x u16 -> u32
 * multiplies (65535^2 fits u32) summed into u64. Coverage uses the original centers.
 * @param result Statistics being accumulated.
 * @param samples Measured samples (raw or averaged).
 * @param centers Original center codes.
 * @param column Native column of lane 0.
 * @param row Native row.
 * @param thresholds Optional coverage band.
 */
static void accumulate_block(struct lwir_temperature_stats *result, uint16x8_t samples, uint16x8_t centers,
                              unsigned column, unsigned row, const struct lwir_thresholds *thresholds)
{
    const uint16_t minimum = vminvq_u16(samples), maximum = vmaxvq_u16(samples);
    if (!result->pixels || minimum < result->min_raw) {
        result->min_raw = minimum; result->min_x = column + first_equal(samples, minimum); result->min_y = row;
    }
    if (!result->pixels || maximum > result->max_raw) {
        result->max_raw = maximum; result->max_x = column + first_equal(samples, maximum); result->max_y = row;
    }
    result->sum_raw += vaddlvq_u16(samples);
    result->sum_squared_raw += vaddlvq_u32(vmull_u16(vget_low_u16(samples), vget_low_u16(samples)));
    result->sum_squared_raw += vaddlvq_u32(vmull_u16(vget_high_u16(samples), vget_high_u16(samples)));
    const uint16x8_t valid = valid_block(centers);
    /* 0xFFFF lanes >> 15 become 1, so a horizontal add counts true lanes. */
    const unsigned valid_count = vaddlvq_u16(vshrq_n_u16(valid, 15));
    result->pixels += 8; result->endpoint_codes += 8 - valid_count;
    if (thresholds && thresholds->enabled) {
        const unsigned below = vaddlvq_u16(vshrq_n_u16(vandq_u16(valid, vcltq_u16(centers, vdupq_n_u16(thresholds->low))), 15));
        const unsigned above = vaddlvq_u16(vshrq_n_u16(vandq_u16(valid, vcgtq_u16(centers, vdupq_n_u16(thresholds->high))), 15));
        result->below += below; result->above += above; result->within += valid_count - below - above;
    }
}

/**
 * @brief Exact per-lane `floor(numerator / denominator)` without integer vector division.
 * @details 1. Estimate q = trunc(float(n) / float(d)); n < 2^24 converts exactly and the
 * correctly rounded division can only land q one unit away from the true floor.
 * 2. If q * d > n, subtract 1. 3. If (q + 1) * d <= n, add 1. Compare masks >> 31 give
 * 0 or 1. Products stay far below 2^32 for these ranges.
 * @param numerator Rounded sums, at most 589810 (nine 65534 codes plus count/2).
 * @param denominator Valid-sample counts 1..9.
 * @return Exact floor quotients.
 */
static uint32x4_t exact_block_divide(uint32x4_t numerator, uint32x4_t denominator)
{
    /* These integers are <=589810. Float estimates can miss by at most one;
     * integer product checks restore the EXACT floor quotient, including boundaries. */
    uint32x4_t quotient = vcvtq_u32_f32(vdivq_f32(vcvtq_f32_u32(numerator), vcvtq_f32_u32(denominator)));
    quotient = vsubq_u32(quotient, vshrq_n_u32(vcgtq_u32(vmulq_u32(quotient, denominator), numerator), 31));
    const uint32x4_t next = vaddq_u32(quotient, vdupq_n_u32(1));
    return vaddq_u32(quotient, vshrq_n_u32(vcleq_u32(vmulq_u32(next, denominator), numerator), 31));
}

/**
 * @brief 3x3 valid-sample average for eight consecutive interior centers.
 * @details Each of the (up to) 3x3 shifted eight-lane loads supplies one neighbour per
 * lane. Endpoint codes are masked to zero and not counted; rows are clipped at the top and
 * bottom edges. Adding count/2 before exact_block_divide() reproduces the scalar
 * round-half-up mean. Lanes whose center is an endpoint code keep the center unchanged
 * (their count may be zero, hence the max(count, 1) guard).
 * @param plane Thermal plane start.
 * @param width Native width.
 * @param height Native height.
 * @param column Column of lane 0; requires column >= 1 and column + 8 < width.
 * @param row Native row.
 * @param centers Original codes at column..column+7.
 * @return Averaged samples, identical to measurement_sample() per lane.
 */
static uint16x8_t average_block(const uint8_t *plane, unsigned width, unsigned height, unsigned column, unsigned row, uint16x8_t centers)
{
    uint32x4_t low_sum = vdupq_n_u32(0), high_sum = low_sum;
    uint16x8_t count = vdupq_n_u16(0);
    const unsigned top = row ? row - 1 : 0, bottom = row + 1 < height ? row + 1 : row;
    /* Caller reserves one real neighbor column on each side; borders use scalar clipping. */
    for (unsigned vertical = top; vertical <= bottom; ++vertical) {
        for (unsigned horizontal = column - 1; horizontal <= column + 1; ++horizontal) {
            const uint16x8_t samples = thermal_block(plane, width, horizontal, vertical), valid = valid_block(samples);
            const uint16x8_t accepted = vandq_u16(samples, valid);
            low_sum = vaddw_u16(low_sum, vget_low_u16(accepted)); high_sum = vaddw_u16(high_sum, vget_high_u16(accepted));
            count = vaddq_u16(count, vshrq_n_u16(valid, 15));
        }
    }
    const uint16x8_t safe_count = vmaxq_u16(count, vdupq_n_u16(1));
    low_sum = vaddw_u16(low_sum, vget_low_u16(vshrq_n_u16(count, 1)));
    high_sum = vaddw_u16(high_sum, vget_high_u16(vshrq_n_u16(count, 1)));
    const uint16x8_t averaged = vcombine_u16(vmovn_u32(exact_block_divide(low_sum, vmovl_u16(vget_low_u16(safe_count)))),
        vmovn_u32(exact_block_divide(high_sum, vmovl_u16(vget_high_u16(safe_count)))));
    return vbslq_u16(valid_block(centers), averaged, centers);
}
#endif

/**
 * @brief Measure a point, line or rectangle in native coordinates.
 * @details Lines use integer Bresenham traversal of every center from start to end
 * inclusive. Rectangles are visited in raster order; on ARM64 NEON handles eight-center
 * blocks and scalar code handles the first column (when averaging), the right border and
 * the tail. Results are built in a local copy and published only on success.
 * @return False for NULL pointers, invalid request/geometry, size mismatch or an empty
 *         threshold band; errno is not set.
 */
bool lwir_measure_temperature(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                                const struct lwir_measurement *measurement, const struct lwir_thresholds *thresholds,
                                struct lwir_temperature_stats *statistics)
{
    if (!frame || !statistics || !lwir_measurement_valid(width, height, measurement) ||
        bytes != lwir_frame_bytes(width, height) ||
        (thresholds && thresholds->enabled && thresholds->low >= thresholds->high)) return false;
    struct lwir_temperature_stats result = {0};
    const uint8_t *plane = frame + bytes / 2;
    if (measurement->kind == LWIR_LINE) {
        int column = (int)measurement->roi.x, row = (int)measurement->roi.y;
        const int end_x = (int)measurement->end_x, end_y = (int)measurement->end_y;
        /* All-octant error form: delta_x = |dx| and delta_y = -|dy| (deliberately negative),
         * error = delta_x + delta_y tracks the signed distance from the ideal line. */
        const int delta_x = end_x > column ? end_x - column : column - end_x;
        const int delta_y = end_y > row ? row - end_y : end_y - row;
        const int step_x = column < end_x ? 1 : -1, step_y = row < end_y ? 1 : -1;
        int error = delta_x + delta_y;
        /* Inclusive Bresenham traversal: max(|dx|,|dy|)+1 centers (<=640), no allocations
         * or float geometry. */
        for (;;) {
            const uint16_t center = thermal_sample(plane, width, (unsigned)column, (unsigned)row);
            const uint16_t raw = measurement_sample(plane, width, height, (unsigned)column, (unsigned)row, measurement->average_3x3, center);
            accumulate_temperature(&result, raw, (unsigned)column, (unsigned)row, thresholds, center);
            if (column == end_x && row == end_y) break;
            /* Doubling avoids the half-pixel midpoint. Stepping x when 2e >= dy and y when
             * 2e <= dx; both may fire, giving a diagonal step. */
            const int twice = error * 2;
            if (twice >= delta_y) { error += delta_y; column += step_x; }
            if (twice <= delta_x) { error += delta_x; row += step_y; }
        }
    } else {
        struct lwir_roi region = measurement->roi;
        if (measurement->kind == LWIR_POINT) { region.width = 1; region.height = 1; }
        else if (!region.width) { region.width = width; region.height = height; }
        for (unsigned row = region.y; row < region.y + region.height; ++row) {
            unsigned column = region.x;
#if defined(__aarch64__) && defined(__AARCH64EL__) && !defined(LWIR_DISABLE_NEON)
            /* Eight native centers per block, no staging frame. Scalar borders/tails
             * preserve clipped neighborhoods and the identical raw-coordinate contract. */
            /* Column 0 has no left neighbour for the shifted loads, so it goes scalar. */
            if (measurement->average_3x3 && !column) {
                const uint16_t center = thermal_sample(plane, width, column, row);
                accumulate_temperature(&result, measurement_sample(plane, width, height, column, row, true, center), column, row, thresholds, center);
                ++column;
            }
            /* Averaging needs column + 8 (right neighbour of lane 7) inside the row. */
            for (; region.x + region.width - column >= 8 && (!measurement->average_3x3 || column + 8 < width); column += 8) {
                const uint16x8_t centers = thermal_block(plane, width, column, row);
                const uint16x8_t samples = measurement->average_3x3 ? average_block(plane, width, height, column, row, centers) : centers;
                accumulate_block(&result, samples, centers, column, row, thresholds);
            }
#endif
            for (; column < region.x + region.width; ++column) {
                const uint16_t center = thermal_sample(plane, width, column, row);
                const uint16_t raw = measurement_sample(plane, width, height, column, row, measurement->average_3x3, center);
                accumulate_temperature(&result, raw, column, row, thresholds, center);
            }
        }
    }
    *statistics = result;
    return true;
}

/**
 * @brief Rectangle statistics without averaging or thresholds.
 * @return False for an invalid roi or any lwir_measure_temperature() failure.
 */
bool lwir_temperature_statistics(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                                   const struct lwir_roi *roi, struct lwir_temperature_stats *statistics)
{
    struct lwir_measurement measurement = {0};
    if (roi) {
        if (!lwir_roi_valid(width, height, roi)) return false;
        measurement.roi = *roi;
    }
    return lwir_measure_temperature(frame, bytes, width, height, &measurement, NULL, statistics);
}

/**
 * @brief Absolute difference of two bytes without signed overflow concerns.
 * @param first First byte.
 * @param second Second byte.
 * @return |first - second|, 0..255.
 */
static unsigned byte_difference(uint8_t first, uint8_t second)
{
    return first > second ? (unsigned)first - second : (unsigned)second - first;
}

/**
 * @brief Structural check: non-NULL, supported geometry, exact combined size.
 * @return True for a structurally valid combined frame; says nothing about content.
 */
bool lwir_frame_valid(const uint8_t *frame, size_t bytes, unsigned width, unsigned height)
{
    const size_t expected = lwir_frame_bytes(width, height);
    return frame != NULL && expected != 0 && bytes == expected;
}

/**
 * @brief Detect the camera's whole-frame neutral fill.
 * @details Scans both planes as byte pairs; any pair other than (0x00, 0x80) ends the scan
 * early, so real frames are usually rejected after a few bytes.
 * @return True only for a valid frame consisting entirely of fill.
 */
bool lwir_frame_is_neutral_fill(const uint8_t *frame, size_t bytes, unsigned width, unsigned height)
{
    if (!lwir_frame_valid(frame, bytes, width, height)) return false;
    for (size_t offset = 0; offset < bytes; offset += 2)
        if (frame[offset] != 0 || frame[offset + 1] != 128) return false;
    return true;
}

/**
 * @brief Live-capture qualification.
 * @details Rejects structurally invalid frames, neutral fill and frames whose thermal plane
 * holds only endpoint codes; stops at the first valid code. Uniform real scenes pass.
 * @return True when the frame carries at least one usable temperature.
 */
bool lwir_frame_capture_valid(const uint8_t *frame, size_t bytes, unsigned width, unsigned height)
{
    if (!lwir_frame_valid(frame, bytes, width, height) || lwir_frame_is_neutral_fill(frame, bytes, width, height)) return false;
    for (size_t offset = bytes / 2; offset < bytes; offset += 2) {
        const uint16_t raw = (uint16_t)frame[offset] | (uint16_t)frame[offset + 1] << 8;
        if (raw && raw != UINT16_MAX) return true;
    }
    return false;
}

/**
 * @brief Conservative scene-plausibility heuristic.
 * @details
 * 1. Chroma: every odd image byte is U or V; more than 1% outside 128 +/- 8 fails.
 * 2. Roughness: sum of |Y(x) - Y(x+1)| must average at most 45 per adjacent pair.
 * 3. Temperature: every eighth thermal sample; more than 10% outside raw 14000..40000
 *    (about -54..352 degC, endpoint codes included) fails.
 * @return True when all checks pass. Unusual but genuine scenes may fail.
 */
bool lwir_frame_plausible(const uint8_t *frame, size_t bytes, unsigned width, unsigned height)
{
    if (!lwir_frame_valid(frame, bytes, width, height)) return false;
    const size_t pixels = (size_t)width * height;
    size_t invalid_chroma = 0;
    uint64_t roughness = 0;
    for (size_t row = 0; row < height; ++row) {
        for (size_t column = 0; column < width; ++column) {
            const size_t offset = (row * width + column) * 2;
            /* offset + 1 alternates U and V; offset + 2 is the next pixel's Y. */
            invalid_chroma += frame[offset + 1] < 120 || frame[offset + 1] > 136;
            if (column + 1 < width) roughness += byte_difference(frame[offset], frame[offset + 2]);
        }
    }
    if (invalid_chroma * 100 > pixels || roughness > (uint64_t)height * (width - 1) * 45) return false;
    size_t invalid_temperatures = 0;
    size_t samples = 0;
    /* Every eighth pixel bounds cost; the 10% tolerance admits some out-of-band samples. */
    for (size_t pixel = 0; pixel < pixels; pixel += 8) {
        const size_t offset = pixels * 2 + pixel * 2;
        const uint16_t temperature = (uint16_t)frame[offset] | (uint16_t)frame[offset + 1] << 8;
        invalid_temperatures += temperature < 14000 || temperature > 40000;
        ++samples;
    }
    return invalid_temperatures * 10 <= samples;
}

/**
 * @brief Image-stability heuristic between consecutive frames.
 * @details Sums absolute luma differences (even image bytes only) and accepts a mean of
 * at most 12 per pixel. Chroma and thermal data are ignored.
 * @return True when settled; false for NULL inputs or unsupported geometry.
 */
bool lwir_frame_settled(const uint8_t *frame, const uint8_t *previous_image,
                         unsigned width, unsigned height)
{
    if (frame == NULL || previous_image == NULL || lwir_frame_bytes(width, height) == 0) return false;
    uint64_t change = 0;
    const size_t pixels = (size_t)width * height;
    for (size_t pixel = 0; pixel < pixels; ++pixel) {
        change += byte_difference(frame[pixel * 2], previous_image[pixel * 2]);
    }
    return change <= pixels * 12;
}

