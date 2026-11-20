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
 * @brief Pose-free thermal components using caller-owned bounded scratch storage.
 *
 * @details Analysis builds only (`LWIR_ENABLE_ANALYSIS=1`). Operates on the exact
 * native Kelvin*64 plane recovered from a saved LWIR JPEG. Algorithm outline:
 * -# Robust frame statistics: exact median (background) and median absolute
 *    deviation (noise) via 65536-bin histograms, so no sorting buffer is needed.
 * -# Global threshold combining an absolute floor with a robust rise over background.
 * -# 8-connected flood fill (breadth-first) of above-threshold pixels.
 * -# Per component: local background, contrast gate, excess-weighted subpixel
 *    centroid and shape features.
 * -# Keep the top two generic and top two tray-like components; report trays if any.
 *
 * Cost trade-off: every component clears and scans the full 65536-bin histogram
 * for its local median, so frames with many small components cost roughly
 * 65536 operations per component in addition to the per-pixel work.
 */
#include "lwir_detection.h"
#include <math.h>

/* Margin in pixels around a component's box used to sample its local background. */
enum { lwir_hotspot_radius = 5 };

/**
 * @brief Decode one unsigned 16-bit little-endian value.
 * @param bytes At least two readable bytes; no alignment requirement.
 * @return The decoded value, independent of host endianness.
 */
static uint16_t read_u16_le(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8);
}

/**
 * @brief Whether a raw code is a real measurement rather than a sensor endpoint code.
 * @details 0 and 65535 (about 751 C, beyond the Tiny1C range) mark invalid or
 * missing samples, never real temperatures, so they must not shape statistics
 * or become hotspots.
 * @param raw Kelvin*64 code.
 * @return True for codes 1..65534.
 */
static bool valid_code(uint16_t raw)
{
    return raw != 0 && raw != UINT16_MAX;
}

/**
 * @brief Test a pixel's visited bit.
 * @param workspace Scratch storage holding the bitset.
 * @param pixel Row-major native index within the current frame.
 * @return True when the pixel was already queued.
 */
static bool bit_is_set(const struct lwir_detection_workspace *workspace, size_t pixel)
{
    return (workspace->seen[pixel >> 3] & (uint8_t)(1U << (pixel & 7U))) != 0;
}

/**
 * @brief Set a pixel's visited bit.
 * @details A bitset (rather than a byte per pixel) keeps the workspace small.
 * @param workspace Scratch storage holding the bitset.
 * @param pixel Row-major native index within the current frame.
 */
static void mark_bit(struct lwir_detection_workspace *workspace, size_t pixel)
{
    workspace->seen[pixel >> 3] |= (uint8_t)(1U << (pixel & 7U));
}

/**
 * @brief Zero all 65536 histogram bins.
 * @param workspace Scratch storage holding the histogram.
 */
static void histogram_clear(struct lwir_detection_workspace *workspace)
{
    for (size_t index = 0; index <= UINT16_MAX; ++index) workspace->histogram[index] = 0;
}

/**
 * @brief Exact median of the values counted in the histogram.
 * @details Walks the cumulative count once to find the lower and upper middle
 * ranks; for an even count the result is their mean, so it can end in .5.
 * Exact for integer inputs, avoiding a sort and a pixel-sized copy.
 * @param workspace Scratch storage whose histogram holds exactly @p count samples.
 * @param count Number of samples; must be at least 1 ((count - 1) would wrap).
 * @return Median value in histogram-bin units.
 */
static double histogram_median(const struct lwir_detection_workspace *workspace, uint64_t count)
{
    const uint64_t lower_index = (count - 1) / 2;
    const uint64_t upper_index = count / 2;
    uint64_t accumulated = 0;
    uint16_t lower = 0, upper = 0;
    bool lower_found = false;
    for (size_t value = 0; value <= UINT16_MAX; ++value) {
        accumulated += workspace->histogram[value];
        if (!lower_found && accumulated > lower_index) { lower = (uint16_t)value; lower_found = true; }
        if (accumulated > upper_index) { upper = (uint16_t)value; break; }
    }
    return ((double)lower + upper) / 2.0;
}

/**
 * @brief Ranking predicate for retained candidates.
 * @details Generic order: higher contrast first, then larger excess heat.
 * Tray order: larger excess heat first (presumably favouring an extended hot
 * tray over a small very hot point), then contrast. A full tie returns false, so the
 * earlier-found component (row-major seed order) keeps its place.
 * @param candidate Newly evaluated component.
 * @param current Retained component it is compared against.
 * @param tray_order Select the tray ordering instead of the generic one.
 * @return True when @p candidate should rank strictly above @p current.
 */
static bool candidate_better(const struct lwir_hotspot *candidate,
                             const struct lwir_hotspot *current, bool tray_order)
{
    if (tray_order && candidate->excess_celsius_pixels != current->excess_celsius_pixels)
        return candidate->excess_celsius_pixels > current->excess_celsius_pixels;
    if (candidate->contrast_celsius != current->contrast_celsius)
        return candidate->contrast_celsius > current->contrast_celsius;
    return candidate->excess_celsius_pixels > current->excess_celsius_pixels;
}

/**
 * @brief Insert a candidate into a sorted top-two list.
 * @details Insertion sort over a fixed two-slot array: while fewer than two are
 * held the list grows; afterwards a candidate must beat the current second place
 * to enter, and the displaced second place is dropped.
 * @param candidates Sorted list, best first.
 * @param[in,out] count Valid entries (0..2); incremented while below 2.
 * @param candidate Component to consider; copied by value.
 * @param tray_order Ordering passed to candidate_better().
 */
static void retain_candidate(struct lwir_hotspot candidates[2], size_t *count,
                             const struct lwir_hotspot *candidate, bool tray_order)
{
    size_t position = *count;
    if (position < 2) ++*count;
    else if (!candidate_better(candidate, &candidates[1], tray_order)) return;
    if (position > 1) position = 1;
    while (position > 0 && candidate_better(candidate, &candidates[position - 1], tray_order)) {
        if (position < 2) candidates[position] = candidates[position - 1];
        --position;
    }
    candidates[position] = *candidate;
}

/**
 * @brief Detect and rank hot connected components in a native temperature plane.
 * @details See lwir_detection.h for the contract and the file overview for the
 * algorithm. All arithmetic stays in raw Kelvin*64 codes until results are
 * converted to Celsius (code / 64 - 273.15); temperature differences convert
 * by dividing by 64 only.
 * @return True for a valid plane (even with zero candidates); false for invalid arguments.
 */
bool lwir_detect_hotspots(const uint8_t *temperature, size_t bytes, unsigned width, unsigned height,
                          struct lwir_detection_workspace *workspace, struct lwir_hotspot_set *result)
{
    if (!temperature || !workspace || !result || !width || !height || width > LWIR_MAX_WIDTH ||
        height > LWIR_MAX_HEIGHT || bytes != (size_t)width * height * 2) return false;
    const size_t pixels = (size_t)width * height;
    /* Frame median over valid codes only: robust against hotspots occupying a minority of pixels. */
    histogram_clear(workspace);
    uint64_t valid_pixels = 0;
    for (size_t pixel = 0; pixel < pixels; ++pixel) {
        const uint16_t raw = read_u16_le(temperature + pixel * 2);
        if (valid_code(raw)) { ++workspace->histogram[raw]; ++valid_pixels; }
    }
    /* A plane without any real measurement is valid input with nothing to detect. */
    if (!valid_pixels) {
        *result = (struct lwir_hotspot_set){.background_celsius = NAN, .threshold_celsius = NAN};
        return true;
    }
    const double background_raw = histogram_median(workspace, valid_pixels);
    /* Median absolute deviation, binned to the nearest whole code, as a noise
     * estimate unaffected by the hot tail. Very large deviations share the top bin. */
    histogram_clear(workspace);
    for (size_t pixel = 0; pixel < pixels; ++pixel) {
        const uint16_t raw = read_u16_le(temperature + pixel * 2);
        if (!valid_code(raw)) continue;
        const double deviation = fabs((double)raw - background_raw);
        const unsigned bucket = (unsigned)(deviation + 0.5);
        ++workspace->histogram[bucket <= UINT16_MAX ? bucket : UINT16_MAX];
    }
    const double noise_raw = histogram_median(workspace, valid_pixels);
    /* Preserve the existing mission policy; this ownership change is not detector tuning. */
    /* Threshold in raw codes, step by step:
     *   minimum = 60 C absolute floor, presumably so ordinary warm ground on a
     *             low-noise scene cannot trigger detections;
     *   rise    = max(20 C, 6 * sigma), sigma = 1.4826 * MAD (Gaussian-consistent);
     *   result  = max(minimum, median + rise).
     * 20 C appears as 20 * 64 because a temperature difference needs no 273.15 offset. */
    const double minimum_raw = (60.0 + 273.15) * 64.0;
    const double rise_raw = fmax(20.0 * 64.0, 6.0 * 1.4826 * noise_raw);
    const double threshold_raw = fmax(minimum_raw, background_raw + rise_raw);
    for (size_t index = 0; index < (pixels + 7) / 8; ++index) workspace->seen[index] = 0;
    struct lwir_hotspot generic[2] = {{0}}, trays[2] = {{0}};
    size_t generic_count = 0, tray_count = 0;
    size_t generic_top_count = 0, tray_top_count = 0;
    for (size_t seed = 0; seed < pixels; ++seed) {
        /* Each unvisited above-threshold pixel seeds one 8-connected component.
         * Code 0 can never reach the >= 60 C threshold, so only 65535 needs exclusion. */
        const uint16_t seed_raw = read_u16_le(temperature + seed * 2);
        if (bit_is_set(workspace, seed) || seed_raw == UINT16_MAX || seed_raw < threshold_raw) continue;
        /* The queue restarts at 0 per component; after the fill, queue[0..tail)
         * lists exactly this component's pixels for the centroid pass below. */
        size_t head = 0, tail = 1;
        workspace->queue[0] = (uint32_t)seed;
        mark_bit(workspace, seed);
        struct lwir_hotspot candidate = {0};
        candidate.min_x = candidate.max_x = candidate.peak_x = seed % width;
        candidate.min_y = candidate.max_y = candidate.peak_y = seed / width;
        uint16_t peak = read_u16_le(temperature + seed * 2);
        uint64_t sum_raw = 0;
        while (head < tail) {
            const size_t pixel = workspace->queue[head++];
            const size_t x = pixel % width, y = pixel / width;
            const uint16_t raw = read_u16_le(temperature + pixel * 2);
            ++candidate.pixels;
            sum_raw += raw;
            if (x < candidate.min_x) candidate.min_x = x;
            if (x > candidate.max_x) candidate.max_x = x;
            if (y < candidate.min_y) candidate.min_y = y;
            if (y > candidate.max_y) candidate.max_y = y;
            if (raw > peak) { peak = raw; candidate.peak_x = x; candidate.peak_y = y; }
            const size_t min_y = y ? y - 1 : 0, max_y = y + 1 < height ? y + 1 : y;
            const size_t min_x = x ? x - 1 : 0, max_x = x + 1 < width ? x + 1 : x;
            for (size_t neighbor_y = min_y; neighbor_y <= max_y; ++neighbor_y) {
                for (size_t neighbor_x = min_x; neighbor_x <= max_x; ++neighbor_x) {
                    const size_t neighbor = neighbor_y * width + neighbor_x;
                    const uint16_t neighbor_raw = read_u16_le(temperature + neighbor * 2);
                    if (!bit_is_set(workspace, neighbor) && neighbor_raw != UINT16_MAX && neighbor_raw >= threshold_raw) {
                        /* Defensive: marking makes overflow impossible, but never write past the queue. */
                        if (tail >= pixels) return false;
                        /* Mark before enqueueing: each pixel occupies at most one queue slot. */
                        mark_bit(workspace, neighbor);
                        workspace->queue[tail++] = (uint32_t)neighbor;
                    }
                }
            }
        }
        /* Local background: median of sub-threshold pixels in the box grown by
         * lwir_hotspot_radius, clamped to the frame. Excluding all hot pixels
         * (including neighbouring components) keeps the estimate from being
         * pulled up by the object itself. */
        const size_t box_min_x = candidate.min_x > lwir_hotspot_radius ? candidate.min_x - lwir_hotspot_radius : 0;
        const size_t box_min_y = candidate.min_y > lwir_hotspot_radius ? candidate.min_y - lwir_hotspot_radius : 0;
        const size_t box_max_x = candidate.max_x + lwir_hotspot_radius < width ? candidate.max_x + lwir_hotspot_radius : width - 1;
        const size_t box_max_y = candidate.max_y + lwir_hotspot_radius < height ? candidate.max_y + lwir_hotspot_radius : height - 1;
        histogram_clear(workspace);
        uint64_t background_count = 0;
        for (size_t y = box_min_y; y <= box_max_y; ++y) for (size_t x = box_min_x; x <= box_max_x; ++x) {
            const uint16_t raw = read_u16_le(temperature + (y * width + x) * 2);
            if (valid_code(raw) && raw < threshold_raw) { ++workspace->histogram[raw]; ++background_count; }
        }
        const double local_background_raw = background_count ? histogram_median(workspace, background_count) : background_raw;
        /* Centroid weights are the excess over local background, clamped at zero:
         * x_c = sum(x * w) / sum(w), w = max(0, raw - background). Hotter pixels pull
         * the centre, giving subpixel precision; sum(w) doubles as the heat score. */
        double weighted_x = 0, weighted_y = 0, excess_raw_pixels = 0;
        for (size_t index = 0; index < tail; ++index) {
            const size_t pixel = workspace->queue[index];
            const double weight = fmax(0, (double)read_u16_le(temperature + pixel * 2) - local_background_raw);
            weighted_x += (pixel % width) * weight;
            weighted_y += (pixel / width) * weight;
            excess_raw_pixels += weight;
        }
        const double peak_celsius = peak / 64.0 - 273.15;
        const double local_celsius = local_background_raw / 64.0 - 273.15;
        const double contrast = peak_celsius - local_celsius;
        /* Reject components that are hot overall but barely stand out locally
         * (e.g. a large warm surface); they are not counted in generic_count. */
        if (contrast < 15.0 || excess_raw_pixels <= 0) continue;
        candidate.center_x = weighted_x / excess_raw_pixels;
        candidate.center_y = weighted_y / excess_raw_pixels;
        candidate.peak_celsius = peak_celsius;
        candidate.mean_celsius = ((double)sum_raw / candidate.pixels) / 64.0 - 273.15;
        candidate.local_background_celsius = local_celsius;
        candidate.contrast_celsius = contrast;
        candidate.excess_celsius_pixels = excess_raw_pixels / 64.0;
        candidate.touches_edge = candidate.min_x == 0 || candidate.min_y == 0 || candidate.max_x + 1 == width || candidate.max_y + 1 == height;
        const double span_x = candidate.max_x - candidate.min_x + 1.0;
        const double span_y = candidate.max_y - candidate.min_y + 1.0;
        candidate.fill_ratio = candidate.pixels / (span_x * span_y);
        candidate.aspect_ratio = fmax(span_x, span_y) / fmin(span_x, span_y);
        /* Fire-tray heuristic: fully inside the frame, at least 2x2 and 4 pixels
         * (so a single hot pixel cannot qualify), peak >= 100 C, contrast >= 40 C,
         * reasonably filled box (>= 25 %) and not elongated (aspect <= 3). Limits
         * apply to apparent, uncorrected temperatures. */
        candidate.tray_like = !candidate.touches_edge && candidate.pixels >= 4 && span_x >= 2 && span_y >= 2
            && candidate.peak_celsius >= 100 && candidate.contrast_celsius >= 40
            && candidate.fill_ratio >= 0.25 && candidate.aspect_ratio <= 3.0;
        ++generic_count;
        retain_candidate(generic, &generic_top_count, &candidate, false);
        if (candidate.tray_like) {
            ++tray_count;
            retain_candidate(trays, &tray_top_count, &candidate, true);
        }
    }
    *result = (struct lwir_hotspot_set){0};
    result->background_celsius = background_raw / 64.0 - 273.15;
    result->threshold_celsius = threshold_raw / 64.0 - 273.15;
    result->generic_count = generic_count;
    result->tray_count = tray_count;
    /* Any tray-like component switches the whole report to the tray ranking;
     * otherwise the generic ranking is the fallback. */
    result->tray_pass_selected = tray_count != 0;
    const struct lwir_hotspot *selected = result->tray_pass_selected ? trays : generic;
    result->count = result->tray_pass_selected ? tray_top_count : generic_top_count;
    for (size_t index = 0; index < result->count; ++index) result->candidates[index] = selected[index];
    return true;
}

