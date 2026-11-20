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

#ifndef SENSORAIA_LWIR_CONTAINER_H
#define SENSORAIA_LWIR_CONTAINER_H

/** @file
 * @brief Exact thermal evidence in JPEG containers, independent of analysis.
 *
 * @details Read side of the format written by lwir_image.c. A saved LWIR JPEG
 * keeps the native temperature plane, little-endian Kelvin*64 per sample, in
 * APP15 chunks signed "LWIRRAW1" (live camera) or "LWIRSIM1" (synthetic
 * replay). Because those bytes bypass the lossy DCT path, temperatures read
 * back here are bit-identical to the captured plane at any JPEG quality.
 *
 * Used after capture by analysis, geolocation and validation tools that need
 * temperatures from a stored photo. Only marker segments up to the first
 * scan are parsed; compressed pixels are never decoded.
 *
 * Invariants enforced by the reader:
 * - Chunks share one signature kind and one width/height, and their offsets
 *   are contiguous from zero; the layer must be complete.
 * - Native dimensions are independent of display orientation.
 * - The file must be a regular file that does not change while being read.
 * - Image-only JPEGs (TDK layout) have no layer: the layer/stream readers
 *   return 0 for them.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Existing APP15 wire layout, shared by the reader and image writer.
 * @details The eight-byte signature is followed by little-endian uint32 width,
 * height and byte offset into the native temperature plane. Samples start at
 * byte 20. Chunk capacity is the writer's policy, not a reader acceptance limit.
 */
enum lwir_thermal_layout {
    LWIR_THERMAL_APP_MARKER = 0xef,         /**< APP15 marker code. */
    LWIR_THERMAL_SIGNATURE_BYTES = 8,       /**< Signature length, no terminator. */
    LWIR_THERMAL_WIDTH_OFFSET = 8,          /**< uint32 LE native width. */
    LWIR_THERMAL_HEIGHT_OFFSET = 12,        /**< uint32 LE native height. */
    LWIR_THERMAL_DATA_OFFSET = 16,          /**< uint32 LE byte offset of this chunk within the plane. */
    LWIR_THERMAL_HEADER_BYTES = 20,         /**< Header size; samples follow immediately. */
    LWIR_THERMAL_CHUNK_CAPACITY = 65000     /**< Writer's sample bytes per chunk; header + capacity stays within one APPn segment. */
};

/** @brief Signature of a temperature plane read from the live camera. */
#define LWIR_THERMAL_SENSOR_SIGNATURE "LWIRRAW1"
/** @brief Signature of a temperature plane produced by synthetic replay. */
#define LWIR_THERMAL_SYNTHETIC_SIGNATURE "LWIRSIM1"

/** @brief Native geometry and identity recovered from the APP14 "LWIREV02" text.
 * @details Fields: @c width, @c height native thermal size; @c rotation
 * clockwise display rotation (0/90/180/270); @c mirror, @c flip display
 * transforms; @c legend whether a legend strip was drawn; @c camera true for
 * source=camera, false for replay; @c sequence frame sequence; @c arrival_us
 * host arrival time; @c boot_id boot identity text; @c frame_sha256 64
 * lowercase hex digits plus NUL. All are validated against the APP15 layer
 * and the JPEG dimensions before being returned.
 */
struct lwir_saved_geometry {
    unsigned width, height, rotation;
    bool mirror, flip, legend, camera;
    uint64_t sequence, arrival_us;
    char boot_id[64];
    char frame_sha256[65];
};

/** @brief Extract the temperature plane plus its recorded provenance geometry.
 * @param filename Regular JPEG path; symlinks are refused (O_NOFOLLOW).
 * @param destination Caller-owned sample storage; ignore on failure.
 * @param capacity Writable bytes in destination.
 * @param geometry Written only on a complete, consistent result.
 * @return 1 for a complete layer with matching APP14 evidence, otherwise -1
 * (invalid input, missing or duplicate APP14 evidence, any inconsistency, or
 * I/O failure).
 * @details Stricter than lwir_read_temperature_layer(): exactly one LWIREV02
 * record with all eleven geometry keys must be present; its native size must
 * equal the layer, its source must match the signature kind, and the JPEG
 * dimensions must equal the native size after the recorded rotation.
 * @note Never returns 0: without a layer the geometry cross-checks cannot pass,
 * so image-only or layer-less JPEGs report -1 here.
 */
int lwir_read_temperature_evidence(const char *filename, uint8_t *destination, size_t capacity,
                                    struct lwir_saved_geometry *geometry);

/** @brief Extract native little-endian Kelvin*64 samples without decoding pixels.
 * @param filename Regular JPEG containing LWIRRAW1 or LWIRSIM1 APP15 chunks.
 * @param destination Caller-owned temperature storage; no intermediate plane is kept.
 * @param capacity Writable bytes in destination.
 * @param width Native thermal width, written only when a complete layer is found.
 * @param height Native thermal height, independent of JPEG display orientation.
 * @param synthetic True for simulated evidence, written only on success.
 * @return 1 for a complete layer, 0 when absent, -1 for invalid input or I/O failure.
 * @warning Ignore destination on failure; a partial layer may have been written.
 * @details Chunk order, dimensions, provenance and file stability are checked.
 * This validates the container header, not the compressed display's pixel content.
 */
int lwir_read_temperature_layer(const char *filename, uint8_t *destination, size_t capacity,
                                unsigned *width, unsigned *height, bool *synthetic);
/** @brief Extract thermal evidence from an already-open regular JPEG stream.
 * @param input Borrowed seekable stream; rewound to SOI, never closed here.
 * @param destination Caller-owned sample storage; ignore on failure.
 * @param capacity Available writable bytes.
 * @param width Native width, written on success only.
 * @param height Native height, written on success only.
 * @param synthetic Native versus simulated evidence, written on success only.
 * @return Same 1/0/-1 contract as lwir_read_temperature_layer.
 * @details Reusing an owned stream avoids reopening a path after identity checks.
 */
int lwir_read_temperature_stream(FILE *input, uint8_t *destination, size_t capacity,
                                 unsigned *width, unsigned *height, bool *synthetic);

#ifdef __cplusplus
}
#endif
#endif