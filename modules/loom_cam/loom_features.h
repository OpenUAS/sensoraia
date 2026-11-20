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

/** @file loom_features.h
 * @brief LOOM-owned bounded feature validation and evidence serialization.
 * @details Raw immutable buffers keep this core independent of libcamera and
 * USHER process ownership. Valid feature transport is never navigation evidence. */
#ifndef LOOM_FEATURES_H
#define LOOM_FEATURES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief LOOM-owned camera-directory tag; its value is a LOOMNN02 opaque record. */
#define LOOM_FEATURES_TAG 0xc7a2U
#define LOOM_ONNX_FEATURES_TAG 0xc7a3U
#define LOOM_ONNX_FEATURE_CAPACITY 32768U

struct inference_model;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Validate finite features against the shared IMX500 shape metadata.
 * @param values Borrowed immutable float array for one exposure.
 * @param count Accessible float count.
 * @param information Borrowed tensor-info bytes from that same exposure.
 * @param information_size Accessible metadata bytes.
 * @return True for bounded, consistent shapes and finite values; false otherwise.
 * @details Check shape products and cumulative length before reading values;
 * rank/count limits bound both work and metadata access. No buffer is retained.
 * @note This does not validate model semantics, feature similarity or a route. */
bool loom_features_valid(const float *values, size_t count,
                         const uint8_t *information, size_t information_size);

/** @brief Required bytes for 1..4096 floats, or zero for unsupported counts. */
size_t loom_features_size(size_t count);
/** @brief Pack validated same-exposure shapes, exact uint64 identities and float bits.
 * @pre Inputs remain immutable and do not overlap the destination.
 * @details Writes the LOOMNN02 format described in docs/exif.md. Invalid inputs
 * leave output unchanged. The caller reserves this value in the JPEG's EXIF
 * document; there is no filesystem access or feature-sized allocation here. */
bool loom_features_pack(unsigned char *record, size_t capacity, const float *values, size_t count,
                        const uint8_t *information, size_t information_size,
                        uint64_t sequence, uint64_t sensor_timestamp_ns);
/** @brief Validate and decode an exact-sized binary record into bounded output.
 * @pre Inputs and writable outputs do not overlap; capacity is a float count.
 * @details On failure count is zero; other outputs must be discarded. Structural
 * validity is not a model match. Check identity against the JPEG capture record
 * before consuming features. No buffer is retained or allocated. */
bool loom_features_unpack(const unsigned char *record, size_t size, float *values, size_t capacity,
                          uint8_t *information, size_t information_size, size_t *count,
                          uint64_t *sequence, uint64_t *sensor_timestamp_ns);
/** @brief Encode a complete LOOMOX01 record without fabricating an IMX500 metadata header.
 * @param record Writable destination, unchanged when validation rejects the input.
 * @param capacity Available bytes; records cannot exceed LOOM_ONNX_FEATURE_CAPACITY.
 * @param model Completed immutable float outputs and their exact graph/descriptor identities.
 * @param sequence Source correlation, retained separately from any flight-controller shot number.
 * @param sensor_timestamp_ns Physical sensor time when available; zero explicitly means unavailable.
 * @return Encoded byte count, or zero for invalid/nonfinite/oversized features.
 * @pre Input tensors and destination do not overlap. This transport does not establish a visual match. */
size_t loom_onnx_features_pack(unsigned char *record, size_t capacity, const struct inference_model *model,
                               uint64_t sequence, uint64_t sensor_timestamp_ns);
/** @brief Validate the exact binary layout and optionally require the same trained feature space.
 * @param record Immutable LOOMOX01 bytes.
 * @param size Exact record length; truncation and trailing bytes are rejected.
 * @param model_sha256 Required graph digest, or NULL for structural validation only.
 * @param descriptor_sha256 Required preprocessing digest, or NULL for structural validation only.
 * @return True for bounded finite features and matching supplied identities, not navigation validity. */
bool loom_onnx_features_valid(const unsigned char *record, size_t size,
                               const char *model_sha256, const char *descriptor_sha256);
/** @brief Decode an explicitly declared two/three-value return-vector record.
 * @param record Retained LOOMOX01 bytes from this JPEG.
 * @param size Exact record length.
 * @param model_sha256 Required graph identity, or NULL.
 * @param descriptor_sha256 Required descriptor identity, or NULL.
 * @param values Three floats: first component, second component, confidence.
 * @return False unless exactly one finite two/three-value tensor is present.
 * @details A two-value model has no confidence output; 1 then means valid
 * computation, not a measured probability. Frame semantics remain caller-owned. */
bool loom_onnx_return_vector(const unsigned char *record, size_t size,
                              const char *model_sha256, const char *descriptor_sha256, float values[3]);

#ifdef __cplusplus
}
#endif
#endif