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

/** @file loom_onnx_features.c
 * @brief Model-identified feature transport independent of IMX500 tensor metadata.
 * @details Fixed little-endian fields preserve float bits and feature-space identity across PC/ARM64. */
#include "loom_features.h"
#include "../../src/inference_model.h"

#include <math.h>
#include <string.h>

/** @brief Encode an already-bounds-checked field without unaligned integer stores. */
static void write_u32(unsigned char *destination, uint32_t value)
{
  for (unsigned index = 0; index < 4; ++index) destination[index] = (unsigned char)(value >> (8 * index));
}

/** @brief Decode an already-bounds-checked field without host-endian or alignment assumptions. */
static uint32_t read_u32(const unsigned char *source)
{
  return (uint32_t)source[0] | (uint32_t)source[1] << 8 | (uint32_t)source[2] << 16 | (uint32_t)source[3] << 24;
}

size_t loom_onnx_features_pack(unsigned char *record, size_t capacity, const struct inference_model *model,
                               uint64_t sequence, uint64_t sensor_timestamp_ns)
{
  if (!record || !model || !model->output_count || model->output_count > ONNX_TENSOR_LIMIT ||
      strnlen(model->graph_sha256, sizeof(model->graph_sha256)) != 64 ||
      strnlen(model->descriptor_sha256, sizeof(model->descriptor_sha256)) != 64) return 0;
    for (size_t index = 0; index < 64; ++index) {
    const unsigned char graph = (unsigned char)model->graph_sha256[index];
    const unsigned char descriptor = (unsigned char)model->descriptor_sha256[index];
    if (!((graph >= '0' && graph <= '9') || (graph >= 'a' && graph <= 'f')) ||
      !((descriptor >= '0' && descriptor <= '9') || (descriptor >= 'a' && descriptor <= 'f'))) return 0;
    }
  size_t size = 156, total = 0;
  /* Validate every output before touching the destination so a rejected feature set leaves no partial record. */
  for (size_t index = 0; index < model->output_count; ++index) {
    const struct onnx_tensor *tensor = &model->outputs[index];
    size_t bytes = 0;
    if (!tensor->name || !tensor->data || tensor->type != ONNX_FLOAT32 || !tensor->rank ||
        !onnx_tensor_bytes(tensor->type, tensor->dimensions, tensor->rank, &bytes) || bytes != tensor->bytes ||
        bytes / sizeof(float) > 4096 - total) return 0;
    const size_t length = strnlen(tensor->name, ONNX_NAME_CAPACITY);
    if (!length || length == ONNX_NAME_CAPACITY) return 0;
    for (size_t character = 0; character < length; ++character)
      if ((unsigned char)tensor->name[character] < 32 || (unsigned char)tensor->name[character] == 127) return 0;
    for (size_t axis = 0; axis < tensor->rank; ++axis) if (tensor->dimensions[axis] > UINT32_MAX) return 0;
    const float *values = tensor->data;
    for (size_t value = 0; value < bytes / sizeof(float); ++value) if (!isfinite(values[value])) return 0;
    total += bytes / sizeof(float);
    size += 12 + length + tensor->rank * 4 + bytes;
  }
  if (size > capacity || size > LOOM_ONNX_FEATURE_CAPACITY) return 0;
  static const unsigned char signature[] = "LOOMOX01";
  for (size_t index = 0; index < 8; ++index) record[index] = signature[index];
  for (size_t index = 0; index < 64; ++index) {
    record[8 + index] = (unsigned char)model->graph_sha256[index];
    record[72 + index] = (unsigned char)model->descriptor_sha256[index];
  }
  for (unsigned index = 0; index < 8; ++index) {
    record[136 + index] = (unsigned char)(sequence >> (8 * index));
    record[144 + index] = (unsigned char)(sensor_timestamp_ns >> (8 * index));
  }
  write_u32(record + 152, (uint32_t)model->output_count);
  size_t offset = 156;
  for (size_t index = 0; index < model->output_count; ++index) {
    const struct onnx_tensor *tensor = &model->outputs[index];
    const size_t length = strlen(tensor->name);
    write_u32(record + offset, (uint32_t)length); offset += 4;
    for (size_t character = 0; character < length; ++character) record[offset++] = (unsigned char)tensor->name[character];
    write_u32(record + offset, (uint32_t)tensor->rank); offset += 4;
    for (size_t axis = 0; axis < tensor->rank; ++axis) {
      write_u32(record + offset, (uint32_t)tensor->dimensions[axis]); offset += 4;
    }
    const size_t count = tensor->bytes / sizeof(float);
    write_u32(record + offset, (uint32_t)count); offset += 4;
    const float *values = tensor->data;
    for (size_t value = 0; value < count; ++value) {
      union { float value; uint32_t bits; } encoded = {.value = values[value]};
      write_u32(record + offset, encoded.bits); offset += 4;
    }
  }
  return offset;
}

bool loom_onnx_features_valid(const unsigned char *record, size_t size,
                               const char *model_sha256, const char *descriptor_sha256)
{
  if (!record || size < 156 || size > LOOM_ONNX_FEATURE_CAPACITY || memcmp(record, "LOOMOX01", 8)) return false;
  for (size_t index = 8; index < 136; ++index)
    if (!((record[index] >= '0' && record[index] <= '9') || (record[index] >= 'a' && record[index] <= 'f'))) return false;
  /* Matching shapes alone cannot make descriptors from different trained feature spaces comparable. */
  if (model_sha256 && (strnlen(model_sha256, 65) != 64 || memcmp(record + 8, model_sha256, 64))) return false;
  if (descriptor_sha256 && (strnlen(descriptor_sha256, 65) != 64 || memcmp(record + 72, descriptor_sha256, 64))) return false;
  const uint32_t tensors = read_u32(record + 152);
  if (!tensors || tensors > ONNX_TENSOR_LIMIT) return false;
  size_t offset = 156, total = 0;
  for (uint32_t tensor = 0; tensor < tensors; ++tensor) {
    if (size - offset < 4) return false;
    const uint32_t name = read_u32(record + offset); offset += 4;
    if (!name || name >= ONNX_NAME_CAPACITY || name > size - offset) return false;
    for (uint32_t index = 0; index < name; ++index) if (record[offset + index] < 32 || record[offset + index] == 127) return false;
    offset += name;
    if (size - offset < 4) return false;
    const uint32_t rank = read_u32(record + offset); offset += 4;
    if (!rank || rank > ONNX_RANK_LIMIT || (size_t)rank * 4 > size - offset) return false;
    uint32_t product = 1;
    for (uint32_t axis = 0; axis < rank; ++axis) {
      const uint32_t dimension = read_u32(record + offset); offset += 4;
      if (!dimension || dimension > 4096 / product) return false;
      product *= dimension;
    }
    if (size - offset < 4) return false;
    const uint32_t count = read_u32(record + offset); offset += 4;
    if (count != product || count > 4096 - total || (size_t)count * 4 > size - offset) return false;
    total += count;
    for (uint32_t value = 0; value < count; ++value) {
      union { float value; uint32_t bits; } decoded = {.bits = read_u32(record + offset)};
      if (!isfinite(decoded.value)) return false;
      offset += 4;
    }
  }
  return offset == size;
}

bool loom_onnx_return_vector(const unsigned char *record, size_t size,
                              const char *model_sha256, const char *descriptor_sha256, float values[3])
{
  if (!values || !loom_onnx_features_valid(record, size, model_sha256, descriptor_sha256) ||
      read_u32(record + 152) != 1) return false;
  size_t offset = 156;
  const uint32_t name = read_u32(record + offset);
  offset += 4 + name;
  const uint32_t rank = read_u32(record + offset);
  offset += 4 + rank * 4;
  const uint32_t count = read_u32(record + offset);
  offset += 4;
  if (count != 2 && count != 3) return false;
  float decoded[3] = {0, 0, 1};
  for (uint32_t index = 0; index < count; ++index) {
    union { uint32_t bits; float value; } number = {.bits = read_u32(record + offset + index * 4)};
    decoded[index] = number.value;
  }
  for (unsigned index = 0; index < 3; ++index) values[index] = decoded[index];
  return true;
}