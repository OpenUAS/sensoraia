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

/** @file loom_features.c
 * @brief Bounded LOOM feature validation and exact binary EXIF transport.
 * @details The capture owner supplies immutable same-exposure buffers; this
 * core neither acquires a camera nor decides whether a navigation match exists. */
#include "loom_features.h"
#include "imx500_tensor.h"

#include <float.h>
#include <math.h>

/** @brief Read four checked little-endian bytes without an unaligned cast.
 * @param data At least four bytes inside validated tensor metadata.
 * @return Unsigned component value in host representation. */
static uint32_t feature_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | (uint32_t)data[1] << 8
         | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

/** @brief Read a bounded shape extent from one driver tensor record.
 * @param record Full accessible tensor-info record.
 * @param dimension Validated zero-based dimension below the rank limit.
 * @return The little-endian unsigned 16-bit extent, widened for checked arithmetic. */
static uint32_t feature_extent(const uint8_t *record, uint32_t dimension)
{
  const size_t offset = IMX500_TENSOR_DIMENSIONS_OFFSET + dimension * IMX500_TENSOR_DIMENSION_BYTES;
  return (uint32_t)record[offset] | (uint32_t)record[offset + 1] << 8;
}

bool loom_features_valid(const float *values, size_t count,
                         const uint8_t *information, size_t information_size)
{
  if (!values || !information || information_size < IMX500_TENSOR_INFO_BYTES
      || count == 0 || count > IMX500_TENSOR_ELEMENT_LIMIT) {
    return false;
  }
  const uint32_t tensors = feature_u32(information + IMX500_TENSOR_COUNT_OFFSET);
  if (tensors == 0 || tensors > IMX500_TENSOR_MAX_COUNT) {
    return false;
  }

  size_t total = 0;
  for (uint32_t index = 0; index < tensors; ++index) {
    const uint8_t *record = information + IMX500_TENSOR_FIRST_RECORD_OFFSET
                           + index * IMX500_TENSOR_RECORD_BYTES;
    const uint32_t length = feature_u32(record + IMX500_TENSOR_LENGTH_OFFSET);
    const uint32_t rank = feature_u32(record + IMX500_TENSOR_RANK_OFFSET);
    if (length == 0 || length > IMX500_TENSOR_ELEMENT_LIMIT
        || rank == 0 || rank > IMX500_TENSOR_MAX_RANK) {
      return false;
    }
    uint32_t product = 1;
    /* Shape metadata is untrusted. Check with division before multiplication,
     * and check the cumulative budget before adding another tensor's elements. */
    for (uint32_t dimension = 0; dimension < rank; ++dimension) {
      const uint32_t extent = feature_extent(record, dimension);
      if (extent == 0 || product > IMX500_TENSOR_ELEMENT_LIMIT / extent) {
        return false;
      }
      product *= extent;
    }
    if (product != length || total > IMX500_TENSOR_ELEMENT_LIMIT - length) {
      return false;
    }
    total += length;
  }
  if (total != count) {
    return false;
  }
  for (size_t index = 0; index < count; ++index) {
    /* NaN/infinity cannot be valid feature evidence or a useful input
     * to future similarity/geometric checks. Transport validity is still not a match. */
    if (!isfinite(values[index])) {
      return false;
    }
  }
  return true;
}

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "LOOM evidence requires IEEE binary32");

size_t loom_features_size(size_t count)
{
  return count && count <= IMX500_TENSOR_ELEMENT_LIMIT ? 32 + IMX500_TENSOR_INFO_BYTES + count * 4 : 0;
}

/** @brief Encode one little-endian field in a LOOM evidence record.
 * @param[out] data Storage validated by the caller for @p bytes octets.
 * @param value Unsigned bits to write.
 * @param bytes Field width, at most eight.
 * @details Pack explicit bytes so persisted data does not depend on host
 * byte order or alignment. */
static void feature_put(unsigned char *data, uint64_t value, size_t bytes)
{
  for (size_t index = 0; index < bytes; ++index) data[index] = (unsigned char)(value >> (8 * index));
}

bool loom_features_pack(unsigned char *record, size_t capacity, const float *values, size_t count,
                        const uint8_t *information, size_t information_size,
                        uint64_t sequence, uint64_t sensor_timestamp_ns)
{
  const size_t size = loom_features_size(count);
  if (!record || !size || capacity < size || !loom_features_valid(values, count, information, information_size)) return false;
  const char magic[] = "LOOMNN02";
  for (size_t index = 0; index < 8; ++index) record[index] = (unsigned char)magic[index];
  feature_put(record + 8, sequence, 8);
  feature_put(record + 16, sensor_timestamp_ns, 8);
  feature_put(record + 24, count, 4);
  feature_put(record + 28, IMX500_TENSOR_INFO_BYTES, 4);
  for (size_t index = 0; index < IMX500_TENSOR_INFO_BYTES; ++index) record[32 + index] = information[index];
  for (size_t index = 0; index < count; ++index) {
    union { float floating; uint32_t bits; } value = {.floating = values[index]};
    feature_put(record + 32 + IMX500_TENSOR_INFO_BYTES + index * 4, value.bits, 4);
  }
  return true;
}

bool loom_features_unpack(const unsigned char *record, size_t size, float *values, size_t capacity,
                          uint8_t *information, size_t information_size, size_t *count,
                          uint64_t *sequence, uint64_t *sensor_timestamp_ns)
{
  if (count) *count = 0;
  if (!record || size < 32 || !values || !information || !count || !sequence || !sensor_timestamp_ns
      || information_size < IMX500_TENSOR_INFO_BYTES) return false;
  const char magic[] = "LOOMNN02";
  for (size_t index = 0; index < 8; ++index) if (record[index] != (unsigned char)magic[index]) return false;
  const size_t total = feature_u32(record + 24);
  if (!loom_features_size(total) || size != loom_features_size(total) || total > capacity
      || feature_u32(record + 28) != IMX500_TENSOR_INFO_BYTES) return false;
  for (size_t index = 0; index < IMX500_TENSOR_INFO_BYTES; ++index) information[index] = record[32 + index];
  for (size_t index = 0; index < total; ++index) {
    union { uint32_t bits; float floating; } value = {.bits = feature_u32(record + 32 + IMX500_TENSOR_INFO_BYTES + index * 4)};
    values[index] = value.floating;
  }
  if (!loom_features_valid(values, total, information, information_size)) return false;
  *count = total;
  *sequence = (uint64_t)feature_u32(record + 8) | (uint64_t)feature_u32(record + 12) << 32;
  *sensor_timestamp_ns = (uint64_t)feature_u32(record + 16) | (uint64_t)feature_u32(record + 20) << 32;
  return true;
}