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

/** @file sensor_capture.h
 * @brief Exact bounded SENSOR02 capture evidence, independent of TIFF byte order.
 * @details See docs/exif.md for byte offsets. Structural validity does not qualify
 * exposure timing; the arbiter checks current boot, JPEG dimensions and freshness. */
#ifndef USHER_SENSOR_CAPTURE_H
#define USHER_SENSOR_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SENSOR_CAPTURE_BYTES 104U
#define SENSOR_CAPTURE_TAG 0xc7a1U

struct sensor_capture {
  char boot_id[37];
  uint32_t width, height, process_id;
  uint64_t sequence, sensor_timestamp_ns, monotonic_us, time_error_us;
  int64_t exposure_us, frame_duration_us;
};

/**
 * @brief Write a fixed-width little-endian integer.
 * @param[out] data Writable storage for @p bytes octets.
 * @param value Integer whose least significant bytes are written.
 * @param bytes Field width from 1 to 8, validated by the caller.
 * @details Explicit byte offsets keep the record independent of alignment and
 * host byte order.
 */
static inline void sensor_capture_put(unsigned char *data, uint64_t value, size_t bytes)
{
  for (size_t index = 0; index < bytes; ++index) data[index] = (unsigned char)(value >> (index * 8));
}

/**
 * @brief Read a fixed-width little-endian integer.
 * @param data Readable storage of at least @p bytes octets.
 * @param bytes Field width from 1 to 8, validated by the caller.
 * @return The value zero-extended to 64 bits.
 * @details Assemble bytes rather than loading a native integer, since fields
 * can start at unaligned offsets.
 */
static inline uint64_t sensor_capture_get(const unsigned char *data, size_t bytes)
{
  uint64_t value = 0;
  for (size_t index = 0; index < bytes; ++index) value |= (uint64_t)data[index] << (index * 8);
  return value;
}

/**
 * @brief Interpret a 64-bit wire field as a signed two's-complement value.
 * @param value Decoded unsigned bit pattern.
 * @return Signed value represented by the same bits.
 * @details For negative values, convert the complement first so the cast never
 * receives a value outside the signed range.
 */
static inline int64_t sensor_capture_signed(uint64_t value)
{
  return value <= INT64_MAX ? (int64_t)value : -1 - (int64_t)(UINT64_MAX - value);
}

/**
 * @brief Check the structural constraints shared by the wire encoder and decoder.
 * @param record Nullable capture record.
 * @return True for JPEG-sized dimensions and a terminated lowercase UUID-shaped boot ID.
 * @details This checks syntax and dimensions, not whether the boot ID and timing
 * actually belong to the current capture.
 */
static inline bool sensor_capture_valid(const struct sensor_capture *record)
{
  if (!record || record->boot_id[36] || record->width < 2 || record->width > 65535
      || record->height < 2 || record->height > 65535) return false;
  for (size_t index = 0; index < 36; ++index) {
    const char character = record->boot_id[index];
    if (index == 8 || index == 13 || index == 18 || index == 23) {
      if (character != '-') return false;
    } else if (!((character >= '0' && character <= '9') || (character >= 'a' && character <= 'f'))) return false;
  }
  return true;
}

/**
 * @brief Encode a SENSOR02 record in the fixed on-disk layout.
 * @param record Validated source record; must not overlap @p data.
 * @param[out] data Destination byte buffer.
 * @param capacity Available bytes in @p data.
 * @return True on success; false leaves the destination unchanged.
 * @details Validate before the first write, then emit the magic and fields at
 * fixed offsets without copying struct padding.
 */
static inline bool sensor_capture_encode(const struct sensor_capture *record, unsigned char *data, size_t capacity)
{
  if (!data || capacity < SENSOR_CAPTURE_BYTES || !sensor_capture_valid(record)) return false;
  const unsigned char magic[8] = {'S','E','N','S','O','R','0','2'};
  for (size_t index = 0; index < 8; ++index) data[index] = magic[index];
  for (size_t index = 0; index < 36; ++index) data[8 + index] = (unsigned char)record->boot_id[index];
  sensor_capture_put(data + 44, record->width, 4);
  sensor_capture_put(data + 48, record->height, 4);
  sensor_capture_put(data + 52, record->sequence, 8);
  sensor_capture_put(data + 60, record->sensor_timestamp_ns, 8);
  sensor_capture_put(data + 68, record->monotonic_us, 8);
  sensor_capture_put(data + 76, record->time_error_us, 8);
  sensor_capture_put(data + 84, (uint64_t)record->exposure_us, 8);
  sensor_capture_put(data + 92, (uint64_t)record->frame_duration_us, 8);
  sensor_capture_put(data + 100, record->process_id, 4);
  return true;
}

/**
 * @brief Decode a complete SENSOR02 record.
 * @param[out] record Destination for the decoded fields.
 * @param data Serialized source record.
 * @param size Exact source length in bytes.
 * @return True on success; false leaves the destination unchanged.
 * @details Check version and length before indexing the data, then validate a
 * temporary record before assigning it to the caller's destination.
 */
static inline bool sensor_capture_decode(struct sensor_capture *record, const unsigned char *data, size_t size)
{
  if (!record || !data || size != SENSOR_CAPTURE_BYTES || memcmp(data, "SENSOR02", 8)) return false;
  struct sensor_capture decoded;
  for (size_t index = 0; index < 36; ++index) decoded.boot_id[index] = (char)data[8 + index];
  decoded.boot_id[36] = '\0';
  decoded.width = (uint32_t)sensor_capture_get(data + 44, 4);
  decoded.height = (uint32_t)sensor_capture_get(data + 48, 4);
  decoded.sequence = sensor_capture_get(data + 52, 8);
  decoded.sensor_timestamp_ns = sensor_capture_get(data + 60, 8);
  decoded.monotonic_us = sensor_capture_get(data + 68, 8);
  decoded.time_error_us = sensor_capture_get(data + 76, 8);
  decoded.exposure_us = sensor_capture_signed(sensor_capture_get(data + 84, 8));
  decoded.frame_duration_us = sensor_capture_signed(sensor_capture_get(data + 92, 8));
  decoded.process_id = (uint32_t)sensor_capture_get(data + 100, 4);
  if (!sensor_capture_valid(&decoded)) return false;
  *record = decoded;
  return true;
}

#endif