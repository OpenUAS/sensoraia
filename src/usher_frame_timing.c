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

/* Request the POSIX clock declarations before any system header is included. */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

/** @file usher_frame_timing.c
 * @brief C implementation of frame-clock conversion and capture EXIF setup.
 * @details Sampling MONOTONIC on both sides of BOOTTIME bounds offset-read
 * latency. Implausible sensor age, duration or sampling cost leaves the original
 * evidence available but keeps its alignment timestamp explicitly unqualified.
 */
#include "usher_frame_timing.h"

#include "boot_id.h"
#include "sensor_capture.h"

#include <sys/types.h>
#include <time.h>
#include <unistd.h>

enum {
  maximum_exposure_us = 100000,
  maximum_frame_duration_us = 200000,
  maximum_clock_sample_ns = 1000000,
  maximum_sensor_age_ns = 500000000,
  unqualified_time_error_us = 100001
};

/** @brief Convert a validated POSIX clock sample to nanoseconds without overflow.
 * @param[in] value Clock sample returned by clock_gettime().
 * @param[out] result Nanoseconds from that clock's origin.
 * @return True only for normalized, nonnegative values representable in uint64_t.
 * @note A malformed sample must invalidate alignment rather than wrap the age.
 */
static bool timespec_to_nanoseconds(const struct timespec *value, uint64_t *result)
{
  if (!value || !result || value->tv_sec < 0 || value->tv_nsec < 0
      || value->tv_nsec >= 1000000000L) {
    return false;
  }
  const uint64_t seconds = (uint64_t)value->tv_sec;
  const uint64_t nanoseconds = (uint64_t)value->tv_nsec;
  if (seconds > (UINT64_MAX - nanoseconds) / UINT64_C(1000000000)) {
    return false;
  }
  *result = seconds * UINT64_C(1000000000) + nanoseconds;
  return true;
}

struct usher_frame_timing usher_frame_timing_estimate(uint64_t sequence, uint64_t sensor_ns,
                                                       int64_t exposure_us, int64_t duration_us)
{
  struct usher_frame_timing result = {
    .sequence = sequence,
    .sensor_ns = sensor_ns,
    .monotonic_us = 0,
    .time_error_us = unqualified_time_error_us,
    .exposure_us = exposure_us,
    .duration_us = duration_us
  };
  struct timespec mono_before, boot, mono_after;
  uint64_t before_ns, boot_ns, after_ns;

  /* Reject implausible input before any unsigned conversion or clock work. */
  if (!result.sensor_ns || exposure_us < 0 || exposure_us > maximum_exposure_us
      || duration_us <= 0 || duration_us > maximum_frame_duration_us
      || clock_gettime(CLOCK_MONOTONIC, &mono_before) != 0
      || clock_gettime(CLOCK_BOOTTIME, &boot) != 0
      || clock_gettime(CLOCK_MONOTONIC, &mono_after) != 0
      || !timespec_to_nanoseconds(&mono_before, &before_ns)
      || !timespec_to_nanoseconds(&boot, &boot_ns)
      || !timespec_to_nanoseconds(&mono_after, &after_ns)) {
    return result;
  }

  /* Slow or impossible samples cannot safely translate one clock into the other. */
  if (after_ns < before_ns || after_ns - before_ns > maximum_clock_sample_ns
      || boot_ns < result.sensor_ns || boot_ns - result.sensor_ns > maximum_sensor_age_ns
      || before_ns < boot_ns - result.sensor_ns) {
    return result;
  }

  /* Frame age maps the BOOTTIME stamp into MONOTONIC; center exposure/readout. */
  const uint64_t age_ns = boot_ns - result.sensor_ns;
  const uint64_t monotonic_midpoint_ns = before_ns + (after_ns - before_ns) / 2 - age_ns;
  const uint64_t interval_us = (uint64_t)exposure_us + (uint64_t)duration_us;
  const uint64_t midpoint_us = monotonic_midpoint_ns / 1000;
  const uint64_t exposure_offset_us = interval_us / 2;
  if (UINT64_MAX - midpoint_us < exposure_offset_us) {
    return result;
  }
  result.monotonic_us = midpoint_us + exposure_offset_us;
  result.time_error_us = (after_ns - before_ns + 1999) / 2000 + (interval_us + 1) / 2 + 2;
  return result;
}

bool usher_frame_document(const struct usher_frame_timing *timing, unsigned width, unsigned height,
                          struct exif_document *document)
{
  if (!timing || !document) {
    return false;
  }
  struct sensor_capture record = {0};
  usher_boot_id(record.boot_id);
  record.width = width;
  record.height = height;
  record.process_id = (uint32_t)getpid();
  record.sequence = timing->sequence;
  record.sensor_timestamp_ns = timing->sensor_ns;
  record.monotonic_us = timing->monotonic_us;
  record.time_error_us = timing->time_error_us;
  record.exposure_us = timing->exposure_us;
  record.frame_duration_us = timing->duration_us;

  exif_document_init(document);
  unsigned char *data = exif_document_reserve(document, EXIF_CAMERA, SENSOR_CAPTURE_TAG,
                                              EXIF_UNDEFINED, SENSOR_CAPTURE_BYTES);
  return data && sensor_capture_encode(&record, data, SENSOR_CAPTURE_BYTES);
}