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

/**
 * @file pose_log.h
 * @brief Non-blocking pose-evidence CSV logging API. */
#ifndef USHER_POSE_LOG_H
#define USHER_POSE_LOG_H

#include <stddef.h>
#include <stdint.h>
#include "clock_alignment.h"

/** @brief Optional token-bound evidence mapping a controller pose interval to MORA time. */
struct pose_clock_evidence {
  union usher_clock_request_union token; /**< Request token joining this pose to one clock exchange. */
  struct clock_interval sample_time; /**< Conservative monotonic interval covering the FC sample. */
  uint64_t probe_sent_us; /**< Host monotonic send time for the matching request. */
  uint64_t probe_received_us; /**< Host monotonic receive time for the matching response. */
  uint32_t probe_receive_fc_us; /**< Flight-controller receive timestamp from the exchange. */
  uint32_t probe_transmit_fc_us; /**< Flight-controller transmit timestamp from the exchange. */
  bool mapped; /**< True only when token and interval passed clock-alignment validation. */
};

/** @brief Snapshot counters for accepted, flushed, dropped, and rejected pose records. */
struct pose_log_stats {
  uint64_t accepted; /**< Records admitted to the in-memory queue. */
  uint64_t synced; /**< Records in batches successfully fdatasync'ed. */
  uint64_t dropped; /**< Valid records discarded due to lock/capacity/I/O pressure. */
  uint64_t rejected; /**< Invalid payloads or clock evidence not admitted. */
  int error; /**< First writer/startup errno-style failure, or zero. */
};

/** @brief Start a durable asynchronous CSV session, or disable logging when directory is empty.
 * @param directory Writable destination directory, optionally using `~` shorthand.
 * @return 0 on active/disabled success, otherwise -1. */
int pose_log_start(const char *directory);
/** @brief Validate and enqueue a raw pose payload without clock mapping evidence.
 * @return 1 when queued, 0 when intentionally dropped due to contention/capacity, or -1 when invalid. */
int pose_log_record(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us);
/** @brief Validate and enqueue a pose payload with optional mapped timing provenance.
 * @details Trylock protects capture latency: logging is best-effort evidence, never a reason
 * to block the real-time UART path. */
int pose_log_record_clocked(const uint8_t *payload, size_t length, uint64_t receive_monotonic_us,
                            const struct pose_clock_evidence *evidence);
/** @brief Return a synchronized snapshot of logging health and loss counters. */
struct pose_log_stats pose_log_status(void);
/** @brief Flush queued records, join the writer, and close the CSV file.
 * @return 0 after durable orderly shutdown, or -1 if a pending I/O error exists. */
int pose_log_stop(void);

#endif