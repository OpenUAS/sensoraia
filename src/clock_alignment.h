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

#ifndef USHER_CLOCK_ALIGNMENT_H
#define USHER_CLOCK_ALIGNMENT_H

/**
 * @file clock_alignment.h
 * @brief Bounded mapping between flight-controller microseconds and USHER monotonic time.
 * @details The protocol exchanges a random token and controller receive/transmit times.
 * The resulting interval is deliberately bounded, short-lived, and token-associated so
 * stale replies or a wrapped 32-bit controller clock cannot geotag audio with false
 * precision.
 */

#include <stddef.h>
#include "usher_protocol.h"

/** Maximum acceptable probe round trip; larger samples are not useful evidence. */
#define CLOCK_ALIGNMENT_MAX_RTT_US UINT64_C(100000)
/** Lifetime of a mapping before drift uncertainty makes it unsuitable for correlation. */
#define CLOCK_ALIGNMENT_MAX_AGE_US UINT64_C(2000000)
/** Conservative oscillator drift allowance used to widen projected intervals. */
#define CLOCK_ALIGNMENT_DRIFT_PPM UINT64_C(1000)

/** @brief Inclusive USHER monotonic-time bounds for a mapped controller timestamp. */
struct clock_interval {
  uint64_t earliest_us;
  uint64_t latest_us;
};

/** @brief Stateful evidence for at most one pending and one accepted clock probe. */
struct clock_alignment {
  union usher_clock_request_union pending_token;
  union usher_clock_request_union mapped_token;
  uint64_t sent_us;
  uint64_t received_us;
  uint64_t mapped_sent_us;
  uint32_t receive_fc_us;
  struct clock_interval anchor;
  uint32_t anchor_fc_us;
  bool pending;
  bool valid;
};

/** @brief Register a newly transmitted clock probe.
 * @param clock Mutable synchronization state.
 * @param token Nonzero random token carried on the wire.
 * @param sent_us USHER monotonic timestamp at transmission.
 * @return True if the probe is accepted as pending.
 * @details Tokens must not repeat the pending or last accepted probe; this prevents a
 * delayed reply from being mistaken for fresh timing evidence. */
bool clock_alignment_request(struct clock_alignment *clock, union usher_clock_request_union token, uint64_t sent_us);
/** @brief Validate a flight-controller clock reply and establish a mapping anchor.
 * @param clock Mutable synchronization state.
 * @param payload Exact reply payload bytes.
 * @param length Payload length.
 * @param received_us USHER monotonic receive timestamp.
 * @return True only for a matching, bounded-latency reply.
 * @details Rather than guessing a one-way delay, this stores an interval spanning the
 * possible controller transmit instant, including bounded service time and clock drift. */
bool clock_alignment_reply(struct clock_alignment *clock, const uint8_t *payload, size_t length, uint64_t received_us);
/** @brief Project a controller timestamp into a conservative USHER time interval.
 * @param clock Accepted alignment state.
 * @param token Token proving the pose belongs to this probe epoch.
 * @param fc_us 32-bit controller microsecond timestamp.
 * @param now_us Current USHER monotonic time, used to reject stale/future projections.
 * @param interval Receives inclusive projected bounds.
 * @return True when @p interval contains a plausible mapping.
 * @warning The result is an interval, not a claimed exact timestamp. */
bool clock_alignment_map(const struct clock_alignment *clock, union usher_clock_request_union token,
                         uint32_t fc_us, uint64_t now_us, struct clock_interval *interval);

#endif