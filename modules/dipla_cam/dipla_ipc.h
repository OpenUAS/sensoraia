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

/** @file dipla_ipc.h
 * @brief Private local worker messages; not a flight-controller wire protocol.
 * @details Unix SOCK_SEQPACKET preserves one fixed-size message per request/reply.
 * Both endpoints are native Sensoraia processes from the same build.
 * @note Integer widths and layout assertions define this local ABI. It is not
 * a replacement for USHER UART framing or a cross-version network service. */
#ifndef SENSORAIA_DIPLA_IPC_H
#define SENSORAIA_DIPLA_IPC_H
#include "dipla_contract.h"
#define DIPLA_IPC_VERSION 1U /**< Reject incompatible packet layouts before interpreting payload. */
#define DIPLA_IPC_START 1U /**< Request a chunk; retain an active belief, reset after STOP. No reply packet. */
#define DIPLA_IPC_STOP 2U /**< Invalidate the matching session; no reply packet. */
#define DIPLA_IPC_SHOT 3U /**< Submit a pose and retrieve one pinned point using dipla_reply. */
#define DIPLA_IPC_PLAN 4U /**< Submit a pose and retrieve one immutable dipla_plan_reply. */
#define DIPLA_IPC_CREDIT_SHOT 5U /**< SHOT with a separate assimilation acknowledgement before the next new observation. */
#define DIPLA_IPC_POINT 6U /**< Retrieve a retained point without admitting another observation. */
#define DIPLA_IPC_ASSIMILATED_METRIC (-1) /**< Invalid local completion credit after assimilation or rejection, never a camera point. */
#define DIPLA_CAMERA_ID 5U /**< Non-image sensor identity, distinct from waypoint indices. */
#define DIPLA_CAMERA_MASK 0x10U /**< Existing eight-slot camera selection bit for camera five. */
#define DIPLA_POINT_MAX_AGE_MS 5000U /**< Assimilated observation lease, not retained-route creation age. */
#define DIPLA_CYCLE_TIMEOUT_MS 1500U /**< Broken one-point cycles restart rather than mixing snapshots. */
/** @brief Pose/control request; session and observation number have separate lifetimes.
 * @details The first accepted pose after START from stopped anchors local
 * (50,50). Active START preserves that survey frame; STOP ends it. */
struct dipla_request {
  /** @var dipla_request::version
   * @brief Layout version; reject before interpreting the remaining packet. */
  /** @var dipla_request::operation
   * @brief START, STOP, SHOT or complete PLAN operation. */
  /** @var dipla_request::session
  * @brief Nonzero chunk token; STOP, not active START, ends survey belief. */
  uint32_t version, operation, session, number; /**< Positive observation identifier for SHOT/PLAN; unused for control packets. */
  /** @var dipla_request::lat
   * @brief Reported latitude in degrees times 1e7. */
  /** @var dipla_request::lon
   * @brief Reported longitude in degrees times 1e7. */
  /** @var dipla_request::alt_mm
   * @brief Owner-supplied FC altitude in millimetres; echoed by the legacy point reply. */
  int32_t lat, lon, alt_mm, agl_q8; /**< Actual reported AGL in metres times 256, not an MSL ground elevation. */
};
/** @brief Legacy one-point retrieval from a generation pinned across eight slots. */
struct dipla_reply {
  /** @var dipla_reply::version
   * @brief Reply layout version, checked before accepting any result. */
  /** @var dipla_reply::session
   * @brief Exact session echo; prevents accepting a previous sortie's completion. */
  /** @var dipla_reply::number
   * @brief Exact observation echo; a retry must not consume another slot. */
  uint32_t version, session, number, valid; /**< Computational validity, never sensor confidence or field accuracy. */
  /** @var dipla_reply::lat
   * @brief Proposed geographic latitude in degrees times 1e7. */
  /** @var dipla_reply::lon
   * @brief Proposed geographic longitude in degrees times 1e7. */
  /** @var dipla_reply::alt_mm
   * @brief Unchanged owner-supplied FC altitude in millimetres, not predicted model Z. */
  /** @var dipla_reply::agl_mm
    * @brief Configured physical flight height in millimetres; default 2000. */
  int32_t lat, lon, alt_mm, agl_mm, metric; /**< Packed (generation<<3)|slot, with internal slot 0..7 mapping to P1..P8. */
};
/** @brief Complete immutable route reply on the local worker socket, not the FC bus.
 * @details All points belong to one generation. Count eight and valid one are
 * required together; incomplete/unready replies cannot authorize publication.
 * Fixed height prevents model-predicted Z from becoming an altitude command. */
struct dipla_plan_reply {
  /** @var dipla_plan_reply::version
   * @brief Complete-plan layout version. */
  /** @var dipla_plan_reply::session
   * @brief Exact session echo, independent of route generation. */
  /** @var dipla_plan_reply::number
   * @brief Exact request echo; route data may originate from an earlier admitted pose. */
  /** @var dipla_plan_reply::valid
   * @brief One only for an entire fresh, bounded route; zero authorizes no points. */
  /** @var dipla_plan_reply::generation
   * @brief Monotonically increasing completed-route identity within this session. */
  /** @var dipla_plan_reply::age_ms
  * @brief Milliseconds since the latest assimilated active-chunk pose was received. */
  uint32_t version, session, number, valid, generation, age_ms, count; /**< Eight only when the complete route is valid. */
  /** @brief Canonical model-field point; (50000,50000) is the captured anchor.
   * @details XY retains the 100-unit domain multiplied by 1000, independently
   * of physical field size. Geographic point replies apply scale and rotation;
   * complete-plan consumers must apply that same configured transform. */
  struct dipla_plan_point {
    /** @var dipla_plan_reply::dipla_plan_point::east_mm
    * @brief Flight-padded model X times 1000; default bounds 4000..96000. */
    /** @var dipla_plan_reply::dipla_plan_point::north_mm
    * @brief Flight-padded model Y times 1000; default bounds 4000..96000. */
    int32_t east_mm, north_mm, height_mm; /**< Configured physical height in millimetres; predicted Z is never forwarded. */
  } points[DIPLA_WAYPOINT_COUNT]; /**< Slot zero maps to P1; a copied bank is never changed mid-reply. */
};
#ifdef __cplusplus
static_assert(sizeof(dipla_request) == 32 && sizeof(dipla_reply) == 36 && sizeof(dipla_plan_reply) == 124,
              "Diplacam local IPC layout changed");
#else
_Static_assert(sizeof(struct dipla_request) == 32 && sizeof(struct dipla_reply) == 36 &&
               sizeof(struct dipla_plan_reply) == 124, "Diplacam local IPC layout changed");
#endif
#endif