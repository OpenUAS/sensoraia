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
 * @file camera_control.h
 * @brief Fixed-size camera-control wire codec and FC-side ticket state.
 * @details Control tickets are distinct from photo numbers and shot requests.
 * Eight per-camera slots keep the flight-controller footprint bounded; a ticket
 * is consumed only after its complete UART frame is admitted. Commands are not
 * replayed after timeout because the camera may already have acted.
 */
#ifndef USHER_CAMERA_CONTROL_H
#define USHER_CAMERA_CONTROL_H

#include <stddef.h>
#include "usher_protocol.h"

#define USHER_CONTROL_TIMEOUT_US UINT32_C(70000000)
#define USHER_CONTROL_LEASE_US UINT32_C(6000000)

/** @brief One addressed command request shared with the companion protocol. */
struct usher_control_request {
  /** @brief Nonzero negotiated companion session and request ticket. */
  uint32_t session, request_id;
  /** @brief Camera selection mask, command identifier, and command value. */
  uint8_t camera_mask, command, value;
};

/** @brief Result evidence for one camera selected by a request mask. */
struct usher_control_result {
  /** @brief Correlated request identity; never substitute a shot number here. */
  struct usher_control_request request;
  /** @brief One-based camera ID, terminal status, observed value, and evidence. */
  uint8_t camera_id, status, observed, evidence;
};

/** @brief Fixed pending/result slot retained for one addressable camera. */
struct usher_control_slot {
  /** @brief Request ticket and monotonic send timestamp in microseconds. */
  uint32_t request_id, sent_us;
  /** @brief Original command plus returned observation/status/evidence. */
  uint8_t camera_mask, command, value, observed, status, evidence;
};

/**
 * @brief Handshake, capability cache, and eight-camera ticket state.
 * @details Contains no heap-owned data or background jobs. Keep the compile-time
 * size limit: this object resides in flight-controller memory.
 */
struct usher_control_client {
  struct usher_control_slot slots[8];
  uint32_t capabilities[8];
  uint32_t session, nonce, next_request_id, hello_us;
  uint32_t negotiating_session;
  bool confirmed;
  bool negotiating;
};

#ifdef __cplusplus
static_assert(sizeof(struct usher_control_client) <= 256, "FC control state exceeds 256 bytes");
extern "C" {
#else
_Static_assert(sizeof(struct usher_control_client) <= 256, "FC control state exceeds 256 bytes");
#endif

/** @brief Encode a 32-bit value as four little-endian protocol bytes.
 * @param[out] bytes Destination payload buffer.
 * @param[in] size Total buffer size.
 * @param[in] offset Byte offset for the field.
 * @param[in] value Value to encode.
 * @return True if the field fits and the buffer is non-null.
 */
bool usher_control_put_u32(uint8_t *bytes, size_t size, size_t offset, uint32_t value);
/** @brief Decode a bounded four-byte little-endian protocol value.
 * @param[in] bytes Source payload buffer.
 * @param[in] size Total buffer size.
 * @param[in] offset Byte offset for the field.
 * @param[out] value Receives the decoded value.
 * @return True if the field fits and both pointers are valid.
 */
bool usher_control_get_u32(const uint8_t *bytes, size_t size, size_t offset, uint32_t *value);
/** @brief Encode one command payload with the fixed wire size and field order.
 * @param[out] bytes Destination payload buffer.
 * @param[in] size Payload size; must equal USHER_CONTROL_COMMAND_SIZE.
 * @param[in] request Session, ticket, camera mask, command, and value.
 * @return True only for valid required fields and exact payload size.
 */
bool usher_control_encode_request(uint8_t *bytes, size_t size, const struct usher_control_request *request);
/** @brief Validate and decode one fixed-size command payload.
 * @param[in] bytes Received payload.
 * @param[in] size Payload size; must equal USHER_CONTROL_COMMAND_SIZE.
 * @param[out] request Receives decoded fields on success.
 * @return True only when version, mask, command, session, and ticket are valid.
 */
bool usher_control_decode_request(const uint8_t *bytes, size_t size, struct usher_control_request *request);
/** @brief Encode one camera's correlated control result.
 * @param[out] bytes Destination payload buffer.
 * @param[in] size Payload size; must equal USHER_CONTROL_RESULT_SIZE.
 * @param[in] result Result status and evidence for one camera.
 * @return True only when camera, request, status, and evidence fields are valid.
 */
bool usher_control_encode_result(uint8_t *bytes, size_t size, const struct usher_control_result *result);
/** @brief Validate and decode one camera result payload.
 * @param[in] bytes Received payload.
 * @param[in] size Payload size; must equal USHER_CONTROL_RESULT_SIZE.
 * @param[out] result Receives decoded fields on success.
 * @return True only when version, identity, status, and evidence are valid.
 */
bool usher_control_decode_result(const uint8_t *bytes, size_t size, struct usher_control_result *result);
/** @brief Validate the command-specific legal value range.
 * @param[in] command Control command ID.
 * @param[in] value Command value.
 * @return True for a supported value, false before any ticket is reserved otherwise.
 */
bool usher_control_value_valid(uint8_t command, uint8_t value);
/** @brief Reset session state and invalidate outstanding tickets without replay.
 * @param[in,out] client Client state to reset.
 * @param[in] nonce New handshake nonce; zero is normalized to one.
 */
void usher_control_client_reset(struct usher_control_client *client, uint32_t nonce);
/** @brief Accept only a HELLO matching the current nonce-bound negotiation.
 * @param[in,out] client Handshake and capability state.
 * @param[in] bytes HELLO payload.
 * @param[in] size Payload size; must equal USHER_CONTROL_HELLO_SIZE.
 * @param[in] now_us Current monotonic time in microseconds.
 * @return True if the reply is structurally valid and belongs to this negotiation.
 */
bool usher_control_client_hello(struct usher_control_client *client, const uint8_t *bytes, size_t size, uint32_t now_us);
/** @brief Check confirmed-session readiness and the bounded HELLO lease.
 * @param[in] client Client state.
 * @param[in] now_us Current monotonic time in microseconds.
 * @return True only while the negotiated session is fresh and confirmed.
 */
bool usher_control_client_ready(const struct usher_control_client *client, uint32_t now_us);
/** @brief Mark expired pending commands UNKNOWN; never resubmit them.
 * @param[in,out] client Client state to update.
 * @param[in] now_us Current monotonic time in microseconds.
 */
void usher_control_client_expire(struct usher_control_client *client, uint32_t now_us);
/** @brief Prepare a request without consuming its ticket before UART admission.
 * @param[in,out] client Session state and pending slots.
 * @param[in] mask Eight-bit camera selection mask.
 * @param[in] command Command identifier.
 * @param[in] value Command-specific value.
 * @param[in] now_us Current monotonic time in microseconds.
 * @param[out] request Receives the tentative request on success.
 * @return True only if the session is ready, values are legal, and selected slots are free.
 * @note Call @ref usher_control_client_commit only after the complete frame is admitted.
 */
bool usher_control_client_prepare(struct usher_control_client *client, uint8_t mask, uint8_t command,
                                 uint8_t value, uint32_t now_us, struct usher_control_request *request);
/** @brief Commit a prepared request after full-frame UART admission.
 * @param[in,out] client Client state whose ticket and selected slots are updated.
 * @param[in] request Request previously returned by @ref usher_control_client_prepare.
 * @param[in] now_us Monotonic send time in microseconds.
 */
void usher_control_client_commit(struct usher_control_client *client, const struct usher_control_request *request,
                                 uint32_t now_us);
/** @brief Accept a result only when session, ticket, camera, and evidence correlate.
 * @param[in,out] client Client state containing pending requests.
 * @param[in] bytes Received result payload.
 * @param[in] size Payload length.
 * @param[in] now_us Current monotonic time in microseconds.
 * @return True when the result belongs to a pending request and its evidence is valid.
 */
bool usher_control_client_receive(struct usher_control_client *client, const uint8_t *bytes, size_t size,
                                  uint32_t now_us);
/** @brief Aggregate per-camera results for one group ticket.
 * @param[in] client Client result slots.
 * @param[in] ticket Request ID; zero means NOT_SENT.
 * @return UNKNOWN if evidence is missing, PENDING if any selected camera is pending,
 * PARTIAL for mixed terminal outcomes, or the common terminal status.
 */
uint8_t usher_control_client_result(const struct usher_control_client *client, uint32_t ticket);
/** @brief Read one camera's status for a group request.
 * @param[in] client Client result slots.
 * @param[in] ticket Request ID; zero means NOT_SENT.
 * @param[in] camera_id One-based camera slot from 1 through 8.
 * @return Matching result status, UNKNOWN for absent/replaced tickets, or NOT_SENT.
 */
uint8_t usher_control_client_camera_result(const struct usher_control_client *client, uint32_t ticket, uint8_t camera_id);

#ifdef __cplusplus
}
#endif

#endif
