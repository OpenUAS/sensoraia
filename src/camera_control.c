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
 * @file camera_control.c
 * @brief Serialize camera-control payloads and track bounded FC-side tickets.
 * @details The request counter advances only after the caller confirms that the
 * entire UART frame was admitted. This avoids phantom pending controls when a
 * frame is rejected locally and prevents unsafe replay after ambiguous delivery.
 */
#include "camera_control.h"

bool usher_control_put_u32(uint8_t *bytes, size_t size, size_t offset, uint32_t value)
{
  if (!bytes || offset > size || size - offset < 4) return false;
  for (size_t index = 0; index < 4; ++index) bytes[offset + index] = (uint8_t)(value >> (8 * index));
  return true;
}

bool usher_control_get_u32(const uint8_t *bytes, size_t size, size_t offset, uint32_t *value)
{
  if (!bytes || !value || offset > size || size - offset < 4) return false;
  *value = 0;
  for (size_t index = 0; index < 4; ++index) *value |= (uint32_t)bytes[offset + index] << (8 * index);
  return true;
}

bool usher_control_value_valid(uint8_t command, uint8_t value)
{
  return command == CAM_CMD_ZOOM_ABSOLUTE || (command == CAM_CMD_LENS_RETRACT && value == 0)
         || (command == CAM_CMD_KEEP_AWAKE && value <= 1);
}

bool usher_control_encode_request(uint8_t *bytes, size_t size, const struct usher_control_request *request)
{
  if (!bytes || !request || size != USHER_CONTROL_COMMAND_SIZE || !request->camera_mask
      || !request->session || !request->request_id || !request->command) return false;
  bytes[0] = USHER_CONTROL_VERSION;
  bytes[1] = request->camera_mask;
  usher_control_put_u32(bytes, size, 2, request->session);
  usher_control_put_u32(bytes, size, 6, request->request_id);
  bytes[10] = request->command;
  bytes[11] = request->value;
  return true;
}

bool usher_control_decode_request(const uint8_t *bytes, size_t size, struct usher_control_request *request)
{
  if (!bytes || !request || size != USHER_CONTROL_COMMAND_SIZE || bytes[0] != USHER_CONTROL_VERSION
      || !bytes[1] || !bytes[10]) return false;
  *request = (struct usher_control_request){.camera_mask = bytes[1], .command = bytes[10], .value = bytes[11]};
  usher_control_get_u32(bytes, size, 2, &request->session);
  usher_control_get_u32(bytes, size, 6, &request->request_id);
  return request->session && request->request_id;
}

bool usher_control_encode_result(uint8_t *bytes, size_t size, const struct usher_control_result *result)
{
  if (!bytes || !result || size != USHER_CONTROL_RESULT_SIZE || result->camera_id < 1 || result->camera_id > 8
      || !result->request.session || !result->request.request_id || !result->request.command
      || result->status < CAM_CONTROL_APPLIED || result->status > CAM_CONTROL_UNKNOWN
      || result->evidence > CAM_CONTROL_EVIDENCE_PLAYBACK) return false;
  bytes[0] = USHER_CONTROL_VERSION;
  bytes[1] = result->camera_id;
  usher_control_put_u32(bytes, size, 2, result->request.session);
  usher_control_put_u32(bytes, size, 6, result->request.request_id);
  bytes[10] = result->request.command;
  bytes[11] = result->observed;
  bytes[12] = result->status;
  bytes[13] = result->evidence;
  return true;
}

bool usher_control_decode_result(const uint8_t *bytes, size_t size, struct usher_control_result *result)
{
  if (!bytes || !result || size != USHER_CONTROL_RESULT_SIZE || bytes[0] != USHER_CONTROL_VERSION
      || bytes[1] < 1 || bytes[1] > 8 || !bytes[10]
      || bytes[12] < CAM_CONTROL_APPLIED || bytes[12] > CAM_CONTROL_UNKNOWN
      || bytes[13] > CAM_CONTROL_EVIDENCE_PLAYBACK) return false;
  *result = (struct usher_control_result){.camera_id = bytes[1], .status = bytes[12],
    .observed = bytes[11], .evidence = bytes[13], .request = {.command = bytes[10]}};
  usher_control_get_u32(bytes, size, 2, &result->request.session);
  usher_control_get_u32(bytes, size, 6, &result->request.request_id);
  return result->request.session && result->request.request_id;
}

void usher_control_client_reset(struct usher_control_client *client, uint32_t nonce)
{
  client->session = 0;
  client->negotiating_session = 0;
  client->confirmed = false;
  client->negotiating = false;
  client->nonce = nonce ? nonce : 1;
  /* Preserve the request counter but make prior-session results explicitly unknown. */
  for (size_t index = 0; index < 8; ++index) {
    client->capabilities[index] = 0;
    if (client->slots[index].request_id) client->slots[index].status = CAM_CONTROL_UNKNOWN;
  }
}

bool usher_control_client_hello(struct usher_control_client *client, const uint8_t *bytes, size_t size, uint32_t now_us)
{
  uint32_t nonce, session;
  if (!client->negotiating || !bytes || size != USHER_CONTROL_HELLO_SIZE || bytes[0] != USHER_CONTROL_VERSION
      || !usher_control_get_u32(bytes, size, 1, &nonce) || nonce != client->nonce
      || !usher_control_get_u32(bytes, size, 5, &session) || !session) return false;
  if (client->session != session) {
    for (size_t index = 0; index < 8; ++index)
      if (client->slots[index].request_id) client->slots[index].status = CAM_CONTROL_UNKNOWN;
  }
  for (size_t index = 0; index < 8; ++index)
    usher_control_get_u32(bytes, size, 9 + 4 * index, &client->capabilities[index]);
  client->confirmed = client->session == session && client->negotiating_session == session;
  client->session = session;
  client->hello_us = now_us;
  return true;
}

bool usher_control_client_ready(const struct usher_control_client *client, uint32_t now_us)
{
  return client->session && client->confirmed && (uint32_t)(now_us - client->hello_us) < USHER_CONTROL_LEASE_US;
}

void usher_control_client_expire(struct usher_control_client *client, uint32_t now_us)
{
  for (size_t index = 0; index < 8; ++index) {
    struct usher_control_slot *slot = &client->slots[index];
    if (slot->status == CAM_CONTROL_PENDING && (uint32_t)(now_us - slot->sent_us) >= USHER_CONTROL_TIMEOUT_US)
      slot->status = CAM_CONTROL_UNKNOWN;
  }
}

bool usher_control_client_prepare(struct usher_control_client *client, uint8_t mask, uint8_t command,
                                 uint8_t value, uint32_t now_us, struct usher_control_request *request)
{
  usher_control_client_expire(client, now_us);
  if (!request || !mask || !usher_control_value_valid(command, value)
      || !usher_control_client_ready(client, now_us) || client->next_request_id == UINT32_MAX) return false;
  for (size_t index = 0; index < 8; ++index)
    if ((mask & (1U << index)) && client->slots[index].status == CAM_CONTROL_PENDING) return false;
  *request = (struct usher_control_request){.session = client->session, .request_id = client->next_request_id + 1,
    .camera_mask = mask, .command = command, .value = value};
  return true;
}

void usher_control_client_commit(struct usher_control_client *client, const struct usher_control_request *request,
                                 uint32_t now_us)
{
  /* This is called after full-frame admission; prepare alone must not consume an ID. */
  client->next_request_id = request->request_id;
  for (size_t index = 0; index < 8; ++index) {
    if (!(request->camera_mask & (1U << index))) continue;
    client->slots[index] = (struct usher_control_slot){.request_id = request->request_id, .sent_us = now_us,
      .camera_mask = request->camera_mask, .command = request->command, .value = request->value,
      .status = CAM_CONTROL_PENDING};
  }
}

bool usher_control_client_receive(struct usher_control_client *client, const uint8_t *bytes, size_t size,
                                  uint32_t now_us)
{
  struct usher_control_result result;
  usher_control_client_expire(client, now_us);
  if (!usher_control_client_ready(client, now_us) || !usher_control_decode_result(bytes, size, &result)
      || result.request.session != client->session) return false;
  struct usher_control_slot *slot = &client->slots[result.camera_id - 1];
  if (slot->status != CAM_CONTROL_PENDING || slot->request_id != result.request.request_id
      || slot->command != result.request.command) return false;
  if (result.status == CAM_CONTROL_APPLIED
      && ((slot->command == CAM_CMD_LENS_RETRACT && result.evidence != CAM_CONTROL_EVIDENCE_PLAYBACK)
          || (slot->command != CAM_CMD_LENS_RETRACT && result.evidence != CAM_CONTROL_EVIDENCE_VALUE))) return false;
  if (result.status != CAM_CONTROL_APPLIED && result.evidence != CAM_CONTROL_EVIDENCE_NONE) return false;
  if (result.status == CAM_CONTROL_APPLIED
      && ((slot->command == CAM_CMD_LENS_RETRACT && result.observed)
          || (slot->command == CAM_CMD_KEEP_AWAKE && result.observed > 1))) return false;
  slot->status = result.status;
  slot->observed = result.observed;
  slot->evidence = result.evidence;
  return true;
}

uint8_t usher_control_client_camera_result(const struct usher_control_client *client, uint32_t ticket, uint8_t camera_id)
{
  if (!ticket) return CAM_CONTROL_NOT_SENT;
  if (camera_id < 1 || camera_id > 8 || client->slots[camera_id - 1].request_id != ticket) return CAM_CONTROL_UNKNOWN;
  return client->slots[camera_id - 1].status;
}

uint8_t usher_control_client_result(const struct usher_control_client *client, uint32_t ticket)
{
  if (!ticket) return CAM_CONTROL_NOT_SENT;
  uint8_t mask = 0, status = 0;
  bool mixed = false, pending = false;
  for (size_t index = 0; index < 8; ++index)
    if (client->slots[index].request_id == ticket) { mask = client->slots[index].camera_mask; break; }
  if (!mask) return CAM_CONTROL_UNKNOWN;
  /* Unknown evidence dominates; different terminal outcomes remain PARTIAL. */
  for (size_t index = 0; index < 8; ++index) {
    if (!(mask & (1U << index))) continue;
    const uint8_t current = usher_control_client_camera_result(client, ticket, (uint8_t)(index + 1));
    if (current == CAM_CONTROL_UNKNOWN) return current;
    if (current == CAM_CONTROL_PENDING) { pending = true; continue; }
    if (status && status != current) mixed = true;
    status = current;
  }
  return pending ? CAM_CONTROL_PENDING : mixed ? CAM_CONTROL_PARTIAL : status;
}
