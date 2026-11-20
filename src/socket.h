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

#ifndef DIGITAL_CAM_USHER_SOCKET_H
#define DIGITAL_CAM_USHER_SOCKET_H

/**
 * @file socket.h
 * @brief Local non-blocking UDP channel used by USHER's development pipeline.
 * @details This is intentionally loopback-only infrastructure for local SODA and
 * simulator integration, not a network-facing control protocol.
 */

/** @brief Create the loopback UDP endpoint and, for a server, claim USHER ownership.
 * @param is_server Nonzero to bind the fixed local port; zero for an unbound sender.
 * @return 0 on success, or -1 on failure; EALREADY preserves an active endpoint.
 * @details The server bind doubles as the physical-mode single-instance claim.
 * A second USHER must fail rather than share the port because it would also
 * compete for the same UART and camera resources. */
int socket_init(int is_server);
/** @brief Close the owned UDP endpoint; repeated calls are harmless. */
void socket_deinit(void);
/** @brief Return the active UDP descriptor for polling.
 * @return Borrowed descriptor, or a negative value before initialization/after teardown. */
int socket_get_fd(void);
/** @brief Receive one datagram without blocking.
 * @param buffer Destination bytes.
 * @param len Destination capacity.
 * @return recvfrom() result, including -1/EAGAIN when no datagram is ready. */
int socket_recv(char *buffer, int len);
/** @brief Send one loopback datagram without blocking.
 * @param buffer Payload bytes.
 * @param len Payload length. */
void socket_send(char *buffer, int len);

#endif /* DIGITAL_CAM_USHER_SOCKET_H */
