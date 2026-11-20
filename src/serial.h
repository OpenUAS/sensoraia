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
 * @file serial.h
 * @brief UART descriptor setup and byte-oriented compatibility helpers.
 * @details New USHER code should use serial_tx.h for output because it preserves
 * frame boundaries under concurrent worker activity. These helpers remain for platform
 * compatibility with the historic transport interface.
 */

#include <stdbool.h>
#include <unistd.h>

extern int fd;

/**
 * @brief Configure the shared non-blocking USHER UART descriptor.
 * @param port_name UART device path.
 * @return 0 on success or a negative setup error.
 * @details The descriptor is set to raw 115200-bit/s 8-bit mode so line discipline cannot
 * translate binary USHER framing bytes. This module owns descriptor setup only; concurrent
 * transmission must use @c serial_tx.h.
 */
int serial_init(const char *port_name);

/**
 * @brief Preserve the legacy "character available" transport API contract.
 * @return Always false because USHER receives through the parser's nonblocking descriptor.
 * @details Retained solely for platform compatibility. New code must not use this predicate to
 * gate UART reads; it would permanently suppress them.
 */
static inline int ttyUSB0ChAvailable(void)
{
  return false;
}

/**
 * @brief Attempt a single-byte write through the legacy UART interface.
 * @param value Byte to send.
 * @details The compatibility API cannot report a short write or a busy
 * descriptor. Use the frame-aware interface in serial_tx.h for USHER output.
 */
static inline void ttyUSB0Transmit(char value)
{
  (void)write(fd, &value, 1);
}

/**
 * @brief Attempt a single-byte read through the legacy UART interface.
 * @return Received byte, or zero when no byte was read.
 * @details The return value cannot distinguish a received NUL from an empty
 * read; binary protocol parsers must check the descriptor read result instead.
 */
static inline char ttyUSB0Getch(void)
{
  char value = 0;
  (void)read(fd, &value, 1);
  return value;
}
