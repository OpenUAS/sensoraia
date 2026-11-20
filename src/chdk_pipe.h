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
 * @file chdk_pipe.h
 * @brief Bounded CHDKbridge subprocess lifecycle for USHER's visible-camera backend.
 * @details Each invocation owns one PTP session. No host Lua or shell is used.
 */

#include <signal.h>
#include <stdint.h>

#define CHDK_CONTROL_REPLY_SIZE 4U
#define CHDK_CONTROL_REPLY_TAG 0xc1U

/** @brief Prepare bounded controller ownership without accessing USB.
 * @return 0 when the executable is accessible and shutdown is not requested.
 * @details Settings and camera readiness are checked in the capture invocation. */
int chdk_pipe_init(void);

void chdk_pipe_set_running(const volatile sig_atomic_t *running);

/** @brief Capture directly to a new JPEG destination without overwriting a file.
 * @param filename Caller-owned destination path.
 * @return 0 for a checked saved JPEG, otherwise -1; capture is never replayed. */
int chdk_pipe_shoot(const char *filename);
uint8_t chdk_pipe_control(uint8_t command, uint8_t value, uint8_t *observed, uint8_t *evidence);

/** @brief Terminate and reap the owned process group using bounded grace periods. */
void chdk_pipe_deinit(void);

