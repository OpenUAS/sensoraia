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

/** @file imx500_pipe.h
 * @brief READY/SHOT/OK process ownership without model-analysis semantics.
 * @details Each backend keeps its own state, rather than sharing a detector's
 * process globals. This supervisor qualifies the command channel and saved file;
 * exposure identity and analysis results are checked by their respective consumers. */
#ifndef SENSORAIA_IMX500_PIPE_H
#define SENSORAIA_IMX500_PIPE_H

#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <signal.h>

/** @brief One caller-serialized native worker and its borrowed configuration.
 * @details Initialize with IMX500_PIPE_INITIALIZER before the first lifecycle
 * call. Keep path strings alive until shutdown; no field makes an owned copy.
 * @warning Calls on one instance must be serialized by its backend owner. */
struct imx500_pipe {
  const char *command; /**< Native executable, passed to posix_spawnp without a shell. */
  const char *model; /**< RPK selected by the application, not interpreted here. */
  const char *settings; /**< Borrowed application_settings.ini path; NULL preserves the explicit RPK argument form. */
  bool analyze; /**< Saved-picture requests rather than capture; this worker must never acquire the sensor. */
  bool photo_only; /**< Live camera capture without a model or inference settings. */
  const char *directory; /**< Photo destination resolved by the parent before each request. */
  char prefix; /**< Backend-selected lowercase photo namespace, such as 'a' or 'o'. */
  pid_t pid; /**< Owned child PID; -1 means no child remains to be reaped. */
  int socket; /**< Parent command/status descriptor; -1 means the channel is closed. */
  const volatile sig_atomic_t *running; /**< Optional borrowed USHER cancellation flag, alive until this owner stops. */
};

/** @brief Safe empty lifecycle state: descriptor zero must never mean uninitialized. */
#define IMX500_PIPE_INITIALIZER {.pid = -1, .socket = -1}

/** @brief Replace any owned worker and require its exact READY response.
 * @param[in,out] pipe Initialized state with live command/model/directory strings.
 * @return Zero after readiness, otherwise -1 with newly acquired resources closed.
 * @details The child owns the camera. Readiness includes its model-specific
 * warm-up, not just successful process creation. Existing owned work is stopped
 * before spawning, preventing a second child from competing for the same sensor.
 * @note No analysis sidecar or object/LOOM result is read by this interface. */
int imx500_pipe_start(struct imx500_pipe *pipe);
/** @brief Request one saved photograph and qualify its acknowledgement/file.
 * @param[in,out] pipe Running worker owned by the calling backend.
 * @param[out] filename Private .pending path; consume it only after successful return.
 * @param[in] capacity Writable bytes in @p filename, including its terminator.
 * @param[in] number Nonnegative photo number used in the backend's filename.
 * @return Zero for an exact OK response and a nonempty regular file, otherwise -1.
 * @details Absolute paths avoid dependence on the child's working directory.
 * Newlines are rejected because the protocol is line-framed. A failed exchange
 * closes/reaps the worker rather than pairing a late response with another shot.
 * A later request starts a replacement worker only after the previous owner exits.
 * The caller finishes metadata and calls usher_complete_photo before final-name publication.
 * @warning This does not decode JPEG pixels, prove exposure timing, or establish
 * valid inference. A failed shot is never automatically replayed. */
int imx500_pipe_shoot(struct imx500_pipe *pipe, char *filename, size_t capacity, int number);
/** @brief Close the channel, allow an exit grace period, then kill/reap if needed.
 * @param[in,out] pipe Initialized state; NULL is ignored.
 * @details Closing the command channel lets a healthy worker finish normally.
 * Reaping keeps stale process state from being mistaken for an available camera.
 * Monotonic grace periods bound polling; an unconfirmed exit retains PID ownership
 * and prevents a replacement worker. Scheduling and kernel stalls are not preempted.
 * @note Repeated stops are harmless once the PID and descriptor are cleared. */
void imx500_pipe_stop(struct imx500_pipe *pipe);
/** @brief Submit one saved-picture request to a warm analysis worker without recapturing pixels.
 * @param pipe Running owner started with analyze=true and explicit settings.
 * @param filename Immutable picture path, or a producer-owned private picture for private-image processing.
 * @param number Positive increasing request number within this worker lifetime.
 * @return Zero for an exact successful reply, otherwise minus one after closing/reaping the failed worker.
 * @details A failed exchange is not retried, preventing a late reply from qualifying another picture. */
int imx500_pipe_analyze(struct imx500_pipe *pipe, const char *filename, unsigned number);

#endif