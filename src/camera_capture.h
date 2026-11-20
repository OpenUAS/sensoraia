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

#ifndef USHER_CAMERA_CAPTURE_H
#define USHER_CAMERA_CAPTURE_H

/**
 * @file camera_capture.h
 * @brief Optional split-phase capture contract for USHER image backends.
 * @details Separating acquisition from publication lets a driver return its camera
 * buffers before JPEG encoding or storage finishes. Only advertise this interface
 * when the driver actually owns the selected pixels; a queued shutter request is
 * not a retained image. Synchronous backends need not implement these hooks.
 */

#include <stdbool.h>
#include <stddef.h>
#include <signal.h>

#if __GCC_ATOMIC_INT_LOCK_FREE != 2
#error "Cancellation requires lock-free integer atomic operations"
#endif
#if defined(__cplusplus)
static_assert(sizeof(sig_atomic_t) == sizeof(int) && alignof(sig_atomic_t) >= alignof(int),
              "Cancellation flags must match the lock-free integer representation");
#else
_Static_assert(sizeof(sig_atomic_t) == sizeof(int) && _Alignof(sig_atomic_t) >= _Alignof(int),
               "Cancellation flags must match the lock-free integer representation");
#endif

/** @brief Atomically poll a borrowed cancellation flag; NULL means no cancellation. */
static inline bool capture_running(const volatile sig_atomic_t *running)
{
  return !running || __atomic_load_n(running, __ATOMIC_RELAXED) != 0;
}

/** @brief Update cancellation without locks; this flag does not publish other shared state. */
static inline void capture_set_running(volatile sig_atomic_t *running, bool active)
{
  __atomic_store_n(running, active ? 1 : 0, __ATOMIC_RELAXED);
}

/** @brief Per-image ownership record, kept alive until the matching finish call.
 * @details Paths are bounded bookkeeping, not pixel buffers. The driver owns any
 * image memory and its capacity accounting. Never reuse this record while acquired.
 */
struct camera_pending {
  char source[4096];       /**< Private driver output; retain on publication failure when possible. */
  char destination[4096];  /**< Requested public filename; existing originals must not be replaced. */
  bool acquired;          /**< The driver owns pixels for this capture and owes a finish operation. */
};

/** @brief Driver capabilities whose table must outlive every pending image.
 * @details USHER snapshots the table pointer with the image, not the currently
 * selected camera. This keeps a later backend switch from finishing the wrong job.
 * Acquisition is serialized by the camera owner; finish may overlap later captures.
 * Implementations must synchronize their shared state and bound memory and waits.
 */
struct camera_capture_ops {
  /** @brief Retain one exposure without waiting for its final JPEG publication.
   * @param[out] pending Ownership record filled by the driver.
   * @param image_number Original USHER shot number.
   * @return Zero with acquired set when pixels are owned; positive for synchronous
   * fallback before any exposure; negative for failure without a duplicate retry.
   */
  int (*acquire)(struct camera_pending *pending, int image_number);
  /** @brief Finish the exact acquisition represented by pending.
   * @param[in,out] pending Record to release, including admission accounting, on every exit.
   * @param[out] filename Published filename on success; empty on failure.
   * @param size Filename capacity including its terminating NUL.
   * @param running Optional shutdown flag to check during bounded waits.
   * @return Zero for a verified published file, negative on failure or cancellation.
   * @warning Failure is not permission to discard an existing original or recapture
   * using the old pose. Preserve recoverable private files and report the failure.
   */
  int (*finish)(struct camera_pending *pending, char *filename, size_t size,
                const volatile sig_atomic_t *running);
  /** @brief Nonblocking capacity snapshot; acquisition remains the admission authority.
   * @return True when another capture may be attempted. NULL means no extra readiness gate.
   */
  bool (*has_capacity)(void);
  unsigned max_in_flight; /**< Per-camera limit, currently 1..2; not a whole-MORA memory budget. */
};

#endif