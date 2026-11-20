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

/** @file onnx_session.h
 * @brief Bounded tensor bindings and cooperative execution policy, independent of CAM semantics. */
#ifndef SENSORAIA_ONNX_SESSION_H
#define SENSORAIA_ONNX_SESSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ONNX_TENSOR_LIMIT 16U
#define ONNX_RANK_LIMIT 8U
#define ONNX_NAME_CAPACITY 128U
#define ONNX_BUFFER_LIMIT (64U * 1024U * 1024U)

/** @brief Supported numeric types; values match the ONNX Runtime C ABI. */
enum onnx_element_type {
  ONNX_FLOAT32 = 1, ONNX_UINT8 = 2, ONNX_INT8 = 3,
  ONNX_INT32 = 6, ONNX_INT64 = 7, ONNX_FLOAT64 = 11
};

/** @brief Named concrete tensor view; its aligned backing storage must outlive the session. */
struct onnx_tensor {
  const char *name; /**< Graph name, copied by session open rather than matched by declaration order. */
  enum onnx_element_type type; /**< Exact numeric storage representation. */
  size_t rank; /**< Concrete rank, including zero for a scalar. */
  int64_t dimensions[ONNX_RANK_LIMIT]; /**< Positive concrete extents; unused axes are ignored. */
  void *data; /**< Aligned caller storage, never freed by the session. */
  size_t bytes; /**< Exact byte count implied by type and concrete dimensions. */
};

/** @brief Explicit execution limits so each model cannot create an unbounded CPU pool. */
struct onnx_session_options {
  unsigned threads; /**< Intra-op thread count in 1..32; inter-op execution remains sequential. */
  const char *library_directory; /**< Optional runtime search directory, consumed during open. */
};

#define ONNX_SESSION_OPTIONS_DEFAULT {1U, NULL}

/** @brief Noncopyable caller-owned session; zero-initialize and serialize open/run/close ownership.
 * @details Fields are implementation state, not configuration knobs. Caller tensors must
 * remain alive until close has completed; a failed run does not validate their output contents. */
struct onnx_session {
  void *handle, *memory, *run_options;
  void *inputs[ONNX_TENSOR_LIMIT], *outputs[ONNX_TENSOR_LIMIT];
  char input_names[ONNX_TENSOR_LIMIT][ONNX_NAME_CAPACITY];
  char output_names[ONNX_TENSOR_LIMIT][ONNX_NAME_CAPACITY];
  struct onnx_tensor output_specs[ONNX_TENSOR_LIMIT];
  size_t input_count, output_count;
  pthread_mutex_t mutex;
  pthread_cond_t condition;
  pthread_t watchdog;
  bool synchronized, watched, stopping, running, timed_out, cancelled;
  uint64_t deadline_us;
};

/** @brief Compute bounded concrete storage, rejecting unsupported types, dynamic extents and overflow.
 * @param type Supported numeric element type.
 * @param dimensions Concrete extents; NULL is allowed only for rank zero.
 * @param rank Number of extents, at most ONNX_RANK_LIMIT.
 * @param bytes Output byte count, meaningful only on success.
 * @return True for a representable tensor within ONNX_BUFFER_LIMIT. */
bool onnx_tensor_bytes(enum onnx_element_type type, const int64_t *dimensions,
                       size_t rank, size_t *bytes);
/** @brief Validate graph contracts and bind reusable caller buffers; failure releases acquired resources.
 * @param session Zero-initialized owner with no open session; do not copy it after opening.
 * @param model Readable ONNX graph path.
 * @param inputs Named input views whose backing buffers outlive the session.
 * @param input_count Number of input views, in 1..ONNX_TENSOR_LIMIT.
 * @param outputs Named writable output views with the same lifetime rule.
 * @param output_count Number of output views, in 1..ONNX_TENSOR_LIMIT.
 * @param options Optional execution/search policy; NULL uses one CPU thread.
 * @param error Writable failure diagnostic, cleared on success.
 * @param capacity Available bytes in error.
 * @return True only after every name, type, shape and backing byte count is accepted.
 * @note Dynamic graph axes bind to the declared concrete shape; outputs are checked again after each run. */
bool onnx_session_open(struct onnx_session *session, const char *model,
                       const struct onnx_tensor *inputs, size_t input_count,
                       const struct onnx_tensor *outputs, size_t output_count,
                       const struct onnx_session_options *options,
                       char *error, size_t capacity);
/** @brief Run once; deadline is absolute CLOCK_MONOTONIC microseconds, or zero for none.
 * @param session Open owner; one run at a time is admitted.
 * @param deadline_us Absolute completion budget in the host monotonic clock domain.
 * @param error Writable failure diagnostic, cleared on success.
 * @param capacity Available bytes in error.
 * @return True for complete execution with valid output shapes and an unexpired budget.
 * @warning Ignore output buffers on false. Cancellation is cooperative, not a hard deadline. */
bool onnx_session_run(struct onnx_session *session, uint64_t deadline_us,
                      char *error, size_t capacity);
/** @brief Request cooperative cancellation from another thread; join the owner before destroying storage.
 * @param session Live owner; NULL/unopened owners are ignored. */
void onnx_session_cancel(struct onnx_session *session);
/** @brief Cancel/wait for an active run and release resources; repeated closes are harmless.
 * @param session Exclusive lifecycle owner; NULL is ignored.
 * @pre No new calls may begin while close destroys synchronization objects. */
void onnx_session_close(struct onnx_session *session);
/** @brief Read the clock used by inference deadlines.
 * @return CLOCK_MONOTONIC microseconds, or zero when the clock cannot be read. */
uint64_t onnx_monotonic_us(void);

#ifdef __cplusplus
}
#endif
#endif