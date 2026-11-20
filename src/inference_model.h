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

/** @file inference_model.h
 * @brief Integrity-checked model contracts and independently owned tensor workspaces. */
#ifndef SENSORAIA_INFERENCE_MODEL_H
#define SENSORAIA_INFERENCE_MODEL_H

#include "inference_image.h"
#include "inference_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Parsed descriptor with self-referencing tensor-name pointers; do not copy after parsing. */
struct inference_model {
  char adapter[64], graph_sha256[65], descriptor_sha256[65];
  char labels[4096];
  char input_names[ONNX_TENSOR_LIMIT][ONNX_NAME_CAPACITY];
  char output_names[ONNX_TENSOR_LIMIT][ONNX_NAME_CAPACITY];
  struct onnx_tensor inputs[ONNX_TENSOR_LIMIT], outputs[ONNX_TENSOR_LIMIT];
  size_t input_count, output_count;
  struct inference_image_spec image;
  unsigned audio_rate, audio_samples;
};

/** @brief Independent, noncopyable model owner; mapped tensor buffers and session are reused between jobs. */
struct inference_context {
  struct inference_model model;
  struct onnx_session session;
  struct inference_image_transform transform;
  void *storage; /**< Private mapped arena; ORT's own allocations are additional. */
  size_t storage_bytes; /**< Mapped tensor bytes, not the whole model/process footprint. */
  unsigned timeout_ms; /**< Per-run cooperative budget. */
  uint64_t frame_sequence, sensor_timestamp_ns;
  bool capture_identity; /**< Sequence/time came from this retained capture rather than an offline request. */
  bool private_image; /**< Producer owns exclusive pre-publication mutation rights. */
};

/** @brief Load a bounded descriptor and verify its exact graph digest; hashes are not authentication.
 * @param model Destination contract; discard it on failure.
 * @param descriptor SENSORAIA_MODEL_V1 flat INI path.
 * @param graph Graph whose bytes must match the descriptor hash.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True for a complete descriptor and matching graph identity. */
bool inference_model_read(struct inference_model *model, const char *descriptor,
                          const char *graph, char *error, size_t capacity);
/** @brief Parse a descriptor, not graph bytes; graph integrity remains a separate read-time check.
 * @param model Destination with name storage backing its own tensor views.
 * @param contents Destructively parsed input with at least length+1 writable bytes.
 * @param length Input bytes excluding the writable terminator.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True for supported schema, bounded concrete tensors and complete preprocessing. */
bool inference_model_parse(struct inference_model *model, char *contents, size_t length,
                           char *error, size_t capacity);
/** @brief Hash a stable nonempty regular file using bounded streaming scratch.
 * @param filename Immutable input path during hashing.
 * @param digest Destination for 64 lowercase hex characters and a NUL.
 * @return True for a complete stable read; digest is undefined on failure. */
bool inference_file_sha256(const char *filename, char digest[65]);
/** @brief Resolve one model-owned label without retaining a pointer into a mutable list.
 * @param model Parsed descriptor with comma-separated labels.
 * @param category Zero-based class index.
 * @param label Writable nontruncated class name.
 * @param capacity Available bytes in label.
 * @return True when the requested label exists and fits. */
bool inference_model_label(const struct inference_model *model, unsigned category,
                            char *label, size_t capacity);
/** @brief Verify a bundle, map bounded tensors and bind one warm session.
 * @param context Zero-initialized noncopyable owner.
 * @param settings Resolved ONNX configuration; values are consumed during open.
 * @param adapter Required application ABI, or NULL when the caller validates it separately.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True after the graph can use this context's tensor buffers.
 * @note Mapping tensor storage does not make ORT allocations heap-free or bound whole-board RAM. */
bool inference_context_open(struct inference_context *context,
                             const struct inference_settings *settings, const char *adapter,
                             char *error, size_t capacity);
/** @brief Run the currently prepared tensors using the context's timeout policy.
 * @param context Exclusive open model owner.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True for valid completed output; ignore output on failure. */
bool inference_context_run(struct inference_context *context, char *error, size_t capacity);
/** @brief Prepare a single image input without running the model, so camera leases can be released first.
 * @param context Open owner with one declared image tensor.
 * @param image Borrowed immutable pixels valid for this call only.
 * @param error Writable failure diagnostic.
 * @param capacity Available bytes in error.
 * @return True after filling input storage and its source-coordinate transform. */
bool inference_context_prepare_image(struct inference_context *context,
                                      const struct inference_image *image,
                                      char *error, size_t capacity);
/** @brief Release the session before unmapping its borrowed tensor storage.
 * @param context Exclusive owner; NULL and repeated closes are harmless. */
void inference_context_close(struct inference_context *context);

#ifdef __cplusplus
}
#endif
#endif