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

/** @file inference_model.c
 * @brief Verified model descriptors and bounded per-context tensor storage.
 * @details Descriptors bind preprocessing to graph bytes; application adapters still qualify semantic outputs. */
#include "inference_model.h"
#include "sha256fst.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>

/** @brief Independent duplicate masks for descriptor-wide and indexed tensor fields. */
struct model_parser {
  struct inference_model *model;
  uint32_t seen;
  unsigned char inputs[ONNX_TENSOR_LIMIT], outputs[ONNX_TENSOR_LIMIT];
};

bool inference_file_sha256(const char *filename, char digest[65])
{
  return sha256_file(filename, digest);
}

/** @brief Store an owned bounded name so parser-buffer destruction cannot invalidate tensor views. */
static bool copy_name(char *destination, size_t capacity, const char *value)
{
  const size_t length = strlen(value);
  if (!length || length >= capacity) return false;
  for (size_t index = 0; index <= length; ++index) {
    if ((unsigned char)value[index] < 32 && value[index]) return false;
    destination[index] = value[index];
  }
  return true;
}

/** @brief Accept concrete positive extents only; export-time dynamic axes need an explicit execution shape. */
static bool parse_dimensions(struct onnx_tensor *tensor, const char *text)
{
  tensor->rank = 0;
  while (*text) {
    if (tensor->rank == ONNX_RANK_LIMIT || *text < '1' || *text > '9') return false;
    errno = 0;
    char *end = NULL;
    const unsigned long long number = strtoull(text, &end, 10);
    if (errno || end == text || number > INT64_MAX || (*end && *end != ',')) return false;
    tensor->dimensions[tensor->rank++] = (int64_t)number;
    text = *end ? end + 1 : end;
    if (*end && !*text) return false;
  }
  return tensor->rank != 0;
}

/** @brief Accept one shared channel value or exactly three, without silently broadcasting an incomplete pair. */
static bool parse_channels(float values[3], const char *text)
{
  for (unsigned channel = 0; channel < 3; ++channel) {
    char token[64];
    const char *end = strchr(text, ',');
    const size_t length = end ? (size_t)(end - text) : strlen(text);
    if (!length || length >= sizeof(token)) return false;
    for (size_t index = 0; index < length; ++index) token[index] = text[index];
    token[length] = '\0';
    double number = 0;
    if (!ini_real(token, &number) || !isfinite((float)number)) return false;
    values[channel] = (float)number;
    if (!end) {
      if (channel) return channel == 2;
      values[1] = values[2] = values[0];
      return true;
    }
    text = end + 1;
  }
  return false;
}

/** @brief Parse one indexed tensor while keeping duplicate detection separate for each tensor and direction. */
static bool tensor_field(struct model_parser *parser, const char *key, const char *value, bool input)
{
  const char *number = key + (input ? 6 : 7);
  const char *separator = strchr(number, '.');
  char index_text[21];
  if (!separator || separator == number || (size_t)(separator - number) >= sizeof(index_text)) return false;
  const size_t length = (size_t)(separator - number);
  for (size_t index = 0; index < length; ++index) index_text[index] = number[index];
  index_text[length] = '\0';
  uint64_t slot = 0;
  if (!ini_uint(index_text, &slot) || slot >= ONNX_TENSOR_LIMIT) return false;
  const char *field = separator + 1;
  struct onnx_tensor *tensor = input ? &parser->model->inputs[slot] : &parser->model->outputs[slot];
  unsigned char *seen = input ? &parser->inputs[slot] : &parser->outputs[slot];
  unsigned char bit;
  if (!strcmp(field, "name")) bit = 1;
  else if (!strcmp(field, "type")) bit = 2;
  else if (!strcmp(field, "shape")) bit = 4;
  else return false;
  if (*seen & bit) return false;
  *seen |= bit;
  if (bit == 1) {
    char *name = input ? parser->model->input_names[slot] : parser->model->output_names[slot];
    if (!copy_name(name, ONNX_NAME_CAPACITY, value)) return false;
    tensor->name = name;
    return true;
  }
  if (bit == 4) return parse_dimensions(tensor, value);
  static const char *const names[] = {"float32", "uint8", "int8", "int32", "int64", "float64"};
  static const enum onnx_element_type types[] = {ONNX_FLOAT32, ONNX_UINT8, ONNX_INT8, ONNX_INT32, ONNX_INT64, ONNX_FLOAT64};
  for (size_t index = 0; index < sizeof(types) / sizeof(types[0]); ++index) {
    if (!strcmp(value, names[index])) { tensor->type = types[index]; return true; }
  }
  return false;
}

/** @brief Recognize descriptor policy, not arbitrary network-specific configuration or custom operators. */
static bool model_field(void *context, const char *key, const char *value)
{
  struct model_parser *parser = context;
  struct inference_model *model = parser->model;
  if (!strncmp(key, "input.", 6)) return tensor_field(parser, key, value, true);
  if (!strncmp(key, "output.", 7)) return tensor_field(parser, key, value, false);
  static const char *const names[] = {"schema", "adapter", "model_sha256", "input_count", "output_count",
    "image.layout", "image.color", "image.resize", "image.scale", "image.mean", "image.padding",
    "image.units", "audio.sample_rate", "audio.samples", "image.interpolation", "labels"};
  size_t field = 0;
  while (field < sizeof(names) / sizeof(names[0]) && strcmp(key, names[field])) ++field;
  if (field == sizeof(names) / sizeof(names[0]) || (parser->seen & (UINT32_C(1) << field))) return false;
  parser->seen |= UINT32_C(1) << field;
  if (!field) return !strcmp(value, "SENSORAIA_MODEL_V1");
  if (field == 1) return copy_name(model->adapter, sizeof(model->adapter), value);
  if (field == 2) return strlen(value) == 64 && strspn(value, "0123456789abcdef") == 64 &&
                          copy_name(model->graph_sha256, sizeof(model->graph_sha256), value);
  if (field == 15) return !strchr(value, ';') && copy_name(model->labels, sizeof(model->labels), value);
  if (field == 3 || field == 4 || field == 12 || field == 13) {
    uint64_t number;
    if (!ini_uint(value, &number) || !number || number > (field < 5 ? ONNX_TENSOR_LIMIT : 192000U)) return false;
    if (field == 3) model->input_count = (size_t)number;
    else if (field == 4) model->output_count = (size_t)number;
    else if (field == 12) model->audio_rate = (unsigned)number;
    else model->audio_samples = (unsigned)number;
    return true;
  }
  if (field == 5) {
    if (!strcmp(value, "nchw")) model->image.layout = INFERENCE_NCHW;
    else if (!strcmp(value, "nhwc")) model->image.layout = INFERENCE_NHWC;
    else return false;
  } else if (field == 6) {
    model->image.bgr = !strcmp(value, "bgr");
    model->image.channels = !strcmp(value, "gray") ? 1 : 3;
    if (strcmp(value, "rgb") && strcmp(value, "bgr") && strcmp(value, "gray")) return false;
  } else if (field == 7) {
    model->image.letterbox = !strcmp(value, "letterbox");
    if (strcmp(value, "letterbox") && strcmp(value, "stretch")) return false;
  } else if (field == 8 || field == 9) return parse_channels(field == 8 ? model->image.scale : model->image.mean, value);
  else if (field == 10) {
    double number;
    if (!ini_real(value, &number) || !isfinite((float)number)) return false;
    model->image.padding = (float)number;
  } else if (field == 11) {
    if (!strcmp(value, "pixel")) model->image.units = INFERENCE_PIXEL;
    else if (!strcmp(value, "kelvin64")) model->image.units = INFERENCE_RAW_KELVIN64;
    else if (!strcmp(value, "kelvin")) model->image.units = INFERENCE_KELVIN;
    else if (!strcmp(value, "celsius")) model->image.units = INFERENCE_CELSIUS;
    else return false;
  } else if (field == 14) return !strcmp(value, "nearest");
  return true;
}

bool inference_model_parse(struct inference_model *model, char *contents, size_t length,
                           char *error, size_t capacity)
{
  if (!model || !contents || !error || !capacity) return false;
  *model = (struct inference_model){0};
  /* Hash the original bytes before destructive parsing so evidence identifies the exact preprocessing contract. */
  sha256_hex(contents, length, model->descriptor_sha256);
  struct model_parser parser = {.model = model};
  if (!ini_parse(contents, length, model_field, &parser) || (parser.seen & 31U) != 31U) goto invalid;
  size_t total = 0;
  for (size_t slot = 0; slot < ONNX_TENSOR_LIMIT; ++slot) {
    const bool input = slot < model->input_count, output = slot < model->output_count;
    if (parser.inputs[slot] != (input ? 7 : 0) || parser.outputs[slot] != (output ? 7 : 0)) goto invalid;
    struct onnx_tensor *tensors[] = {&model->inputs[slot], &model->outputs[slot]};
    for (size_t side = 0; side < 2; ++side) {
      if ((side == 0 && !input) || (side == 1 && !output)) continue;
      struct onnx_tensor *tensor = tensors[side];
      if (!onnx_tensor_bytes(tensor->type, tensor->dimensions, tensor->rank, &tensor->bytes) ||
          tensor->bytes > ONNX_BUFFER_LIMIT - total) goto invalid;
      total += tensor->bytes;
      for (size_t previous = 0; previous < slot; ++previous)
        if (!strcmp(tensor->name, side ? model->outputs[previous].name : model->inputs[previous].name)) goto invalid;
    }
  }
  if (model->image.channels) {
    const uint32_t required = (UINT32_C(1) << 5) | (UINT32_C(1) << 6) | (UINT32_C(1) << 7) |
      (UINT32_C(1) << 8) | (UINT32_C(1) << 9) | (UINT32_C(1) << 11) | (UINT32_C(1) << 14);
    const struct onnx_tensor *input = &model->inputs[0];
    if ((parser.seen & required) != required || input->rank != 4 || input->dimensions[0] != 1) goto invalid;
    const unsigned channel_axis = model->image.layout == INFERENCE_NCHW ? 1 : 3;
    const unsigned height_axis = model->image.layout == INFERENCE_NCHW ? 2 : 1;
    const unsigned width_axis = model->image.layout == INFERENCE_NCHW ? 3 : 2;
    if (input->dimensions[channel_axis] != model->image.channels || input->dimensions[width_axis] > 2048 ||
        input->dimensions[height_axis] > 2048) goto invalid;
    model->image.width = (unsigned)input->dimensions[width_axis];
    model->image.height = (unsigned)input->dimensions[height_axis];
    model->image.type = input->type;
    if (input->type != ONNX_FLOAT32 && input->type != ONNX_UINT8 && input->type != ONNX_INT8) goto invalid;
    for (unsigned channel = 0; channel < model->image.channels; ++channel)
      if (!model->image.scale[channel]) goto invalid;
  }
  error[0] = '\0';
  return true;
invalid:
  snprintf(error, capacity, "Invalid model descriptor: schema, duplicate, tensor bounds or preprocessing contract");
  return false;
}

bool inference_model_label(const struct inference_model *model, unsigned category,
                            char *label, size_t capacity)
{
  if (!model || !label || !capacity) return false;
  const char *cursor = model->labels;
  for (unsigned index = 0; index < category; ++index) {
    cursor = strchr(cursor, ',');
    if (!cursor) return false;
    ++cursor;
  }
  const char *end = strchr(cursor, ',');
  const size_t length = end ? (size_t)(end - cursor) : strlen(cursor);
  if (!length || length >= capacity) return false;
  for (size_t index = 0; index < length; ++index) {
    if ((unsigned char)cursor[index] < 32 || cursor[index] == 127) return false;
    label[index] = cursor[index];
  }
  label[length] = '\0';
  return true;
}

bool inference_model_read(struct inference_model *model, const char *descriptor,
                          const char *graph, char *error, size_t capacity)
{
  if (!model || !descriptor || !graph || !error || !capacity) return false;
  char contents[16385], digest[65];
  size_t length = 0;
  if (!ini_read_file(descriptor, contents, sizeof(contents), &length)) {
    snprintf(error, capacity, "Cannot read model descriptor: %s", descriptor);
    return false;
  }
  if (!inference_model_parse(model, contents, length, error, capacity)) return false;
  /* A well-formed descriptor for a different graph is still an incompatible assignment bundle. */
  if (!inference_file_sha256(graph, digest) || strcmp(digest, model->graph_sha256)) {
    snprintf(error, capacity, "Model graph checksum mismatch or unreadable graph: %s", graph);
    return false;
  }
  return true;
}

bool inference_context_open(struct inference_context *context,
                             const struct inference_settings *settings, const char *adapter,
                             char *error, size_t capacity)
{
  if (!context || !settings || !error || !capacity) return false;
  error[0] = '\0';
  if (context->storage || context->session.handle || settings->backend != INFERENCE_ONNX ||
      !settings->model[0] || !settings->descriptor[0]) {
    snprintf(error, capacity, "ONNX requires a model and model_descriptor in settings");
    return false;
  }
  if (!inference_model_read(&context->model, settings->descriptor, settings->model, error, capacity)) return false;
  if (adapter && strcmp(context->model.adapter, adapter)) {
    snprintf(error, capacity, "Unsupported model adapter: %s (expected %s)", context->model.adapter, adapter);
    return false;
  }
  size_t total = 0;
  /* Align every tensor independently; mixed numeric types may otherwise receive misaligned storage. */
  for (size_t side = 0; side < 2; ++side) {
    const size_t count = side ? context->model.output_count : context->model.input_count;
    const struct onnx_tensor *tensors = side ? context->model.outputs : context->model.inputs;
    for (size_t index = 0; index < count; ++index) {
      const size_t aligned = (tensors[index].bytes + 63U) & ~(size_t)63U;
      if (aligned > ONNX_BUFFER_LIMIT - total) {
        snprintf(error, capacity, "Model tensor workspace exceeds bounded storage");
        return false;
      }
      total += aligned;
    }
  }
  if (!total) {
    snprintf(error, capacity, "Model tensor workspace is empty");
    return false;
  }
  /* POSIX-backed private mapping avoids anonymous-mapping extensions and gives tensor storage a clear lifetime. */
  const int descriptor = open("/dev/zero", O_RDWR | O_CLOEXEC);
  if (descriptor < 0) {
    snprintf(error, capacity, "Cannot open model workspace backing: %s", strerror(errno));
    return false;
  }
  void *storage = mmap(NULL, total, PROT_READ | PROT_WRITE, MAP_PRIVATE, descriptor, 0);
  const int closed = close(descriptor);
  if (storage == MAP_FAILED || closed) {
    if (storage != MAP_FAILED) munmap(storage, total);
    snprintf(error, capacity, "Cannot map bounded model tensor workspace");
    return false;
  }
  context->storage = storage;
  context->storage_bytes = total;
  context->timeout_ms = settings->timeout_ms;
  size_t offset = 0;
  for (size_t side = 0; side < 2; ++side) {
    const size_t count = side ? context->model.output_count : context->model.input_count;
    struct onnx_tensor *tensors = side ? context->model.outputs : context->model.inputs;
    for (size_t index = 0; index < count; ++index) {
      tensors[index].data = (unsigned char *)storage + offset;
      offset += (tensors[index].bytes + 63U) & ~(size_t)63U;
    }
  }
  const struct onnx_session_options options = {.threads = settings->threads,
                                                .library_directory = settings->library_directory};
  if (!onnx_session_open(&context->session, settings->model, context->model.inputs, context->model.input_count,
                         context->model.outputs, context->model.output_count, &options, error, capacity)) {
    inference_context_close(context);
    return false;
  }
  return true;
}

bool inference_context_run(struct inference_context *context, char *error, size_t capacity)
{
  if (!context || !context->storage) {
    if (error && capacity) snprintf(error, capacity, "Inference context is not open");
    return false;
  }
  const uint64_t now = onnx_monotonic_us();
  if (!now) {
    if (error && capacity) snprintf(error, capacity, "Monotonic inference clock unavailable");
    return false;
  }
  return onnx_session_run(&context->session, context->timeout_ms ? now + (uint64_t)context->timeout_ms * 1000 : 0,
                           error, capacity);
}

bool inference_context_prepare_image(struct inference_context *context,
                                      const struct inference_image *image,
                                      char *error, size_t capacity)
{
  if (!context || !context->storage || context->model.input_count != 1 || !context->model.image.channels) {
    if (error && capacity) snprintf(error, capacity, "Model does not declare a single image input");
    return false;
  }
  return inference_image_prepare(image, &context->model.image, context->model.inputs[0].data,
                                  context->model.inputs[0].bytes, &context->transform, error, capacity);
}

void inference_context_close(struct inference_context *context)
{
  if (!context) return;
  /* Session values borrow the mapping, so their destruction must precede munmap. */
  onnx_session_close(&context->session);
  if (context->session.handle) return;
  if (context->storage) munmap(context->storage, context->storage_bytes);
  *context = (struct inference_context){0};
}