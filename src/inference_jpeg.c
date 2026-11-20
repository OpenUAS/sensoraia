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
 * @file inference_jpeg.c
 * @brief Streaming JPEG input for the same preprocessing used by live frames.
 * @details Application scratch is two rows, not a decoded full-resolution RGB image;
 * libjpeg's internal allocations remain separate from this bounded workspace.
 */
#include "inference_jpeg.h"
#include "jpeg_marker_guard.h"

#include <stdio.h>
#include <jpeglib.h>
#include <math.h>
#include <setjmp.h>
#include <string.h>
#include <sys/stat.h>

/** @brief Caller-owned codec/error state that remains valid across decoder longjmp exits. */
struct jpeg_reader {
  struct jpeg_decompress_struct codec;
  struct jpeg_error_mgr failure;
  void (*emit)(j_common_ptr, int);
  jmp_buf jump;
  FILE *input;
  bool created;
  unsigned char row[8192 * 3];
  union { float alignment; unsigned char bytes[2048 * 3 * sizeof(float)]; } prepared;
};

/** @brief Convert libjpeg's fatal callback into a failed input instead of terminating the CAM worker. */
static void jpeg_failure(j_common_ptr codec)
{
  struct jpeg_reader *reader = codec->client_data;
  longjmp(reader->jump, 1);
}

/** @brief Check JPEG marker bounds ahead of normal codec diagnostics.
 * @param codec Decoder carrying the borrowed jpeg_reader in client_data.
 * @param level libjpeg diagnostic level.
 * @details Preserve the original handler for accepted messages so the
 * inference wrapper can still observe codec warnings. */
static void jpeg_message(j_common_ptr codec, int level)
{
  struct jpeg_reader *reader = codec->client_data;
  usher_jpeg_validate_marker(codec);
  reader->emit(codec, level);
}

/** @brief Fill a bound input tensor; ignore partial tensor/transform contents after any failure. */
static bool decode_jpeg(struct jpeg_reader *reader, struct inference_context *context)
{
  /* Keeping codec ownership in the caller avoids indeterminate cleanup locals after longjmp. */
  if (setjmp(reader->jump)) return false;
  reader->codec.err = jpeg_std_error(&reader->failure);
  reader->emit = reader->failure.emit_message;
  reader->failure.emit_message = jpeg_message;
  reader->failure.error_exit = jpeg_failure;
  reader->codec.client_data = reader;
  reader->created = true;
  jpeg_create_decompress(&reader->codec);
  jpeg_stdio_src(&reader->codec, reader->input);
  if (jpeg_read_header(&reader->codec, TRUE) != JPEG_HEADER_OK ||
      !reader->codec.image_width || !reader->codec.image_height ||
      reader->codec.image_width > 8192 || reader->codec.image_height > 8192) return false;
  reader->codec.out_color_space = JCS_RGB;
  if (!jpeg_start_decompress(&reader->codec) || reader->codec.output_components != 3) return false;
  const struct inference_image_spec *spec = &context->model.image;
  if (!spec->width || !spec->height || !spec->channels || spec->units != INFERENCE_PIXEL ||
      (spec->type != ONNX_FLOAT32 && spec->type != ONNX_UINT8 && spec->type != ONNX_INT8)) return false;
  struct inference_image_transform *transform = &context->transform;
  *transform = (struct inference_image_transform){
    .source_width = reader->codec.output_width, .source_height = reader->codec.output_height,
    .input_width = spec->width, .input_height = spec->height,
    .content_width = spec->width, .content_height = spec->height
  };
  if (spec->letterbox) {
    const double horizontal = (double)spec->width / transform->source_width;
    const double vertical = (double)spec->height / transform->source_height;
    const double scale = horizontal < vertical ? horizontal : vertical;
    transform->content_width = (unsigned)floor(transform->source_width * scale + 0.5);
    transform->content_height = (unsigned)floor(transform->source_height * scale + 0.5);
    if (!transform->content_width) transform->content_width = 1;
    if (!transform->content_height) transform->content_height = 1;
    transform->left = (spec->width - transform->content_width) / 2;
    transform->top = (spec->height - transform->content_height) / 2;
  }
  const size_t element = spec->type == ONNX_FLOAT32 ? sizeof(float) : 1;
  const size_t row_bytes = (size_t)transform->content_width * spec->channels * element;
  struct inference_image_spec row_spec = *spec;
  row_spec.width = transform->content_width;
  row_spec.height = 1;
  row_spec.letterbox = false;
  row_spec.layout = INFERENCE_NHWC;
  /* Padding is normalized exactly like image samples, including quantized input ranges. */
  for (unsigned row = 0; row < spec->height; ++row) {
    for (unsigned column = 0; column < spec->width; ++column) {
      for (unsigned channel = 0; channel < spec->channels; ++channel) {
        const double padding = (spec->padding - spec->mean[channel]) * spec->scale[channel];
        if (!isfinite(padding)) return false;
        const size_t index = spec->layout == INFERENCE_NCHW ?
          ((size_t)channel * spec->height + row) * spec->width + column :
          ((size_t)row * spec->width + column) * spec->channels + channel;
        if (spec->type == ONNX_FLOAT32) {
          ((float *)context->model.inputs[0].data)[index] = (float)padding;
          if (!isfinite(((float *)context->model.inputs[0].data)[index])) return false;
        } else if (spec->type == ONNX_UINT8) {
          if (padding < 0 || padding > UINT8_MAX) return false;
          ((uint8_t *)context->model.inputs[0].data)[index] = (uint8_t)lround(padding);
        } else {
          if (padding < INT8_MIN || padding > INT8_MAX) return false;
          ((int8_t *)context->model.inputs[0].data)[index] = (int8_t)lround(padding);
        }
      }
    }
  }
  unsigned target_row = 0;
  /* Consume every source row to validate the JPEG, but retain only rows selected by nearest resize. */
  while (reader->codec.output_scanline < reader->codec.output_height) {
    const unsigned source_row = reader->codec.output_scanline;
    JSAMPROW row_pointer = reader->row;
    if (jpeg_read_scanlines(&reader->codec, &row_pointer, 1) != 1) return false;
    while (target_row < transform->content_height) {
      const unsigned selected = (unsigned)(((uint64_t)target_row * 2 + 1) * transform->source_height /
                                            ((uint64_t)transform->content_height * 2));
      if (selected != source_row) break;
      const struct inference_image image = {.planes = {reader->row},
        .lengths = {(size_t)transform->source_width * 3}, .strides = {(size_t)transform->source_width * 3},
        .width = transform->source_width, .height = 1, .format = INFERENCE_RGB8};
      struct inference_image_transform unused;
      char error[128];
      /* Reuse the live-image conversion so color order and normalization cannot drift between sources. */
      if (!inference_image_prepare(&image, &row_spec, reader->prepared.bytes, row_bytes, &unused, error, sizeof(error))) return false;
      for (unsigned column = 0; column < transform->content_width; ++column) {
        for (unsigned channel = 0; channel < spec->channels; ++channel) {
          const size_t input = ((size_t)column * spec->channels + channel) * element;
          const unsigned final_row = target_row + transform->top, final_column = column + transform->left;
          const size_t output = (spec->layout == INFERENCE_NCHW ?
            ((size_t)channel * spec->height + final_row) * spec->width + final_column :
            ((size_t)final_row * spec->width + final_column) * spec->channels + channel) * element;
          for (size_t byte = 0; byte < element; ++byte)
            ((unsigned char *)context->model.inputs[0].data)[output + byte] = reader->prepared.bytes[input + byte];
        }
      }
      ++target_row;
    }
  }
  /* libjpeg may repair truncated input; repaired pixels are not qualified model evidence. */
  return target_row == transform->content_height && jpeg_finish_decompress(&reader->codec) && !reader->failure.num_warnings;
}

bool inference_jpeg_prepare(struct inference_context *context, const char *filename,
                             char *error, size_t capacity)
{
  if (!error || !capacity) return false;
  error[0] = '\0';
  if (!context || !filename || !filename[0]) {
    snprintf(error, capacity, "Invalid inference JPEG request");
    return false;
  }
  if (!context->storage || context->model.input_count != 1 || !context->model.image.channels) {
    snprintf(error, capacity, "Model does not declare one image input");
    return false;
  }
  const struct inference_image_spec *spec = &context->model.image;
  const size_t element = spec->type == ONNX_FLOAT32 ? sizeof(float) : 1;
  if (!context->model.inputs[0].data || !spec->width || spec->width > 2048 ||
      !spec->height || spec->height > 2048 || (spec->channels != 1 && spec->channels != 3) ||
      (spec->layout != INFERENCE_NCHW && spec->layout != INFERENCE_NHWC) ||
      (spec->type != ONNX_FLOAT32 && spec->type != ONNX_UINT8 && spec->type != ONNX_INT8) ||
      spec->units != INFERENCE_PIXEL || !isfinite(spec->padding) ||
      context->model.inputs[0].bytes != (size_t)spec->width * spec->height * spec->channels * element ||
      (spec->type == ONNX_FLOAT32 && (uintptr_t)context->model.inputs[0].data % _Alignof(float))) {
    snprintf(error, capacity, "Invalid JPEG tensor storage or preprocessing contract");
    return false;
  }
  for (unsigned channel = 0; channel < spec->channels; ++channel) {
    if (!isfinite(spec->scale[channel]) || !spec->scale[channel] || !isfinite(spec->mean[channel])) {
      snprintf(error, capacity, "Invalid JPEG normalization contract");
      return false;
    }
  }
  struct stat status;
  if (stat(filename, &status) || !S_ISREG(status.st_mode) || status.st_size <= 0 || status.st_size > 128 * 1024 * 1024) {
    snprintf(error, capacity, "Invalid inference JPEG: %s", filename);
    return false;
  }
  struct jpeg_reader reader = {0};
  reader.input = fopen(filename, "rb");
  if (!reader.input) {
    snprintf(error, capacity, "Cannot read inference JPEG: %s", filename);
    return false;
  }
  bool valid = decode_jpeg(&reader, context);
  if (reader.created) jpeg_destroy_decompress(&reader.codec);
  if (fclose(reader.input)) valid = false;
  if (!valid) snprintf(error, capacity, "Invalid JPEG or image preprocessing contract: %s", filename);
  else error[0] = '\0';
  return valid;
}