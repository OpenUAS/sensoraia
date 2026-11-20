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

/** @file inference_image.c
 * @brief Bounded pixel/thermal conversion shared by live frames and decoded JPEG rows.
 * @details Resize, channel order and physical units are declared model policy, not camera defaults. */
#include "inference_image.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

/** @brief Prove the last addressed row fits before any stride-based pointer arithmetic. */
static bool plane_valid(const struct inference_image *image, size_t plane,
                         unsigned width, unsigned height, unsigned bytes)
{
  const size_t row = (size_t)width * bytes;
  return width && height && image->planes[plane] && image->strides[plane] >= row &&
    image->strides[plane] <= SIZE_MAX / height &&
    image->lengths[plane] >= (size_t)(height - 1) * image->strides[plane] + row;
}

/** @brief Clamp full-range JPEG/YUV color conversion without changing the raw source evidence. */
static double limited(double value)
{
  return value < 0 ? 0 : value > 255 ? 255 : value;
}

/** @brief Read one native pixel; missing thermal samples never become invented finite temperatures. */
static bool source_pixel(const struct inference_image *image, const struct inference_image_spec *spec,
                          unsigned column, unsigned row, double pixel[3])
{
  const unsigned char *source = image->planes[0] + (size_t)row * image->strides[0];
  if (image->format == INFERENCE_KELVIN64) {
    const unsigned raw = (unsigned)source[column * 2] | (unsigned)source[column * 2 + 1] << 8;
    if (!raw || raw == UINT16_MAX || spec->units == INFERENCE_PIXEL) return false;
    pixel[0] = spec->units == INFERENCE_RAW_KELVIN64 ? raw : raw / 64.0;
    if (spec->units == INFERENCE_CELSIUS) pixel[0] -= 273.15;
    pixel[1] = pixel[2] = pixel[0];
  } else if (spec->units != INFERENCE_PIXEL) return false;
  else if (image->format == INFERENCE_GRAY8) pixel[0] = pixel[1] = pixel[2] = source[column];
  else if (image->format == INFERENCE_RGB8) {
    for (size_t channel = 0; channel < 3; ++channel) pixel[channel] = source[(size_t)column * 3 + channel];
  } else {
    const double cb = image->planes[1][(size_t)(row / 2) * image->strides[1] + column / 2] - 128.0;
    const double cr = image->planes[2][(size_t)(row / 2) * image->strides[2] + column / 2] - 128.0;
    const double luma = source[column];
    pixel[0] = limited(luma + 1.402 * cr);
    pixel[1] = limited(luma - 0.344136 * cb - 0.714136 * cr);
    pixel[2] = limited(luma + 1.772 * cb);
  }
  return true;
}

bool inference_image_prepare(const struct inference_image *image,
                              const struct inference_image_spec *spec,
                              void *tensor, size_t capacity,
                              struct inference_image_transform *transform,
                              char *error, size_t error_capacity)
{
  if (!error || !error_capacity) return false;
  error[0] = '\0';
  if (!image || !spec || !tensor || !transform || !image->width || !image->height ||
      image->width > 8192 || image->height > 8192 || !spec->width || !spec->height ||
      spec->width > 2048 || spec->height > 2048 || (spec->channels != 1 && spec->channels != 3) ||
      (spec->layout != INFERENCE_NCHW && spec->layout != INFERENCE_NHWC) ||
      (spec->type != ONNX_FLOAT32 && spec->type != ONNX_UINT8 && spec->type != ONNX_INT8) ||
      image->format < INFERENCE_RGB8 || image->format > INFERENCE_KELVIN64) goto invalid;
  const unsigned pixel_bytes = image->format == INFERENCE_RGB8 ? 3 : image->format == INFERENCE_KELVIN64 ? 2 : 1;
  if (!plane_valid(image, 0, image->width, image->height, pixel_bytes)) goto invalid;
  if (image->format == INFERENCE_YUV420 &&
      (image->width % 2 || image->height % 2 ||
       !plane_valid(image, 1, image->width / 2, image->height / 2, 1) ||
       !plane_valid(image, 2, image->width / 2, image->height / 2, 1))) goto invalid;
  const size_t count = (size_t)spec->width * spec->height * spec->channels;
  const size_t element_bytes = spec->type == ONNX_FLOAT32 ? sizeof(float) : 1;
  if (count > ONNX_BUFFER_LIMIT / element_bytes || capacity != count * element_bytes || !isfinite(spec->padding)) goto invalid;
  if (spec->type == ONNX_FLOAT32 && (uintptr_t)tensor % _Alignof(float)) goto invalid;
  for (size_t channel = 0; channel < spec->channels; ++channel)
    if (!isfinite(spec->scale[channel]) || !spec->scale[channel] || !isfinite(spec->mean[channel])) goto invalid;
  *transform = (struct inference_image_transform){
    .source_width = image->width, .source_height = image->height,
    .input_width = spec->width, .input_height = spec->height,
    .content_width = spec->width, .content_height = spec->height
  };
  if (spec->letterbox) {
    /* Retain the rounded content rectangle so later box projection uses the actual resize, not an ideal scale. */
    const double horizontal = (double)spec->width / image->width;
    const double vertical = (double)spec->height / image->height;
    const double scale = horizontal < vertical ? horizontal : vertical;
    transform->content_width = (unsigned)floor(image->width * scale + 0.5);
    transform->content_height = (unsigned)floor(image->height * scale + 0.5);
    if (!transform->content_width) transform->content_width = 1;
    if (!transform->content_height) transform->content_height = 1;
    transform->left = (spec->width - transform->content_width) / 2;
    transform->top = (spec->height - transform->content_height) / 2;
  }
  for (unsigned row = 0; row < spec->height; ++row) {
    for (unsigned column = 0; column < spec->width; ++column) {
      double pixel[3] = {spec->padding, spec->padding, spec->padding};
      if (column >= transform->left && row >= transform->top &&
          column - transform->left < transform->content_width && row - transform->top < transform->content_height) {
        /* Pixel-center nearest sampling is also used by the streaming JPEG adapter. */
        unsigned source_x = (unsigned)(((uint64_t)(column - transform->left) * 2 + 1) * image->width /
                                        ((uint64_t)transform->content_width * 2));
        unsigned source_y = (unsigned)(((uint64_t)(row - transform->top) * 2 + 1) * image->height /
                                        ((uint64_t)transform->content_height * 2));
        if (source_x >= image->width) source_x = image->width - 1;
        if (source_y >= image->height) source_y = image->height - 1;
        if (!source_pixel(image, spec, source_x, source_y, pixel)) goto invalid;
      }
      if (spec->channels == 1 && image->format != INFERENCE_KELVIN64)
        pixel[0] = 0.299 * pixel[0] + 0.587 * pixel[1] + 0.114 * pixel[2];
      for (unsigned channel = 0; channel < spec->channels; ++channel) {
        const unsigned source_channel = spec->bgr && spec->channels == 3 ? 2 - channel : channel;
        const double value = (pixel[source_channel] - spec->mean[channel]) * spec->scale[channel];
        const size_t index = spec->layout == INFERENCE_NCHW ?
          ((size_t)channel * spec->height + row) * spec->width + column :
          ((size_t)row * spec->width + column) * spec->channels + channel;
        if (!isfinite(value)) goto invalid;
        if (spec->type == ONNX_FLOAT32) {
          ((float *)tensor)[index] = (float)value;
          if (!isfinite(((float *)tensor)[index])) goto invalid;
        } else if (spec->type == ONNX_UINT8) {
          if (value < 0 || value > UINT8_MAX) goto invalid;
          ((uint8_t *)tensor)[index] = (uint8_t)lround(value);
        } else {
          if (value < INT8_MIN || value > INT8_MAX) goto invalid;
          ((int8_t *)tensor)[index] = (int8_t)lround(value);
        }
      }
    }
  }
  return true;
invalid:
  snprintf(error, error_capacity, "Invalid image layout, sample or model preprocessing contract");
  return false;
}

bool inference_image_point(const struct inference_image_transform *transform,
                            double input_x, double input_y, double *source_x, double *source_y)
{
  if (!transform || !source_x || !source_y || !transform->content_width || !transform->content_height ||
      !isfinite(input_x) || !isfinite(input_y) || input_x < transform->left || input_y < transform->top ||
      input_x > transform->left + transform->content_width || input_y > transform->top + transform->content_height) return false;
  *source_x = (input_x - transform->left) * transform->source_width / transform->content_width;
  *source_y = (input_y - transform->top) * transform->source_height / transform->content_height;
  return true;
}