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

/** @file inference_image.h
 * @brief Source-independent, bounded model preprocessing with reversible coordinate bookkeeping. */
#ifndef SENSORAIA_INFERENCE_IMAGE_H
#define SENSORAIA_INFERENCE_IMAGE_H

#include "onnx_session.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Accepted source layouts; Kelvin64 is little-endian native radiometric evidence. */
enum inference_pixel_format { INFERENCE_RGB8, INFERENCE_GRAY8, INFERENCE_YUV420, INFERENCE_KELVIN64 };
/** @brief Destination memory order, independent of source strides. */
enum inference_layout { INFERENCE_NCHW, INFERENCE_NHWC };
/** @brief Explicit physical input domain; display brightness is never inferred to be a temperature. */
enum inference_units { INFERENCE_PIXEL, INFERENCE_RAW_KELVIN64, INFERENCE_KELVIN, INFERENCE_CELSIUS };

/** @brief Borrowed immutable planes; lengths/strides prove all addressed rows are available. */
struct inference_image {
  const unsigned char *planes[3];
  size_t lengths[3], strides[3];
  unsigned width, height;
  enum inference_pixel_format format;
};

/** @brief Model-owned preprocessing; normalized samples are (sample - mean) * scale. */
struct inference_image_spec {
  unsigned width, height, channels;
  enum inference_layout layout;
  enum inference_units units;
  enum onnx_element_type type;
  bool letterbox, bgr;
  float scale[3], mean[3], padding;
};

/** @brief Exact resized content rectangle, including rounded letterbox offsets. */
struct inference_image_transform {
  unsigned source_width, source_height, input_width, input_height;
  unsigned content_width, content_height, left, top;
};

/** @brief Convert with the declared nearest-neighbor policy, not a guessed training transform.
 * @param image Immutable bounded source planes.
 * @param spec Validated model preprocessing contract.
 * @param tensor Aligned writable destination, separate from the source planes.
 * @param capacity Exact destination tensor bytes, not an oversized capacity hint.
 * @param transform Source-to-input rectangle written for later result projection.
 * @param error Writable failure diagnostic.
 * @param error_capacity Available bytes in error.
 * @return True after complete finite conversion; discard partial output/transform on false. */
bool inference_image_prepare(const struct inference_image *image,
                              const struct inference_image_spec *spec,
                              void *tensor, size_t capacity,
                              struct inference_image_transform *transform,
                              char *error, size_t error_capacity);
/** @brief Map a model-input point back to source pixels, rejecting letterbox-only coordinates.
 * @param transform Rectangle produced by successful preparation.
 * @param input_x Horizontal model-input pixel coordinate.
 * @param input_y Vertical model-input pixel coordinate.
 * @param source_x Writable horizontal source coordinate.
 * @param source_y Writable vertical source coordinate.
 * @return True for a finite point inside the actual content, including its boundary. */
bool inference_image_point(const struct inference_image_transform *transform,
                            double input_x, double input_y, double *source_x, double *source_y);

#ifdef __cplusplus
}
#endif
#endif