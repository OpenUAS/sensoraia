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

/** @file lwir_inference.c
 * @brief Classifier evidence with explicit display-versus-radiometric model inputs.
 * @details A class probability is neither a temperature correction nor a geographic hotspot result.
 * One model is held in module-static storage (no heap here; the inference context
 * owns its own storage). Pixel-domain models read the display JPEG; other models
 * require the embedded temperature layer. Results go to an exclusively created
 * "<photo>.lwir.ini" sidecar; the photograph itself is never modified. */
#include "lwir_inference.h"
#include "lwir_container.h"
#include "lwir_frame.h"
#include "inference_jpeg.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static struct inference_context network; ///< The single open classifier; storage != NULL means open.
static unsigned char thermal_plane[LWIR_MAX_WIDTH * LWIR_MAX_HEIGHT * 2]; ///< Kelvin*64 samples for radiometric models.
static double score_threshold; ///< Inclusive acceptance threshold set at open.
static unsigned class_index; ///< Reported class within the probability vector.

/**
 * @brief Open and qualify the LWIR classifier (see lwir_inference.h).
 * @return True if open (or already open); false on bad arguments, open failure or
 * an unsupported model contract, leaving nothing open.
 * @details Requires exactly one input and one float32 output of at most 4096
 * classes containing @p category; non-pixel (temperature) inputs must be single
 * channel.
 * @note Already open: returns true immediately and keeps the earlier threshold/category.
 */
bool lwir_inference_open(const struct inference_settings *settings, double threshold, unsigned category,
                          char *error, size_t capacity)
{
  if (!error || !capacity) return false;
  /* Pre-filled so every early false return carries a diagnostic. */
  snprintf(error, capacity, "Invalid LWIR classifier settings or probability threshold");
  if (!settings || !isfinite(threshold) || threshold < 0 || threshold > 1) return false;
  if (network.storage) return true;
  if (!inference_context_open(&network, settings, "lwir_classifier_v1", error, capacity)) return false;
  if (network.model.input_count != 1 || network.model.output_count != 1 || !network.model.image.channels ||
      network.model.outputs[0].type != ONNX_FLOAT32 || network.model.outputs[0].bytes / sizeof(float) > 4096 ||
      category >= network.model.outputs[0].bytes / sizeof(float) ||
      (network.model.image.units != INFERENCE_PIXEL && network.model.image.channels != 1)) {
    snprintf(error, capacity, "LWIR classifier requires one declared image/temperature input and float probabilities");
    inference_context_close(&network);
    return false;
  }
  score_threshold = threshold;
  class_index = category;
  return true;
}

/**
 * @brief Classify one saved photo and publish its evidence sidecar.
 * @return True once the sidecar is fully written and fsynced. False if the model
 * is not open, input is missing/invalid, any probability is outside [0,1],
 * hashing fails, or the sidecar exists or cannot be written (a partial sidecar
 * is removed). The photograph is untouched in every case.
 * @details Records image and model hashes, input units, synthetic-temperature
 * flag, source size, class, probability and accepted (probability >= threshold).
 * @note synthetic_temperature is only meaningful for temperature-input models;
 * pixel-input models always record 0.
 * @note The image hash is computed after inference by a separate file read.
 */
bool lwir_inference_image(const char *filename, unsigned long long sequence, char *error, size_t capacity)
{
  if (!error || !capacity) return false;
  snprintf(error, capacity, "LWIR classifier is unavailable or its input is invalid");
  if (!filename || !network.storage) return false;
  const size_t length = strnlen(filename, INFERENCE_PATH_CAPACITY);
  if (length < 4 || length >= INFERENCE_PATH_CAPACITY || strcmp(filename + length - 4, ".jpg")) {
    snprintf(error, capacity, "LWIR classifier requires a bounded .jpg input path");
    return false;
  }
  bool synthetic = false;
  if (network.model.image.units == INFERENCE_PIXEL) {
    if (!inference_jpeg_prepare(&network, filename, error, capacity)) return false;
  } else {
    /* A display JPEG cannot replace missing physical temperature samples, even if it looks thermal. */
    unsigned width = 0, height = 0;
    if (lwir_read_temperature_layer(filename, thermal_plane, sizeof(thermal_plane), &width, &height, &synthetic) != 1) {
      snprintf(error, capacity, "Model requires a valid radiometric temperature plane; display pixels are not temperatures");
      return false;
    }
    const struct inference_image image = {.planes = {thermal_plane}, .lengths = {(size_t)width * height * 2},
      .strides = {(size_t)width * 2}, .width = width, .height = height, .format = INFERENCE_KELVIN64};
    if (!inference_context_prepare_image(&network, &image, error, capacity)) return false;
  }
  if (!inference_context_run(&network, error, capacity)) return false;
  const float *scores = network.model.outputs[0].data;
  const size_t classes = network.model.outputs[0].bytes / sizeof(float);
  /* Range check only; probabilities are not required to sum to 1. */
  for (size_t index = 0; index < classes; ++index) {
    if (!isfinite(scores[index]) || scores[index] < 0 || scores[index] > 1) {
      snprintf(error, capacity, "Invalid thermal classifier probability");
      return false;
    }
  }
  char output_path[INFERENCE_PATH_CAPACITY], digest[65];
  if (!inference_file_sha256(filename, digest)) return false;
  const int count = snprintf(output_path, sizeof(output_path), "%.*s.lwir.ini", (int)(length - 4), filename);
  if (count < 0 || (size_t)count >= sizeof(output_path)) return false;
  /* Exclusive creation prevents a retry from silently replacing evidence for an earlier input. */
  FILE *output = fopen(output_path, "wx");
  if (!output) { snprintf(error, capacity, "Cannot exclusively create LWIR classifier evidence: %s", output_path); return false; }
  fprintf(output, "schema=LWIR_ONNX_CLASS_V1\nsequence=%llu\nimage_sha256=%s\nmodel_sha256=%s\n"
    "model_descriptor_sha256=%s\ninput_units=%u\nsynthetic_temperature=%u\n"
    "source_width=%u\nsource_height=%u\nclass=%u\nprobability=%.9g\naccepted=%u\n",
    sequence, digest, network.model.graph_sha256, network.model.descriptor_sha256,
    (unsigned)network.model.image.units, synthetic ? 1U : 0U,
    network.transform.source_width, network.transform.source_height, class_index,
    (double)scores[class_index], scores[class_index] >= score_threshold ? 1U : 0U);
  bool valid = !ferror(output) && fflush(output) == 0 && fsync(fileno(output)) == 0;
  if (fclose(output)) valid = false;
  /* Never leave a truncated sidecar that a reader could mistake for evidence. */
  if (!valid) { unlink(output_path); snprintf(error, capacity, "Cannot publish LWIR classifier evidence"); }
  return valid;
}

/** @brief Release the classifier; harmless when not open. Threshold/category are reset by the next open. */
void lwir_inference_close(void)
{
  inference_context_close(&network);
}