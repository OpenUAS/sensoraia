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

/** @file loom_capture.cpp
 * @brief LOOM executable entry point and neural-feature capture policy.
 * @details Shared IMX500 mechanics know nothing about feature interpretation or
 * waypoint selection. This application never calls the object-box decoder. */
#include "../../src/version.h"
#include "../../src/inference_worker.h"
#include "../../src/image_exif.h"
#include "loom_features.h"
#include "loom_return.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <cerrno>
#include <sys/stat.h>

#ifndef SENSORAIA_INFERENCE_ONLY
#include "../../src/imx500_camera.hpp"
#include "../../src/inference_capture.hpp"
#endif

/** @brief Dispatch the saved-photo CLI through its C linkage boundary.
 * @param argc Argument count including the command name.
 * @param argv Arguments for the saved-photo path.
 * @return CLI exit status; successful processing may report no match.
 * @details This entry does not acquire a sensor. Keep the saved-photo CLI
 * separate from the native capture worker. */
extern "C" int loomcam_photo_main(int argc, char **argv);

namespace {

/** @brief Establish finite-storage feature transport limits without claiming a trained visual match. */
bool validate_onnx_features(void *configuration, const inference_context *context, char *error, size_t capacity)
{
  const auto &options = *static_cast<const loom_return_options *>(configuration);
  if (!loom_return_options_valid(&options)) {
    std::snprintf(error, capacity, "Inconsistent LOOM return limits");
    return false;
  }
  if (context->model.input_count != 1 || !context->model.image.channels) {
    std::snprintf(error, capacity, "LOOM requires one declared image input");
    return false;
  }
  size_t values = 0;
  for (size_t index = 0; index < context->model.output_count; ++index) {
    const auto &tensor = context->model.outputs[index];
    if (tensor.type != ONNX_FLOAT32 || tensor.bytes / sizeof(float) > 4096 - values) {
      std::snprintf(error, capacity, "LOOM requires bounded float32 feature outputs");
      return false;
    }
    values += tensor.bytes / sizeof(float);
  }
  if (options.frame != LOOM_RETURN_NONE && options.frame != LOOM_RETURN_SIMULATED &&
      (context->model.output_count != 1 || (values != 2 && values != 3))) {
    std::snprintf(error, capacity, "Declared LOOM return vectors require one float32 output with two or three values");
    return false;
  }
  return true;
}

/** @brief Embed only in an exclusively owned private JPEG so published or hard-linked references stay immutable. */
bool embed_onnx_features(const inference_context *context, const char *image, uint32_t sequence)
{
  /* The worker serializes callbacks; reusable bounded EXIF storage avoids a large per-request stack frame. */
  static exif_document document;
  static unsigned char payload[EXIF_MAX_PAYLOAD], record[LOOM_ONNX_FEATURE_CAPACITY];
  const size_t record_size = loom_onnx_features_pack(record, sizeof(record), &context->model,
    context->capture_identity ? context->frame_sequence : sequence, context->sensor_timestamp_ns);
  if (!record_size) return false;
  exif_document_init(&document);
  size_t payload_size = 0;
  if (exif_read_jpeg(image, payload, sizeof(payload), &payload_size) == 0) {
    exif_view view;
    if (exif_parse(&view, payload, payload_size) || image_exif_import(&document, &view)) return false;
  } else if (errno != ENODATA) return false;
  unsigned char *destination = exif_document_reserve(&document, EXIF_CAMERA, LOOM_ONNX_FEATURES_TAG,
                                                      EXIF_UNDEFINED, static_cast<uint32_t>(record_size));
  if (!destination) return false;
  for (size_t index = 0; index < record_size; ++index) destination[index] = record[index];
  if (exif_serialize(&document, payload, sizeof(payload), &payload_size)) return false;
  FILE *output = std::fopen(image, "r+b");
  if (!output) return false;
  struct stat info{};
  bool valid = !fstat(fileno(output), &info) && S_ISREG(info.st_mode) && info.st_nlink == 1 &&
    exif_update_jpeg(output, payload, payload_size) == 0 && !std::fflush(output) && !fsync(fileno(output));
  if (std::fclose(output)) valid = false;
  return valid;
}

/** @brief Retain exact model/preprocessing identity with features; ordinary offline analysis never mutates its input. */
bool write_onnx_features(void *configuration, const inference_context *context,
                          const char *image, uint32_t sequence, char *error, size_t capacity)
{
  const auto &options = *static_cast<const loom_return_options *>(configuration);
  const auto &model = context->model;
  size_t total = 0;
  for (size_t slot = 0; slot < model.output_count; ++slot) {
    if (model.outputs[slot].type != ONNX_FLOAT32 || model.outputs[slot].bytes / sizeof(float) > 4096 - total) {
      std::snprintf(error, capacity, "LOOM features require at most 4096 float32 output values");
      return false;
    }
    total += model.outputs[slot].bytes / sizeof(float);
    const auto *values = static_cast<const float *>(model.outputs[slot].data);
    for (size_t index = 0; index < model.outputs[slot].bytes / sizeof(float); ++index)
      if (!std::isfinite(values[index])) { std::snprintf(error, capacity, "Nonfinite LOOM feature"); return false; }
  }
  if (context->private_image && !embed_onnx_features(context, image, sequence)) {
    std::snprintf(error, capacity, "Cannot embed model-identified LOOM feature evidence");
    return false;
  }
  char filename[INFERENCE_PATH_CAPACITY], digest[65];
  const size_t length = std::strlen(image);
  if (length < 4 || std::strcmp(image + length - 4, ".jpg") || !inference_file_sha256(image, digest)) return false;
  const int count = std::snprintf(filename, sizeof(filename), "%.*s.loom.ini", static_cast<int>(length - 4), image);
  if (count < 0 || static_cast<size_t>(count) >= sizeof(filename)) return false;
  FILE *output = std::fopen(filename, "wx");
  if (!output) { std::snprintf(error, capacity, "Cannot exclusively create LOOM features: %s", filename); return false; }
  std::fprintf(output, "schema=LOOM_ONNX_FEATURES_V1\nsequence=%u\nimage_sha256=%s\n"
               "model_sha256=%s\nmodel_descriptor_sha256=%s\noutput_count=%zu\n",
               sequence, digest, model.graph_sha256, model.descriptor_sha256, model.output_count);
  if (options.frame == LOOM_RETURN_NORTH_EAST || options.frame == LOOM_RETURN_BODY_FORWARD_RIGHT) {
    const auto *values = static_cast<const float *>(model.outputs[0].data);
    std::fprintf(output, "return_vector_frame=%s\nreturn_first=%.9g\nreturn_second=%.9g\nreturn_confidence=%.9g\n",
      options.frame == LOOM_RETURN_NORTH_EAST ? "north_east" : "body_forward_right",
      static_cast<double>(values[0]), static_cast<double>(values[1]),
      model.outputs[0].bytes == 3 * sizeof(float) ? static_cast<double>(values[2]) : 1.0);
  } else if (options.frame == LOOM_RETURN_SIMULATED) {
    std::fprintf(output, "return_vector_frame=simulated\n");
  }
  for (size_t slot = 0; slot < model.output_count; ++slot) {
    const auto &tensor = model.outputs[slot];
    std::fprintf(output, "output.%zu.name=%s\noutput.%zu.shape=", slot, tensor.name, slot);
    for (size_t axis = 0; axis < tensor.rank; ++axis)
      std::fprintf(output, "%s%lld", axis ? "," : "", static_cast<long long>(tensor.dimensions[axis]));
    std::fprintf(output, "\noutput.%zu.values=", slot);
    const auto *values = static_cast<const float *>(tensor.data);
    for (size_t index = 0; index < tensor.bytes / sizeof(float); ++index)
      std::fprintf(output, "%s%.9g", index ? "," : "", static_cast<double>(values[index]));
    std::fputc('\n', output);
  }
  bool written = !std::ferror(output) && std::fflush(output) == 0 && fsync(fileno(output)) == 0;
  if (std::fclose(output)) written = false;
  if (!written) { unlink(filename); std::snprintf(error, capacity, "Cannot publish LOOM features"); }
  return written;
}

#ifndef SENSORAIA_INFERENCE_ONLY

/** @brief Qualify feature transport without interpreting it as a return direction.
 * @param frame Borrowed same-exposure neural values and shape metadata.
 * @return True for a bounded finite tensor contract, not a navigable match. */
bool features_ready(const imx500::FrameView &frame)
{
  return loom_features_valid(frame.tensor.data(), frame.tensor.size(),
                             frame.tensor_info.data(), frame.tensor_info.size());
}

/** @brief Serialize only this retained exposure's feature evidence.
 * @param frame Immutable view whose lease remains alive during serialization.
 * @param filename Exclusive-create analysis path supplied by the publication owner.
 * @return True when the core publishes complete feature evidence.
 * @details Keep camera request identity separate from the FC photo number;
 * downstream code must establish their association rather than guessing it. */
bool write_features(const imx500::FrameView &frame, exif_document *document)
{
  if (!features_ready(frame)) return true;
  const size_t size = loom_features_size(frame.tensor.size());
  auto *record = exif_document_reserve(document, EXIF_CAMERA, LOOM_FEATURES_TAG, EXIF_UNDEFINED,
                                        static_cast<uint32_t>(size));
  return record && loom_features_pack(record, size, frame.tensor.data(), frame.tensor.size(),
                                       frame.tensor_info.data(), frame.tensor_info.size(),
                                       frame.sequence, frame.sensor_timestamp_ns);
}
#endif

} // namespace

/** @brief Dispatch saved-photo processing without opening a second camera owner.
 * @param argc Command-line token count.
 * @param argv Saved-photo or neutral capture-worker mode arguments.
 * @return Zero for success, two for syntax errors, one for capture runtime failure.
 * @details Capture retains features; saved-photo mode validates declared return
 * vectors and observation identity. Generic tensors are not navigation evidence. */
int main(int argc, char **argv)
{
  if (argc == 2 && !std::strcmp(argv[1], "--help")) {
    std::puts(LOOMCAM_BUILD_VERSION);
    std::puts("Usage: loomcam --analyze FILE.jpg | --analyze-server | --check-model [--settings FILE]\n"
              "       loomcam --capture --settings FILE /absolute/photo.jpg\n"
              "       loomcam --serve --settings FILE WIDTH HEIGHT\n"
              "       loomcam --image FILE --sequence N [--reference-dir DIR]\n"
              "       loomcam --return-server [--settings FILE] [--simulation] [--seed N]\n"
              "       loomcam --simulation-mode [--settings FILE] [--strict]\n"
              "       loomcam --self-test | --version\n"
              "Feature-only is the default; declared return vectors use guarded T1 policy.\n"
              "Strict model selection rejects unavailable/feature-only assignments.\n"
              "Return-server skips rejected vectors unless --simulation is explicit; ACK budget is 4 seconds.");
    return 0;
  }
  loom_return_options options;
  loom_return_options_init(&options);
  const inference_image_processor onnx_processor{"loomcam", "loom_cam", "loom_features_v1",
                                                 loom_return_setting, &options, write_onnx_features, validate_onnx_features, ".loom.ini"};
  const int analyzed = inference_worker_cli(argc, argv, &onnx_processor);
  if (analyzed >= 0) return analyzed;
  if (argc > 1 && (std::strcmp(argv[1], "--image") == 0
                  || std::strcmp(argv[1], "--return-server") == 0
                  || std::strcmp(argv[1], "--simulation-mode") == 0
                  || std::strcmp(argv[1], "--self-test") == 0
                  || std::strcmp(argv[1], "--help") == 0)) {
    return loomcam_photo_main(argc, argv);
  }
  if (argc == 2 && std::strcmp(argv[1], "--version") == 0) {
    std::puts(LOOMCAM_BUILD_VERSION);
    return 0;
  }
#ifdef SENSORAIA_INFERENCE_ONLY
  std::fprintf(stderr, "Usage: loomcam --analyze FILE.jpg | --analyze-server | --check-model [--settings FILE]\n");
  return 2;
#else
  const imx500::FrameProcessor processor{nullptr, features_ready, nullptr, write_features};
  const int configured = inference_capture_cli(argc, argv, onnx_processor, processor);
  if (configured >= 0) return configured;
  if (argc < 3 || (std::strcmp(argv[1], "--probe") != 0
                  && std::strcmp(argv[1], "--serve") != 0
                  && std::strcmp(argv[1], "--capture") != 0)) {
    std::fprintf(stderr,
                 "Usage: loomcam --probe nn_loom.rpk\n"
                 "       loomcam --serve nn_loom.rpk WIDTH HEIGHT\n"
                 "       loomcam --capture nn_loom.rpk /absolute/photo.jpg\n"
                 "       loomcam --image FILE --sequence N [--reference-dir DIR] [--model FILE]\n"
                 "       loomcam --self-test | --version\n");
    return 2;
  }
  return imx500::run_worker(argc, argv, processor, "loomcam");
#endif
}