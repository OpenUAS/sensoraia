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

/** @file inference_capture.cpp
 * @brief One-owner capture, model preparation and no-overwrite result publication.
 * @details CPU inference runs after the frame lease ends so model latency does not hold camera DMA ownership. */
#include "inference_capture.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace {
/** @brief Ensure exceptions release the model session before its mapped input/output storage. */
struct ContextOwner {
  inference_context context{};
  ~ContextOwner() { inference_context_close(&context); }
};

/** @brief Borrowed bridge state used only while the camera grants a scoped pixel visit. */
struct Preparation {
  inference_context *context;
  char *error;
  size_t capacity;
};

/** @brief Copy only model-sized input while retaining the original frame's timing identity. */
bool prepare_pixels(const imx500::FrameView &view, const std::array<const uint8_t *, 3> &planes,
                      unsigned stride, void *data)
{
  auto &preparation = *static_cast<Preparation *>(data);
  inference_image image{};
  image.width = view.image.width;
  image.height = view.image.height;
  image.format = INFERENCE_YUV420;
  for (size_t plane = 0; plane < 3; ++plane) {
    image.planes[plane] = planes[plane];
    image.strides[plane] = plane ? stride / 2 : stride;
    image.lengths[plane] = image.strides[plane] * (plane ? image.height / 2 : image.height);
  }
  preparation.context->frame_sequence = view.sequence;
  preparation.context->sensor_timestamp_ns = view.sensor_timestamp_ns;
  preparation.context->capture_identity = true;
  return inference_context_prepare_image(preparation.context, &image, preparation.error, preparation.capacity);
}

/** @brief Permit the neutral encoder to retain a useful photograph before optional CPU analysis succeeds. */
bool retain_pixels(const imx500::FrameView &, exif_document *) { return true; }

/** @brief Preserve fake-camera JPEG bytes without reencoding, refusing overwrite and changing input files. */
bool copy_picture(const char *source, const char *destination)
{
  const int input = open(source, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
  if (input < 0) return false;
  struct stat before{}, after{};
  if (fstat(input, &before) || !S_ISREG(before.st_mode) || before.st_size <= 0 || before.st_size > 128 * 1024 * 1024) {
    close(input);
    return false;
  }
  const int output = open(destination, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (output < 0) { close(input); return false; }
  unsigned char buffer[16384];
  bool valid = true;
  off_t copied = 0;
  while (valid) {
    const ssize_t count = read(input, buffer, sizeof(buffer));
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) valid = false;
    if (count <= 0) break;
    if (copied > before.st_size - count) { valid = false; break; }
    copied += count;
    size_t written = 0;
    while (written < static_cast<size_t>(count)) {
      const ssize_t bytes = write(output, buffer + written, static_cast<size_t>(count) - written);
      if (bytes < 0 && errno == EINTR) continue;
      if (bytes <= 0) { valid = false; break; }
      written += static_cast<size_t>(bytes);
    }
  }
  valid = valid && copied == before.st_size && !fstat(input, &after) && before.st_size == after.st_size &&
    before.st_mtim.tv_sec == after.st_mtim.tv_sec && before.st_mtim.tv_nsec == after.st_mtim.tv_nsec && !fsync(output);
  if (close(input)) valid = false;
  if (close(output)) valid = false;
  if (!valid) unlink(destination);
  return valid;
}
}

int inference_capture_cli(int argc, char **argv, const inference_image_processor &processor,
                           const imx500::FrameProcessor &sensor_processor)
{
  if (argc < 4 || std::strcmp(argv[2], "--settings")) return -1;
  const bool serve = !std::strcmp(argv[1], "--serve"), probe = !std::strcmp(argv[1], "--probe");
  const bool capture = !std::strcmp(argv[1], "--capture");
  if ((!serve && !probe && !capture) || (serve && argc != 6) || (probe && argc != 4) || (capture && argc != 5)) return 2;
  inference_settings settings;
  inference_settings_init(&settings, INFERENCE_IMX500);
  char error[2048]{};
  char discovered[INFERENCE_PATH_CAPACITY];
  const char *settings_path = argv[3];
  if (!std::strcmp(settings_path, "-")) {
    if (!inference_settings_find(processor.application, processor.module, discovered, sizeof(discovered))) {
      std::fprintf(stderr, "%s: settings not found; use --settings FILE\n", processor.application);
      return 1;
    }
    settings_path = discovered;
  }
  if (!inference_settings_read(&settings, settings_path, processor.setting, processor.context, error, sizeof(error))) {
    std::fprintf(stderr, "%s: %s\n", processor.application, error);
    return 1;
  }
  if (settings.backend == INFERENCE_IMX500) {
    if (settings.source != INFERENCE_LIVE || !settings.model[0]) return 2;
    char *arguments[] = {argv[0], argv[1], settings.model,
      capture ? argv[4] : serve ? argv[4] : nullptr, serve ? argv[5] : nullptr, nullptr};
    return imx500::run_worker(serve ? 5 : capture ? 4 : 3, arguments, sensor_processor, processor.application);
  }
  if (settings.backend != INFERENCE_ONNX) return 2;
  uint64_t width = 2028, height = 1520;
  if (serve && (!ini_uint(argv[4], &width) || !ini_uint(argv[5], &height))) return 2;
  if (width > std::numeric_limits<unsigned>::max() || height > std::numeric_limits<unsigned>::max() ||
      !imx500::valid_capture_dimensions(static_cast<unsigned>(width), static_cast<unsigned>(height))) return 2;
  try {
    ContextOwner owner;
    auto &context = owner.context;
    if (!inference_context_open(&context, &settings, processor.adapter, error, sizeof(error))) throw std::runtime_error(error);
    if (processor.validate && !processor.validate(processor.context, &context, error, sizeof(error))) throw std::runtime_error(error);
    std::unique_ptr<imx500::Camera> camera;
    Preparation preparation{&context, error, sizeof(error)};
    if (settings.source == INFERENCE_LIVE) {
      camera = std::make_unique<imx500::Camera>();
      camera->open(nullptr, static_cast<unsigned>(width), static_cast<unsigned>(height));
      {
        auto frame = camera->capture();
        if (!camera->with_pixels(frame, prepare_pixels, &preparation)) throw std::runtime_error(error);
      }
    } else if (!settings.input[0] || !inference_jpeg_prepare(&context, settings.input, error, sizeof(error))) {
      throw std::runtime_error(settings.input[0] ? error : "Picture source requires input in settings");
    }
    if (!inference_context_run(&context, error, sizeof(error))) throw std::runtime_error(error);
    if (probe) { std::puts("camera and ONNX model ready"); return 0; }
    const auto shoot = [&](const char *filename, uint32_t sequence) {
      struct stat existing{};
      if (!filename || lstat(filename, &existing) == 0 || errno != ENOENT) return false;
      const std::string staged = std::string(filename) + ".onnx.pending.jpg";
      if (staged.size() >= INFERENCE_PATH_CAPACITY) return false;
      bool saved = false, prepared = false;
      if (camera) {
        auto frame = camera->capture();
        prepared = camera->with_pixels(frame, prepare_pixels, &preparation);
        const imx500::FrameProcessor photograph{nullptr, nullptr, nullptr, retain_pixels};
        saved = camera->save(frame, staged.c_str(), photograph);
      } else {
        saved = copy_picture(settings.input, staged.c_str());
        prepared = saved && inference_jpeg_prepare(&context, staged.c_str(), error, sizeof(error));
      }
      if (!saved) return false;
      /* The staged image is still exclusively producer-owned, so LOOM may embed feature evidence here. */
      context.private_image = true;
      const bool analyzed = prepared && inference_context_run(&context, error, sizeof(error)) &&
        processor.write(processor.context, &context, staged.c_str(), sequence, error, sizeof(error));
      if (!analyzed)
        std::fprintf(stderr, "%s: analysis_failed: %s\n", processor.application, error);
      std::string staged_result, published_result;
      bool result_published = false;
      if (analyzed && processor.result_suffix) {
        staged_result = staged.substr(0, staged.size() - 4) + processor.result_suffix;
        const size_t length = std::strlen(filename);
        if (length < 4 || std::strcmp(filename + length - 4, ".jpg")) {
          unlink(staged_result.c_str());
          unlink(staged.c_str());
          return false;
        }
        published_result = std::string(filename, length - 4) + processor.result_suffix;
        result_published = link(staged_result.c_str(), published_result.c_str()) == 0;
        if (!result_published) {
          unlink(staged_result.c_str());
          unlink(staged.c_str());
          return false;
        }
      }
      const bool published = link(staged.c_str(), filename) == 0;
      /* Remove only names created by this attempt; never erase a pre-existing image or analysis result. */
      if (!published && result_published) unlink(published_result.c_str());
      if (!staged_result.empty()) unlink(staged_result.c_str());
      unlink(staged.c_str());
      if (!published) return false;
      return true;
    };
    if (capture) return shoot(argv[4], 1) ? 0 : 1;
    std::puts("READY");
    if (std::fflush(stdout)) return 1;
    char command[INFERENCE_PATH_CAPACITY + 16];
    uint32_t sequence = 0;
    while (std::fgets(command, sizeof(command), stdin)) {
      const size_t length = std::strlen(command);
      if (!length || command[length - 1] != '\n' || std::strncmp(command, "SHOT ", 5) || sequence == UINT32_MAX) return 2;
      command[length - 1] = '\0';
      if (command[5] != '/' || std::strchr(command + 5, '\r')) return 2;
      std::puts(shoot(command + 5, ++sequence) ? "OK" : "ERROR");
      if (std::fflush(stdout)) return 1;
    }
    return std::ferror(stdin) ? 1 : 0;
  } catch (const std::exception &failure) {
    std::fprintf(stderr, "%s: %s\n", processor.application, failure.what());
    return 1;
  }
}