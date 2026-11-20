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

/** @file inference_worker.c
 * @brief Persistent saved-picture inference with explicit request correlation and module-owned results.
 * @details No sensor is acquired here; simulated and real saved images enter the same model path. */
#include "inference_worker.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

/** @brief Publish a result only after complete preparation and execution for this picture. */
static bool analyze_image(struct inference_context *context,
                           const struct inference_image_processor *processor,
                           const char *image, uint32_t sequence, char *error, size_t capacity)
{
  return inference_jpeg_prepare(context, image, error, capacity) &&
    inference_context_run(context, error, capacity) &&
    processor->write(processor->context, context, image, sequence, error, capacity);
}

/** @brief Probe sibling INT8 then FP32 ONNX exports when automatic settings point to an RPK or unsupported INT8 graph. */
static bool open_fallback_onnx(struct inference_context *context,
                                struct inference_settings *settings,
                                const struct inference_image_processor *processor,
                                char *error, size_t capacity)
{
  const char *suffix = strrchr(settings->model, '.');
  const char *slash = strrchr(settings->model, '/');
  if (!suffix || !slash || (settings->backend != INFERENCE_IMX500 &&
      strcmp(slash + 1, "vehicle_yolo11n_int8.onnx") != 0)) return false;
  static const char *const candidates[][2] = {
    {"vehicle_yolo11n_int8.onnx", "vehicle_yolo11n_int8.ini"},
    {"vehicle_yolo11n_fp32.onnx", "vehicle_yolo11n_fp32.ini"}
  };
  const int directory_length = (int)(slash - settings->model);
  for (size_t index = 0; index < sizeof(candidates) / sizeof(candidates[0]); ++index) {
    struct inference_settings candidate = *settings;
    candidate.backend = INFERENCE_ONNX;
    const int model_len = snprintf(candidate.model, sizeof(candidate.model), "%.*s/%s",
                                   directory_length, settings->model, candidates[index][0]);
    const int desc_len = snprintf(candidate.descriptor, sizeof(candidate.descriptor), "%.*s/%s",
                                  directory_length, settings->model, candidates[index][1]);
    if (model_len < 0 || (size_t)model_len >= sizeof(candidate.model) ||
        desc_len < 0 || (size_t)desc_len >= sizeof(candidate.descriptor)) continue;
    char probe_error[2048];
    if (inference_context_open(context, &candidate, processor->adapter, probe_error, sizeof(probe_error))) {
      if (!processor->validate || processor->validate(processor->context, context, probe_error, sizeof(probe_error))) {
        *settings = candidate;
        if (capacity) error[0] = '\0';
        return true;
      }
      inference_context_close(context);
    }
    if (probe_error[0]) snprintf(error, capacity, "%s", probe_error);
  }
  return false;
}

int inference_worker_cli(int argc, char **argv, const struct inference_image_processor *processor)
{
  bool server = false, check = false, selected = false, private_images = false;
  const char *image = NULL, *settings_path = NULL;
  uint32_t sequence = 1;
  if (!processor || !processor->write || !processor->application || !processor->module || !processor->adapter) return 2;
  for (int index = 1; index < argc; ++index) {
    if (!strcmp(argv[index], "--analyze") || !strcmp(argv[index], "--analyze-server") || !strcmp(argv[index], "--check-model")) selected = true;
  }
  if (!selected) return -1;
  for (int index = 1; index < argc; ++index) {
    if (!strcmp(argv[index], "--analyze-server")) { if (server) return 2; server = true; }
    else if (!strcmp(argv[index], "--check-model")) { if (check) return 2; check = true; }
    else if (!strcmp(argv[index], "--private-images")) { if (private_images) return 2; private_images = true; }
    else {
      const char *option = argv[index];
      if (++index == argc) return 2;
      if (!strcmp(option, "--analyze")) { if (image) return 2; image = argv[index]; }
      else if (!strcmp(option, "--settings")) { if (settings_path) return 2; settings_path = argv[index]; }
      else if (!strcmp(option, "--sequence")) {
        uint64_t number;
        if (!ini_uint(argv[index], &number) || !number || number > UINT32_MAX) return 2;
        sequence = (uint32_t)number;
      } else return 2;
    }
  }
  if ((unsigned)server + (unsigned)check + (unsigned)(image != NULL) != 1) return 2;
  char discovered[INFERENCE_PATH_CAPACITY], error[2048];
  if (!settings_path) {
    if (!inference_settings_find(processor->application, processor->module, discovered, sizeof(discovered))) {
      fprintf(stderr, "%s: settings not found; use --settings FILE\n", processor->application);
      return 1;
    }
    settings_path = discovered;
  }
  struct inference_settings settings;
  inference_settings_init(&settings, INFERENCE_ONNX);
  if (!inference_settings_read(&settings, settings_path, processor->setting, processor->context, error, sizeof(error))) {
    fprintf(stderr, "%s: %s\n", processor->application, error);
    return 1;
  }
  struct inference_context context = {0};
  if (!inference_context_open(&context, &settings, processor->adapter, error, sizeof(error)) &&
      !open_fallback_onnx(&context, &settings, processor, error, sizeof(error))) {
    fprintf(stderr, "%s: %s\n", processor->application, error);
    return 1;
  }
  if (processor->validate && !processor->validate(processor->context, &context, error, sizeof(error))) {
    fprintf(stderr, "%s: %s\n", processor->application, error);
    inference_context_close(&context);
    return 1;
  }
  int result = 0;
  context.private_image = private_images;
  /* Only a producer may opt into mutation; normal analysis must leave published source images immutable. */
  if (check) {
    if (!inference_context_run(&context, error, sizeof(error))) result = 1;
    else printf("%s model ready adapter=%s sha256=%s\n", processor->application, context.model.adapter, context.model.graph_sha256);
  } else if (image) {
    if (!analyze_image(&context, processor, image, sequence, error, sizeof(error))) result = 1;
    else puts("OK");
  } else {
    puts("READY");
    if (fflush(stdout)) result = 1;
    uint32_t previous = 0;
    char command[INFERENCE_PATH_CAPACITY + 64];
    while (!result && fgets(command, sizeof(command), stdin)) {
      const size_t length = strlen(command);
      if (!length || command[length - 1] != '\n' || strncmp(command, "ANALYZE ", 8)) { result = 2; break; }
      command[length - 1] = '\0';
      char *number = command + 8, *path = strchr(number, ' ');
      if (!path) { result = 2; break; }
      *path++ = '\0';
      uint64_t parsed;
      /* Reject replay/reordering before running the graph, not after an old result has already been written. */
      if (!ini_uint(number, &parsed) || parsed <= previous || parsed > UINT32_MAX || !path[0] || strchr(path, '\r')) { result = 2; break; }
      previous = (uint32_t)parsed;
      const bool analyzed = analyze_image(&context, processor, path, previous, error, sizeof(error));
      if (!analyzed) fprintf(stderr, "%s: %s\n", processor->application, error);
      puts(analyzed ? "OK" : "ERROR");
      if (fflush(stdout)) result = 1;
    }
    if (ferror(stdin)) result = 1;
  }
  if (result == 1) fprintf(stderr, "%s: %s\n", processor->application, error);
  inference_context_close(&context);
  return result;
}