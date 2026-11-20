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

/** @file inference_settings.c
 * @brief Strict shared fields and deterministic application_settings.ini lookup.
 * @details Module callbacks own module-specific keys; this layer never guesses a model's meaning. */
#include "inference_settings.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

void inference_settings_init(struct inference_settings *settings,
                              enum inference_backend backend)
{
  if (!settings) return;
  unsigned char *bytes = (unsigned char *)settings;
  for (size_t index = 0; index < sizeof(*settings); ++index) bytes[index] = 0;
  settings->backend = backend;
  settings->threads = 1;
  settings->timeout_ms = 2000;
}

/** @brief Reject truncation and control characters before accepting configuration text. */
static bool text_copy(char *destination, size_t capacity, const char *source)
{
  if (!source || !capacity) return false;
  const size_t length = strlen(source);
  if (length >= capacity) return false;
  for (size_t index = 0; index <= length; ++index) {
    const unsigned char character = (unsigned char)source[index];
    if ((character < 32 && character) || character == 127) return false;
    destination[index] = source[index];
  }
  return true;
}

int inference_settings_field(struct inference_settings *settings,
                              const char *key, const char *value)
{
  if (!settings || !key || !value || !value[0]) return -1;
  static const char *const keys[] = {"inference_backend", "input_source", "model", "model_descriptor",
                                    "onnx_lib_path", "input", "onnx_threads", "inference_timeout_ms"};
  size_t field = 0;
  while (field < sizeof(keys) / sizeof(keys[0]) && strcmp(keys[field], key)) ++field;
  if (field == sizeof(keys) / sizeof(keys[0])) return 0;
  /* Duplicates are errors instead of making behavior depend on field order. */
  const uint32_t bit = UINT32_C(1) << field;
  if (settings->seen & bit) return -1;
  settings->seen |= bit;
  if (field == 0) {
    if (!strcmp(value, "onnx")) settings->backend = INFERENCE_ONNX;
    else if (!strcmp(value, "imx500")) settings->backend = INFERENCE_IMX500;
    else if (!strcmp(value, "native")) settings->backend = INFERENCE_NATIVE;
    else if (!strcmp(value, "auto")) settings->backend = INFERENCE_AUTO;
    else return -1;
    return 1;
  }
  if (field == 1) {
    if (!strcmp(value, "live")) settings->source = INFERENCE_LIVE;
    else if (!strcmp(value, "image")) settings->source = INFERENCE_IMAGE;
    else if (!strcmp(value, "fake")) settings->source = INFERENCE_FAKE;
    else return -1;
    return 1;
  }
  if (field >= 2 && field <= 5) {
    char *destination = field == 2 ? settings->model : field == 3 ? settings->descriptor :
                        field == 4 ? settings->library_directory : settings->input;
    return text_copy(destination, INFERENCE_PATH_CAPACITY, value) ? 1 : -1;
  }
  uint64_t number = 0;
  if (!ini_uint(value, &number) || !number || number > (field == 6 ? 32U : 120000U)) return -1;
  if (field == 6) settings->threads = (unsigned)number;
  else settings->timeout_ms = (unsigned)number;
  return 1;
}

bool inference_resolve_path(const char *base_file, const char *value,
                            char *destination, size_t capacity)
{
  if (!base_file || !value || !value[0] || !destination || !capacity) return false;
  char absolute[INFERENCE_PATH_CAPACITY], joined[INFERENCE_PATH_CAPACITY];
  if (base_file[0] == '/') {
    if (!text_copy(absolute, sizeof(absolute), base_file)) return false;
  } else {
    char working[INFERENCE_PATH_CAPACITY];
    if (!getcwd(working, sizeof(working))) return false;
    const int count = snprintf(absolute, sizeof(absolute), "%s/%s", working, base_file);
    if (count < 0 || (size_t)count >= sizeof(absolute)) return false;
  }
  char *separator = strrchr(absolute, '/');
  if (!separator) return false;
  separator[1] = '\0';
  const int count = snprintf(joined, sizeof(joined), "%s%s", value[0] == '/' ? "" : absolute, value);
  if (count < 0 || (size_t)count >= sizeof(joined)) return false;
  size_t used = 1;
  /* Normalize lexically: a packaged output path need not exist on the development machine. */
  if (capacity < 2) return false;
  destination[0] = '/';
  const char *cursor = joined;
  while (*cursor) {
    while (*cursor == '/') ++cursor;
    const char *start = cursor;
    while (*cursor && *cursor != '/') ++cursor;
    const size_t length = (size_t)(cursor - start);
    if (!length || (length == 1 && start[0] == '.')) continue;
    if (length == 2 && start[0] == '.' && start[1] == '.') {
      if (used > 1) {
        --used;
        while (used > 1 && destination[used - 1] != '/') --used;
      }
      continue;
    }
    if (length > capacity - used - 1) return false;
    for (size_t index = 0; index < length; ++index) {
      const unsigned char character = (unsigned char)start[index];
      if (character < 32 || character == 127) return false;
      destination[used++] = (char)character;
    }
    if (used >= capacity - 1) return false;
    destination[used++] = '/';
  }
  if (used > 1) --used;
  destination[used] = '\0';
  return true;
}

/** @brief Borrowed dispatch state scoped to one destructive INI parse. */
struct settings_parser {
  struct inference_settings *settings;
  ini_field_handler extra;
  void *context;
};

/** @brief Delegate only unknown common keys; an invalid common key must not be accepted by a module. */
static bool settings_field(void *context, const char *key, const char *value)
{
  struct settings_parser *parser = context;
  const int common = inference_settings_field(parser->settings, key, value);
  return common ? common > 0 : parser->extra && parser->extra(parser->context, key, value);
}

bool inference_settings_read(struct inference_settings *settings, const char *filename,
                              ini_field_handler extra, void *context,
                              char *error, size_t capacity)
{
  if (!settings || !filename || !error || !capacity) return false;
  error[0] = '\0';
  char contents[8193];
  size_t length = 0;
  struct settings_parser parser = {.settings = settings, .extra = extra, .context = context};
  if (!ini_read_file(filename, contents, sizeof(contents), &length) ||
      !ini_parse(contents, length, settings_field, &parser)) {
    snprintf(error, capacity, "Invalid inference settings: %s", filename);
    return false;
  }
  char *paths[] = {settings->model, settings->descriptor, settings->library_directory, settings->input};
  for (size_t index = 0; index < sizeof(paths) / sizeof(paths[0]); ++index) {
    if (!paths[index][0]) continue;
    char resolved[INFERENCE_PATH_CAPACITY];
    if (!inference_resolve_path(filename, paths[index], resolved, sizeof(resolved)) ||
        !text_copy(paths[index], INFERENCE_PATH_CAPACITY, resolved)) {
      snprintf(error, capacity, "Invalid inference settings path: %s", filename);
      return false;
    }
  }
  if (settings->backend == INFERENCE_AUTO) {
    const char *suffix = strrchr(settings->model, '.');
    if (!settings->model[0]) settings->backend = INFERENCE_NATIVE;
    else if (suffix && !strcmp(suffix, ".onnx")) {
      settings->backend = INFERENCE_ONNX;
      if (!settings->descriptor[0]) {
        const int count = snprintf(settings->descriptor, sizeof(settings->descriptor), "%.*s.ini",
                                   (int)(suffix - settings->model), settings->model);
        if (count < 0 || (size_t)count >= sizeof(settings->descriptor)) return false;
      }
    }
    else if (suffix && !strcmp(suffix, ".rpk")) settings->backend = INFERENCE_IMX500;
    else {
      snprintf(error, capacity, "Automatic inference requires a .onnx or .rpk model: %s", settings->model);
      return false;
    }
  }
  return true;
}

bool inference_settings_find(const char *application, const char *module,
                              char *filename, size_t capacity)
{
  if (!application || !module || !filename || !capacity ||
      strspn(application, "abcdefghijklmnopqrstuvwxyz0123456789_") != strlen(application) ||
      strspn(module, "abcdefghijklmnopqrstuvwxyz0123456789_") != strlen(module)) return false;
  char executable[INFERENCE_PATH_CAPACITY], relative[INFERENCE_PATH_CAPACITY];
  const ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length <= 0 || (size_t)length >= sizeof(executable) - 1) return false;
  executable[length] = '\0';
  for (unsigned directory = 0; directory < 5; ++directory) {
    /* Never search cwd: launching from another directory must not silently select another assignment. */
    int count;
    if (!directory) count = snprintf(relative, sizeof(relative), "%s_settings.ini", application);
    else if (directory == 1) count = snprintf(relative, sizeof(relative), "../modules/%s/%s_settings.ini", module, application);
    else if (directory == 2) count = snprintf(relative, sizeof(relative), "../share/%s/%s_settings.ini", application, application);
    else if (directory == 3) count = snprintf(relative, sizeof(relative), "modules/%s/%s_settings.ini", module, application);
    else count = snprintf(relative, sizeof(relative), "share/%s/%s_settings.ini", application, application);
    if (count < 0 || (size_t)count >= sizeof(relative) ||
        !inference_resolve_path(executable, relative, filename, capacity)) return false;
    struct stat status;
    if (!stat(filename, &status)) return S_ISREG(status.st_mode);
  }
  return false;
}