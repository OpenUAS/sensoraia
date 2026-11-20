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

/** @file inference_settings.h
 * @brief Shared application_settings.ini discovery with module-owned extra keys. */
#ifndef SENSORAIA_INFERENCE_SETTINGS_H
#define SENSORAIA_INFERENCE_SETTINGS_H

#include "bounded_ini.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INFERENCE_PATH_CAPACITY 4096U

/** @brief Backend selection; AUTO resolves the model suffix, never an execution error. */
enum inference_backend { INFERENCE_NATIVE, INFERENCE_ONNX, INFERENCE_IMX500, INFERENCE_AUTO };
/** @brief Acquisition choice, deliberately independent of the inference backend. */
enum inference_source { INFERENCE_LIVE, INFERENCE_IMAGE, INFERENCE_FAKE };

/** @brief Per-application configuration; all resolved paths belong to this caller-owned value. */
struct inference_settings {
  enum inference_backend backend;
  enum inference_source source;
  char model[INFERENCE_PATH_CAPACITY];
  char descriptor[INFERENCE_PATH_CAPACITY];
  char library_directory[INFERENCE_PATH_CAPACITY];
  char input[INFERENCE_PATH_CAPACITY];
  unsigned threads, timeout_ms;
  uint32_t seen;
};

/** @brief Reset storage and select conservative thread/timeout defaults.
 * @param settings Writable owner, or NULL to do nothing.
 * @param backend Application default used when the INI omits the backend. */
void inference_settings_init(struct inference_settings *settings,
                              enum inference_backend backend);
/** @brief Parse one common key without consuming module-specific keys.
 * @param settings Initialized destination; its seen mask rejects duplicates.
 * @param key Flat key emitted by the bounded INI parser.
 * @param value Nonempty parsed value; no pointer into it is retained.
 * @return One if accepted, zero if unknown, minus one if invalid/duplicate. */
int inference_settings_field(struct inference_settings *settings,
                              const char *key, const char *value);
/** @brief Parse one flat INI, delegate application keys and resolve paths against the selected file.
 * @param settings Initialized destination; discard partially parsed values on failure.
 * @param filename Selected settings file, independent of the caller's current directory.
 * @param extra Optional handler for application-specific fields.
 * @param context Borrowed extra-handler state, used only during this call.
 * @param error Writable failure diagnostic, cleared on success.
 * @param capacity Available bytes in error.
 * @return True for valid bounded syntax, accepted fields and resolvable lexical paths.
 * @details AUTO resolves .onnx/.rpk suffixes and an omitted ONNX descriptor to
 * the same-name .ini. It does not test availability or fall back after errors. */
bool inference_settings_read(struct inference_settings *settings, const char *filename,
                              ini_field_handler extra, void *context,
                              char *error, size_t capacity);
/** @brief Discover application_settings.ini in adjacent, development-module, then installed-share order.
 * @details Supports both bin/ application executables and the root-level USHER
 * owner, without consulting the caller's current directory.
 * @param application Lowercase executable name without an architecture suffix.
 * @param module Lowercase module directory name, such as spotter_cam.
 * @param filename Destination path; meaningful only on success.
 * @param capacity Available bytes in filename.
 * @return True after finding a regular file, never a current-directory fallback. */
bool inference_settings_find(const char *application, const char *module,
                              char *filename, size_t capacity);
/** @brief Resolve an INI-relative path without requiring its destination to exist yet.
 * @param base_file Settings filename whose parent supplies the relative base.
 * @param value Absolute or relative path; control characters are rejected.
 * @param destination Writable normalized absolute path, distinct from the inputs.
 * @param capacity Destination capacity including its terminator.
 * @return True for a complete, nontruncated path. This is lexical normalization, not symlink resolution. */
bool inference_resolve_path(const char *base_file, const char *value,
                            char *destination, size_t capacity);

#ifdef __cplusplus
}
#endif
#endif