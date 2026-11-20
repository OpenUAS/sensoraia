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

/** @file onnx_runtime.c
 * @brief Local-only dynamic loading and process-lifetime ONNX environment ownership.
 * @details Keeping one retained API avoids incompatible per-module loaders and dangling session callbacks. */
#include "onnx_runtime.h"

#include <onnxruntime_c_api.h>
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static pthread_mutex_t runtime_mutex = PTHREAD_MUTEX_INITIALIZER;
static const OrtApi *runtime_api;
static OrtEnv *runtime_environment;
static void *runtime_library;

bool onnx_runtime_open(const char *library_directory, char *error, size_t capacity)
{
  if (!error || !capacity) return false;
  error[0] = '\0';
  const int locked = pthread_mutex_lock(&runtime_mutex);
  if (locked) {
    snprintf(error, capacity, "Cannot lock ONNX runtime: %s", strerror(locked));
    return false;
  }
  if (runtime_api) {
    pthread_mutex_unlock(&runtime_mutex);
    return true;
  }
  char executable[PATH_MAX];
  const ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length <= 0 || (size_t)length >= sizeof(executable) - 1) {
    snprintf(error, capacity, "Cannot resolve executable for ONNX runtime");
    pthread_mutex_unlock(&runtime_mutex);
    return false;
  }
  executable[length] = '\0';
  char *separator = strrchr(executable, '/');
  if (!separator) {
    snprintf(error, capacity, "Invalid executable path for ONNX runtime");
    pthread_mutex_unlock(&runtime_mutex);
    return false;
  }
  *separator = '\0';
  char parent[PATH_MAX];
  snprintf(parent, sizeof(parent), "%s", executable);
  for (unsigned level = 0; level < 2; ++level) {
    char *slash = strrchr(parent, '/');
    if (!slash || slash == parent) {
      parent[0] = '/';
      parent[1] = '\0';
      break;
    }
    *slash = '\0';
  }
  char secondary[PATH_MAX];
  const int secondary_length = snprintf(secondary, sizeof(secondary), "%s%sbin", parent,
                                         parent[strlen(parent) - 1] == '/' ? "" : "/");
  if (secondary_length < 0 || (size_t)secondary_length >= sizeof(secondary)) {
    snprintf(error, capacity, "ONNX runtime directory exceeds path capacity");
    pthread_mutex_unlock(&runtime_mutex);
    return false;
  }
  const char *directories[] = {executable, secondary, library_directory};
  const char *names[] = {
#if defined(__aarch64__)
    /* Architecture suffixes let ARM64 and host libraries coexist in the development bin directory. */
    "libonnxruntime.so-arm64", "libonnxruntime.so.1-arm64", "libonnxruntime.so.1.31.0-arm64",
#endif
    "libonnxruntime.so", "libonnxruntime.so.1", "libonnxruntime.so.1.31.0"
  };
  bool opened = false;
  for (size_t directory = 0; directory < sizeof(directories) / sizeof(directories[0]); ++directory) {
    if (!directories[directory] || !directories[directory][0]) continue;
    for (size_t name = 0; name < sizeof(names) / sizeof(names[0]); ++name) {
      char filename[PATH_MAX];
      const int count = snprintf(filename, sizeof(filename), "%s/%s", directories[directory], names[name]);
      if (count < 0 || (size_t)count >= sizeof(filename)) {
        snprintf(error, capacity, "ONNX runtime filename exceeds path capacity");
        goto finished;
      }
      struct stat info;
      if (stat(filename, &info) != 0) {
        if (errno == ENOENT || errno == ENOTDIR) continue;
        snprintf(error, capacity, "Cannot inspect ONNX Runtime %s: %s", filename, strerror(errno));
        goto finished;
      }
      if (!S_ISREG(info.st_mode)) {
        snprintf(error, capacity, "ONNX Runtime is not a regular file: %s", filename);
        goto finished;
      }
      void *library = dlopen(filename, RTLD_NOW | RTLD_LOCAL);
      /* An existing broken candidate is a package error, not permission to try a different runtime. */
      if (!library) {
        snprintf(error, capacity, "Cannot load ONNX Runtime %s: %s", filename, dlerror());
        goto finished;
      }
      union {
        void *object;
        const OrtApiBase *(*function)(void);
      } symbol = {.object = dlsym(library, "OrtGetApiBase")};
      /* Resolve through the C API so C and C++ consumers share the same ABI check. */
      const OrtApiBase *base = symbol.function ? symbol.function() : NULL;
      const OrtApi *api = base ? base->GetApi(ORT_API_VERSION) : NULL;
      if (!api) {
        snprintf(error, capacity, "Incompatible ONNX Runtime API in %s", filename);
        dlclose(library);
        goto finished;
      }
      OrtEnv *environment = NULL;
      OrtStatus *status = api->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "sensoraia", &environment);
      if (status) {
        snprintf(error, capacity, "Cannot initialize ONNX Runtime: %s", api->GetErrorMessage(status));
        api->ReleaseStatus(status);
        dlclose(library);
        goto finished;
      }
      /* Retain this handle: environments and tensor objects may call the API until process exit. */
      runtime_library = library;
      runtime_environment = environment;
      runtime_api = api;
      opened = true;
      goto finished;
    }
  }
  snprintf(error, capacity,
           "ONNX Runtime not found in searched directories: %s, %s, %s. "
           "Tried libonnxruntime.so, libonnxruntime.so.1, libonnxruntime.so.1.31.0%s. "
           "Copy a compatible runtime beside the executable or set onnx_lib_path in diplacam_settings.ini.",
           executable, secondary, library_directory ? library_directory : "(not configured)",
#if defined(__aarch64__)
           " and their -arm64 variants"
#else
           ""
#endif
  );
finished:
  if (!opened) {
    const size_t used = strlen(error);
    if (used < capacity)
      snprintf(error + used, capacity - used, ". Copy a compatible %s into %s or set onnx_lib_path in diplacam_settings.ini.",
               names[0], executable);
  }
  pthread_mutex_unlock(&runtime_mutex);
  return opened;
}

const OrtApi *onnx_runtime_api(void)
{
  pthread_mutex_lock(&runtime_mutex);
  const OrtApi *api = runtime_library ? runtime_api : NULL;
  pthread_mutex_unlock(&runtime_mutex);
  return api;
}

OrtEnv *onnx_runtime_environment(void)
{
  pthread_mutex_lock(&runtime_mutex);
  OrtEnv *environment = runtime_library ? runtime_environment : NULL;
  pthread_mutex_unlock(&runtime_mutex);
  return environment;
}