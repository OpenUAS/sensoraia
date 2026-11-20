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

/** @file onnx_session.c
 * @brief Reusable numeric tensor sessions without image, acoustic or planning semantics.
 * @details Cancellation is cooperative; outer worker supervision remains the recovery boundary. */
#include "onnx_session.h"
#include "onnx_runtime.h"

#include <onnxruntime_c_api.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/** @brief Consume every ORT error object exactly once while copying its diagnostic before release. */
static bool checked(OrtStatus *status, char *error, size_t capacity)
{
  if (!status) return true;
  const OrtApi *api = onnx_runtime_api();
  snprintf(error, capacity, "%s", api->GetErrorMessage(status));
  api->ReleaseStatus(status);
  return false;
}

uint64_t onnx_monotonic_us(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || now.tv_sec < 0) return 0;
  return (uint64_t)now.tv_sec * UINT64_C(1000000) + (uint64_t)now.tv_nsec / 1000U;
}

bool onnx_tensor_bytes(enum onnx_element_type type, const int64_t *dimensions,
                       size_t rank, size_t *bytes)
{
  if (!bytes || rank > ONNX_RANK_LIMIT || (rank && !dimensions)) return false;
  size_t size;
  switch (type) {
    case ONNX_FLOAT32: case ONNX_INT32: size = 4; break;
    case ONNX_FLOAT64: case ONNX_INT64: size = 8; break;
    case ONNX_UINT8: case ONNX_INT8: size = 1; break;
    default: return false;
  }
  for (size_t axis = 0; axis < rank; ++axis) {
    /* Divide before multiplying so malformed dimensions cannot wrap a valid-looking allocation size. */
    if (dimensions[axis] <= 0 || (uint64_t)dimensions[axis] > ONNX_BUFFER_LIMIT / size) return false;
    size *= (size_t)dimensions[axis];
  }
  *bytes = size;
  return true;
}

/** @brief Compare concrete execution shapes, allowing dynamic graph axes only during initial binding. */
static bool tensor_contract(const OrtTensorTypeAndShapeInfo *info,
                            const struct onnx_tensor *spec, bool dynamic,
                            char *error, size_t capacity)
{
  const OrtApi *api = onnx_runtime_api();
  ONNXTensorElementDataType type;
  size_t rank = 0;
  int64_t dimensions[ONNX_RANK_LIMIT];
  if (!checked(api->GetTensorElementType(info, &type), error, capacity) ||
      !checked(api->GetDimensionsCount(info, &rank), error, capacity)) return false;
  if ((int)type != (int)spec->type || rank != spec->rank || rank > ONNX_RANK_LIMIT) {
    snprintf(error, capacity, "Unexpected model tensor type/rank: %s", spec->name);
    return false;
  }
  if (!checked(api->GetDimensions(info, dimensions, rank), error, capacity)) return false;
  for (size_t axis = 0; axis < rank; ++axis) {
    if (dimensions[axis] != spec->dimensions[axis] && !(dynamic && dimensions[axis] == -1)) {
      snprintf(error, capacity, "Unexpected model tensor dimension: %s", spec->name);
      return false;
    }
  }
  return true;
}

/** @brief Bind by graph name rather than declaration order, which may change during export optimization. */
static bool bind_tensors(struct onnx_session *session, const struct onnx_tensor *specs,
                         size_t count, bool input, char *error, size_t capacity)
{
  const OrtApi *api = onnx_runtime_api();
  OrtAllocator *allocator = NULL;
  size_t actual_count = 0;
  if (!checked(api->GetAllocatorWithDefaultOptions(&allocator), error, capacity) ||
      !checked(input ? api->SessionGetInputCount(session->handle, &actual_count) :
                       api->SessionGetOutputCount(session->handle, &actual_count), error, capacity)) return false;
  if (actual_count != count) {
    snprintf(error, capacity, "Unexpected model input/output count");
    return false;
  }
  bool seen[ONNX_TENSOR_LIMIT] = {false};
  size_t total_bytes = 0;
  for (size_t index = 0; index < count; ++index) {
    char *name = NULL;
    if (!checked(input ? api->SessionGetInputName(session->handle, index, allocator, &name) :
                         api->SessionGetOutputName(session->handle, index, allocator, &name), error, capacity)) return false;
    size_t slot = 0;
    while (slot < count && (!specs[slot].name || strcmp(name, specs[slot].name))) ++slot;
    OrtStatus *released = api->AllocatorFree(allocator, name);
    if (!checked(released, error, capacity)) return false;
    if (slot == count || seen[slot]) {
      snprintf(error, capacity, "Unexpected model %s name", input ? "input" : "output");
      return false;
    }
    seen[slot] = true;
    const struct onnx_tensor *spec = &specs[slot];
    size_t bytes = 0;
    if (!spec->name[0] || strlen(spec->name) >= ONNX_NAME_CAPACITY || !spec->data ||
        !onnx_tensor_bytes(spec->type, spec->dimensions, spec->rank, &bytes) || spec->bytes != bytes ||
        bytes > ONNX_BUFFER_LIMIT - total_bytes) {
      snprintf(error, capacity, "Invalid tensor storage contract");
      return false;
    }
    total_bytes += bytes;
    OrtTypeInfo *type = NULL;
    if (!checked(input ? api->SessionGetInputTypeInfo(session->handle, index, &type) :
                         api->SessionGetOutputTypeInfo(session->handle, index, &type), error, capacity)) return false;
    const OrtTensorTypeAndShapeInfo *info = NULL;
    const bool valid = checked(api->CastTypeInfoToTensorInfo(type, &info), error, capacity) && info &&
                       tensor_contract(info, spec, true, error, capacity);
    api->ReleaseTypeInfo(type);
    if (!valid) {
      if (!error[0]) snprintf(error, capacity, "Expected model tensor: %s", spec->name);
      return false;
    }
    OrtValue *value = NULL;
    if (!checked(api->CreateTensorWithDataAsOrtValue(session->memory, spec->data, bytes,
                   spec->dimensions, spec->rank, (ONNXTensorElementDataType)spec->type, &value), error, capacity)) return false;
    char *destination = input ? session->input_names[slot] : session->output_names[slot];
    for (size_t character = 0; character <= strlen(spec->name); ++character) destination[character] = spec->name[character];
    if (input) session->inputs[slot] = value;
    else {
      session->outputs[slot] = value;
      struct onnx_tensor *output = &session->output_specs[slot];
      output->name = destination;
      output->type = spec->type;
      output->rank = spec->rank;
      output->data = spec->data;
      output->bytes = spec->bytes;
      for (size_t axis = 0; axis < spec->rank; ++axis) output->dimensions[axis] = spec->dimensions[axis];
    }
  }
  return true;
}

/** @brief Request termination without releasing buffers still borrowed by a running graph. */
static void *watch_deadline(void *context)
{
  struct onnx_session *session = context;
  pthread_mutex_lock(&session->mutex);
  while (!session->stopping) {
    if (!session->running || !session->deadline_us || session->timed_out) {
      pthread_cond_wait(&session->condition, &session->mutex);
      continue;
    }
    const struct timespec deadline = {
      .tv_sec = (time_t)(session->deadline_us / UINT64_C(1000000)),
      .tv_nsec = (long)(session->deadline_us % UINT64_C(1000000)) * 1000L
    };
    const int result = pthread_cond_timedwait(&session->condition, &session->mutex, &deadline);
    if (result == ETIMEDOUT && session->running && session->deadline_us &&
        onnx_monotonic_us() >= session->deadline_us) {
      session->timed_out = true;
      OrtStatus *status = onnx_runtime_api()->RunOptionsSetTerminate(session->run_options);
      if (status) onnx_runtime_api()->ReleaseStatus(status);
    }
  }
  pthread_mutex_unlock(&session->mutex);
  return NULL;
}

bool onnx_session_open(struct onnx_session *session, const char *model,
                       const struct onnx_tensor *inputs, size_t input_count,
                       const struct onnx_tensor *outputs, size_t output_count,
                       const struct onnx_session_options *options,
                       char *error, size_t capacity)
{
  if (!session || !error || !capacity) return false;
  error[0] = '\0';
  if (session->handle || session->synchronized || !model || !model[0] || !inputs || !outputs ||
      !input_count || input_count > ONNX_TENSOR_LIMIT || !output_count || output_count > ONNX_TENSOR_LIMIT ||
      (options && (!options->threads || options->threads > 32))) {
    snprintf(error, capacity, "Invalid ONNX session configuration");
    return false;
  }
  if (!onnx_runtime_open(options ? options->library_directory : NULL, error, capacity)) return false;
  const OrtApi *api = onnx_runtime_api();
  OrtSessionOptions *configuration = NULL;
  OrtSession *handle = NULL;
  OrtMemoryInfo *memory = NULL;
  OrtRunOptions *run_options = NULL;
  if (!checked(api->CreateSessionOptions(&configuration), error, capacity)) return false;
  const unsigned threads = options ? options->threads : 1;
  /* Per-model default pools otherwise compete for every core and keep the small board spinning. */
  bool valid = checked(api->SetIntraOpNumThreads(configuration, (int)threads), error, capacity) &&
    checked(api->SetInterOpNumThreads(configuration, 1), error, capacity) &&
    checked(api->SetSessionExecutionMode(configuration, ORT_SEQUENTIAL), error, capacity) &&
    checked(api->SetSessionGraphOptimizationLevel(configuration, ORT_ENABLE_ALL), error, capacity) &&
    checked(api->AddSessionConfigEntry(configuration, "session.intra_op.allow_spinning", "0"), error, capacity) &&
    checked(api->AddSessionConfigEntry(configuration, "session.inter_op.allow_spinning", "0"), error, capacity) &&
    checked(api->CreateSession(onnx_runtime_environment(), model, configuration, &handle), error, capacity);
  api->ReleaseSessionOptions(configuration);
  if (!valid) return false;
  session->handle = handle;
  session->input_count = input_count;
  session->output_count = output_count;
  valid = checked(api->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &memory), error, capacity) &&
          checked(api->CreateRunOptions(&run_options), error, capacity);
  session->memory = memory;
  session->run_options = run_options;
  if (!valid || !bind_tensors(session, inputs, input_count, true, error, capacity) ||
      !bind_tensors(session, outputs, output_count, false, error, capacity)) goto failed;
  if (pthread_mutex_init(&session->mutex, NULL) != 0) goto synchronization_failed;
  pthread_condattr_t attributes;
  if (pthread_condattr_init(&attributes) != 0) {
    pthread_mutex_destroy(&session->mutex);
    goto synchronization_failed;
  }
  /* The waiter and public deadlines must share a clock unaffected by wall-clock corrections. */
  const int configured = pthread_condattr_setclock(&attributes, CLOCK_MONOTONIC);
  const int initialized = configured ? configured : pthread_cond_init(&session->condition, &attributes);
  pthread_condattr_destroy(&attributes);
  if (initialized) {
    pthread_mutex_destroy(&session->mutex);
    goto synchronization_failed;
  }
  session->synchronized = true;
  return true;
synchronization_failed:
  snprintf(error, capacity, "Cannot initialize ONNX session synchronization");
failed:
  onnx_session_close(session);
  return false;
}

bool onnx_session_run(struct onnx_session *session, uint64_t deadline_us,
                      char *error, size_t capacity)
{
  if (!session || !error || !capacity) return false;
  error[0] = '\0';
  if (!session->handle || !session->synchronized) {
    snprintf(error, capacity, "ONNX session is not open");
    return false;
  }
  const OrtApi *api = onnx_runtime_api();
  pthread_mutex_lock(&session->mutex);
  if (session->running || session->stopping || (deadline_us && onnx_monotonic_us() >= deadline_us)) {
    pthread_mutex_unlock(&session->mutex);
    snprintf(error, capacity, "ONNX session busy, stopped or deadline expired");
    return false;
  }
  if (deadline_us && !session->watched) {
    /* Lazily create one monitor only when this session first needs a deadline. */
    const int started = pthread_create(&session->watchdog, NULL, watch_deadline, session);
    if (started) {
      pthread_mutex_unlock(&session->mutex);
      snprintf(error, capacity, "Cannot start ONNX deadline monitor: %s", strerror(started));
      return false;
    }
    session->watched = true;
  }
  if (!checked(api->RunOptionsUnsetTerminate(session->run_options), error, capacity)) {
    pthread_mutex_unlock(&session->mutex);
    return false;
  }
  session->timed_out = false;
  session->cancelled = false;
  session->deadline_us = deadline_us;
  session->running = true;
  pthread_cond_signal(&session->condition);
  pthread_mutex_unlock(&session->mutex);
  const char *input_names[ONNX_TENSOR_LIMIT], *output_names[ONNX_TENSOR_LIMIT];
  const OrtValue *inputs[ONNX_TENSOR_LIMIT];
  OrtValue *outputs[ONNX_TENSOR_LIMIT];
  for (size_t index = 0; index < session->input_count; ++index) {
    input_names[index] = session->input_names[index];
    inputs[index] = session->inputs[index];
  }
  for (size_t index = 0; index < session->output_count; ++index) {
    output_names[index] = session->output_names[index];
    outputs[index] = session->outputs[index];
  }
  bool valid = checked(api->Run(session->handle, session->run_options, input_names, inputs,
                                session->input_count, output_names, session->output_count, outputs), error, capacity);
  /* Initial graph metadata alone cannot qualify outputs whose graph axes are dynamic. */
  for (size_t index = 0; valid && index < session->output_count; ++index) {
    OrtTensorTypeAndShapeInfo *info = NULL;
    if (!checked(api->GetTensorTypeAndShape(outputs[index], &info), error, capacity)) valid = false;
    else {
      valid = tensor_contract(info, &session->output_specs[index], false, error, capacity);
      api->ReleaseTensorTypeAndShapeInfo(info);
    }
  }
  pthread_mutex_lock(&session->mutex);
  if (session->timed_out || (deadline_us && onnx_monotonic_us() >= deadline_us)) {
    snprintf(error, capacity, "ONNX inference deadline expired");
    valid = false;
  }
  if (session->cancelled || session->stopping) {
    snprintf(error, capacity, "ONNX inference cancelled");
    valid = false;
  }
  session->running = false;
  session->deadline_us = 0;
  pthread_cond_broadcast(&session->condition);
  pthread_mutex_unlock(&session->mutex);
  return valid;
}

void onnx_session_cancel(struct onnx_session *session)
{
  if (!session || !session->synchronized) return;
  pthread_mutex_lock(&session->mutex);
  if (session->running) {
    session->cancelled = true;
    OrtStatus *status = onnx_runtime_api()->RunOptionsSetTerminate(session->run_options);
    if (status) onnx_runtime_api()->ReleaseStatus(status);
  }
  pthread_mutex_unlock(&session->mutex);
}

void onnx_session_close(struct onnx_session *session)
{
  if (!session) return;
  if (session->synchronized) {
    pthread_mutex_lock(&session->mutex);
    session->stopping = true;
    if (session->running) {
      session->cancelled = true;
      OrtStatus *status = onnx_runtime_api()->RunOptionsSetTerminate(session->run_options);
      if (status) onnx_runtime_api()->ReleaseStatus(status);
      /* Termination is not completion; unmapping caller buffers before Run returns would race ORT. */
      while (session->running) pthread_cond_wait(&session->condition, &session->mutex);
    }
    pthread_cond_broadcast(&session->condition);
    pthread_mutex_unlock(&session->mutex);
    if (session->watched) pthread_join(session->watchdog, NULL);
    pthread_cond_destroy(&session->condition);
    pthread_mutex_destroy(&session->mutex);
  }
  const OrtApi *api = onnx_runtime_api();
  if (api) {
    for (size_t index = 0; index < ONNX_TENSOR_LIMIT; ++index) {
      if (session->inputs[index]) api->ReleaseValue(session->inputs[index]);
      if (session->outputs[index]) api->ReleaseValue(session->outputs[index]);
    }
    if (session->run_options) api->ReleaseRunOptions(session->run_options);
    if (session->memory) api->ReleaseMemoryInfo(session->memory);
    if (session->handle) api->ReleaseSession(session->handle);
  }
  *session = (struct onnx_session){0};
}