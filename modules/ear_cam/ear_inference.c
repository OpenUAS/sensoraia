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

/** @file ear_inference.c
 * @brief One-window acoustic inference worker with source-time and measured-level evidence.
 * @details The capture thread never shares its live sample buffer with ORT; stale generations are discarded. */
#include "ear_inference.h"
#include "../../src/inference_model.h"

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#define EAR_INFERENCE_SAMPLE_LIMIT 192000U

/* One capture producer owns collected samples; the mutex protects queued/completed mailboxes. */
static struct inference_context network;
static pthread_mutex_t inference_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t inference_available = PTHREAD_COND_INITIALIZER;
static pthread_t inference_thread;
static bool started, stopping, pending, ready;
static int16_t collected[EAR_INFERENCE_SAMPLE_LIMIT], queued[EAR_INFERENCE_SAMPLE_LIMIT];
static size_t collected_count;
static bool collected_timing_valid;
static uint64_t collected_error_us;
static atomic_uint_fast64_t generation;
static uint64_t queued_generation, completed_generation;
static struct ear_inference_result queued_evidence, completed;
static double score_threshold = 0.6;
static unsigned class_index, option_seen;

/** @brief Keep class/threshold selection module-owned rather than assigning acoustic semantics to the common parser. */
static bool model_setting(void *unused, const char *key, const char *value)
{
  (void)unused;
  if (!strcmp(key, "score_threshold")) {
    if (option_seen & 1U) return false;
    option_seen |= 1U;
    return ini_real(value, &score_threshold) && score_threshold >= 0 && score_threshold <= 1;
  }
  if (!strcmp(key, "class_index")) {
    if (option_seen & 2U) return false;
    option_seen |= 2U;
    uint64_t category;
    if (!ini_uint(value, &category) || category > 4095) return false;
    class_index = (unsigned)category;
    return true;
  }
  return false;
}

bool ear_inference_configure(const char *settings_path, unsigned rate, bool *enabled,
                              char *error, size_t capacity)
{
  if (!enabled || !error || !capacity || started || network.storage) return false;
  *enabled = false;
  option_seen = 0;
  score_threshold = 0.6;
  class_index = 0;
  struct inference_settings settings;
  inference_settings_init(&settings, INFERENCE_NATIVE);
  if (!inference_settings_read(&settings, settings_path, model_setting, NULL, error, capacity)) return false;
  if (settings.backend == INFERENCE_NATIVE) return true;
  if (settings.backend != INFERENCE_ONNX || settings.source != INFERENCE_LIVE ||
      !inference_context_open(&network, &settings, "ear_pcm_classifier_v1", error, capacity)) return false;
  const struct inference_model *model = &network.model;
  const bool valid = model->input_count == 1 && model->output_count == 1 &&
    model->audio_rate == rate && model->audio_samples && model->audio_samples <= rate &&
    model->audio_samples <= EAR_INFERENCE_SAMPLE_LIMIT &&
    model->inputs[0].type == ONNX_FLOAT32 && model->inputs[0].bytes == (size_t)model->audio_samples * sizeof(float) &&
    model->outputs[0].type == ONNX_FLOAT32 && model->outputs[0].bytes / sizeof(float) <= 4096 &&
    class_index < model->outputs[0].bytes / sizeof(float) && !model->image.channels;
  if (!valid || !inference_context_run(&network, error, capacity)) {
    if (!error[0]) snprintf(error, capacity, "EAR classifier requires normalized mono PCM, matching sample rate and float probabilities");
    inference_context_close(&network);
    return false;
  }
  *enabled = true;
  return true;
}

/** @brief Accept finite probabilities without confusing them with measured loudness or localization confidence. */
static bool run_window(struct ear_inference_result *result)
{
  char error[1024];
  if (!inference_context_run(&network, error, sizeof(error))) {
    fprintf(stderr, "EARCAM_ERROR inference %s\n", error);
    return false;
  }
  const float *scores = network.model.outputs[0].data;
  const size_t count = network.model.outputs[0].bytes / sizeof(float);
  for (size_t index = 0; index < count; ++index) {
    if (!isfinite(scores[index]) || scores[index] < 0 || scores[index] > 1) {
      fprintf(stderr, "EARCAM_ERROR inference invalid classifier probability\n");
      return false;
    }
  }
  result->score = scores[class_index];
  result->category = class_index;
  result->alarm = result->score >= score_threshold;
  return true;
}

/** @brief Own the model session off the ALSA thread and publish only still-current capture generations. */
static void *classify_windows(void *unused)
{
  (void)unused;
  pthread_mutex_lock(&inference_mutex);
  while (!stopping) {
    while (!pending && !stopping) pthread_cond_wait(&inference_available, &inference_mutex);
    if (stopping) break;
    if (queued_generation != atomic_load(&generation)) { pending = false; continue; }
    struct ear_inference_result result = queued_evidence;
    const uint64_t identity = queued_generation;
    float *input = network.model.inputs[0].data;
    double energy = 0;
    for (size_t index = 0; index < network.model.audio_samples; ++index) {
      input[index] = queued[index] / 32768.0f;
      energy += (double)input[index] * input[index];
    }
    /* Derive level from the same PCM window, never from classifier confidence. */
    result.level_dbfs = energy > 0 ? 10 * log10(energy / network.model.audio_samples) : -120;
    pending = false;
    /* ORT may be slow; only the private input tensor remains borrowed during graph execution. */
    pthread_mutex_unlock(&inference_mutex);
    const bool valid = run_window(&result);
    pthread_mutex_lock(&inference_mutex);
    if (valid && !stopping && identity == atomic_load(&generation)) {
      completed = result;
      completed_generation = identity;
      ready = true;
    }
  }
  pthread_mutex_unlock(&inference_mutex);
  return NULL;
}

bool ear_inference_start(char *error, size_t capacity)
{
  if (!network.storage || started) return false;
  stopping = pending = ready = false;
  collected_count = 0;
  const int result = pthread_create(&inference_thread, NULL, classify_windows, NULL);
  if (result) { snprintf(error, capacity, "Cannot start acoustic inference worker: %s", strerror(result)); return false; }
  started = true;
  return true;
}

void ear_inference_push(const int16_t *samples, size_t count, uint64_t end_us, uint64_t error_us)
{
  if (!started || !samples || !network.model.audio_rate) return;
  for (size_t index = 0; index < count; ++index) {
    if (!collected_count) { collected_timing_valid = true; collected_error_us = 0; }
    collected_timing_valid = collected_timing_valid && end_us != 0;
    if (error_us > collected_error_us) collected_error_us = error_us;
    collected[collected_count++] = samples[index];
    if (collected_count != network.model.audio_samples) continue;
    /* Dropping an overloaded window is safer than delaying audio reads and joining samples across an overrun. */
    if (!pthread_mutex_trylock(&inference_mutex)) {
      queued_evidence = (struct ear_inference_result){.error_us = collected_error_us};
      const uint64_t tail = (uint64_t)(count - index - 1) * UINT64_C(1000000) / network.model.audio_rate;
      const uint64_t duration = (uint64_t)network.model.audio_samples * UINT64_C(1000000) / network.model.audio_rate;
      if (collected_timing_valid && end_us > tail && end_us - tail > duration) {
        queued_evidence.end_us = end_us - tail;
        queued_evidence.begin_us = queued_evidence.end_us - duration;
      }
      static uint64_t sequence;
      queued_evidence.sequence = ++sequence;
      for (size_t sample = 0; sample < collected_count; ++sample) {
        queued[sample] = collected[sample];
        if (collected[sample] >= 32760 || collected[sample] <= -32760) queued_evidence.clipped = true;
      }
      queued_generation = atomic_load(&generation);
      pending = true;
      pthread_cond_signal(&inference_available);
      pthread_mutex_unlock(&inference_mutex);
    }
    collected_count = 0;
  }
}

bool ear_inference_take(struct ear_inference_result *result)
{
  if (!result || !started || pthread_mutex_trylock(&inference_mutex)) return false;
  const bool valid = ready && completed_generation == atomic_load(&generation);
  if (valid) *result = completed;
  ready = false;
  pthread_mutex_unlock(&inference_mutex);
  return valid;
}

void ear_inference_discontinuity(void)
{
  collected_count = 0;
  /* A running graph cannot be recalled immediately, so its generation must become unpublishable. */
  atomic_fetch_add(&generation, 1);
  if (pthread_mutex_trylock(&inference_mutex)) return;
  pending = ready = false;
  pthread_mutex_unlock(&inference_mutex);
}

void ear_inference_stop(void)
{
  if (started) {
    pthread_mutex_lock(&inference_mutex);
    stopping = true;
    pthread_cond_signal(&inference_available);
    pthread_mutex_unlock(&inference_mutex);
    onnx_session_cancel(&network.session);
    pthread_join(inference_thread, NULL);
    started = false;
  }
  inference_context_close(&network);
}

int ear_inference_file(const char *filename)
{
  if (!network.storage || !filename) return 1;
  struct stat info;
  if (stat(filename, &info) || !S_ISREG(info.st_mode) || info.st_size <= 0 ||
      (uint64_t)info.st_size % ((uint64_t)network.model.audio_samples * 2)) return 1;
  FILE *input = fopen(filename, "rb");
  if (!input) return 1;
  uint64_t sequence = 0;
  int result = 0;
  while (!result) {
    float *samples = network.model.inputs[0].data;
    double energy = 0;
    for (unsigned index = 0; index < network.model.audio_samples; ++index) {
      unsigned char bytes[2];
      const size_t read_count = fread(bytes, 1, 2, input);
      if (!read_count && !index && feof(input)) goto finished;
      if (read_count != 2) { result = 1; break; }
      const uint16_t raw = (uint16_t)bytes[0] | (uint16_t)bytes[1] << 8;
      const int32_t signed_sample = raw >= 32768 ? (int32_t)raw - 65536 : raw;
      samples[index] = signed_sample / 32768.0f;
      energy += (double)samples[index] * samples[index];
    }
    if (result) break;
    struct ear_inference_result classified = {.sequence = ++sequence,
      .level_dbfs = energy > 0 ? 10 * log10(energy / network.model.audio_samples) : -120};
    if (!run_window(&classified)) { result = 1; break; }
    printf("CLASS sequence=%llu level=%.6f score=%.6f class=%u alarm=%u\n",
      (unsigned long long)classified.sequence, classified.level_dbfs, classified.score,
      classified.category, classified.alarm ? 1U : 0U);
  }
finished:
  if (ferror(input)) result = 1;
  if (fclose(input)) result = 1;
  return result;
}