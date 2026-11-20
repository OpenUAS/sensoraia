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

#define _POSIX_C_SOURCE 200809L
/**
 * @file capture_arbiter.c
 * @brief Bounded camera-independent capture admission, coverage and timing evidence.
 * @details The serial owner offers intentions; camera workers report acquired and
 * published images. Public state access uses guard, and internal state helpers
 * require that lock. No camera callback runs under it. File parsing happens before
 * taking the lock so an invalid profile or sidecar cannot retain camera admission.
 * An intention is not an image: only verified publication may credit coverage.
 */
#include "capture_arbiter.h"
#include "boot_id.h"
#include "camera_profile.h"
#include "motion_compensation.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "exifite.h"
#include "sensor_capture.h"

#define ARBITER_COUNT(array) (sizeof(array) / sizeof((array)[0]))

/** @brief One camera's intent; acquired means pixels are owned, not yet durable. */
struct capture_intent {
  union dc_shot_union shot;
  uint32_t mask;
  uint64_t received_us;
  bool acquired;
};

/** @brief FC pose and bounded local clock mapping, rather than a precise exposure claim. */
struct capture_pose {
  union dc_shot_union shot;
  uint64_t time_us, error_us;
  double accuracy_m;
  uint32_t navigation_age_ms;
  bool navigation_projected;
};

static pthread_mutex_t guard = PTHREAD_MUTEX_INITIALIZER;
/* Keep the 3 MiB ledger in zero-initialized static storage, off worker stacks. */
static struct coverage_state coverage_ledger;
static struct coverage_state *covered;
static struct coverage_camera cameras[COVERAGE_CAMERAS];
static bool calibrated[COVERAGE_CAMERAS];
/* Fixed camera slices stop a fast sensor from evicting another sensor's pending view. */
static struct capture_intent waiting[COVERAGE_CANDIDATES * COVERAGE_CAMERAS];
static struct capture_intent active[2 * COVERAGE_CAMERAS];
static unsigned capacity[COVERAGE_CAMERAS] = {1, 1, 1};
static union usher_capture_arbiter_result_union results[64];
static size_t result_head, result_count;
static uint32_t session, scope_mask, saved_count;
static uint64_t scope_us;
static bool scope_active, enforce, origin_set;
static double latitude, longitude;
static uint64_t service_us[COVERAGE_CAMERAS] = {1000000, 1000000, 1000000};
static struct coverage_candidate last_saved[COVERAGE_CAMERAS];
static bool have_saved[COVERAGE_CAMERAS];
static double minimum_overlap = 0.3;
static const uint64_t scope_lifetime_us = 750000;
static const uint64_t intent_lifetime_us = 2000000;
static struct capture_pose poses[512];
static size_t pose_head, pose_count;
static uint32_t gps_week, gps_tow_ms;
static uint64_t gps_changed_us;

/**
 * @brief Read the local clock used for capture deadlines and pose alignment.
 * @return Monotonic microseconds, or zero when the clock read fails.
 * @details Use the same time base for age and ordering checks so wall-clock
 * corrections do not make old evidence appear fresh.
 */
static uint64_t monotonic_us(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now)) return 0;
  return (uint64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

/**
 * @brief Obtain a nonzero token for a new survey session.
 * @param[out] token Destination; use only on success.
 * @return True for a complete, nonzero token and successful descriptor close.
 * @details Accumulate short reads and retry interruptions instead of accepting
 * a partial token. Zero is reserved for inactive session state.
 */
static bool random_session(uint32_t *token)
{
  const int descriptor = open("/dev/urandom", O_RDONLY | O_CLOEXEC);
  if (descriptor < 0) return false;
  size_t received = 0;
  while (received < sizeof(*token)) {
    const ssize_t count = read(descriptor, (unsigned char *)token + received, sizeof(*token) - received);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) break;
    received += (size_t)count;
  }
  const bool valid = close(descriptor) == 0 && received == sizeof(*token) && *token != 0;
  return valid;
}

/** @brief Record a terminal outcome without allowing telemetry backlog to block camera release. */
static void report(const struct capture_intent *intent, unsigned camera_id, uint32_t outcome)
{
  fprintf(stderr, "USHER SURVEY: session=%u shot=%d camera=%u result=%u saved=%u\n",
          session, intent->shot.data.nr, camera_id, outcome, saved_count);
  if (result_count == ARBITER_COUNT(results)) {
    fprintf(stderr, "USHER SURVEY: result telemetry full; capture continues\n");
    return;
  }
  union usher_capture_arbiter_result_union *result = &results[(result_head + result_count++) % ARBITER_COUNT(results)];
  *result = (union usher_capture_arbiter_result_union) {0};
  result->data.session = session;
  result->data.number = intent->shot.data.nr;
  result->data.camera_id = camera_id;
  result->data.result = outcome;
  result->data.saved_count = saved_count;
}

/**
 * @brief Retire an intention and record its terminal outcome.
 * @param[in,out] intent Owned entry to clear.
 * @param outcome Result code to report to each camera in the intent mask.
 * @details Report per camera before clearing the slot. The caller holds the
 * arbiter lock, and result queue overflow must not prevent retirement.
 */
static void expire(struct capture_intent *intent, uint32_t outcome)
{
  for (unsigned camera_id = 1; camera_id <= COVERAGE_CAMERAS; ++camera_id)
    if (intent->mask & (1U << (camera_id - 1))) report(intent, camera_id, outcome);
  *intent = (struct capture_intent) {0};
}

/**
 * @brief Check whether the active survey scope is still current.
 * @param now_us Current local monotonic time in microseconds.
 * @return True for an active session with in-window scope evidence.
 * @details Check ordering before subtracting unsigned times; a timestamp
 * later than the query is not evidence that the scope is fresh.
 */
static bool fresh_scope(uint64_t now_us)
{
  return session && scope_active && now_us >= scope_us && now_us - scope_us <= scope_lifetime_us;
}

/** @brief Convert FC units and estimate geometric detail using this camera's calibration. */
static bool project(const union dc_shot_union *shot, unsigned camera_id, double uncertainty,
                    struct coverage_candidate *candidate)
{
  if (!origin_set || camera_id < 1 || camera_id > COVERAGE_CAMERAS || !calibrated[camera_id - 1]) return false;
  const struct coverage_pose pose = {
    .latitude_deg = shot->data.lat / 1e7, .longitude_deg = shot->data.lon / 1e7,
    .agl_m = shot->data.groundalt / 256.0,
    .roll_deg = shot->data.phi * (57.29577951308232 / 4096.0),
    .pitch_deg = shot->data.theta * (57.29577951308232 / 4096.0),
    .yaw_deg = shot->data.psi * (57.29577951308232 / 4096.0), .uncertainty_m = uncertainty
  };
  *candidate = (struct coverage_candidate) {.camera_id = camera_id};
  const struct coverage_camera *camera = &cameras[camera_id - 1];
  if (!coverage_project(camera, &pose, latitude, longitude, &candidate->footprint)) return false;
  double twice_area = 0;
  for (size_t index = 0; index < 4; ++index) {
    const struct coverage_point *first = &candidate->footprint.corner[index];
    const struct coverage_point *second = &candidate->footprint.corner[(index + 1) % 4];
    twice_area += first->east * second->north - first->north * second->east;
  }
  const double metres_per_pixel = sqrt(fabs(twice_area) / (2.0 * camera->width * camera->height));
  if (!isfinite(metres_per_pixel) || metres_per_pixel <= 0) return false;
  candidate->quality = (uint8_t)fmax(1.0, fmin(255.0, floor(1 / metres_per_pixel)));
  return true;
}

/** @brief Take the short arc across heading wrap, using widened subtraction before conversion. */
static double angle_delta(int32_t first, int32_t second)
{
  return remainder(((int64_t)second - first) / 4096.0, 6.283185307179586);
}

/** @brief Forecast a short continuation only when mapped motion is suitably well behaved. */
static bool predict(uint64_t time_us, union dc_shot_union *shot, double *uncertainty)
{
  if (pose_count < 2) return false;
  const struct capture_pose *latest = &poses[(pose_head + pose_count - 1) % ARBITER_COUNT(poses)];
  const struct capture_pose *previous = &poses[(pose_head + pose_count - 2) % ARBITER_COUNT(poses)];
  /* Do not extend an old straight segment through a turn or missing telemetry. */
  if (time_us < latest->time_us || time_us - latest->time_us > 500000
      || latest->time_us <= previous->time_us || latest->time_us - previous->time_us > 200000) return false;
  const double elapsed = (time_us - latest->time_us) / 1e6;
  const double sample_time = (latest->time_us - previous->time_us) / 1e6;
  const double roll_rate = angle_delta(previous->shot.data.phi, latest->shot.data.phi) / sample_time;
  const double pitch_rate = angle_delta(previous->shot.data.theta, latest->shot.data.theta) / sample_time;
  const double yaw_rate = angle_delta(previous->shot.data.psi, latest->shot.data.psi) / sample_time;
  if (fabs(roll_rate) > 0.15 || fabs(pitch_rate) > 0.15 || fabs(yaw_rate) > 0.25
      || fabs(latest->shot.data.phi / 4096.0) > 0.6 || fabs(latest->shot.data.theta / 4096.0) > 0.6) return false;
  *shot = latest->shot;
  const double radians = 0.017453292519943295;
  const double speed = shot->data.vground / 524288.0, course = shot->data.course / 4096.0;
  const double predicted_latitude = shot->data.lat / 1e7 + speed * elapsed * cos(course) / 6378137 / radians;
  const double predicted_longitude = shot->data.lon / 1e7
                                     + speed * elapsed * sin(course) / (6378137 * cos(shot->data.lat / 1e7 * radians)) / radians;
  if (fabs(predicted_latitude) > 85 || fabs(predicted_longitude) > 180) return false;
  shot->data.lat = (int32_t)llround(predicted_latitude * 1e7);
  shot->data.lon = (int32_t)llround(predicted_longitude * 1e7);
  shot->data.phi += (int32_t)llround(roll_rate * elapsed * 4096);
  shot->data.theta += (int32_t)llround(pitch_rate * elapsed * 4096);
  shot->data.psi = (int32_t)llround(remainder(shot->data.psi / 4096.0 + yaw_rate * elapsed, 6.283185307179586) * 4096);
  *uncertainty = latest->accuracy_m + speed * (latest->error_us / 1e6 + elapsed * 0.25)
                 + 5 * elapsed * elapsed + shot->data.groundalt / 256.0 * (0.02 + elapsed * 0.1);
  return isfinite(*uncertainty) && *uncertainty <= 15;
}

bool capture_arbiter_load_camera(const char *filename)
{
  struct usher_camera_profile profile;
  if (!usher_camera_profile_read(filename, &profile)) {
    fprintf(stderr, "USHER SURVEY: camera profile rejected: invalid bounded flat profile\n");
    return false;
  }
  if (profile.camera_id < 1 || profile.camera_id > COVERAGE_CAMERAS) {
    fprintf(stderr, "USHER SURVEY: camera profile rejected: prefix must be c, a or l\n");
    return false;
  }
  const struct coverage_camera *camera = &profile.calibration;
  const struct coverage_pose pose = {.agl_m = 10};
  struct coverage_footprint footprint;
  if (!coverage_project(camera, &pose, 0, 0, &footprint)) {
    fprintf(stderr, "USHER SURVEY: camera profile rejected: invalid calibration\n");
    return false;
  }

  bool accepted = false;
  const char *error = "coverage initialization";
  pthread_mutex_lock(&guard);
  if (!covered) {
    /* 1024 cells * 4 m = 4096 m per side. Centre the ledger on the first trusted
     * saved exposure (chosen later); -2048 is a metre offset, not image resolution. */
    if (!coverage_init(&coverage_ledger, -2048, -2048, 4)) goto unlock;
    uint32_t token = 0;
    error = "session token";
    if (!random_session(&token)) goto unlock;
    covered = &coverage_ledger;
    session = token;
  }
  cameras[profile.camera_id - 1] = *camera;
  calibrated[profile.camera_id - 1] = true;
  accepted = true;
unlock:
  pthread_mutex_unlock(&guard);
  if (!accepted) fprintf(stderr, "USHER SURVEY: camera profile rejected: %s\n", error);
  return accepted;
}

void capture_arbiter_set_enforce(bool enabled)
{
  pthread_mutex_lock(&guard);
  enforce = enabled;
  pthread_mutex_unlock(&guard);
}

bool capture_arbiter_set_overlap(double overlap)
{
  if (!isfinite(overlap) || overlap < 0 || overlap > 0.9) return false;
  pthread_mutex_lock(&guard);
  minimum_overlap = overlap;
  pthread_mutex_unlock(&guard);
  return true;
}

bool capture_arbiter_set_camera_capacity(unsigned camera_id, unsigned limit)
{
  if (camera_id < 1 || camera_id > COVERAGE_CAMERAS || limit < 1 || limit > 2) return false;
  pthread_mutex_lock(&guard);
  capacity[camera_id - 1] = limit;
  pthread_mutex_unlock(&guard);
  return true;
}

uint32_t capture_arbiter_session_id(void)
{
  pthread_mutex_lock(&guard);
  const uint32_t token = session;
  pthread_mutex_unlock(&guard);
  return token;
}

void capture_arbiter_set_scope(const union usher_capture_arbiter_state_union *scope, uint64_t now_us)
{
  if (!scope || scope->data.active > 1 || scope->data.camera_mask > USHER_CAMERA_MASK_ALL) return;
  pthread_mutex_lock(&guard);
  if (!session || scope->data.session != session) goto done;
  scope_active = scope->data.active != 0;
  scope_mask = scope->data.camera_mask & (USHER_CAMERA_MASK_CHDK | USHER_CAMERA_MASK_AICAM | USHER_CAMERA_MASK_LWIR);
  scope_us = now_us;
  for (size_t index = 0; index < ARBITER_COUNT(waiting); ++index) {
    struct capture_intent *intent = &waiting[index];
    if (!scope_active) expire(intent, USHER_CAPTURE_ARBITER_EXPIRED);
    else {
      struct capture_intent removed = *intent;
      removed.mask &= ~scope_mask;
      expire(&removed, USHER_CAPTURE_ARBITER_EXPIRED);
      intent->mask &= scope_mask;
    }
  }
done:
  pthread_mutex_unlock(&guard);
}

bool capture_arbiter_offer(const union usher_capture_arbiter_shoot_union *request, uint64_t now_us)
{
  if (!request || request->data.camera_mask > USHER_CAMERA_MASK_ALL) return false;
  bool accepted = false;
  pthread_mutex_lock(&guard);
  if (!session || request->data.session != session) goto done;
  accepted = true;
  struct capture_intent offered = {.shot = request->data.shot, .mask = request->data.camera_mask & 7U, .received_us = now_us};
  if (!offered.mask) goto done;
  if (!fresh_scope(now_us)) {
    expire(&offered, USHER_CAPTURE_ARBITER_EXPIRED);
    goto done;
  }
  offered.mask &= scope_mask;
  /* Do not create another exposure for a request still owned by that camera. */
  for (size_t index = 0; index < ARBITER_COUNT(active); ++index)
    if (active[index].mask && active[index].shot.data.nr == offered.shot.data.nr) offered.mask &= ~active[index].mask;
  for (unsigned camera_index = 0; camera_index < COVERAGE_CAMERAS; ++camera_index) {
    const uint32_t bit = 1U << camera_index;
    if (!(offered.mask & bit)) continue;
    const size_t begin = camera_index * COVERAGE_CANDIDATES;
    size_t free_entry = ARBITER_COUNT(waiting), oldest = begin;
    bool duplicate = false;
    for (size_t index = begin; index < begin + COVERAGE_CANDIDATES; ++index) {
      const struct capture_intent *intent = &waiting[index];
      if (intent->mask && intent->shot.data.nr == offered.shot.data.nr) duplicate = true;
      if (!intent->mask && free_entry == ARBITER_COUNT(waiting)) free_entry = index;
      if (intent->received_us < waiting[oldest].received_us) oldest = index;
    }
    if (duplicate) continue;
    const size_t destination = free_entry != ARBITER_COUNT(waiting) ? free_entry : oldest;
    if (waiting[destination].mask) expire(&waiting[destination], USHER_CAPTURE_ARBITER_SUPERSEDED);
    waiting[destination] = (struct capture_intent) {.shot = offered.shot, .mask = bit, .received_us = now_us};
  }
done:
  pthread_mutex_unlock(&guard);
  return accepted;
}

/** @brief Make one admission decision while guard is held; no callbacks or dynamic allocation. */
static bool take_locked(uint32_t ready_mask, uint64_t now_us, union dc_shot_union *shot, uint8_t *mask)
{
  for (size_t index = 0; index < ARBITER_COUNT(waiting); ++index) {
    struct capture_intent *intent = &waiting[index];
    if (intent->mask && (!fresh_scope(now_us) || now_us < intent->received_us
                         || now_us - intent->received_us > intent_lifetime_us)) expire(intent, USHER_CAPTURE_ARBITER_EXPIRED);
  }
  unsigned free_slots = 0;
  for (size_t index = 0; index < ARBITER_COUNT(active); ++index) if (!active[index].mask) ++free_slots;
  if (!free_slots) return false;
  for (unsigned camera_id = 1; camera_id <= COVERAGE_CAMERAS; ++camera_id) {
    unsigned in_flight = 0;
    for (size_t index = 0; index < ARBITER_COUNT(active); ++index)
      if (active[index].mask & (1U << (camera_id - 1))) ++in_flight;
    if (in_flight >= capacity[camera_id - 1]) ready_mask &= ~(1U << (camera_id - 1));
  }
  /* An unavailable sensor must not head-of-line block another bit of the FC mask. */
  size_t oldest = ARBITER_COUNT(waiting);
  for (size_t index = 0; index < ARBITER_COUNT(waiting); ++index)
    if ((waiting[index].mask & ready_mask) && (oldest == ARBITER_COUNT(waiting)
        || waiting[index].received_us < waiting[oldest].received_us)) oldest = index;
  if (oldest == ARBITER_COUNT(waiting)) return false;
  struct capture_intent *intent = &waiting[oldest];
  uint32_t selected = intent->mask & ready_mask;
  for (unsigned camera_id = 1; camera_id <= COVERAGE_CAMERAS; ++camera_id) {
    const uint32_t bit = 1U << (camera_id - 1);
    if (!(selected & bit)) continue;
    struct coverage_candidate candidate;
    union dc_shot_union current;
    double uncertainty = 0;
    if (predict(now_us, &current, &uncertainty) && project(&current, camera_id, uncertainty, &candidate)) {
      const double gain = coverage_gain(covered, &candidate);
      struct coverage_candidate opportunities[COVERAGE_CANDIDATES] = {0};
      size_t count = 0;
      for (unsigned step = 0; step < ARBITER_COUNT(opportunities); ++step) {
        const uint64_t planned = now_us + step * 75000;
        union dc_shot_union prediction;
        double error = 0;
        struct coverage_candidate opportunity;
        if (!predict(planned, &prediction, &error) || !project(&prediction, camera_id, error, &opportunity)) break;
        opportunity.ready_us = planned;
        opportunity.deadline_us = planned + 75000;
        opportunities[count++] = opportunity;
      }
      const int choice = coverage_choose(covered, opportunities, count, now_us, service_us[camera_id - 1]);
      fprintf(stderr, "USHER SURVEY: shot=%d camera=%u coverage_score_m2=%.1f mode=%s\n",
              intent->shot.data.nr, camera_id, gain, enforce ? "enforce" : "observe");
      if (choice > 0) fprintf(stderr, "USHER SURVEY: lookahead=%d defer_us=%llu\n", choice,
                                (unsigned long long)(opportunities[choice].ready_us - now_us));
      /* Zero gain is not proof of redundancy: uncertainty-expanded coverage is. */
      if (enforce && coverage_wholly_saved(covered, &candidate)) {
        report(intent, camera_id, USHER_CAPTURE_ARBITER_COVERED);
        selected &= ~bit;
        intent->mask &= ~bit;
      } else if (enforce && choice > 0 && now_us - intent->received_us < 225000
                 && gain > 0 && gain < coverage_gain(covered, &opportunities[choice]) * 0.5
                 && have_saved[camera_id - 1]
                 && coverage_overlap(covered, &opportunities[choice], &last_saved[camera_id - 1]) >= minimum_overlap)
        selected &= ~bit;
    }
  }
  if (!selected) return false;
  /* Forecasts guide selection; the returned request must retain its original pose. */
  *shot = intent->shot;
  *mask = (uint8_t)selected;
  for (unsigned index = 0; index < COVERAGE_CAMERAS; ++index)
    if (selected & (1U << index)) {
      for (size_t slot = 0; slot < ARBITER_COUNT(active); ++slot) if (!active[slot].mask) {
          active[slot] = (struct capture_intent) {.shot = intent->shot, .mask = 1U << index, .received_us = now_us};
          break;
        }
    }
  intent->mask &= ~selected;
  return true;
}

bool capture_arbiter_take(uint32_t ready_mask, uint64_t now_us, union dc_shot_union *shot, uint8_t *mask)
{
  if (!shot || !mask) return false;
  pthread_mutex_lock(&guard);
  const bool taken = take_locked(ready_mask, now_us, shot, mask);
  pthread_mutex_unlock(&guard);
  return taken;
}

void capture_arbiter_complete(const union dc_shot_union *shot, unsigned camera_id, bool saved,
                              bool exposure_pose_valid, double uncertainty_m, uint64_t now_us)
{
  if (!shot || camera_id < 1 || camera_id > COVERAGE_CAMERAS) return;
  pthread_mutex_lock(&guard);
  struct capture_intent *intent = NULL;
  for (size_t index = 0; index < ARBITER_COUNT(active); ++index)
    if (active[index].mask == (1U << (camera_id - 1)) && active[index].shot.data.nr == shot->data.nr) {
      intent = &active[index];
      break;
    }
  if (!intent) goto done;
  /* Do not overwrite split-phase acquisition timing with later disk/EXIF latency. */
  if (!intent->acquired && now_us >= intent->received_us && now_us - intent->received_us >= 10000
      && now_us - intent->received_us <= 30000000) {
    const uint64_t measured = now_us - intent->received_us;
    const uint64_t averaged = (3 * service_us[camera_id - 1] + measured) / 4;
    service_us[camera_id - 1] = averaged > 10000 ? averaged : 10000;
  }
  if (saved) {
    ++saved_count;
    /* File success and trusted ground coverage are separate facts. */
    if (exposure_pose_valid && isfinite(uncertainty_m) && uncertainty_m >= 0 && now_us >= intent->received_us) {
      if (!origin_set) {
        /* Anchor once: moving it would silently assign saved cells to different ground. */
        latitude = shot->data.lat / 1e7;
        longitude = shot->data.lon / 1e7;
        origin_set = true;
      }
      struct coverage_candidate candidate;
      if (project(shot, camera_id, uncertainty_m, &candidate) && coverage_commit(covered, &candidate)) {
        last_saved[camera_id - 1] = candidate;
        have_saved[camera_id - 1] = true;
      }
    }
  }
  report(intent, camera_id, saved ? USHER_CAPTURE_ARBITER_SAVED : USHER_CAPTURE_ARBITER_FAILED);
  *intent = (struct capture_intent) {0};
done:
  pthread_mutex_unlock(&guard);
}

bool capture_arbiter_result(union usher_capture_arbiter_result_union *result, uint32_t ready_mask)
{
  if (!result) return false;
  pthread_mutex_lock(&guard);
  const bool available = result_count != 0;
  if (available) {
    *result = results[result_head];
    result->data.ready_mask = ready_mask & 7U;
    result_head = (result_head + 1) % ARBITER_COUNT(results);
    --result_count;
  }
  pthread_mutex_unlock(&guard);
  return available;
}

bool capture_arbiter_pending(void)
{
  pthread_mutex_lock(&guard);
  bool pending = false;
  for (size_t index = 0; index < ARBITER_COUNT(waiting); ++index) if (waiting[index].mask) pending = true;
  for (size_t index = 0; index < ARBITER_COUNT(active); ++index) if (active[index].mask) pending = true;
  pthread_mutex_unlock(&guard);
  return pending;
}

bool capture_arbiter_owns(int32_t number)
{
  pthread_mutex_lock(&guard);
  bool owns = false;
  for (size_t index = 0; index < ARBITER_COUNT(active); ++index)
    if (active[index].mask && active[index].shot.data.nr == number) owns = true;
  pthread_mutex_unlock(&guard);
  return owns;
}

void capture_arbiter_acquired(int32_t number, unsigned camera_id, uint64_t now_us)
{
  if (camera_id < 1 || camera_id > COVERAGE_CAMERAS) return;
  pthread_mutex_lock(&guard);
  for (size_t index = 0; index < ARBITER_COUNT(active); ++index) {
    struct capture_intent *intent = &active[index];
    if (intent->mask != (1U << (camera_id - 1)) || intent->shot.data.nr != number) continue;
    if (now_us >= intent->received_us && now_us - intent->received_us <= 5000000) {
      const uint64_t elapsed = now_us - intent->received_us;
      if (!intent->acquired) service_us[camera_id - 1] = elapsed > 10000 ? elapsed : 10000;
      intent->acquired = true;
    }
    break;
  }
  pthread_mutex_unlock(&guard);
}

void capture_arbiter_stop(void)
{
  pthread_mutex_lock(&guard);
  scope_active = false;
  /* Withdraw future intent, not pixels already acquired and still owed publication. */
  for (size_t index = 0; index < ARBITER_COUNT(waiting); ++index) expire(&waiting[index], USHER_CAPTURE_ARBITER_EXPIRED);
  pthread_mutex_unlock(&guard);
}

void capture_arbiter_pose_invalidate(void)
{
  pthread_mutex_lock(&guard);
  pose_count = 0;
  pose_head = 0;
  gps_changed_us = 0;
  pthread_mutex_unlock(&guard);
}

void capture_arbiter_pose_sample(const union usher_pose_sample_union *sample, uint64_t earliest_us, uint64_t latest_us)
{
  pthread_mutex_lock(&guard);
  if (!sample || !earliest_us || latest_us < earliest_us || latest_us - earliest_us > 100000
      || !usher_pose_flags_valid(sample->data.flags)
      || (uint32_t)(sample->data.sample_end_us - sample->data.sample_begin_us) > 10000
      || !(sample->data.flags & USHER_POSE_SAMPLE_GPS_PRESENT) || sample->data.gps_fix < 3
      || !sample->data.gps_hacc_cm || sample->data.gps_hacc_cm > 1000
      || sample->data.gps_tow_ms >= 604800000U
      || llabs((long long)sample->data.shot.data.lat) > 850000000
      || llabs((long long)sample->data.shot.data.lon) > 1800000000
      || sample->data.shot.data.groundalt <= 0 || sample->data.shot.data.groundalt > 10000 * 256
      || sample->data.shot.data.vground < 0 || sample->data.shot.data.vground > 100 * 524288) {
    pose_count = 0;
    goto done;
  }
  const uint64_t middle = earliest_us + (latest_us - earliest_us) / 2;
  /* The midpoint is useful for interpolation only when its uncertainty travels with it. */
  if (pose_count && middle <= poses[(pose_head + pose_count - 1) % ARBITER_COUNT(poses)].time_us) goto done;
  if (sample->data.gps_week || sample->data.gps_tow_ms) {
    if (!gps_changed_us || gps_week != sample->data.gps_week || gps_tow_ms != sample->data.gps_tow_ms) {
      if (gps_changed_us && (sample->data.gps_week < gps_week
                             || (sample->data.gps_week == gps_week && sample->data.gps_tow_ms < gps_tow_ms))) pose_count = 0;
      gps_week = sample->data.gps_week;
      gps_tow_ms = sample->data.gps_tow_ms;
      gps_changed_us = middle;
    } else if (middle < gps_changed_us || middle - gps_changed_us >= 2000000) {
      pose_count = 0;
      goto done;
    }
  }
  union dc_shot_union aligned = sample->data.shot;
  double accuracy_m = fmax(3.0, sample->data.gps_hacc_cm / 100.0);
  const bool projected = (sample->data.flags & USHER_POSE_SAMPLE_GPS_HELD) != 0;
  const uint32_t age_ms = sample->data.flags >> USHER_POSE_SAMPLE_GPS_AGE_SHIFT;
  if (projected) {
    const double north = sample->data.velocity_north_bfp / 524288.0;
    const double east = sample->data.velocity_east_bfp / 524288.0;
    const double speed = hypot(north, east), age_s = age_ms / 1000.0;
    if (age_ms > 500 || !(sample->data.gps_valid_fields & ((1U << 3) | (1U << 4)))
        || speed > 100 || sample->data.gps_sacc_cm_s > 1000
        || !compensate_ground_position(&aligned.data.lat, &aligned.data.lon, speed, atan2(east, north), age_s)) {
      pose_count = 0;
      goto done;
    }
    accuracy_m += fmax(1.0, sample->data.gps_sacc_cm_s / 100.0) * age_s + 5.0 * age_s * age_s;
  }
  if (pose_count == ARBITER_COUNT(poses)) {
    pose_head = (pose_head + 1) % ARBITER_COUNT(poses);
    --pose_count;
  }
  poses[(pose_head + pose_count++) % ARBITER_COUNT(poses)] = (struct capture_pose) {
    .shot = aligned, .time_us = middle, .error_us = (latest_us - earliest_us + 1) / 2,
                                        .accuracy_m = accuracy_m, .navigation_age_ms = projected ? age_ms : 0, .navigation_projected = projected
  };
done:
  pthread_mutex_unlock(&guard);
}

/**
 * @brief Interpolate a scalar fixed-point pose field.
 * @param first Value at the earlier sample.
 * @param second Value at the later sample.
 * @param fraction Position between the endpoints, from zero to one.
 * @return Nearest representable value in the original fixed-point units.
 * @details Widen before subtracting to accommodate opposite signed endpoints.
 * Use wrapped angular deltas instead for headings and attitude.
 */
static int32_t interpolate(int32_t first, int32_t second, double fraction)
{
  return (int32_t)llround(first + fraction * ((int64_t)second - first));
}

/**
 * @brief Interpolate a qualified pose from bounded sample history.
 * @param time_us Query time in local monotonic microseconds.
 * @param time_error_us Timing uncertainty around the query, in microseconds.
 * @param[out] shot Interpolated pose; use only on success.
 * @param[out] uncertainty_m Estimated position uncertainty in metres; use only on success.
 * @param[in,out] observation Optional destination for navigation provenance on success.
 * @return True if the query is bracketed and passes motion and uncertainty checks.
 * @details Do not extrapolate beyond recorded pose samples. Reject discontinuous
 * or widely spaced pairs before interpolating scalars and wrapped angles.
 * Combine timing, navigation accuracy, and attitude change as a conservative
 * operational check, not a surveyed accuracy guarantee.
 */
static bool pose_at(uint64_t time_us, uint64_t time_error_us, union dc_shot_union *shot,
                    double *uncertainty_m, struct capture_observation *observation)
{
  if (!shot || !uncertainty_m || time_error_us > 100000) return false;
  bool valid = false;
  pthread_mutex_lock(&guard);
  for (size_t index = 1; index < pose_count; ++index) {
    const struct capture_pose *before = &poses[(pose_head + index - 1) % ARBITER_COUNT(poses)];
    const struct capture_pose *after = &poses[(pose_head + index) % ARBITER_COUNT(poses)];
    if (time_us < before->time_us || time_us > after->time_us) continue;
    const uint64_t span = after->time_us - before->time_us;
    if (!span || span > 200000) break;
    const double fraction = (double)(time_us - before->time_us) / span;
    const double roll_change = angle_delta(before->shot.data.phi, after->shot.data.phi);
    const double pitch_change = angle_delta(before->shot.data.theta, after->shot.data.theta);
    if (fabs(roll_change) > 0.1 || fabs(pitch_change) > 0.1
        || fabs(before->shot.data.phi / 4096.0) > 0.6 || fabs(after->shot.data.phi / 4096.0) > 0.6
        || fabs(before->shot.data.theta / 4096.0) > 0.6 || fabs(after->shot.data.theta / 4096.0) > 0.6
        || llabs((long long)after->shot.data.lat - before->shot.data.lat) > 10000
        || llabs((long long)after->shot.data.lon - before->shot.data.lon) > 10000) break;
    *shot = before->shot;
    shot->data.lat = interpolate(before->shot.data.lat, after->shot.data.lat, fraction);
    shot->data.lon = interpolate(before->shot.data.lon, after->shot.data.lon, fraction);
    shot->data.alt = interpolate(before->shot.data.alt, after->shot.data.alt, fraction);
    shot->data.groundalt = interpolate(before->shot.data.groundalt, after->shot.data.groundalt, fraction);
    shot->data.phi = (int32_t)llround(before->shot.data.phi + fraction * roll_change * 4096);
    shot->data.theta = (int32_t)llround(before->shot.data.theta + fraction * pitch_change * 4096);
    shot->data.psi = (int32_t)llround(before->shot.data.psi
                                      + fraction * angle_delta(before->shot.data.psi, after->shot.data.psi) * 4096);
    shot->data.course = (int32_t)llround(before->shot.data.course
                                         + fraction * angle_delta(before->shot.data.course, after->shot.data.course) * 4096);
    shot->data.vground = interpolate(before->shot.data.vground, after->shot.data.vground, fraction);
    const double speed = fmax(before->shot.data.vground, after->shot.data.vground) / 524288.0;
    /* Clock width, sample spacing and attitude change are an engineering error
     * margin, not a statistical covariance or a guarantee of surveyed accuracy. */
    *uncertainty_m = fmax(before->accuracy_m, after->accuracy_m)
                     + speed * (time_error_us + before->error_us + after->error_us + span / 2) / 1e6
                     + shot->data.groundalt / 256.0 * tan(hypot(roll_change, pitch_change));
    valid = isfinite(*uncertainty_m) && *uncertainty_m <= 15;
    if (valid && observation) {
      observation->navigation_projected = before->navigation_projected || after->navigation_projected;
      observation->navigation_age_ms = before->navigation_age_ms > after->navigation_age_ms
                                       ? before->navigation_age_ms : after->navigation_age_ms;
    }
    break;
  }
  pthread_mutex_unlock(&guard);
  return valid;
}

bool capture_arbiter_pose_at(uint64_t time_us, uint64_t time_error_us, union dc_shot_union *shot, double *uncertainty_m)
{
  return pose_at(time_us, time_error_us, shot, uncertainty_m, NULL);
}

bool capture_arbiter_observation_pose(struct capture_observation *observation, uint64_t wait_us)
{
  if (!observation) return false;
  observation->pose_valid = false;
  if (!capture_observation_valid(observation) || wait_us > 200000) return false;
  const uint64_t span = observation->latest_us - observation->earliest_us;
  if (span > 200000) return false;
  const uint64_t time = observation->earliest_us + span / 2;
  const uint64_t started = monotonic_us();
  if (!started || time > started || started - time > 30000000) return false;
  for (;;) {
    if (pose_at(time, (span + 1) / 2, &observation->pose, &observation->pose_uncertainty_m, observation)) {
      observation->pose_valid = true;
      return true;
    }
    const uint64_t now = monotonic_us();
    if (!now || now < started || now - started >= wait_us) return false;
    pthread_mutex_lock(&guard);
    const bool following_sample_missing = !pose_count
                                          || poses[(pose_head + pose_count - 1) % ARBITER_COUNT(poses)].time_us < time;
    pthread_mutex_unlock(&guard);
    if (!following_sample_missing) return false;
    const struct timespec delay = {.tv_nsec = 5000000};
    if (nanosleep(&delay, NULL) && errno != EINTR) return false;
  }
}

bool capture_arbiter_frame_timing(const char *filename, unsigned camera_id, struct capture_observation *observation)
{
  if (!filename || !observation || camera_id < 1 || camera_id > COVERAGE_CAMERAS) return false;
  *observation = (struct capture_observation) {0};
  struct stat before, after, path;
  if (lstat(filename, &path) || !S_ISREG(path.st_mode)) return false;
  const int descriptor = open(filename, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) return false;
  FILE *input = fdopen(descriptor, "rb");
  if (!input) { close(descriptor); return false; }
  unsigned char payload[EXIF_MAX_PAYLOAD];
  size_t size = 0;
  struct exif_view view;
  struct exif_value value;
  struct sensor_capture record;
  uint32_t width, height;
  unsigned components;
  bool valid = false;
  if (fstat(descriptor, &before) || !S_ISREG(before.st_mode) || before.st_size <= 0
    || before.st_dev != path.st_dev || before.st_ino != path.st_ino
    || exif_read_stream(input, payload, sizeof(payload), &size)
    || exif_parse(&view, payload, size)
    || !exif_get(&view, EXIF_CAMERA, SENSOR_CAPTURE_TAG, &value)
    || value.format != EXIF_UNDEFINED || !sensor_capture_decode(&record, value.data, value.size)
    || fseek(input, 0, SEEK_SET)
    || exif_jpeg_dimensions(input, &width, &height, &components)
    || width != record.width || height != record.height
    || fstat(descriptor, &after) || lstat(filename, &path)
    || after.st_size != before.st_size || after.st_mtim.tv_sec != before.st_mtim.tv_sec
    || after.st_mtim.tv_nsec != before.st_mtim.tv_nsec
    || path.st_dev != before.st_dev || path.st_ino != before.st_ino) goto done;
  char boot[37];
  usher_boot_id(boot);
  if (strcmp(record.boot_id, boot)) goto done;
  pthread_mutex_lock(&guard);
  const bool calibrated_frame = !calibrated[camera_id - 1]
                                || (width == cameras[camera_id - 1].width && height == cameras[camera_id - 1].height);
  pthread_mutex_unlock(&guard);
  if (!calibrated_frame) goto done;
  const uint64_t time = record.monotonic_us, error = record.time_error_us;
  const uint64_t now = monotonic_us();
  *observation = (struct capture_observation) {
    .sequence = record.sequence, .sensor_timestamp_ns = record.sensor_timestamp_ns, .frame_bound = true
  };
  valid = true;
  if (!time || time <= error || time > now || now - time > 30000000
      || error > 100000 || time > UINT64_MAX - error) goto done;
  observation->earliest_us = time - error;
  observation->latest_us = time + error;
  observation->kind = CAPTURE_TIME_SENSOR_EXPOSURE;
done:
  if (fclose(input)) valid = false;
  if (!valid) *observation = (struct capture_observation) {0};
  return valid;
}

bool capture_arbiter_frame_pose(const char *filename, unsigned camera_id, const union dc_shot_union *request,
                                union dc_shot_union *exposure, double *uncertainty_m)
{
  struct capture_observation observation = {0};
  if (!request || !exposure || !uncertainty_m
      || !capture_arbiter_frame_timing(filename, camera_id, &observation)
      || !capture_arbiter_observation_pose(&observation, 0)) return false;
  *exposure = observation.pose;
  exposure->data.nr = request->data.nr;
  *uncertainty_m = observation.pose_uncertainty_m;
  return true;
}