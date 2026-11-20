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

/**
 * @file coverage_scheduler.c
 * @brief Bounded geometric decisions without pixel decoding or camera dependencies.
 * @details Two different margins are intentional: shrink footprints when granting
 * credit, but expand them before deciding a new photograph is redundant. Using
 * the same test for both would turn uncertainty into falsely claimed coverage.
 */
#include "coverage_scheduler.h"

#include <math.h>
#include <string.h>

/** @brief Validated inclusive grid bounds and polygon winding for half-plane tests. */
struct coverage_bounds { int left, right, bottom, top; double winding; };

/** @brief Apply Rz*Ry*Rx in place, matching the mount/body order used by PANGEA. */
static void rotate(double vector[3], double roll_deg, double pitch_deg, double yaw_deg)
{
  const double radians = 0.017453292519943295;
  const double roll = roll_deg * radians, pitch = pitch_deg * radians, yaw = yaw_deg * radians;
  const double east = cos(roll) * vector[1] - sin(roll) * vector[2];
  const double down = sin(roll) * vector[1] + cos(roll) * vector[2];
  const double north = cos(pitch) * vector[0] + sin(pitch) * down;
  vector[2] = -sin(pitch) * vector[0] + cos(pitch) * down;
  vector[0] = cos(yaw) * north - sin(yaw) * east;
  vector[1] = sin(yaw) * north + cos(yaw) * east;
}

bool coverage_project(const struct coverage_camera *camera, const struct coverage_pose *pose,
                      double origin_latitude_deg, double origin_longitude_deg, struct coverage_footprint *footprint)
{
  if (!camera || !pose || !footprint || camera->width < 2 || camera->width > 4096
      || camera->height < 2 || camera->height > 4096 || !isfinite(camera->fx) || camera->fx < 1
      || !isfinite(camera->fy) || camera->fy < 1 || !isfinite(camera->cx) || camera->cx < 0
      || camera->cx >= camera->width || !isfinite(camera->cy) || camera->cy < 0 || camera->cy >= camera->height
      || !isfinite(camera->mount_roll_deg) || !isfinite(camera->mount_pitch_deg) || !isfinite(camera->mount_yaw_deg)
      || fabs(camera->mount_roll_deg) > 180 || fabs(camera->mount_pitch_deg) > 180 || fabs(camera->mount_yaw_deg) > 180
      || !isfinite(pose->latitude_deg) || fabs(pose->latitude_deg) > 85
      || !isfinite(pose->longitude_deg) || fabs(pose->longitude_deg) > 180
      || !isfinite(origin_latitude_deg) || fabs(origin_latitude_deg) > 85
      || !isfinite(origin_longitude_deg) || fabs(origin_longitude_deg) > 180
      || !isfinite(pose->agl_m) || pose->agl_m < 0.1 || pose->agl_m > 10000
      || !isfinite(pose->roll_deg) || !isfinite(pose->pitch_deg) || !isfinite(pose->yaw_deg)
      || !isfinite(pose->uncertainty_m) || pose->uncertainty_m < 0) return false;
  const double radians = 0.017453292519943295, earth_radius = 6378137;
  /* A local plane keeps decisions cheap. Reject distant poses instead of silently
   * stretching the approximation; the caller can still take an uncredited photo. */
  const double north = earth_radius * (pose->latitude_deg - origin_latitude_deg) * radians;
  const double east = earth_radius * cos(origin_latitude_deg * radians)
                      * remainder(pose->longitude_deg - origin_longitude_deg, 360) * radians;
  if (hypot(east, north) > 5000) return false;
  struct coverage_footprint projected = {.uncertainty_m = pose->uncertainty_m + (east * east + north * north) / (2 * earth_radius)};
  for (unsigned index = 0; index < 4; ++index) {
    const double column = index == 1 || index == 2 ? camera->width - 1 : 0;
    const double row = index >= 2 ? camera->height - 1 : 0;
    /* Image-up is body-forward, image-right is starboard, optical-axis is down.
     * Mount first, then aircraft attitude: reversing these changes the footprint. */
    double ray[3] = {-(row - camera->cy) / camera->fy, (column - camera->cx) / camera->fx, 1};
    rotate(ray, camera->mount_roll_deg, camera->mount_pitch_deg, camera->mount_yaw_deg);
    rotate(ray, pose->roll_deg, pose->pitch_deg, pose->yaw_deg);
    if (!isfinite(ray[2]) || ray[2] <= 0.1) return false;
    projected.corner[index] = (struct coverage_point) {
      east + pose->agl_m * ray[1] / ray[2],
           north + pose->agl_m * ray[0] / ray[2]
    };
  }
  *footprint = projected;
  return true;
}

bool coverage_project_pixel(const struct coverage_camera *camera, const struct coverage_pose *pose,
                            double column, double row, struct coverage_point *offset)
{
  struct coverage_footprint footprint;
  if (!camera || !pose || !offset || !isfinite(column) || !isfinite(row) ||
      column < 0 || row < 0 || column >= camera->width || row >= camera->height ||
      !coverage_project(camera, pose, pose->latitude_deg, pose->longitude_deg, &footprint)) return false;
  double ray[3] = {-(row - camera->cy) / camera->fy, (column - camera->cx) / camera->fx, 1};
  rotate(ray, camera->mount_roll_deg, camera->mount_pitch_deg, camera->mount_yaw_deg);
  rotate(ray, pose->roll_deg, pose->pitch_deg, pose->yaw_deg);
  if (!isfinite(ray[2]) || ray[2] <= 0.1) return false;
  const struct coverage_point projected = {pose->agl_m * ray[1] / ray[2], pose->agl_m * ray[0] / ray[2]};
  if (!isfinite(projected.east) || !isfinite(projected.north)) return false;
  *offset = projected;
  return true;
}

bool coverage_init(struct coverage_state *state, double east, double north, double cell_m)
{
  if (!state || !isfinite(east) || !isfinite(north) || !isfinite(cell_m) || cell_m < 0.25 || cell_m > 100)
    return false;
  memset(state, 0, sizeof(*state));
  state->origin_east = east;
  state->origin_north = north;
  state->cell_m = cell_m;
  return true;
}

/** @brief Reject unusable polygons before converting coordinates or scanning cells.
 * @details Convexity and work limits belong here so every scoring/credit path
 * shares the same conservative definition of a computable footprint.
 */
static bool bounds(const struct coverage_state *state, const struct coverage_candidate *candidate,
                   struct coverage_bounds *box)
{
  if (!state || !candidate || state->cell_m < 0.25 || !isfinite(state->cell_m)
      || !isfinite(state->origin_east) || !isfinite(state->origin_north)
      || candidate->camera_id < 1 || candidate->camera_id > COVERAGE_CAMERAS || !candidate->quality
      || !isfinite(candidate->footprint.uncertainty_m) || candidate->footprint.uncertainty_m < 0)
    return false;
  double left = INFINITY, right = -INFINITY, bottom = INFINITY, top = -INFINITY;
  double winding = 0;
  for (unsigned index = 0; index < 4; ++index) {
    const struct coverage_point first = candidate->footprint.corner[index];
    const struct coverage_point second = candidate->footprint.corner[(index + 1) % 4];
    const struct coverage_point third = candidate->footprint.corner[(index + 2) % 4];
    if (!isfinite(first.east) || !isfinite(first.north)) return false;
    const double turn = (second.east - first.east) * (third.north - second.north)
                        - (second.north - first.north) * (third.east - second.east);
    if (!isfinite(turn) || fabs(turn) < 1e-9 || (winding != 0 && turn * winding <= 0)) return false;
    winding = turn;
    left = fmin(left, first.east);
    right = fmax(right, first.east);
    bottom = fmin(bottom, first.north);
    top = fmax(top, first.north);
  }
  left = floor((left - state->origin_east) / state->cell_m);
  right = ceil((right - state->origin_east) / state->cell_m) - 1;
  bottom = floor((bottom - state->origin_north) / state->cell_m);
  top = ceil((top - state->origin_north) / state->cell_m) - 1;
  if (left < 0 || bottom < 0 || right >= COVERAGE_SIDE || top >= COVERAGE_SIDE
      || right < left || top < bottom || (right - left + 1) * (top - bottom + 1) > COVERAGE_SCAN_LIMIT)
    return false;
  *box = (struct coverage_bounds) {(int)left, (int)right, (int)bottom, (int)top, winding > 0 ? 1 : -1};
  return true;
}

/** @brief Require the whole cell, not merely its centre, inside every polygon edge. */
static bool contains_cell(const struct coverage_state *state, const struct coverage_candidate *candidate,
                          const struct coverage_bounds *box, int column, int row)
{
  const double east = state->origin_east + (column + 0.5) * state->cell_m;
  const double north = state->origin_north + (row + 0.5) * state->cell_m;
  for (unsigned index = 0; index < 4; ++index) {
    const struct coverage_point first = candidate->footprint.corner[index];
    const struct coverage_point second = candidate->footprint.corner[(index + 1) % 4];
    const double delta_east = second.east - first.east, delta_north = second.north - first.north;
    const double distance = box->winding * (delta_east * (north - first.north) - delta_north * (east - first.east));
    /* The square-cell support term plus the position margin erodes each edge.
     * This avoids crediting a boundary cell whose unobserved corner hides a gap. */
    const double margin = 0.5 * state->cell_m * (fabs(delta_east) + fabs(delta_north))
                          + candidate->footprint.uncertainty_m * hypot(delta_east, delta_north);
    if (distance + 1e-9 < margin) return false;
  }
  return true;
}

/** @brief Score a second view against a hypothetical first, without committing either.
 * @details The overlay is local to this calculation. A predicted image must never
 * leak into the saved ledger, even if it scores well or is next in the queue.
 */
static double gain_after(const struct coverage_state *state, const struct coverage_candidate *candidate,
                         const struct coverage_candidate *previous)
{
  struct coverage_bounds box, previous_box;
  if (!bounds(state, candidate, &box)) return -1;
  const bool same_camera = previous && previous->camera_id == candidate->camera_id;
  if (same_camera && !bounds(state, previous, &previous_box)) return -1;
  uint64_t gain = 0;
  for (int row = box.bottom; row <= box.top; ++row) {
    for (int column = box.left; column <= box.right; ++column) {
      if (!contains_cell(state, candidate, &box, column, row)) continue;
      uint8_t saved = state->saved[candidate->camera_id - 1][row * COVERAGE_SIDE + column];
      if (same_camera && contains_cell(state, previous, &previous_box, column, row) && previous->quality > saved)
        saved = previous->quality;
      if (candidate->quality > saved) gain += candidate->quality - saved;
    }
  }
  return gain * state->cell_m * state->cell_m / 255.0;
}

double coverage_gain(const struct coverage_state *state, const struct coverage_candidate *candidate)
{
  return gain_after(state, candidate, NULL);
}

bool coverage_commit(struct coverage_state *state, const struct coverage_candidate *candidate)
{
  struct coverage_bounds box;
  if (!bounds(state, candidate, &box)) return false;
  for (int row = box.bottom; row <= box.top; ++row) {
    for (int column = box.left; column <= box.right; ++column) {
      if (!contains_cell(state, candidate, &box, column, row)) continue;
      uint8_t *saved = &state->saved[candidate->camera_id - 1][row * COVERAGE_SIDE + column];
      if (candidate->quality > *saved) *saved = candidate->quality;
    }
  }
  return true;
}

int coverage_choose(const struct coverage_state *state, const struct coverage_candidate *candidates,
                    size_t count, uint64_t now_us, uint64_t service_us)
{
  if (!state || !candidates || !count || count > COVERAGE_CANDIDATES || !service_us) return -1;
  double best = 0, best_first_gain = 0;
  /* Enumerating four candidates is cheap and deterministic. Looking one view
   * ahead preserves a small last-chance area that a greedy largest-area choice
   * could miss before taking a larger, still-visible area in the following shot. */
  uint64_t best_time = UINT64_MAX;
  int chosen = -1;
  for (size_t first = 0; first < count; ++first) {
    const uint64_t start = candidates[first].ready_us > now_us ? candidates[first].ready_us : now_us;
    if (start > candidates[first].deadline_us || service_us > UINT64_MAX - start) continue;
    const double first_gain = coverage_gain(state, &candidates[first]);
    if (first_gain <= 0) continue;
    double total = first_gain;
    for (size_t second = 0; second < count; ++second) {
      if (second == first) continue;
      const uint64_t next = candidates[second].ready_us > start + service_us
                            ? candidates[second].ready_us : start + service_us;
      if (next > candidates[second].deadline_us) continue;
      const double additional = gain_after(state, &candidates[second], &candidates[first]);
      if (additional > 0) total = fmax(total, first_gain + additional);
    }
    if (total > best || (total == best && (start < best_time || (start == best_time && first_gain > best_first_gain)))) {
      best = total;
      best_time = start;
      best_first_gain = first_gain;
      chosen = (int)first;
    }
  }
  return chosen;
}

bool coverage_wholly_saved(const struct coverage_state *state, const struct coverage_candidate *candidate)
{
  struct coverage_bounds box;
  if (!bounds(state, candidate, &box)) return false;
  const double margin = ceil(candidate->footprint.uncertainty_m / state->cell_m) + 1;
  /* A bounding rectangle deliberately asks for more evidence than the polygon.
   * At the map boundary or work limit, prefer an extra photo over a false skip. */
  if (margin > COVERAGE_SIDE || box.left < margin || box.bottom < margin
      || box.right + margin >= COVERAGE_SIDE || box.top + margin >= COVERAGE_SIDE
      || (box.right - box.left + 1 + 2 * margin) * (box.top - box.bottom + 1 + 2 * margin) > COVERAGE_SCAN_LIMIT) return
          false;
  for (int row = box.bottom - (int)margin; row <= box.top + (int)margin; ++row)
    for (int column = box.left - (int)margin; column <= box.right + (int)margin; ++column)
      if (state->saved[candidate->camera_id - 1][row * COVERAGE_SIDE + column] < candidate->quality) return false;
  return true;
}

double coverage_overlap(const struct coverage_state *state, const struct coverage_candidate *first,
                        const struct coverage_candidate *second)
{
  struct coverage_bounds first_box, second_box;
  if (!bounds(state, first, &first_box) || !bounds(state, second, &second_box)
      || first->camera_id != second->camera_id) return -1;
  unsigned total = 0, intersection = 0;
  /* Count uncertain/bounding cells in the denominator too. This lower estimate
   * prevents an optimistic overlap fraction from justifying an unsafe delay. */
  for (int row = first_box.bottom; row <= first_box.top; ++row)
    for (int column = first_box.left; column <= first_box.right; ++column) {
      ++total;
      if (!contains_cell(state, first, &first_box, column, row)) continue;
      if (contains_cell(state, second, &second_box, column, row)) ++intersection;
    }
  return total ? (double)intersection / total : -1;
}