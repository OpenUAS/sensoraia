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
 * @file ear_fusion.c
 * @brief Bounded ground-source localization from a geotagged acoustic survey.
 * @details Relative band power supplies a conservative starting centroid;
 * inverse-distance fitting improves it when survey geometry supports a fit.
 * Constants below preserve the existing estimator policy. They are documented
 * heuristics, not universal field-calibration values or flight-safety limits.
 */
#include "earcam_core.h"

#include <math.h>
#include <pthread.h>
#include <string.h>

#define EAR_MIN_SAMPLES 8U
#define EARTH_RADIUS_M 6378137.0
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

/** Legacy near-zero-latitude placeholder gate; it also excludes equatorial tracks. */
#define EAR_POSE_LATITUDE_EPSILON_DEG 1e-6
/** Gaussian-consistent conversion from median absolute deviation to sigma. */
#define EAR_MAD_TO_SIGMA 1.4826
#define EAR_MIN_LEVEL_SIGMA_DB 0.5
#define EAR_QUIET_OUTLIER_SIGMAS 4.0
#define EAR_PEAK_QUARTILE_DIVISOR 4
#define EAR_MIN_PEAK_WINDOWS 4
#define EAR_DB_LOG_BASE 10.0
#define EAR_POWER_DB_PER_DECADE 10.0
#define EAR_DISTANCE_DB_PER_DECADE 20.0
#define EAR_TREND_BONUS_LIMIT_DB 6.0
#define EAR_TREND_BONUS_SCALE_DB 12.0
/** Bound representative-height scratch space independently of survey size. */
#define EAR_HEIGHT_WINDOW_LIMIT 64
#define EAR_MIN_USABLE_HEIGHT_M 0.5
#define EAR_FALLBACK_HEIGHT_M 10.0
#define EAR_REFERENCE_DISTANCE_M 1.0
/** Fit east, north and unknown source level; altitude is supplied per window. */
#define EAR_FIT_PARAMETER_COUNT 3
#define EAR_FIT_EAST 0
#define EAR_FIT_NORTH 1
#define EAR_FIT_SOURCE_LEVEL 2
#define EAR_FIT_ITERATION_LIMIT 8
#define EAR_FIT_RESIDUAL_LIMIT_DB 6.0
#define EAR_FIT_DAMPING 1e-3
#define EAR_FIT_DETERMINANT_EPSILON 1e-12
#define EAR_FIT_HORIZONTAL_STEP_LIMIT_M 25.0
#define EAR_FIT_LEVEL_STEP_LIMIT_DB 6.0
#define EAR_FIT_CONVERGENCE_M 0.05
#define EAR_FIT_CENTROID_DRIFT_LIMIT_M 60.0
/** These three weights form a heuristic score, not calibrated detection odds. */
#define EAR_CONFIDENCE_PROMINENCE_WEIGHT 0.5
#define EAR_CONFIDENCE_CONCENTRATION_WEIGHT 0.3
#define EAR_CONFIDENCE_SUPPORT_WEIGHT 0.2
#define EAR_CONFIDENCE_PROMINENCE_SCALE_DB 12.0
#define EAR_CONFIDENCE_SPREAD_SCALE_M 40.0
#define EAR_CONFIDENCE_SUPPORT_WINDOWS 100.0

/** @brief Restore a zero-based binary max heap without allocating scratch memory. */
static void sift_down(double *values, size_t root, size_t count)
{
  for (;;) {
    size_t child = root * 2 + 1;
    if (child >= count) return;
    if (child + 1 < count && values[child] < values[child + 1]) ++child;
    if (values[root] >= values[child]) return;
    const double temporary = values[root];
    values[root] = values[child];
    values[child] = temporary;
    root = child;
  }
}

/** @brief In-place heapsort keeps worst-case workspace bounded for a full survey. */
static void sort_double(double *values, size_t count)
{
  for (size_t root = count / 2; root > 0; --root) sift_down(values, root - 1, count);
  for (size_t end = count; end > 1; --end) {
    const double temporary = values[0];
    values[0] = values[end - 1];
    values[end - 1] = temporary;
    sift_down(values, 0, end - 1);
  }
}

/** @brief Median of nonempty scratch values; sorting intentionally destroys their order. */
static double median_of(double *values, size_t count)
{
  sort_double(values, count);
  if (count % 2 == 1) {
    return values[count / 2];
  }
  return 0.5 * (values[count / 2 - 1] + values[count / 2]);
}

/** @brief Solver-local horizontal coordinates plus each window's own squared height. */
struct enu_sample {
  double east;
  double north;
  double height2;
  double level;
  double weight;
  const struct ear_sample *source;
};

static pthread_mutex_t solver_mutex = PTHREAD_MUTEX_INITIALIZER;
/* One workspace avoids per-session heap or stack growth. Public calls serialize
 * here; audio acquisition and USHER's UART thread do not run this computation. */
static struct enu_sample solver_points[EAR_MAX_SAMPLES];
static double solver_levels[EAR_MAX_SAMPLES];

int calculated_loudestspot(const struct ear_sample *samples, size_t count,
                           struct ear_loudest_spot *result)
{
  if (result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  result->sample_count = (uint32_t)count;
  if (samples == NULL || count < EAR_MIN_SAMPLES || count > EAR_MAX_SAMPLES) {
    return -1;
  }
  pthread_mutex_lock(&solver_mutex);

  /* Prefer detector-confirmed windows only when enough survive. The fallback
   * remains useful for unclassified tones, but is not an absolute silence gate. */
  size_t alarm_count = 0;
  for (size_t index = 0; index < count; index++) {
    const struct ear_sample *sample = &samples[index];
    if (sample->alarm && !sample->clipped && isfinite(sample->lat_deg)
        && isfinite(sample->lon_deg) && fabs(sample->lat_deg) > EAR_POSE_LATITUDE_EPSILON_DEG) {
      alarm_count++;
    }
  }
  bool require_alarm = alarm_count >= EAR_MIN_SAMPLES;
  struct enu_sample *points = solver_points;
  double *levels = solver_levels;
  size_t used = 0;
  for (size_t index = 0; index < count; index++) {
    const struct ear_sample *sample = &samples[index];
    if (sample->clipped || !isfinite(sample->lat_deg) || !isfinite(sample->lon_deg)
        || !isfinite(sample->level_db) || fabs(sample->lat_deg) <= EAR_POSE_LATITUDE_EPSILON_DEG
        || (require_alarm && !sample->alarm)) {
      continue;
    }
    points[used].source = sample;
    points[used].level = sample->level_db;
    levels[used] = sample->level_db;
    used++;
  }
  if (used < EAR_MIN_SAMPLES) {
    pthread_mutex_unlock(&solver_mutex);
    return -1;
  }

  /* A one-sided Hampel gate removes implausibly quiet evidence without deleting
   * the loud peak that the survey is trying to find. Preserve a noise-scale floor
   * so nearly identical levels do not make the rejection threshold collapse. */
  double median_level = median_of(levels, used);
  for (size_t index = 0; index < used; index++) {
    levels[index] = fabs(points[index].level - median_level);
  }
  double mad = median_of(levels, used) * EAR_MAD_TO_SIGMA;
  if (mad < EAR_MIN_LEVEL_SIGMA_DB) {
    mad = EAR_MIN_LEVEL_SIGMA_DB;
  }
  size_t kept = 0;
  for (size_t index = 0; index < used; index++) {
    if (points[index].level < median_level - EAR_QUIET_OUTLIER_SIGMAS * mad) {
      continue;
    }
    points[kept++] = points[index];
  }
  used = kept;
  if (used < EAR_MIN_SAMPLES) {
    pthread_mutex_unlock(&solver_mutex);
    return -1;
  }

  size_t peak_index = 0;
  for (size_t index = 1; index < used; index++) {
    if (points[index].level > points[peak_index].level) {
      peak_index = index;
    }
  }
  /* Work in a local east/north plane around the strongest window. This is a
   * small-survey approximation, not a global geodesic or terrain model. */
  double origin_lat = points[peak_index].source->lat_deg;
  double origin_lon = points[peak_index].source->lon_deg;
  double cos_lat = cos(origin_lat * DEG_TO_RAD);
  double max_level = points[peak_index].level;
  for (size_t index = 0; index < used; index++) {
    points[index].east = (points[index].source->lon_deg - origin_lon) * DEG_TO_RAD
                         * EARTH_RADIUS_M * cos_lat;
    points[index].north = (points[index].source->lat_deg - origin_lat) * DEG_TO_RAD
                          * EARTH_RADIUS_M;
  }

  for (size_t index = 0; index < used; index++) {
    levels[index] = points[index].level;
  }
  sort_double(levels, used);
  size_t top_count = used / EAR_PEAK_QUARTILE_DIVISOR;
  if (top_count < EAR_MIN_PEAK_WINDOWS) {
    top_count = used < EAR_MIN_PEAK_WINDOWS ? used : EAR_MIN_PEAK_WINDOWS;
  }
  double threshold = levels[used - top_count];
  double sum_weight = 0.0;
  double east0 = 0.0;
  double north0 = 0.0;
  for (size_t index = 0; index < used; index++) {
    if (points[index].level < threshold) {
      points[index].weight = 0.0;
      continue;
    }
    /* Normalizing power to the loudest window avoids huge absolute weights.
     * A bounded positive trend bonus mildly favors approaching the source. */
    double weight = pow(EAR_DB_LOG_BASE, (points[index].level - max_level) / EAR_POWER_DB_PER_DECADE);
    if (points[index].source->trend_db > 0.0) {
      weight *= 1.0 + fmin(points[index].source->trend_db, EAR_TREND_BONUS_LIMIT_DB) / EAR_TREND_BONUS_SCALE_DB;
    }
    points[index].weight = weight;
    sum_weight += weight;
    east0 += weight * points[index].east;
    north0 += weight * points[index].north;
  }
  if (sum_weight <= 0.0) {
    pthread_mutex_unlock(&solver_mutex);
    return -1;
  }
  east0 /= sum_weight;
  north0 /= sum_weight;

  /* A representative loud-window height is only a fallback and output summary.
   * Each usable window keeps its own AGL in the fit, so changing survey height
   * is not mistaken for a lateral change in source distance. */
  double agl_values[EAR_HEIGHT_WINDOW_LIMIT];
  size_t agl_count = 0;
  for (size_t index = 0; index < used && agl_count < EAR_HEIGHT_WINDOW_LIMIT; index++) {
    if (points[index].weight > 0.0 && isfinite(points[index].source->agl_m)
        && points[index].source->agl_m > EAR_MIN_USABLE_HEIGHT_M) {
      agl_values[agl_count++] = points[index].source->agl_m;
    }
  }
  double height = agl_count > 0 ? median_of(agl_values, agl_count) : EAR_FALLBACK_HEIGHT_M;
  for (size_t index = 0; index < used; index++) {
    double h = points[index].source->agl_m;
    if (!isfinite(h) || h < EAR_MIN_USABLE_HEIGHT_M) {
      h = height;
    }
    points[index].height2 = h * h;
  }
  double east = east0;
  double north = north0;
  double peak_height = sqrt(points[peak_index].height2);
  double level0 = max_level + EAR_DISTANCE_DB_PER_DECADE * log10(fmax(peak_height, EAR_REFERENCE_DISTANCE_M));
  bool fit_ok = true;
  /* Fit L = L0 - 10*log10(dx*dx + dy*dy + h*h). L0 absorbs unknown source
   * strength/gain; soft residual weighting and diagonal damping stabilize the
   * three-parameter normal equations when a track provides weak geometry. */
  for (int iteration = 0; iteration < EAR_FIT_ITERATION_LIMIT; iteration++) {
    double jtj[EAR_FIT_PARAMETER_COUNT][EAR_FIT_PARAMETER_COUNT] = {{0}};
    double jtr[EAR_FIT_PARAMETER_COUNT] = {0};
    for (size_t index = 0; index < used; index++) {
      double dx = points[index].east - east;
      double dy = points[index].north - north;
      double r2 = dx * dx + dy * dy + points[index].height2;
      double predicted = level0 - EAR_POWER_DB_PER_DECADE * log10(r2);
      double residual = points[index].level - predicted;
      double w = fabs(residual) > EAR_FIT_RESIDUAL_LIMIT_DB ? EAR_FIT_RESIDUAL_LIMIT_DB / fabs(residual) : 1.0;
      double k = EAR_DISTANCE_DB_PER_DECADE / (r2 * log(EAR_DB_LOG_BASE));
      double j[EAR_FIT_PARAMETER_COUNT] = {k * dx, k * dy, 1.0};
      for (int a = 0; a < EAR_FIT_PARAMETER_COUNT; a++) {
        jtr[a] += w * j[a] * residual;
        for (int b = 0; b < EAR_FIT_PARAMETER_COUNT; b++) {
          jtj[a][b] += w * j[a] * j[b];
        }
      }
    }
    for (int a = 0; a < EAR_FIT_PARAMETER_COUNT; a++) {
      jtj[a][a] += EAR_FIT_DAMPING * (jtj[a][a] + 1.0);
    }
    double det = jtj[0][0] * (jtj[1][1] * jtj[2][2] - jtj[1][2] * jtj[2][1])
                 - jtj[0][1] * (jtj[1][0] * jtj[2][2] - jtj[1][2] * jtj[2][0])
                 + jtj[0][2] * (jtj[1][0] * jtj[2][1] - jtj[1][1] * jtj[2][0]);
    if (!isfinite(det) || fabs(det) < EAR_FIT_DETERMINANT_EPSILON) {
      fit_ok = false;
      break;
    }
    /* The fixed 3x3 system uses Cramer's rule. Named dimensions and parameter
     * offsets are intentionally fixed; adding a parameter needs a new solver. */
    double delta[EAR_FIT_PARAMETER_COUNT];
    for (int column = 0; column < EAR_FIT_PARAMETER_COUNT; column++) {
      double m[EAR_FIT_PARAMETER_COUNT][EAR_FIT_PARAMETER_COUNT];
      for (int row = 0; row < EAR_FIT_PARAMETER_COUNT; row++) {
        for (int col = 0; col < EAR_FIT_PARAMETER_COUNT; col++) {
          m[row][col] = (col == column) ? jtr[row] : jtj[row][col];
        }
      }
      double d = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                 - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                 + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      delta[column] = d / det;
    }
    double step = sqrt(delta[EAR_FIT_EAST] * delta[EAR_FIT_EAST] + delta[EAR_FIT_NORTH] * delta[EAR_FIT_NORTH]);
    if (step > EAR_FIT_HORIZONTAL_STEP_LIMIT_M) {
      delta[EAR_FIT_EAST] *= EAR_FIT_HORIZONTAL_STEP_LIMIT_M / step;
      delta[EAR_FIT_NORTH] *= EAR_FIT_HORIZONTAL_STEP_LIMIT_M / step;
    }
    east += delta[EAR_FIT_EAST];
    north += delta[EAR_FIT_NORTH];
    level0 += fmax(-EAR_FIT_LEVEL_STEP_LIMIT_DB, fmin(EAR_FIT_LEVEL_STEP_LIMIT_DB, delta[EAR_FIT_SOURCE_LEVEL]));
    if (step < EAR_FIT_CONVERGENCE_M) {
      break;
    }
  }
  /* An unconstrained fit can wander far outside a narrow/flat survey. Keep the
   * robust seed rather than presenting that extrapolation as improved accuracy. */
  double drift = hypot(east - east0, north - north0);
  if (!fit_ok || !isfinite(east) || !isfinite(north) || drift > EAR_FIT_CENTROID_DRIFT_LIMIT_M) {
    east = east0;
    north = north0;
  }

  double prominence = fmin(1.0, fmax(0.0, (max_level - median_level) / EAR_CONFIDENCE_PROMINENCE_SCALE_DB));
  double spread = 0.0;
  for (size_t index = 0; index < used; index++) {
    if (points[index].weight > 0.0) {
      spread += points[index].weight * hypot(points[index].east - east, points[index].north - north);
    }
  }
  spread /= sum_weight;
  double concentration = fmin(1.0, fmax(0.0, 1.0 - spread / EAR_CONFIDENCE_SPREAD_SCALE_M));
  double support = fmin(1.0, (double)used / EAR_CONFIDENCE_SUPPORT_WINDOWS);
  double confidence = EAR_CONFIDENCE_PROMINENCE_WEIGHT * prominence
                      + EAR_CONFIDENCE_CONCENTRATION_WEIGHT * concentration
                      + EAR_CONFIDENCE_SUPPORT_WEIGHT * support;

  result->valid = true;
  result->lat_deg = origin_lat + (north / EARTH_RADIUS_M) * RAD_TO_DEG;
  result->lon_deg = origin_lon + (east / (EARTH_RADIUS_M * cos_lat)) * RAD_TO_DEG;
  result->agl_m = height;
  result->alt_m = points[peak_index].source->alt_m - sqrt(points[peak_index].height2);
  result->level_db = max_level;
  result->confidence = confidence;
  result->used_count = (uint32_t)used;
  pthread_mutex_unlock(&solver_mutex);
  return 0;
}