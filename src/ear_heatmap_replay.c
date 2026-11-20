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
 * @file ear_heatmap_replay.c
 * @brief Offline CSV replay utility for validating EARcam heatmaps and fusion. */
// C11 + POSIX.1-2008.
#define _POSIX_C_SOURCE 200809L

// Re-render an EAR session log (earlogs/ear_*.csv) into the loudest-spot heatmap
// exactly as USHER does in flight: fuse the samples, write the JPEG and its .geo
// georeference sidecar. The host-only ear_heatmap_overlay application can place
// the field over local or online satellite tiles.
//
//   ear_heatmap_replay earlogs/ear_20260908_103533.csv photos/e000686.jpg
//   -> photos/e000686.jpg, e000686.geo, e000686_field.jpg (undecorated overlay input)
#include "modules/ear_cam/ear_heatmap.h"
#include "version.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Bounded workspace matches EARcam's runtime sample ceiling; replay never grows with log size. */
static struct ear_sample samples[EAR_MAX_SAMPLES];

/**
 * @brief Decode one EAR session row into the shared fusion sample structure.
 * @param line Comma-separated log row in the order documented by the session header.
 * @param s Receives the parsed measurement; the caller owns this sample slot.
 * @return 0 for all twelve required fields, or -1 for an incomplete/malformed row.
 * @details Keeping replay on `ear_sample` makes the same solver and renderer handle
 * recorded evidence and live USHER snapshots.
 */
static int parse_line(const char *line, struct ear_sample *s)
{
  unsigned long long t_ms;
  int shot, alarm, clip;
  double freq;
  int fields = sscanf(line, "%llu,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d",
                      &t_ms, &shot, &s->lat_deg, &s->lon_deg, &s->agl_m, &s->alt_m,
                      &s->level_db, &s->trend_db, &s->contrast_db, &freq, &alarm, &clip);
  if (fields != 12) {
    return -1;
  }
  s->timestamp_ms = t_ms;
  s->shot_nr = shot;
  s->frequency_hz = freq;
  s->alarm = alarm != 0;
  s->clipped = clip != 0;
  return 0;
}

/**
 * @brief Recompute an EAR loudest-spot result and emit decorated/field JPEGs.
 * @param argc Command-line argument count.
 * @param argv Expects a session CSV and destination JPEG, or a help/version switch.
 * @return 0 on completed output, 2 for usage errors, or 1 when the log has no valid solve.
 * @details The undecorated sibling preserves clean heat colors for later map overlay;
 * the decorated JPEG remains the human-readable result and both share one `.geo` frame.
 */
int main(int argc, char **argv)
{
  if (argc == 2 && (!strcmp(argv[1], "--version") || !strcmp(argv[1], "-V"))) {
    puts(EAR_HEATMAP_REPLAY_BUILD_VERSION);
    return 0;
  }
  if (argc == 2 && (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h"))) {
    printf("%s\nUsage: %s SESSION.csv OUT.jpg\n", EAR_HEATMAP_REPLAY_BUILD_VERSION, argv[0]);
    return 0;
  }
  if (argc != 3) {
    fprintf(stderr, "usage: %s SESSION.csv OUT.jpg\n", argv[0]);
    return 2;
  }
  FILE *input = fopen(argv[1], "r");
  if (input == NULL) {
    perror(argv[1]);
    return 1;
  }
  size_t count = 0;
  char line[512];
  // Stop at the runtime sample bound so hostile/oversized logs cannot grow replay state.
  while (fgets(line, sizeof(line), input) != NULL && count < EAR_MAX_SAMPLES) {
    if (line[0] == '#' || line[0] == 't' || line[0] == '\n') {
      continue;   // comment, header or blank
    }
    if (parse_line(line, &samples[count]) == 0) {
      count++;
    }
  }
  fclose(input);

  struct ear_loudest_spot spot;
  memset(&spot, 0, sizeof(spot));
  if (calculated_loudestspot(samples, count, &spot) != 0 || !spot.valid) {
    fprintf(stderr, "%s: no valid loudest spot from %zu samples\n", argv[1], count);
    return 1;
  }
  printf("samples=%zu used=%u spot lat=%.7f lon=%.7f level=%.2f dB conf=%.3f\n",
         count, spot.used_count, spot.lat_deg, spot.lon_deg, spot.level_db, spot.confidence);
  int status = ear_heatmap_write(argv[2], samples, count, &spot);
  if (status == 0) {
    // Keep the overlay input clean; blending the decorated markers twice obscures the field.
    char field[512];
    size_t n = strlen(argv[2]);
    size_t stem = (n > 4 && strcmp(argv[2] + n - 4, ".jpg") == 0) ? n - 4 : n;
    if (stem + 11 < sizeof(field)) {
      for (size_t i = 0; i < stem; i++) {
        field[i] = argv[2][i];
      }
      field[stem] = '\0';
      strcat(field, "_field.jpg");
      if (ear_heatmap_write_field(field, samples, count, &spot) == 0) {
        printf("wrote %s\n", field);
      }
    }
  }
  if (status != 0) {
    fprintf(stderr, "failed to write %s\n", argv[2]);
    return 1;
  }
  printf("wrote %s (+ .geo sidecar)\n", argv[2]);
  return 0;
}
