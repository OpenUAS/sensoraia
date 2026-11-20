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
 * @file camera_profile.c
 * @brief Parse the bounded, flat PANGEA camera calibration format.
 * @details Require all fields, finite geometry values, and a stable regular-file
 * read before publishing the parsed record. A valid profile can still have an
 * unknown USHER camera ID; the consumer checks identity and geometry for its use.
 */
#define _POSIX_C_SOURCE 200809L
#include "camera_profile.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define CAMERA_PROFILE_FIELD_COUNT 11

/** @brief Trim surrounding whitespace in a mutable field.
 * @param[in,out] text Null-terminated input.
 * @return Borrowed pointer to the start of the trimmed field.
 * @details Advance past leading whitespace and terminate in place to keep
 * parsing within the original file buffer. */
static char *trim(char *text)
{
  while (isspace((unsigned char)*text)) ++text;
  char *end = text + strlen(text);
  while (end > text && isspace((unsigned char)end[-1])) --end;
  *end = '\0';
  return text;
}

/** @brief Parse an exact unsigned decimal field.
 * @param text Null-terminated decimal digits without sign or suffix.
 * @param[out] number Result written only on success.
 * @return True for a nonempty value representable as uint64_t.
 * @details Check headroom before multiplying so an overflowing value
 * cannot wrap into a plausible raster dimension. */
static bool parse_uint(const char *text, uint64_t *number)
{
  if (!text || !*text || !number) return false;
  uint64_t value = 0;
  for (const unsigned char *digit = (const unsigned char *)text; *digit; ++digit) {
    if (*digit < '0' || *digit > '9') return false;
    const unsigned int next = *digit - '0';
    if (value > (UINT64_MAX - next) / 10) return false;
    value = value * 10 + next;
  }
  *number = value;
  return true;
}

/** @brief Parse a finite camera geometry value.
 * @param text Null-terminated numeric field.
 * @param[out] number Result written only on success.
 * @return True if the full field converted without a range error.
 * @details Reject non-finite values before they can propagate into projection. */
static bool parse_real(const char *text, double *number)
{
  if (!text || !*text || !number) return false;
  errno = 0;
  char *end = NULL;
  const double value = strtod(text, &end);
  if (errno == ERANGE || end == text || *end || !isfinite(value)) return false;
  *number = value;
  return true;
}

/** @brief Read one bounded, text-only snapshot of a profile.
 * @param filename Path to the profile.
 * @param[out] buffer Writable storage for the text and terminator.
 * @param capacity Buffer size, greater than the profile size limit.
 * @param[out] length Bytes of text before the terminator; use only on success.
 * @return True for a stable, nonempty regular-file read and successful close.
 * @details Open nonblocking so a non-regular input does not stall the owner.
 * Compare size and modification time after reading to detect ordinary
 * concurrent changes; this is not a content integrity check. */
static bool read_file(const char *filename, char *buffer, size_t capacity,
                      size_t *length)
{
  if (!filename || !*filename || !buffer || !length ||
      capacity <= CAMERA_PROFILE_MAX_BYTES) return false;
  const int descriptor = open(filename, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) return false;
  struct stat before, after;
  size_t used = 0;
  bool valid = false;
  if (fstat(descriptor, &before) || !S_ISREG(before.st_mode) ||
      before.st_size <= 0 || (uint64_t)before.st_size > CAMERA_PROFILE_MAX_BYTES)
    goto done;
  while (used < capacity) {
    const ssize_t count = read(descriptor, buffer + used, capacity - used);
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) goto done;
    if (count == 0) break;
    used += (size_t)count;
  }
  if (!used || used > CAMERA_PROFILE_MAX_BYTES || used != (size_t)before.st_size ||
      fstat(descriptor, &after) || after.st_size != before.st_size ||
      after.st_mtim.tv_sec != before.st_mtim.tv_sec ||
      after.st_mtim.tv_nsec != before.st_mtim.tv_nsec ||
      memchr(buffer, '\0', used)) goto done;
  buffer[used] = '\0';
  *length = used;
  valid = true;
done:
  if (close(descriptor)) valid = false;
  return valid;
}

bool usher_camera_profile_read(const char *filename,
                               struct usher_camera_profile *profile)
{
  static const char *const names[CAMERA_PROFILE_FIELD_COUNT] = {
    "schema", "filename_prefix", "width", "height", "fx", "fy", "cx", "cy",
    "mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg"
  };
  char buffer[CAMERA_PROFILE_MAX_BYTES + 1];
  size_t length = 0;
  if (!profile || !read_file(filename, buffer, sizeof(buffer), &length)) return false;

  uint32_t seen = 0;
  char prefix[65] = {0};
  uint64_t width = 0, height = 0;
  double values[CAMERA_PROFILE_FIELD_COUNT] = {0};
  char *line = buffer;
  char *limit = buffer + length;
  while (line < limit) {
    char *next = memchr(line, '\n', (size_t)(limit - line));
    if (next) *next++ = '\0';
    else next = limit;
    char *entry = trim(line);
    line = next;
    if (!*entry || *entry == '#') continue;
    char *separator = strchr(entry, '=');
    if (!separator) return false;
    *separator++ = '\0';
    const char *key = trim(entry);
    const char *value = trim(separator);
    size_t field = 0;
    while (field < CAMERA_PROFILE_FIELD_COUNT && strcmp(key, names[field]) != 0)
      ++field;
    if (field == CAMERA_PROFILE_FIELD_COUNT || (seen & (UINT32_C(1) << field)))
      return false;
    seen |= UINT32_C(1) << field;
    if (field == 0) {
      if (strcmp(value, "PANGEA_CAMERA_V1") != 0) return false;
    } else if (field == 1) {
      const size_t prefix_length = strlen(value);
      if (prefix_length == 0 || prefix_length > 64) return false;
      for (size_t index = 0; index < prefix_length; ++index) {
        const unsigned char character = (unsigned char)value[index];
        if (!((character >= 'A' && character <= 'Z') ||
              (character >= 'a' && character <= 'z') ||
              (character >= '0' && character <= '9') || character == '_' ||
              character == '-')) return false;
      }
      memcpy(prefix, value, prefix_length + 1);
    } else if (field == 2 || field == 3) {
      uint64_t parsed;
      if (!parse_uint(value, &parsed) || parsed < 2 || parsed > 4096)
        return false;
      if (field == 2) width = parsed;
      else height = parsed;
    } else if (!parse_real(value, &values[field])) {
      return false;
    }
  }

  const uint32_t required = (UINT32_C(1) << CAMERA_PROFILE_FIELD_COUNT) - 1;
  const unsigned camera_id = prefix[1] != '\0' ? 0 :
                             prefix[0] == 'c' ? 1 : prefix[0] == 'a' ? 2 :
                             prefix[0] == 'l' ? 3 : 0;
  if (seen != required || !prefix[0] || !width || !height) return false;
  struct usher_camera_profile result = {
    .calibration = {
      .width = (unsigned)width, .height = (unsigned)height,
      .fx = values[4], .fy = values[5], .cx = values[6], .cy = values[7],
      .mount_roll_deg = values[8], .mount_pitch_deg = values[9], .mount_yaw_deg = values[10]
    },
    .camera_id = camera_id,
    .filename_prefix = {0}
  };
  memcpy(result.filename_prefix, prefix, sizeof(prefix));
  *profile = result;
  return true;
}
