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

/** @file bounded_ini.h
 * @brief Allocation-free flat key=value mechanics; callers own schemas and storage. */
#ifndef USHER_BOUNDED_INI_H
#define USHER_BOUNDED_INI_H

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

typedef bool (*ini_field_handler)(void *context, const char *key, const char *value);

/** @brief Trim surrounding whitespace in a mutable INI field.
 * @param[in,out] text Null-terminated field storage.
 * @return Borrowed pointer into @p text.
 * @details Terminate the field in place so the parser need not allocate
 * storage for individual keys or values. */
static inline char *ini_trim(char *text)
{
  while (isspace((unsigned char)*text)) ++text;
  char *end = text + strlen(text);
  while (end > text && isspace((unsigned char)end[-1])) --end;
  *end = '\0';
  return text;
}

/** @brief Parse digits-only uint64; overflow and signs are rejected. */
static inline bool ini_uint(const char *text, uint64_t *number)
{
  if (!text || !*text || !number) return false;
  uint64_t value = 0;
  for (const unsigned char *digit = (const unsigned char *)text; *digit; ++digit) {
    if (*digit < '0' || *digit > '9') return false;
    const unsigned next = *digit - '0';
    if (value > (UINT64_MAX - next) / 10) return false;
    value = value * 10 + next;
  }
  *number = value;
  return true;
}

/** @brief Parse finite decimal/exponent notation, rejecting range errors and suffixes. */
static inline bool ini_real(const char *text, double *number)
{
  if (!text || !*text || !number) return false;
  const char *cursor = text;
  if (*cursor == '+' || *cursor == '-') ++cursor;
  bool digit = false;
  while (*cursor >= '0' && *cursor <= '9') { digit = true; ++cursor; }
  if (*cursor == '.') {
    ++cursor;
    while (*cursor >= '0' && *cursor <= '9') { digit = true; ++cursor; }
  }
  if (!digit) return false;
  if (*cursor == 'e' || *cursor == 'E') {
    ++cursor;
    if (*cursor == '+' || *cursor == '-') ++cursor;
    const char *start = cursor;
    while (*cursor >= '0' && *cursor <= '9') ++cursor;
    if (cursor == start) return false;
  }
  if (*cursor) return false;
  errno = 0;
  char *end = NULL;
  const double value = strtod(text, &end);
  if (errno == ERANGE || *end || !isfinite(value)) return false;
  *number = value;
  return true;
}

/** @brief Read a nonempty regular file smaller than capacity and append NUL.
 * @details Checks descriptor size/mtime across reading. Outputs are usable only
 * on true; callers supply stable paths and validate exposure identity separately. */
static inline bool ini_read_file(const char *filename, char *buffer, size_t capacity, size_t *length)
{
  if (!filename || !*filename || !buffer || capacity < 2 || !length) return false;
  const int descriptor = open(filename, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) return false;
  struct stat before, after;
  size_t used = 0;
  bool valid = false;
  if (fstat(descriptor, &before) || !S_ISREG(before.st_mode) || before.st_size <= 0
      || (uint64_t)before.st_size >= capacity) goto done;
  while (used < capacity) {
    const ssize_t count = read(descriptor, buffer + used, capacity - used);
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) goto done;
    if (!count) break;
    used += (size_t)count;
  }
  if (!used || used >= capacity || used != (size_t)before.st_size
      || fstat(descriptor, &after) || after.st_size != before.st_size
      || after.st_mtim.tv_sec != before.st_mtim.tv_sec
      || after.st_mtim.tv_nsec != before.st_mtim.tv_nsec) goto done;
  buffer[used] = '\0';
  *length = used;
  valid = true;
done:
  if (close(descriptor)) valid = false;
  return valid;
}

/** @brief Destructively parse flat fields with blank lines and #/; comment lines.
 * @pre Buffer has at least length+1 writable bytes; callbacks retain no pointers.
 * @details Rejects embedded NUL, sections and empty keys/values. The handler must
 * reject unknown/duplicate keys and validate completeness after successful parsing.
 * There are no quoting, escaping, multiline or inline-comment rules. */
static inline bool ini_parse(char *buffer, size_t length, ini_field_handler handler, void *context)
{
  if (!buffer || !length || !handler || memchr(buffer, '\0', length)) return false;
  buffer[length] = '\0';
  char *line = buffer;
  char *limit = buffer + length;
  while (line < limit) {
    char *next = (char *)memchr(line, '\n', (size_t)(limit - line));
    if (next) *next++ = '\0';
    else next = limit;
    char *entry = ini_trim(line);
    line = next;
    if (!*entry || *entry == '#' || *entry == ';') continue;
    char *separator = strchr(entry, '=');
    if (!separator) return false;
    *separator++ = '\0';
    const char *key = ini_trim(entry);
    const char *value = ini_trim(separator);
    if (!*key || !*value) return false;
    for (const unsigned char *character = (const unsigned char *)key; *character; ++character) {
      if (!isalnum(*character) && *character != '_' && *character != '.') return false;
    }
    if (!handler(context, key, value)) return false;
  }
  return true;
}

#endif