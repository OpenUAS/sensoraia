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

#ifndef USHER_PATH_UTILS_H
#define USHER_PATH_UTILS_H

/**
 * @file path_utils.h
 * @brief Small path and writable-directory helpers shared by USHER backends.
 * @details Paths are intentionally resolved only for a leading `~`: USHER configuration
 * is not a shell and must not expand arbitrary variables or execute shell syntax.
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef PATH_MAX
  #define PATH_MAX 4096
#endif

/** @brief Change to the directory containing the running USHER executable.
 * @return 0 on success, otherwise -1 with errno set.
 * @details Project resources and sibling workers use relative paths, so anchor
 * them to the installed executable rather than the caller's working directory. */
static inline int usher_enter_executable_directory(void)
{
  char executable[PATH_MAX];
  ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length <= 0 || (size_t)length >= sizeof(executable) - 1) {
    return -1;
  }
  executable[length] = '\0';
  char *separator = strrchr(executable, '/');
  if (separator == NULL) {
    errno = ENOENT;
    return -1;
  }
  if (separator == executable) {
    separator[1] = '\0';
  } else {
    *separator = '\0';
  }
  return chdir(executable);
}

/** @brief Expand a leading home-directory shorthand without allocating memory.
 * @param path Configured path, possibly beginning with `~` or `~/`.
 * @param buffer Caller-owned expansion storage.
 * @param buffer_size Capacity of @p buffer.
 * @return @p buffer for a successful expansion, @p path when no expansion is needed,
 * or NULL when @p path is NULL.
 * @details Returning the original pointer avoids copying the common literal-path case. */
static inline const char *usher_resolve_path(const char *path, char *buffer, size_t buffer_size)
{
  if (path == NULL) {
    return NULL;
  }
  if (path[0] == '~' && (path[1] == '/' || path[1] == '\0')) {
    const char *home = getenv("HOME");
    if (home != NULL && home[0] != '\0') {
      int length = snprintf(buffer, buffer_size, "%s%s", home, path + 1);
      if (length > 0 && (size_t)length < buffer_size) {
        return buffer;
      }
    }
  }
  return path;
}

/** @brief Create missing directory components and verify the final directory is writable.
 * @param path Directory path, with optional leading home shorthand.
 * @return 0 for a writable directory, otherwise -1.
 * @details Components are created one at a time to avoid invoking a shell. Existing
 * parents are harmless, while final stat/access checks distinguish a file or read-only
 * path from a usable capture destination. */
static inline int usher_ensure_directory(const char *path)
{
  if (path == NULL || path[0] == '\0') {
    return -1;
  }
  char resolved[PATH_MAX];
  const char *target = usher_resolve_path(path, resolved, sizeof(resolved));
  if (target == NULL) {
    return -1;
  }

  char temp[PATH_MAX];
  size_t length = 0;
  while (target[length] != '\0' && length < sizeof(temp) - 1) {
    temp[length] = target[length];
    length++;
  }
  if (target[length] != '\0') {
    return -1;
  }
  temp[length] = '\0';

  for (size_t index = 1; index <= length; index++) {
    if (temp[index] == '/' || temp[index] == '\0') {
      char character = temp[index];
      temp[index] = '\0';
      if (temp[0] != '\0' && mkdir(temp, 0755) != 0 && errno != EEXIST) {
        // Parent directory attempt - ignore if it already exists
      }
      temp[index] = character;
    }
  }

  struct stat status;
  if (stat(target, &status) != 0 || !S_ISDIR(status.st_mode) || access(target, W_OK) != 0) {
    return -1;
  }
  return 0;
}

/** @brief Redirect an intended public photo into its same-filesystem private staging directory.
 * @param[in,out] filename Bounded public path, replaced with a sibling .pending path on success.
 * @param capacity Writable capacity of @p filename, including its terminator.
 * @return 0 on success, otherwise -1 with errno set; no existing photo is overwritten.
 * @details The public parent must exist. Staging must be owned by this user with mode 0700.
 * A failed call may change the path buffer, but never creates a photo. */
static inline int usher_prepare_photo(char *filename, size_t capacity)
{
  if (!filename || !capacity) { errno = EINVAL; return -1; }
  const size_t length = strnlen(filename, capacity);
  const char *separator = length < capacity ? strrchr(filename, '/') : NULL;
  const char staging[] = ".pending/";
  const size_t extra = sizeof(staging) - 1;
  if (!separator || !separator[1] || length >= PATH_MAX - extra || length + extra >= capacity) {
    errno = ENAMETOOLONG; return -1;
  }
  struct stat status;
  if (lstat(filename, &status) == 0 || errno != ENOENT) { errno = EEXIST; return -1; }
  const size_t parent_length = (size_t)(separator - filename);
  char directory[PATH_MAX];
  const int count = snprintf(directory, sizeof(directory), "%.*s/.pending", (int)parent_length, filename);
  if (count < 0 || (size_t)count >= sizeof(directory)) { errno = ENAMETOOLONG; return -1; }
  if ((mkdir(directory, 0700) != 0 && errno != EEXIST) || lstat(directory, &status) != 0) return -1;
  if (!S_ISDIR(status.st_mode) || status.st_uid != geteuid() || (status.st_mode & 077)) { errno = EACCES; return -1; }
  for (size_t index = length + 1; index > parent_length + 1; --index)
    filename[index - 1 + extra] = filename[index - 1];
  for (size_t index = 0; index < extra; ++index) filename[parent_length + 1 + index] = staging[index];
  if (lstat(filename, &status) == 0 || errno != ENOENT) { errno = EEXIST; return -1; }
  return 0;
}

/** @brief Publish completed immutable bytes atomically without copying or replacing an existing file.
 * @param source Exclusively owned, nonempty regular file with no other hardlinks.
 * @param destination Public path on the same filesystem.
 * @return 0 for durable publication and private removal; 1 if published but directory sync or
 * private removal failed; -1 if not published, with errno describing the failure.
 * @details Call only after every metadata write. Readers may open the final name immediately.
 * A positive result must never trigger a capture retry or another write to either alias.
 * Crash recovery may retain a private alias; callers must not modify published originals. */
static inline int usher_publish_photo(const char *source, const char *destination)
{
  if (!source || !destination || !source[0] || !destination[0]) { errno = EINVAL; return -1; }
  char parent[PATH_MAX];
  const char *basename = strrchr(destination, '/');
  const size_t length = basename ? (size_t)(basename - destination) : 0;
  if (length >= sizeof(parent)) { errno = ENAMETOOLONG; return -1; }
  if (basename) {
    for (size_t index = 0; index < length; ++index) parent[index] = destination[index];
    parent[length ? length : 1] = '\0';
    if (!length) parent[0] = '/';
    ++basename;
  } else {
    parent[0] = '.'; parent[1] = '\0'; basename = destination;
  }
  if (!basename[0]) { errno = EINVAL; return -1; }
  int directory = open(parent, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  if (directory < 0) return -1;
  int input = open(source, O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
  struct stat status;
  int result = -1;
  if (input >= 0 && fstat(input, &status) == 0) {
    if (!S_ISREG(status.st_mode) || status.st_size <= 0 || status.st_nlink != 1) errno = EINVAL;
    else if (fsync(input) == 0 && linkat(AT_FDCWD, source, directory, basename, 0) == 0) {
      result = 1;
      if (fsync(directory) == 0 && unlink(source) == 0) result = 0;
    }
  }
  const int saved_errno = errno;
  if (input >= 0) close(input);
  close(directory);
  errno = saved_errno;
  return result;
}

/** @brief Publish a staged photo and return its immutable public path.
 * @param[in,out] filename Path produced by usher_prepare_photo; public path on nonnegative return.
 * @param capacity Writable capacity of @p filename, including its terminator.
 * @return usher_publish_photo status; on failure the private path remains unchanged.
 * @details Only the immediate .pending parent is removed. No metadata may change after success. */
static inline int usher_complete_photo(char *filename, size_t capacity)
{
  if (!filename || !capacity) { errno = EINVAL; return -1; }
  const size_t length = strnlen(filename, capacity);
  const char *basename = length < capacity ? strrchr(filename, '/') : NULL;
  const char staging[] = "/.pending";
  const size_t extra = sizeof(staging) - 1;
  if (!basename || (size_t)(basename - filename) < extra || length >= PATH_MAX
      || strncmp(basename - extra, staging, extra)) { errno = EINVAL; return -1; }
  char destination[PATH_MAX] = {0};
  const size_t parent_length = (size_t)(basename - filename) - extra;
  for (size_t index = 0; index < parent_length; ++index) destination[index] = filename[index];
  for (size_t index = parent_length; index <= length - extra; ++index) destination[index] = filename[index + extra];
  const int result = usher_publish_photo(filename, destination);
  if (result >= 0) for (size_t index = 0; index <= length - extra; ++index) filename[index] = destination[index];
  return result;
}

#endif // USHER_PATH_UTILS_H
