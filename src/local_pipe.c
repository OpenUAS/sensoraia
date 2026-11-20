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
 * @file local_pipe.c
 * @brief Hardware-free camera backend for fixture copies and cached map views.
 * @details Use the requested camera identity for naming while keeping simulation
 * outputs distinct from hardware captures. Map rendering delegates projection and
 * publication to the shared image workspace. Fixture mode copies an existing image
 * without recoding it. */
#include "local_pipe.h"
#include "image_mock_transform.h"
#include "path_utils.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef USHER_LOCAL_PHOTO_DIR
  #define USHER_LOCAL_PHOTO_DIR "~/Pictures/sensoraia"
#endif

#ifndef USHER_MOCK_IMAGE
  #define USHER_MOCK_IMAGE "resources/mock_image_01.jpg"
#endif

#ifndef PATH_MAX
  #define PATH_MAX 4096
#endif

static char source_image[PATH_MAX];
static char test_photo_directory[PATH_MAX];
static char photo_directory[PATH_MAX];

static int prepare_photo_directory(void);
static int resolve_photo_directory(void);
static int find_test_photo_directory(char *directory, size_t directory_size);
static int select_random_test_photo(char *path, size_t path_size);
static bool is_readable_jpg(DIR *directory, const struct dirent *entry);
static int random_index(size_t count, size_t *index);

int local_pipe_init(const char *mock_image)
{
  const size_t source_length = mock_image ? strnlen(mock_image, sizeof(source_image)) : 0;
  if (!mock_image || source_length >= sizeof(source_image)) {
    errno = mock_image ? ENAMETOOLONG : EINVAL;
    return -1;
  }
  if (mock_image == NULL || access(mock_image, R_OK) != 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to access mock image %s: %s\n",
            mock_image != NULL ? mock_image : "(null)", strerror(errno));
    return -1;
  }

  if (prepare_photo_directory() != 0) {
    return -1;
  }

  for (size_t index = 0; index <= source_length; ++index) source_image[index] = mock_image[index];
  test_photo_directory[0] = '\0';
  printf("LOCAL_PIPE:\tphoto directory: %s\n", photo_directory);
  return 0;
}

int local_pipe_test_init(const char *mock_image)
{
  if (mock_image != NULL) {
    return local_pipe_init(mock_image);
  }

  if (prepare_photo_directory() != 0) {
    return -1;
  }

  int photo_count = find_test_photo_directory(test_photo_directory, sizeof(test_photo_directory));
  if (photo_count < 0) {
    return -1;
  }
  if (photo_count == 0) {
    fprintf(stderr, "LOCAL_PIPE:\tno readable .jpg files found beside the executable; using %s\n",
            USHER_MOCK_IMAGE);
    return local_pipe_init(USHER_MOCK_IMAGE);
  }

  source_image[0] = '\0';
  printf("LOCAL_PIPE:\ttest photo directory: %s (%d image%s)\n",
         test_photo_directory, photo_count, photo_count == 1 ? "" : "s");
  printf("LOCAL_PIPE:\tphoto directory: %s\n", photo_directory);
  return 0;
}

/** @brief Prepare usable output storage before accepting local shots.
 * @return Zero for a writable directory, or -1 on resolution or storage failure.
 * @details Confirm directory type after creation; an existing regular file
 * at the configured path is not a usable output location. */
static int prepare_photo_directory(void)
{
  if (resolve_photo_directory() != 0) {
    return -1;
  }
  if (usher_ensure_directory(photo_directory) != 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to create photo directory %s: %s\n",
            photo_directory, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(photo_directory, &directory_status) != 0 || !S_ISDIR(directory_status.st_mode)
      || access(photo_directory, W_OK) != 0) {
    fprintf(stderr, "LOCAL_PIPE:\tphoto directory is not writable: %s\n", photo_directory);
    return -1;
  }
  return 0;
}

/** @brief Resolve the configured local output path into backend storage.
 * @return Zero if the path fits, or -1 on invalid configuration or path lookup failure.
 * @details Anchor relative paths beside the executable rather than the
 * caller's working directory so service and interactive launches agree. */
static int resolve_photo_directory(void)
{
  char resolved_dir[PATH_MAX];
  const char *configured = usher_resolve_path(USHER_LOCAL_PHOTO_DIR, resolved_dir, sizeof(resolved_dir));
  if (configured == NULL || configured[0] == '\0') {
    return -1;
  }
  if (configured[0] == '/') {
    int length = snprintf(photo_directory, sizeof(photo_directory), "%s", configured);
    return length > 0 && (size_t)length < sizeof(photo_directory) ? 0 : -1;
  }

  ssize_t length = readlink("/proc/self/exe", photo_directory, sizeof(photo_directory) - 1);
  if (length < 0 || (size_t)length >= sizeof(photo_directory) - 1) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to locate USHER executable: %s\n", strerror(errno));
    return -1;
  }
  photo_directory[length] = '\0';
  char *separator = strrchr(photo_directory, '/');
  if (separator == NULL) {
    return -1;
  }
  *separator = '\0';
  size_t directory_length = (size_t)(separator - photo_directory);
  int path_length = snprintf(photo_directory + directory_length,
                             sizeof(photo_directory) - directory_length, "/%s", configured);
  return path_length > 0 && (size_t)path_length < sizeof(photo_directory) - directory_length ? 0 : -1;
}

int local_pipe_map_init(const char *source)
{
  (void)source;
  if (prepare_photo_directory() != 0) return -1;
  printf("LOCAL_PIPE:\tmap mock photo directory: %s\n", photo_directory);
  return 0;
}

int local_pipe_map_shoot(char *filename, size_t filename_size, const union dc_shot_union *shot,
                         const struct camera_backend *identity, const char *cache_directory,
                         double horizontal_fov_deg, double vertical_fov_deg)
{
  if (filename == NULL || filename_size == 0 || shot == NULL || photo_directory[0] == '\0'
      || !identity || !identity->prefix || !identity->mock.width || !identity->mock.height) return -1;
  const char *marker = identity->mock.filename_marker ? identity->mock.filename_marker : "m";
  int length = snprintf(filename, filename_size, "%s/%s%c%06d.jpg", photo_directory,
                        marker, identity->prefix, shot->data.nr);
  if (length < 0 || (size_t)length >= filename_size) return -1;
  return image_mock_map(filename, cache_directory, shot, identity->mock.width, identity->mock.height,
                        horizontal_fov_deg, vertical_fov_deg);
}

int local_pipe_shoot(char *filename, size_t filename_size, int image_number, const struct camera_backend *identity)
{
  if ((source_image[0] == '\0' && test_photo_directory[0] == '\0')
      || filename == NULL || filename_size == 0
      || identity == NULL || identity->prefix == '\0') {
    return -1;
  }

  char selected_source[PATH_MAX];
  const char *current_source = source_image;
  if (test_photo_directory[0] != '\0') {
    if (select_random_test_photo(selected_source, sizeof(selected_source)) != 0) {
      return -1;
    }
    current_source = selected_source;
    printf("LOCAL_PIPE:\tselected test image: %s\n", current_source);
  }

  int length = snprintf(filename, filename_size, "%s/%s%c%06d.jpg", photo_directory,
                        identity->mock.filename_marker ? identity->mock.filename_marker : "m",
                        identity->prefix, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    return -1;
  }

  int source_fd = open(current_source, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (source_fd < 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to open mock image %s: %s\n", current_source, strerror(errno));
    return -1;
  }

  struct stat source_status;
  if (fstat(source_fd, &source_status) != 0 || !S_ISREG(source_status.st_mode)
      || source_status.st_size <= 0) {
    close(source_fd);
    errno = EINVAL;
    return -1;
  }

  int destination_fd = open(filename, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0644);
  if (destination_fd < 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to create %s: %s\n", filename, strerror(errno));
    const int error = errno;
    close(source_fd);
    errno = error;
    return -1;
  }

  char buffer[4096];
  /* Copy only the initially observed size and compare metadata afterward,
   * so a growing or ordinarily modified fixture is not accepted as a shot. */
  off_t remaining = source_status.st_size;
  int result = 0;
  while (remaining > 0) {
    const size_t requested = remaining > (off_t)sizeof(buffer) ? sizeof(buffer) : (size_t)remaining;
    const ssize_t bytes_read = read(source_fd, buffer, requested);
    if (bytes_read < 0 && errno == EINTR) continue;
    if (bytes_read <= 0) {
      if (bytes_read == 0) errno = EIO;
      result = -1;
      break;
    }
    ssize_t written = 0;
    while (written < bytes_read) {
      ssize_t count = write(destination_fd, &buffer[written], (size_t)(bytes_read - written));
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) {
        if (count == 0) errno = EIO;
        result = -1;
        break;
      }
      written += count;
    }
    if (result != 0) {
      break;
    }
    remaining -= bytes_read;
  }
  struct stat completed;
  if (result == 0 && (fstat(source_fd, &completed) != 0
      || completed.st_size != source_status.st_size
      || completed.st_mtim.tv_sec != source_status.st_mtim.tv_sec
      || completed.st_mtim.tv_nsec != source_status.st_mtim.tv_nsec
      || completed.st_ctim.tv_sec != source_status.st_ctim.tv_sec
      || completed.st_ctim.tv_nsec != source_status.st_ctim.tv_nsec)) {
    errno = EIO;
    result = -1;
  }

  int error = result != 0 ? errno : 0;
  if (close(destination_fd) != 0 && result == 0) {
    error = errno;
    result = -1;
  }
  close(source_fd);
  if (result != 0) {
    unlink(filename);
    errno = error;
  }
  return result;
}

void local_pipe_deinit(void)
{
  source_image[0] = '\0';
  test_photo_directory[0] = '\0';
}

/** @brief Locate and count eligible fixtures beside the executable.
 * @param[out] directory Buffer for the discovered path.
 * @param directory_size Capacity including the terminator.
 * @return Eligible fixture count, zero when absent or empty, or -1 on setup failure.
 * @details Missing fixture storage is a normal case that lets the caller
 * fall back to the bundled image. */
static int find_test_photo_directory(char *directory, size_t directory_size)
{
  char executable[PATH_MAX];
  ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length < 0 || (size_t)length >= sizeof(executable) - 1) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to locate USHER executable: %s\n", strerror(errno));
    return -1;
  }
  executable[length] = '\0';

  char *separator = strrchr(executable, '/');
  if (separator == NULL) {
    fprintf(stderr, "LOCAL_PIPE:\tUSHER executable path has no directory: %s\n", executable);
    return -1;
  }
  if (separator == executable) {
    separator[1] = '\0';
  } else {
    *separator = '\0';
  }

  int path_length = snprintf(directory, directory_size, "%s%stestphotos",
                             executable, separator == executable ? "" : "/");
  if (path_length < 0 || (size_t)path_length >= directory_size) {
    fprintf(stderr, "LOCAL_PIPE:\ttest photo directory path is too long\n");
    return -1;
  }

  DIR *stream = opendir(directory);
  if (stream == NULL) {
    if (errno != ENOENT && errno != ENOTDIR) {
      fprintf(stderr, "LOCAL_PIPE:\tfailed to open test photo directory %s: %s\n",
              directory, strerror(errno));
      return -1;
    }
    directory[0] = '\0';
    return 0;
  }

  int count = 0;
  struct dirent *entry;
  while ((entry = readdir(stream)) != NULL) {
    if (is_readable_jpg(stream, entry)) {
      count++;
    }
  }
  closedir(stream);
  return count;
}

/** @brief Select a fixture from the current test photo directory.
 * @param[out] path Buffer for the selected pathname.
 * @param path_size Capacity including the terminator.
 * @return Zero when a fixture path fits, or -1 when none can be selected.
 * @details Count entries, choose an index, then rescan. This needs no
 * directory-sized allocation, at the cost of a second scan that may see changes. */
static int select_random_test_photo(char *path, size_t path_size)
{
  DIR *stream = opendir(test_photo_directory);
  if (stream == NULL) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to open test photo directory %s: %s\n",
            test_photo_directory, strerror(errno));
    return -1;
  }

  size_t count = 0;
  struct dirent *entry;
  while ((entry = readdir(stream)) != NULL) {
    if (is_readable_jpg(stream, entry)) {
      count++;
    }
  }
  if (count == 0) {
    fprintf(stderr, "LOCAL_PIPE:\ttest photo directory contains no readable .jpg files\n");
    closedir(stream);
    return -1;
  }

  size_t selected_index;
  if (random_index(count, &selected_index) != 0) {
    closedir(stream);
    return -1;
  }

  rewinddir(stream);
  size_t current_index = 0;
  while ((entry = readdir(stream)) != NULL) {
    if (!is_readable_jpg(stream, entry)) {
      continue;
    }
    if (current_index == selected_index) {
      int length = snprintf(path, path_size, "%s/%s", test_photo_directory, entry->d_name);
      closedir(stream);
      if (length < 0 || (size_t)length >= path_size) {
        fprintf(stderr, "LOCAL_PIPE:\tselected test image path is too long\n");
        return -1;
      }
      return 0;
    }
    current_index++;
  }

  closedir(stream);
  fprintf(stderr, "LOCAL_PIPE:\ttest photo directory changed during image selection\n");
  return -1;
}

/** @brief Test whether a directory entry is an eligible JPEG fixture.
 * @param directory Open stream containing the entry.
 * @param entry Current directory entry.
 * @return True for a readable regular file with a case-insensitive .jpg suffix.
 * @details Check metadata relative to the open directory without building
 * a pathname per candidate. This is eligibility only, not JPEG validation. */
static bool is_readable_jpg(DIR *directory, const struct dirent *entry)
{
  size_t name_length = strlen(entry->d_name);
  if (name_length < 4 || strcasecmp(&entry->d_name[name_length - 4], ".jpg") != 0) {
    return false;
  }

  struct stat status;
    const int descriptor = dirfd(directory);
    if (descriptor < 0) return false;
    return fstatat(descriptor, entry->d_name, &status, 0) == 0
         && S_ISREG(status.st_mode)
      && faccessat(descriptor, entry->d_name, R_OK, 0) == 0;
}

/** @brief Select an index for fixture playback.
 * @param count Nonzero number of eligible entries.
 * @param[out] index Selected index in [0, count).
 * @return Zero on success, or -1 if the random word cannot be read.
 * @details Use a fixed-size random word and modulo reduction. The slight
 * modulo bias is acceptable for fixture variety, not for statistical sampling. */
static int random_index(size_t count, size_t *index)
{
  int random_fd = open("/dev/urandom", O_RDONLY);
  if (random_fd < 0) {
    fprintf(stderr, "LOCAL_PIPE:\tfailed to open /dev/urandom: %s\n", strerror(errno));
    return -1;
  }

  uint64_t random_value;
  unsigned char *bytes = (unsigned char *)&random_value;
  size_t bytes_read = 0;
  while (bytes_read < sizeof(random_value)) {
    ssize_t count_read = read(random_fd, &bytes[bytes_read], sizeof(random_value) - bytes_read);
    if (count_read < 0 && errno == EINTR) {
      continue;
    }
    if (count_read <= 0) {
      fprintf(stderr, "LOCAL_PIPE:\tfailed to read /dev/urandom: %s\n", strerror(errno));
      close(random_fd);
      return -1;
    }
    bytes_read += (size_t)count_read;
  }
  close(random_fd);

  *index = (size_t)(random_value % (uint64_t)count);
  return 0;
}