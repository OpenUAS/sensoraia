/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 */
#include "image_lens.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef IMAGE_LENS_TIMEOUT_MS
#define IMAGE_LENS_TIMEOUT_MS 90000
#endif

static bool monotonic_ms(uint64_t *value)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || now.tv_sec < 0) return false;
  *value = (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
  return true;
}

bool image_lens_emulate(const char *application, const char *input,
                        const char *profile, const char *cache,
                        char *output, size_t capacity,
                        const volatile sig_atomic_t *running)
{
  if (!application || !input || !profile || !output || !capacity ||
      !application[0] || !input[0] || !profile[0]) {
    fprintf(stderr, "IMAGE_LENS: invalid renderer request\n");
    return false;
  }
  output[0] = '\0';
  const char *extension = strrchr(input, '.');
  const char *separator = strrchr(input, '/');
  const size_t stem = extension && (!separator || extension > separator)
                      ? (size_t)(extension - input) : strlen(input);
  const char suffix[] = ".lens.jpg";
  if (stem >= capacity || sizeof(suffix) > capacity - stem) {
    fprintf(stderr, "IMAGE_LENS: derivative path too long\n");
    return false;
  }
  uint64_t started;
  if (!monotonic_ms(&started) || (running && !*running)) {
    fprintf(stderr, "IMAGE_LENS: clock unavailable or capture cancelled\n");
    return false;
  }
  for (size_t index = 0; index < stem; ++index) output[index] = input[index];
  for (size_t index = 0; index < sizeof(suffix); ++index) output[stem + index] = suffix[index];
  char *arguments[9] = {(char *)application, (char *)"--profile", (char *)profile};
  size_t count = 3;
  if (cache) {
    arguments[count++] = (char *)"--cache";
    arguments[count++] = (char *)cache;
  }
  arguments[count++] = (char *)input;
  arguments[count++] = output;
  arguments[count] = NULL;
  struct sigaction defaults = {.sa_handler = SIG_DFL};
  sigemptyset(&defaults.sa_mask);
  sigset_t mask;
  sigemptyset(&mask);
  const pid_t parent = getpid();
  const pid_t child = fork();
  if (child == 0) {
    /* Only Linux/POSIX system calls after fork: this also runs in USHER's
     * multithreaded capture worker. Never leave a renderer behind its owner. */
    if (sigaction(SIGTERM, &defaults, NULL) != 0 || sigaction(SIGINT, &defaults, NULL) != 0 ||
        sigprocmask(SIG_SETMASK, &mask, NULL) != 0 ||
        prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || getppid() != parent) _exit(1);
    execv(application, arguments);
    _exit(127);
  }
  if (child < 0) {
    fprintf(stderr, "IMAGE_LENS: cannot start %s: %s\n", application, strerror(errno));
    output[0] = '\0';
    return false;
  }
  int status = 0;
  for (;;) {
    const pid_t result = waitpid(child, &status, WNOHANG);
    if (result == child) break;
    if (result < 0 && errno != EINTR) {
      fprintf(stderr, "IMAGE_LENS: cannot wait for renderer: %s\n", strerror(errno));
      output[0] = '\0';
      return false;
    }
    uint64_t now;
    if (!monotonic_ms(&now) || now < started || now - started >= IMAGE_LENS_TIMEOUT_MS ||
        (running && !*running)) {
      fprintf(stderr, "IMAGE_LENS: renderer deadline/cancellation; analysis skipped\n");
      kill(child, SIGKILL);
      while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
      output[0] = '\0';
      return false;
    }
    const struct timespec delay = {.tv_nsec = 10000000L};
    nanosleep(&delay, NULL);
  }
  struct stat saved;
  if ((running && !*running) || !WIFEXITED(status) || WEXITSTATUS(status) != 0 ||
      stat(output, &saved) != 0 || !S_ISREG(saved.st_mode) || saved.st_size <= 0) {
    fprintf(stderr, "IMAGE_LENS: renderer failed or derivative unavailable; analysis skipped\n");
    output[0] = '\0';
    return false;
  }
  return true;
}
