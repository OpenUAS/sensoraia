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

/** @file aicam.cpp
 * @brief Command-line entry point for the native AIcam photo worker.
 * @details Delegate camera ownership and the capture protocol to the shared
 * IMX500 implementation. This entry point does not interpret neural outputs. */
#include <cstdio>
#include <cstring>

#include "imx500_camera.hpp"
#include "version.h"

/** @brief Report CLI metadata or run the photo worker.
 * @param argc Argument count including the program name.
 * @param argv Command-line vector; a leading photo-only marker is consumed here.
 * @return Zero for help or version, otherwise the worker's exit status.
 * @details Handle metadata requests before entering the worker so they do not
 * require camera access. */
int main(int argc, char **argv)
{
  if (argc == 2 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h"))) {
    std::puts(AICAM_BUILD_VERSION);
    std::puts("Usage: aicam --probe | --capture /absolute/photo.jpg | --serve WIDTH HEIGHT | --version");
    return 0;
  }
  if (argc == 2 && (!std::strcmp(argv[1], "--version") || !std::strcmp(argv[1], "-V"))) {
    std::puts(AICAM_BUILD_VERSION);
    return 0;
  }
  if (argc > 1 && !std::strcmp(argv[1], "--photo-only")) {
    return imx500::run_photo_worker(argc - 1, argv + 1, "aicam");
  }
  return imx500::run_photo_worker(argc, argv, "aicam");
}