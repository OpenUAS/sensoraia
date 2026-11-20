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
 * @file soda.cpp
 * @brief Small validated command-line dispatch point for
 * post-capture analysis handlers. */
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "soda_object_postprocess.h"
#include "image_lens.h"
#include "version.h"

namespace {
void print_usage(const char* program) {
  std::cout
      << SODA_BUILD_VERSION << '\n'
      << "Usage: " << program << " [OPTIONS] IMAGE [10 shot integers]\n"
      << "  --local     Development-PC mode; no hardware or remote "
         "connection.\n"
      << "  --aicam     Select the AI camera handler.\n"
      << "  --chdkcam   Select the CHDK camera handler.\n"
      << "  --lwircam   Select the thermal camera handler.\n"
      << "  --earcam    Select the acoustic camera handler.\n"
      << "  --lens-profile FILE  Emulate a lens before AIcam analysis "
         "(requires --local --aicam).\n"
      << "  --lens-cache DIR     Optional zoom-18 XYZ cache (pprzgcs legacy path GOOGLE/18) "
         "coverage.\n"
      << "  --version   Show application name, version, and build Git "
         "revision.\n"
      << "  --help      Show this help.\n"
      << "  --          End option parsing.\n"
      << "Select at most one camera; otherwise use the generic handler.\n"
      << "IMAGE must be readable and nonempty. Optional int32 shot values:\n"
      << "number latitude longitude altitude roll pitch yaw speed course "
         "groundalt\n";
}

void process_aicam() {
  std::cout << "Now I can do nifty stuff for aicam" << std::endl;
}

void process_chdkcam() {
  std::cout << "Now I can do nifty stuff for chdkcam" << std::endl;
}

void process_lwircam() {
  std::cout << "Now I can do nifty stuff for lwircam" << std::endl;
}

void process_earcam() {
  std::cout << "Now I can do nifty stuff for earcam" << std::endl;
}

struct CameraHandler {
  const char* option;
  void (*process)();
};

const CameraHandler handlers[] = {{"--aicam", process_aicam},
                                  {"--chdkcam", process_chdkcam},
                                  {"--lwircam", process_lwircam},
                                  {"--earcam", process_earcam}};

bool is_shot_value(const char* text) {
  const char* digit = text;
  if (*digit == '-' || *digit == '+') ++digit;
  if (*digit == '\0') return false;
  for (; *digit != '\0'; ++digit) {
    if (*digit < '0' || *digit > '9') return false;
  }
  errno = 0;
  char* end = nullptr;
  const long long value = std::strtoll(text, &end, 10);
  return errno != ERANGE && *end == '\0' &&
         value >= std::numeric_limits<int32_t>::min() &&
         value <= std::numeric_limits<int32_t>::max();
}

bool emulate_lens(const char* input, const char* profile, const char* cache,
                  std::string& output) {
  char executable[4096];
  ssize_t length =
      readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(executable) - 1)
    return false;
  executable[length] = '\0';
  std::string directory(executable);
  directory.resize(directory.find_last_of('/'));
  const std::string executable_path = directory + "/camera_lens";
  if (access(executable_path.c_str(), X_OK) != 0) {
    std::cerr << "SODA: native lens executable missing; run make camera-lens "
                 "in the USHER directory\n";
    return false;
  }
  char derivative[4096];
  if (!image_lens_emulate(executable_path.c_str(), input, profile, cache,
                           derivative, sizeof(derivative), nullptr))
    return false;
  output = derivative;
  return true;
}
}  // namespace

/** @brief Select and run the postprocessor for one saved camera image.
 * @param argc Argument count including the program name.
 * @param argv Command-line argument vector.
 * @return Process status for argument handling and the selected processing path.
 * @details Resolve camera and optional lens settings before dispatching;
 * local-camera derivatives are explicitly selected, not inferred from a live shot. */
int main(int argc, char* argv[]) {
  const CameraHandler* selected = nullptr;
  const char* lens_profile = nullptr;
  const char* lens_cache = nullptr;
  std::vector<const char*> positional;
  bool options = true;
  bool local = false;
  bool help = false;
  bool version = false;
  for (int argument = 1; argument < argc; ++argument) {
    const char* value = argv[argument];
    if (options && (std::strcmp(value, "--lens-profile") == 0 ||
                    std::strcmp(value, "--lens-cache") == 0)) {
      const char** destination = std::strcmp(value, "--lens-profile") == 0
                                     ? &lens_profile
                                     : &lens_cache;
      if (*destination != nullptr || argument + 1 >= argc ||
          argv[argument + 1][0] == '\0') {
        std::cerr << "SODA: missing or duplicate lens option\n";
        return 2;
      }
      *destination = argv[++argument];
      continue;
    }
    if (options && std::strcmp(value, "--") == 0) {
      options = false;
      continue;
    }
    if (options && std::strcmp(value, "--local") == 0) {
      if (local) {
        std::cerr << "SODA: duplicate --local option" << std::endl;
        return 2;
      }
      local = true;
      continue;
    }
    if (options && std::strcmp(value, "--help") == 0) {
      help = true;
      continue;
    }
    if (options && std::strcmp(value, "--version") == 0) {
      version = true;
      continue;
    }
    const CameraHandler* match = nullptr;
    if (options) {
      for (const CameraHandler& handler : handlers) {
        if (std::strcmp(value, handler.option) == 0) match = &handler;
      }
    }
    if (match != nullptr) {
      if (selected != nullptr) {
        std::cerr << "SODA: specify only one camera selector" << std::endl;
        return 2;
      }
      selected = match;
    } else if (options && value[0] == '-' && !is_shot_value(value)) {
      std::cerr << "SODA: unknown option " << value << std::endl;
      return 2;
    } else {
      positional.push_back(value);
    }
  }
  if (help) {
    print_usage(argv[0]);
    return 0;
  }
  if (version) {
    std::cout << SODA_BUILD_VERSION << std::endl;
    return 0;
  }
  if (positional.empty()) {
    std::cerr << "SODA: no image was provided" << std::endl;
    return 1;
  }
  if ((lens_cache != nullptr && lens_profile == nullptr) ||
      (lens_profile != nullptr && (!local || selected != &handlers[0]))) {
    std::cerr << "SODA: lens emulation requires --local --aicam --lens-profile "
                 "FILE\n";
    return 2;
  }
  if (positional.size() != 1 && positional.size() != 11) {
    std::cerr << "SODA: expected IMAGE optionally followed by 10 shot integers"
              << std::endl;
    return 2;
  }
  for (size_t index = 1; index < positional.size(); ++index) {
    if (!is_shot_value(positional[index])) {
      std::cerr << "SODA: invalid shot integer " << positional[index]
                << std::endl;
      return 2;
    }
  }

  std::ifstream image(positional[0], std::ios::binary | std::ios::ate);
  if (!image || image.tellg() <= 0) {
    std::cerr << "SODA: could not inspect image " << positional[0] << std::endl;
    return 1;
  }

  if (local) std::cerr << "SODA: development-PC mode (--local)" << std::endl;
  std::string lens_output;
  const char* analysis_image = positional[0];
  if (lens_profile != nullptr) {
    if (!emulate_lens(positional[0], lens_profile, lens_cache, lens_output)) {
      std::cerr << "SODA: lens emulation failed; clean image retained, "
                   "analysis skipped\n";
      return 1;
    }
    analysis_image = lens_output.c_str();
    std::cout << "SODA: lens-emulated analysis input " << analysis_image
              << '\n';
  }
  // Every backend's image is checked here, independent of `selected` -- usher's
  // own
  // "--aicam" selector looks identical whether or not --spottercam produced
  // this image, so the only reliable signal is whatever this image's own EXIF
  // says.
  if (!soda_process_object_detection(analysis_image)) {
    // A partial output failure must reach USHER's bounded retry policy.
    // Returning success here would silently abandon missing crops/CSV while
    // claiming completion.
    std::cerr << "SODA: object post-processing incomplete; original retained\n";
    return 1;
  }
  if (selected != nullptr) {
    selected->process();
  } else {
    std::cout << "SODA: I looked at the image. Cool, right?" << std::endl;
  }
  return 0;
}