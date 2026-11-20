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
 * @file version.h
 * @brief Compile-time build identity strings for USHER executables. */
#ifndef USHER_VERSION_H
#define USHER_VERSION_H

#ifndef USHER_GIT_SHA
  #define USHER_GIT_SHA "unknown"
#endif

#ifndef USHER_VERSION
  #define USHER_VERSION "0.42.0"
#endif

#ifndef SODA_VERSION
  #define SODA_VERSION "0.42.0"
#endif

#ifndef SPOTTERCAM_VERSION
  #define SPOTTERCAM_VERSION "0.42.0"
#endif

#ifndef AICAM_VERSION
  #define AICAM_VERSION "1.0.0"
#endif

#ifndef PANGEA_VERSION
  #define PANGEA_VERSION "0.42.0"
#endif

#ifndef CAMERA_LENS_VERSION
  #define CAMERA_LENS_VERSION "0.42.0"
#endif

#ifndef LWIR_CALIBRATE_VERSION
  #define LWIR_CALIBRATE_VERSION "1.0.0"
#endif

#ifndef DIPLACAM_VERSION
  #define DIPLACAM_VERSION "0.42.0"
#endif

#ifndef LOOMCAM_VERSION
  #define LOOMCAM_VERSION "0.42.0"
#endif

#ifndef EAR_HEATMAP_REPLAY_VERSION
  #define EAR_HEATMAP_REPLAY_VERSION "1.0.0"
#endif

#ifndef EAR_HEATMAP_OVERLAY_VERSION
  #define EAR_HEATMAP_OVERLAY_VERSION "1.0.0"
#endif

#define USHER_BUILD_VERSION "Usher " USHER_VERSION " (Git " USHER_GIT_SHA ")"
#define SODA_BUILD_VERSION "Soda " SODA_VERSION " (Git " USHER_GIT_SHA ")"
#define AICAM_BUILD_VERSION "AIcam " AICAM_VERSION " (Git " USHER_GIT_SHA ")"
#define SPOTTERCAM_BUILD_VERSION "Spottercam " SPOTTERCAM_VERSION " (Git " USHER_GIT_SHA ")"
#define PANGEA_BUILD_VERSION "Pangea " PANGEA_VERSION " (Git " USHER_GIT_SHA ")"
#define CAMERA_LENS_BUILD_VERSION "Camera Lens " CAMERA_LENS_VERSION " (Git " USHER_GIT_SHA ")"
#define LWIR_CALIBRATE_BUILD_VERSION "LWIR Calibrate " LWIR_CALIBRATE_VERSION " (Git " USHER_GIT_SHA ")"
#define DIPLACAM_BUILD_VERSION "Diplacam " DIPLACAM_VERSION " (Git " USHER_GIT_SHA ")"
#define LOOMCAM_BUILD_VERSION "loomcam " LOOMCAM_VERSION " (Git " USHER_GIT_SHA ")"
#define EAR_HEATMAP_REPLAY_BUILD_VERSION "EAR Heatmap Replay " EAR_HEATMAP_REPLAY_VERSION " (Git " USHER_GIT_SHA ")"
#define EAR_HEATMAP_OVERLAY_BUILD_VERSION "EAR Heatmap Overlay " EAR_HEATMAP_OVERLAY_VERSION " (Git " USHER_GIT_SHA ")"
#endif