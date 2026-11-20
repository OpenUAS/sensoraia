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
 * @brief Compile-time EARcam version and source-revision identity strings. */
#ifndef EARCAM_VERSION_H
#define EARCAM_VERSION_H
#ifndef EARCAM_GIT_SHA
#define EARCAM_GIT_SHA "unknown"
#endif
#ifndef EARCAM_VERSION
#define EARCAM_VERSION "1.0.0"
#endif
#define EARCAM_BUILD_VERSION "EARcam " EARCAM_VERSION " (Git " EARCAM_GIT_SHA ")"
#endif