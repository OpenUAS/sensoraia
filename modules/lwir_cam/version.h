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
 * @brief Build-version string exported by the standalone LWIR application.
 * @details The build may inject LWIRCAM_GIT_SHA and LWIRCAM_APP_VERSION with -D;
 * the fallbacks keep standalone or out-of-repository compiles working. */
#ifndef LWIRCAM_BUILD_VERSION_H
#define LWIRCAM_BUILD_VERSION_H
#ifndef LWIRCAM_GIT_SHA
/** @brief Source revision; "unknown" when the build did not provide one. */
#define LWIRCAM_GIT_SHA "unknown"
#endif
#ifndef LWIRCAM_APP_VERSION
/** @brief Application version used when the build did not provide one. */
#define LWIRCAM_APP_VERSION "1.0.0"
#endif
/** @brief Human-readable identity, e.g. "LWIRcam 1.0.0 (Git unknown)", built by literal concatenation. */
#define LWIRCAM_BUILD_VERSION "LWIRcam " LWIRCAM_APP_VERSION " (Git " LWIRCAM_GIT_SHA ")"
#endif