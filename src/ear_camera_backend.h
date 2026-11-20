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
 * @file ear_camera_backend.h
 * @brief EAR-specific startup configuration behind USHER's generic backend interface.
 * @details Lifecycle hooks are serial-owner-only. The backend owns bounded
 * finalization and sensor-to-point conversion; USHER owns transport/correlation.
 */
#ifndef USHER_EAR_CAMERA_BACKEND_H
#define USHER_EAR_CAMERA_BACKEND_H

#include "camera_backend.h"

/**
 * @brief Return the EAR backend descriptor by value; its session hook table has static lifetime.
 * @return Descriptor advertising point-session capability, not optical still capture.
 */
struct camera_backend ear_camera_backend_get(void);
/**
 * @brief Copy an ALSA PCM name before initialization or worker startup.
 * @param device Nonempty bounded ALSA name, such as default or plughw:CARD=USB,DEV=0.
 * @return True when copied; false for invalid input or active lifecycle state.
 * @details Caller memory may be released on return. Simulation is selected separately.
 */
bool ear_camera_backend_set_device(const char *device);
/**
 * @brief Select deterministic synthetic audio before initialization, bypassing ALSA.
 * @param lat_deg Fixed source latitude, degrees.
 * @param lon_deg Fixed source longitude, degrees.
 * @param level_db_at_1m Simulated level at one metre; a test model, not measured calibration.
 */
void ear_camera_backend_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m);
/**
 * @brief Forward startup tone-band selection to the audio adapter.
 * @param low_cut_hz Lower band edge, hertz.
 * @param high_cut_hz Upper band edge, hertz; valid ordering is the CLI caller's responsibility.
 */
void ear_camera_backend_set_band(double low_cut_hz, double high_cut_hz);

#endif