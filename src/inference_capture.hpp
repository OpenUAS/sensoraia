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

/** @file inference_capture.hpp
 * @brief Settings-driven capture bridge; neutral camera mechanics never interpret neural outputs. */
#ifndef SENSORAIA_INFERENCE_CAPTURE_HPP
#define SENSORAIA_INFERENCE_CAPTURE_HPP

#include "imx500_camera.hpp"
#include "inference_worker.h"

/** @brief Dispatch configured RPK/ONNX capture while preserving one physical camera owner.
 * @param argc Count of borrowed command-line arguments.
 * @param argv Capture tokens; --settings - requests executable-relative discovery.
 * @param processor CPU-model contract, output callbacks and application settings handler.
 * @param sensor_processor Existing RPK feature/detection policy for the on-sensor backend.
 * @return Minus one for another CLI mode, zero on success, one on runtime failure, two on misuse.
 * @details Pixels are prepared and encoded under the frame lease; slow CPU inference follows release.
 * Private image/result names are finalized before no-overwrite publication; failures never replay exposure. */
int inference_capture_cli(int argc, char **argv, const inference_image_processor &processor,
                           const imx500::FrameProcessor &sensor_processor);

#endif