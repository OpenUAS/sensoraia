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
 * @file dipla_worker.hpp
 * @brief Public entry point for Diplacam's bounded local IPC worker.
 * @details The worker receives fixed-size requests over a connected Unix
 * socket, keeps a bounded ordered observation FIFO, and returns immutable route
 * snapshots or pinned legacy point slots. Transport details and synchronization state remain private to the
 * implementation.
 */
#pragma once
#include "dipla_planner.hpp"
#include <string>
namespace dipla {
/**
 * @brief Serve a Diplacam session over an already-connected IPC descriptor.
 * @details Starts one planning thread and services requests on the calling
 * thread. One GP owner consumes each accepted observation once. Queue overflow
 * fails closed. START during an active survey requests a new retained chunk;
 * its first accepted pose anchors field centre (50,50). Active START retains
 * that frame and belief; STOP followed by START resets both. Complete-route replies copy one
 * generation; legacy retrieval pins a route across eight points. The caller retains descriptor ownership and
 * should close it after this returns.
 * Reply backpressure fails the connection instead of blocking request handling.
 * @param descriptor Connected socket descriptor; ownership remains with caller.
 * @param config Worker INI containing field, cadence and planning-pose age settings.
 * @param default_model Default model path supplied by the executable settings/CLI.
 * @param default_metadata Default metadata path supplied by the executable settings/CLI.
 * @param cmaes Select the explicit comparison-only planner; false uses diffusion.
 * @param comparison Bounded CMA-ES options used only when `cmaes` is true.
 * @param network_options Descriptor and bounded ONNX execution policy, ignored in comparison mode.
 * @param experiment Startup GP, synthetic sensor, seed and flight-margin policy.
 * @return Zero after orderly transport and worker shutdown; nonzero on failure.
 * @throws std::exception For invalid configuration or unrecoverable setup errors.
 * @note Computational validity is not route execution or field accuracy. This
 * worker neither publishes GCS moves nor changes navigation blocks.
 */
int serve(int descriptor, const std::string& config, const std::string& default_model,
          const std::string& default_metadata, bool cmaes = false,
          const CmaesOptions& comparison = {}, const NetworkOptions& network_options = {},
          const ExperimentOptions& experiment = {});
}
