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
 * @file diplacam.cpp
 * @brief Command-line and worker-process entry point for Diplacam.
 * @details
 * Keep application policy here and numerical work in its owning modules. This
 * entry point resolves settings, selects one planner, and chooses one lifetime:
 * a single standalone observation/route, or a long-lived IPC service whose
 * planner thread retains survey state across route chunks.
 *
 * Model settings are discovered relative to the executable, not the launcher's
 * working directory. Explicit CLI model/metadata paths override their defaults,
 * but do not turn off an existing INI's runtime, descriptor, or comparison policy.
 * The distinction matters when launching from a service, a shell, or a package.
 *
 * @par Planner selection is not error recovery
 * cmaes=true explicitly selects the native comparison planner. It bypasses ONNX
 * initialization rather than trying diffusion first and quietly changing the
 * experiment on failure. An invalid selected planner reports an error.
 *
 * @par Output contract
 * Standalone stdout is one eight-point XY CSV, with diagnostics on stderr.
 * Model-predicted Z is retained inside the sampler for numerical parity, but
 * is not an altitude command. Both planner choices share flight-margin preparation.
 * @see dipla_planner.hpp Survey, inference, and publication-preparation boundaries.
 * @see dipla_worker.hpp Persistent IPC service and descriptor ownership.
 */
#include "dipla_planner.hpp"
#include "dipla_worker.hpp"
#include "aigenlib/compat.hpp"
#include "../../src/bounded_ini.h"
#include "../../src/inference_settings.h"
#include "../../src/inference_model.h"
#include "../../src/version.h"
#include <charconv>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string_view>

namespace {
/**
 * @brief Parsed application settings before CLI overrides and mode dispatch.
 * @details Optional values preserve the difference between absent and explicitly
 * supplied settings. In particular, an explicitly false cmaes must still count
 * as present when detecting duplicate keys; false is not a missing-value marker.
 */
struct ModelFiles {
    /** @var ModelFiles::model
     * @brief Graph path resolved by the shared inference-settings reader. */
    /** @var ModelFiles::metadata
     * @brief Diplacam deployment JSON path, rebased locally against the selected INI. */
    /** Optional runtime directory resolved by the shared settings reader, not a library filename. */
    std::string model, metadata, onnx_lib_path, conditioning_cache;
    /** Shared backend, source, descriptor, thread, and deadline policy; not a second INI dialect. */
    inference_settings inference{};
    /** Optional explicit comparison-planner selector. */
    std::optional<bool> cmaes;
    /** Validated startup comparison policy; omitted keys retain its defaults. */
    dipla::CmaesOptions comparison;
    /** Startup-only experiment snapshot passed identically to standalone and worker modes. */
    dipla::ExperimentOptions experiment;
    /** Presence mask for application-specific keys, including explicitly false values. */
    std::uint32_t seen = 0;
};

/**
 * @brief Parse one bounded integer setting without accepting partial numbers.
 * @details `from_chars` avoids locale-dependent parsing; checking the final
 * pointer rejects suffixes such as `100ms`, while the closed interval protects
 * downstream work and integer conversions from unreasonable configuration.
 * @param[in,out] destination Field populated only after complete validation.
 * @param[in] text NUL-terminated decimal integer from the bounded parser.
 * @param[in] minimum Inclusive lower bound.
 * @param[in] maximum Inclusive upper bound.
 * @return True only when the value is fully parsed and in range.
 * @post On failure, destination is unchanged; a malformed value cannot partially apply.
 */
bool integer_setting(int& destination, const char* text, int minimum, int maximum) {
    const std::string value(text);
    int parsed = 0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed < minimum || parsed > maximum)
        return false;
    destination = parsed;
    return true;
}

/**
 * @brief Validate and store one key/value pair from the flat application INI.
 * @details The shared inference-settings reader handles its common keys and
 * delegates application-specific entries here. Keep descriptor/runtime/backend
 * parsing there; copying it here would let two consumers disagree about one INI.
 * Returning false rejects unknown or duplicate application settings rather than
 * silently making behavior depend on file order.
 * @param[in,out] context Pointer to the live ModelFiles being populated.
 * @param[in] key Application setting name, NUL-terminated.
 * @param[in] value Setting value, NUL-terminated and nonempty after common parsing.
 * @return True when the key is recognized and its value is valid.
 * @pre context points to ModelFiles for the duration of the synchronous read.
 */
bool model_setting(void* context, const char* key, const char* value) {
    auto& files = *static_cast<ModelFiles*>(context);
    static constexpr const char* keys[] = {
        "cmaes", "metadata", "cmaes_budget_ms", "cmaes_maxfevals", "cmaes_seed",
        "diffusion_seed", "sensor_noise_seed", "flight_margin_cells", "gp_prior_mean",
        "gp_kernel_variance", "gp_lengthscale_m", "sensor_fov_deg", "sensor_noise_variance",
        "cmaes_utility_threshold", "cmaes_confidence_beta", "cmaes_step_xy_m", "conditioning_cache"
    };
    static_assert(std::size(keys) <= std::numeric_limits<decltype(files.seen)>::digits);
    std::size_t index = 0;
    while (index < std::size(keys) && std::string_view(key) != keys[index]) ++index;
    if (index == std::size(keys) || (files.seen & (1U << index))) return false;
    files.seen |= 1U << index;
    if (std::string(key) == "cmaes") {
        // Accept one spelling for each boolean. A typo must not quietly select
        // another planner, and an earlier explicit false still forbids a duplicate.
        if (files.cmaes.has_value()) return false;
        if (std::string(value) == "true") files.cmaes = true;
        else if (std::string(value) == "false") files.cmaes = false;
        else return false;
        return true;
    }
    if (std::string_view(key) == "cmaes_budget_ms")
        return integer_setting(files.comparison.budget_ms, value, 1, dipla::CmaesOptions::max_budget_ms);
    if (std::string_view(key) == "cmaes_maxfevals")
        return integer_setting(files.comparison.maxfevals, value, dipla::CmaesOptions::population_size,
                               dipla::CmaesOptions::max_evaluations);
    if (std::string_view(key) == "cmaes_seed")
        return integer_setting(files.comparison.seed, value, 1, std::numeric_limits<int>::max());
    if (std::string_view(key) == "diffusion_seed") return ini_uint(value, &files.experiment.diffusion_seed);
    if (std::string_view(key) == "sensor_noise_seed") return ini_uint(value, &files.experiment.sensor_noise_seed);
    if (std::string_view(key) == "flight_margin_cells") {
        uint64_t margin = 0;
        if (!ini_uint(value, &margin) || margin >= (DIPLA_GRID_SIDE - 1U) / 2U) return false;
        files.experiment.flight_margin_cells = static_cast<unsigned>(margin);
        return true;
    }
    struct RealSetting { const char* name; double* destination; };
    const RealSetting real_settings[] = {
        {"gp_prior_mean", &files.experiment.gp_prior_mean},
        {"gp_kernel_variance", &files.experiment.gp_kernel_variance},
        {"gp_lengthscale_m", &files.experiment.gp_lengthscale_m},
        {"sensor_fov_deg", &files.experiment.sensor_fov_deg},
        {"sensor_noise_variance", &files.experiment.sensor_noise_variance},
        {"cmaes_utility_threshold", &files.comparison.utility_threshold},
        {"cmaes_confidence_beta", &files.comparison.confidence_beta},
        {"cmaes_step_xy_m", &files.comparison.step_xy_m}
    };
    for (const auto& setting : real_settings)
        if (std::string_view(key) == setting.name) return ini_real(value, setting.destination);
    // Metadata belongs to Diplacam's DDIM adapter, not the generic inference
    // schema. Store its raw path now; the owning reader resolves its INI base.
    std::string* destination = nullptr;
    if (std::string(key) == "metadata") destination = &files.metadata;
    if (std::string(key) == "conditioning_cache") destination = &files.conditioning_cache;
    if (!destination || !destination->empty()) return false;
    *destination = value;
    return true;
}

/**
 * @brief Read, validate, and resolve application settings.
 * @details Reads at most 8 KiB through the bounded INI parser.
 * Model, metadata, and runtime-directory paths are resolved against the INI
 * location so invocation from an unrelated working directory is reproducible.
 * Comparison mode may omit model files because it does not construct ONNX.
 * @param[in] filename Selected settings file.
 * @param[in] require_model_files Whether missing CLI paths need diffusion defaults.
 * @return Validated values, including empty fields for omitted optional settings.
 * @throws std::runtime_error If the file is oversized, malformed, duplicated,
 * contains an unknown key, or lacks required diffusion paths.
 * @note Reading settings does not load the runtime or graph. Help/version and
 * comparison-only execution must not acquire those resources as a side effect.
 * @see model_setting Application-specific extension to the common INI grammar.
 */
ModelFiles read_model_settings(const std::filesystem::path& filename, bool require_model_files) {
    ModelFiles files;
    // Diplacam is a diffusion consumer unless comparison is explicitly selected.
    // Do not let generic backend discovery invent a different experiment.
    inference_settings_init(&files.inference, INFERENCE_ONNX);
    std::array<char, 1024> error{};
    if (!inference_settings_read(&files.inference, filename.c_str(), model_setting, &files,
                                 error.data(), error.size()))
        throw std::runtime_error("Invalid model settings: " + filename.string());
    files.model = files.inference.model;
    files.onnx_lib_path = files.inference.library_directory;
    try {
        files.experiment.validate();
        files.comparison.validate();
    } catch (const std::runtime_error& error) {
        throw std::runtime_error("Invalid model settings: " + filename.string() + ": " + error.what());
    }
    // The native backend is meaningful here only for explicit CMA-ES. LIVE is
    // the supported source selector; it does not turn the synthetic GP field
    // into a physical camera driver or promise real-world observation accuracy.
    if ((files.inference.backend != INFERENCE_ONNX &&
         !(files.inference.backend == INFERENCE_NATIVE && files.cmaes.value_or(false))) ||
        files.inference.source != INFERENCE_LIVE ||
        (require_model_files && !files.cmaes.value_or(false) &&
         (files.model.empty() || files.metadata.empty())))
        throw std::runtime_error("Invalid model settings: " + filename.string());
    // Common paths were already resolved by inference_settings_read. Rebase
    // only the adapter-specific metadata path; rebasing a resolved path again
    // would make launch-directory changes surprisingly educational.
    const auto directory = std::filesystem::absolute(filename).parent_path();
    if (!files.metadata.empty()) files.metadata = (directory / files.metadata).lexically_normal().string();
    if (!files.conditioning_cache.empty()) files.conditioning_cache = (directory / files.conditioning_cache).lexically_normal().string();
    return files;
}
}

/**
 * @brief Parse CLI options and run standalone planning or the IPC worker.
 * @details Defaults are discovered relative to the executable rather than the
 * process working directory. Explicit CLI paths override model settings, while
 * an existing INI is still read for runtime lookup and comparison controls.
 * Standalone mode emits the eight XY points as CSV and keeps diagnostics on
 * stderr; the fixed 2 m survey footprint is separate from model-predicted Z.
 * @param[in] argc Number of command-line arguments, including the executable name.
 * @param[in] argv Process-owned, NUL-terminated argument strings.
 * @retval 0 Standalone output, information request, or orderly IPC service completion.
 * @retval 1 A standard exception was reported on stderr, or the IPC service failed.
 * @note Information options return when encountered, before reading settings or
 * initializing inference. Earlier malformed arguments can still report errors.
 * @warning The standalone command produces one route, not a simulated flight
 * or a live waypoint publisher. The persistent owner uses serve() instead.
 */
int main(int argc, char** argv) {
    try {
        // Resolve the real executable rather than argv[0], which may be a PATH
        // lookup or symlink. An adjacent INI wins over the source-tree layout,
        // then the installed share layout is the final default. A present but
        // invalid selected INI is an error, not permission to try another one.
        const auto executable = std::filesystem::canonical("/proc/self/exe");
        const auto local = executable.parent_path() / "../modules/dipla_cam";
        const auto directory = std::filesystem::exists(executable.parent_path() / "diplacam_settings.ini") ?
                       executable.parent_path() : std::filesystem::exists(local / "diplacam_settings.ini") ?
                       local : executable.parent_path() / "../share/diplacam";
        std::string settings = (directory / "diplacam_settings.ini").string();
        std::string model, metadata;
        std::string field;
        std::string configuration;
        std::optional<std::string> conditioning_cache;
        int descriptor = -1;
        // Start inside the two-cell horizontal margin. Height stays at the
        // declared 2 m experiment; model-predicted Z must not replace this input.
        std::array<double, 3> position{{4, 4, DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M}};
        bool show_config = false;
        for (int argument = 1; argument < argc; ++argument) {
            const std::string option = argv[argument];
            if (option == "--show-config") { show_config = true; continue; }
            // Information requests are model-free so broken deployments can
            // still identify their binary/backend and show usable diagnostics.
            if (option == "--version" || option == "-V") {
                std::cout << DIPLACAM_BUILD_VERSION << '\n';
                return 0;
            }
            if (option == "--build-info") {
                std::cout << aipp::math::build_info << '\n';
                return 0;
            }
            if (option == "--help" || option == "-h") {
                std::cout << DIPLACAM_BUILD_VERSION << '\n'
                          << "Usage: diplacam --field FIELD.csv [OPTIONS]\n"
                             "       diplacam --config CONFIG.ini --ipc-fd FD [OPTIONS]\n\n"
                             "Standalone planning:\n"
                             "  --field FILE     Required field CSV; emits eight XY slots as CSV.\n"
                             "  --east M         Initial east coordinate in meters (default: 4).\n"
                             "  --north M        Initial north coordinate in meters (default: 4).\n"
                             "                   Demonstration altitude is fixed at 2 m.\n\n"
                             "IPC worker (normally started by USHER):\n"
                             "  --config FILE    Worker INI: field, cadence_ms, planning_pose_max_age_ms.\n"
                             "  --ipc-fd FD      Inherited IPC descriptor; requires --config, not --field.\n\n"
                             "Model selection (both modes):\n"
                             "  --settings FILE  Model-settings INI (default: executable-relative\n"
                             "                   diplacam_settings.ini in the app, module or share directory).\n"
                             "  --model FILE     Override the ONNX model from settings.\n"
                             "  --metadata FILE  Override the deployment JSON from settings.\n"
                             "  --conditioning-cache DIR  Opt-in verified encoder/denoiser bundle.\n"
                             "                   INI paths are relative to the INI; CLI paths use cwd.\n"
                             "                   Set onnx_lib_path in settings for a runtime directory.\n"
                             "                   cmaes=true selects comparison-only planning without ONNX.\n"
                             "                   Comparison settings: cmaes_budget_ms (1..4000),\n"
                             "                   cmaes_maxfevals (12..3000), cmaes_seed (positive int32).\n\n"
                             "Information (no model or runtime required):\n"
                             "  --version, -V    Show application version and build Git revision.\n"
                             "  --build-info     Show the compiled numerical backend.\n"
                             "  --help, -h       Show this help.\n";
                std::cout << "  --show-config    Print effective settings without loading the model or field.\n";
                return 0;
            }
            // Remaining options consume exactly one value. Check availability
            // before advancing argv, and reject partially parsed numeric values.
            if (argument + 1 == argc) throw std::runtime_error("Missing argument for " + option);
            const std::string value = argv[++argument];
            if (option == "--field") field = value;
            else if (option == "--settings") settings = value;
            else if (option == "--config") configuration = value;
            else if (option == "--ipc-fd") {
                std::size_t used = 0;
                descriptor = std::stoi(value, &used);
                if (used != value.size() || descriptor < 0) throw std::runtime_error("Invalid IPC descriptor");
            }
            else if (option == "--model") model = value;
            else if (option == "--metadata") metadata = value;
            else if (option == "--conditioning-cache") conditioning_cache = value;
            else if (option == "--east" || option == "--north") {
                std::size_t used = 0;
                const double number = std::stod(value, &used);
                if (used != value.size() || !std::isfinite(number)) throw std::runtime_error("Invalid coordinate");
                // Finite parsing belongs here; the selected survey/planner owns
                // field bounds. Keep east as X and north as Y at every boundary.
                position[option == "--east" ? 0 : 1] = number;
            } else throw std::runtime_error("Unknown option: " + option);
        }
        std::string onnx_lib_path;
        bool cmaes = false;
        dipla::CmaesOptions comparison;
        dipla::ExperimentOptions experiment;
        dipla::NetworkOptions network_options;
        // Read an existing INI even with both CLI model paths: it still chooses
        // runtime search, descriptor, resource limits, and comparison mode.
        // Only two explicit paths AND an absent INI bypass settings altogether.
        if (model.empty() || metadata.empty() || std::filesystem::exists(settings)) {
            const auto files = read_model_settings(settings, !show_config && (model.empty() || metadata.empty()));
            if (model.empty()) model = files.model;
            if (metadata.empty()) metadata = files.metadata;
            onnx_lib_path = files.onnx_lib_path;
            // A CLI graph override does not disable the settings descriptor.
            // If it names another export, the planner's integrity checks should
            // reject that mismatch rather than silently weaken the model contract.
            network_options.descriptor = files.inference.descriptor;
            network_options.threads = files.inference.threads;
            network_options.timeout_ms = files.inference.timeout_ms;
            network_options.conditioning_cache = files.conditioning_cache;
            cmaes = files.cmaes.value_or(false);
            comparison = files.comparison;
            experiment = files.experiment;
        }
        if (conditioning_cache) network_options.conditioning_cache = *conditioning_cache;
        if (show_config) {
            std::cout << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << "model=" << model << '\n' << "metadata=" << metadata << '\n'
                      << "model_descriptor=" << network_options.descriptor << '\n'
                      << "onnx_lib_path=" << onnx_lib_path << '\n'
                      << "onnx_threads=" << network_options.threads << '\n'
                      << "inference_timeout_ms=" << network_options.timeout_ms << '\n'
                      << "conditioning_cache=" << network_options.conditioning_cache << '\n'
                      << "cmaes=" << (cmaes ? "true" : "false") << '\n'
                      << "cmaes_budget_ms=" << comparison.budget_ms << '\n'
                      << "cmaes_maxfevals=" << comparison.maxfevals << '\n'
                      << "cmaes_seed=" << comparison.seed << '\n'
                      << "cmaes_utility_threshold=" << comparison.utility_threshold << '\n'
                      << "cmaes_confidence_beta=" << comparison.confidence_beta << '\n'
                      << "cmaes_step_xy_m=" << comparison.step_xy_m << '\n'
                      << "diffusion_seed=" << experiment.diffusion_seed << '\n'
                      << "sensor_noise_seed=" << experiment.sensor_noise_seed << '\n'
                      << "flight_margin_cells=" << experiment.flight_margin_cells << '\n'
                      << "gp_prior_mean=" << experiment.gp_prior_mean << '\n'
                      << "gp_kernel_variance=" << experiment.gp_kernel_variance << '\n'
                      << "gp_lengthscale_m=" << experiment.gp_lengthscale_m << '\n'
                      << "sensor_fov_deg=" << experiment.sensor_fov_deg << '\n'
                      << "sensor_noise_variance=" << experiment.sensor_noise_variance << '\n';
            return 0;
        }
        // The descriptor, not --config by itself, selects persistent IPC mode.
        // Forbid a second field source so the worker's config has one clear owner.
        // Runtime selection must precede creation of the planner's ONNX session.
        if (descriptor >= 0) {
            if (configuration.empty() || !field.empty()) throw std::runtime_error("IPC mode requires only --config");
            if (!cmaes) dipla::initialize_runtime(onnx_lib_path);
            return dipla::serve(descriptor, configuration, model, metadata, cmaes, comparison, network_options, experiment);
        }
        if (!configuration.empty()) throw std::runtime_error("--config requires --ipc-fd");
        if (field.empty()) throw std::runtime_error("An explicit --field CSV is required");
        if (!cmaes) dipla::initialize_runtime(onnx_lib_path);
        // Standalone means a fresh experiment: one initial noisy observation,
        // then one route from that posterior. Do not feed private full-field truth
        // to the network or quietly plan from an entirely unobserved prior.
        dipla::Survey survey(field, experiment);
        survey.observe(position[0], position[1], 1);
        dipla::Waypoints route{};
        if (cmaes) {
            const auto result = survey.compare(position, comparison);
            // Never print half a route as a successful eight-point result.
            if (!result) throw std::runtime_error("CMA-ES comparison budget expired before a complete route");
            route = *result;
        } else {
            dipla::Network network(model, metadata, network_options);
            dipla::Noise noise{};
            // Repeatable diffusion noise, independent of Survey's sensor RNG.
            // The sampler expects axis-major X/Y/Z blocks. Matching this seed
            // alone does not reproduce NumPy's different random-number stream.
            std::mt19937_64 generator(experiment.diffusion_seed);
            std::normal_distribution<double> normal;
            for (double& value : noise) value = normal(generator);
            route = network.sample(survey.belief(), position, noise);
        }
        // Apply flight preparation once, regardless of which planner produced
        // the route. Keeping padding outside sample() preserves raw model parity.
        dipla::apply_flight_margin(route, experiment.flight_margin_cells);
        std::cerr << "diplacam planner=" << (cmaes ? "cmaes comparison_only=true onnx=false" : "diffusion onnx=true") << '\n';
        std::cerr << "diplacam field_fnv1a=" << survey.field_digest() << " total_variance=" << survey.total_variance()
                  << " actual_agl_m=2 predicted_z_discarded=true\n";
        // stdout is a machine-readable contract: no banners, diagnostics, or Z
        // column mixed into a valid standalone result. Internal slots 0..7 map
        // to P1..P8 at publication, not to eight automatic navigation commands.
        std::cout << "slot,east_m,north_m\n" << std::setprecision(12);
        for (std::size_t slot = 0; slot < route.size(); ++slot)
            std::cout << slot << ',' << route[slot][0] << ',' << route[slot][1] << '\n';
        return 0;
    } catch (const std::exception& error) {
        // One process-level error boundary keeps diagnostics on stderr and
        // ensures invalid setup or planning is never reported as a successful CSV.
        std::cerr << "diplacam: " << error.what() << '\n';
        return 1;
    }
}
