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

/* Additional author attribution: Copyright 2026 Akshat Jain */

/**
 * @file dipla_planner.hpp
 * @brief Public planning, synthetic-survey, and comparison APIs for Diplacam.
 * @details
 * The production path conditions a finite-grid Gaussian-process belief from
 * synthetic observations and samples eight waypoints with a native DDIM loop
 * backed by ONNX Runtime. The opt-in comparison path plans over the same belief
 * with the native greedy/CMA-ES reference; it is not an automatic fallback.
 * Coordinates are local east/north metres. This header intentionally exposes
 * fixed-size production contracts while hiding Eigen, ONNX Runtime, and planner
 * implementation state from callers.
 */
#pragma once

#include "dipla_contract.h"
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace dipla {
/** Number of cells in the supported 51-by-51 world grid. */
constexpr std::size_t cells = DIPLA_GRID_SIDE * DIPLA_GRID_SIDE;
/** Number of ordered control points returned by either planner. */
constexpr std::size_t points = DIPLA_WAYPOINT_COUNT;
/** Flattened XYZ diffusion noise, grouped by coordinate axis. */
using Noise = std::array<double, DIPLA_COORDINATE_COUNT * points>;
/** Eight ordered XYZ waypoints in local east/north/metre coordinates. */
using Waypoints = std::array<std::array<double, DIPLA_COORDINATE_COUNT>, points>;

/** @brief Startup-only synthetic experiment settings shared by both planners. */
struct ExperimentOptions {
    /** Independent diffusion stream seed; retained across worker survey resets. */
    std::uint64_t diffusion_seed = 123;
    /** Independent sensor stream seed; reset only for a new survey. */
    std::uint64_t sensor_noise_seed = 149;
    /** Reference horizontal inset, in grid cells. */
    static constexpr unsigned default_margin_cells = 2;
    /** Horizontal inset for both comparison controls and published targets. */
    unsigned flight_margin_cells = default_margin_cells;
    /** Initial scalar field estimate, independent of the comparison threshold. */
    double gp_prior_mean = 0.2;
    /** Positive Matern marginal variance in squared field units. */
    double gp_kernel_variance = 0.0079;
    /** Positive Matern spatial length scale in metres. */
    double gp_lengthscale_m = 4.78;
    /** Full synthetic sensor field of view in degrees, strictly between 0 and 180. */
    double sensor_fov_deg = 60.0;
    /** Positive independent measurement variance used for noise and likelihood. */
    double sensor_noise_variance = 0.01;
    /** @brief Reject nonfinite values and incompatible margins before allocation. */
    void validate() const;
};

/**
 * @brief Apply the configured horizontal flight margin.
 * @details Raw model outputs remain unchanged until the application prepares
 * a flight route. The default two-cell inset clips XY to 4..96 metres without
 * modifying model Z.
 * @param route Complete finite route to constrain in place.
 * @param margin_cells Horizontal inset in grid cells, inclusive range 0..24.
 * @throws std::runtime_error If the margin is invalid or any coordinate is nonfinite.
 */
void apply_flight_margin(Waypoints& route, unsigned margin_cells = ExperimentOptions::default_margin_cells);

/**
 * @brief Load the ONNX Runtime C API before constructing an inference session.
 * @details Searches executable-relative directories in order, trying
 * architecture-specific runtime names before generic names on ARM64. The
 * optional directory is searched last and is normally resolved from the model
 * settings INI by the application entry point. The loaded handle is retained
 * for process lifetime so every shared session retains a valid runtime API.
 * @param onnx_lib_path Optional final directory to search; empty skips it.
 * @throws std::runtime_error If no candidate exists, cannot load, or exposes an
 * incompatible API.
 */
void initialize_runtime(const std::string& onnx_lib_path = {});

/**
 * @brief Fixed-shape posterior summary consumed by the diffusion model.
 * @details Arrays use row-major grid order: index `row * 51 + column`. Keeping
 * this transport type independent of Eigen makes the ONNX boundary explicit.
 */
struct Belief {
    /** Posterior mean at each grid cell. */
    std::array<double, cells> mean{};
    /** Posterior marginal variance at each grid cell. */
    std::array<double, cells> variance{};
};

/**
 * @brief Bounded parameters for the comparison-only CMA-ES planner.
 * @details This planner shares the observed belief but does not load the ONNX
 * model. The limits are checked by Survey::compare before expensive planning.
 */
struct CmaesOptions {
    /** Smallest complete comparison population. */
    static constexpr int population_size = 12;
    /** Largest accepted cooperative budget in milliseconds. */
    static constexpr int max_budget_ms = 4000;
    /** Largest accepted evaluation allowance. */
    static constexpr int max_evaluations = 3000;
    /** Cooperative wall-clock budget in milliseconds, inclusive range 1..4000. */
    int budget_ms = 1000;
    /** Maximum objective evaluations, inclusive range 12..3000. */
    int maxfevals = 300;
    /** Positive deterministic seed used by the native optimizer. */
    int seed = 55;
    /** Confidence-bound field importance threshold. */
    double utility_threshold = 0.3;
    /** Nonnegative confidence multiplier. */
    double confidence_beta = 1.0;
    /** Positive initial horizontal optimizer standard deviation in metres. */
    double step_xy_m = 20.0;
    /** @brief Validate the complete comparison policy before planning. */
    void validate() const;
};

/** @brief Model identity and execution limits for a diffusion network.
 * @details A graph and its preprocessing/tensor contract form one assignment.
 * The timeout applies to the complete 20-pass sampler, not separately to each
 * pass; otherwise a nominal budget could expand twentyfold. */
struct NetworkOptions {
    /** Optional integrity-checked model descriptor path. */
    std::string descriptor;
    /** CPU inference threads, bounded to 1..32; one avoids competing with other MORA workers by default. */
    unsigned threads = 1;
    /** Whole-plan cooperative deadline in milliseconds; zero leaves supervision to the caller. */
    unsigned timeout_ms = 0;
    /** Optional verified encoder/denoiser bundle; empty retains the original graph. */
    std::string conditioning_cache;
};

/**
 * @brief Owns an ONNX session and samples eight metre-valued control points.
 * @details Construction validates metadata and tensors; serialize sampling on one instance.
 */
class Network {
public:
    /**
     * @brief Load and validate a deployment model.
        * @details Construction performs the expensive one-time graph, metadata,
        * input-name, element-type, and shape checks before the object can sample.
     * @param model Path to the ONNX model.
     * @param metadata Path to its bounded deployment JSON metadata.
        * @param options Optional descriptor and bounded ONNX execution policy.
     * @throws std::exception If files, metadata, runtime, or tensor contracts are invalid.
     */
        Network(const std::string& model, const std::string& metadata,
            const NetworkOptions& options = {});
    /**
     * @brief Releases the owned ONNX session and inference buffers.
     * @details Pimpl ownership ensures runtime objects are destroyed with the network.
     */
    ~Network();

    /**
     * @brief Run the complete deterministic DDIM sampler for a supplied noise vector.
     * @details Belief channels and position are normalized according to model
     * metadata; the eight XYZ outputs are converted back to metres. The supplied
     * noise is not mutated, so identical inputs produce identical waypoints.
     * @param belief Posterior mean and variance on the fixed world grid.
     * @param position Current local east, north, and actual altitude in metres.
     * @param noise Initial 24-value standard-normal sample, grouped as X/Y/Z.
     * @return Eight ordered XYZ waypoints in metres.
     * @throws std::runtime_error If inputs are nonfinite/out of bounds or inference fails.
    * @pre One caller at a time owns this network; reusable tensors are mutable.
    * @note Returned Z is part of the trained tensor contract. The fixed-height
    * demo publishes only XY and must not substitute predicted Z for actual AGL.
     */
    Waypoints sample(const Belief& belief, const std::array<double, 3>& position,
                     const Noise& noise);
private:
    /** Opaque owner of ONNX resources and reusable tensor storage. */
    struct State;
    /** Exclusive ownership keeps the runtime session lifetime tied to this object. */
    std::unique_ptr<State> state_;
};

/**
 * @brief Maintains a synthetic field, GP posterior, and planning comparisons.
 * @details Field truth is private to the observation source. Callers receive
 * only posterior statistics; monotonically increasing observation numbers
 * prevent replaying a shot from conditioning the belief twice.
 */
class Survey {
public:
    /**
     * @brief Load the complete finite-grid synthetic field.
        * @details The field is retained privately and the GP prior is initialized
        * once; only observations and posterior summaries are exposed to callers.
     * @param field CSV containing one sample for each supported even-coordinate cell.
    * @param options Startup GP, synthetic sensor, seed and flight-margin policy.
     * @throws std::exception If the file is malformed, oversized, or incomplete.
     */
    explicit Survey(const std::string& field, const ExperimentOptions& options = {});
    /**
     * @brief Releases field and covariance storage.
     * @details The private pimpl owns the dense posterior and synthetic truth.
     */
    ~Survey();

    /**
     * @brief Assimilate one synthetic observation at a local pose.
        * @details Builds the supplied-altitude footprint, adds independently seeded
        * Gaussian sensor noise and applies one GP conditioning update. At the
        * demo height both injected noise and likelihood use the configured
        * positive variance, 0.01 in the default profile.
        * Non-increasing shot numbers are
        * discarded before any expensive matrix work. The full truth field never
        * becomes a network input: only the footprint conditions the belief.
     * @param east Local east coordinate in metres.
     * @param north Local north coordinate in metres.
     * @param number Nonzero, strictly increasing shot number.
    * @param height Model-domain sensor height; defaults to the original 2 m experiment.
     * @return True if the sensor footprint covered at least one grid cell; false
     * for a duplicate, stale, zero, or otherwise non-increasing number.
     * @throws std::runtime_error If coordinates are nonfinite or outside the field.
    * @note The low-height noise floor avoids extrapolating the reference law
    * to a negative variance; this remains a synthetic sensor experiment.
     */
    bool observe(double east, double north, std::uint32_t number,
                 double height = DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M);

    /**
     * @brief Run the explicitly requested native CMA-ES comparison planner.
     * @details Planning operates on a covariance copy and must not mutate the
     * observation posterior. It returns no route if cancellation or the deadline
     * prevents completion; a partial route is never published.
     * @param position Current local east, north, and altitude in metres.
    * @param options Validated comparison search, utility, budget and seed policy.
     * @param cancelled Optional cooperative cancellation predicate.
     * @return Complete eight-waypoint route, or `std::nullopt` on interruption.
     * @throws std::runtime_error If position or budget is invalid, or planning fails.
     */
    std::optional<Waypoints> compare(const std::array<double, 3>& position,
                                   const CmaesOptions& options = {},
                                   const std::function<bool()>& cancelled = {}) const;

    /**
     * @brief Copy the current GP marginal mean and variance into fixed-size storage.
     * @details Converts Eigen's internal vectors to the stable public row-major arrays.
     * @return Snapshot of posterior mean and nonnegative marginal variances.
     */
    Belief belief() const;
    /**
     * @brief Return the sum of posterior marginal variances.
     * @details Uses the covariance trace as a compact scalar uncertainty measure.
     * @return Sum of diagonal covariance entries.
     */
    double total_variance() const;
    /**
     * @brief Return the FNV-1a digest of the input CSV bytes.
     * @details This is diagnostic provenance, not cryptographic authentication.
        * @return 64-bit FNV-1a digest.
     */
    std::uint64_t field_digest() const;
private:
    /** Opaque owner of field truth, posterior mean/covariance, and replay state. */
    struct State;
    /** Exclusive ownership keeps private truth and dense GP state out of the API. */
    std::unique_ptr<State> state_;
};
}
