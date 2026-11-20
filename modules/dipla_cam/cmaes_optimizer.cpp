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
 * @file cmaes_optimizer.cpp
 * @brief Bounded CMA-ES refinement for the native reference planner.
 * @details Starts from the greedy lattice route, searches in CMA-ES latent
 * coordinates, transforms candidates into flight bounds, and retains the best
 * feasible route seen. Fixed-altitude tasks omit Z from the optimizer vector.
 */
#include "dipla_gp_belief_planner.hpp"
#include <cmaes_interface.h>
extern "C" {
#include <boundary_transformation.h>
}
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aipp {
namespace {
/** RAII guard for the C optimizer and its separately allocated boundary transform. */
struct Optimizer {
    cmaes_t state{}; /**< Optimizer-owned population/covariance resources. */
    cmaes_boundary_transformation_t bounds{}; /**< Bounded-coordinate transform, separate from optimizer state. */
    bool initialized = false; /**< Exit only after successful final initialization. */
    bool bounds_initialized = false; /**< Transform cleanup remains valid if optimizer setup fails. */
    /**
     * @brief Release initialized C resources in reverse dependency order.
     * @details Partial initialization is possible when the C API throws through
     * an error path, so each resource has an independent readiness flag.
     */
    ~Optimizer() {
        if (initialized) cmaes_exit(&state);
        if (bounds_initialized) cmaes_boundary_transformation_exit(&bounds);
    }
};
}
std::vector<Point> refine(const Grid& grid, const Vector& mean, const Matrix& covariance,
                          const Point& start, const std::vector<Point>& initial, const PlannerOptions& options) {
    if (initial.empty() || (options.keep_planning && !options.keep_planning())) return initial;
    // Excluding fixed Z is more than an optimization: it prevents numerical
    // search from wasting evaluations on a coordinate that cannot change.
    const int coordinates = options.zmin == options.zmax ? 2 : 3;
    const int dimension = static_cast<int>(coordinates * initial.size());
    std::vector<double> lower(dimension), upper(dimension), deviations(dimension), latent(dimension), bounded(dimension);
    auto best = initial;
    for (std::size_t waypoint = 0; waypoint < initial.size(); ++waypoint) {
        best[waypoint] = clip_point(grid, initial[waypoint], options);
        for (int coordinate = 0; coordinate < coordinates; ++coordinate) {
            const auto index = coordinates * waypoint + coordinate;
            lower[index] = coordinate == 0 ? grid.xmin + options.margin_cells * grid.step : coordinate == 1 ? grid.ymin + options.margin_cells * grid.step : options.zmin;
            upper[index] = coordinate == 0 ? grid.xmax - options.margin_cells * grid.step : coordinate == 1 ? grid.ymax - options.margin_cells * grid.step : options.zmax;
            deviations[index] = coordinate == 2 ? options.step_z : options.step_xy;
            bounded[index] = best[waypoint][coordinate];
        }
    }
    Optimizer optimizer;
    cmaes_boundary_transformation_init(&optimizer.bounds, lower.data(), upper.data(), lower.size());
    optimizer.bounds_initialized = true;
    // CMA-ES samples an unbounded Gaussian. Inverse-transform the warm start so
    // the corresponding bounded point remains the center of its first search.
    cmaes_boundary_transformation_inverse(&optimizer.bounds, bounded.data(), latent.data(), latent.size());
    char silent_filename[] = "non";
    cmaes_init_para(&optimizer.state, dimension, latent.data(), deviations.data(), options.seed, options.popsize, silent_filename);
    optimizer.state.sp.filename = silent_filename;
    double* fitness = cmaes_init_final(&optimizer.state);
    optimizer.state.sp.filename = nullptr;
    optimizer.initialized = true;
    const auto mask = importance_mask(mean, covariance, options);
    double best_value = trajectory_objective(grid, covariance, mask, start, best, options);
    int evaluations = 0;
    // Consume complete populations only: the C library updates its covariance
    // from a generation as a unit, so a partial final batch is not meaningful.
    for (int generation = 0; generation < options.maxiter && evaluations + options.popsize <= options.maxfevals; ++generation) {
        double* const* population = cmaes_SamplePopulation(&optimizer.state);
        for (int member = 0; member < options.popsize; ++member) {
            if (options.keep_planning && !options.keep_planning()) return best;
            cmaes_boundary_transformation(&optimizer.bounds, population[member], bounded.data(), bounded.size());
            // Start from the best full route and replace only searched axes;
            // in fixed-altitude mode this preserves every waypoint's Z exactly.
            auto proposal = best;
            for (std::size_t waypoint = 0; waypoint < proposal.size(); ++waypoint) {
                for (int coordinate = 0; coordinate < coordinates; ++coordinate)
                    proposal[waypoint][coordinate] = bounded[coordinates * waypoint + coordinate];
            }
            fitness[member] = trajectory_objective(grid, covariance, mask, start, proposal, options);
            if (!std::isfinite(fitness[member])) throw std::runtime_error("Nonfinite CMA-ES objective");
            if (fitness[member] < best_value) { best_value = fitness[member]; best = std::move(proposal); }
        }
        evaluations += options.popsize;
        cmaes_UpdateDistribution(&optimizer.state, fitness);
        if (cmaes_TestForTermination(&optimizer.state)) break;
    }
    return best;
}

std::vector<Point> plan(const Grid& grid, const Vector& mean, const Matrix& covariance,
                        const Point& start, const PlannerOptions& options) {
    auto initial = grid_search(grid, mean, covariance, start, options);
    return options.refine ? refine(grid, mean, covariance, start, initial, options) : initial;
}
}
