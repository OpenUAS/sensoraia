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
 * @file dipla_gp_belief_planner.hpp
 * @brief Native finite-grid GP sensor model and trajectory-planning API.
 * @details Provides the reusable research/reference planner used by the
 * explicitly selected CMA-ES comparison mode. The implementation stores dense
 * covariance in build-selected row-major matrices, so the 2601-cell cap is a deliberate
 * memory bound rather than an arbitrary input restriction.
 */
#pragma once

#include "aigenlib/compat.hpp"
#include <cstddef>
#include <functional>
#include <random>
#include <vector>

namespace aipp {
/** Default route-pose sampling interval for look-ahead scoring. */
inline constexpr int lookahead_stride = 4;
/** Innovation-system row budget, independent of worker FIFO capacity. */
inline constexpr int lookahead_measurement_budget = 64;
/** Default emitted spline samples per control segment. */
inline constexpr int spline_samples_per_segment = 5;

/** Dense row-major covariance layout chosen for contiguous row updates. */
using Matrix = math::Matrix;
/** Dynamic GP mean, measurement, or per-cell variance vector. */
using Vector = math::Vector;
/** Three-dimensional local east/north/altitude point, in metres. */
using Point = math::Point;

/**
 * @brief Rectangular, evenly spaced local map grid.
 * @details The upper bounds are snapped to the last grid coordinate that fits
 * the requested spacing. Construction rejects grids larger than 2601 cells to
 * bound dense covariance memory on the target board.
 */
struct Grid {
    /** Inclusive lower east bound in metres. */
    double xmin = 0.0;
    /** Inclusive, grid-aligned upper east bound in metres. */
    double xmax = 100.0;
    /** Inclusive lower north bound in metres. */
    double ymin = 0.0;
    /** Inclusive, grid-aligned upper north bound in metres. */
    double ymax = 100.0;
    /** Uniform spacing in metres. */
    double step = 2.0;
    /** Number of columns. */
    int width = 0;
    /** Number of rows. */
    int height = 0;

        /**
        * @brief Create a grid and align its effective upper bounds.
        * @details Dimensions include only complete spacing intervals; effective
        * upper bounds become the final sampled grid coordinates.
        * @param xminimum Inclusive lower east coordinate.
        * @param xmaximum Requested upper east coordinate.
        * @param yminimum Inclusive lower north coordinate.
        * @param ymaximum Requested upper north coordinate.
        * @param spacing Positive uniform sample interval.
        * @throws std::invalid_argument If bounds/spacing are invalid or exceed the dense-grid cap.
        */
    Grid(double xminimum = 0.0, double xmaximum = 100.0,
         double yminimum = 0.0, double ymaximum = 100.0, double spacing = 2.0);
        /** @brief Return the number of grid cells.
        * @details The grid is stored as a dense row-major domain.
        * @return `width * height`.
        */
    int size() const;
        /** @brief Convert a column index to its east coordinate.
            * @details Uses the affine grid mapping `xmin + column * step`.
        * @param column Zero-based column.
        * @return East coordinate in metres.
        */
    double x(int column) const;
        /** @brief Convert a row index to its north coordinate.
            * @details Uses the affine grid mapping `ymin + row * step`.
        * @param row Zero-based row.
        * @return North coordinate in metres.
        */
    double y(int row) const;
        /** @brief Find the nearest clamped column; half-cell ties round downward.
            * @details The explicit tie rule avoids dependence on the process rounding mode.
        * @param value East coordinate in metres.
        * @return Valid zero-based column.
        * @throws std::invalid_argument For a nonfinite coordinate.
        */
    int nearest_x(double value) const;
        /** @brief Find the nearest clamped row; half-cell ties round downward.
            * @details The explicit tie rule avoids dependence on the process rounding mode.
        * @param value North coordinate in metres.
        * @return Valid zero-based row.
        * @throws std::invalid_argument For a nonfinite coordinate.
        */
    int nearest_y(double value) const;
};

/**
 * @brief Sparse block-averaging observation operator.
 * @details Each row lists grid-cell indices averaged into one measurement;
 * `variances` stores the independent observation variance for each row.
 */
using Sensor = dipla::aigenlib::Sensor;

/**
 * @brief Search, utility, and optimizer controls for the reference planner.
 * @details `keep_planning` is polled inside expensive search/refinement loops;
 * it provides cooperative cancellation, not preemptive interruption.
 */
struct PlannerOptions {
    /** Confidence multiplier applied to posterior standard deviation. */
    double beta = 1.0;
    /** Field-value threshold used to define planning importance. */
    double threshold = 0.3;
    /** Minimum permitted altitude in metres. */
    double zmin = 10.0;
    /** Maximum permitted altitude in metres. */
    double zmax = 40.0;
    /** Full sensor field-of-view angle in degrees. */
    double angle_of_view = 60.0;
    /** Exponent used to penalize travel distance. */
    double cost_exponent = 1.0;
    /** Initial CMA-ES standard deviation for horizontal coordinates. */
    double step_xy = 20.0;
    /** Initial CMA-ES standard deviation for altitude. */
    double step_z = 8.0;
    /** Greedy waypoint count. */
    int horizon = 8;
    /** Maximum CMA-ES generations. */
    int maxiter = 25;
    /** CMA-ES population size and evaluation batch size. */
    int popsize = 12;
    /** Maximum objective evaluations; an incomplete population is not run. */
    int maxfevals = 1000;
    /** Reproducible optimizer seed. */
    long seed = 55;
    /** Choose lower-confidence-bound versus upper-confidence-bound utility. */
    bool lower_confidence_bound = true;
    /** Run CMA-ES refinement after greedy lattice selection. */
    bool refine = true;
    /** Fixed observation variance; negative selects the altitude noise model. */
    double observation_variance = -1.0;
    /** Horizontal control inset in grid cells, independent of altitude. */
    unsigned margin_cells = 2;
    /** Optional cooperative continue predicate, checked during planning. */
    std::function<bool()> keep_planning;
};

/**
 * @brief Compute the ground-plane footprint radius from altitude and full FOV angle.
 * @details Uses the tangent of the half-angle because the input is the full FOV.
 * @param altitude Sensor altitude above ground in metres.
 * @param angle Full field-of-view angle in degrees.
 * @return Ground-plane footprint radius in metres.
 */
double fov_radius(double altitude, double angle = 60.0);
/**
 * @brief Select the research sensor's altitude-dependent aggregation block size.
 * @details Piecewise aggregation preserves the reference sensor's coarser
 * effective resolution at higher altitude.
 * @param altitude Sensor altitude in metres.
 * @return Aggregation block width in cells.
 */
int resolution_block_size(double altitude);
/**
 * @brief Compute modeled observation variance at an altitude.
 * @details Exponentially approaches the reference high-altitude variance rather
 * than increasing without bound.
 * @param altitude Sensor altitude in metres.
 * @param minimum_altitude Reference altitude for the minimum variance.
 * @return Modeled measurement variance.
 */
double noise_model(double altitude, double minimum_altitude = 10.0);
/**
 * @brief Initialize a dense Matérn-3/2 GP covariance matrix.
 * @details Kernel values are cached by displacement on the uniform grid; the
 * dense matrix then supports the block updates used by conditioning.
 * @param grid Grid defining the covariance coordinates.
 * @param sigma2 Kernel marginal variance.
 * @param lengthscale Matérn spatial length scale in metres.
 * @return Symmetric dense prior covariance.
 * @throws std::invalid_argument If kernel amplitude or length scale is invalid.
 */
Matrix initialize_covariance(const Grid& grid, double sigma2, double lengthscale);
/**
 * @brief Build one block-averaged sensor footprint at a pose.
 * @details Grid cells inside the square bounding the circular FOV are grouped
 * by altitude-dependent resolution blocks, matching the reference sensor's
 * coarse measurement behavior.
 * @param grid Map grid.
 * @param pose Sensor east/north/altitude in metres.
 * @param angle Full field-of-view angle in degrees.
 * @param observation_variance Fixed variance, or negative to use the altitude model.
 * @return Sparse sensor rows and their independent variances.
 */
Sensor build_sensor(const Grid& grid, const Point& pose, double angle = 60.0,
                    double observation_variance = -1.0);
/**
 * @brief Approximate future measurements along a path under a bounded budget.
 * @details Samples path poses by `stride`, always includes the final waypoint,
 * and evenly subsamples sensor rows if their total exceeds `max_measurements`.
 * This trades a compact innovation matrix for some spatial detail.
 * @param grid Map grid.
 * @param path Ordered trajectory in metres.
 * @param angle Full field-of-view angle in degrees.
 * @param stride Path-pose interval used for sampling.
 * @param max_measurements Maximum number of returned measurement rows.
 * @param observation_variance Fixed variance, or negative to use the altitude model.
 * @return Combined sensor operator within the requested measurement budget.
 * @throws std::invalid_argument If stride or the measurement budget is nonpositive.
 */
Sensor future_sensor(const Grid& grid, const std::vector<Point>& path,
                     double angle = 60.0, int stride = lookahead_stride,
                     int max_measurements = lookahead_measurement_budget,
                     double observation_variance = -1.0);
/**
 * @brief Apply the sensor's block averages to a full-grid field.
 * @details Each sparse row is treated as a uniform averaging operator.
 * @param sensor Sparse sensor operator.
 * @param field One value per grid cell.
 * @return One averaged value per sensor row.
 */
Vector observe(const Sensor& sensor, const Vector& field);
/**
 * @brief Draw independent zero-mean noise using each sensor row's variance.
 * @details Uses one standard-normal draw per row and scales by that row's
 * standard deviation; the caller owns the generator to control reproducibility.
 * @param sensor Per-row variances define Gaussian scales.
 * @param generator Caller-owned random engine for reproducibility.
 * @return One independent noise value per sensor row.
 */
Vector sample_sensor_noise(const Sensor& sensor, std::mt19937_64& generator);
/**
 * @brief Select cells whose confidence bounds satisfy the utility threshold.
 * @details If no cell meets the requested bound, all cells are selected so the
 * optimizer does not receive a degenerate zero-utility objective.
 * @param mean Posterior mean.
 * @param covariance Posterior covariance.
 * @param options Confidence-bound direction, beta, and threshold.
 * @return Boolean importance mask in grid order.
 */
std::vector<bool> importance_mask(const Vector& mean, const Matrix& covariance,
                                  const PlannerOptions& options);
/**
 * @brief Sum expected marginal-variance reduction over important cells.
 * @details Uses the sensor innovation covariance and clamps numerical roundoff
 * to each cell's prior marginal variance.
 * @param covariance Prior covariance.
 * @param mask Cells contributing to utility.
 * @param sensor Candidate observation operator.
 * @return Sum of marginal-variance reduction over selected cells.
 */
double expected_reduction(const Matrix& covariance, const std::vector<bool>& mask,
                          const Sensor& sensor);
/**
 * @brief Apply a Gaussian linear-observation update in place.
 * @details Solves the innovation system rather than forming an inverse, updates
 * the posterior mean only when both optional pointers are supplied, then updates
 * covariance in row blocks to limit temporary allocations and restores symmetry.
 * @param covariance Prior covariance, replaced by the posterior covariance.
 * @param mean Optional prior mean; used only when `measurement` is non-null.
 * @param sensor Observation operator.
 * @param measurement Optional observed vector paired with `mean`.
 */
void condition(Matrix& covariance, Vector* mean, const Sensor& sensor, const Vector* measurement);
/**
 * @brief Clamp a candidate to the map safety inset and altitude interval.
 * @details Horizontal margins are two grid steps, independent of optimizer
 * penalty settings, so downstream routes retain a geometric boundary buffer.
 * @param grid Map geometry and horizontal inset.
 * @param point Candidate east/north/altitude point.
 * @param options Permitted altitude interval.
 * @return Bounded point.
 */
Point clip_point(const Grid& grid, const Point& point, const PlannerOptions& options);
/**
 * @brief Generate the deduplicated coarse-to-fine candidate lattice.
 * @details Nested layers trade uniform resolution for broad coverage and
 * progressively finer central choices.
 * @param grid Map geometry and spacing.
 * @param options Altitude range for lattice layers.
 * @return Unique candidate positions.
 */
std::vector<Point> lattice(const Grid& grid, const PlannerOptions& options);
/**
 * @brief Interpolate controls with a natural cubic spline parameterized by distance.
 * @details Coincident consecutive knots are removed because their zero chord
 * lengths make the spline system singular; an all-coincident path is returned
 * as its single unique point.
 * @param start Initial path point.
 * @param waypoints Ordered spline controls.
 * @param samples_per_segment Number of emitted samples per chord.
 * @return Interpolated path including the final knot.
 * @throws std::invalid_argument If samples per segment is not positive.
 */
std::vector<Point> spline(const Point& start, const std::vector<Point>& waypoints,
                          int samples_per_segment = spline_samples_per_segment);
/**
 * @brief Score a trajectory by safety violation and distance-normalized information gain.
 * @details Controls are clipped for geometric evaluation, while raw-control and
 * spline overshoot is retained as a penalty so the optimizer cannot hide an
 * unsafe proposal by clipping it before scoring.
 * @param grid Map geometry and safety bounds.
 * @param covariance Current covariance.
 * @param mask Cells included in information gain.
 * @param start Current position.
 * @param raw_waypoints Candidate controls before clipping.
 * @param options Sensor geometry and distance-cost settings.
 * @return Scalar objective; lower values are preferred.
 */
double trajectory_objective(const Grid& grid, const Matrix& covariance,
                            const std::vector<bool>& mask, const Point& start,
                            const std::vector<Point>& raw_waypoints, const PlannerOptions& options);
/**
 * @brief Greedily select a bounded route by information gain per travel cost.
 * @details Each chosen sensor conditions a private covariance copy before the
 * next selection, giving cheap sequential look-ahead without mutating observed
 * belief state. Cancellation returns an empty/incomplete route.
 * @param grid Candidate world grid.
 * @param mean Current posterior mean.
 * @param covariance Current posterior covariance.
 * @param start Initial position.
 * @param options Horizon, utility, sensor, and cancellation settings.
 * @return Ordered greedy controls, possibly empty when interrupted.
 */
std::vector<Point> grid_search(const Grid& grid, const Vector& mean, const Matrix& covariance,
                               const Point& start, const PlannerOptions& options);
/**
 * @brief Refine a greedy route with bounded CMA-ES while retaining its best candidate.
 * @details Fixed-altitude problems omit Z from the search dimension; boundary
 * transforms keep the optimizer's latent Gaussian unconstrained while all
 * evaluated waypoints remain inside flight bounds.
 * @param grid Map geometry and safety bounds.
 * @param mean Current posterior mean.
 * @param covariance Current posterior covariance.
 * @param start Initial position.
 * @param initial Greedy route used as the warm start and incumbent.
 * @param options Optimizer budget, seed, bounds, and cancellation predicate.
 * @return Best route found, or the unchanged initial route if refinement cannot start.
 */
std::vector<Point> refine(const Grid& grid, const Vector& mean, const Matrix& covariance,
                          const Point& start, const std::vector<Point>& initial,
                          const PlannerOptions& options);
/**
 * @brief Run greedy route selection followed by optional CMA-ES refinement.
 * @details Greedy search provides a feasible incumbent; refinement is skipped
 * when `options.refine` is false.
 * @param grid Map geometry.
 * @param mean Current posterior mean.
 * @param covariance Current posterior covariance.
 * @param start Initial position.
 * @param options Search and optimizer settings.
 * @return Ordered control points, or an empty route if cancelled during greedy search.
 */
std::vector<Point> plan(const Grid& grid, const Vector& mean, const Matrix& covariance,
                        const Point& start, const PlannerOptions& options);

}