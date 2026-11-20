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
 * @file dipla_gp_belief_planner.cpp
 * @brief Finite-grid GP observations, uncertainty updates, and reference route scoring.
 * @details
 * Despite the historical filename, this file does not train the ONNX denoiser.
 * It maintains and queries a Gaussian-process (GP) belief about a scalar field,
 * builds synthetic sensor operators, and provides the greedy/spline machinery
 * used by the explicitly selected comparison planner.
 *
 * @par The shared linear-observation model
 * For N grid cells and M sensor rows, each row of H uniformly averages its
 * listed cells. We store those indices rather than a mostly zero M-by-N matrix.
 * Sigma is the dense N-by-N covariance; R is diagonal independent sensor noise.
 * @code{.text}
 * P = H * Sigma                         (M by N)
 * S = H * Sigma * H^T + R                (M by M)
 * B = solve(S, P)                       (M by N)
 * posterior_mean = mean + B^T * (measurement - H * mean)
 * posterior_covariance = Sigma - P^T * B
 * @endcode
 * Expected information gain needs the covariance update but not future measured
 * values. That distinction lets look-ahead reduce hypothetical uncertainty
 * without handing the planner the hidden field's answer sheet.
 *
 * @par When changing numerical behavior
 * The Eigen branch below and the default aigenlib kernels implement the same
 * valid-input model. Keep operator weights, grid order, solver cutoffs, and
 * reduction rules aligned, and run differential tests when changing either.
 * Public contracts stay in the header; implementation notes here explain why.
 *
 * @warning Grid construction caps N, not whole-process memory. One 2601-by-2601
 * double covariance is about 51.6 MiB; hypothetical copies, M-by-N workspaces,
 * and solver allocations are additional. Some helpers assume valid inputs
 * rather than checking every precondition in both numerical backends.
 * @see dipla_gp_belief_planner.hpp Public geometry, sensor, and planner API.
 */
#include "dipla_gp_belief_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace aipp {
namespace {
constexpr double grid_interval_tolerance = 1e-9;
constexpr double minimum_spline_chord_m = 1e-9;
constexpr int eigen_covariance_block_rows = 64;
constexpr double sensor_base_variance = 0.01;
constexpr double sensor_variance_increment = 0.03;
constexpr double sensor_noise_height_scale_m = 30.0;
constexpr double sensor_noise_saturation_factor = 4.0;
}
#if DIPLA_USE_EIGEN
namespace {

/**
 * @brief Form measurement-to-grid cross-covariance without materializing H.
 * @details Averaging covariance rows applies the same weights as observe().
 * Summing instead of averaging would silently give larger footprint blocks
 * more signal simply because they contain more cells, changing the sensor model.
 * @param[in] sensor Sparse uniform-average rows.
 * @param[in] covariance Symmetric N-by-N prior or posterior covariance.
 * @return M-by-N projected covariance P = H * Sigma.
 * @pre Every sensor row is nonempty and contains valid grid indices.
 * @note Sparse H does not make P sparse; budget M * N doubles for this result.
 */
Matrix project(const Sensor& sensor, const Matrix& covariance) {
    Matrix projected = Matrix::Zero(static_cast<int>(sensor.rows.size()), covariance.cols());
    for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
        for (const int index : sensor.rows[row]) {
            projected.row(static_cast<int>(row)) += covariance.row(index);
        }
        projected.row(static_cast<int>(row)) /= static_cast<double>(sensor.rows[row].size());
    }
    return projected;
}

/**
 * @brief Form the measurement-space innovation covariance.
 * @details Applies the same row-average operator on both sides of the prior
 * covariance and adds independent measurement variance to the diagonal. Shared
 * field cells correlate measurements through Sigma, not through extra entries
 * in R; changing to correlated sensor noise requires changing this construction.
 * @param[in] sensor Operator and one finite, nonnegative noise variance per row.
 * @param[in] projected M-by-N cross-covariance from project().
 * @return M-by-M innovation system S = H * Sigma * H^T + R.
 * @pre Sensor indices and dimensions match projected, with no empty sensor rows.
 */
Matrix innovation(const Sensor& sensor, const Matrix& projected) {
    const int count = static_cast<int>(sensor.rows.size());
    Matrix result = Matrix::Zero(count, count);
    for (int row = 0; row < count; ++row) {
        for (int column = 0; column < count; ++column) {
            for (const int index : sensor.rows[row]) {
                result(row, column) += projected(column, index);
            }
            result(row, column) /= static_cast<double>(sensor.rows[row].size());
        }
        result(row, row) += sensor.variances[row];
    }
    return result;
}

/**
 * @brief Solve a symmetric positive-definite system with a stable fallback.
 * @details Cholesky is the fast path for the expected positive-definite sensor
 * system. Near-singular duplicate/overlapping measurements can defeat LLT, so a
 * self-adjoint eigensolve applies a scale-relative pseudoinverse cutoff instead
 * of amplifying numerical null modes.
 * @param[in] system Symmetric M-by-M innovation system.
 * @param[in] right M-by-N right-hand sides, solved together.
 * @return Owning solution with the same shape as right.
 * @pre Inputs are finite and dimensionally compatible.
 * @throws std::runtime_error If the fallback eigendecomposition fails.
 * @note The fallback retains the sign of significant eigenvalues. It is a
 * symmetric pseudoinverse, not a repair that forces an indefinite system to be
 * positive definite. Do not add jitter or discard negative modes without
 * treating that as a numerical-model change and testing it explicitly.
 */
Matrix solve_spd(const Matrix& system, const Matrix& right) {
    Eigen::LLT<Matrix> decomposition(system);
    if (decomposition.info() == Eigen::Success) {
        return decomposition.solve(right);
    }
    Eigen::SelfAdjointEigenSolver<Matrix> fallback(system);
    if (fallback.info() != Eigen::Success) {
        throw std::runtime_error("Cannot factor sensor innovation covariance");
    }
    // Scale the null-space threshold to the system spectrum; a fixed absolute
    // epsilon would behave differently when kernel amplitude is rescaled.
    const double cutoff = dipla::aigenlib::pseudoinverse_relative_cutoff * fallback.eigenvalues().cwiseAbs().maxCoeff();
    Vector inverse = fallback.eigenvalues().unaryExpr([cutoff](double value) {
        return std::abs(value) > cutoff ? 1.0 / value : 0.0;
    });
    return fallback.eigenvectors() * inverse.asDiagonal() * fallback.eigenvectors().transpose() * right;
}

}
#endif

/**
 * @details Validate geometry before converting dimensions to integers or
 * allocating any dense GP matrix. The four-step width check accommodates the
 * planner's two-step inset on both sides; the cell cap limits quadratic storage.
 * @note The small tolerance in interval counting avoids losing a final sample
 * to floating-point noise near an exact spacing multiple. Effective upper
 * bounds are then the last lattice coordinates, not unsampled requested edges.
 * @warning The cell cap is not a whole-board 512 MB admission check. Enlarging
 * it requires budgeting covariance copies and solver workspaces, not just one matrix.
 */
Grid::Grid(double xminimum, double xmaximum, double yminimum, double ymaximum, double spacing)
    : xmin(xminimum), xmax(xmaximum), ymin(yminimum), ymax(ymaximum), step(spacing) {
    if (!std::isfinite(xmin) || !std::isfinite(xmax) || !std::isfinite(ymin) ||
        !std::isfinite(ymax) || !std::isfinite(step) || step <= 0.0 ||
        xmax - xmin <= 4.0 * step || ymax - ymin <= 4.0 * step) {
        throw std::invalid_argument("Grid must be finite, positive, and wider than the safety margins");
    }
    const double columns = std::floor((xmax - xmin) / step + grid_interval_tolerance) + 1.0;
    const double rows = std::floor((ymax - ymin) / step + grid_interval_tolerance) + 1.0;
    if (columns * rows > dipla::aigenlib::max_extent) {
        throw std::invalid_argument("Dense GP is limited to 2601 cells for the 512 MB Pi memory budget");
    }
    width = static_cast<int>(columns);
    height = static_cast<int>(rows);
    xmax = x(width - 1);
    ymax = y(height - 1);
}

/** @details All field vectors and covariance axes share row * width + column ordering. */
int Grid::size() const { return width * height; }
/** @details Use the grid's affine mapping everywhere; an array index is not a metre coordinate. */
double Grid::x(int column) const { return xmin + column * step; }
/** @details North is the row coordinate, consistent with field flattening and sensor indices. */
double Grid::y(int row) const { return ymin + row * step; }
/**
 * @details ceil(offset - 0.5) deliberately sends exact half-cell ties to the
 * lower index, independent of the process rounding mode. This is the research
 * grid rule, not the neural input marker's separate NumPy-style rounding rule.
 * @throws std::invalid_argument For nonfinite coordinates.
 */
int Grid::nearest_x(double value) const {
    if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite grid coordinate");
    const double offset = (value - xmin) / step;
    if (offset <= 0) return 0;
    if (offset >= width - 1) return width - 1;
    return static_cast<int>(std::ceil(offset - 0.5));
}
/**
 * @details Apply the same lower-index tie rule as nearest_x() so axis changes
 * do not change which cell a half-grid pose selects.
 * @throws std::invalid_argument For nonfinite coordinates.
 */
int Grid::nearest_y(double value) const {
    if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite grid coordinate");
    const double offset = (value - ymin) / step;
    if (offset <= 0) return 0;
    if (offset >= height - 1) return height - 1;
    return static_cast<int>(std::ceil(offset - 0.5));
}

/**
 * @details The angle is the full FOV in degrees. Dividing by 360 both halves
 * it and converts to radians; using 180 here would double the angular footprint.
 * @pre altitude is finite and nonnegative; angle is finite and between 0 and 180 degrees.
 * @note This ground-plane geometry assumes a level footprint. build_sensor()
 * selects its square bounds, not a ray-traced or circular visibility mask.
 */
double fov_radius(double altitude, double angle) {
    return altitude * std::tan(angle * std::acos(-1.0) / 360.0);
}
/**
 * @details Higher altitude coarsens one measurement into averages of wider
 * grid blocks. These thresholds belong to the research observation model,
 * not the ONNX tensor resolution; changing them changes the experiment's H.
 */
int resolution_block_size(double altitude) { return altitude <= 20.0 ? 1 : altitude <= 30.0 ? 2 : 4; }
/**
 * @details Return variance, not standard deviation: 0.01 at minimum_altitude,
 * approaching 0.04 from below at increasing height. The exponential preserves
 * that saturation rather than making high-altitude noise grow without limit.
 * @warning This formula is not clamped below its reference altitude and can
 * become negative there. The 2 m application explicitly supplies variance 0.01;
 * do not extrapolate this law to low flight or take sqrt of a negative result.
 * @see build_sensor Fixed-variance override for a separately declared sensor experiment.
 */
double noise_model(double altitude, double minimum_altitude) {
    return sensor_base_variance + sensor_variance_increment *
        (1.0 - std::exp(-std::log(sensor_noise_saturation_factor) * (altitude - minimum_altitude) / sensor_noise_height_scale_m));
}

/**
 * @details The Matern-3/2 kernel is sigma2 * (1 + q) * exp(-q), with
 * q = sqrt(3) * distance / lengthscale. sigma2 is already the marginal variance;
 * squaring it again would change both uncertainty scale and inferred utility.
 * At zero distance the diagonal is sigma2; nearby cells share information
 * according to lengthscale measured in the same metres as the grid.
 * @note This initializes covariance only. The application owns the prior mean;
 * do not infer its value from a bare GP initializer in another caller.
 */
Matrix initialize_covariance(const Grid& grid, double sigma2, double lengthscale) {
    if (!(sigma2 > 0.0) || !(lengthscale > 0.0) || !std::isfinite(sigma2) || !std::isfinite(lengthscale)) {
        throw std::invalid_argument("GP kernel parameters must be finite and positive");
    }
    Matrix kernel(grid.height, grid.width);
    for (int north = 0; north < grid.height; ++north) {
        for (int east = 0; east < grid.width; ++east) {
            const double distance = std::hypot(east * grid.step, north * grid.step);
            const double scaled = std::sqrt(3.0) * distance / lengthscale;
            kernel(north, east) = sigma2 * (1.0 + scaled) * std::exp(-scaled);
        }
    }
    Matrix covariance(grid.size(), grid.size());
    for (int row = 0; row < grid.size(); ++row) {
        for (int column = 0; column < grid.size(); ++column) {
            covariance(row, column) = kernel(std::abs(row / grid.width - column / grid.width),
                                             std::abs(row % grid.width - column % grid.width));
        }
    }
    return covariance;
}

/**
 * @details Use the square bounding the FOV, then group included cells by
 * globally anchored resolution blocks. Blocks stay anchored to grid row/column
 * zero, not to the moving footprint edge; otherwise tiny pose changes would
 * redefine the measurement operator even over the same physical cells.
 * @note Edge blocks average only their included cells. Do not normalize them
 * by the nominal block area or add a circular-distance rejection: either would
 * describe a different observation model from this reference implementation.
 * @pre Geometry and pose are finite, altitude/FOV are valid, and the chosen
 * fixed or altitude-derived observation variance is finite and nonnegative.
 * @see observe The same row-average weights must be used when generating measurements.
 */
Sensor build_sensor(const Grid& grid, const Point& pose, double angle, double observation_variance) {
    Sensor sensor;
    const double radius = fov_radius(pose.z(), angle);
    const int block = resolution_block_size(pose.z());
    // Column-first traversal creates rows in first-encounter order. The map
    // finds an existing block; its sorted keys do NOT reorder the emitted rows.
    // Keep traversal stable because it also fixes measurement and noise ordering.
    std::map<std::pair<int, int>, std::size_t> lookup;
    for (int column = 0; column < grid.width; ++column) {
        if (grid.x(column) < pose.x() - radius || grid.x(column) > pose.x() + radius) continue;
        for (int row = 0; row < grid.height; ++row) {
            if (grid.y(row) < pose.y() - radius || grid.y(row) > pose.y() + radius) continue;
            const auto key = std::make_pair(row / block, column / block);
            const auto inserted = lookup.emplace(key, sensor.rows.size());
            if (inserted.second) {
                sensor.rows.emplace_back();
                // A nonnegative override bypasses the altitude law, including
                // zero for explicit noiseless tests. Each ROW has one variance;
                // it is not divided by cell count as though every cell were
                // a separately noisy measurement before averaging.
                sensor.variances.push_back(observation_variance >= 0.0 ? observation_variance : noise_model(pose.z()));
            }
            sensor.rows[inserted.first->second].push_back(row * grid.width + column);
        }
    }
    return sensor;
}

/**
 * @details Approximate a route's future observations without querying truth.
 * Snap each sampled XY pose to the research grid before constructing its H;
 * the supplied altitude still controls footprint, resolution, and default noise.
 * @note Endpoint inclusion happens before row limiting. The final bounded
 * operator evenly retains rows, not entire footprints, so a small budget may
 * omit some cells or rows from any particular pose, including the endpoint.
 * @warning max_measurements bounds the returned innovation dimension, not
 * peak construction storage: the combined operator is built before subsampling.
 * @pre Path values, sensor geometry, and selected variances satisfy build_sensor()'s preconditions.
 */
Sensor future_sensor(const Grid& grid, const std::vector<Point>& path, double angle,
                     int stride, int max_measurements, double observation_variance) {
    Sensor combined;
    if (stride <= 0 || max_measurements <= 0) throw std::invalid_argument("Invalid sensor sampling budget");
    std::vector<std::size_t> selected;
    for (std::size_t index = 0; index < path.size(); index += static_cast<std::size_t>(stride)) {
        selected.push_back(index);
    }
    // Preserve the destination even when stride sampling would omit it; the
    // end of a route is a meaningful observation opportunity, not a midpoint.
    if (!path.empty() && path[selected.back()] != path.back()) selected.push_back(path.size() - 1);
    for (const auto index : selected) {
        Point pose = path[index];
        pose.x() = grid.x(grid.nearest_x(pose.x()));
        pose.y() = grid.y(grid.nearest_y(pose.y()));
        // Keep repeated/overlapping rows: distinct future observations have
        // independent noise even when their averaging operators coincide.
        Sensor sensor = build_sensor(grid, pose, angle, observation_variance);
        for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
            combined.rows.push_back(std::move(sensor.rows[row]));
            combined.variances.push_back(sensor.variances[row]);
        }
    }
    if (static_cast<int>(combined.rows.size()) <= max_measurements) return combined;
    // The innovation solve scales cubically with measurement count. Evenly
    // retain rows across the route instead of allowing long paths to create an
    // unbounded dense measurement system.
    Sensor limited;
    for (int index = 0; index < max_measurements; ++index) {
        const auto selected_row = max_measurements == 1 ? 0 : static_cast<std::size_t>(
            static_cast<double>(combined.rows.size() - 1) * index / (max_measurements - 1));
        limited.rows.push_back(std::move(combined.rows[selected_row]));
        limited.variances.push_back(combined.variances[selected_row]);
    }
    return limited;
}

/**
 * @details Apply H identically to hidden truth and predicted mean; the latter
 * gives the expected measurement used by the GP residual. Noise is deliberately
 * separate so scoring can use the operator without drawing synthetic evidence.
 * @pre Rows are nonempty and indices address finite entries in field.
 * @see sample_sensor_noise Independent noise is added by the observation owner.
 */
Vector observe(const Sensor& sensor, const Vector& field) {
    Vector result(static_cast<int>(sensor.rows.size()));
    for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
        double sum = 0.0;
        for (const int index : sensor.rows[row]) sum += field[index];
        result[static_cast<int>(row)] = sum / static_cast<double>(sensor.rows[row].size());
    }
    return result;
}

/**
 * @details Scale a standard normal by sqrt(variance), not variance itself.
 * Using the same per-row variance in innovation() makes sampled measurements
 * and GP likelihood describe one experiment rather than two incompatible sensors.
 * @pre Each sensor row has one finite, nonnegative variance.
 * @note The caller owns random-stream lifetime. Even a zero-variance row draws
 * a normal here; changing row order or draw count changes later noise samples.
 * Equal seeds alone do not promise NumPy or cross-library bitwise equivalence.
 */
Vector sample_sensor_noise(const Sensor& sensor, std::mt19937_64& generator) {
    std::normal_distribution<double> normal(0.0, 1.0);
    Vector noise(static_cast<int>(sensor.rows.size()));
    for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
        noise[static_cast<int>(row)] = normal(generator) * std::sqrt(sensor.variances[row]);
    }
    return noise;
}

/**
 * @details Select utility using mean plus/minus beta times marginal standard
 * deviation. The lower-bound mode asks whether a cell could lie below threshold;
 * the upper-bound mode asks whether it could lie above. Preserve the inequality
 * directions when adapting a task: swapping them changes what exploration values.
 * @note If every cell fails, score the whole field instead of handing search a
 * flat zero-gain objective. This is an explicit exploration policy, not a claim
 * that every cell now meets the requested confidence-bound test.
 * @pre Mean, covariance, and utility settings are finite and dimensionally compatible.
 */
std::vector<bool> importance_mask(const Vector& mean, const Matrix& covariance, const PlannerOptions& options) {
    std::vector<bool> mask(static_cast<std::size_t>(mean.size()));
    bool any = false;
    for (int index = 0; index < mean.size(); ++index) {
        const double uncertainty = options.beta * std::sqrt(std::max(0.0, covariance(index, index)));
        mask[index] = options.lower_confidence_bound ? mean[index] - uncertainty <= options.threshold
                                                    : mean[index] + uncertainty >= options.threshold;
        any = any || mask[index];
    }
    if (!any) std::fill(mask.begin(), mask.end(), true);
    return mask;
}

/**
 * @details For each selected cell, the corresponding column of P and B yields
 * the diagonal reduction in Sigma - P^T * B. Sum only those diagonals rather
 * than allocating a full posterior just to obtain one candidate's utility.
 * Future measurement values are unnecessary: linear-Gaussian covariance updates
 * depend on the observation operator and noise, not the unknown observed value.
 * @note Clamp each computed reduction to [0, prior marginal variance] to contain
 * round-off. This is a trace-reduction utility, not entropy or a field-value reward.
 * @pre Covariance is finite and symmetric; mask has N entries and sensor rows
 * and variances are valid for the same N-cell domain.
 */
double expected_reduction(const Matrix& covariance, const std::vector<bool>& mask, const Sensor& sensor) {
#if DIPLA_USE_EIGEN
    if (sensor.rows.empty()) return 0.0;
    const Matrix projected = project(sensor, covariance);
    const Matrix solved = solve_spd(innovation(sensor, projected), projected);
    double gain = 0.0;
    for (int index = 0; index < covariance.cols(); ++index) {
        if (mask[index]) {
            gain += std::clamp(projected.col(index).dot(solved.col(index)), 0.0,
                               std::max(0.0, covariance(index, index)));
        }
    }
    return gain;
#else
    // Keep the default backend's checked, bounded-storage kernel as the owner
    // of its numerical implementation; do not grow a third GP algorithm here.
    return dipla::aigenlib::expected_reduction(covariance, mask, sensor);
#endif
}

/**
 * @details Compute P and solve S * B = P once, then reuse B for both posterior
 * mean and covariance. Never form a general explicit inverse merely to multiply
 * it away again; the measurement-space solve is the smaller numerical problem.
 * @pre Supply mean and measurement together with compatible finite dimensions,
 * or pass both as nullptr for hypothetical covariance-only conditioning.
 * @pre Covariance and sensor describe the same valid finite grid domain.
 * @warning The update mutates storage in place; it is not a transactional
 * operation with rollback after an arithmetic or allocation failure. Symmetry
 * restoration fixes triangle disagreement, not arbitrary loss of definiteness.
 * @see expected_reduction Same covariance algebra without materializing its update.
 */
void condition(Matrix& covariance, Vector* mean, const Sensor& sensor, const Vector* measurement) {
    if (sensor.rows.empty()) return;
#if DIPLA_USE_EIGEN
    const Matrix projected = project(sensor, covariance);
    const Matrix system = innovation(sensor, projected);
    const Matrix solved = solve_spd(system, projected);
    // Null mean/measurement is used by look-ahead planning: it updates only
    // simulated covariance, without pretending an unobserved value is known.
    // There is no crystal ball hiding in the optional measurement pointer.
    if (mean && measurement) {
        const Vector residual = *measurement - observe(sensor, *mean);
        *mean += solved.transpose() * residual;
    }
    // Bound the temporary transpose/product workspace. A full 2601x2601 dense
    // temporary would add roughly 51 MiB to an already memory-heavy GP update.
    // noalias is valid because projected/solved are separate owning matrices;
    // do not replace them with views into covariance without revisiting aliasing.
    for (int first = 0; first < covariance.rows(); first += eigen_covariance_block_rows) {
        const int count = std::min(eigen_covariance_block_rows, static_cast<int>(covariance.rows()) - first);
        covariance.middleRows(first, count).noalias() -= projected.middleCols(first, count).transpose() * solved;
    }
    // Roundoff can make the independently updated triangles differ slightly;
    // restore symmetry before later Cholesky/eigen solves depend on it.
    for (int row = 0; row < covariance.rows(); ++row) {
        for (int column = 0; column < row; ++column) {
            covariance(row, column) = covariance(column, row) =
                0.5 * (covariance(row, column) + covariance(column, row));
        }
    }
#else
    // aigenlib applies the same update with its own one-row scratch and tiled
    // symmetry pass. Keep reference comparisons when changing operation order.
    dipla::aigenlib::condition(covariance, mean, sensor, measurement);
#endif
}

/**
 * @details Apply the comparison planner's two-grid-step XY inset and declared
 * Z interval. This constrains a control point, not every point of a cubic spline
 * between controls; trajectory_objective() separately scores spline overshoot.
 * @pre Coordinates and altitude bounds are finite, with options.zmin <= options.zmax.
 * @warning A geometric target inset is not an aircraft geofence or an arrival test.
 */
Point clip_point(const Grid& grid, const Point& point, const PlannerOptions& options) {
        return {std::clamp(point.x(), grid.xmin + options.margin_cells * grid.step, grid.xmax - options.margin_cells * grid.step),
            std::clamp(point.y(), grid.ymin + options.margin_cells * grid.step, grid.ymax - options.margin_cells * grid.step),
            std::clamp(point.z(), options.zmin, options.zmax)};
}

/**
 * @details Combine three nested XY layers at progressively higher requested
 * altitudes. Snap XY and Z to their respective grid-step lattices, then remove
 * exact duplicate points so a shared location cannot consume two greedy slots.
 * Fixed-height settings collapse Z layers naturally without inventing a Z search.
 * @note Enumeration order breaks equal-score ties in grid_search(). Changing
 * it can change a deterministic route even when the candidate set is unchanged.
 * @pre Altitude bounds are finite, ordered, and their step count fits int.
 */
std::vector<Point> lattice(const Grid& grid, const PlannerOptions& options) {
    std::vector<Point> candidates;
    // Broad 9-by-9 coverage comes first, then smaller 3-by-3 and 2-by-2
    // central layers. This is a compact candidate policy, not an exhaustive
    // uniformly spaced 3-D search; layer density and height change together.
    struct Layer { int side_count; double inset_fraction; };
    static constexpr Layer layers[] = {{9, 0.0}, {3, 0.30}, {2, 0.60}};
    const int altitude_count = static_cast<int>(std::floor((options.zmax - options.zmin) / grid.step + grid_interval_tolerance));
    for (std::size_t layer = 0; layer < std::size(layers); ++layer) {
        const double inset = layers[layer].inset_fraction;
        const int side_count = layers[layer].side_count;
        const double half_x = 0.5 * (grid.xmax - grid.xmin - 2.0 * options.margin_cells * grid.step) * (1.0 - inset);
        const double half_y = 0.5 * (grid.ymax - grid.ymin - 2.0 * options.margin_cells * grid.step) * (1.0 - inset);
        const double lower_x = 0.5 * (grid.xmin + grid.xmax) - half_x;
        const double lower_y = 0.5 * (grid.ymin + grid.ymax) - half_y;
        const double requested_z = options.zmin + layer * 0.5 * (options.zmax - options.zmin);
        const int altitude_index = std::clamp(static_cast<int>(std::ceil((requested_z - options.zmin) / grid.step - 0.5)), 0, altitude_count);
        for (int column = 0; column < side_count; ++column) {
            for (int row = 0; row < side_count; ++row) {
                Point point(grid.x(grid.nearest_x(lower_x + 2.0 * half_x * column / (side_count - 1))),
                            grid.y(grid.nearest_y(lower_y + 2.0 * half_y * row / (side_count - 1))),
                            options.zmin + altitude_index * grid.step);
                if (std::find(candidates.begin(), candidates.end(), point) == candidates.end()) candidates.push_back(point);
            }
        }
    }
    return candidates;
}

/**
 * @details Parameterize by 3-D chord length rather than control index so unequal
 * waypoint spacing affects interpolation geometrically. Natural endpoint
 * conditions set endpoint second derivatives to zero; an in-place tridiagonal
 * solve obtains the interior derivatives for all three coordinates together.
 * @note Consecutive chords no longer than 1e-9 m are omitted. This avoids a
 * singular system but intentionally differs from retaining coincident controls
 * with an artificial minimum chord length in another reference implementation.
 * @warning This is a sampled scoring path, not the flight controller's path
 * implementation, a constant-speed parameterization, or a bound on continuous
 * spline overshoot between emitted samples.
 * @pre Input coordinates are finite and the requested sample storage is practical.
 */
std::vector<Point> spline(const Point& start, const std::vector<Point>& waypoints, int samples_per_segment) {
    if (samples_per_segment <= 0) throw std::invalid_argument("Spline sampling must be positive");
    std::vector<Point> knots{start};
    std::vector<double> distances;
    // Zero-length chords make the natural-spline system singular. Removing
    // consecutive duplicates preserves the geometric path and its endpoint.
    for (const auto& point : waypoints) {
        const double length = (point - knots.back()).norm();
        if (length > minimum_spline_chord_m) { distances.push_back(length); knots.push_back(point); }
    }
    if (knots.size() == 1) return knots;
    const std::size_t count = knots.size();
    std::vector<double> diagonal(count, 1.0);
    std::vector<Point> second(count, Point::Zero());
    // Solve the tridiagonal natural-spline equations in place: endpoint second
    // derivatives remain zero, avoiding a general dense solve per objective.
    for (std::size_t index = 1; index + 1 < count; ++index) {
        diagonal[index] = 2.0 * (distances[index - 1] + distances[index]);
        second[index] = 6.0 * ((knots[index + 1] - knots[index]) / distances[index]
                              - (knots[index] - knots[index - 1]) / distances[index - 1]);
        if (index > 1) {
            const double ratio = distances[index - 1] / diagonal[index - 1];
            diagonal[index] -= ratio * distances[index - 1];
            second[index] -= ratio * second[index - 1];
        }
    }
    // Back-substitute only interior knots. The unsigned loop stops above zero;
    // changing the condition to index >= 0 would underflow rather than terminate.
    for (std::size_t index = count - 2; index > 0; --index) {
        second[index] = (second[index] - distances[index] * second[index + 1]) / diagonal[index];
    }
    std::vector<Point> path;
    path.reserve(distances.size() * static_cast<std::size_t>(samples_per_segment) + 1);
    for (std::size_t segment = 0; segment < distances.size(); ++segment) {
        const double length = distances[segment];
        // Emit each segment's start, not its endpoint; the next segment owns
        // that shared knot. Append the final endpoint once after all segments.
        for (int sample = 0; sample < samples_per_segment; ++sample) {
            const double fraction = static_cast<double>(sample) / samples_per_segment;
            const double left = 1.0 - fraction;
            path.push_back(left * knots[segment] + fraction * knots[segment + 1]
                + length * length / 6.0 * ((left * left * left - left) * second[segment]
                                       + (fraction * fraction * fraction - fraction) * second[segment + 1]));
        }
    }
    path.push_back(knots.back());
    return path;
}

/**
 * @details Minimize boundary violation minus information gain per travel cost.
 * Raw controls contribute a penalty even though scoring uses clipped controls;
 * otherwise many out-of-bounds proposals would become the same cheap clipped
 * path. Sampled spline overshoot adds a penalty of its own.
 * @note Travel cost is the clipped control polyline's length, not spline arc
 * length. Its grid-step floor prevents division by zero and infinite reward for
 * hovering on coincident controls. Sensor stride and the 64-row budget bound the
 * scoring solve at the cost of an approximate future-observation operator.
 * @warning Soft penalties do not certify continuous-path containment. Changing
 * samples_per_segment, stride, or row budget changes the objective, not just speed.
 * @pre Inputs satisfy spline(), clip_point(), and expected_reduction() contracts.
 */
double trajectory_objective(const Grid& grid, const Matrix& covariance, const std::vector<bool>& mask,
                            const Point& start, const std::vector<Point>& raw_waypoints, const PlannerOptions& options) {
    std::vector<Point> clipped;
    clipped.reserve(raw_waypoints.size());
    double overshoot = 0.0;
    // Keep a penalty for the un-clipped proposal. Scoring only its clipped
    // geometry would make out-of-bounds controls appear deceptively optimal.
    // Each point contributes its largest coordinate violation, not the sum
    // over axes. Preserve that norm when comparing optimizer objective values.
    const auto violation = [&](const Point& point) {
        return std::max({point.x() - (grid.xmax - options.margin_cells * grid.step), grid.xmin + options.margin_cells * grid.step - point.x(),
                 point.y() - (grid.ymax - options.margin_cells * grid.step), grid.ymin + options.margin_cells * grid.step - point.y(),
                         point.z() - options.zmax, options.zmin - point.z(), 0.0});
    };
    double distance = 0.0;
    Point previous = start;
    for (const auto& point : raw_waypoints) {
        overshoot += violation(point);
        clipped.push_back(clip_point(grid, point, options));
        distance += (clipped.back() - previous).norm();
        previous = clipped.back();
    }
    const auto path = spline(start, clipped);
    for (const auto& point : path) overshoot += violation(point);
    const double gain = expected_reduction(covariance, mask,
        future_sensor(grid, path, options.angle_of_view, lookahead_stride, lookahead_measurement_budget, options.observation_variance));
    return overshoot - gain / std::pow(std::max(distance, grid.step), options.cost_exponent);
}

/**
 * @details Choose a gain-per-distance winner, condition a private hypothetical
 * covariance, then score the next choice against reduced uncertainty. Reusing
 * the original covariance every round would repeatedly reward already-covered
 * regions instead of planning complementary observations.
 * @note The mean remains the observed mean throughout look-ahead: future sensor
 * values are unknown. Importance masks can still change as simulated uncertainty
 * shrinks. One extra dense covariance is the principal storage cost of this search.
 * @warning Cancellation returns an empty vector, not the partial prefix already
 * selected. A short nonempty route can still result from exhausted candidates;
 * the application must enforce its required eight-point publication contract.
 * @pre Geometry, posterior, utility, and sensor settings satisfy the called helpers.
 * @see PlannerOptions::keep_planning Cooperative polling, not preemptive interruption.
 */
std::vector<Point> grid_search(const Grid& grid, const Vector& mean, const Matrix& covariance,
                               const Point& start, const PlannerOptions& options) {
    auto available = lattice(grid, options);
    // Greedy look-ahead needs a hypothetical posterior after each selected
    // sensor, but must not mutate the caller's observed posterior.
    Matrix simulated = covariance;
    Point pose = start;
    std::vector<Point> waypoints;
    for (int round = 0; round < options.horizon && !available.empty(); ++round) {
        if (options.keep_planning && !options.keep_planning()) return {};
        const auto mask = importance_mask(mean, simulated, options);
        double best_score = -std::numeric_limits<double>::infinity();
        std::size_t best_index = 0;
        Sensor best_sensor;
        for (std::size_t index = 0; index < available.size(); ++index) {
            if (options.keep_planning && !options.keep_planning()) return {};
            Sensor sensor = future_sensor(grid, {available[index]}, options.angle_of_view, 1, lookahead_measurement_budget,
                                          options.observation_variance);
            const double score = expected_reduction(simulated, mask, sensor)
                / std::pow(std::max((available[index] - pose).norm(), grid.step), options.cost_exponent);
            // Strict comparison keeps the first enumerated candidate on ties.
            // Deterministic ordering is part of reproducible greedy behavior.
            if (score > best_score) {
                best_score = score;
                best_index = index;
                best_sensor = std::move(sensor);
            }
        }
        pose = available[best_index];
        waypoints.push_back(pose);
        // Do not choose the same lattice point twice, and do not condition the
        // caller's real posterior on an observation we have only imagined.
        available.erase(available.begin() + static_cast<std::ptrdiff_t>(best_index));
        condition(simulated, nullptr, best_sensor, nullptr);
    }
    return waypoints;
}

}