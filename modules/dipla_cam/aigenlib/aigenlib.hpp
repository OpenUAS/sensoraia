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
 * @file aigenlib.hpp
 * @brief Bounded, owning numerical types and Gaussian-process operations.
 * @details No expression templates, BLAS, LAPACK, Eigen, or OpenCV dependency.
 * Operations allocate explicit bounded workspaces and throw on invalid input.
 */
#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace dipla::aigenlib {

/** Maximum matrix extent, matching the current dense GP grid ceiling. */
inline constexpr std::size_t max_extent = 2601;
/** Relative spectral cutoff shared with the Eigen reference implementation. */
inline constexpr double pseudoinverse_relative_cutoff = 1e-15;
/** Owning scalar vector; size validation belongs to the consuming operation. */
using Vector = std::vector<double>;

/**
 * @brief Three-dimensional value type for route arithmetic.
 * @details Coordinates use metres; norm uses overflow-resistant hypot.
 */
struct Point {
    std::array<double, 3> coordinates{}; /**< East, north, altitude. */
    /** @brief Construct a point. @param east East. @param north North. @param altitude Height. */
    Point(double east = 0.0, double north = 0.0, double altitude = 0.0);
    /** @brief Access east. @return Mutable coordinate. */
    double& x() noexcept;
    /** @brief Access east. @return Coordinate. */
    double x() const noexcept;
    /** @brief Access north. @return Mutable coordinate. */
    double& y() noexcept;
    /** @brief Access north. @return Coordinate. */
    double y() const noexcept;
    /** @brief Access altitude. @return Mutable coordinate. */
    double& z() noexcept;
    /** @brief Access altitude. @return Coordinate. */
    double z() const noexcept;
    /** @brief Checked coordinate access. @param index Coordinate 0..2. @return Mutable value. */
    double& operator[](std::size_t index);
    /** @brief Checked coordinate access. @param index Coordinate 0..2. @return Value. */
    double operator[](std::size_t index) const;
    /** @brief Euclidean length. @return Nonnegative length. */
    double norm() const noexcept;
    /** @brief Check every coordinate. @return True if finite. */
    bool finite() const noexcept;
    /** @brief Add coordinates. @param other Addend. @return This point. */
    Point& operator+=(const Point& other) noexcept;
    /** @brief Subtract coordinates. @param other Subtrahend. @return This point. */
    Point& operator-=(const Point& other) noexcept;
};

/** @brief Add points. @param left First. @param right Second. @return Sum. */
Point operator+(Point left, const Point& right) noexcept;
/** @brief Subtract points. @param left First. @param right Second. @return Difference. */
Point operator-(Point left, const Point& right) noexcept;
/** @brief Scale coordinates. @param point Input. @param scale Factor. @return Scaled point. */
Point operator*(Point point, double scale) noexcept;
/** @brief Scale coordinates. @param scale Factor. @param point Input. @return Scaled point. */
Point operator*(double scale, Point point) noexcept;
/** @brief Divide coordinates. @param point Input. @param divisor Finite nonzero divisor. @return Quotient. */
Point operator/(Point point, double divisor);
/** @brief Exact equality. @param left First. @param right Second. @return All coordinates equal. */
bool operator==(const Point& left, const Point& right) noexcept;
/** @brief Exact inequality. @param left First. @param right Second. @return Any coordinate differs. */
bool operator!=(const Point& left, const Point& right) noexcept;

/**
 * @brief Owning, contiguous, row-major double matrix.
 * @details Each extent is at most max_extent. Construction checks before
 * multiplying dimensions. Element/row access is checked, including release
 * builds. A borrowed row pointer is valid until assignment, move, or destruction;
 * callers must bound column accesses. Moved-from matrices are empty 0-by-0.
 */
class Matrix {
public:
    /** @brief Construct an empty matrix. */
    Matrix() = default;
    /** @brief Allocate initialized storage. @param rows Row count. @param columns Column count. @param value Initial scalar. */
    Matrix(std::size_t rows, std::size_t columns, double value = 0.0);
    /** @brief Deep copy. @param other Source. */
    Matrix(const Matrix& other) = default;
    /** @brief Deep assignment with strong exception guarantee. @param other Source. @return This matrix. */
    Matrix& operator=(const Matrix& other);
    /** @brief Transfer storage. @param other Source, emptied afterwards. */
    Matrix(Matrix&& other) noexcept;
    /** @brief Transfer storage. @param other Source, emptied afterwards. @return This matrix. */
    Matrix& operator=(Matrix&& other) noexcept;
    /** @brief Row count. @return Rows. */
    std::size_t rows() const noexcept;
    /** @brief Column count. @return Columns. */
    std::size_t cols() const noexcept;
    /** @brief Allocated scalar count. @return Rows times columns. */
    std::size_t size() const noexcept;
    /** @brief Checked access. @param row Row. @param column Column. @return Mutable scalar. */
    double& operator()(std::size_t row, std::size_t column);
    /** @brief Checked access. @param row Row. @param column Column. @return Scalar. */
    double operator()(std::size_t row, std::size_t column) const;
    /** @brief Borrow a row. @param row Row. @return Pointer, null for zero columns. */
    double* row_data(std::size_t row);
    /** @brief Borrow a row. @param row Row. @return Pointer, null for zero columns. */
    const double* row_data(std::size_t row) const;
    /** @brief Validate all stored scalars. @return True if finite, including empty matrices. */
    bool finite() const noexcept;
    /** @brief Sum a square matrix's diagonal. @return Trace. */
    double trace() const;

private:
    std::size_t rows_ = 0; /**< Row extent. */
    std::size_t columns_ = 0; /**< Column extent. */
    Vector values_; /**< Contiguous owning storage. */
};

/** Factorization selected for a solve. */
enum class SolveMethod {
    cholesky, /**< Positive-definite triangular factorization. */
    pseudoinverse /**< Signed spectral inverse with null-mode truncation. */
};
/**
 * @brief Solve a finite symmetric system with multiple right-hand sides.
 * @details Cholesky is attempted first. On failure, bounded cyclic Jacobi sweeps
 * compute a spectral pseudoinverse with abs(lambda) > 1e-15 * max(abs(lambda)).
 * Negative eigenvalues retain their sign. Input is scaled before factoring;
 * neither input is mutated. Symmetry is checked relative to matrix scale and
 * the lower triangle defines the system within that tolerance. Zero systems
 * produce zero solutions. Throws on invalid shape, nonfinite data, numerical
 * overflow, or nonconvergence; never silently adds jitter. Not a general SVD.
 * @param system Square coefficient matrix, including 0-by-0.
 * @param right Matrix with system.rows() rows, possibly zero columns.
 * @param method Optional output reporting the selected factorization.
 * @return Owning solution with the same shape as right.
 */
Matrix solve_symmetric(const Matrix& system, const Matrix& right, SolveMethod* method = nullptr);

/**
 * @brief Sparse uniform-averaging measurement operator.
 * @details Each row must be nonempty and contain valid cell indices. Repeated
 * indices retain their averaging weight. Row count and individual row lengths
 * are bounded by max_extent, not by the planner's 64-row look-ahead budget.
 */
struct Sensor {
    std::vector<std::vector<int>> rows; /**< Cell indices in each measurement. */
    Vector variances; /**< Finite nonnegative independent noise variances. */
};

/**
 * @brief Construct a symmetric Matern-3/2 covariance over supplied positions.
 * @details Kernel parameters are supplied by the caller, not selected by this
 * library. Coordinate and length-scale units must agree. One triangle is
 * evaluated and mirrored. Rejects nonfinite positions/parameters.
 * @param positions Grid-cell coordinates, at most max_extent.
 * @param sigma2 Positive marginal variance.
 * @param lengthscale Positive correlation length in the coordinates' units.
 * @return Dense symmetric prior.
 */
Matrix matern32(const std::vector<Point>& positions, double sigma2, double lengthscale);
/**
 * @brief Project covariance through sparse averaging rows: B = H P.
 * @details Requires a square finite covariance. Checks shape/sensor before
 * allocation and detects nonfinite projected values in the sensor's accessed
 * rows, without rescanning the whole grid per candidate. Unused rows are not
 * inspected; conditioning checks the complete covariance before mutation.
 * @param sensor Averaging operator.
 * @param covariance Square cell covariance.
 * @return Measurement-by-cell cross covariance.
 */
Matrix project(const Sensor& sensor, const Matrix& covariance);
/**
 * @brief Construct innovation covariance S = H P H^T + R from B = H P.
 * @details Validates rows and variances against the projected cell count.
 * @param sensor Averaging operator.
 * @param projected Measurement-by-cell cross covariance.
 * @return Measurement-by-measurement innovation covariance.
 */
Matrix innovation(const Sensor& sensor, const Matrix& projected);
/** @brief Average field values through a sensor. @param sensor Operator. @param field Finite cell values. @return Measurements. */
Vector observe(const Sensor& sensor, const Vector& field);
/**
 * @brief Sum the masked posterior diagonal reduction without modifying belief.
 * @details Solves S X = B and clamps each selected B-column dot X-column to
 * [0, max(0, Pii)], matching the production utility calculation. Checks the
 * accessed projection and selected diagonal, not unused covariance entries.
 * @param covariance Square finite cell covariance.
 * @param mask One selection bit per cell.
 * @param sensor Averaging operator.
 * @return Expected trace reduction.
 */
double expected_reduction(const Matrix& covariance, const std::vector<bool>& mask, const Sensor& sensor);
/**
 * @brief Condition a GP in place with optional measurement assimilation.
 * @details Both mean and measurement must be supplied, or both null for
 * look-ahead. All shapes, indices, noise and input finiteness are checked before
 * mutation. Workspace is two m-by-n matrices, small solver storage and one
 * 4 KiB scratch tile per active lane, never an n-by-n product temporary.
 * Large multi-row updates use at most three lanes and join before symmetry
 * restoration; a thread-creation failure falls back to remaining serial work.
 * A single-row observation fuses updating and symmetry restoration without scratch;
 * both triangle calculations and their averaging order are preserved exactly.
 * Arithmetic overflow throws; after mutation begins it does
 * not roll back the belief. Concurrent access requires caller synchronization.
 * @param covariance Square finite covariance, updated in place.
 * @param mean Optional finite cell mean, updated in place.
 * @param sensor Averaging operator.
 * @param measurement Optional finite measurement values.
 */
void condition(Matrix& covariance, Vector* mean, const Sensor& sensor, const Vector* measurement);

}