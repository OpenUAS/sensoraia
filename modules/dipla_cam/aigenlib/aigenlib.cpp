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

/*
 * Portions of Jacobi rotation adapted from OpenCV's BSD-3-Clause JacobiImpl_:
 * Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 * Copyright (C) 2009, Willow Garage Inc., all rights reserved.
 * Full original notice and modification details are in NOTICE.md.
 */

/**
 * @file aigenlib.cpp
 * @brief Bounded numerical storage and scaled small-system solvers.
 * @details Contiguous right-hand-side row updates avoid strided inner loops.
 * The fallback uses deterministic cyclic rotations rather than cached pivots.
 */
#include "aigenlib.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace dipla::aigenlib {
namespace {
constexpr unsigned jacobi_sweep_limit = 80;
constexpr double jacobi_epsilon_multiplier = 0.25;
constexpr double symmetry_epsilon_multiplier = 64.0;

/** @brief Check extents before multiplication. @param rows Rows. @param columns Columns. @return Scalar count. */
std::size_t element_count(std::size_t rows, std::size_t columns) {
    if (rows > max_extent || columns > max_extent ||
        (columns != 0 && rows > Vector{}.max_size() / columns)) {
        throw std::length_error("aigenlib matrix exceeds the dense GP storage bound");
    }
    return rows * columns;
}

/** @brief Reject nonfinite vector values. @param values Input. @return Whether finite. */
bool finite_vector(const Vector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

/** @brief Validate the operator before indexed kernels run. @param sensor Operator. @param cells Cell count. */
void validate_sensor(const Sensor& sensor, std::size_t cells) {
    if (cells > max_extent || sensor.rows.size() > max_extent || sensor.variances.size() != sensor.rows.size()) {
        throw std::invalid_argument("Invalid aigenlib sensor dimensions");
    }
    for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
        if (sensor.rows[row].empty() || sensor.rows[row].size() > max_extent ||
            !std::isfinite(sensor.variances[row]) || sensor.variances[row] < 0.0) {
            throw std::invalid_argument("Invalid aigenlib sensor row or variance");
        }
        for (const int index : sensor.rows[row]) {
            if (index < 0 || static_cast<std::size_t>(index) >= cells) throw std::out_of_range("aigenlib sensor cell index");
        }
    }
}

/** @brief Check GP covariance shape. @param covariance Input. */
void validate_covariance_shape(const Matrix& covariance) {
    if (covariance.rows() != covariance.cols()) {
        throw std::invalid_argument("Invalid aigenlib covariance");
    }
}

/** @brief Factor the lower triangle in place. @param factor Scaled symmetric workspace. @return Whether positive definite. */
bool cholesky(Matrix& factor) {
    for (std::size_t row = 0; row < factor.rows(); ++row) {
        double* current = factor.row_data(row);
        for (std::size_t column = 0; column <= row; ++column) {
            const double* previous = factor.row_data(column);
            double value = current[column];
            for (std::size_t inner = 0; inner < column; ++inner) {
                value -= current[inner] * previous[inner];
            }
            if (column == row) {
                if (!(value > 0.0) || !std::isfinite(value)) return false;
                current[column] = std::sqrt(value);
            } else {
                current[column] = value / previous[column];
                if (!std::isfinite(current[column])) return false;
            }
        }
    }
    return true;
}

/** @brief Apply two triangular solves. @param factor Lower Cholesky factor. @param result Scaled RHS, replaced by solution. */
void triangular_solve(const Matrix& factor, Matrix& result) {
    for (std::size_t row = 0; row < factor.rows(); ++row) {
        double* destination = result.row_data(row);
        const double* coefficients = factor.row_data(row);
        for (std::size_t inner = 0; inner < row; ++inner) {
            const double* source = result.row_data(inner);
            for (std::size_t column = 0; column < result.cols(); ++column) {
                destination[column] -= coefficients[inner] * source[column];
            }
        }
        for (std::size_t column = 0; column < result.cols(); ++column) destination[column] /= coefficients[row];
    }
    for (std::size_t remaining = factor.rows(); remaining > 0; --remaining) {
        const std::size_t row = remaining - 1;
        double* destination = result.row_data(row);
        for (std::size_t inner = row + 1; inner < factor.rows(); ++inner) {
            const double coefficient = factor(inner, row);
            const double* source = result.row_data(inner);
            for (std::size_t column = 0; column < result.cols(); ++column) destination[column] -= coefficient * source[column];
        }
        const double diagonal = factor(row, row);
        for (std::size_t column = 0; column < result.cols(); ++column) destination[column] /= diagonal;
    }
}

/**
 * @brief Diagonalize a scaled symmetric matrix with cyclic Jacobi rotations.
 * @details The stable hypot rotation follows OpenCV JacobiImpl_; see NOTICE.md.
 * Eigenvectors are columns. At most 80 sweeps, with an explicit convergence
 * check, avoid returning an unfinished factorization as a valid result.
 * @param diagonal Workspace, replaced by a near-diagonal matrix.
 * @param vectors Output orthonormal eigenvector matrix.
 */
void jacobi(Matrix& diagonal, Matrix& vectors) {
    for (std::size_t index = 0; index < vectors.rows(); ++index) vectors(index, index) = 1.0;
    for (unsigned sweep = 0; sweep <= jacobi_sweep_limit; ++sweep) {
        double largest = 0.0;
        double scale = 0.0;
        for (std::size_t row = 0; row < diagonal.rows(); ++row) {
            scale = std::max(scale, std::abs(diagonal(row, row)));
            for (std::size_t column = row + 1; column < diagonal.cols(); ++column) {
                largest = std::max(largest, std::abs(diagonal(row, column)));
            }
        }
        const double tolerance = jacobi_epsilon_multiplier * std::numeric_limits<double>::epsilon() * std::max(scale, 1.0);
        if (largest <= tolerance) return;
        if (sweep == jacobi_sweep_limit) break;
        for (std::size_t first = 0; first < diagonal.rows(); ++first) {
            for (std::size_t second = first + 1; second < diagonal.cols(); ++second) {
                const double off = diagonal(first, second);
                if (std::abs(off) <= tolerance) continue;
                const double delta = 0.5 * (diagonal(second, second) - diagonal(first, first));
                double tangent = off / (std::abs(delta) + std::hypot(off, delta));
                if (delta < 0.0) tangent = -tangent;
                const double cosine = 1.0 / std::hypot(1.0, tangent);
                const double sine = tangent * cosine;
                diagonal(first, first) -= tangent * off;
                diagonal(second, second) += tangent * off;
                diagonal(first, second) = diagonal(second, first) = 0.0;
                for (std::size_t index = 0; index < diagonal.rows(); ++index) {
                    if (index != first && index != second) {
                        const double before_first = diagonal(index, first);
                        const double before_second = diagonal(index, second);
                        diagonal(index, first) = diagonal(first, index) = cosine * before_first - sine * before_second;
                        diagonal(index, second) = diagonal(second, index) = sine * before_first + cosine * before_second;
                    }
                    const double before_first = vectors(index, first);
                    const double before_second = vectors(index, second);
                    vectors(index, first) = cosine * before_first - sine * before_second;
                    vectors(index, second) = sine * before_first + cosine * before_second;
                }
            }
        }
    }
    throw std::runtime_error("aigenlib symmetric eigensolver did not converge");
}

}

Point::Point(double east, double north, double altitude) : coordinates{east, north, altitude} {}
double& Point::x() noexcept { return coordinates[0]; }
double Point::x() const noexcept { return coordinates[0]; }
double& Point::y() noexcept { return coordinates[1]; }
double Point::y() const noexcept { return coordinates[1]; }
double& Point::z() noexcept { return coordinates[2]; }
double Point::z() const noexcept { return coordinates[2]; }
double& Point::operator[](std::size_t index) { return coordinates.at(index); }
double Point::operator[](std::size_t index) const { return coordinates.at(index); }
double Point::norm() const noexcept { return std::hypot(std::hypot(x(), y()), z()); }
bool Point::finite() const noexcept { return std::isfinite(x()) && std::isfinite(y()) && std::isfinite(z()); }
Point& Point::operator+=(const Point& other) noexcept {
    for (std::size_t index = 0; index < coordinates.size(); ++index) coordinates[index] += other[index];
    return *this;
}
Point& Point::operator-=(const Point& other) noexcept {
    for (std::size_t index = 0; index < coordinates.size(); ++index) coordinates[index] -= other[index];
    return *this;
}
Point operator+(Point left, const Point& right) noexcept { return left += right; }
Point operator-(Point left, const Point& right) noexcept { return left -= right; }
Point operator*(Point point, double scale) noexcept {
    for (double& coordinate : point.coordinates) coordinate *= scale;
    return point;
}
Point operator*(double scale, Point point) noexcept { return point * scale; }
Point operator/(Point point, double divisor) {
    if (divisor == 0.0 || !std::isfinite(divisor)) throw std::invalid_argument("Invalid aigenlib point divisor");
    for (double& coordinate : point.coordinates) coordinate /= divisor;
    return point;
}
bool operator==(const Point& left, const Point& right) noexcept { return left.coordinates == right.coordinates; }
bool operator!=(const Point& left, const Point& right) noexcept { return !(left == right); }

Matrix::Matrix(std::size_t rows, std::size_t columns, double value)
    : rows_(rows), columns_(columns), values_(element_count(rows, columns), value) {}
Matrix& Matrix::operator=(const Matrix& other) {
    if (this != &other) {
        Matrix replacement(other);
        *this = std::move(replacement);
    }
    return *this;
}
Matrix::Matrix(Matrix&& other) noexcept
    : rows_(std::exchange(other.rows_, 0)), columns_(std::exchange(other.columns_, 0)), values_(std::move(other.values_)) {}
Matrix& Matrix::operator=(Matrix&& other) noexcept {
    if (this != &other) {
        values_ = std::move(other.values_);
        rows_ = std::exchange(other.rows_, 0);
        columns_ = std::exchange(other.columns_, 0);
    }
    return *this;
}
std::size_t Matrix::rows() const noexcept { return rows_; }
std::size_t Matrix::cols() const noexcept { return columns_; }
std::size_t Matrix::size() const noexcept { return values_.size(); }
double& Matrix::operator()(std::size_t row, std::size_t column) {
    if (column >= columns_) throw std::out_of_range("aigenlib matrix column");
    return row_data(row)[column];
}
double Matrix::operator()(std::size_t row, std::size_t column) const {
    if (column >= columns_) throw std::out_of_range("aigenlib matrix column");
    return row_data(row)[column];
}
double* Matrix::row_data(std::size_t row) {
    if (row >= rows_) throw std::out_of_range("aigenlib matrix row");
    return columns_ == 0 ? nullptr : values_.data() + row * columns_;
}
const double* Matrix::row_data(std::size_t row) const {
    if (row >= rows_) throw std::out_of_range("aigenlib matrix row");
    return columns_ == 0 ? nullptr : values_.data() + row * columns_;
}
bool Matrix::finite() const noexcept {
    return std::all_of(values_.begin(), values_.end(), [](double value) { return std::isfinite(value); });
}
double Matrix::trace() const {
    if (rows_ != columns_) throw std::invalid_argument("aigenlib trace requires a square matrix");
    double result = 0.0;
    for (std::size_t index = 0; index < rows_; ++index) result += (*this)(index, index);
    return result;
}

Matrix solve_symmetric(const Matrix& system, const Matrix& right, SolveMethod* method) {
    if (system.rows() != system.cols() || right.rows() != system.rows() || !system.finite() || !right.finite()) {
        throw std::invalid_argument("Invalid aigenlib symmetric system");
    }
    double scale = 0.0;
    for (std::size_t row = 0; row < system.rows(); ++row) {
        for (std::size_t column = 0; column < system.cols(); ++column) scale = std::max(scale, std::abs(system(row, column)));
    }
    Matrix scaled(system.rows(), system.cols());
    for (std::size_t row = 0; row < system.rows(); ++row) {
        for (std::size_t column = 0; column <= row; ++column) {
            const double lower = scale == 0.0 ? 0.0 : system(row, column) / scale;
            const double upper = scale == 0.0 ? 0.0 : system(column, row) / scale;
            if (std::abs(lower - upper) > symmetry_epsilon_multiplier * std::numeric_limits<double>::epsilon()) {
                throw std::invalid_argument("aigenlib system is not symmetric");
            }
            scaled(row, column) = scaled(column, row) = lower;
        }
    }
    Matrix factor = scaled;
    Matrix result(right.rows(), right.cols());
    if (cholesky(factor)) {
        for (std::size_t row = 0; row < right.rows(); ++row) {
            for (std::size_t column = 0; column < right.cols(); ++column) result(row, column) = right(row, column) / scale;
        }
        triangular_solve(factor, result);
        if (method) *method = SolveMethod::cholesky;
    } else {
        Matrix vectors(system.rows(), system.cols());
        jacobi(scaled, vectors);
        double largest = 0.0;
        for (std::size_t index = 0; index < system.rows(); ++index) largest = std::max(largest, std::abs(scaled(index, index)));
        Vector coefficients(right.cols());
        for (std::size_t mode = 0; mode < system.rows(); ++mode) {
            const double eigenvalue = scaled(mode, mode);
            if (std::abs(eigenvalue) <= pseudoinverse_relative_cutoff * largest) continue;
            std::fill(coefficients.begin(), coefficients.end(), 0.0);
            for (std::size_t row = 0; row < right.rows(); ++row) {
                const double weight = vectors(row, mode);
                const double* source = right.row_data(row);
                for (std::size_t column = 0; column < right.cols(); ++column) coefficients[column] += weight * source[column];
            }
            for (double& coefficient : coefficients) coefficient = (coefficient / eigenvalue) / scale;
            for (std::size_t row = 0; row < result.rows(); ++row) {
                double* destination = result.row_data(row);
                const double weight = vectors(row, mode);
                for (std::size_t column = 0; column < result.cols(); ++column) destination[column] += weight * coefficients[column];
            }
        }
        if (method) *method = SolveMethod::pseudoinverse;
    }
    if (!result.finite()) throw std::overflow_error("Nonfinite aigenlib symmetric solution");
    return result;
}

Matrix matern32(const std::vector<Point>& positions, double sigma2, double lengthscale) {
    if (positions.size() > max_extent || !(sigma2 > 0.0) || !(lengthscale > 0.0) ||
        !std::isfinite(sigma2) || !std::isfinite(lengthscale) ||
        !std::all_of(positions.begin(), positions.end(), [](const Point& point) { return point.finite(); })) {
        throw std::invalid_argument("Invalid aigenlib Matern parameters");
    }
    Matrix result(positions.size(), positions.size());
    for (std::size_t row = 0; row < positions.size(); ++row) {
        for (std::size_t column = 0; column <= row; ++column) {
            const double distance = (positions[row] - positions[column]).norm();
            const double scaled = std::sqrt(3.0) * (distance / lengthscale);
            const double value = std::isinf(scaled) ? 0.0 : sigma2 * ((1.0 + scaled) * std::exp(-scaled));
            result(row, column) = result(column, row) = value;
        }
    }
    return result;
}

Matrix project(const Sensor& sensor, const Matrix& covariance) {
    validate_covariance_shape(covariance);
    validate_sensor(sensor, covariance.rows());
    Matrix result(sensor.rows.size(), covariance.cols());
    for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
        double* destination = result.row_data(row);
        for (const int index : sensor.rows[row]) {
            const double* source = covariance.row_data(static_cast<std::size_t>(index));
            for (std::size_t column = 0; column < covariance.cols(); ++column) destination[column] += source[column];
        }
        const double count = static_cast<double>(sensor.rows[row].size());
        for (std::size_t column = 0; column < covariance.cols(); ++column) destination[column] /= count;
    }
    if (!result.finite()) throw std::overflow_error("Nonfinite aigenlib covariance projection");
    return result;
}

Matrix innovation(const Sensor& sensor, const Matrix& projected) {
    validate_sensor(sensor, projected.cols());
    if (projected.rows() != sensor.rows.size() || !projected.finite()) throw std::invalid_argument("Invalid aigenlib projection");
    Matrix result(sensor.rows.size(), sensor.rows.size());
    for (std::size_t row = 0; row < result.rows(); ++row) {
        for (std::size_t column = 0; column < result.cols(); ++column) {
            double value = 0.0;
            const double* source = projected.row_data(column);
            for (const int index : sensor.rows[row]) value += source[index];
            result(row, column) = value / static_cast<double>(sensor.rows[row].size());
        }
        result(row, row) += sensor.variances[row];
    }
    if (!result.finite()) throw std::overflow_error("Nonfinite aigenlib innovation covariance");
    return result;
}

Vector observe(const Sensor& sensor, const Vector& field) {
    validate_sensor(sensor, field.size());
    if (!finite_vector(field)) throw std::invalid_argument("Nonfinite aigenlib field");
    Vector result(sensor.rows.size());
    for (std::size_t row = 0; row < sensor.rows.size(); ++row) {
        for (const int index : sensor.rows[row]) result[row] += field[index];
        result[row] /= static_cast<double>(sensor.rows[row].size());
    }
    if (!finite_vector(result)) throw std::overflow_error("Nonfinite aigenlib observation");
    return result;
}

double expected_reduction(const Matrix& covariance, const std::vector<bool>& mask, const Sensor& sensor) {
    validate_covariance_shape(covariance);
    validate_sensor(sensor, covariance.rows());
    if (mask.size() != covariance.rows()) throw std::invalid_argument("Invalid aigenlib importance mask");
    if (sensor.rows.empty()) return 0.0;
    const Matrix projected = project(sensor, covariance);
    const Matrix solved = solve_symmetric(innovation(sensor, projected), projected);
    double gain = 0.0;
    for (std::size_t column = 0; column < covariance.cols(); ++column) {
        if (!mask[column]) continue;
        const double variance = covariance(column, column);
        if (!std::isfinite(variance)) throw std::invalid_argument("Nonfinite aigenlib selected variance");
        double reduction = 0.0;
        for (std::size_t row = 0; row < projected.rows(); ++row) reduction += projected(row, column) * solved(row, column);
        if (!std::isfinite(reduction)) throw std::overflow_error("Nonfinite aigenlib variance reduction");
        gain += std::clamp(reduction, 0.0, std::max(0.0, variance));
    }
    if (!std::isfinite(gain)) throw std::overflow_error("Nonfinite aigenlib information gain");
    return gain;
}

void condition(Matrix& covariance, Vector* mean, const Sensor& sensor, const Vector* measurement) {
    validate_covariance_shape(covariance);
    if (!covariance.finite()) throw std::invalid_argument("Nonfinite aigenlib covariance");
    validate_sensor(sensor, covariance.rows());
    if ((mean == nullptr) != (measurement == nullptr) ||
        (mean && (mean->size() != covariance.rows() || measurement->size() != sensor.rows.size() ||
                  !finite_vector(*mean) || !finite_vector(*measurement)))) {
        throw std::invalid_argument("Invalid aigenlib GP measurement or mean");
    }
    if (sensor.rows.empty()) return;
    const Matrix projected = project(sensor, covariance);
    const Matrix solved = solve_symmetric(innovation(sensor, projected), projected);
    Vector updated_mean;
    if (mean) {
        Vector residual = observe(sensor, *mean);
        for (std::size_t row = 0; row < residual.size(); ++row) residual[row] = (*measurement)[row] - residual[row];
        updated_mean = *mean;
        for (std::size_t row = 0; row < solved.rows(); ++row) {
            const double* source = solved.row_data(row);
            for (std::size_t column = 0; column < solved.cols(); ++column) updated_mean[column] += source[column] * residual[row];
        }
        if (!finite_vector(updated_mean)) throw std::overflow_error("Nonfinite aigenlib posterior mean");
    }
    constexpr std::size_t tile_size = 32;
    if (projected.rows() == 1) {
        const double* projection = projected.row_data(0);
        const double* solution = solved.row_data(0);
        for (std::size_t row = 0; row < covariance.rows(); ++row) {
            covariance(row, row) -= 0.0 + projection[row] * solution[row];
        }
        for (std::size_t row_begin = 0; row_begin < covariance.rows(); row_begin += tile_size) {
            const std::size_t row_end = std::min(row_begin + tile_size, covariance.rows());
            for (std::size_t column_begin = 0; column_begin <= row_begin; column_begin += tile_size) {
                const std::size_t column_end = std::min(column_begin + tile_size, covariance.cols());
                for (std::size_t row = row_begin; row < row_end; ++row) {
                    double* destination = covariance.row_data(row);
                    double* opposite = covariance.row_data(column_begin);
                    for (std::size_t column = column_begin; column < std::min(column_end, row); ++column) {
                        const double lower = destination[column] - (0.0 + projection[row] * solution[column]);
                        const double upper = opposite[row] - (0.0 + projection[column] * solution[row]);
                        const double average = 0.5 * lower + 0.5 * upper;
                        destination[column] = opposite[row] = average;
                        opposite += covariance.cols();
                    }
                }
            }
        }
    } else {
        constexpr std::size_t update_rows = 8;
        constexpr std::size_t update_columns = 64;
        const auto update = [&](std::size_t first_row, std::size_t last_row) {
            double scratch[update_rows][update_columns];
            for (std::size_t row_begin = first_row; row_begin < last_row; row_begin += update_rows) {
                const std::size_t row_count = std::min(update_rows, last_row - row_begin);
                for (std::size_t column_begin = 0; column_begin < covariance.cols(); column_begin += update_columns) {
                    const std::size_t column_count = std::min(update_columns, covariance.cols() - column_begin);
                    for (std::size_t row = 0; row < row_count; ++row)
                        std::fill_n(scratch[row], column_count, 0.0);
                    for (std::size_t inner = 0; inner < projected.rows(); ++inner) {
                        const double* weights = projected.row_data(inner) + row_begin;
                        const double* source = solved.row_data(inner) + column_begin;
                        for (std::size_t row = 0; row < row_count; ++row) {
                            const double weight = weights[row];
                            for (std::size_t column = 0; column < column_count; ++column)
                                scratch[row][column] += weight * source[column];
                        }
                    }
                    for (std::size_t row = 0; row < row_count; ++row) {
                        double* destination = covariance.row_data(row_begin + row) + column_begin;
                        for (std::size_t column = 0; column < column_count; ++column)
                            destination[column] -= scratch[row][column];
                    }
                }
            }
        };
        if (covariance.rows() >= 1024 && projected.rows() >= 16) {
            constexpr std::size_t lane_count = 3;
            std::array<std::thread, lane_count - 1> workers;
            std::array<std::exception_ptr, lane_count> errors{};
            const auto run_lane = [&](std::size_t lane) {
                try { update(covariance.rows() * lane / lane_count, covariance.rows() * (lane + 1) / lane_count); }
                catch (...) { errors[lane] = std::current_exception(); }
            };
            std::size_t launched = 0;
            try {
                for (; launched < workers.size(); ++launched)
                    workers[launched] = std::thread(run_lane, launched);
            } catch (...) {}
            for (std::size_t lane = launched; lane < lane_count; ++lane) run_lane(lane);
            for (auto& worker : workers) if (worker.joinable()) worker.join();
            for (const auto& error : errors) if (error) std::rethrow_exception(error);
        } else update(0, covariance.rows());
        for (std::size_t row_begin = 0; row_begin < covariance.rows(); row_begin += tile_size) {
            const std::size_t row_end = std::min(row_begin + tile_size, covariance.rows());
            for (std::size_t column_begin = 0; column_begin <= row_begin; column_begin += tile_size) {
                const std::size_t column_end = std::min(column_begin + tile_size, covariance.cols());
                for (std::size_t row = row_begin; row < row_end; ++row) {
                    double* destination = covariance.row_data(row);
                    for (std::size_t column = column_begin; column < std::min(column_end, row); ++column) {
                        double* opposite = covariance.row_data(column);
                        const double average = 0.5 * destination[column] + 0.5 * opposite[row];
                        destination[column] = opposite[row] = average;
                    }
                }
            }
        }
    }
    if (!covariance.finite()) throw std::overflow_error("Nonfinite aigenlib posterior covariance");
    if (mean) mean->swap(updated_mean);
}

}