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
 * @file compat.hpp
 * @brief Eigen-style Aigen values and build-selected planner aliases.
 * @details Aigen always uses aigenlib and can coexist with Eigen in one translation
 * unit. The planner aliases follow DIPLA_USE_EIGEN consistently across units.
 * Supports a small value API, not Eigen's general expression templates.
 */
#pragma once

#include "aigenlib.hpp"

#ifndef DIPLA_USE_EIGEN
/** Select Eigen when nonzero; the default planner backend is Aigen/aigenlib. */
#define DIPLA_USE_EIGEN 0
#endif

#if DIPLA_USE_EIGEN
#include <Eigen/Dense>
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace Aigen {

/**
 * @brief aigenlib storage with the planner's signed dimensions and finite predicate.
 * @details Public inheritance permits zero-copy calls into aigenlib kernels.
 */
class Matrix : public dipla::aigenlib::Matrix {
public:
    using dipla::aigenlib::Matrix::Matrix;
    /** @brief Accept an owning kernel result. @param value Matrix storage. */
    Matrix(dipla::aigenlib::Matrix value) : dipla::aigenlib::Matrix(std::move(value)) {}
    /** @brief Signed row count. @return Bounded rows. */
    int rows() const noexcept { return static_cast<int>(dipla::aigenlib::Matrix::rows()); }
    /** @brief Signed column count. @return Bounded columns. */
    int cols() const noexcept { return static_cast<int>(dipla::aigenlib::Matrix::cols()); }
    /** @brief Signed scalar count. @return Bounded scalar count. */
    int size() const noexcept { return static_cast<int>(dipla::aigenlib::Matrix::size()); }
    /** @brief Test every scalar. @return Whether finite. */
    bool allFinite() const noexcept { return finite(); }
    /** @brief Allocate zero storage. @param rows Rows. @param columns Columns. @return Zero matrix. */
    static Matrix Zero(std::size_t rows, std::size_t columns) { return Matrix(rows, columns); }
};

/**
 * @brief Bounded owning vector with the planner's small Eigen-style value API.
 * @details No expression objects or implicit unbounded allocations are exposed.
 */
class Vector : public dipla::aigenlib::Vector {
public:
    /** @brief Allocate initialized scalars. @param count Length. @param value Initial scalar. */
    explicit Vector(std::size_t count = 0, double value = 0.0)
        : dipla::aigenlib::Vector(checked_size(count), value) {}
    /** @brief Accept owning kernel output. @param values Scalars. */
    Vector(dipla::aigenlib::Vector values) : dipla::aigenlib::Vector(std::move(values)) { checked_size(dipla::aigenlib::Vector::size()); }
    /** @brief Signed vector length. @return Bounded length. */
    int size() const noexcept { return static_cast<int>(dipla::aigenlib::Vector::size()); }
    /** @brief Matrix-compatible row count. @return Length. */
    int rows() const noexcept { return size(); }
    /** @brief Matrix-compatible column count. @return One. */
    int cols() const noexcept { return 1; }
    /** @brief Checked matrix-style access. @param row Row. @param column Must be zero. @return Scalar. */
    double operator()(std::size_t row, std::size_t column) const {
        if (column != 0) throw std::out_of_range("Vector column must be zero");
        return at(row);
    }
    /** @brief Allocate zeros. @param count Length. @return Owning vector. */
    static Vector Zero(std::size_t count) { return Vector(count); }
    /** @brief Allocate a constant. @param count Length. @param value Scalar. @return Owning vector. */
    static Vector Constant(std::size_t count, double value) { return Vector(count, value); }
    /** @brief Produce a value, never an expression. @return Owning copy. */
    Vector eval() const { return *this; }
    /** @brief Validate every scalar. @return Whether finite. */
    bool allFinite() const {
        return std::all_of(begin(), end(), [](double value) { return std::isfinite(value); });
    }
    /** @brief Sum all entries. @return Sum. */
    double sum() const { return std::accumulate(begin(), end(), 0.0); }
    /** @brief Stable Euclidean length. @return Norm. */
    double norm() const {
        double result = 0.0;
        for (const double value : *this) result = std::hypot(result, value);
        return result;
    }
    /** @brief Add a same-sized vector. @param other Addend. @return This vector. */
    Vector& operator+=(const Vector& other) {
        if (size() != other.size()) throw std::invalid_argument("Vector dimensions differ");
        for (int index = 0; index < size(); ++index) (*this)[index] += other[index];
        return *this;
    }
    /** @brief Relative norm comparison. @param other Vector. @param tolerance Relative limit. @return Whether approximately equal. */
    bool isApprox(const Vector& other, double tolerance) const {
        if (size() != other.size() || !allFinite() || !other.allFinite() || !(tolerance >= 0.0)) return false;
        double error = 0.0;
        for (int index = 0; index < size(); ++index) error = std::hypot(error, (*this)[index] - other[index]);
        return error <= tolerance * std::min(norm(), other.norm());
    }
private:
    /** @brief Validate before allocating. @param count Requested length. @return Checked length. */
    static std::size_t checked_size(std::size_t count) {
        if (count > dipla::aigenlib::max_extent) throw std::length_error("Planner vector exceeds grid bound");
        return count;
    }
};

/** @brief Add owning vectors. @param left First. @param right Second. @return Sum. */
inline Vector operator+(Vector left, const Vector& right) { return left += right; }

/**
 * @brief aigenlib point with familiar initialization and finite-check spelling.
 * @details Arithmetic uses the underlying value operations without heap storage.
 */
class Point : public dipla::aigenlib::Point {
public:
    using dipla::aigenlib::Point::Point;
    /** @brief Accept the result of value arithmetic. @param point Coordinates. */
    Point(const dipla::aigenlib::Point& point) : dipla::aigenlib::Point(point) {}
    /** @brief Construct the origin. @return Zero coordinates. */
    static Point Zero() { return Point(0.0, 0.0, 0.0); }
    /** @brief Reset every coordinate to zero. */
    void setZero() noexcept { x() = y() = z() = 0.0; }
    /** @brief Validate coordinates. @return Whether finite. */
    bool allFinite() const noexcept { return finite(); }
    /** @brief Return a value, never an expression. @return This point. */
    Point eval() const { return *this; }
};

/** Eigen-style name for the bounded dynamic double matrix. */
using MatrixXd = Matrix;
/** Eigen-style name for the bounded dynamic double vector. */
using VectorXd = Vector;
/** Eigen-style name for the three-coordinate double point. */
using Vector3d = Point;

/**
 * @brief Owning, zero-initialized 4-by-4 column-major value matrix.
 * @details Storage is sixteen scalars on the stack; products return owning
 * values and support aliasing assignments without expression-lifetime issues.
 * This subset provides initialization, access, transpose, trace and products,
 * not decompositions or a general Eigen expression API.
 * @tparam Number Floating-point scalar type.
 */
template <class Number>
class Matrix4 {
    static_assert(std::is_floating_point_v<Number>, "Matrix4 needs floating-point scalars");
public:
    /** Eigen-compatible scalar type. */
    using Scalar = Number;
    /** @brief Row count. @return Four. */
    constexpr int rows() const noexcept { return 4; }
    /** @brief Column count. @return Four. */
    constexpr int cols() const noexcept { return 4; }
    /** @brief Scalar count. @return Sixteen. */
    constexpr int size() const noexcept { return 16; }
    /** @brief Checked mutable element. @param row Row. @param column Column. @return Scalar reference. */
    Scalar& operator()(int row, int column) { return values_[offset(row, column)]; }
    /** @brief Checked element. @param row Row. @param column Column. @return Scalar. */
    Scalar operator()(int row, int column) const { return values_[offset(row, column)]; }
    /** @brief Allocate a zero value. @return Zero matrix. */
    static Matrix4 Zero() { return {}; }
    /** @brief Construct a constant value. @param value Scalar. @return Constant matrix. */
    static Matrix4 Constant(Scalar value) {
        Matrix4 result;
        result.values_.fill(value);
        return result;
    }
    /** @brief Construct the identity. @return Identity matrix. */
    static Matrix4 Identity() {
        Matrix4 result;
        for (int index = 0; index < 4; ++index) result(index, index) = Scalar(1);
        return result;
    }
    /** @brief Validate all entries. @return Whether finite. */
    bool allFinite() const {
        return std::all_of(values_.begin(), values_.end(), [](Scalar value) { return std::isfinite(value); });
    }
    /** @brief Sum the diagonal. @return Trace. */
    Scalar trace() const {
        Scalar result = 0;
        for (int index = 0; index < 4; ++index) result += (*this)(index, index);
        return result;
    }
    /** @brief Return an owning transpose. @return Transposed matrix. */
    Matrix4 transpose() const {
        Matrix4 result;
        for (int row = 0; row < 4; ++row) {
            for (int column = 0; column < 4; ++column) result(row, column) = (*this)(column, row);
        }
        return result;
    }
    /** @brief Multiply two owning values. @param right Right operand. @return Product. */
    Matrix4 operator*(const Matrix4& right) const {
        Matrix4 result;
        for (int column = 0; column < 4; ++column) {
            for (int inner = 0; inner < 4; ++inner) {
                const Scalar factor = right(inner, column);
                for (int row = 0; row < 4; ++row) result(row, column) += (*this)(row, inner) * factor;
            }
        }
        return result;
    }
private:
    /** Zero-initialized, fixed-size column-major storage. */
    std::array<Scalar, 16> values_{};
    /** @brief Validate an element address. @param row Row. @param column Column. @return Flat offset. */
    static std::size_t offset(int row, int column) {
        if (row < 0 || row >= 4 || column < 0 || column >= 4) throw std::out_of_range("Matrix4 index");
        return static_cast<std::size_t>(column * 4 + row);
    }
};

/** Eigen-style 4-by-4 float matrix, with actual float storage/arithmetic. */
using Matrix4f = Matrix4<float>;
/** Eigen-style 4-by-4 double matrix. */
using Matrix4d = Matrix4<double>;

}

namespace aipp::math {
#if DIPLA_USE_EIGEN
/** Row-major Eigen matrix selected explicitly at build time. */
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
/** Eigen vector selected explicitly at build time. */
using VectorXd = Eigen::VectorXd;
/** Eigen three-coordinate point selected explicitly at build time. */
using Vector3d = Eigen::Vector3d;
/** Selected 4-by-4 float matrix. */
using Matrix4f = Eigen::Matrix4f;
/** Selected 4-by-4 double matrix. */
using Matrix4d = Eigen::Matrix4d;
/** Name of the compiled backend, independent of runtime configuration. */
inline constexpr const char* backend = "eigen";
/** Artifact-readable record retained by the executable's --build-info output. */
inline constexpr const char* build_info = "DIPLA_MATH_BACKEND=eigen";
#else
/** Selected dynamic double matrix. */
using MatrixXd = Aigen::MatrixXd;
/** Selected dynamic double vector. */
using VectorXd = Aigen::VectorXd;
/** Selected three-coordinate double point. */
using Vector3d = Aigen::Vector3d;
/** Selected 4-by-4 float matrix. */
using Matrix4f = Aigen::Matrix4f;
/** Selected 4-by-4 double matrix. */
using Matrix4d = Aigen::Matrix4d;
/** Name of the default compiled backend. */
inline constexpr const char* backend = "aigenlib";
/** Artifact-readable record retained by the executable's --build-info output. */
inline constexpr const char* build_info = "DIPLA_MATH_BACKEND=aigenlib";
#endif

/** Existing planner matrix spelling. */
using Matrix = MatrixXd;
/** Existing planner vector spelling. */
using Vector = VectorXd;
/** Existing planner point spelling. */
using Point = Vector3d;

}