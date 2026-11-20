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
 * @file image_framing.h
 * @brief Crop, resample, and rotate BGRA images into the requested camera frame.
 * @details Framing is shared by rendering and metadata generation: returning
 * the exact crop geometry lets callers derive matching intrinsics. A single
 * scale on both axes preserves aspect ratio, while reusable buffers avoid a
 * second full image allocation when the input can be viewed directly.
 */
#ifndef PANGEA_IMAGE_FRAMING_H
#define PANGEA_IMAGE_FRAMING_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#if defined(__SSE2__)
#include <emmintrin.h>
#elif defined(__aarch64__)
#include <arm_neon.h>
#endif
#include <array>
#include <stdexcept>
#include <vector>

namespace pangea_survey {
/**
 * @brief Validate encoded-image dimensions before allocating pixel buffers.
 * @param[in] width Image width in pixels.
 * @param[in] height Image height in pixels.
 * @throws std::runtime_error If dimensions exceed per-axis or total-pixel bounds.
 */
inline void validateJpegSize(int width, int height) {
    if (width < 1 || height < 1 || width > 16384 || height > 16384 ||
        static_cast<std::int64_t>(width) * height > 64 * 1024 * 1024)
        throw std::runtime_error("JPEG width/height must be integers 1-16384, with at most 64 megapixels");
}


/**
 * @brief Produce a bounded crop/resample view over a source BGRA image.
 * @details The view may borrow the source pixels for an exact crop or point to
 * this object's reusable output buffer after resampling/rotation. In the latter
 * case, the bytes remain valid only until the next call to @ref apply.
 */
class ImageFraming {
    struct Sample {
        int first, second;
        float weight;
    };
    int sourceWidth_ = 0, sourceHeight_ = 0, width_ = 0, height_ = 0, inset_ = 0;
    double rotationBound_ = 0;
    std::vector<Sample> columns_, rows_;
    std::vector<unsigned char> buffer_;
    std::vector<float> horizontal_[2];

#if defined(__SSE2__)
    using Channels = __m128;
    static Channels splat(float value) { return _mm_set1_ps(value); }
    static Channels sum(Channels first, Channels second) { return _mm_add_ps(first, second); }
    static Channels difference(Channels first, Channels second) { return _mm_sub_ps(first, second); }
    static Channels product(Channels first, Channels second) { return _mm_mul_ps(first, second); }
    static Channels load(const float* values) { return _mm_loadu_ps(values); }
    static void store(float* target, Channels values) { _mm_storeu_ps(target, values); }
    static Channels channels(const unsigned char* pixel) {
        std::uint32_t packed = 0;
        for (std::size_t channel = 0; channel < sizeof(packed); ++channel)
            packed |= static_cast<std::uint32_t>(pixel[channel]) << (channel * 8);
        const auto bytes = _mm_cvtsi32_si128(static_cast<int>(packed));
        const auto zero = _mm_setzero_si128();
        return _mm_cvtepi32_ps(_mm_unpacklo_epi16(_mm_unpacklo_epi8(bytes, zero), zero));
    }
    template<typename SamplePixel>
    static void writeRow(unsigned char* destination, int width, SamplePixel&& sample) {
        const auto rounding = _mm_set1_ps(0.5f);
        int x = 0;
        for (; x + 1 < width; x += 2) {
            const auto first = _mm_cvttps_epi32(_mm_add_ps(sample(x), rounding));
            const auto second = _mm_cvttps_epi32(_mm_add_ps(sample(x + 1), rounding));
            const auto words = _mm_packs_epi32(first, second);
            const auto bytes = _mm_packus_epi16(words, words);
            const auto packed = static_cast<std::uint64_t>(_mm_cvtsi128_si64(bytes));
            for (std::size_t channel = 0; channel < sizeof(packed); ++channel)
                destination[x * 4 + channel] = static_cast<unsigned char>(packed >> (channel * 8));
        }
        if (x < width) {
            const auto value = _mm_cvttps_epi32(_mm_add_ps(sample(x), rounding));
            const auto words = _mm_packs_epi32(value, value);
            const auto packed = static_cast<std::uint32_t>(_mm_cvtsi128_si32(_mm_packus_epi16(words, words)));
            for (std::size_t channel = 0; channel < sizeof(packed); ++channel)
                destination[x * 4 + channel] = static_cast<unsigned char>(packed >> (channel * 8));
        }
    }
#else
#if defined(__aarch64__)
    using Channels = float32x4_t;
    /** @brief Broadcast a scalar interpolation weight to all BGRA lanes.
     * @param value Weight to broadcast.
     * @return Four identical lanes. */
    static Channels splat(float value) { return vdupq_n_f32(value); }
    /** @brief Add independent BGRA lanes without mixing channels.
     * @param first Left operand.
     * @param second Right operand.
     * @return Lane-wise sum. */
    static Channels sum(Channels first, Channels second) { return vaddq_f32(first, second); }
    /** @brief Form independent BGRA deltas for interpolation.
     * @param first Reference value in each lane.
     * @param second Value subtracted from each lane.
     * @return Lane-wise difference. */
    static Channels difference(Channels first, Channels second) { return vsubq_f32(first, second); }
    /** @brief Multiply each BGRA lane by its corresponding weight.
     * @param first Channel values or weights.
     * @param second Matching weights or channel values.
     * @return Lane-wise product. */
    static Channels product(Channels first, Channels second) { return vmulq_f32(first, second); }
    /** @brief Load interpolation values in BGRA order.
     * @param values Four readable floats.
     * @return Loaded channels. */
    static Channels load(const float* values) { return vld1q_f32(values); }
    /** @brief Store floating-point channels before narrowing to bytes.
     * @param[out] target Four writable floats.
     * @param values Channel vector in BGRA order. */
    static void store(float* target, Channels values) { vst1q_f32(target, values); }
#else
    using Channels = std::array<float, 4>;
    /** @brief Broadcast a scalar to all four portable BGRA lanes.
     * @param value Weight to broadcast.
     * @return Four identical lanes. */
    static Channels splat(float value) { return {value, value, value, value}; }
    /** @brief Accumulate channel values without mixing color and alpha.
     * @param first Left operand.
     * @param second Right operand.
     * @return Lane-wise sum. */
    static Channels sum(Channels first, Channels second) {
        for (int channel = 0; channel < 4; ++channel) first[channel] += second[channel];
        return first;
    }
    /** @brief Form independent channel deltas for interpolation.
     * @param first Reference values.
     * @param second Values to subtract.
     * @return Lane-wise difference. */
    static Channels difference(Channels first, Channels second) {
        for (int channel = 0; channel < 4; ++channel) first[channel] -= second[channel];
        return first;
    }
    /** @brief Multiply each channel by its corresponding weight.
     * @param first First lane vector.
     * @param second Second lane vector.
     * @return Lane-wise product. */
    static Channels product(Channels first, Channels second) {
        for (int channel = 0; channel < 4; ++channel) first[channel] *= second[channel];
        return first;
    }
    /** @brief Load four floating-point channel values in BGRA order.
     * @param values Four readable floats.
     * @return Loaded channels. */
    static Channels load(const float* values) { return {values[0], values[1], values[2], values[3]}; }
    /** @brief Store channels in the order expected by byte conversion.
     * @param[out] target Four writable floats.
     * @param values Channel values in BGRA order. */
    static void store(float* target, Channels values) {
        for (int channel = 0; channel < 4; ++channel) target[channel] = values[channel];
    }
#endif
    /** @brief Widen a packed BGRA pixel for floating-point interpolation.
     * @param pixel Four readable bytes in BGRA order.
     * @return The same channel values without byte-range arithmetic. */
    static Channels channels(const unsigned char* pixel) {
        const float values[] = {float(pixel[0]), float(pixel[1]), float(pixel[2]), float(pixel[3])};
        return load(values);
    }
    /** @brief Write one sampled row to the byte raster.
     * @tparam SamplePixel Callable yielding four lanes for an output column.
     * @param[out] destination Row storage of at least four bytes per pixel.
     * @param width Output column count.
     * @param sample Interpolating sampler.
     * @details Round and clamp at the byte boundary, not during interpolation. */
    template<typename SamplePixel>
    static void writeRow(unsigned char* destination, int width, SamplePixel&& sample) {
        for (int column = 0; column < width; ++column) {
            float values[4]; store(values, sample(column));
            for (int channel = 0; channel < 4; ++channel)
                destination[column * 4 + channel] = static_cast<unsigned char>(
                    std::clamp(static_cast<int>(values[channel] + .5f), 0, 255));
        }
    }
#endif
    /** @brief Precompute bilinear indices and fractions along one output axis.
     * @param[out] result One entry per output sample.
     * @param count Output samples on the axis.
     * @param sourceSize Positive source extent in pixels.
     * @param offset Crop offset in source pixels.
     * @param scale Source pixels per output sample.
     * @details Map pixel centers and clamp before computing neighbors.
     * Reusing these entries avoids repeated coordinate arithmetic in each row. */
    static void samples(std::vector<Sample>& result, int count, int sourceSize,
                        double offset, double scale) {
        result.resize(count);
        for (int i = 0; i < count; ++i) {
            const double position = std::clamp(offset + (i + 0.5) * scale - 0.5,
                                              0.0, static_cast<double>(sourceSize - 1));
            const int first = static_cast<int>(position);
            result[i] = {first, std::min(first + 1, sourceSize - 1),
                         static_cast<float>(position - first)};
        }
    }
public:
    /** @brief Source-space crop and uniform scale needed for an output frame. */
    struct Crop {
        double x, y, width, height, scale;
    };
    /**
     * @brief Fit an output aspect ratio inside source bounds and rotation margin.
     * @param[in] sourceWidth Source image width in pixels.
     * @param[in] sourceHeight Source image height in pixels.
     * @param[in] width Requested output width in pixels.
     * @param[in] height Requested output height in pixels.
     * @param[in] inset Minimum source-edge margin in pixels.
     * @param[in] rotationBoundDegrees Maximum absolute rotation to accommodate.
     * @return Centered crop rectangle and the common scale for both axes.
     * @throws std::runtime_error If dimensions, margin, or rotation are invalid.
     */
    static Crop crop(int sourceWidth, int sourceHeight, int width, int height, int inset = 0,
                     double rotationBoundDegrees = 0) {
        validateJpegSize(width, height);
        if (sourceWidth < 1 || sourceHeight < 1 || sourceWidth > 16384 || sourceHeight > 16384)
            throw std::runtime_error("Invalid source dimensions for JPEG framing");
        if (inset < 0 || sourceWidth <= 2 * inset || sourceHeight <= 2 * inset)
            throw std::runtime_error("Capture inset leaves no usable image");
        if (!std::isfinite(rotationBoundDegrees) || std::abs(rotationBoundDegrees) > 45)
            throw std::runtime_error("Image framing rotation must be within +/-45 degrees");
        const double angle = std::abs(rotationBoundDegrees) * 0.01745329251994329577;
        const double c = std::cos(angle), s = std::sin(angle);
        const double scale = std::min((sourceWidth - 2.0 * inset) / (width * c + height * s),
                                      (sourceHeight - 2.0 * inset) / (height * c + width * s));
        const double cropWidth = width * scale, cropHeight = height * scale;
        return {(sourceWidth - cropWidth) / 2, (sourceHeight - cropHeight) / 2,
                cropWidth, cropHeight, scale};
    }
    struct View {
        const unsigned char* bgra;
        int width, height, stride;
        double cropX, cropY, cropWidth, cropHeight;
    };

    /**
     * @brief Frame BGRA pixels into a crop, resampled image, or rotated image.
     * @param[in] bgra Borrowed source pixel buffer.
     * @param[in] sourceWidth Source width in pixels.
     * @param[in] sourceHeight Source height in pixels.
     * @param[in] stride Source row stride in bytes.
     * @param[in] width Requested output width in pixels.
     * @param[in] height Requested output height in pixels.
     * @param[in] inset Excluded border in source pixels.
     * @param[in] rotationDegrees Applied rotation in the renderer's image coordinates.
     * @param[in] rotationBoundDegrees Maximum rotation used when selecting a safe crop.
     * @return BGRA view and the source crop represented by its pixels.
     * @throws std::runtime_error If input bounds or the rotated footprint are invalid.
     * @note A returned pointer to internal storage is invalidated by the next call.
     */
    View apply(const unsigned char* bgra, int sourceWidth, int sourceHeight, int stride,
               int width, int height, int inset = 0, double rotationDegrees = 0,
               double rotationBoundDegrees = 0) {
        if (!std::isfinite(rotationDegrees))
            throw std::runtime_error("Nonfinite image framing rotation");
        const auto bounds = crop(sourceWidth, sourceHeight, width, height, inset,
                                 std::max(std::abs(rotationDegrees), std::abs(rotationBoundDegrees)));
        if (!bgra || sourceWidth < 1 || sourceHeight < 1 ||
            sourceWidth > 16384 || sourceHeight > 16384 || stride < sourceWidth * 4)
            throw std::runtime_error("Invalid source BGRA dimensions/stride for JPEG framing");
        // A single sampling scale on both axes prevents distortion, even for fractional crop edges.
        const double scale = bounds.scale, cropWidth = bounds.width, cropHeight = bounds.height;
        const double cropX = bounds.x, cropY = bounds.y;
        if (rotationDegrees != 0) {
            buffer_.resize(static_cast<std::size_t>(width) * height * 4);
            const double angle = rotationDegrees * 0.01745329251994329577;
            const double c = std::cos(angle), s = std::sin(angle);
            for (int y = 0; y < height; ++y) {
                const double dy = (y + 0.5 - height / 2.0) * scale;
                writeRow(buffer_.data() + static_cast<std::size_t>(y) * width * 4, width, [&](int x) {
                    const double dx = (x + 0.5 - width / 2.0) * scale;
                    const double px = sourceWidth / 2.0 - 0.5 + c * dx - s * dy;
                    const double py = sourceHeight / 2.0 - 0.5 + s * dx + c * dy;
                    const int ix = static_cast<int>(std::floor(px)), iy = static_cast<int>(std::floor(py));
                    if (ix < 0 || iy < 0 || ix + 1 >= sourceWidth || iy + 1 >= sourceHeight)
                        throw std::runtime_error("Rotated image footprint lies outside capture pixels");
                    const auto* top = bgra + static_cast<std::size_t>(iy) * stride + ix * 4;
                    const auto* bottom = top + stride;
                    const auto hx = splat(static_cast<float>(px - ix));
                    const auto vy = splat(static_cast<float>(py - iy));
                    const auto ta = channels(top), tb = channels(top + 4);
                    const auto ba = channels(bottom), bb = channels(bottom + 4);
                    const auto upper = sum(ta, product(difference(tb, ta), hx));
                    const auto lower = sum(ba, product(difference(bb, ba), hx));
                    return sum(upper, product(difference(lower, upper), vy));
                });
            }
            return {buffer_.data(), width, height, width * 4, cropX, cropY, cropWidth, cropHeight};
        }
        if (scale == 1 && std::floor(cropX) == cropX && std::floor(cropY) == cropY)
            return {bgra + static_cast<std::size_t>(cropY) * stride + static_cast<std::size_t>(cropX) * 4,
                    width, height, stride, cropX, cropY, cropWidth, cropHeight};
        if (sourceWidth_ != sourceWidth || sourceHeight_ != sourceHeight ||
            width_ != width || height_ != height || inset_ != inset || rotationBound_ != rotationBoundDegrees) {
            samples(columns_, width, sourceWidth, cropX, scale);
            samples(rows_, height, sourceHeight, cropY, scale);
            buffer_.resize(static_cast<std::size_t>(width) * height * 4);
            sourceWidth_ = sourceWidth;
            sourceHeight_ = sourceHeight;
            width_ = width;
            height_ = height;
            inset_ = inset;
            rotationBound_ = rotationBoundDegrees;
        }
        if (scale < 1) {
            horizontal_[0].resize(static_cast<std::size_t>(width) * 4);
            horizontal_[1].resize(static_cast<std::size_t>(width) * 4);
            // Cache only within this image; neighboring output rows reuse source rows when upscaling.
            int cached[2] = {-1, -1};
            const auto horizontalRow = [&](int row, int protectedSlot) {
                for (int slot = 0; slot < 2; ++slot)
                    if (cached[slot] == row) return slot;
                const int slot = protectedSlot == 0 ? 1 : 0;
                const auto* source = bgra + static_cast<std::size_t>(row) * stride;
                auto* destination = horizontal_[slot].data();
                for (int x = 0; x < width; ++x) {
                    const auto& column = columns_[x];
                    const auto a = channels(source + column.first * 4);
                    const auto b = channels(source + column.second * 4);
                    store(destination + x * 4,
                        sum(a, product(difference(b, a), splat(column.weight))));
                }
                cached[slot] = row;
                return slot;
            };
            for (int y = 0; y < height; ++y) {
                const auto& row = rows_[y];
                const int first = horizontalRow(row.first, -1);
                const int second = horizontalRow(row.second, first);
                const auto* top = horizontal_[first].data();
                const auto* bottom = horizontal_[second].data();
                auto* destination = buffer_.data() + static_cast<std::size_t>(y) * width * 4;
                const auto vertical = splat(row.weight);
                writeRow(destination, width, [&](int x) {
                    const auto upper = load(top + x * 4), lower = load(bottom + x * 4);
                    return sum(upper, product(difference(lower, upper), vertical));
                });
            }
            return {buffer_.data(), width, height, width * 4, cropX, cropY, cropWidth, cropHeight};
        }
        for (int y = 0; y < height; ++y) {
            const auto& row = rows_[y];
            const auto* top = bgra + static_cast<std::size_t>(row.first) * stride;
            const auto* bottom = bgra + static_cast<std::size_t>(row.second) * stride;
            auto* destination = buffer_.data() + static_cast<std::size_t>(y) * width * 4;
            const auto vertical = splat(row.weight);
            writeRow(destination, width, [&](int x) {
                const auto& column = columns_[x];
                const auto horizontal = splat(column.weight);
                const auto a = channels(top + column.first * 4);
                const auto b = channels(top + column.second * 4);
                const auto d = channels(bottom + column.first * 4);
                const auto e = channels(bottom + column.second * 4);
                const auto upper = sum(a, product(difference(b, a), horizontal));
                const auto lower = sum(d, product(difference(e, d), horizontal));
                return sum(upper, product(difference(lower, upper), vertical));
            });
        }
        return {buffer_.data(), width, height, width * 4, cropX, cropY, cropWidth, cropHeight};
    }
};
}
#endif
