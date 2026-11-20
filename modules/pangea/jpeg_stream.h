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
 * @file jpeg_stream.h
 * @brief Decode JPEGs with IJG scanlines into BGRA previews or disk-backed rasters.
 * @details Streaming one scanline at a time avoids retaining both the
 * compressed source and a full decoded frame in memory. The working raster is
 * an internal lossless intermediate. Its bounded raster path rejects
 * progressive JPEGs; preview decoding retains the codec's progressive support.
 */
#ifndef PANGEA_JPEG_STREAM_H
#define PANGEA_JPEG_STREAM_H

#include "map_storage.h"
#include "jpeg_marker_guard.h"
#include <algorithm>
#include <cstdio>
#include <csetjmp>
#include <jpeglib.h>
#include <jerror.h>
#include <memory>
#include <functional>
#include <vector>
#include <utility>

namespace pangea_survey {
/** @brief Apply the shared JPEG marker bounds to the current decoder state.
 * @param state Decoder reporting the marker. */
inline void validateJpegScanMessage(j_common_ptr state) {
    usher_jpeg_validate_marker(state);
}
/**
 * @brief Convert one baseline JPEG into a row-major BGRA working raster.
 * @details Instances own decoder state and a reusable scanline buffer and must
 * not be shared across concurrent decodes.
 */
class JpegStream {
    struct Error {
        jpeg_error_mgr manager{};
        std::jmp_buf recovery;
        char message[JMSG_LENGTH_MAX]{};
        void (*emit)(j_common_ptr, int) = nullptr;
    } error_;
    jpeg_decompress_struct decoder_{};
    FILE* input_ = nullptr;
    bool created_ = false;
    std::vector<unsigned char> row_;
    std::unique_ptr<std::ofstream> output_;
    /** @brief Preserve the codec diagnostic and jump to the decode recovery point.
     * @param state Decoder carrying this instance's error manager.
     * @details The outer operation owns cleanup and C++ exception translation. */
    static void failure(j_common_ptr state) {
        auto* error = reinterpret_cast<Error*>(state->err);
        state->err->format_message(state, error->message);
        std::longjmp(error->recovery, 1);
    }
    /** @brief Check markers before forwarding the codec diagnostic.
     * @param state Decoder carrying the saved diagnostic handler.
     * @param level libjpeg diagnostic level. */
    static void message(j_common_ptr state, int level) {
        validateJpegScanMessage(state);
        reinterpret_cast<Error*>(state->err)->emit(state, level);
    }
    /** @brief Release per-decode state while retaining reusable pixel storage.
     * @details Clear the creation flag and stream pointers so repeated cleanup
     * after an error is safe. */
    void reset() noexcept {
        if (created_) jpeg_destroy_decompress(&decoder_);
        created_ = false;
        decoder_ = {};
        if (input_) fclose(input_);
        input_ = nullptr;
        output_.reset();
    }
    struct Operation {
        JpegStream& owner;
        /** @brief Release the streams and codec on every operation exit. */
        ~Operation() { owner.reset(); }
    };
public:
    /** @brief Decoded preview and its scale relative to the source camera image. */
    struct Image {
        int width = 0;
        int height = 0;
        double scale = 1;
        std::vector<unsigned char> pixels;
    };
private:
    Image image_;
public:
    /** @brief Construct an idle decoder; files are opened per operation. */
    JpegStream() = default;
    /** @brief Release any remaining codec and stream resources. */
    ~JpegStream() { reset(); }
    /** @brief Noncopyable: the decoder and streams have a single owner. */
    JpegStream(const JpegStream&) = delete;
    /** @brief Nonassignable: active streams and recovery state cannot be copied. */
    JpegStream& operator=(const JpegStream&) = delete;
    /** @brief Read a JPEG preview without retaining the compressed file in memory.
     * @param source JPEG path with dimensions matching its capture metadata.
     * @param width Expected full-resolution width.
     * @param height Expected full-resolution height.
     * @param maxSide Preferred longest decoded side, or zero for full resolution.
     * @return Owned BGRA pixels and the decoder's intrinsic scaling ratio.
     * @throws std::runtime_error For damaged JPEGs, invalid dimensions or I/O errors. */
    Image read(const std::filesystem::path& source, int width, int height, int maxSide) {
        reset();
        const Operation operation{*this};
        const auto bytes = std::filesystem::file_size(source);
        if (!bytes || bytes > 128 * 1024 * 1024 || width <= 0 || height <= 0 ||
            width > 16384 || height > 16384 || maxSide < 0)
            throw std::runtime_error("Invalid source JPEG size or dimensions");
        input_ = fopen(source.c_str(), "rb");
        if (!input_) throw std::runtime_error("Cannot open source JPEG");
        decoder_.err = jpeg_std_error(&error_.manager);
        error_.manager.error_exit = failure;
        error_.emit = error_.manager.emit_message;
        error_.manager.emit_message = message;
        if (setjmp(error_.recovery)) throw std::runtime_error(std::string("JPEG decode: ") + error_.message);
        created_ = true; jpeg_create_decompress(&decoder_);
        jpeg_stdio_src(&decoder_, input_);
        jpeg_read_header(&decoder_, TRUE);
        if (decoder_.image_width != static_cast<unsigned>(width) ||
            decoder_.image_height != static_cast<unsigned>(height))
            throw std::runtime_error("Source JPEG dimensions disagree with capture CSV");
        decoder_.scale_denom = 8;
        for (unsigned numerator = 8; numerator > 0; --numerator) {
            decoder_.scale_num = numerator;
            jpeg_calc_output_dimensions(&decoder_);
            if (!maxSide || std::max(decoder_.output_width, decoder_.output_height) <= static_cast<unsigned>(maxSide) || numerator == 1)
                break;
        }
        decoder_.out_color_space = JCS_RGB;
        jpeg_start_decompress(&decoder_);
        image_.width = static_cast<int>(decoder_.output_width);
        image_.height = static_cast<int>(decoder_.output_height);
        image_.scale = double(decoder_.scale_num) / decoder_.scale_denom;
        image_.pixels.resize(static_cast<std::size_t>(image_.width) * image_.height * 4);
        row_.resize(static_cast<std::size_t>(image_.width) * 3);
        while (decoder_.output_scanline < decoder_.output_height) {
            const auto row = decoder_.output_scanline;
            JSAMPROW pixels = row_.data();
            if (jpeg_read_scanlines(&decoder_, &pixels, 1) != 1)
                throw std::runtime_error("Incomplete JPEG scanline");
            auto* destination = image_.pixels.data() + static_cast<std::size_t>(row) * image_.width * 4;
            for (int column = 0; column < image_.width; ++column) {
                destination[column * 4] = row_[column * 3 + 2];
                destination[column * 4 + 1] = row_[column * 3 + 1];
                destination[column * 4 + 2] = row_[column * 3];
                destination[column * 4 + 3] = 255;
            }
        }
        jpeg_finish_decompress(&decoder_);
        if (decoder_.err->num_warnings) throw std::runtime_error("Damaged source JPEG");
        return std::move(image_);
    }
    /**
     * @brief Stream a source JPEG into a new, atomically published raster file.
     * @param[in] source Input baseline JPEG path.
     * @param[in] target Destination raster path; existing files are not replaced.
     * @param[in] width Expected source width in pixels.
     * @param[in] height Expected source height in pixels.
     * @param[in] stopped Cooperative cancellation predicate.
     * @throws std::runtime_error For invalid dimensions, decode/write errors,
     * damaged JPEG data, or cancellation.
     */
    void decode(const std::filesystem::path& source, const std::filesystem::path& target,
                int width, int height, const std::function<bool()>& stopped) {
        reset();
        const Operation operation{*this};
        if (width <= 0 || height <= 0 || width > 8192 || height > 8192)
            throw std::runtime_error("Invalid streamed JPEG dimensions");
        if (std::filesystem::exists(target)) throw std::runtime_error("Working raster already exists");
        input_ = fopen(source.c_str(), "rb");
        if (!input_) throw std::runtime_error("Cannot open JPEG for streamed decoding");
        const auto temporary = target.string() + ".partial";
        output_ = std::make_unique<std::ofstream>(temporary, std::ios::binary | std::ios::trunc);
        if (!*output_) throw std::runtime_error("Cannot create working raster");
        row_.resize(static_cast<std::size_t>(width) * 4);
        decoder_.err = jpeg_std_error(&error_.manager);
        error_.manager.error_exit = failure;
        error_.emit = error_.manager.emit_message;
        error_.manager.emit_message = message;
        if (setjmp(error_.recovery)) throw std::runtime_error(std::string("JPEG decode: ") + error_.message);
        created_ = true; jpeg_create_decompress(&decoder_);
        jpeg_stdio_src(&decoder_, input_);
        jpeg_read_header(&decoder_, TRUE);
        if (decoder_.image_width != static_cast<unsigned>(width) ||
            decoder_.image_height != static_cast<unsigned>(height) || decoder_.progressive_mode)
            throw std::runtime_error("Streamed JPEG requires matching dimensions and baseline encoding");
        decoder_.out_color_space = JCS_RGB;
        jpeg_start_decompress(&decoder_);
        while (decoder_.output_scanline < decoder_.output_height) {
            if (stopped && stopped()) throw std::runtime_error("Streamed source preparation canceled");
            JSAMPROW pixels = row_.data();
            if (jpeg_read_scanlines(&decoder_, &pixels, 1) != 1)
                throw std::runtime_error("Incomplete JPEG scanline");
            for (std::size_t pixel = static_cast<std::size_t>(width); pixel > 0; --pixel) {
                const std::size_t source = (pixel - 1) * 3;
                const std::size_t destination = (pixel - 1) * 4;
                const unsigned char red = row_[source];
                const unsigned char green = row_[source + 1];
                const unsigned char blue = row_[source + 2];
                row_[destination] = blue;
                row_[destination + 1] = green;
                row_[destination + 2] = red;
                row_[destination + 3] = 255;
            }
            output_->write(reinterpret_cast<const char*>(row_.data()), static_cast<std::streamsize>(row_.size()));
            if (!*output_) throw std::runtime_error("Cannot write working raster");
        }
        jpeg_finish_decompress(&decoder_); output_->close();
        if (!*output_ || decoder_.err->num_warnings) throw std::runtime_error("Damaged JPEG or working raster write");
        publishWorkingFile(temporary, target, false);
    }
};
}
#endif
