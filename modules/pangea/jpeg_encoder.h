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
 * @file jpeg_encoder.h
 * @brief Encode BGRA map pixels and publish JPEGs with optional EXIF.
 * @details The encoder owns and reuses its IJG workspace to avoid a new
 * large output allocation for every tile. EXIF is inserted into the compressed
 * stream, preserving encoded image bytes rather than decoding and encoding the
 * pixels a second time.
 */
#ifndef PANGEA_JPEG_ENCODER_H
#define PANGEA_JPEG_ENCODER_H

#include "exifite.h"
#include "map_storage.h"
#include <algorithm>
#include <csetjmp>
#include <cstdio>
#include <jpeglib.h>
#include <jerror.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @brief Reusable IJG compressor and JPEG publisher.
 * @details Instances are stateful and are not safe for concurrent calls; create
 * one per worker when tiles are encoded in parallel.
 */
class JpegEncoder {
    struct Error {
        jpeg_error_mgr manager{};
        std::jmp_buf recovery;
        char message[JMSG_LENGTH_MAX]{};
    } error_;
    jpeg_compress_struct compressor_{};
    jpeg_destination_mgr destination_{};
    std::vector<unsigned char> buffer_;
    std::vector<unsigned char> row_;
    std::size_t size_ = 0;
    std::size_t limit_ = 0;

    /** @brief Save the codec diagnostic and transfer control to recovery.
     * @param codec Compressor carrying the installed error manager.
     * @details Convert to a C++ exception at the recovery site, not in this callback. */
    static void failure(j_common_ptr codec) {
        auto& error = *reinterpret_cast<Error*>(codec->err);
        codec->err->format_message(codec, error.message);
        std::longjmp(error.recovery, 1);
    }
    /** @brief Point the codec at reusable compressed-output storage.
     * @param codec Compressor whose client data identifies the encoder.
     * @details Cap the writable span at the current operation limit even
     * when a previous call allocated a larger buffer. */
    static void startOutput(j_compress_ptr codec) {
        auto& encoder = *static_cast<JpegEncoder*>(codec->client_data);
        codec->dest->next_output_byte = encoder.buffer_.data();
        codec->dest->free_in_buffer = std::min(encoder.buffer_.size(), encoder.limit_);
    }
    /** @brief Grow compressed storage without exceeding the operation limit.
     * @param codec Compressor that filled its destination span.
     * @return TRUE once more room is available; failures take codec recovery.
     * @details Resume writing at the consumed offset after resize, because
     * vector growth can invalidate the codec's previous output pointer. */
    static boolean growOutput(j_compress_ptr codec) {
        auto& encoder = *static_cast<JpegEncoder*>(codec->client_data);
        const std::size_t used = std::min(encoder.buffer_.size(), encoder.limit_);
        if (used >= encoder.limit_) ERREXIT(codec, JERR_BUFFER_SIZE);
        bool resized = true;
        try {
            encoder.buffer_.resize(std::min(encoder.limit_, encoder.buffer_.size() * 2));
        } catch (const std::exception&) {
            resized = false;
        }
        if (!resized) ERREXIT1(codec, JERR_OUT_OF_MEMORY, 0);
        codec->dest->next_output_byte = encoder.buffer_.data() + used;
        codec->dest->free_in_buffer = encoder.buffer_.size() - used;
        return TRUE;
    }
    /** @brief Record the compressed byte count, excluding unused buffer space.
     * @param codec Compressor supplying the remaining destination bytes. */
    static void finishOutput(j_compress_ptr codec) {
        auto& encoder = *static_cast<JpegEncoder*>(codec->client_data);
        encoder.size_ = std::min(encoder.buffer_.size(), encoder.limit_) - codec->dest->free_in_buffer;
    }
public:
    /** @brief Borrowed view of a JPEG held in this encoder's reusable buffer. */
    struct Image {
        const unsigned char* data;
        std::size_t size;
    };

    /** @brief Create a compressor; throws if the codec cannot be initialized. */
    JpegEncoder() {
        compressor_.err = jpeg_std_error(&error_.manager);
        error_.manager.error_exit = failure;
        if (setjmp(error_.recovery)) {
            jpeg_destroy_compress(&compressor_);
            throw std::runtime_error(std::string("Create IJG encoder: ") + error_.message);
        }
        jpeg_create_compress(&compressor_);
        compressor_.client_data = this;
        destination_.init_destination = startOutput;
        destination_.empty_output_buffer = growOutput;
        destination_.term_destination = finishOutput;
        compressor_.dest = &destination_;
    }
    /** @brief Release the compressor and invalidate any borrowed JPEG view. */
    ~JpegEncoder() { jpeg_destroy_compress(&compressor_); }
    /** @brief Noncopyable: the compressor stores a pointer to its encoder owner. */
    JpegEncoder(const JpegEncoder&) = delete;
    /** @brief Nonassignable: codec and destination state must remain paired. */
    JpegEncoder& operator=(const JpegEncoder&) = delete;

    /**
     * @brief Encode one BGRA image using the configured JPEG quality.
     * @param[in] bgra Top-left pixel of the source image.
     * @param[in] width Image width in pixels.
     * @param[in] height Image height in pixels.
     * @param[in] stride Source row stride in bytes.
     * @param[in] quality JPEG quality from 1 through 100.
     * @return JPEG bytes borrowed from this encoder; the next encode may replace them.
     * @throws std::runtime_error If dimensions, quality, or codec processing is invalid.
     */
    Image encode(const unsigned char* bgra, int width, int height, int stride, int quality) {
        if (!bgra || width <= 0 || height <= 0 || width > 16384 || height > 16384 ||
            stride < width * 4 || quality < 1 || quality > 100)
            throw std::runtime_error("Invalid BGRA dimensions, stride or JPEG quality");
        row_.resize(static_cast<std::size_t>(width) * 3);
        if (buffer_.empty()) buffer_.resize(4096);
        limit_ = static_cast<std::size_t>(width) * height * 6 + 65536;
        size_ = 0;
        if (setjmp(error_.recovery)) {
            jpeg_abort_compress(&compressor_);
            throw std::runtime_error(std::string("Compress BGRA image to JPEG: ") + error_.message);
        }
        compressor_.image_width = static_cast<JDIMENSION>(width);
        compressor_.image_height = static_cast<JDIMENSION>(height);
        compressor_.input_components = 3;
        compressor_.in_color_space = JCS_RGB;
        jpeg_set_defaults(&compressor_);
        if (quality > 90) {
            for (int component = 0; component < compressor_.num_components; ++component) {
                compressor_.comp_info[component].h_samp_factor = 1;
                compressor_.comp_info[component].v_samp_factor = 1;
            }
        }
        jpeg_set_quality(&compressor_, quality, TRUE);
        jpeg_start_compress(&compressor_, TRUE);
        while (compressor_.next_scanline < compressor_.image_height) {
            const auto* source = bgra + static_cast<std::size_t>(compressor_.next_scanline) * stride;
            for (int column = 0; column < width; ++column) {
                row_[column * 3] = source[column * 4 + 2];
                row_[column * 3 + 1] = source[column * 4 + 1];
                row_[column * 3 + 2] = source[column * 4];
            }
            JSAMPROW pixels = row_.data();
            jpeg_write_scanlines(&compressor_, &pixels, 1);
        }
        jpeg_finish_compress(&compressor_);
        return {buffer_.data(), size_};
    }

    /**
     * @brief Publish encoded JPEG bytes, optionally adding one EXIF APP1 segment.
     * @param[in] image Encoded JPEG view returned by @ref encode.
     * @param[in] path New destination path; the caller controls overwrite policy.
     * @param[in] exif Serialized EXIF payload, or null when no metadata is added.
     * @param[in] exifSize EXIF payload size in bytes.
     * @param[in] durable Whether to use the fsync-backed working-file publisher.
     * @throws std::runtime_error If JPEG framing, EXIF, or file publication fails.
     */
    static void save(const Image& image, const std::filesystem::path& path,
                     const unsigned char* exif = nullptr, std::size_t exifSize = 0, bool durable = false) {
        if (!image.data || image.size < 4 || image.size > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()) ||
            image.data[0] != 0xff || image.data[1] != 0xd8 ||
            image.data[image.size - 2] != 0xff || image.data[image.size - 1] != 0xd9)
            throw std::runtime_error("Invalid encoded JPEG view");
        std::size_t headerSize = 0;
        if (exif || exifSize) {
            exif_view view{};
            if (exif_parse(&view, exif, exifSize) != 0)
                throw std::runtime_error("Invalid EXIF payload for JPEG");
            headerSize = 2;
            // Preserve leading JFIF/APP0 markers before the new EXIF APP1.
            while (image.size - headerSize >= 2 && image.data[headerSize] == 0xff &&
                   image.data[headerSize + 1] == 0xe0) {
                if (image.size - headerSize < 4)
                    throw std::runtime_error("Truncated JPEG APP0 header");
                const std::size_t length = image.data[headerSize + 2] * 256 + image.data[headerSize + 3];
                if (length < 2 || length > image.size - headerSize - 2)
                    throw std::runtime_error("Invalid JPEG APP0 length");
                headerSize += length + 2;
            }
        }
        const std::filesystem::path temporary = path.string() + ".partial";
        try {
            std::ofstream output(temporary, std::ios::binary);
            if (!output) throw std::runtime_error("Cannot create JPEG: " + temporary.u8string());
            if (exif) {
                output.write(reinterpret_cast<const char*>(image.data), static_cast<std::streamsize>(headerSize));
                const unsigned char marker[] = {0xff, 0xe1, static_cast<unsigned char>((exifSize + 2) >> 8),
                                               static_cast<unsigned char>(exifSize + 2)};
                output.write(reinterpret_cast<const char*>(marker), sizeof(marker));
                output.write(reinterpret_cast<const char*>(exif), static_cast<std::streamsize>(exifSize));
            }
            output.write(reinterpret_cast<const char*>(image.data + headerSize),
                         static_cast<std::streamsize>(image.size - headerSize));
            output.close();
            if (!output) throw std::runtime_error("Failed writing JPEG: " + temporary.u8string());
            std::error_code ec;
            if (durable) pangea_survey::publishWorkingFile(temporary, path);
            else std::filesystem::rename(temporary, path, ec);
            if (ec)
                throw std::runtime_error("Finalize JPEG failed: " + ec.message());
        } catch (...) {
            std::error_code error;
            std::filesystem::remove(temporary, error);
            if (error) std::cerr << "WARNING: Cannot remove partial JPEG: " << error.message() << "\n";
            throw;
        }
    }
};
#endif
