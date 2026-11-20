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
 * @file map_storage.h
 * @brief Integrity checks and bounded disk-backed intermediates for surveys.
 * @details Checkpoints are identified by content hashes and published through
 * fsync plus rename so resume can distinguish complete work from interrupted
 * writes. SHA-256 here detects accidental mismatch; it is not an authenticity
 * or security boundary.
 */
#ifndef PANGEA_MAP_STORAGE_H
#define PANGEA_MAP_STORAGE_H

#include "sha256fst.h"
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <array>
#include <algorithm>
#include <functional>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/statvfs.h>

namespace pangea_survey {
/** @brief Own a working-file descriptor through success and exception paths. */
class StorageDescriptor {
    int descriptor_;
public:
    /** @brief Adopt a descriptor without duplicating it.
     * @param descriptor Owned descriptor, or -1 for no open file. */
    explicit StorageDescriptor(int descriptor = -1) : descriptor_(descriptor) {}
    /** @brief Noncopyable: ownership cannot be shared by copying the FD number. */
    StorageDescriptor(const StorageDescriptor&) = delete;
    /** @brief Nonassignable: use reset() to replace the owned descriptor. */
    StorageDescriptor& operator=(const StorageDescriptor&) = delete;
    /** @brief Close an owned descriptor; use close() when the status matters. */
    ~StorageDescriptor() { if (descriptor_ >= 0) ::close(descriptor_); }
    /** @brief Borrow the descriptor without transferring ownership.
     * @return Open descriptor number, or -1. */
    int get() const { return descriptor_; }
    /** @brief Relinquish the old descriptor and adopt a distinct replacement.
     * @param descriptor New owned descriptor, or -1 to leave the slot empty.
     * @details The old close status is not reported; do not pass get() back in. */
    void reset(int descriptor) {
        if (descriptor_ >= 0) ::close(descriptor_);
        descriptor_ = descriptor;
    }
    /** @brief Close explicitly and report the system call result.
     * @return Close status; the owner is empty even when close fails.
     * @details Clear ownership first so destruction cannot close a reused number. */
    int close() {
        const int descriptor = descriptor_;
        descriptor_ = -1;
        return ::close(descriptor);
    }
};
/** @brief Compute a lowercase SHA-256 digest for an in-memory byte range. */
inline std::string dataDigest(const void* data, std::size_t size) {
    std::array<char, SHA256_HEX_SIZE> digest{};
    sha256_hex(data, size, digest.data());
    return digest.data();
}
/**
 * @brief Hash a file in bounded chunks, optionally honoring cancellation.
 * @param[in] path Input file.
 * @param[in] stopped Optional cooperative cancellation predicate.
 * @return Lowercase SHA-256 digest.
 * @throws std::runtime_error If the file cannot be read or cancellation is requested.
 */
inline std::string fileDigest(const std::filesystem::path& path, const std::function<bool()>& stopped = {}) {
    if (stopped && stopped()) throw std::runtime_error("Checksum verification canceled");
    StorageDescriptor input(open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW));
    struct stat before{};
    if (input.get() < 0 || fstat(input.get(), &before) != 0 || !S_ISREG(before.st_mode) || before.st_size < 0)
        throw std::runtime_error("Cannot hash regular working input: " + path.string());
    sha256 context; sha256_init(&context);
    std::array<char, 65536> buffer{};
    auto remaining = static_cast<std::uint64_t>(before.st_size);
    while (remaining) {
        if (stopped && stopped()) throw std::runtime_error("Checksum verification canceled");
        const auto count = read(input.get(), buffer.data(), std::min<std::uint64_t>(remaining, buffer.size()));
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) throw std::runtime_error("Cannot finish input checksum");
        sha256_append(&context, buffer.data(), static_cast<std::size_t>(count));
        remaining -= static_cast<std::uint64_t>(count);
    }
    struct stat after{};
    if (fstat(input.get(), &after) != 0 || before.st_size != after.st_size ||
        before.st_mtim.tv_sec != after.st_mtim.tv_sec || before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
        before.st_ctim.tv_sec != after.st_ctim.tv_sec || before.st_ctim.tv_nsec != after.st_ctim.tv_nsec)
        throw std::runtime_error("Working input changed during checksum");
    std::array<char, SHA256_HEX_SIZE> digest{};
    sha256_finalize_hex(&context, digest.data());
    return digest.data();
}
/**
 * @brief Require requested free space plus a fixed safety reserve.
 * @param[in] directory Filesystem on which work will be created.
 * @param[in] bytes Estimated bytes needed by the next operation.
 * @throws std::runtime_error If free space cannot be queried or is insufficient.
 */
inline void storageHeadroom(const std::filesystem::path& directory, std::uint64_t bytes) {
    struct statvfs status{};
    constexpr std::uint64_t reserve = 512ull * 1024 * 1024;
    if (bytes > UINT64_MAX - reserve || statvfs(directory.c_str(), &status) != 0 || !status.f_frsize ||
        status.f_bavail < (bytes + reserve) / status.f_frsize + ((bytes + reserve) % status.f_frsize != 0))
        throw std::runtime_error("Insufficient working disk space; checkpoint retained");
}
/**
 * @brief Fsync a completed working file, rename it, then sync its directory.
 * @param[in] temporary Completed sibling temporary file.
 * @param[in] target Final checkpoint path on the same filesystem.
 * @param replace True to rename over the target; false to publish without replacement.
 * @throws std::runtime_error If synchronization, rename, or directory access fails.
 */
inline void publishWorkingFile(const std::filesystem::path& temporary, const std::filesystem::path& target,
                               bool replace = true) {
    StorageDescriptor descriptor(open(temporary.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW));
    struct stat completed{};
    if (descriptor.get() < 0 || fstat(descriptor.get(), &completed) != 0 || !S_ISREG(completed.st_mode))
        throw std::runtime_error("Cannot open completed regular working file");
    const int status = fsync(descriptor.get());
    const int closed = descriptor.close();
    if (status != 0 || closed != 0) throw std::runtime_error("Cannot synchronize working file");
    if (replace) std::filesystem::rename(temporary, target);
    else {
        if (link(temporary.c_str(), target.c_str()) != 0)
            throw std::runtime_error("Cannot exclusively publish working file");
        if (unlink(temporary.c_str()) != 0)
            throw std::runtime_error("Cannot remove published working-file temporary");
    }
    const auto parent = target.has_parent_path() ? target.parent_path() : std::filesystem::path(".");
    StorageDescriptor directory(open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
    if (directory.get() < 0) throw std::runtime_error("Cannot open checkpoint directory");
    const int synced = fsync(directory.get());
    if (synced != 0) throw std::runtime_error("Cannot synchronize checkpoint directory");
}
/** @brief Save checkpoint text through the durable working-file publisher. */
inline void saveWorkingText(const std::filesystem::path& target, const std::string& text) {
    auto temporary = target.string() + ".partial.XXXXXX";
    StorageDescriptor output(mkstemp(temporary.data()));
    if (output.get() < 0) throw std::runtime_error("Cannot create working metadata");
    try {
        if (fcntl(output.get(), F_SETFD, FD_CLOEXEC) != 0)
            throw std::runtime_error("Cannot protect working metadata descriptor");
        std::size_t written = 0;
        while (written < text.size()) {
            const auto count = write(output.get(), text.data() + written,
                                     std::min<std::size_t>(text.size() - written, 65536));
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) throw std::runtime_error("Cannot save working metadata");
            written += static_cast<std::size_t>(count);
        }
        if (output.close() != 0) throw std::runtime_error("Cannot close working metadata");
        publishWorkingFile(temporary, target);
    } catch (...) {
        unlink(temporary.c_str());
        throw;
    }
}
/**
 * @brief Read bounded rectangular regions from a lossless BGRA disk raster.
 * @details Disk-backed source pixels trade I/O for a small resident working
 * set; each read is capped so callers cannot accidentally materialize an
 * unbounded source image through this interface.
 */
class DiskRaster {
    std::filesystem::path path_;
    int width_, height_;
public:
    /**
     * @brief Open a raster whose byte length must match its declared dimensions.
     * @param[in] path Raster file containing row-major BGRA bytes.
     * @param[in] width Raster width in pixels.
     * @param[in] height Raster height in pixels.
     * @throws std::runtime_error If dimensions or file length are invalid.
     */
    DiskRaster(std::filesystem::path path, int width, int height)
        : path_(std::move(path)), width_(width), height_(height) {
        if (width < 1 || height < 1 || width > 8192 || height > 8192 ||
            std::filesystem::file_size(path_) != static_cast<std::uint64_t>(width) * height * 4)
            throw std::runtime_error("Invalid lossless working raster");
    }
    /**
     * @brief Read a rectangle into a new BGRA matrix.
     * @param[in] rectangle Nonempty in-bounds raster region no larger than 16 MiB.
     * @return Owned BGRA pixels for the requested rectangle.
     * @throws std::runtime_error If the rectangle exceeds bounds or file reading fails.
     */
    cv::Mat read(const cv::Rect& rectangle) const {
        if (rectangle.empty() || (rectangle & cv::Rect(0, 0, width_, height_)) != rectangle ||
            static_cast<std::uint64_t>(rectangle.area()) * 4 > 16ull * 1024 * 1024)
            throw std::runtime_error("Working raster read exceeds bounded region");
        std::ifstream input(path_, std::ios::binary);
        cv::Mat result(rectangle.size(), CV_8UC4);
        for (int row = 0; row < rectangle.height; ++row) {
            input.seekg((static_cast<std::uint64_t>(rectangle.y + row) * width_ + rectangle.x) * 4);
            input.read(reinterpret_cast<char*>(result.ptr(row)), rectangle.width * 4);
            if (!input) throw std::runtime_error("Cannot read lossless working raster");
        }
        return result;
    }
};
}
#endif
