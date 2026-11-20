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
 * @file application_runtime.h
 * @brief Shared logging and operating-system checks for the survey executable.
 * @details Session output is mirrored rather than redirected exclusively so
 * interactive progress remains visible while a diagnostic trail is collected.
 * The stream-buffer lock keeps concurrent worker messages from interleaving
 * between the console and session log.
 */
#ifndef PANGEA_APPLICATION_RUNTIME_H
#define PANGEA_APPLICATION_RUNTIME_H

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <cstring>
#include <cerrno>
#include <algorithm>

namespace fs = std::filesystem;
/** @brief Stream one message to both the original console and the session log. */
class TeeBuffer : public std::streambuf {
    std::streambuf* console_;
    std::streambuf* file_;
    std::mutex& mutex_;
protected:
    /** @brief Write a block to both outputs under the shared lock.
     * @param data Borrowed character block.
     * @param size Character count.
     * @return The shorter of the two writes, exposing failure of either output.
     * @details Keep the lock across both writes so competing diagnostics
     * appear in the same order in the console and session log. */
    std::streamsize xsputn(const char* data, std::streamsize size) override {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto consoleSize = console_->sputn(data, size);
        const auto fileSize = file_->sputn(data, size);
        return std::min(consoleSize, fileSize);
    }
    /** @brief Mirror a single character not handled by a put area.
     * @param character Streambuf integer character, or the EOF sentinel.
     * @return The written character, or EOF on either write failure.
     * @details Treat an EOF sentinel as a no-op, not as a byte to write. */
    int overflow(int character) override {
        if (traits_type::eq_int_type(character, traits_type::eof()))
            return traits_type::not_eof(character);
        std::lock_guard<std::mutex> lock(mutex_);
        const auto value = traits_type::to_char_type(character);
        if (traits_type::eq_int_type(console_->sputc(value), traits_type::eof()) ||
            traits_type::eq_int_type(file_->sputc(value), traits_type::eof()))
            return traits_type::eof();
        return character;
    }
    /** @brief Synchronize the two outputs without interleaving other writes.
     * @return Zero if both sync calls succeed, otherwise -1.
     * @details A console sync failure short-circuits the file sync. */
    int sync() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return console_->pubsync() == 0 && file_->pubsync() == 0 ? 0 : -1;
    }
public:
    /**
     * @brief Bind two destinations to a caller-owned synchronization lock.
     * @param[in] console Original console stream buffer.
     * @param[in] file Session-log stream buffer.
     * @param[in,out] mutex Lock shared by the stdout and stderr tee buffers.
     */
    TeeBuffer(std::streambuf* console, std::streambuf* file, std::mutex& mutex)
        : console_(console), file_(file), mutex_(mutex) {}
};

/**
 * @brief Mirror stdout and stderr into one session file for a scoped run.
 * @details Construction installs the tee buffers and destruction restores the
 * streams that were active before the session. Keep this object alive for the
 * whole operation so worker diagnostics cannot outlive the file stream.
 */
class SessionLog {
    std::ofstream file_;
    std::mutex mutex_;
    TeeBuffer output_;
    TeeBuffer errors_;
    std::streambuf* oldOutput_;
    std::streambuf* oldErrors_;
public:
    /**
     * @brief Open an append-only session log and install console mirroring.
     * @param[in] directory Existing working directory for the session.
     * @param[in] filename Log name relative to @p directory.
     * @throws std::runtime_error If the log cannot be opened.
     */
    explicit SessionLog(const fs::path& directory, const fs::path& filename = "pangea.log")
        : file_(directory / filename, std::ios::app),
          output_(std::cout.rdbuf(), file_.rdbuf(), mutex_), errors_(std::cerr.rdbuf(), file_.rdbuf(), mutex_),
          oldOutput_(std::cout.rdbuf()), oldErrors_(std::cerr.rdbuf()) {
        if (!file_) throw std::runtime_error("Cannot create application log in output directory");
        std::cout.rdbuf(&output_);
        std::cerr.rdbuf(&errors_);
        std::cout << std::unitbuf << "\n--- Pangea survey session ---\n";
    }
    /** @brief Detach the tee buffers before their file and lock are destroyed.
     * @details Workers must already be stopped before restoring global streams. */
    ~SessionLog() {
        std::cout.rdbuf(oldOutput_);
        std::cerr.rdbuf(oldErrors_);
    }
};

/** @brief Convert a failed POSIX result into an operation-specific exception. */
inline void posixCheck(bool success, const char* operation) {
    if (!success) {
        throw std::runtime_error(std::string(operation) + " failed (" +
                                 std::strerror(errno) + ")");
    }
}

/** @brief Compatibility name for the shared POSIX failure check. */
inline void winCheck(bool success, const char* operation) {
    posixCheck(success, operation);
}

#endif
