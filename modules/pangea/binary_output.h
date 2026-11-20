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
 * @file binary_output.h
 * @brief Publish immutable result files without exposing partial contents.
 * @details A sibling temporary file allows readers to see either no result or
 * the complete renamed result. Existing targets are rejected because survey
 * products are evidence; silently replacing one could conceal a prior run.
 * This helper provides atomic visibility on the same filesystem, not an fsync
 * durability guarantee.
 */
#ifndef PANGEA_BINARY_OUTPUT_H
#define PANGEA_BINARY_OUTPUT_H

#include "application_runtime.h"

/**
 * @brief Write bytes to a new path and publish them by same-directory rename.
 * @param[in] path Final output path; it and its `.partial` sibling must not exist.
 * @param[in] data Address of the bytes to write; may be null only when @p size is zero.
 * @param[in] size Number of bytes to publish.
 * @throws std::runtime_error If output exists, writing fails, or rename fails.
 * @note Rename prevents readers from observing a partially written target;
 * use the durable checkpoint helpers when crash/power-loss durability matters.
 */
inline void saveBinary(const fs::path& path, const void* data, std::size_t size) {
    const fs::path temporary = path.string() + ".partial";
    if (fs::exists(path) || fs::exists(temporary))
        throw std::runtime_error("Refusing to overwrite output: " + path.u8string());
    try {
        // Only the completed sibling is renamed into the public name.
        std::ofstream output(temporary, std::ios::binary);
        output.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
        output.close();
        if (!output) throw std::runtime_error("Cannot write output: " + path.u8string());
        std::error_code ec;
        fs::rename(temporary, path, ec);
        if (ec) throw std::runtime_error("Publish output failed: " + ec.message());
    } catch (...) {
        std::error_code error;
        fs::remove(temporary, error);
        if (error) std::cerr << "WARNING: Cannot clean partial output: " << error.message() << '\n';
        throw;
    }
}

#endif
