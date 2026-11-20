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
 * @file ini_file.h
 * @brief Parse strict section-qualified INI settings for the survey mapper.
 * @details Parsing and allow-list validation are separate so each owning
 * configuration schema can reject unknown keys. This catches misspellings
 * instead of quietly running with a default that looks like an accepted value.
 */
#ifndef PANGEA_INI_FILE_H
#define PANGEA_INI_FILE_H

#include "route.h"
#include <initializer_list>

namespace pangea_survey {
/**
 * @brief Read scalar values from a bounded, section-based INI file.
 * @details Keys are stored as `section.name`; duplicate and empty settings are
 * rejected during construction. Call @ref validate with the owning schema's
 * allow-list before consuming values.
 */
class IniFile {
    std::map<std::string, std::string> values_;
public:
    /**
     * @brief Parse an INI file into unique section-qualified keys.
     * @param[in] path Input file path.
     * @throws std::runtime_error If the file is unreadable or malformed.
     */
    explicit IniFile(const std::filesystem::path& path) {
        std::ifstream input(path);
        if (!input) throw std::runtime_error("Cannot open INI: " + path.u8string());
        std::string line, section;
        bool firstLine = true;
        while (std::getline(input, line)) {
            if (firstLine && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
            firstLine = false;
            line = trim(line.substr(0, line.find_first_of(";#")));
            if (line.empty()) continue;
            if (line.front() == '[' && line.back() == ']') {
                section = trim(line.substr(1, line.size() - 2));
                continue;
            }
            const auto equal = line.find('=');
            if (section.empty() || equal == std::string::npos)
                throw std::runtime_error("Expected section/key=value in " + path.u8string());
            const auto key = section + "." + trim(line.substr(0, equal));
            const auto value = trim(line.substr(equal + 1));
            if (value.empty() || !values_.emplace(key, value).second)
                throw std::runtime_error("Empty or duplicate INI key: " + key);
        }
        if (input.bad()) throw std::runtime_error("Cannot read INI: " + path.u8string());
    }
    /**
     * @brief Reject keys not owned by the caller's configuration schema.
     * @param[in] allowed Exact `section.name` keys accepted by the schema.
     * @throws std::runtime_error On the first unknown key.
     */
    void validate(std::initializer_list<const char*> allowed) const {
        for (const auto& entry : values_)
            if (std::none_of(allowed.begin(), allowed.end(),
                [&](const char* key) { return entry.first == key; }))
                throw std::runtime_error("Unknown INI key: " + entry.first);
    }
    /** @brief Test whether a section-qualified key was present in the file. */
    bool has(const std::string& key) const { return values_.count(key) != 0; }
    /** @brief Read text or return a fallback when the key is absent. */
    std::string text(const std::string& key, const std::string& fallback = {}) const {
        const auto found = values_.find(key);
        return found == values_.end() ? fallback : found->second;
    }
    /** @brief Read a finite numeric value, using @p fallback only when absent. */
    double numeric(const std::string& key, double fallback) const {
        return has(key) ? number(text(key), key) : fallback;
    }
    /** @brief Read a required finite numeric value. */
    double required(const std::string& key) const {
        if (!has(key)) throw std::runtime_error("Missing INI key: " + key);
        return number(text(key), key);
    }
    /** @brief Read a strict numeric boolean whose only accepted values are 0 and 1. */
    bool boolean(const std::string& key, bool fallback = false) const {
        const auto value = numeric(key, fallback ? 1 : 0);
        if (value != 0 && value != 1) throw std::runtime_error(key + " must be 0 or 1");
        return value == 1;
    }
};
}

#endif
