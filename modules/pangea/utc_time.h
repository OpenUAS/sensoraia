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
 * @file utc_time.h
 * @brief Validate Gregorian UTC targets and qualify simulator time samples.
 * @details The UTC lock is based on repeated simulator clock writes and
 * readback; it is deliberately not treated as the camera's shutter clock.
 * Frame-age and monotonic timestamp checks prevent stale readbacks from being
 * promoted into capture metadata.
 */
#ifndef PANGEA_UTC_TIME_H
#define PANGEA_UTC_TIME_H

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pangea_survey {
/** @brief One simulator event/value pair used to set the Zulu clock. */
struct UtcEvent {
    const char* name;
    std::uint32_t value;
};

/**
 * @brief A Gregorian UTC minute and its simulator clock-matching rules.
 * @details `text` is canonical `YYYY-MM-DDTHH:MMZ`; absolute seconds are a
 * calendar coordinate, not a monotonic or exposure timestamp.
 */
struct UtcDateTime {
    int year = 1, month = 1, day = 1, hour = 0, minute = 0;
    std::string text;

    /** @brief Return whether a Gregorian calendar year has 366 days. */
    static bool leapYear(int value) {
        return value % 4 == 0 && (value % 100 != 0 || value % 400 == 0);
    }
    /** @brief Get the length of a Gregorian calendar month.
     * @param year Calendar year.
     * @param month Validated one-based month (1 through 12).
     * @return Days in the requested month.
     * @details Only February varies under the Gregorian leap-year rule. */
    static int daysInMonth(int year, int month) {
        constexpr int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        return days[month - 1] + (month == 2 && leapYear(year) ? 1 : 0);
    }
    /** @brief Return the one-based ordinal day used by simulator clock events. */
    int dayOfYear() const {
        int result = day;
        for (int m = 1; m < month; ++m) result += daysInMonth(year, m);
        return result;
    }
    /** @brief Build the ordered simulator event sequence for this UTC minute. */
    std::array<UtcEvent, 5> events() const {
        // ZULU_DAY_SET uses 1-based ordinal days, unlike the ZULU DAY OF YEAR SimVar.
        return {{{"ZULU_YEAR_SET", static_cast<std::uint32_t>(year)},
                 {"ZULU_DAY_SET", static_cast<std::uint32_t>(dayOfYear())},
                 {"ZULU_HOURS_SET", static_cast<std::uint32_t>(hour)},
                 {"ZULU_MINUTES_SET", static_cast<std::uint32_t>(minute)},
                 {"CLOCK_SECONDS_ZERO", 0}}};
    }
    /** @brief Convert the calendar minute to a monotonically increasing calendar coordinate. */
    double absoluteSeconds() const {
        const long long previousYears = year - 1;
        const long long days = previousYears * 365 + previousYears / 4 -
                               previousYears / 100 + previousYears / 400 + dayOfYear() - 1;
        return static_cast<double>(days) * 86400 + hour * 3600 + minute * 60;
    }
    /** @brief Check whether the simulator clock is in the requested UTC minute.
     * @param actual Absolute calendar seconds, in the same epoch as absoluteSeconds().
     * @return True for a finite time in the half-open minute.
     * @details A reading exactly at the next minute belongs to that minute. */
    bool matchesAbsoluteTime(double actual) const {
        const double expected = absoluteSeconds();
        return std::isfinite(actual) && actual >= expected && actual < expected + 60;
    }
    /** @brief Check the stricter time window used for capture.
     * @param actual Absolute calendar seconds.
     * @return True only during the first second of the requested minute.
     * @details Minute-level matching is sufficient for date validation but
     * not for confirming a capture immediately after zeroing the clock. */
    bool matchesCaptureTime(double actual) const {
        return std::isfinite(actual) && actual >= absoluteSeconds() &&
               actual < absoluteSeconds() + 1;
    }
};

/**
 * @brief Parse a strict Gregorian UTC minute in `YYYY-MM-DDTHH:MMZ` form.
 * @param[in] text Candidate ASCII date/time.
 * @return Validated date/time preserving the canonical input string.
 * @throws std::invalid_argument If syntax or any calendar field is invalid.
 */
inline UtcDateTime parseUtcDateTime(std::string_view text) {
    const auto invalid = [] {
        return std::invalid_argument("UTC must be a valid Gregorian date/time YYYY-MM-DDTHH:MMZ "
                                     "(years 0001-9999, hours 00-23, minutes 00-59)");
    };
    if (text.size() != 17 || text[4] != '-' || text[7] != '-' ||
        text[10] != 'T' || text[13] != ':' || text[16] != 'Z')
        throw invalid();
    const auto digits = [&](std::size_t start, std::size_t count) {
        const auto part = text.substr(start, count);
        int value = 0;
        const auto parsed = std::from_chars(part.data(), part.data() + part.size(), value);
        if (part.find_first_not_of("0123456789") != std::string_view::npos ||
            parsed.ec != std::errc{} || parsed.ptr != part.data() + part.size())
            throw invalid();
        return value;
    };
    UtcDateTime result{digits(0, 4), digits(5, 2), digits(8, 2),
                       digits(11, 2), digits(14, 2), std::string(text)};
    if (result.year < 1 || result.month < 1 || result.month > 12 || result.day < 1 ||
        result.day > UtcDateTime::daysInMonth(result.year, result.month) ||
        result.hour > 23 || result.minute > 59)
        throw invalid();
    return result;
}

/**
 * @brief Convert simulator calendar seconds to their containing UTC minute.
 * @param[in] actual Finite, nonnegative simulator calendar seconds.
 * @return Canonical UTC minute containing @p actual.
 * @throws std::runtime_error If the value lies outside years 0001 through 9999.
 */
inline UtcDateTime utcMinuteFromAbsoluteTime(double actual) {
    if (!std::isfinite(actual) || actual < 0 ||
        actual >= UtcDateTime{9999, 12, 31, 23, 59, {}}.absoluteSeconds() + 60)
        throw std::runtime_error("Simulator UTC is outside the supported Gregorian calendar");
    int low = 1, high = 9999;
    while (low < high) {
        const int middle = (low + high + 1) / 2;
        if (UtcDateTime{middle, 1, 1, 0, 0, {}}.absoluteSeconds() <= actual) low = middle;
        else high = middle - 1;
    }
    UtcDateTime result{low, 1, 1, 0, 0, {}};
    const auto wholeSeconds = static_cast<long long>(std::floor(actual - result.absoluteSeconds()));
    int days = static_cast<int>(wholeSeconds / 86400);
    while (days >= UtcDateTime::daysInMonth(result.year, result.month)) {
        days -= UtcDateTime::daysInMonth(result.year, result.month);
        ++result.month;
    }
    result.day = days + 1;
    result.hour = static_cast<int>(wholeSeconds % 86400 / 3600);
    result.minute = static_cast<int>(wholeSeconds % 3600 / 60);
    std::ostringstream text;
    text << std::setfill('0') << std::setw(4) << result.year << '-'
         << std::setw(2) << result.month << '-' << std::setw(2) << result.day << 'T'
         << std::setw(2) << result.hour << ':' << std::setw(2) << result.minute << 'Z';
    result.text = text.str();
    return result;
}

/**
 * @brief Decide whether simulator UTC is stable enough to admit a capture.
 * @param[in] target Requested UTC minute.
 * @param[in] actual Latest simulator clock readback in absolute seconds.
 * @param[in] readAgeSeconds Age of that readback in seconds.
 * @param[in] verifiedFrame Frame number at which the target was last verified.
 * @param[in] frame Current frame number.
 * @param[in] pending Whether another time-setting event remains pending.
 * @return True only for a fresh matching readback after at least three frames.
 */
inline bool captureUtcReady(const UtcDateTime& target, double actual, double readAgeSeconds,
                            unsigned long long verifiedFrame, unsigned long long frame,
                            bool pending) {
    return !pending && target.matchesCaptureTime(actual) &&
        std::isfinite(readAgeSeconds) && readAgeSeconds >= 0 && readAgeSeconds < 0.5 &&
        frame >= verifiedFrame && frame - verifiedFrame >= 3;
}

/**
 * @brief Validate that a frame's UTC readback is recent and in the locked second.
 * @param[in] target Requested UTC minute.
 * @param[in] actual Simulator calendar time sampled for the frame.
 * @param[in] read100ns Monotonic clock count when UTC was read.
 * @param[in] frame100ns Monotonic clock count associated with the rendered frame.
 * @return True when the sample is ordered, less than one second old, and in target.
 * @note Valid simulator-clock provenance does not imply exposure synchronization.
 */
inline bool captureUtcSampleValid(const UtcDateTime& target, double actual,
                                  long long read100ns, long long frame100ns) {
    return target.matchesCaptureTime(actual) && read100ns > 0 &&
        frame100ns >= read100ns && frame100ns - read100ns < 10000000;
}
}
#endif
