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
 * @file map_progress.h
 * @brief Report monotonic progress, stage timings, and explicit interruption.
 * @details Progress percentages are estimates while work is active and remain
 * below 100 until the caller confirms completion. This avoids presenting a
 * slow finalization or canceled run as a successful dataset.
 */
#ifndef PANGEA_MAP_PROGRESS_H
#define PANGEA_MAP_PROGRESS_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace pangea_survey {
/**
 * @brief Thread-safe progress reporter for staged map work.
 * @details A reporter thread emits a heartbeat even when a computation has no
 * fine-grained updates. Destruction joins that thread and reports interruption
 * unless @ref finish was called.
 */
class MapProgress {
    using Clock = std::chrono::steady_clock;
    std::string label_, stage_ = "Preparing", detail_;
    Clock::time_point started_ = Clock::now(), stageStarted_ = started_;
    double first_ = 0, last_ = 0, fraction_ = 0, estimateSeconds_ = 0;
    bool complete_ = false, closing_ = false, changed_ = true;
    bool timedStage_ = false;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::thread reporter_;

    /** @brief Emit at most one elapsed-time record for the active timed stage.
     * @param now Current monotonic time.
     * @param status Terminal stage label.
     * @details Disarm reporting after emission so teardown cannot repeat it. */
    void stageTiming(Clock::time_point now, const char* status) {
        if (!timedStage_) return;
        std::ostringstream text;
        text.imbue(std::locale::classic());
        text << "TIMING stage=" << std::quoted(stage_) << " wall_seconds="
             << std::chrono::duration<double>(now - stageStarted_).count()
             << " status=" << status << '\n';
        const auto line = text.str();
        std::cout.write(line.data(), static_cast<std::streamsize>(line.size()));
        timedStage_ = false;
    }

    /** @brief Estimate overall progress from reported work and stage time.
     * @param now Current monotonic time.
     * @return Estimated percentage; 100 is reserved for explicit completion.
     * @details The time estimate approaches the stage endpoint without
     * claiming all its work is done. Prefer measured progress when greater. */
    double percentage(Clock::time_point now) const {
        const double elapsed = std::chrono::duration<double>(now - stageStarted_).count();
        const double guess = estimateSeconds_ > 0 ? .9 * elapsed / (elapsed + estimateSeconds_) : 0;
        return complete_ ? 100 : std::min(99.9, first_ + (last_ - first_) * std::max(fraction_, guess));
    }
    /** @brief Format an elapsed duration at whole-second precision.
     * @param value Nonnegative interval.
     * @return Hours, minutes and seconds without wrapping hours at a day. */
    static std::string duration(Clock::duration value) {
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(value).count();
        std::ostringstream text;
        text << std::setfill('0') << std::setw(2) << seconds / 3600 << ':'
             << std::setw(2) << seconds / 60 % 60 << ':' << std::setw(2) << seconds % 60;
        return text.str();
    }
    /** @brief Format a consistent progress snapshot under the state mutex.
     * @param now Monotonic time shared by all elapsed fields on the line.
     * @return Complete line including its newline.
     * @details Distinguish an interrupted shutdown from completed publication. */
    std::string line(Clock::time_point now) const {
        std::ostringstream text;
        text << label_ << ": " << std::fixed << std::setprecision(1) << percentage(now)
             << "% " << (complete_ ? "| complete" : "(estimated) | " + stage_)
             << " | elapsed " << duration(now - started_);
        if (!complete_) text << " | stage " << duration(now - stageStarted_);
        if (!detail_.empty()) text << " | " << detail_;
        if (closing_ && !complete_) text << " | stopped before completion; check status/errors";
        return text.str() + '\n';
    }
    /** @brief Run the progress reporter until teardown.
     * @details Wake on changes or every five seconds so a long stage remains
     * observable. Emit a last snapshot even when stopped early. */
    void report() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!closing_) {
            changed_ = false;
            const auto text = line(Clock::now());
            std::cout.write(text.data(), static_cast<std::streamsize>(text.size()));
            std::cout.flush();
            wake_.wait_for(lock, std::chrono::seconds(5), [&] { return closing_ || changed_; });
        }
        const auto text = line(Clock::now());
        std::cout.write(text.data(), static_cast<std::streamsize>(text.size()));
        std::cout.flush();
    }
public:
    /**
     * @brief Start reporting progress under a human-readable label.
     * @param[in] label Prefix shown on each progress line.
     */
    explicit MapProgress(std::string label = "Map progress") : label_(std::move(label)),
        reporter_([this] { report(); }) {}
    /** @brief Signal the reporter to emit its final status, then join it.
     * @details Drop the state mutex before joining so the reporter can exit. */
    ~MapProgress() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stageTiming(Clock::now(), "interrupted");
            closing_ = true;
        }
        wake_.notify_one();
        reporter_.join();
    }
    /**
     * @brief Begin a monotonic percentage range for one named stage.
     * @param[in] name Stage label.
     * @param[in] first First percentage assigned to the stage.
     * @param[in] last Upper percentage bound, at most 99.9 until finish.
     * @param[in] estimateSeconds Optional duration used only for a bounded estimate.
     * @throws std::runtime_error If bounds are invalid or progress would move backward.
     */
    void stage(const std::string& name, double first, double last, double estimateSeconds = 0) {
        if (!std::isfinite(first) || !std::isfinite(last) || !std::isfinite(estimateSeconds) ||
            first < 0 || last < first || last > 99.9 || estimateSeconds < 0)
            throw std::runtime_error("Invalid map progress stage");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (complete_ || first < percentage(Clock::now()))
                throw std::runtime_error("Map progress stage would go backwards");
            stageTiming(Clock::now(), "finished");
            first_ = first; last_ = last; fraction_ = 0; estimateSeconds_ = estimateSeconds;
            stage_ = name; detail_.clear(); stageStarted_ = Clock::now(); changed_ = true;
            timedStage_ = true;
        }
        wake_.notify_one();
    }
    /**
     * @brief Advance the current stage without allowing its fraction to regress.
     * @param[in] fraction Stage completion estimate in the closed interval [0, 1].
     * @param[in] detail Optional current item or count shown to the operator.
     * @throws std::runtime_error If @p fraction is nonfinite or outside [0, 1].
     */
    void update(double fraction, const std::string& detail = {}) {
        if (!std::isfinite(fraction) || fraction < 0 || fraction > 1)
            throw std::runtime_error("Invalid map progress fraction");
        std::lock_guard<std::mutex> lock(mutex_);
        fraction_ = std::max(fraction_, fraction);
        if (!detail.empty()) detail_ = detail;
    }
    /** @brief Replace the operator-facing detail without changing progress. */
    void detail(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex_);
        detail_ = text;
    }
    /** @brief Mark the entire operation complete and permit 100 percent output. */
    void finish() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stageTiming(Clock::now(), "finished");
            complete_ = true; stage_ = "Complete"; detail_.clear(); changed_ = true;
        }
        wake_.notify_one();
    }
    /** @brief Exercise stage progress, monotonic updates, and completion behavior.
     * @throws std::runtime_error If a progress invariant fails.
     * @details Sample estimated progress at fixed synthetic offsets instead
     * of waiting for a long stage in real time. */
    static void selfTest() {
        const auto require = [](bool value) {
            if (!value) throw std::runtime_error("Map progress regression failed");
        };
        {
            MapProgress progress("Progress self-test");
            progress.stage("Capture", 0, 35);
            progress.update(.5);
            require(progress.percentage(progress.stageStarted_) == 17.5);
            progress.update(.25);
            require(progress.percentage(progress.stageStarted_) == 17.5);
            progress.update(1);
            require(progress.percentage(progress.stageStarted_) == 35);
            progress.stage("Dense reconstruction", 35, 80, 900);
            const auto later = progress.stageStarted_ + std::chrono::seconds(900);
            require(std::abs(progress.percentage(later) - 55.25) < 1e-9);
            require(progress.percentage(later + std::chrono::hours(10000)) < 80);
            progress.stage("Publication", 80, 99);
            progress.update(1);
            require(progress.percentage(progress.stageStarted_) == 99);
            require(progress.line(progress.stageStarted_).find("(estimated)") != std::string::npos);
            bool rejected = false;
            try { progress.update(std::numeric_limits<double>::quiet_NaN()); }
            catch (const std::runtime_error&) { rejected = true; }
            require(rejected);
            progress.finish();
            require(progress.percentage(progress.stageStarted_) == 100);
            require(progress.line(progress.stageStarted_).find("| complete") != std::string::npos);
        }
        struct CaptureOutput {
            std::ostringstream text;
            std::streambuf* previous = std::cout.rdbuf(text.rdbuf());
            ~CaptureOutput() { std::cout.rdbuf(previous); }
        };
        std::string heartbeatOutput, interruptedOutput;
        {
            CaptureOutput capture;
            {
                MapProgress progress;
                progress.stage("Long calculation", 0, 99, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(5500));
                progress.finish();
            }
            heartbeatOutput = capture.text.str();
        }
        require(heartbeatOutput.find("(estimated) | Long calculation") != std::string::npos &&
                heartbeatOutput.find("| stage 00:00:05") != std::string::npos &&
            heartbeatOutput.find("TIMING stage=\"Long calculation\" wall_seconds=") != std::string::npos &&
                heartbeatOutput.find("100.0% | complete") != std::string::npos);
        {
            CaptureOutput capture;
            {
                MapProgress progress;
                progress.stage("Rejected output", 0, 99);
                progress.update(1);
            }
            interruptedOutput = capture.text.str();
        }
        require(interruptedOutput.find("stopped before completion") != std::string::npos &&
            interruptedOutput.find("status=interrupted") != std::string::npos &&
                interruptedOutput.find("100.0%") == std::string::npos);
        std::cout << "PASS: monotonic estimated progress, bounded long-stage guesses, five-second heartbeat, "
                     "explicit interruption and completion-only 100%.\n";
    }
};
}
#endif
