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
 * @file map_resources.h
 * @brief Apply and periodically check the survey process's low-memory policy.
 * @details The address-space ceiling, current RSS, and system-available reserve
 * are different measurements. Checking all three helps a job stop with a
 * resumable checkpoint before it crowds the camera board, but does not prove
 * whole-board safety or replace physical RSS/PSS/CMA/concurrency measurements.
 */
#ifndef PANGEA_MAP_RESOURCES_H
#define PANGEA_MAP_RESOURCES_H

#include "map_config.h"
#include "opencv_parallel.hpp"
#include <opencv2/core.hpp>
#include <sys/resource.h>
#include <sched.h>
#include <linux/ioprio.h>
#include <sys/syscall.h>
#include <chrono>
#include <fstream>

namespace pangea_survey {
class MapResourcePressure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
/**
 * @brief Read a named Linux memory counter and convert its KiB value to bytes.
 * @param[in] path procfs file containing the counter.
 * @param[in] key Exact counter name, including its trailing colon.
 * @return Counter value in bytes.
 * @throws std::runtime_error If the counter is unavailable.
 */
inline std::uint64_t procMemory(const char* path, const char* key) {
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream fields(line);
        std::string name; std::uint64_t value = 0;
        if ((fields >> name >> value) && name == key) return value * 1024;
    }
    throw std::runtime_error("Required memory telemetry is unavailable");
}
/**
 * @brief Enforce the configured bounded-work policy for a survey process.
 * @details Low-memory mode limits address space, worker count, open files and
 * scheduler priority. Periodic checks also preserve system-available memory;
 * they are admission/overload guards, not a hard RSS cap or a whole-board proof.
 */
class MapResources {
    bool bounded_;
    std::uint64_t limit_, reserve_;
    std::chrono::steady_clock::time_point checked_{};
public:
    /**
     * @brief Establish process limits for the selected runtime profile.
     * @param[in] config Validated map configuration.
     * @throws std::runtime_error If the profile is unsupported or any required
     * Linux resource policy cannot be established.
     */
    explicit MapResources(const MapConfig& config) : bounded_(config.lowMemory),
        limit_(static_cast<std::uint64_t>(config.memoryMiB) * 1024 * 1024),
        reserve_(static_cast<std::uint64_t>(config.reserveMiB) * 1024 * 1024) {
        usher_cv_parallel::ensure_installed();
        if (!bounded_) {
            if (procMemory("/proc/meminfo", "MemTotal:") <= 1024ull * 1024 * 1024)
                throw std::runtime_error("This board requires runtime.profile=low_memory and Great XYZ");
            return;
        }
        cv::setNumThreads(1);
        struct rlimit inherited{};
        if (getrlimit(RLIMIT_AS, &inherited) != 0) throw std::runtime_error("Cannot inspect address limit");
        const rlim_t cap = std::min<rlim_t>(limit_, inherited.rlim_cur);
        struct rlimit address{cap, std::min<rlim_t>(inherited.rlim_max, cap)};
        struct rlimit files{64, 64}, core{0, 0};
        struct sched_param priority{};
        if (setrlimit(RLIMIT_AS, &address) != 0 || setrlimit(RLIMIT_NOFILE, &files) != 0 ||
            setrlimit(RLIMIT_CORE, &core) != 0 || setpriority(PRIO_PROCESS, 0, 19) != 0 ||
            sched_setscheduler(0, SCHED_IDLE, &priority) != 0 ||
            syscall(SYS_ioprio_set, IOPRIO_WHO_PROCESS, 0, IOPRIO_PRIO_VALUE(IOPRIO_CLASS_IDLE, 0)) != 0)
            throw std::runtime_error("Cannot establish low-memory process policy");
        checkpoint();
    }
    /**
     * @brief Check current process RSS and system reserve at a bounded cadence.
     * @throws std::runtime_error If memory headroom closes; existing checkpoints
     * are intentionally left in place for a later resume.
     */
    void checkpoint() {
        if (!bounded_) return;
        const auto now = std::chrono::steady_clock::now();
        if (now - checked_ < std::chrono::milliseconds(250)) return;
        checked_ = now;
        const auto resident = procMemory("/proc/self/status", "VmRSS:");
        const auto available = procMemory("/proc/meminfo", "MemAvailable:");
        if (resident > limit_ || available < reserve_ + limit_ - std::min(resident, limit_))
            throw MapResourcePressure("RAM headroom closed; working checkpoint retained for resume");
    }
};
}
#endif
