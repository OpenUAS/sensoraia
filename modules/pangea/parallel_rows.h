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
 * @file parallel_rows.h
 * @brief Run independent row work with a bounded native thread team.
 * @details Nested callers fall back to serial execution so inner helpers do not
 * multiply worker counts and oversubscribe a small processor. Exceptions are
 * captured on workers and rethrown only after every thread has joined.
 */
#ifndef PANGEA_PARALLEL_ROWS_H
#define PANGEA_PARALLEL_ROWS_H

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace pangea_survey {
inline thread_local unsigned parallelRowsDepth = 0;

/**
 * @brief Apply a row operation in parallel without creating nested teams.
 * @tparam Function Callable accepting one zero-based row index.
 * @param[in] rows Number of rows to process; nonpositive values are a no-op.
 * @param[in] function Independent work for each row.
 * @param[in] requested Upper bound on workers; hardware availability and row count
 * further reduce the team.
 * @throws Any exception thrown by @p function, rethrown after all workers stop.
 */
template<class Function>
void parallelRows(int rows, Function function, unsigned requested = 4) {
    if (rows <= 0) return;
    struct DepthScope {
        DepthScope() { ++parallelRowsDepth; }
        ~DepthScope() { --parallelRowsDepth; }
    };
    const unsigned count = (std::max)(1u, (std::min)(requested, (std::min)(std::thread::hardware_concurrency(),
                                                                        static_cast<unsigned>(rows))));
    if (count == 1 || parallelRowsDepth != 0) {
        const DepthScope depth;
        for (int row = 0; row < rows; ++row) function(row);
        return;
    }
    std::atomic<bool> stop{false};
    std::atomic<int> next{0};
    std::mutex mutex;
    std::exception_ptr error;
    const auto run = [&] {
        const DepthScope depth;
        while (!stop.load(std::memory_order_relaxed)) {
            const int row = next.fetch_add(1, std::memory_order_relaxed);
            if (row >= rows) break;
            try {
                function(row);
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex);
                if (!error) error = std::current_exception();
                stop.store(true, std::memory_order_relaxed);
            }
        }
    };
    std::vector<std::thread> workers;
    bool launch_failed = false;
    try {
        workers.reserve(count - 1);
        for (unsigned worker = 1; worker < count; ++worker) workers.emplace_back(run);
    } catch (...) {
        launch_failed = true;
        stop.store(true, std::memory_order_relaxed);
    }
    if (!launch_failed) run();
    for (auto& worker : workers) if (worker.joinable()) worker.join();
    if (launch_failed && !error) {
        stop.store(false, std::memory_order_relaxed);
        run();
    }
    if (error) std::rethrow_exception(error);
}
}
#endif
