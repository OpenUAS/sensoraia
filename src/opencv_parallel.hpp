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
 * @file opencv_parallel.hpp
 * @brief Header-only C++17 `cv::parallel::ParallelForAPI` backend for Sensoraia.
 * @details Replaces OpenCV's internal `pthreads` thread pool (`parallel_impl.cpp`,
 * disabled at build time with `-DWITH_PTHREADS_PF=OFF`) so `ext/opencv` remains
 * an unmodified upstream release tag while all multi-threaded `cv::parallel_for_`
 * regions execute without lock-free data races under both GCC and Clang.
 *
 * Worker threads are created lazily on the first parallel region that needs more
 * than one thread, parked on a condition variable between regions to avoid
 * per-call thread-creation overhead, and joined deterministically when
 * `cv::setNumThreads(1)` is requested or when the outermost `ScopedBackend`
 * guard leaves `main()`.
 */
#ifndef USHER_OPENCV_PARALLEL_HPP
#define USHER_OPENCV_PARALLEL_HPP

#include <opencv2/core.hpp>
#include <opencv2/core/parallel/parallel_backend.hpp>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <new>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

/** @brief Deterministic, race-free `std::thread` backend for OpenCV parallel regions. */
namespace usher_cv_parallel {

/**
 * @brief Persistent `std::thread` pool implementing `cv::parallel::ParallelForAPI`.
 * @details All batch state transitions (`busy_`, `generation_`, `pending_workers_`,
 * worker resize/shutdown) occur under `mutex_`, while stripe indices within an
 * active batch are claimed via `next_task_` using guided chunk sizes matching
 * OpenCV's stripe granularity. Nested or concurrent callers execute inline on
 * the calling thread without blocking or oversubscribing workers.
 */
class ThreadPoolBackend final : public cv::parallel::ParallelForAPI {
 public:
  /** @brief Upper bound on total OpenCV team size (calling thread plus workers). */
  static constexpr int kMaxThreads = 256;

  /** @brief Construct an idle backend sized to the host's default CPU budget. */
  ThreadPoolBackend()
      : configured_threads_(default_threads()),
        atomic_threads_(configured_threads_) {}

  /** @brief Wait for any active region to finish, then stop and join all workers. */
  ~ThreadPoolBackend() override {
    std::vector<std::unique_ptr<Worker>> excess;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      done_cv_.wait(lock, [this] { return !busy_; });
      shutting_down_ = true;
      excess = extract_excess_workers_locked(0);
    }
    join_extracted_workers(excess);
  }

  /** @brief Noncopyable: workers and synchronization state belong to one backend. */
  ThreadPoolBackend(const ThreadPoolBackend&) = delete;
  /** @brief Nonassignable: replacing worker state would break active region ownership. */
  ThreadPoolBackend& operator=(const ThreadPoolBackend&) = delete;

  /**
   * @brief Execute `[0, tasks)` across the configured thread team.
   * @param[in] tasks Number of stripes prepared by OpenCV's `ProxyLoopBody`.
   * @param[in] body_callback OpenCV callback accepting a `[start, end)` subrange.
   * @param[in] callback_data Opaque pointer forwarded to @p body_callback.
   */
  void parallel_for(int tasks,
                    FN_parallel_for_body_cb_t body_callback,
                    void* callback_data) override {
    if (tasks <= 0 || body_callback == nullptr) {
      return;
    }
    if (tasks == 1) {
      body_callback(0, 1, callback_data);
      return;
    }

    std::vector<std::unique_ptr<Worker>> pre_excess;
    int divisor = 1;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (shutting_down_ || busy_ || configured_threads_ <= 1) {
        lock.unlock();
        body_callback(0, tasks, callback_data);
        return;
      }

      busy_ = true;
      const size_t max_workers = static_cast<size_t>(configured_threads_ - 1);
      pre_excess = extract_excess_workers_locked(max_workers);
      const size_t needed_workers =
          std::min(max_workers, static_cast<size_t>(tasks - 1));
      while (workers_.size() < needed_workers) {
        auto worker = std::make_unique<Worker>();
        worker->slot_index = workers_.size();
        worker->slot_id = static_cast<unsigned>(workers_.size() + 1);
        worker->seen_generation = generation_;
        try {
          worker->thread =
              std::thread(&ThreadPoolBackend::worker_loop, this, worker.get());
        } catch (const std::system_error&) {
          break;
        } catch (const std::bad_alloc&) {
          break;
        }
        workers_.push_back(std::move(worker));
      }

      if (workers_.empty()) {
        busy_ = false;
        done_cv_.notify_all();
        lock.unlock();
        join_extracted_workers(pre_excess);
        body_callback(0, tasks, callback_data);
        return;
      }

      const size_t active_workers = std::min(needed_workers, workers_.size());
      const int team = static_cast<int>(active_workers + 1);
      divisor = std::min(tasks, std::max(std::min(100, team * 4), team * 2));
      current_body_ = body_callback;
      current_data_ = callback_data;
      current_tasks_ = tasks;
      current_divisor_ = divisor;
      next_task_.store(0, std::memory_order_relaxed);
      cancel_batch_.store(false, std::memory_order_relaxed);
      batch_exception_ = nullptr;
      active_limit_ = active_workers;
      pending_workers_ = active_workers;
      ++generation_;
    }

    if (!pre_excess.empty()) {
      join_extracted_workers(pre_excess);
    } else {
      wake_cv_.notify_all();
    }

    run_chunks(body_callback, callback_data, tasks, divisor);

    std::vector<std::unique_ptr<Worker>> post_excess;
    std::exception_ptr error;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      done_cv_.wait(lock, [this] { return pending_workers_ == 0; });
      error = std::move(batch_exception_);
      current_body_ = nullptr;
      current_data_ = nullptr;
      current_tasks_ = 0;
      current_divisor_ = 1;
      active_limit_ = 0;
      post_excess = extract_excess_workers_locked(
          static_cast<size_t>(configured_threads_ - 1));
      busy_ = false;
      done_cv_.notify_all();
    }

    join_extracted_workers(post_excess);
    if (error) {
      std::rethrow_exception(error);
    }
  }

  /**
   * @brief Return the zero-based index of the calling thread within the pool.
   * @return `0` on the calling/external thread, or `1..N` on pool workers.
   */
  int getThreadNum() const override { return tls_thread_num(); }

  /**
   * @brief Return the configured thread count for OpenCV parallel regions.
   * @return Configured thread team size (`>= 1`).
   */
  int getNumThreads() const override {
    return atomic_threads_.load(std::memory_order_relaxed);
  }

  /**
   * @brief Set the thread team size and immediately reclaim excess idle workers.
   * @param[in] nThreads Requested thread count (`< 0` resets to default, `0`/`1` disables worker threads).
   * @return Previous configured thread count.
   */
  int setNumThreads(int nThreads) override {
    std::vector<std::unique_ptr<Worker>> excess;
    int previous = 1;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      previous = configured_threads_;
      configured_threads_ = normalize_threads(nThreads);
      atomic_threads_.store(configured_threads_, std::memory_order_relaxed);
      if (!busy_) {
        excess = extract_excess_workers_locked(
            static_cast<size_t>(configured_threads_ - 1));
      }
    }
    join_extracted_workers(excess);
    return previous;
  }

  /**
   * @brief Return the backend identifier reported by `cv::currentParallelFramework()`.
   * @return Static backend name string.
   */
  const char* getName() const override { return "sensoraia-std-thread"; }

  /** @brief Record entry into a `ScopedBackend` lifetime scope. */
  void enter_scope() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    ++scope_depth_;
  }

  /** @brief Leave a `ScopedBackend` scope and join idle workers when the outermost scope exits. */
  void leave_scope() noexcept {
    std::vector<std::unique_ptr<Worker>> excess;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (scope_depth_ > 0 && --scope_depth_ == 0) {
        done_cv_.wait(lock, [this] { return !busy_; });
        excess = extract_excess_workers_locked(0);
      }
    }
    join_extracted_workers(excess);
  }

 private:
  /** @brief Per-worker state owned until after its `std::thread` has joined. */
  struct Worker {
    std::thread thread;             /**< OS thread running `worker_loop`. */
    size_t slot_index = 0;          /**< Zero-based index within `workers_`. */
    unsigned slot_id = 1;           /**< One-based thread index reported by `getThreadNum()`. */
    uint64_t seen_generation = 0;   /**< Last batch generation executed by this worker. */
    bool stop = false;              /**< Set under `mutex_` when retiring this worker. */
  };

  /**
   * @brief Access the thread-local OpenCV worker index for the current thread.
   * @return Reference to the calling thread's zero-based index (`0` unless set by a pool worker).
   */
  static int& tls_thread_num() noexcept {
    thread_local int index = 0;
    return index;
  }

  /**
   * @brief Determine the default thread count from OpenCV's CPU discovery.
   * @return Clamped positive CPU count (`1..kMaxThreads`).
   */
  static int default_threads() noexcept {
    const int cpus = cv::getNumberOfCPUs();
    return std::min(cpus > 0 ? cpus : 1, kMaxThreads);
  }

  /**
   * @brief Normalize an OpenCV thread count argument into `[1, kMaxThreads]`.
   * @param[in] requested Thread count passed to `setNumThreads`.
   * @return Normalized positive team size.
   */
  static int normalize_threads(int requested) noexcept {
    if (requested < 0) {
      return default_threads();
    }
    if (requested <= 1) {
      return 1;
    }
    return std::min(requested, kMaxThreads);
  }

  /**
   * @brief Mark workers at indices `[keep, workers_.size())` for termination and detach them.
   * @param[in] keep Number of leading workers to retain in `workers_`.
   * @return Detached worker records whose threads must be joined after unlocking `mutex_`.
   */
  std::vector<std::unique_ptr<Worker>> extract_excess_workers_locked(size_t keep) {
    std::vector<std::unique_ptr<Worker>> excess;
    if (workers_.size() <= keep) {
      return excess;
    }
    excess.reserve(workers_.size() - keep);
    for (size_t index = keep; index < workers_.size(); ++index) {
      workers_[index]->stop = true;
      excess.push_back(std::move(workers_[index]));
    }
    workers_.resize(keep);
    return excess;
  }

  /**
   * @brief Wake and join workers detached by `extract_excess_workers_locked`.
   * @param[in,out] excess Detached worker list; cleared once all threads join.
   */
  void join_extracted_workers(std::vector<std::unique_ptr<Worker>>& excess) noexcept {
    if (excess.empty()) {
      return;
    }
    wake_cv_.notify_all();
    for (auto& worker : excess) {
      if (worker && worker->thread.joinable()) {
        worker->thread.join();
      }
    }
    excess.clear();
  }

  /**
   * @brief Claim and execute guided `[start, end)` stripe chunks until exhausted.
   * @param[in] body OpenCV stripe callback.
   * @param[in] data Opaque callback context.
   * @param[in] task_count Total stripes in the current batch.
   * @param[in] divisor Guided-scheduling divisor for the current team size.
   */
  void run_chunks(FN_parallel_for_body_cb_t body,
                  void* data,
                  int task_count,
                  int divisor) noexcept {
    const int64_t total = static_cast<int64_t>(task_count);
    const int64_t scale = static_cast<int64_t>(divisor > 0 ? divisor : 1);
    while (!cancel_batch_.load(std::memory_order_relaxed)) {
      const int64_t current = next_task_.load(std::memory_order_relaxed);
      if (current >= total) {
        break;
      }
      const int64_t remaining = total - current;
      const int64_t chunk = std::max<int64_t>(1, remaining / scale);
      const int64_t start = next_task_.fetch_add(chunk, std::memory_order_relaxed);
      if (start >= total) {
        break;
      }
      const int64_t end = std::min<int64_t>(total, start + chunk);
      try {
        body(static_cast<int>(start), static_cast<int>(end), data);
      } catch (...) {
        cancel_batch_.store(true, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!batch_exception_) {
          batch_exception_ = std::current_exception();
        }
        break;
      }
    }
  }

  /**
   * @brief Persistent worker main loop awaiting new batch generations or retirement.
   * @param[in] self Pointer to this worker's state record.
   */
  void worker_loop(Worker* self) noexcept {
    tls_thread_num() = static_cast<int>(self->slot_id);
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      wake_cv_.wait(lock, [this, self] {
        return self->stop ||
               (busy_ && self->slot_index < active_limit_ &&
                self->seen_generation != generation_);
      });
      if (self->stop) {
        return;
      }
      self->seen_generation = generation_;
      const FN_parallel_for_body_cb_t body = current_body_;
      void* const data = current_data_;
      const int task_count = current_tasks_;
      const int divisor = current_divisor_;
      lock.unlock();

      run_chunks(body, data, task_count, divisor);

      lock.lock();
      if (--pending_workers_ == 0) {
        done_cv_.notify_all();
      }
    }
  }

  std::mutex mutex_;                                /**< Guards pool membership and batch lifecycle. */
  std::condition_variable wake_cv_;                 /**< Signals workers on new batches or retirement. */
  std::condition_variable done_cv_;                 /**< Signals batch completion (`pending_workers_ == 0` / `!busy_`). */
  std::vector<std::unique_ptr<Worker>> workers_;    /**< Live persistent workers (`0..configured_threads_-1`). */
  int configured_threads_ = 1;                      /**< Configured team size under `mutex_`. */
  std::atomic<int> atomic_threads_{1};              /**< Mirror of `configured_threads_` for lock-free `getNumThreads()`. */
  size_t scope_depth_ = 0;                          /**< Nesting count of active `ScopedBackend` guards. */
  bool busy_ = false;                               /**< True while a parallel batch owns the worker team. */
  bool shutting_down_ = false;                      /**< Set during destruction to prevent further worker creation. */
  uint64_t generation_ = 0;                         /**< Monotonic batch identifier. */
  size_t active_limit_ = 0;                         /**< Number of leading workers participating in `generation_`. */
  size_t pending_workers_ = 0;                      /**< Participating workers not yet checked in for `generation_`. */
  FN_parallel_for_body_cb_t current_body_ = nullptr;/**< Active batch callback pointer. */
  void* current_data_ = nullptr;                    /**< Active batch opaque callback data. */
  int current_tasks_ = 0;                           /**< Active batch stripe count. */
  int current_divisor_ = 1;                         /**< Active batch guided-scheduling divisor. */
  std::atomic<int64_t> next_task_{0};               /**< Next unclaimed stripe index in `[0, current_tasks_)`. */
  std::atomic<bool> cancel_batch_{false};           /**< Set if a callback throws outside OpenCV's wrapper. */
  std::exception_ptr batch_exception_;              /**< First uncaught callback exception in the batch. */
};

/**
 * @brief Install the process-wide Sensoraia OpenCV parallel backend once.
 * @return Shared handle to the installed `ThreadPoolBackend` instance.
 */
inline const std::shared_ptr<ThreadPoolBackend>& ensure_installed() {
  static const std::shared_ptr<ThreadPoolBackend> backend = [] {
    auto instance = std::make_shared<ThreadPoolBackend>();
    cv::parallel::setParallelForBackend(instance, true);
    return instance;
  }();
  return backend;
}

/**
 * @brief RAII guard installing the backend at startup and joining workers before `main()` exits.
 * @details Avoids leaving worker threads alive into static storage teardown in
 * `-static` executables while still allowing unit tests without `main()` to
 * call `ensure_installed()` directly.
 */
class ScopedBackend {
 public:
  /** @brief Ensure the OpenCV parallel backend is registered and enter a lifetime scope. */
  ScopedBackend() : backend_(ensure_installed()) { backend_->enter_scope(); }

  /** @brief Exit the scope and join idle worker threads when the outermost guard ends. */
  ~ScopedBackend() { backend_->leave_scope(); }

  /** @brief Noncopyable: each guard must pair one scope entry with one exit. */
  ScopedBackend(const ScopedBackend&) = delete;
  /** @brief Nonassignable: a guard cannot transfer an active scope. */
  ScopedBackend& operator=(const ScopedBackend&) = delete;

 private:
  std::shared_ptr<ThreadPoolBackend> backend_; /**< Retained backend instance for symmetric scope exit. */
};

}  // namespace usher_cv_parallel

#endif  // USHER_OPENCV_PARALLEL_HPP
