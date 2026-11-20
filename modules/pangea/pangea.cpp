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
 * @file pangea.cpp
 * @brief Resumable, resource-bounded USHER photo-map assembler.
 * @details Consumes clean USHER JPEGs, validates pose/geometry provenance, and
 * incrementally blends them into checksummed map tiles. Atomic state/checkpoint
 * files allow interrupted runs to resume. MIMIC and explicit real-camera
 * profiles are separate input modes; this flat-ground renderer does not certify
 * map accuracy.
 */

// clang-format off
#include <stddef.h>
#include <stdio.h>
#include <jpeglib.h>
// clang-format on
#include "exifite.h"
extern "C" {
#include "sha256fst.h"
}

#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <dirent.h>
#include <fcntl.h>
#include <linux/ioprio.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csetjmp>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "version.h"
#include "map_storage.h"
#include "camera_profile.h"
#include "opencv_json.hpp"
#include "opencv_parallel.hpp"
#include "jpeg_stream.h"

/** Internal worker implementation; keeping the pipeline in one namespace aids
 * code navigation. */
namespace pangea_detail {
using Json = usher_cv_json::Value;
namespace fs = std::filesystem;
/** Binary-size unit for explicit resident-memory and disk-reserve accounting.
 */
constexpr uint64_t mib = 1024 * 1024;
/** Hard virtual-address ceiling prevents OpenCV allocations exceeding the
 * worker budget. */
constexpr uint64_t memory_limit = 128 * mib;
/** Radians per degree conversion used by pose and camera rotations. */
constexpr double pi = 3.14159265358979323846;
/** Slippy-map world width at zoom 18: 256 pixels per tile times 2^18 tiles. */
constexpr double world_pixels = 256.0 * (1 << 18);
/** Web-Mercator ground resolution scale at the equator for this zoom level. */
constexpr double projected_pixel_m = 2 * pi * 6378137.0 / world_pixels;
/** Slippy-map tile side length in pixels. */
constexpr int tile_size = 256;
/** Bound total accepted source photos and the on-disk checkpoint sequence. */
constexpr int max_photos = 10000;
std::atomic<sig_atomic_t> stopping{0};
static_assert(decltype(stopping)::is_always_lock_free,
              "Signal cancellation requires lock-free atomic access");
pid_t parent_pid = 0;

/** Signal handler records cancellation; normal code performs checkpoint-safe
 * shutdown. */
void stop(int) { stopping = 1; }

/** Throw a worker-level error that reaches the single CLI diagnostic handler.
 */
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

/** Read one clock as fractional seconds for CPU-duty accounting and deadlines.
 * @param clock Clock identifier.
 * @return Seconds with nanosecond fraction. */
double seconds(clockid_t clock) {
  timespec value{};
  check(clock_gettime(clock, &value) == 0, "clock unavailable");
  return value.tv_sec + value.tv_nsec * 1e-9;
}

/** Read one named kB counter from Linux procfs and return bytes.
 * @param path procfs status file.
 * @param wanted Counter name including trailing colon.
 * @return Counter in bytes, or zero when unavailable. */
uint64_t proc_kib(const char* path, const char* wanted) {
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream values(line);
    std::string key, unit;
    uint64_t amount = 0;
    if (values >> key >> amount >> unit && key == wanted && unit == "kB")
      return amount * 1024;
  }
  return 0;
}

class DesktopPool {
  std::vector<std::thread> workers;
  std::mutex mutex;
  std::condition_variable ready, completed;
  std::function<void(size_t)> task;
  size_t next_job = 0, job_count = 0, active = 0;
  size_t peak_active = 0;
  bool shutdown = false;
  std::exception_ptr failure;
  std::atomic<bool> cancel{false};

  /** @brief Claim and execute jobs until the coordinator shuts down the pool.
   * @details Execute callbacks outside the mutex. On failure, stop assigning
   * jobs but let in-flight callbacks drain before the batch returns. */
  void work() {
    std::unique_lock<std::mutex> lock(mutex);
    for (;;) {
      ready.wait(lock, [this] { return shutdown || next_job < job_count; });
      if (shutdown) return;
      const size_t job = next_job++;
      ++active;
      peak_active = std::max(peak_active, active);
      lock.unlock();
      try {
        task(job);
      } catch (...) {
        lock.lock();
        if (!failure) failure = std::current_exception();
        cancel = true;
        next_job = job_count;
        lock.unlock();
      }
      lock.lock();
      --active;
      if (active == 0 && next_job == job_count) completed.notify_one();
    }
  }

 public:
  /** @brief Start the desktop worker pool.
   * @param count Number of worker threads.
   * @details Block termination signals while spawning so workers inherit
   * the mask and shutdown remains the coordinator's responsibility. */
  explicit DesktopPool(size_t count) {
    sigset_t blocked, previous;
    sigemptyset(&blocked);
    sigaddset(&blocked, SIGINT);
    sigaddset(&blocked, SIGTERM);
    check(pthread_sigmask(SIG_BLOCK, &blocked, &previous) == 0,
          "cannot mask desktop worker signals");
    try {
      for (size_t index = 0; index < count; ++index)
        workers.emplace_back([this] { work(); });
      check(pthread_sigmask(SIG_SETMASK, &previous, nullptr) == 0,
            "cannot restore coordinator signals");
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        shutdown = true;
      }
      ready.notify_all();
      for (auto& worker : workers) worker.join();
      pthread_sigmask(SIG_SETMASK, &previous, nullptr);
      throw;
    }
  }
  /** @brief Wake and join the workers before destroying shared batch state.
   * @pre No coordinator call to @ref run is in progress. */
  ~DesktopPool() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      shutdown = true;
    }
    ready.notify_all();
    for (auto& worker : workers) worker.join();
  }
  /** @brief Report the fixed worker capacity.
   * @return Number of threads, whether idle or active. */
  size_t size() const { return workers.size(); }
  /** @brief Read the lifetime concurrency high-water mark.
   * @return Greatest number of simultaneously active jobs. */
  size_t peak_workers() {
    std::lock_guard<std::mutex> lock(mutex);
    return peak_active;
  }
  /** @brief Check whether the current batch has failed or been interrupted.
   * @return True when workers should stop at their next checkpoint. */
  bool cancelled() const { return cancel.load(); }

  /** @brief Run indexed work and wait for every active callback to finish.
   * @param count Number of jobs.
   * @param operation Worker callback; invocations may overlap.
   * @param checkpoint Coordinator callback for resource and stop checks.
   * @details Poll the checkpoint while waiting. Preserve the first failure
   * and rethrow it only after callbacks no longer borrow batch state.
   * @pre One coordinator calls this method; it is not reentrant. */
  void run(size_t count, std::function<void(size_t)> operation,
           const std::function<void()>& checkpoint) {
    std::unique_lock<std::mutex> lock(mutex);
    task = std::move(operation);
    job_count = count;
    next_job = 0;
    failure = nullptr;
    cancel = false;
    ready.notify_all();
    while (active != 0 || next_job != job_count) {
      completed.wait_for(lock, std::chrono::milliseconds(20));
      lock.unlock();
      try {
        checkpoint();
      } catch (...) {
        lock.lock();
        if (!failure) failure = std::current_exception();
        cancel = true;
        next_job = job_count;
        lock.unlock();
      }
      lock.lock();
    }
    task = {};
    if (failure) std::rethrow_exception(failure);
  }
};

/** Apply idle-priority scheduling and cooperative memory/CPU/pause checks. */
class Resources {
  double last_cpu = seconds(CLOCK_PROCESS_CPUTIME_ID);
  double last_wall = seconds(CLOCK_MONOTONIC);
  const double started_cpu = last_cpu;
  const double started_wall = last_wall;
  uint64_t previous_total = 0, previous_idle = 0;
  double idle_fraction = 0;
  double sampled_at = 0;
  uint64_t reserve; /**< RAM left for Linux, camera buffers and concurrent
                       services. */
  double duty; /**< Maximum fraction of one CPU core used over each checkpoint
                  interval. */
  fs::path pause_file; /**< Operator-controlled file whose existence suspends
                          processing. */
  bool desktop_mode = false;
  uint64_t address_limit = memory_limit;
  std::unique_ptr<DesktopPool> pool;
 public:
  /** @brief Establish mapping limits for onboard or desktop execution.
   * @param reserve_mb Required available memory reserve in MiB.
   * @param cpu_percent Onboard CPU budget as a percentage of one core.
   * @param pause Optional path whose existence pauses work.
   * @param desktop Whether to provision desktop workers.
   * @details Keep the onboard policy conservative; desktop concurrency is
   * capped by both available memory and CPU affinity. */
  Resources(uint64_t reserve_mb, int cpu_percent, fs::path pause = {},
            bool desktop = false)
      : reserve(reserve_mb * mib),
        duty(cpu_percent / 100.0),
        pause_file(std::move(pause)), desktop_mode(desktop) {
    check(reserve_mb >= 192 && reserve_mb <= 4096 && cpu_percent >= 1 &&
              cpu_percent <= 20,
          "reserve must be 192..4096 MiB; CPU duty must be 1..20 percent of "
          "one core");
    rlimit address{memory_limit, memory_limit}, core{0, 0}, files{64, 64};
        if (desktop_mode) {
      check(proc_kib("/proc/meminfo", "MemTotal:") >= 2 * 1024 * mib,
        "--desktop requires at least 2 GiB RAM; MORA must use default mode");
      check(getrlimit(RLIMIT_AS, &address) == 0,
        "cannot read inherited address-space limit");
      address_limit = address.rlim_cur == RLIM_INFINITY ? 0 : address.rlim_cur;
      duty = 1;
      files = {256, 256};
        } else {
      check(setrlimit(RLIMIT_AS, &address) == 0,
        "cannot install resource limits");
        }
        check(setrlimit(RLIMIT_CORE, &core) == 0 &&
              setrlimit(RLIMIT_NOFILE, &files) == 0,
          "cannot install resource limits");
    sched_param parameters{};
    if (!desktop_mode) {
    check(setpriority(PRIO_PROCESS, 0, 19) == 0 &&
              sched_setscheduler(0, SCHED_IDLE, &parameters) == 0,
          "cannot enable idle CPU scheduling");
    check(syscall(SYS_ioprio_set, IOPRIO_WHO_PROCESS, 0,
                  IOPRIO_PRIO_VALUE(IOPRIO_CLASS_IDLE, 0)) == 0,
          "cannot enable idle I/O scheduling");
        }
    check(prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) == 0,
          "cannot disable privilege escalation");
    cv::setNumThreads(1);
    if (desktop_mode) {
      cpu_set_t affinity;
      CPU_ZERO(&affinity);
      check(sched_getaffinity(0, sizeof(affinity), &affinity) == 0,
            "desktop CPU affinity unavailable");
      const uint64_t available_memory =
          proc_kib("/proc/meminfo", "MemAvailable:");
      check(available_memory > reserve + memory_limit + 16 * mib,
            "insufficient RAM headroom for desktop workers");
      const uint64_t memory_workers =
          (available_memory - reserve - memory_limit) / (16 * mib);
      const size_t count = std::min<uint64_t>(
          std::min(CPU_COUNT(&affinity), 64), memory_workers);
      check(count > 0, "no desktop worker capacity");
      pool = std::make_unique<DesktopPool>(count);
    }
  }

  /** @brief Check which execution policy was selected.
   * @return True for desktop execution. */
  bool desktop() const { return desktop_mode; }
  /** @brief Check whether tile processing may continue.
   * @details In desktop mode the coordinator samples resources; workers only
   * observe batch cancellation. Serial execution runs the full checkpoint. */
  void tile_checkpoint() {
    if (pool) check(!pool->cancelled(), "desktop tile work cancelled");
    else checkpoint();
  }
  /** @brief Run a desktop batch under the coordinator's resource policy.
   * @param count Number of indexed jobs.
   * @param task Thread-safe job callback.
   * @details Check headroom before dispatch and continue sampling while waiting. */
  void parallel_tiles(size_t count, std::function<void(size_t)> task) {
    check(bool(pool), "desktop worker pool unavailable");
    checkpoint();
    pool->run(count, std::move(task), [this] { checkpoint(); });
  }
  /** @brief Estimate how many photos can be prepared in one desktop batch.
   * @param photo_budget_mib Nonzero estimated peak memory per photo in MiB.
   * @return Batch size from one photo through the worker count.
   * @details Preserve reserve and worker overhead before dividing the remaining
   * headroom. The minimum allows serial preparation, not a memory guarantee.
   * @pre Desktop worker pool is initialized. */
  size_t photo_batch_capacity(uint64_t photo_budget_mib = 128) const {
    const uint64_t available_memory =
        proc_kib("/proc/meminfo", "MemAvailable:");
    const uint64_t overhead = reserve + memory_limit + pool->size() * 16 * mib;
    if (available_memory <= overhead) return 1;
    return std::max<size_t>(1, std::min<uint64_t>(
        pool->size(), (available_memory - overhead) / (photo_budget_mib * mib)));
  }

  /** Report whether system headroom currently meets the configured reserve and
   * duty budget. */
  bool available() {
    if (!pause_file.empty()) {
      struct stat status{};
      if (lstat(pause_file.c_str(), &status) == 0 || errno != ENOENT)
        return false;
    }
    const double now = seconds(CLOCK_MONOTONIC);
    if (desktop_mode)
      return proc_kib("/proc/meminfo", "MemAvailable:") >= reserve;
    // Sample procfs every quarter-second rather than once per pixel, row, or
    // tile.
    if (now - sampled_at >= 0.25) {
      std::ifstream input("/proc/stat");
      std::string label;
      std::array<uint64_t, 8> ticks{};
      input >> label;
      uint64_t total = 0;
      for (auto& value : ticks) {
        input >> value;
        total += value;
      }
      check(input.good() && label == "cpu", "CPU headroom unavailable");
      if (previous_total && total > previous_total &&
          ticks[3] >= previous_idle) {
        idle_fraction = static_cast<double>(ticks[3] - previous_idle) /
                        (total - previous_total);
      }
      previous_total = total;
      previous_idle = ticks[3];
      sampled_at = now;
    }
    const uint64_t resident = proc_kib("/proc/self/status", "VmRSS:");
    const uint64_t available_memory =
        proc_kib("/proc/meminfo", "MemAvailable:");
    // Require one quarter of system CPU to appear idle before this best-effort
    // worker proceeds.
    return resident > 0 &&
           available_memory >=
               reserve + memory_limit - std::min(resident, memory_limit) &&
           idle_fraction >= 0.25;
  }

  /** Bound CPU duty by sleeping and stop work when memory, CPU or pause gates
   * close.
   * @throws std::runtime_error on cancellation or unavailable required system
   * telemetry. */
  /** Sleep to enforce CPU duty and wait while system headroom or operator pause
   * is closed.
   * @throws std::runtime_error on process cancellation or unavailable
   * telemetry. */
  void checkpoint() {
    if (parent_pid && getppid() != parent_pid) stopping = 1;
    check(!stopping, "interrupted; disk checkpoint retained");
    const double now_cpu = seconds(CLOCK_PROCESS_CPUTIME_ID);
    const double now_wall = seconds(CLOCK_MONOTONIC);
    const double rest = desktop_mode ? 0 :
      (now_cpu - last_cpu) / duty - (now_wall - last_wall);
    const double deadline = now_wall + std::max(0.0, rest);
    while (!stopping && seconds(CLOCK_MONOTONIC) < deadline) {
      poll(nullptr, 0,
           static_cast<int>(std::clamp(
               std::ceil((deadline - seconds(CLOCK_MONOTONIC)) * 1000), 1.0,
               1000.0)));
      if (parent_pid && getppid() != parent_pid) stopping = 1;
    }
    while (!stopping && !available()) {
      poll(nullptr, 0, 1000);
      if (parent_pid && getppid() != parent_pid) stopping = 1;
    }
    check(!stopping, "interrupted; disk checkpoint retained");
    last_cpu = now_cpu;
    last_wall = seconds(CLOCK_MONOTONIC);
  }

  /** Summarize active limits and measured process peak in the run report. */
  Json report() {
      Json result = Json::object({{"schema", "PANGEA_RESOURCES_V1"},
            {"address_space_limit_bytes", address_limit},
            {"reserve_bytes", reserve},
            {"cpu_duty_percent", duty * 100 * (pool ? pool->size() : 1)},
            {"scheduler", desktop_mode ? "inherited" : "SCHED_IDLE"},
            {"threads", pool ? static_cast<int>(pool->size()) : cv::getNumThreads()},
            {"available", available()},
        {"peak_rss_bytes", proc_kib("/proc/self/status", "VmHWM:")}});
      if (desktop_mode) {
        result.set("mode", "desktop");
        result.set("cpu_throttled", false);
        const double elapsed = seconds(CLOCK_MONOTONIC) - started_wall;
        const double cpu = seconds(CLOCK_PROCESS_CPUTIME_ID) - started_cpu;
        result.set("elapsed_wall_seconds", elapsed);
        result.set("process_cpu_seconds", cpu);
        result.set("average_cpu_percent", elapsed > 0 ? cpu / elapsed * 100 : 0);
        result.set("peak_active_workers", static_cast<int>(pool->peak_workers()));
      }
      return result;
  }
};

/** RAII closer keeps regular-file descriptors bounded on exception paths. */
struct FileCloser {
  /** @brief Release an owned, non-null file stream.
   * @param file Stream transferred to the deleter.
   * @note Callers that need to detect flush/close failures must close explicitly. */
  void operator()(FILE* file) const { fclose(file); }
};
using File = std::unique_ptr<FILE, FileCloser>;
/** RAII closer for directory scans that may exit through validation errors. */
struct DirectoryCloser {
  /** @brief Release an owned, non-null directory stream.
   * @param directory Stream transferred to the deleter. */
  void operator()(DIR* directory) const { closedir(directory); }
};
using Directory = std::unique_ptr<DIR, DirectoryCloser>;
/** Distinguish invalid source photos (record and continue) from failed map
 * operations. */
struct InvalidPhoto : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/** Reject an unusable photo without treating the whole batch as an
 * infrastructure error. */
void photo_check(bool condition, const char* message) {
  if (!condition) throw InvalidPhoto(message);
}

/** Open a no-follow regular file and enforce its byte ceiling before decoding.
 * @param path Input path.
 * @param maximum Maximum file size in bytes.
 * @return Owning stream, or throws InvalidPhoto for unsafe/unusable input. */
File regular_file(const fs::path& path, uint64_t maximum) {
  int descriptor =
      open(path.c_str(), O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC);
  check(descriptor >= 0, "cannot open regular input");
  struct stat status{};
  if (fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode) ||
      status.st_size < 0 || static_cast<uint64_t>(status.st_size) > maximum) {
    close(descriptor);
    throw InvalidPhoto("input exceeds regular-file limits");
  }
  File result(fdopen(descriptor, "rb"));
  if (!result) close(descriptor);
  check(bool(result), "cannot create input stream");
  return result;
}

/** Persist directory-entry changes after atomic checkpoint publication.
 * @param path Directory to synchronize. */
void sync_directory(const fs::path& path) {
  const int descriptor = open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  check(descriptor >= 0, "cannot open checkpoint directory");
  const int result = fsync(descriptor);
  close(descriptor);
  check(result == 0, "cannot synchronize checkpoint directory");
}

/** Same-directory temporary file committed with fsync+rename for crash-safe
 * checkpoints. */
/** Same-directory temporary output committed with flush, fsync, rename, and
 * directory fsync. */
struct AtomicFile {
  fs::path target, temporary;
  File file;
  /** @param destination Final path; its parent directory must already exist. */
  explicit AtomicFile(const fs::path& destination) : target(destination) {
    std::string pattern = destination.string() + ".tmp-XXXXXX";
    const int descriptor = mkstemp(pattern.data());
    check(descriptor >= 0, "cannot create checkpoint temporary");
    temporary = pattern;
    file.reset(fdopen(descriptor, "wb"));
    if (!file) {
      close(descriptor);
      unlink(temporary.c_str());
    }
    check(bool(file), "cannot open checkpoint stream");
  }
  /** @brief Remove abandoned staging data without removing committed output. */
  ~AtomicFile() {
    if (!temporary.empty()) unlink(temporary.c_str());
  }
  /** Append exact bytes to the temporary output. */
  void write(const void* data, size_t length) {
    check(fwrite(data, 1, length, file.get()) == length,
          "checkpoint write failed");
  }
  /** @brief Append serialized text without adding a terminator or newline.
   * @param value Exact bytes to append. */
  void text(const std::string& value) { write(value.data(), value.size()); }
  /** Durably publish the complete temporary file at target. */
  void commit() {
    check(fflush(file.get()) == 0 && fsync(fileno(file.get())) == 0,
          "checkpoint flush failed");
    check(fclose(file.release()) == 0, "checkpoint close failed");
    check(rename(temporary.c_str(), target.c_str()) == 0,
          "checkpoint publish failed");
    temporary.clear();
    sync_directory(target.parent_path());
  }
};

/** Serialize one JSON checkpoint through AtomicFile rather than truncating live
 * state. */
/** Serialize a small state/profile record through atomic, durable file
 * publication. */
void write_json(const fs::path& path, const Json& value) {
  AtomicFile output(path);
  output.text(value.dump(2) + "\n");
  output.commit();
}

/** Read a bounded regular JSON checkpoint. */
/** Read a regular JSON state file subject to the 16 KiB input ceiling. */
Json read_json(const fs::path& path) {
  File input = regular_file(path, 16384);
  std::array<char, 16385> bytes{};
  size_t count = fread(bytes.data(), 1, bytes.size(), input.get());
  check(!ferror(input.get()) && count <= 16384, "invalid checkpoint length");
  try {
    return Json::parse(bytes.begin(), bytes.begin() + count);
  } catch (const Json::exception& error) {
    throw std::runtime_error("invalid JSON file " + path.string() + ": " + error.what());
  }
}

/** Return lowercase SHA-256 hex for tile payload and source-identity checks. */
std::string digest(const void* data, size_t size) {
  std::array<uint8_t, 32> hash{};
  sha256_bytes(data, size, hash.data());
  std::ostringstream result;
  for (uint8_t byte : hash)
    result << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
  return result.str();
}

/** Fingerprint a source from stable filesystem identity/metadata for resume
 * validation.
 * @param path Candidate photo.
 * @param allow_missing Return empty for a source already removed by its
 * producer.
 * @return SHA-256 identity; content itself is not re-read for every directory
 * scan. */
std::string identity(const fs::path& path, bool allow_missing = false) {
  struct stat status{};
  const int result = lstat(path.c_str(), &status);
  if (allow_missing && ((result != 0 && (errno == ENOENT || errno == ESTALE)) ||
                        (result == 0 && !S_ISREG(status.st_mode))))
    return {};
  check(result == 0 && S_ISREG(status.st_mode),
        "source disappeared or is not regular");
  std::ostringstream value;
  value << path.filename().string() << ':' << status.st_dev << ':'
        << status.st_ino << ':' << status.st_size << ':'
        << status.st_mtim.tv_sec << ':' << status.st_mtim.tv_nsec << ':'
        << status.st_ctim.tv_sec << ':' << status.st_ctim.tv_nsec;
  const std::string text = value.str();
  return digest(text.data(), text.size());
}

/** Read one bounded EXIF text entry without using NUL termination as its
 * length. */
std::string exif_text(const exif_view* metadata, exif_directory directory, uint16_t tag) {
  exif_value entry{};
  if (!exif_get(metadata, directory, tag, &entry)) return {};
  photo_check(entry.size <= EXIF_MAX_PAYLOAD &&
                  (entry.format == EXIF_ASCII || entry.format == EXIF_UNDEFINED), "invalid EXIF field");
  std::string text(reinterpret_cast<const char*>(entry.data), entry.size);
  if (directory == EXIF_CAMERA && tag == EXIF_USER_COMMENT &&
      text.compare(0, 8, std::string("ASCII\0\0\0", 8)) == 0)
    text.erase(0, 8);
  while (!text.empty() && text.back() == '\0') text.pop_back();
  return text;
}

/** Require a finite JSON number within the field's physical or geometric range.
 */
double numeric(const Json& values, const char* key, double minimum,
               double maximum) {
  photo_check(values.contains(key) && values[key].is_number(),
              (std::string("missing numeric geometry: ") + key).c_str());
  const double result = values[key].get<double>();
  photo_check(std::isfinite(result) && result >= minimum && result <= maximum,
              "geometry out of range");
  return result;
}

/** Compose roll, pitch, and yaw matrices in the aircraft frame's declared
 * order. */
cv::Matx33d rotation(double roll, double pitch, double yaw) {
  const cv::Matx33d roll_matrix(1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll),
                                cos(roll));
  const cv::Matx33d pitch_matrix(cos(pitch), 0, sin(pitch), 0, 1, 0,
                                 -sin(pitch), 0, cos(pitch));
  const cv::Matx33d yaw_matrix(cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0,
                               0, 1);
  return yaw_matrix * pitch_matrix * roll_matrix;
}

/** Load explicit real-camera intrinsics and mount geometry; reject unknown
 * fields.
 * @param path Empty for MIMIC, otherwise a PANGEA_CAMERA_V1 key=value file.
 * @return Null for MIMIC, validated camera profile for real JPEGs. */
Json camera_profile(const fs::path& path) {
  if (path.empty()) return nullptr;
  struct usher_camera_profile parsed{};
  check(usher_camera_profile_read(path.c_str(), &parsed),
        "invalid flat real-camera profile");
  const struct coverage_camera& camera = parsed.calibration;
  const std::string prefix(parsed.filename_prefix);
  return Json::object({{"schema", "PANGEA_CAMERA_V1"},
                       {"width", camera.width},
                       {"height", camera.height},
                       {"fx", camera.fx},
                       {"fy", camera.fy},
                       {"cx", camera.cx},
                       {"cy", camera.cy},
                       {"mount_roll_deg", camera.mount_roll_deg},
                       {"mount_pitch_deg", camera.mount_pitch_deg},
                       {"mount_yaw_deg", camera.mount_yaw_deg},
                       {"filename_prefix", prefix}});
}

/** Inclusive slippy-map tile rectangle for accumulated mosaic coverage. */
struct Bounds {
  int left = 0, top = 0, right = -1, bottom = -1;
  /** Empty until an accepted source photo contributes map coverage. */
  bool empty() const { return right < left; }
  /** Serialize the inclusive tile rectangle into checkpoint JSON. */
  Json json() const { return Json::array({left, top, right, bottom}); }
  /** Parse bounds and enforce zoom-18 world and maximum map-side constraints.
   */
  static Bounds from(const Json& value, int max_map_side = 8192) {
    check(value.is_array() && value.size() == 4, "invalid map bounds");
    Bounds result{value[0].get<int>(), value[1].get<int>(), value[2].get<int>(),
                  value[3].get<int>()};
    check(
        result.empty() ||
            (result.left >= 0 && result.top >= 0 && result.right < (1 << 18) &&
         result.bottom < (1 << 18) &&
         result.right - result.left < max_map_side / tile_size &&
         result.bottom - result.top < max_map_side / tile_size),
      "map exceeds configured pixel side or Mercator limits");
    return result;
  }
  /** @brief Extend the committed tile envelope without changing either input.
   * @param other New footprint in tile coordinates.
   * @param max_map_side Maximum allowed map side in pixels.
   * @return Bounding rectangle, not a claim that all enclosed tiles have coverage.
   * @details Reapply map limits to the union; individually acceptable
   * footprints can breach the limit when combined. */
  Bounds include(const Bounds& other, int max_map_side = 8192) const {
    if (empty()) return other;
    return from(Json::array({std::min(left, other.left), std::min(top, other.top),
                 std::max(right, other.right),
                 std::max(bottom, other.bottom)}), max_map_side);
  }
};

/** Decoded source frame, pixel-to-world homography, and intersected tile
 * bounds. */
struct Photo {
  cv::Mat image;
  cv::Matx33d to_world;
  Bounds bounds;
  /** Transform the four source-image corners into another image/tile plane. */
  std::array<cv::Point2f, 4> corners(const cv::Matx33d& transform) const {
    std::array<cv::Point2f, 4> result{};
    const std::array<cv::Point, 4> pixels{{{0, 0},
                                           {image.cols - 1, 0},
                                           {image.cols - 1, image.rows - 1},
                                           {0, image.rows - 1}}};
    for (size_t index = 0; index < pixels.size(); ++index) {
      const cv::Vec3d point =
          transform * cv::Vec3d(pixels[index].x, pixels[index].y, 1);
      result[index] = {float(point[0] / point[2]), float(point[1] / point[2])};
    }
    return result;
  }
  /** Convert source corners back to longitude/latitude for per-photo
   * provenance. */
  Json geographic_corners() const {
    Json result = Json::array();
    for (const cv::Point& pixel :
         std::array<cv::Point, 4>{{{0, 0},
                                   {image.cols - 1, 0},
                                   {image.cols - 1, image.rows - 1},
                                   {0, image.rows - 1}}}) {
      const cv::Vec3d point = to_world * cv::Vec3d(pixel.x, pixel.y, 1);
      const double world_x = point[0] / point[2], world_y = point[1] / point[2];
        result.push_back(Json::array({world_x / world_pixels * 360 - 180,
                      atan(sinh(pi * (1 - 2 * world_y / world_pixels))) * 180 / pi}));
    }
    return result;
  }
  /** Validate source EXIF and derive a flat-ground source-pixel-to-world
  * transform. MIMIC uses embedded geometry; real JPEGs require explicit
   * intrinsics and mount angles so the worker never guesses a sensor model. */
  void geometry(const exif_view* metadata, int width, int height, const Json& camera,
                bool standard_profile) {
    Json provenance;
    const std::string document =
        exif_text(metadata, EXIF_PRIMARY, EXIF_DOCUMENT_NAME);
    standard_profile = standard_profile &&
               document.find("USHER_MOCK_MAP_V1") == std::string::npos;
    if (camera.is_null() && !standard_profile) {
      try {
        provenance = Json::parse(document);
      } catch (const Json::exception&) {
        throw InvalidPhoto(
            "missing MIMIC geometry; real JPEGs require --camera-profile");
      }
      const bool xyz_geometry = provenance.value("tile_scheme", "") == "XYZ";
      // Older MIMIC EXIF used GOOGLE as a cache-layout token, not imagery provenance.
      const bool legacy_cache_geometry = provenance.value("provider", "") == "GOOGLE";
      photo_check(
          provenance.is_object() &&
              provenance.value("schema", "") == "USHER_MOCK_MAP_V1" &&
              provenance.value("projection", "") == "pinhole_flat_ground" &&
              (xyz_geometry || legacy_cache_geometry) &&
              provenance.value("zoom", 0) == 18,
          "only clean USHER MIMIC pinhole photos are supported");
    } else if (!camera.is_null()) {
      photo_check(document.find("USHER_MOCK_MAP_V1") == std::string::npos,
                  "mock photo in real-camera input");
      photo_check(width == camera.at("width") && height == camera.at("height"),
                  "JPEG dimensions differ from real-camera profile");
    }
    photo_check(exif_text(metadata, EXIF_CAMERA, EXIF_USER_COMMENT)
                        .find("USHER_LENS_V1") == std::string::npos,
                "lens derivative cannot be inverted without its shutter/lens "
                "model; use clean photo");
    exif_value orientation{};
    uint32_t orientation_value = 0;
    photo_check(!exif_get(metadata, EXIF_PRIMARY, EXIF_ORIENTATION, &orientation) ||
                    (orientation.format == EXIF_SHORT && orientation.count == 1 &&
                     exif_value_unsigned(&orientation, 0, &orientation_value) && orientation_value == 1),
                "unsupported EXIF orientation");
    std::istringstream description(
        exif_text(metadata, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION));
    std::string field;
    std::getline(description, field, ';');
    Json shot = Json::object();
    const bool flyeye = (standard_profile || !camera.is_null()) &&
                       field.find("FlyEye simulated camera") == 0;
    if (flyeye) {
      const Json pose = Json::parse(
          exif_text(metadata, EXIF_CAMERA, EXIF_USER_COMMENT));
      photo_check(pose.value("source", "") == "FlyEye/MSFS2024" &&
                      pose.value("datum", "") == "WGS-84",
                  "unsupported FlyEye position metadata");
      shot.set("lat_deg", pose.at("latitude_deg"));
      shot.set("lon_deg", pose.at("longitude_deg"));
      shot.set("groundalt_m", pose.at("agl_m"));
      shot.set("roll_deg", 0);
      shot.set("pitch_deg", 0);
      shot.set("yaw_deg", 0);
    } else {
    photo_check(field.find("Paparazzi USHER ") == 0,
                "missing USHER shot metadata");
    std::map<std::string, std::string> fields;
    while (std::getline(description, field, ';')) {
      const size_t start = field.find_first_not_of(" \t\r\n"),
                   end = field.find_last_not_of(" \t\r\n");
      photo_check(start != std::string::npos, "empty metadata field");
      field = field.substr(start, end - start + 1);
      const size_t separator = field.find('=');
      photo_check(separator != std::string::npos &&
                      fields
                          .emplace(field.substr(0, separator),
                                   field.substr(separator + 1))
                          .second,
                  "duplicate or invalid metadata field");
    }
    for (const char* key : {"lat_deg", "lon_deg", "roll_deg", "pitch_deg",
                            "yaw_deg", "groundalt_m"}) {
      std::string selected = key;
      if (camera.is_null() &&
          (selected == "lat_deg" || selected == "lon_deg") &&
          fields.count("original_" + selected))
        selected = "original_" + selected;
      photo_check(fields.count(selected), "missing shot field");
      try {
        size_t consumed = 0;
        shot.set(key, std::stod(fields.at(selected), &consumed));
        photo_check(consumed == fields.at(selected).size(),
                    "invalid shot number");
      } catch (const std::logic_error&) {
        throw InvalidPhoto("invalid shot number");
      }
    }
    }
    const double latitude = numeric(shot, "lat_deg", -85, 85) * pi / 180;
    const double longitude = numeric(shot, "lon_deg", -180, 180);
    const double altitude = numeric(shot, "groundalt_m", 0.1, 10000) /
                            (projected_pixel_m * cos(latitude));
    const double roll = numeric(shot, "roll_deg", -180, 180) * pi / 180;
    const double pitch = numeric(shot, "pitch_deg", -180, 180) * pi / 180;
    const double yaw = numeric(shot, "yaw_deg", -360, 360) * pi / 180;
    cv::Matx33d rays, mount = cv::Matx33d::eye();
    if (camera.is_null()) {
      const double horizontal =
        standard_profile ? tan(60 * pi / 360) :
        tan(numeric(provenance, "horizontal_fov_deg", 1, 170) * pi / 360);
      const double vertical =
        standard_profile ? horizontal * (height - 1) / (width - 1) :
        tan(numeric(provenance, "vertical_fov_deg", 1, 170) * pi / 360);
      rays = cv::Matx33d(0, -2 * vertical / (height - 1), vertical,
                         2 * horizontal / (width - 1), 0, -horizontal, 0, 0, 1);
    } else {
      const double fx = camera.at("fx").get<double>(), fy = camera.at("fy").get<double>(),
           cx = camera.at("cx").get<double>(), cy = camera.at("cy").get<double>();
      rays = cv::Matx33d(0, -1 / fy, cy / fy, 1 / fx, 0, -cx / fx, 0, 0, 1);
      mount = rotation(camera.at("mount_roll_deg").get<double>() * pi / 180,
                       camera.at("mount_pitch_deg").get<double>() * pi / 180,
                       camera.at("mount_yaw_deg").get<double>() * pi / 180);
    }
    const cv::Matx33d ground = rotation(roll, pitch, yaw) * mount * rays;
    const double center_x = (longitude + 180) / 360 * world_pixels;
    const double center_y =
        (1 - asinh(tan(latitude)) / pi) * 0.5 * world_pixels;
    to_world =
        cv::Matx33d(0, altitude, center_x, -altitude, 0, center_y, 0, 0, 1) *
        ground;
    update_bounds(width, height);
  }

  /** @brief Recompute the tile footprint after setting the image homography.
   * @param width Source columns in pixels.
   * @param height Source rows in pixels.
   * @details Check homogeneous depth before dividing. Reject horizon,
   * dateline and oversized footprints instead of creating unbounded tile work.
   * @throws InvalidPhoto If the candidate footprint violates a mapping limit. */
  void update_bounds(int width, int height) {
    double minimum_x = INFINITY, minimum_y = INFINITY, maximum_x = -INFINITY,
           maximum_y = -INFINITY;
    for (int row : {0, height - 1})
      for (int column : {0, width - 1}) {
        const cv::Vec3d point = to_world * cv::Vec3d(column, row, 1);
        photo_check(point[2] > 0.01, "footprint reaches the horizon");
        const double world_x = point[0] / point[2],
                     world_y = point[1] / point[2];
        photo_check(std::isfinite(world_x) && std::isfinite(world_y) &&
                        world_x >= 0 && world_y >= 0 &&
                        world_x < world_pixels && world_y < world_pixels,
                    "footprint crosses Mercator/dateline boundary");
        minimum_x = std::min(minimum_x, world_x);
        maximum_x = std::max(maximum_x, world_x);
        minimum_y = std::min(minimum_y, world_y);
        maximum_y = std::max(maximum_y, world_y);
      }
    bounds = {
        int(floor(minimum_x / tile_size)), int(floor(minimum_y / tile_size)),
        int(floor(maximum_x / tile_size)), int(floor(maximum_y / tile_size))};
    photo_check(bounds.right - bounds.left < 16 &&
                    bounds.bottom - bounds.top < 16 &&
                    (bounds.right - bounds.left + 1) *
                            (bounds.bottom - bounds.top + 1) <=
                        64,
                "photo footprint exceeds 64-tile work limit");
  }
};

struct JpegError {
  jpeg_error_mgr manager{};
  jmp_buf recovery;
  void (*emit)(j_common_ptr, int) = nullptr;
};
/** @brief Route a libjpeg fatal callback to the active recovery point.
 * @param state JPEG state whose error manager owns the jump buffer.
 * @warning Does not return; recovery must be established before codec calls. */
void jpeg_failure(j_common_ptr state) {
  longjmp(reinterpret_cast<JpegError*>(state->err)->recovery, 1);
}
/** @brief Screen scan diagnostics before dispatching to libjpeg's usual handler.
 * @param state JPEG state with the installed mapper error manager.
 * @param level libjpeg message level. */
void jpeg_message(j_common_ptr state, int level) {
  pangea_survey::validateJpegScanMessage(state);
  reinterpret_cast<JpegError*>(state->err)->emit(state, level);
}
struct Decoder {
  jpeg_decompress_struct state{};
  JpegError error{};
  bool created = false;
  exif_view metadata{};
  Photo photo;
  /** @brief Release codec allocations only if creation succeeded. */
  ~Decoder() {
    if (created) jpeg_destroy_decompress(&state);
  }
};

/** @brief Admit and decode one source JPEG into a projection-ready photo.
 * @param path Source JPEG path.
 * @param resources Resource and cancellation policy.
 * @param camera Camera calibration/profile fields.
 * @param standard_profile Whether the standard camera profile is active.
 * @param parallel_decode Whether to use reusable per-worker decode state.
 * @return Decoded image with its transform and tile footprint.
 * @details Reuse decoder storage only within a worker thread; discard that
 * state after fatal libjpeg recovery rather than reusing a failed codec. */
Photo read_photo(const fs::path& path, Resources& resources,
                 const Json& camera, bool standard_profile = false,
                 bool parallel_decode = false) {
  File input = regular_file(path, 32 * mib);
  static thread_local std::unique_ptr<Decoder> cached_decoder;
  std::unique_ptr<Decoder> local_decoder;
  if (parallel_decode) {
    if (!cached_decoder) cached_decoder = std::make_unique<Decoder>();
  } else local_decoder = std::make_unique<Decoder>();
  Decoder* const volatile decoder = parallel_decode ? cached_decoder.get() : local_decoder.get();
  decoder->state.err = jpeg_std_error(&decoder->error.manager);
  decoder->error.manager.error_exit = jpeg_failure;
  decoder->error.emit = decoder->error.manager.emit_message;
  decoder->error.manager.emit_message = jpeg_message;
  if (setjmp(decoder->error.recovery)) {
    if (parallel_decode) cached_decoder.reset();
    throw InvalidPhoto("invalid JPEG stream");
  }
  if (decoder->created) {
    jpeg_abort_decompress(&decoder->state);
    decoder->metadata = {};
    decoder->photo = {};
  } else {
    decoder->created = true;
    jpeg_create_decompress(&decoder->state);
  }
  jpeg_stdio_src(&decoder->state, input.get());
  jpeg_save_markers(&decoder->state, JPEG_APP0 + 1, 65533);
  photo_check(jpeg_read_header(&decoder->state, TRUE) == JPEG_HEADER_OK,
              "invalid JPEG header");
  const unsigned width = decoder->state.image_width,
                 height = decoder->state.image_height;
  photo_check(width >= 2 && height >= 2 && width <= 4096 && height <= 4096 &&
                  uint64_t(width) * height <=
                      (camera.is_null() ? 8388608u : 16777216u) &&
                  !decoder->state.progressive_mode,
              "unsupported JPEG size or progressive encoding");
  for (jpeg_saved_marker_ptr marker = decoder->state.marker_list; marker;
       marker = marker->next) {
    if (marker->marker != JPEG_APP0 + 1 || marker->data_length < 6 ||
        std::string(reinterpret_cast<const char*>(marker->data), 6) !=
            std::string("Exif\0\0", 6))
      continue;
    photo_check(!decoder->metadata.tiff, "multiple EXIF records");
    photo_check(exif_parse(&decoder->metadata, marker->data, marker->data_length) == 0,
                "malformed EXIF");
  }
  photo_check(decoder->metadata.tiff != nullptr, "missing EXIF");
  try {
    decoder->photo.geometry(&decoder->metadata, width, height, camera,
                standard_profile);
  } catch (const Json::exception&) {
    throw InvalidPhoto("malformed EXIF geometry");
  }
  if (parallel_decode) resources.tile_checkpoint();
  else resources.checkpoint();
  decoder->state.out_color_space = JCS_RGB;
  jpeg_start_decompress(&decoder->state);
  decoder->photo.image.create(height, width, CV_8UC3);
  while (decoder->state.output_scanline < height) {
    if (decoder->state.output_scanline % 32 == 0) {
      if (parallel_decode) resources.tile_checkpoint();
      else resources.checkpoint();
    }
    JSAMPROW row = decoder->photo.image.ptr(decoder->state.output_scanline);
    photo_check(jpeg_read_scanlines(&decoder->state, &row, 1) == 1,
                "incomplete JPEG scanline");
  }
  jpeg_finish_decompress(&decoder->state);
  photo_check(decoder->error.manager.num_warnings == 0, "damaged JPEG");
  return std::move(decoder->photo);
}

/** Per-tile four-channel sums (RGB weighted totals and accumulated feather
 * weight). */
/** Per-tile RGB weighted sums plus total feather weight, stored for resumable
 * blending. */
struct Tile {
  uint64_t sequence = 0;
  cv::Mat sums{tile_size, tile_size, CV_32FC4, cv::Scalar::all(0)};
  /** Load and verify an existing tile; absent files represent zero coverage. */
  void load(const fs::path& path) {
    if (!fs::exists(path)) return;
    File input = regular_file(path, sums.total() * sums.elemSize() + 512);
    std::array<char, 512> header{};
    check(fgets(header.data(), header.size(), input.get()) != nullptr,
          "tile header missing");
    Json record;
    try {
      record = Json::parse(header.data());
    } catch (const Json::exception& error) {
      throw std::runtime_error("invalid tile header " + path.string() + ": " + error.what());
    }
    check(record.value("schema", "") == "PANGEA_TILE_V1",
          "invalid tile format");
    sequence = record.at("sequence").get<uint64_t>();
    check(sequence <= max_photos, "invalid tile sequence");
    const size_t length = sums.total() * sums.elemSize();
    check(fread(sums.data, 1, length, input.get()) == length &&
              fgetc(input.get()) == EOF && !ferror(input.get()),
          "invalid tile data length");
    check(record.at("sha256") == digest(sums.data, length),
          "tile checksum mismatch");
    check(cv::checkRange(sums, true, nullptr, 0, 255.0 * max_photos + 1),
          "invalid tile accumulation");
  }
  /** Atomically persist tile data and sequence so a pending photo can be
   * replayed safely. */
  void save(const fs::path& path) {
    const size_t length = sums.total() * sums.elemSize();
    AtomicFile output(path);
    output.text(Json::object({{"schema", "PANGEA_TILE_V1"},
                     {"sequence", sequence},
             {"sha256", digest(sums.data, length)}})
                    .dump() +
                "\n");
    output.write(sums.data, length);
    output.commit();
  }
};

/** Return the deterministic checkpoint path for one zoom-18 tile coordinate. */
fs::path tile_path(const fs::path& root, int column, int row) {
  return root / "tiles" /
         (std::to_string(column) + "-" + std::to_string(row) + ".tile");
}

/** Retain a 512 MiB filesystem reserve in addition to the next operation
 * estimate.
 * @param root Output/checkpoint filesystem path.
 * @param required Estimated additional bytes for the pending operation. */
void disk_headroom(const fs::path& root, uint64_t required) {
  struct statvfs status{};
  check(statvfs(root.c_str(), &status) == 0, "disk headroom unavailable");
  // Retain 512 MiB free for the OS, camera services and recovery beyond this
  // operation's estimate.
  check(uint64_t(status.f_bavail) * status.f_frsize >= 512 * mib + required,
        "disk reserve reached; checkpoint retained, free space and resume");
}

/** Blend one validated photo into intersecting tiles, checkpointing each tile
 * atomically.
 * @param photo Source pixels, projection, and affected tile bounds.
 * @param root Tile/state directory.
 * @param sequence Unique accepted-photo sequence used to prevent duplicate
 * contribution.
 * @param resources Cooperative CPU/RAM/pause policy.
 * @param tile_column Independent destination tile column.
 * @param tile_row Independent destination tile row. */
void blend_tile(const Photo& photo, const fs::path& root, uint64_t sequence,
           Resources& resources, int tile_column, int tile_row) {
    resources.tile_checkpoint();
      disk_headroom(root, 2 * mib);
      const cv::Matx33d local = cv::Matx33d(1, 0, -tile_column * tile_size, 0,
                                            1, -tile_row * tile_size, 0, 0, 1) *
                                photo.to_world;
      const auto footprint = photo.corners(local);
      const std::array<cv::Point2f, 4> rectangle{
          {{0, 0},
           {tile_size - 1, 0},
           {tile_size - 1, tile_size - 1},
           {0, tile_size - 1}}};
      std::vector<cv::Point2f> intersection;
      if (cv::intersectConvexConvex(footprint, rectangle, intersection) <= 0)
        return;
      const fs::path path = tile_path(root, tile_column, tile_row);
      Tile tile;
      tile.load(path);
      check(tile.sequence <= sequence, "tile newer than pending photo");
      if (tile.sequence == sequence) return;
      bool contributed = false;
      // Process 16-row strips so warp temporaries stay small on the 512 MiB
      // board.
      for (int start_row = 0; start_row < tile_size; start_row += 16) {
        resources.tile_checkpoint();
        const cv::Matx33d transform =
            cv::Matx33d(1, 0, -tile_column * tile_size, 0, 1,
                        -tile_row * tile_size - start_row, 0, 0, 1) *
            photo.to_world;
        const cv::Matx33d inverse = transform.inv();
        cv::Mat warped;
        cv::warpPerspective(photo.image, warped, transform,
                            cv::Size(tile_size, 16), cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT);
        for (int row = 0; row < warped.rows; ++row) {
          auto* sums = tile.sums.ptr<cv::Vec4f>(start_row + row);
          const auto* colors = warped.ptr<cv::Vec3b>(row);
          for (int column = 0; column < tile_size; ++column) {
            const cv::Vec3d source = inverse * cv::Vec3d(column, row, 1);
            if (source[2] <= 0) continue;
            const double source_x = source[0] / source[2],
                         source_y = source[1] / source[2];
            const double edge =
                std::min({source_x, source_y, photo.image.cols - 1 - source_x,
                          photo.image.rows - 1 - source_y});
            if (edge < 1) continue;
            // Feather the source edge over 12.5% of its smaller dimension to
            // soften seams.
            const float weight = std::min(
                1.0,
                edge / (0.125 * std::min(photo.image.cols, photo.image.rows)));
            contributed = true;
            for (int channel = 0; channel < 3; ++channel)
              sums[column][channel] += colors[column][channel] * weight;
            sums[column][3] += weight;
          }
        }
      }
      if (contributed) {
        tile.sequence = sequence;
        tile.save(path);
      }
}

/** @brief Apply one photo to the tiles enclosed by its footprint.
 * @param photo Validated source image and projection.
 * @param root Output map directory.
 * @param sequence Pending photo sequence used for replay protection.
 * @param resources Serial or desktop execution policy.
 * @details Desktop jobs operate on distinct tile files; onboard processing
 * walks the same tile envelope serially to limit resource use. */
void blend(const Photo& photo, const fs::path& root, uint64_t sequence,
           Resources& resources) {
  if (resources.desktop()) {
    const int columns = photo.bounds.right - photo.bounds.left + 1;
    const int rows = photo.bounds.bottom - photo.bounds.top + 1;
    resources.parallel_tiles(static_cast<size_t>(columns) * rows,
        [&](size_t index) {
          blend_tile(photo, root, sequence, resources,
                     photo.bounds.left + static_cast<int>(index % columns),
                     photo.bounds.top + static_cast<int>(index / columns));
        });
    return;
  }
  for (int tile_row = photo.bounds.top; tile_row <= photo.bounds.bottom;
       ++tile_row) {
    for (int tile_column = photo.bounds.left; tile_column <= photo.bounds.right;
         ++tile_column) {
      blend_tile(photo, root, sequence, resources, tile_column, tile_row);
    }
  }
}

struct ProjectedTile {
  int column = 0, row = 0;
  cv::Mat colors, weights;
};

struct ImageFeatures {
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  cv::Matx33d to_world;
  cv::Matx33d position_prior;
  int width = 0, height = 0;
  std::string name;
};

struct PreparedPhoto {
  Photo photo;
  std::vector<ProjectedTile> tiles;
  std::unique_ptr<ImageFeatures> features;
};

/** @brief Stage projected colour and weights without modifying durable tiles.
 * @param[in,out] prepared Decoded photo and destination for its contributions.
 * @param resources Resource and cancellation policy.
 * @details Reject tiles outside the actual quadrilateral before allocating
 * buffers. Separate projection from commit so photos can be prepared in parallel. */
void prepare_projection(PreparedPhoto& prepared, Resources& resources) {
  const Photo& photo = prepared.photo;
  static thread_local std::vector<cv::Point2f> intersection;
  for (int tile_row = photo.bounds.top; tile_row <= photo.bounds.bottom; ++tile_row) {
    for (int tile_column = photo.bounds.left; tile_column <= photo.bounds.right;
         ++tile_column) {
      resources.tile_checkpoint();
      const cv::Matx33d local =
          cv::Matx33d(1, 0, -tile_column * tile_size, 0, 1,
                      -tile_row * tile_size, 0, 0, 1) * photo.to_world;
      const auto footprint = photo.corners(local);
      const std::array<cv::Point2f, 4> rectangle{{{0, 0}, {tile_size - 1, 0},
          {tile_size - 1, tile_size - 1}, {0, tile_size - 1}}};
      intersection.clear();
      if (cv::intersectConvexConvex(footprint, rectangle, intersection) <= 0)
        continue;
      ProjectedTile tile;
      tile.column = tile_column;
      tile.row = tile_row;
      tile.colors.create(tile_size, tile_size, CV_8UC3);
      tile.weights = cv::Mat(tile_size, tile_size, CV_32FC1, cv::Scalar::all(0));
      bool contributed = false;
      for (int start_row = 0; start_row < tile_size; start_row += 16) {
        resources.tile_checkpoint();
        const cv::Matx33d transform =
            cv::Matx33d(1, 0, -tile_column * tile_size, 0, 1,
                        -tile_row * tile_size - start_row, 0, 0, 1) * photo.to_world;
        const cv::Matx33d inverse = transform.inv();
        cv::Mat colors = tile.colors.rowRange(start_row, start_row + 16);
        cv::warpPerspective(photo.image, colors, transform, cv::Size(tile_size, 16),
                            cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        for (int row = 0; row < 16; ++row) {
          auto* weights = tile.weights.ptr<float>(start_row + row);
          for (int column = 0; column < tile_size; ++column) {
            const cv::Vec3d source = inverse * cv::Vec3d(column, row, 1);
            if (source[2] <= 0) continue;
            const double source_x = source[0] / source[2];
            const double source_y = source[1] / source[2];
            const double edge = std::min({source_x, source_y,
                photo.image.cols - 1 - source_x, photo.image.rows - 1 - source_y});
            if (edge < 1) continue;
            weights[column] = std::min(1.0,
                edge / (0.125 * std::min(photo.image.cols, photo.image.rows)));
            contributed = true;
          }
        }
      }
      if (contributed) prepared.tiles.push_back(std::move(tile));
    }
  }
}

/** @brief Commit a prepared photo's contributions to independent tile files.
 * @param prepared Precomputed colour and weight data for this photo.
 * @param root Output map directory.
 * @param sequence Pending photo sequence.
 * @param resources Desktop execution policy.
 * @details Skip tiles already at this sequence to resume a partially
 * committed photo without double-counting its weights. */
void blend_prepared(const PreparedPhoto& prepared, const fs::path& root,
                    uint64_t sequence, Resources& resources) {
  resources.parallel_tiles(prepared.tiles.size(), [&](size_t index) {
    resources.tile_checkpoint();
    disk_headroom(root, 2 * mib);
    const ProjectedTile& projected = prepared.tiles[index];
    const fs::path path = tile_path(root, projected.column, projected.row);
    Tile tile;
    tile.load(path);
    check(tile.sequence <= sequence, "tile newer than pending photo");
    if (tile.sequence == sequence) return;
    for (int row = 0; row < tile_size; ++row) {
      if (row % 16 == 0) resources.tile_checkpoint();
      auto* sums = tile.sums.ptr<cv::Vec4f>(row);
      const auto* colors = projected.colors.ptr<cv::Vec3b>(row);
      const auto* weights = projected.weights.ptr<float>(row);
      for (int column = 0; column < tile_size; ++column) {
        if (weights[column] <= 0) continue;
        for (int channel = 0; channel < 3; ++channel)
          sums[column][channel] += colors[column][channel] * weights[column];
        sums[column][3] += weights[column];
      }
    }
    tile.sequence = sequence;
    tile.save(path);
  });
}

struct Encoder {
  jpeg_compress_struct state{};
  JpegError error{};
  bool created = false;
  cv::Mat stripe, covered;
  /** @brief Release initialized compressor state on success or recovery. */
  ~Encoder() {
    if (created) jpeg_destroy_compress(&state);
  }
};

/** Stream the committed tile bounds into a snapshot JPEG, mask, world file, and
 * VRT.
 * @param root Map output directory.
 * @param state Durable sequence/bounds checkpoint.
 * @param resources Cooperative resource policy.
 * @param max_map_side Explicit pixel-side ceiling for this invocation. */
void export_map(const fs::path& root, const Json& state, Resources& resources,
                int max_map_side) {
  const Bounds bounds = Bounds::from(state.at("bounds"), max_map_side);
  if (bounds.empty()) return;
  const int width = (bounds.right - bounds.left + 1) * tile_size;
  const int height = (bounds.bottom - bounds.top + 1) * tile_size;
  disk_headroom(root, uint64_t(width) * height * 4 + 8 * mib);
  const std::string name =
      "snapshot-" + std::to_string(state.at("sequence").get<uint64_t>());
  const fs::path destination = root / name;
  if (fs::is_symlink(root / "latest") &&
      fs::read_symlink(root / "latest") == name) {
    const Json published = read_json(destination / "map.json");
    check(published.at("sequence") == state.at("sequence") &&
              published.at("bounds") == state.at("bounds"),
          "published snapshot disagrees with checkpoint");
    return;
  }
  fs::create_directory(destination);
  AtomicFile output(destination / "map.jpg"),
      mask(destination / "map-mask.pgm");
  mask.text("P5\n" + std::to_string(width) + " " + std::to_string(height) +
            "\n255\n");
  auto encoder = std::make_unique<Encoder>();
  encoder->state.err = jpeg_std_error(&encoder->error.manager);
  encoder->error.manager.error_exit = jpeg_failure;
  if (setjmp(encoder->error.recovery))
    throw std::runtime_error("map JPEG encoding failed");
  jpeg_create_compress(&encoder->state);
  encoder->created = true;
  jpeg_stdio_dest(&encoder->state, output.file.get());
  encoder->state.image_width = width;
  encoder->state.image_height = height;
  encoder->state.input_components = 3;
  encoder->state.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder->state);
  jpeg_set_quality(&encoder->state, 95, TRUE);
  jpeg_start_compress(&encoder->state, TRUE);
  const int stripe_rows = width > 8192 && !resources.desktop() ? 16 : tile_size;
  for (int stripe_top = bounds.top * tile_size;
       stripe_top < (bounds.bottom + 1) * tile_size; stripe_top += stripe_rows) {
    const int tile_row = stripe_top / tile_size;
    const int start_row = stripe_top % tile_size;
    resources.checkpoint();
    encoder->stripe = cv::Mat(stripe_rows, width, CV_8UC3, cv::Scalar::all(0));
    encoder->covered = cv::Mat(stripe_rows, width, CV_8UC1, cv::Scalar::all(0));
    const auto fill_tile = [&](int tile_column) {
      resources.tile_checkpoint();
      Tile tile;
      tile.load(tile_path(root, tile_column, tile_row));
      for (int row = 0; row < stripe_rows; ++row) {
        if (resources.desktop() && row % 16 == 0) resources.tile_checkpoint();
        auto* colors = encoder->stripe.ptr<cv::Vec3b>(row) +
                       (tile_column - bounds.left) * tile_size;
        auto* valid = encoder->covered.ptr<uint8_t>(row) +
                      (tile_column - bounds.left) * tile_size;
        const auto* sums = tile.sums.ptr<cv::Vec4f>(start_row + row);
        for (int column = 0; column < tile_size; ++column)
          if (sums[column][3] > 0) {
            for (int channel = 0; channel < 3; ++channel)
              colors[column][channel] = cv::saturate_cast<uint8_t>(
                  sums[column][channel] / sums[column][3]);
            valid[column] = 255;
          }
      }
    };
    if (resources.desktop()) {
      resources.parallel_tiles(bounds.right - bounds.left + 1,
          [&](size_t index) {
            fill_tile(bounds.left + static_cast<int>(index));
          });
    } else {
      for (int tile_column = bounds.left; tile_column <= bounds.right;
           ++tile_column)
        fill_tile(tile_column);
    }
    for (int row = 0; row < stripe_rows; ++row) {
      if (row % 16 == 0) resources.checkpoint();
      JSAMPROW pixels = encoder->stripe.ptr(row);
      check(jpeg_write_scanlines(&encoder->state, &pixels, 1) == 1,
            "map scanline write failed");
      mask.write(encoder->covered.ptr(row), width);
    }
  }
  jpeg_finish_compress(&encoder->state);
  output.commit();
  mask.commit();
  const double origin_x =
      (bounds.left * tile_size - world_pixels / 2) * projected_pixel_m;
  const double origin_y =
      (world_pixels / 2 - bounds.top * tile_size) * projected_pixel_m;
  std::ostringstream world;
  world << std::setprecision(17) << projected_pixel_m << "\n0\n0\n"
        << -projected_pixel_m << '\n'
        << origin_x << '\n'
        << origin_y << '\n';
  AtomicFile world_file(destination / "map.jgw");
  world_file.text(world.str());
  world_file.commit();
  std::ostringstream vrt;
  vrt << std::setprecision(17) << "<VRTDataset rasterXSize=\"" << width
      << "\" rasterYSize=\"" << height
      << "\"><SRS>EPSG:3857</SRS><GeoTransform>"
      << origin_x - projected_pixel_m / 2 << ',' << projected_pixel_m << ",0,"
      << origin_y + projected_pixel_m / 2 << ",0," << -projected_pixel_m
      << "</GeoTransform>";
  for (int band = 1; band <= 4; ++band) {
    vrt << "<VRTRasterBand dataType=\"Byte\" band=\"" << band
        << "\"><ColorInterp>"
        << std::array<const char*, 4>{"Red", "Green", "Blue", "Alpha"}[band - 1]
        << "</ColorInterp><SimpleSource><SourceFilename relativeToVRT=\"1\">"
        << (band == 4 ? "map-mask.pgm" : "map.jpg")
        << "</SourceFilename><SourceBand>" << (band == 4 ? 1 : band)
        << "</SourceBand></SimpleSource></VRTRasterBand>";
  }
  vrt << "</VRTDataset>\n";
  AtomicFile vrt_file(destination / "map.vrt");
  vrt_file.text(vrt.str());
  vrt_file.commit();
  Json metadata = state;
    metadata.set("resources", resources.report());
    metadata.set("crs", "EPSG:3857");
    metadata.set("projected_pixel_m", projected_pixel_m);
    metadata.set("width", width);
    metadata.set("height", height);
    metadata.set("model", "pose_pinhole_flat_ground_feather_v1");
    metadata.set("source_mode",
      state.value("standard_profile", false) ? "assumed_camera" :
      state.value("camera", Json()).is_null() ? "mimic" : "real_camera");
    metadata.set("limitations",
      "Flat-ground pinhole projection of input JPEGs; no terrain/3D "
      "reconstruction, lens correction, added effects or accuracy "
      "certification");
  if (state.value("align_images", false)) {
    metadata.set("model", "neighbor_similarity_registered_visual_map_v1");
    metadata.set("limitations",
        "Visual neighbor registration with pose-selected candidates and pose "
        "fallbacks; first-anchor scale/position and accumulated drift are not "
        "surveyed accuracy. Source overlays, parallax and renderer artifacts "
        "remain; no GPS correction or terrain reconstruction.");
  }
  write_json(destination / "map.json", metadata);
  const fs::path temporary = root / ".latest-next";
  unlink(temporary.c_str());
  check(symlink(name.c_str(), temporary.c_str()) == 0 &&
            rename(temporary.c_str(), (root / "latest").c_str()) == 0,
        "cannot publish completed map snapshot");
  sync_directory(root);
  for (const auto& entry : fs::directory_iterator(root)) {
    if (!entry.is_directory() || entry.is_symlink() ||
        entry.path().filename().string().rfind("snapshot-", 0) != 0 ||
        entry.path().filename() == name)
      continue;
    for (const char* file :
         {"map.jpg", "map-mask.pgm", "map.jgw", "map.vrt", "map.json"})
      fs::remove(entry.path() / file);
    std::error_code error;
    fs::remove(entry.path(), error);
  }
  std::cout << "PANGEA: map published " << (root / "latest/map.jpg") << '\n';
}

/** Coordinate discovery, validation, resumable blending, and snapshot
 * publication. */
class Worker {
  fs::path input, root;
  Resources& resources;
  Json state;
  pangea_survey::StorageDescriptor output_lock, process_lock;
  bool allow_prompt = false;
  bool profile_asked = false;
  int max_map_side = 8192;
  bool align_images = false;
  using AlignmentReference = ImageFeatures;
  std::vector<AlignmentReference> references;

  /** @brief Extract bounded SIFT features for a possible alignment reference.
   * @param photo Source image and initial world transform.
   * @param name Source name for the alignment record.
   * @return Descriptors and keypoints expressed in original image pixels.
   * @details Detect on reduced, border-masked imagery to limit feature work;
   * restore original coordinates before fitting geometry. */
  AlignmentReference features(const Photo& photo, const std::string& name) {
    struct Workspace {
      cv::Mat resized, gray, mask;
      cv::Ptr<cv::SIFT> detector = cv::SIFT::create(4000);
    };
    static thread_local Workspace workspace;
    const double scale = std::min(1.0, 1024.0 / std::max(photo.image.cols, photo.image.rows));
    cv::resize(photo.image, workspace.resized, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::cvtColor(workspace.resized, workspace.gray, cv::COLOR_RGB2GRAY);
    workspace.mask.create(workspace.gray.size(), CV_8UC1);
    workspace.mask.setTo(0);
    const int margin = std::max(1, int(40 * scale));
    workspace.mask(cv::Rect(margin, 2 * margin, workspace.gray.cols - 2 * margin,
                           workspace.gray.rows - 3 * margin)).setTo(255);
    AlignmentReference current;
    current.width = photo.image.cols;
    current.height = photo.image.rows;
    current.name = name;
    current.to_world = photo.to_world;
    current.position_prior = photo.to_world;
    workspace.detector->detectAndCompute(workspace.gray, workspace.mask,
                       current.keypoints, current.descriptors);
    for (auto& point : current.keypoints) point.pt *= float(1 / scale);
    return current;
  }

  /** @brief Try to refine a photo's georeferenced transform using recent photos.
   * @param[in,out] photo Source whose transform may be updated.
   * @param name Source name for evidence and reference tracking.
   * @param[in,out] cached Optional precomputed features, moved from when used.
   * @return Alignment decision and available supporting evidence.
   * @details Use position priors to screen candidate references; keep the
   * original projection if no feature match passes the geometric checks. */
  Json align(Photo& photo, const std::string& name, ImageFeatures* cached = nullptr) {
    Json result = Json::object({{"status", "gps_fallback"}});
    const int margin = 40;
    if (photo.image.cols <= 2 * margin || photo.image.rows <= 3 * margin)
      return Json::object({{"status", "image_too_small"}});
    AlignmentReference current = cached ? std::move(*cached) : features(photo, name);
    size_t best_inliers = 0;
    cv::Matx33d selected = photo.to_world;
    const cv::Vec3d center_pixel((photo.image.cols - 1) / 2.0,
                                (photo.image.rows - 1) / 2.0, 1);
    const cv::Vec3d predicted_center = photo.to_world * center_pixel;
    struct Candidate {
      size_t inliers = 0;
      cv::Matx33d to_world;
      Json evidence;
    };
    std::vector<Candidate> candidates(references.size());
    resources.parallel_tiles(references.size(), [&](size_t candidate_index) {
      const auto* reference = &references[references.size() - 1 - candidate_index];
      if (current.descriptors.empty() || reference->descriptors.empty()) return;
      const cv::Vec3d relative = reference->position_prior.inv() * predicted_center;
      if (relative[2] <= 0 || relative[0] / relative[2] < -reference->width * 0.25 ||
          relative[0] / relative[2] > reference->width * 1.25 ||
          relative[1] / relative[2] < -reference->height * 0.25 ||
          relative[1] / relative[2] > reference->height * 1.25) return;
      resources.tile_checkpoint();
      struct Workspace {
        cv::BFMatcher matcher{cv::NORM_L2};
        std::vector<std::vector<cv::DMatch>> matches;
        std::vector<cv::Point2f> source, target, supported;
        std::vector<double> errors;
        cv::Mat inliers;
      };
      static thread_local Workspace workspace;
      auto& matches = workspace.matches;
      auto& source = workspace.source;
      auto& target = workspace.target;
      auto& supported = workspace.supported;
      auto& errors = workspace.errors;
      matches.clear(); source.clear(); target.clear();
      supported.clear(); errors.clear();
      workspace.matcher.knnMatch(current.descriptors, reference->descriptors, matches, 2);
      for (const auto& pair : matches) {
        if (pair.size() != 2 || pair[0].distance >= 0.7f * pair[1].distance) continue;
        source.push_back(current.keypoints[pair[0].queryIdx].pt);
        target.push_back(reference->keypoints[pair[0].trainIdx].pt);
      }
      if (source.size() < 30) return;
      cv::Mat& inliers = workspace.inliers;
      const cv::Mat affine = cv::estimateAffinePartial2D(
          source, target, inliers, cv::RANSAC, 3, 2000, 0.995, 10);
      const int count = cv::countNonZero(inliers);
      if (affine.empty() || count < 30 || count < source.size() * 0.25 ||
          count <= 0) return;
      const double scale = std::hypot(affine.at<double>(0, 0), affine.at<double>(1, 0));
      const double angle = std::atan2(affine.at<double>(1, 0),
                                    affine.at<double>(0, 0)) * 180 / pi;
      if (!std::isfinite(scale) || scale < 0.9 || scale > 1.1 || std::abs(angle) > 5) return;
      const cv::Matx33d transform(
          affine.at<double>(0, 0), affine.at<double>(0, 1), affine.at<double>(0, 2),
          affine.at<double>(1, 0), affine.at<double>(1, 1), affine.at<double>(1, 2),
          0, 0, 1);
      for (size_t index = 0; index < source.size(); ++index) {
        if (!inliers.at<uint8_t>(static_cast<int>(index))) continue;
        const cv::Vec3d point = transform * cv::Vec3d(source[index].x, source[index].y, 1);
        supported.push_back(source[index]);
        errors.push_back(std::hypot(point[0] - target[index].x, point[1] - target[index].y));
      }
      const cv::Rect support = cv::boundingRect(supported);
      if (support.width < photo.image.cols * 0.15 ||
          support.height < photo.image.rows * 0.15) return;
      std::sort(errors.begin(), errors.end());
      if (errors[errors.size() / 2] > 2) return;
      const cv::Matx33d candidate = reference->to_world * transform;
      const cv::Vec3d candidate_center = candidate * center_pixel;
      const double correction = std::hypot(candidate_center[0] - predicted_center[0],
                                           candidate_center[1] - predicted_center[1]);
      const cv::Vec3d matched_center = transform * center_pixel;
      const double local_distance = std::hypot(
          matched_center[0] - (reference->width - 1) / 2.0,
          matched_center[1] - (reference->height - 1) / 2.0);
      if (!std::isfinite(correction) ||
          local_distance > std::hypot(reference->width, reference->height) * 1.25)
        return;
      Candidate& match = candidates[candidate_index];
      match.inliers = count;
      match.to_world = candidate;
      match.evidence = Json::object({{"status", "registered"}, {"reference", reference->name},
          {"inliers", count}, {"median_error_px", errors[errors.size() / 2]},
          {"center_correction_map_px", correction}, {"relative_scale", scale}});
    });
    for (const auto& candidate : candidates) {
      if (candidate.inliers <= best_inliers) continue;
      best_inliers = candidate.inliers;
      selected = candidate.to_world;
      result = candidate.evidence;
    }
    photo.to_world = selected;
    photo.update_bounds(photo.image.cols, photo.image.rows);
    current.to_world = photo.to_world;
    if (references.size() == 8) references.erase(references.begin());
    references.push_back(std::move(current));
    return result;
  }

  /** @brief Publish the current resume checkpoint through the atomic JSON writer. */
  void save() { write_json(root / "state.json", state); }

  /** @brief Recheck source identity before modifying map state.
   * @param source Current source path.
   * @param job Discovery record containing the expected identity.
   * @return True on a match; false when a changing input can be deferred.
   * @details Reject an identity change while resuming a pending photo: some
   * tiles may already incorporate the original input. */
  bool unchanged(const fs::path& source, const Json& job) {
    if (identity(source, true) == job.at("identity")) return true;
    check(state.at("pending").is_null(),
          "pending source changed; retain original photos to resume this map");
    std::cout << "PANGEA: deferred changing photo " << source.filename()
              << '\n';
    return false;
  }

  /** Validate one discovered source and checkpoint its blend as an idempotent
   * job. */
  void process(const Json& job, PreparedPhoto* prepared = nullptr,
               std::exception_ptr prepared_error = nullptr) {
    const fs::path source = input / job.at("name").get<std::string>();
    check(source.parent_path() == input, "invalid source path");
    if (!unchanged(source, job)) return;
    Photo photo;
    try {
      try {
        if (prepared) photo = std::move(prepared->photo);
        else if (prepared_error) std::rethrow_exception(prepared_error);
        else photo = read_photo(source, resources, state.value("camera", Json()),
                                state.value("standard_profile", false));
      } catch (const InvalidPhoto& error) {
        if (!allow_prompt || profile_asked ||
            std::string(error.what()) !=
                "missing MIMIC geometry; real JPEGs require --camera-profile")
          throw;
        profile_asked = true;
        std::cerr << "PANGEA: " << error.what() << '\n'
                  << "Standard profile assumes 60-degree horizontal FOV, "
                     "centered square pixels and nadir mounting; FlyEye "
                     "images assume north-up, zero roll/pitch. "
                     "Map geometry is approximate, not calibrated.\n"
                  << "use standard profile Y/n: " << std::flush;
        std::string answer;
        if (!std::getline(std::cin, answer) ||
            !(answer.empty() || answer == "Y" || answer == "y"))
          throw;
        state.set("standard_profile", true);
        state.set("assumed_geometry", Json::object({
            {"horizontal_fov_deg", 60}, {"mount", "nadir"},
            {"flyeye_roll_deg", 0}, {"flyeye_pitch_deg", 0},
            {"flyeye_yaw_deg", 0}, {"accuracy", "uncalibrated"}}));
        save();
        photo = read_photo(source, resources, Json(), true);
      }
    } catch (const std::exception& error) {
      if (!unchanged(source, job)) return;
      if (dynamic_cast<const InvalidPhoto*>(&error) == nullptr) throw;
      check(state.at("pending").is_null(),
            "pending photo no longer decodes; checkpoint retained");
        write_json(
          root / "done" / (job.at("identity").get<std::string>() + ".json"),
          Json::object({{"status", "rejected"}, {"source", job}, {"reason", error.what()}}));
      state.set("rejected", state.at("rejected").get<unsigned>() + 1);
      save();
      std::cerr << "PANGEA: skipped " << source.filename() << ": "
                << error.what() << '\n';
      return;
    }
    if (!unchanged(source, job)) return;
    Json alignment;
    if (align_images) {
      if (!state.at("pending").is_null() &&
          state.at("pending").contains("registered_to_world")) {
        const Json saved = state.at("pending").at("registered_to_world");
        const cv::Matx33d position_prior = photo.to_world;
        check(saved.is_array() && saved.size() == 9, "invalid saved alignment");
        for (int index = 0; index < 9; ++index) {
          photo.to_world.val[index] = saved[index].get<double>();
          check(std::isfinite(photo.to_world.val[index]), "nonfinite saved alignment");
        }
        photo.update_bounds(photo.image.cols, photo.image.rows);
        alignment = state.at("pending").at("alignment");
        if (photo.image.cols > 80 && photo.image.rows > 120) {
          if (references.size() == 8) references.erase(references.begin());
          auto reference = features(photo, job.at("name").get<std::string>());
          reference.position_prior = position_prior;
          references.push_back(std::move(reference));
        }
      } else alignment = align(photo, job.at("name").get<std::string>(),
                prepared ? prepared->features.get() : nullptr);
      if (prepared) {
        prepared->photo = photo;
        prepared->tiles.clear();
        prepare_projection(*prepared, resources);
      }
    }
    Bounds combined;
    try {
      combined = Bounds::from(state.at("bounds"), max_map_side)
             .include(photo.bounds, max_map_side);
    } catch (const std::exception&) {
      throw std::runtime_error(
          "map extent limit reached; resume with a larger --max-map-side "
          "for standalone stitching, or use a new output for another area");
    }
    const uint64_t sequence = state.at("sequence").get<uint64_t>() + 1;
    check(sequence <= max_photos, "10000-photo limit reached; start a new map");
    disk_headroom(root, 64 * mib);
    // Persist intent before touching tiles so a restart can replay partial tile
    // updates.
    if (state.at("pending").is_null()) {
      state.set("pending", job);
      if (align_images) {
        Json transform = Json::array();
        for (double value : photo.to_world.val) transform.push_back(value);
        Json pending = state.at("pending");
        pending.set("registered_to_world", transform);
        pending.set("alignment", alignment);
        state.set("pending", pending);
      }
      save();
    }
    if (prepared) blend_prepared(*prepared, root, sequence, resources);
    else blend(photo, root, sequence, resources);
     Json completed = Json::object({{"status", "assembled"},
         {"source", job},
         {"sequence", sequence},
       {"corners_lon_lat", photo.geographic_corners()}});
     if (align_images) completed.set("alignment", alignment);
     write_json(root / "done" / (job.at("identity").get<std::string>() + ".json"), completed);
    state.set("bounds", combined.json());
    state.set("sequence", sequence);
    state.set("pending", nullptr);
    if (align_images) {
      Json saved = Json::array();
      for (const auto& reference : references) {
        Json transform = Json::array();
        for (double value : reference.to_world.val) transform.push_back(value);
        saved.push_back(Json::object({{"name", reference.name},
            {"identity", identity(input / reference.name)}, {"to_world", transform}}));
      }
      state.set("alignment_references", saved);
    }
    save();
    std::cout << "PANGEA: assembled " << source.filename() << " (#" << sequence
              << ")\n";
  }

 public:
  /** Acquire per-user/output locks and load or initialize the resumable state
   * file. */
  Worker(fs::path source, fs::path output, Resources& budget, uint64_t since,
        const Json& camera, bool interactive, int side_limit, bool image_alignment)
      : resources(budget), allow_prompt(interactive), max_map_side(side_limit),
        align_images(image_alignment) {
    input = fs::canonical(source);
    check(fs::is_directory(input), "input must be a photo directory");
    fs::create_directories(output);
    root = fs::canonical(output);
    check(root != input, "output must differ from input");
    // Serialize this user's workers before taking the output-specific
    // checkpoint lock.
    const std::string lock_path =
        "/tmp/usher-pangea-" + std::to_string(getuid()) + ".lock";
    process_lock.reset(open(lock_path.c_str(),
              O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600));
    check(process_lock.get() >= 0 && flock(process_lock.get(), LOCK_EX | LOCK_NB) == 0,
          "another PANGEA worker is active for this user");
    output_lock.reset(open((root / ".lock").c_str(),
               O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600));
    check(output_lock.get() >= 0 && flock(output_lock.get(), LOCK_EX | LOCK_NB) == 0,
          "output already has a PANGEA worker");
    if (fs::exists(root / "state.json")) {
      state = read_json(root / "state.json");
      check(state.value("schema", "") == "PANGEA_STATE_V1" &&
                state.at("input") == input.string(),
            "checkpoint belongs to another input or format");
      check(state.value("camera", Json()) == camera,
            "camera profile differs from checkpoint; use original profile or a "
            "new output");
      Bounds::from(state.at("bounds"), max_map_side);
      check(state.value("align_images", false) == align_images,
        "alignment mode differs from checkpoint; use a new output directory");
      check(state.at("sequence").get<uint64_t>() <= max_photos &&
                state.at("rejected").get<unsigned>() <= max_photos,
            "invalid checkpoint counts");
    } else {
      for (const auto& entry : fs::directory_iterator(root))
        check(entry.path().filename() == ".lock",
              "output must be empty or a PANGEA checkpoint");
      state = Json::object({{"schema", "PANGEA_STATE_V1"},
               {"input", input.string()},
               {"since", since},
               {"camera", camera},
               {"sequence", 0},
               {"rejected", 0},
               {"exported", 0},
               {"bounds", Bounds{}.json()},
               {"pending", nullptr}});
            if (align_images) state.set("align_images", true);
      save();
    }
    fs::create_directory(root / "tiles");
    fs::create_directory(root / "done");
    if (align_images && state.contains("alignment_references")) {
      const Json saved = state.at("alignment_references");
      check(saved.is_array() && saved.size() <= 8, "invalid alignment references");
      for (size_t saved_index = 0; saved_index < saved.size(); ++saved_index) {
        const Json record = saved[saved_index];
        const std::string name = record.at("name").get<std::string>();
        const fs::path source = input / name;
        check(source.parent_path() == input &&
                  identity(source) == record.at("identity"),
              "alignment reference changed; retain original photos");
        Photo photo = read_photo(source, resources, camera,
                                 state.value("standard_profile", false));
        const cv::Matx33d position_prior = photo.to_world;
        const Json transform = record.at("to_world");
        check(transform.is_array() && transform.size() == 9, "invalid reference transform");
        for (int index = 0; index < 9; ++index) {
          photo.to_world.val[index] = transform[index].get<double>();
          check(std::isfinite(photo.to_world.val[index]), "nonfinite reference transform");
        }
        photo.update_bounds(photo.image.cols, photo.image.rows);
        auto reference = features(photo, name);
        reference.position_prior = position_prior;
        references.push_back(std::move(reference));
      }
    }
  }
  /** Process stable jobs; in watch mode, defer changing files and export
   * periodically. */
  void run(bool watch) {
    double exported_at = seconds(CLOCK_MONOTONIC);
    const auto export_due = [&](bool force) {
      if (state.at("exported") != state.at("sequence") &&
          (force || (watch && seconds(CLOCK_MONOTONIC) - exported_at >= 10))) {
        export_map(root, state, resources, max_map_side);
        state.set("exported", state.at("sequence"));
        save();
        exported_at = seconds(CLOCK_MONOTONIC);
      }
    };
    std::map<std::string, std::pair<std::string, double>> observed;
    while (!stopping) {
      resources.checkpoint();
      if (!state.at("pending").is_null()) {
        process(state.at("pending"));
        export_due(false);
        continue;
      }
      Directory directory(opendir(input.c_str()));
      check(bool(directory), "input directory unavailable");
      std::vector<Json> jobs;
      jobs.reserve(64);
      std::map<std::string, std::pair<std::string, double>> next_observed;
      size_t entries = 0;
      while (dirent* entry = readdir(directory.get())) {
        // Check cancellation/headroom during very large directory scans without
        // a syscall per entry.
        if (++entries % 64 == 0) resources.checkpoint();
        check(entries <= 100000, "input directory entry limit reached");
        const std::string name = entry->d_name;
        if (fs::path(name).extension() != ".jpg" ||
            name.find(".lens.") != std::string::npos)
          continue;
        if (!state.value("camera", Json()).is_null() &&
            name.rfind(
                state.at("camera").at("filename_prefix").get<std::string>(),
                0) != 0)
          continue;
        struct stat status{};
        if (lstat((input / name).c_str(), &status) != 0 ||
            !S_ISREG(status.st_mode) ||
            status.st_mtim.tv_sec <
            static_cast<time_t>(state.at("since").get<uint64_t>()))
          continue;
        const std::string key = identity(input / name, true);
        if (key.empty()) continue;
        if (fs::exists(root / "done" / (key + ".json"))) continue;
        if (watch) {
          const auto previous = observed.find(name);
          const double now = seconds(CLOCK_MONOTONIC);
          const double stable_since =
              previous != observed.end() && previous->second.first == key
                  ? previous->second.second
                  : now;
          next_observed.emplace(name, std::make_pair(key, stable_since));
          // Require two stable seconds so a producer cannot expose a
          // half-written JPEG.
          if (now - stable_since < 2) {
            if (next_observed.size() == 64) break;
            continue;
          }
        }
        jobs.push_back(Json::object({{"name", name}, {"identity", key}}));
        if (jobs.size() == 64 || next_observed.size() == 64) break;
      }
      observed = std::move(next_observed);
      if (!jobs.empty()) {
        std::sort(jobs.begin(), jobs.end(),
                  [](const Json& first, const Json& second) {
                    return first.at("name") < second.at("name");
                  });
        if (!resources.desktop()) {
          for (const Json& job : jobs) {
            check(state.at("sequence").get<unsigned>() +
                          state.at("rejected").get<unsigned>() < max_photos,
                  "photo record limit reached; start a new output");
            process(job);
                export_due(false);
          }
        } else {
        for (size_t batch_start = 0; batch_start < jobs.size();) {
          const size_t batch_size =
              std::min(resources.photo_batch_capacity(align_images ? 256 : 128),
                       jobs.size() - batch_start);
          bool standard_profile = state.value("standard_profile", false);
          std::vector<std::unique_ptr<PreparedPhoto>> prepared(batch_size);
          std::vector<std::exception_ptr> errors(batch_size);
          const Json camera = state.value("camera", Json());
          const auto prepare_one = [&](size_t index) {
              prepared[index].reset();
              errors[index] = nullptr;
              try {
                auto decoded = std::make_unique<PreparedPhoto>();
                decoded->photo = read_photo(
                    input / jobs[batch_start + index].at("name").get<std::string>(),
                  resources, camera, standard_profile, true);
                if (!align_images) prepare_projection(*decoded, resources);
                else if (decoded->photo.image.cols > 80 && decoded->photo.image.rows > 120)
                  decoded->features = std::make_unique<ImageFeatures>(features(
                      decoded->photo, jobs[batch_start + index].at("name").get<std::string>()));
                prepared[index] = std::move(decoded);
              } catch (...) {
                errors[index] = std::current_exception();
              }
          };
          resources.parallel_tiles(batch_size, prepare_one);
          for (size_t index = 0; index < batch_size; ++index) {
          const Json& job = jobs[batch_start + index];
          check(state.at("sequence").get<unsigned>() +
                        state.at("rejected").get<unsigned>() <
                    max_photos,
                "photo record limit reached; start a new output");
          if (standard_profile != state.value("standard_profile", false)) {
            standard_profile = state.value("standard_profile", false);
            resources.parallel_tiles(batch_size - index, [&](size_t remaining) {
              prepare_one(index + remaining);
            });
          }
          process(job, prepared[index].get(), errors[index]);
          }
          batch_start += batch_size;
        }
        }
        export_due(false);
        continue;
      }
      export_due(!watch);
      if (!watch) break;
      poll(nullptr, 0, 1000);
    }
  }
};
}  // namespace pangea_detail

using namespace pangea_detail;

/** Parse worker options, install signal/resource policy, and run one map
 * session.
 * @param argc Argument count.
 * @param argv Input/output paths and optional watch, camera, and resource
 * settings.
 * @return Zero on successful completion, otherwise one with a diagnostic. */
int pangeaSurveyMain(int argc, char** argv);

/** @brief Dispatch PANGEA survey commands or run incremental photo mapping.
 * @param argc Command-line argument count.
 * @param argv Command-line arguments, including the executable name.
 * @return Exit status for the selected operation.
 * @details Establish restrictive output permissions and the OpenCV backend
 * before dispatching work; the direct mapper installs cooperative stop handlers. */
int main(int argc, char** argv) {
  umask(0077);
  try {
    if (argc == 2 && (std::string(argv[1]) == "--version" || std::string(argv[1]) == "-v")) {
      std::cout << PANGEA_BUILD_VERSION << '\n';
      return 0;
    }
    const usher_cv_parallel::ScopedBackend opencv_parallel_backend;
    if (argc > 1) {
      const std::string command = argv[1];
      if (command == "--run" || command == "--check" || command == "--diagnostic" ||
        command == "--update-great" || command == "--follow-great" || command == "--finalize-great" ||
          command == "--stop" || command == "--self-test")
        return pangeaSurveyMain(argc, argv);
    }
    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    umask(0077);
    std::cout.setf(std::ios::unitbuf);
    if (argc == 2 && std::string(argv[1]) == "--check-resources") {
      Resources resources(192, 20);
      std::cout << resources.report().dump() << '\n';
      return 0;
    }
    fs::path input, output, profile_path, pause_file;
    uint64_t reserve = 192, since = 0;
    int cpu_percent = 20, max_map_side = 8192;
    bool watch = false, desktop = false, cpu_option = false, align_images = false;
    for (int index = 1; index < argc; ++index) {
      const std::string option = argv[index];
      if (option == "--help" || option == "-h") {
        std::cout
            << "PANGEA: Photo-Assisted Neighboring Geographic Edge Assembly\n"
               "Usage: pangea --input PHOTOS --output MAP [--watch] "
               "[--reserve-mb 192] [--cpu-percent 20]\n"
               "       Incremental GREAT XYZ: --update-great INI, --follow-great INI, --finalize-great INI.\n"
               "       Add --camera-profile FILE for real USHER JPEGs; no "
               "synthetic images or effects.\n"
               "       --pause-file FILE defers work while FILE exists; remove "
               "it to resume.\n"
               "       --max-map-side N permits larger standalone maps; "
               "default 8192, multiples of 256 up to 65280.\n"
               "       --desktop enables parallel, unthrottled replay on "
               "hosts with at least 2 GiB RAM; never for MORA.\n"
               "       --align-images registers overlapping photos in desktop "
               "mode; requires a new output directory.\n"
               "       pangea --check-resources\n"
               "       pangea --check SURVEY.ini\n"
               "       pangea --run SURVEY.ini [--mode simple|great]\n"
               "       pangea --diagnostic SURVEY.ini [--mode MODE]\n"
               "       pangea --stop\n"
#if SENSORAIA_BUILD_TESTING
               "       pangea --self-test\n"
#endif
               "       pangea --version\n"
               "Default: clean MIMIC input. Real mode requires explicit "
               "camera geometry. Standalone replay offers an approximate "
               "standard profile when MIMIC geometry is missing. "
               "No realtime deadline.\n";
        return 0;
      }
      if (option == "--watch") {
        watch = true;
        continue;
      }
      if (option == "--desktop") {
        desktop = true;
        continue;
      }
      if (option == "--align-images") {
        align_images = true;
        continue;
      }
      check(index + 1 < argc, "option needs a value");
      const std::string value = argv[++index];
      if (option == "--input")
        input = value;
      else if (option == "--output")
        output = value;
      else if (option == "--camera-profile")
        profile_path = value;
      else if (option == "--pause-file") {
        check(!value.empty(), "pause file must not be empty");
        pause_file = value;
      } else {
        check(!value.empty() &&
                  value.find_first_not_of("0123456789") == std::string::npos,
              "numeric option must be unsigned");
        const uint64_t parsed = std::stoull(value);
        if (option == "--reserve-mb")
          reserve = parsed;
        else if (option == "--max-map-side") {
          check(parsed >= 8192 && parsed <= 65280 && parsed % tile_size == 0,
                "max-map-side must be a multiple of 256 in 8192..65280");
          max_map_side = static_cast<int>(parsed);
        }
        else if (option == "--cpu-percent") {
          check(parsed <= 20, "CPU duty exceeds limit");
          cpu_percent = parsed;
          cpu_option = true;
        } else if (option == "--since")
          since = parsed;
        else if (option == "--parent-pid") {
          check(parsed > 1 && parsed <= INT32_MAX, "invalid parent PID");
          parent_pid = parsed;
        } else
          throw std::runtime_error("unknown option");
      }
    }
    check(!input.empty() && !output.empty(),
          "--input PHOTOS and --output MAP are required");
        check(!desktop || (!watch && !parent_pid && !cpu_option),
          "--desktop is standalone replay only; incompatible with --watch, "
          "--parent-pid and --cpu-percent");
        check(!align_images || desktop, "--align-images requires --desktop");
        Resources resources(reserve, cpu_percent, pause_file, desktop);
    resources.checkpoint();
    const Json camera = camera_profile(profile_path);
    while (watch && !fs::exists(input)) {
      resources.checkpoint();
      poll(nullptr, 0, 1000);
    }
    Worker worker(input, output, resources, since, camera,
          !watch && !parent_pid && camera.is_null(), max_map_side, align_images);
    worker.run(watch);
    std::cout << resources.report().dump() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PANGEA: " << error.what() << '\n';
    return 1;
  }
}