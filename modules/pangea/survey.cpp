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
 * @file survey.cpp
 * @brief Command-line entry point for completed-session PANGEA mapping.
 * @details Survey processing consumes saved photos and validated session
 * metadata; it is not part of the camera trigger path. Separate per-user locks
 * exclude both another survey and the live PANGEA worker so two independent
 * consumers cannot compete for memory or checkpoint files.
 */
#include "application_runtime.h"
#include "version.h"
#include "map_config.h"
#include "ortho_map.h"
#include "map_exif.h"
#include "map_writer.h"
#include "map_progress.h"
#include "jpeg_encoder.h"
#include "map_resources.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <functional>
#include <cstring>
#include <sys/file.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

using Clock = std::chrono::steady_clock;
static std::atomic<sig_atomic_t> stopping{0};
static_assert(decltype(stopping)::is_always_lock_free,
              "Signal cancellation requires lock-free atomic access");
static std::unique_ptr<pangea_survey::MapResources> resources;
static fs::path activePauseFile;
static int liveCpuPercent = 0;
static double liveCpuStarted = 0;
static Clock::time_point liveStarted;
/** @brief Read cumulative CPU time across the mapping process's threads.
 * @return CPU seconds, independent of elapsed wall time.
 * @throws std::runtime_error If the process CPU clock cannot be read. */
static double processCpuSeconds() {
    struct timespec value{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0)
        throw std::runtime_error("Cannot measure mapping CPU time");
    return value.tv_sec + value.tv_nsec / 1e9;
}
/** @brief Check cancellation at a cooperative mapping boundary.
 * @return True when a stop signal has been recorded.
 * @details Wait out operator pauses and throttle against cumulative CPU time.
 * Poll in short sleeps so cancellation remains responsive while paused. */
static bool isStopping() {
    while (!stopping && !activePauseFile.empty() && fs::exists(activePauseFile))
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (liveCpuPercent) {
        const auto target = (processCpuSeconds() - liveCpuStarted) * 100 / liveCpuPercent;
        while (!stopping && std::chrono::duration<double>(Clock::now() - liveStarted).count() < target) {
            if (resources) resources->checkpoint();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    // Check headroom at the same cooperative boundaries used for cancellation.
    if (resources && !stopping) resources->checkpoint();
    return stopping != 0;
}
/** @brief Record a stop request without performing cleanup in signal context.
 * @param sig Signal number; all registered stop signals have the same effect. */
static void signalHandler(int sig) {
    (void)sig;
    stopping = 1;
}
/** @brief Resolve the running executable independently of the launch directory.
 * @return Canonical Linux executable path.
 * @throws std::filesystem::filesystem_error If procfs resolution fails. */
static fs::path executablePath() {
    return fs::canonical("/proc/self/exe");
}
/** @brief Locate the default configuration alongside the executable.
 * @return Candidate path; does not check whether the INI exists. */
static fs::path defaultIni() { return executablePath().parent_path() / "pangea.ini"; }
/** @brief Convert a monotonic interval without truncating fractional seconds.
 * @param duration Clock interval.
 * @return Interval in seconds. */
static double seconds(Clock::duration duration) { return std::chrono::duration<double>(duration).count(); }
/** @brief Create a readable, millisecond-resolution session label.
 * @return Local wall-clock label, not a uniqueness guarantee. */
static std::string sessionName() {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::ostringstream text;
    text << std::setfill('0') << std::setw(4) << (tm.tm_year + 1900) << std::setw(2) << (tm.tm_mon + 1)
         << std::setw(2) << tm.tm_mday << '_' << std::setw(2) << tm.tm_hour
         << std::setw(2) << tm.tm_min << std::setw(2) << tm.tm_sec << '_'
         << std::setw(3) << ms << '_' << getpid();
    return text.str();
}
#include "map_pipeline.h"
#include "xyz_export.h"
#include "great_session.h"
#if SENSORAIA_BUILD_TESTING
static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(std::string("TEST FAILED: ") + message);
}
static void testOpenCvParallelBackend() {
    const int previousThreads = cv::getNumThreads();
    struct RestoreThreads {
        int count;
        ~RestoreThreads() { cv::setNumThreads(count); }
    } restore{previousThreads};

    constexpr int totalItems = 513;
    for (const int team : {4, 3}) {
        cv::setNumThreads(team);
        require(cv::getNumThreads() == team, "OpenCV parallel backend did not apply thread count");
        std::vector<std::atomic<int>> hits(totalItems);
        for (auto& hit : hits) hit.store(0, std::memory_order_relaxed);
        std::atomic<bool> validThreadIds{true};

        cv::parallel_for_(cv::Range(0, totalItems), [&](const cv::Range& range) {
            const int tid = cv::getThreadNum();
            if (tid < 0 || tid >= team) validThreadIds.store(false, std::memory_order_relaxed);
            int nestedCount = 0;
            cv::parallel_for_(cv::Range(0, 4), [&](const cv::Range& inner) {
                nestedCount += inner.size();
            });
            if (nestedCount != 4) validThreadIds.store(false, std::memory_order_relaxed);
            for (int index = range.start; index < range.end; ++index)
                hits[static_cast<std::size_t>(index)].fetch_add(1, std::memory_order_relaxed);
        });

        require(validThreadIds.load(std::memory_order_relaxed), "OpenCV parallel backend reported invalid thread index");
        for (const auto& hit : hits)
            require(hit.load(std::memory_order_relaxed) == 1, "OpenCV parallel backend missed or duplicated stripe work");
        if (team == 4) {
            cv::setNumThreads(1);
            require(cv::getNumThreads() == 1, "OpenCV parallel backend did not shrink to single thread");
        }
    }

    bool caught = false;
    try {
        cv::parallel_for_(cv::Range(0, 64), [&](const cv::Range& range) {
            if (range.start <= 17 && 17 < range.end)
                throw std::runtime_error("synthetic parallel_for failure");
        });
    } catch (const std::exception& error) {
        caught = std::string(error.what()).find("synthetic parallel_for failure") != std::string::npos;
    }
    require(caught, "OpenCV parallel backend lost body exception");

    cv::setNumThreads(1);
    require(cv::getNumThreads() == 1, "OpenCV parallel backend did not shrink to single thread");
    std::atomic<bool> serialOnCaller{true};
    cv::parallel_for_(cv::Range(0, 32), [&](const cv::Range&) {
        if (cv::getThreadNum() != 0) serialOnCaller.store(false, std::memory_order_relaxed);
    });
    require(serialOnCaller.load(std::memory_order_relaxed), "Single-thread OpenCV region used worker thread");
}
#include "map_exif_tests.h"
#include "xyz_tests.h"
#include "pipeline_tests.h"
#endif

/** @brief Validate a source session and print its output layout offline.
 * @param config Requested map and source configuration.
 * @details For XYZ output, validate the internal render grid separately
 * from the published tile plan; they need not share pixel dimensions.
 * @throws std::runtime_error If source admission or layout validation fails. */
static void checkInputs(const pangea_survey::MapConfig& config) {
    const auto working = config.outputFormat == "xyz" ? pangea_survey::XyzPlan(config).workingConfig(config) : config;
    pangea_survey::Orthorectifier checked(working, config.sourceCsv);
    std::cout << "Input valid: " << config.sourceCsv.u8string() << "\nMode: " << config.mode
              << "; " << checked.columns() << " x " << checked.rows()
              << (config.outputFormat == "xyz" ? " internal working tiles; " : " map tiles; ")
              << working.mapPixelSize << " m/working pixel; no MSFS connection.\n";
    if (config.outputFormat == "xyz") {
        const pangea_survey::XyzPlan plan(config);
        std::cout << "XYZ output: " << plan.size() << " complete 256-pixel tiles at zoom "
                  << plan.minZoom << ".." << plan.base.zoom << "; "
                  << pangea_survey::XyzTiles::groundPixel(plan.base.zoom, (plan.north + plan.south) / 2)
                  << " m/ground pixel at area centre.\n";
    }
}

int pangeaSurveyMain(int argc, char** argv) {
    try {
        if (argc == 2 && (std::strcmp(argv[1], "--version") == 0 || std::strcmp(argv[1], "-v") == 0)) {
            std::cout << PANGEA_BUILD_VERSION << '\n';
            return 0;
        }
        if (argc == 2 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
            std::cout << "Pangea: completed-survey processing\n"
                         "  pangea --check [pangea.ini]\n"
                         "  pangea --run [pangea.ini] [--mode simple|great]\n"
                         "  pangea --diagnostic [pangea.ini] [--mode simple|great]\n"
                         "  pangea --update-great pangea.ini\n"
                         "  pangea --follow-great pangea.ini\n"
                         "  pangea --finalize-great pangea.ini\n"
                         "  pangea --stop\n"
#if SENSORAIA_BUILD_TESTING
                         "  pangea --self-test\n"
#endif
                         "  pangea --version\n";
            return 0;
        }
        if (argc == 2 && std::strcmp(argv[1], "--self-test") == 0) {
#if SENSORAIA_BUILD_TESTING
            testOpenCvParallelBackend();
            testSurveyTiming();
            pangea_survey::MapProgress::selfTest();
            pangea_survey::Orthorectifier::selfTest();
            testXyzGrid();
            testMapExif();
            testSplitPipeline();
            return 0;
#else
            throw std::runtime_error("Self-tests require the private development checkout and BUILD_TESTING=ON");
#endif
        }
        const std::string lockPath = "/tmp/pangea-survey-" + std::to_string(getuid()) + ".lock";
        if (argc == 2 && std::strcmp(argv[1], "--stop") == 0) {
            std::ifstream lockFile(lockPath);
            pid_t pid = 0;
            if (lockFile >> pid && pid > 1) {
                const auto process = fs::path("/proc") / std::to_string(pid);
                std::ifstream arguments(process / "cmdline", std::ios::binary);
                std::string program, command;
                std::getline(arguments, program, '\0'); std::getline(arguments, command, '\0');
                std::error_code error;
                const auto executable = fs::canonical(process / "exe", error);
                if (!error && executable == executablePath() &&
                    (command == "--run" || command == "--diagnostic" || command == "--update-great" ||
                     command == "--follow-great" || command == "--finalize-great") &&
                    kill(pid, SIGTERM) == 0) {
                    std::cout << "Stopped Pangea survey PID " << pid << ".\n";
                    return 0;
                }
            }
            std::cerr << "No Pangea survey is running.\n";
            return 1;
        }
        if (argc == 3 && (std::strcmp(argv[1], "--update-great") == 0 ||
                          std::strcmp(argv[1], "--follow-great") == 0 ||
                          std::strcmp(argv[1], "--finalize-great") == 0)) {
            const auto ini = fs::path(argv[2]);
            const auto config = pangea_survey::loadMapConfig(ini);
            pangea_survey::GreatSessionLease lease;
            resources = std::make_unique<pangea_survey::MapResources>(config);
            signal(SIGINT, signalHandler);
            signal(SIGTERM, signalHandler);
            activePauseFile = config.livePauseFile;
            liveCpuPercent = config.liveCpuPercent;
            liveCpuStarted = processCpuSeconds();
            liveStarted = Clock::now();
            const bool follow = std::strcmp(argv[1], "--follow-great") == 0;
            const bool finalize = std::strcmp(argv[1], "--finalize-great") == 0;
            auto updated = Clock::now() - std::chrono::seconds(config.liveUpdateSeconds);
            std::string previous;
            std::string prepared;
            bool pressureReported = false;
            do {
                try {
                if (isStopping()) break;
                const bool final = finalize || (!config.liveFinalFile.empty() && fs::exists(config.liveFinalFile));
                auto snapshot = pangea_survey::greatInputText(config);
                const auto end = snapshot.rfind('\n');
                if (end != std::string::npos) snapshot.resize(end + 1);
                const auto added = std::count(snapshot.begin(), snapshot.end(), '\n') -
                                   std::count(previous.begin(), previous.end(), '\n');
                if (follow && !final && snapshot != prepared &&
                    pangea_survey::updateGreatSession(config, ini, false, false)) prepared = snapshot;
                if (!follow || final || (snapshot != previous &&
                    (added >= config.liveBatchPhotos || Clock::now() - updated >= std::chrono::seconds(config.liveUpdateSeconds)))) {
                    if (pangea_survey::updateGreatSession(config, ini, final)) {
                        previous = snapshot;
                        updated = Clock::now();
                    } else if (final) throw std::runtime_error("Cannot finalize an empty photo session");
                }
                if (!follow || final) return 0;
                pressureReported = false;
                } catch (const pangea_survey::MapResourcePressure& error) {
                    if (!follow) throw;
                    if (!pressureReported) std::cout << error.what() << "; waiting for headroom.\n";
                    pressureReported = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            } while (!stopping);
            std::cout << "GREAT session stopped; previous revision and working checkpoints retained.\n";
            return 0;
        }
        const bool run = argc >= 2 && std::strcmp(argv[1], "--run") == 0;
        const bool check = argc >= 2 && std::strcmp(argv[1], "--check") == 0;
        const bool diagnostic = argc >= 2 && std::strcmp(argv[1], "--diagnostic") == 0;
        if ((run || check || diagnostic) && (argc == 2 || argc == 3 || argc == 5)) {
            if (argc == 5 && std::strcmp(argv[3], "--mode") != 0) throw std::runtime_error("Expected --mode");
            const auto iniPath = argc >= 3 ? fs::path(argv[2]) : defaultIni();
            const std::string modeOverride = argc == 5 ? argv[4] : "";
            auto config = pangea_survey::loadMapConfig(iniPath, modeOverride);
            if (!config.photoDirectory.empty())
                throw std::runtime_error("AI Camera directories use --update-great, --follow-great or --finalize-great");
            if (!check) resources = std::make_unique<pangea_survey::MapResources>(config);
            checkInputs(config);
            if (check) return 0;
            int lockFd = open(lockPath.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
            if (lockFd < 0 || flock(lockFd, LOCK_EX | LOCK_NB) != 0) {
                if (lockFd >= 0) close(lockFd);
                throw std::runtime_error("Pangea survey already running");
            }
            const auto liveLockPath = "/tmp/usher-pangea-" + std::to_string(getuid()) + ".lock";
            // The live worker has its own lock; taking both prevents offline mapping from competing with capture.
            const int liveLock = open(liveLockPath.c_str(), O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
            if (liveLock < 0 || flock(liveLock, LOCK_EX | LOCK_NB) != 0)
                throw std::runtime_error("Another Pangea live or survey worker is active; mapping remains optional");
            if (ftruncate(lockFd, 0) == 0) {
                std::string pidStr = std::to_string(getpid()) + "\n";
                ssize_t written = write(lockFd, pidStr.data(), pidStr.size());
                (void)written;
                fsync(lockFd);
            }
            signal(SIGINT, signalHandler);
            signal(SIGTERM, signalHandler);
            if (config.outputFormat == "xyz" && !diagnostic && !config.lowMemory)
                pangea_survey::XyzPlan(config).checkOutput(config.output);
            auto workDirectory = config.outputFormat == "xyz" ?
                config.output.parent_path() / "pangea_exports" / ("xyz_" + sessionName()) : config.output;
            if (config.lowMemory) {
                if (diagnostic) throw std::runtime_error("Low-memory profile publishes strict XYZ; use --run");
                const auto output = fs::weakly_canonical(fs::absolute(config.output)).generic_string();
                const auto key = pangea_survey::dataDigest(output.data(), output.size());
                const auto jobName = "xyz_low_memory_" + key.substr(0, 16);
                workDirectory = config.output.parent_path() / "pangea_exports" / jobName;
                const auto ini = argc >= 3 ? fs::path(argv[2]) : defaultIni();
                const auto fingerprint =
                    pangea_survey::fileDigest(ini) + "\n" + pangea_survey::fileDigest(config.sourceCsv) + "\n" +
                    (config.captureManifest.empty() ? "none" : pangea_survey::fileDigest(config.captureManifest)) + "\n" +
                    fs::canonical(config.sourceCsv).generic_string() + "\n" + output + "\n" + config.mode + "\n";
                const auto identity = std::string("PANGEA_LOW_MEMORY_V1\n") + fingerprint;
                const auto record = workDirectory / "identity";
                // Reusing tiles is safe only when inputs, geographic bounds, and mode are identical.
                if (fs::exists(workDirectory)) {
                    std::ifstream input(record);
                    const std::string saved((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
                    if (!input || saved != identity)
                        throw std::runtime_error("Working job identity changed; choose a new output directory");
                } else {
                    pangea_survey::XyzPlan(config).checkOutput(config.output);
                    fs::create_directories(workDirectory);
                    pangea_survey::saveWorkingText(record, identity);
                }
                config.workingDirectory = workDirectory;
            }
            fs::create_directories(workDirectory);
            SessionLog log(workDirectory, "pangea.log");
            config.originalSourceCsv = config.sourceCsv;
            if (!config.lowMemory) config.sourceCsv = pangea_survey::Orthorectifier::normalizeSources(config.sourceCsv,
                workDirectory / ("rectified_" + sessionName()), isStopping, config.mosaicThreads,
                static_cast<unsigned>(config.mosaicPhotoCacheMiB));
            int result = config.outputFormat == "xyz" ? exportXyzTiles(config, config.sourceCsv, workDirectory, diagnostic) :
                                                       orthorectify(config, config.sourceCsv, diagnostic);
            close(lockFd);
            close(liveLock);
            unlink(lockPath.c_str());
            return result;
        }
        std::cout << "Pangea: completed-survey processing\n"
                 "  pangea --check [pangea.ini]\n"
                 "  pangea --run [pangea.ini] [--mode simple|great]\n"
                 "  pangea --diagnostic [pangea.ini] [--mode simple|great]\n"
                 "  pangea --update-great pangea.ini\n"
                 "  pangea --follow-great pangea.ini\n"
                 "  pangea --finalize-great pangea.ini\n"
                 "  pangea --stop\n"
#if SENSORAIA_BUILD_TESTING
                 "  pangea --self-test\n"
#endif
                 ;
        return argc == 1 ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
