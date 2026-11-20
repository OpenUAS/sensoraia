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
 * @file great_session.h
 * @brief Coordinate append-only, restartable GREAT XYZ survey sessions.
 * @details The session lease excludes both another offline mapper and the live
 * capture mapper. Each revision is fingerprinted and prepared before `latest`
 * moves, so interruption cannot replace the last usable revision.
 */
#ifndef PANGEA_GREAT_SESSION_H
#define PANGEA_GREAT_SESSION_H

#include <sys/stat.h>

namespace pangea_survey {
/**
 * @brief Hold the per-user GREAT and live-PANGEA locks for one session update.
 * @details Both locks are necessary because offline remapping and the live
 * watcher compete for the same camera-compute memory and output resources.
 */
class GreatSessionLease {
    std::array<int, 2> descriptors_{{-1, -1}};
public:
    /** @brief Acquire both nonblocking locks or fail without disturbing the owner. */
    GreatSessionLease() {
        const std::array<std::string, 2> paths{{
            "/tmp/pangea-survey-" + std::to_string(getuid()) + ".lock",
            "/tmp/usher-pangea-" + std::to_string(getuid()) + ".lock"}};
        for (std::size_t index = 0; index < paths.size(); ++index) {
            const int descriptor = open(paths[index].c_str(), O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
            if (descriptor < 0 || flock(descriptor, LOCK_EX | LOCK_NB) != 0) {
                if (descriptor >= 0) close(descriptor);
                for (const auto held : descriptors_) if (held >= 0) close(held);
                throw std::runtime_error("Another Pangea mapper is active; capture remains independent");
            }
            descriptors_[index] = descriptor;
        }
        const auto pid = std::to_string(getpid()) + "\n";
        if (ftruncate(descriptors_[0], 0) != 0 ||
            write(descriptors_[0], pid.data(), pid.size()) != static_cast<ssize_t>(pid.size()) ||
            fsync(descriptors_[0]) != 0) {
            for (const auto held : descriptors_) close(held);
            throw std::runtime_error("Cannot record GREAT session owner");
        }
    }
    /** @brief Release both locks without removing the lock paths.
     * @details Other callers must continue locking the same files. */
    ~GreatSessionLease() { for (const auto held : descriptors_) if (held >= 0) close(held); }
    /** @brief Noncopyable: both descriptors have a single lifetime owner. */
    GreatSessionLease(const GreatSessionLease&) = delete;
    /** @brief Nonassignable: an active lease cannot be replaced by copying. */
    GreatSessionLease& operator=(const GreatSessionLease&) = delete;
};

/**
 * @brief Read a bounded regular session file without following symlinks.
 * @param[in] path Session input, identity, or admission record.
 * @return Complete file contents, up to 4 MiB.
 * @throws std::runtime_error If the path is not a regular bounded file or reading fails.
 */
inline std::string sessionText(const fs::path& path) {
    const int descriptor = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
    if (descriptor < 0) throw std::runtime_error("Cannot open session input: " + path.string());
    struct stat status{};
    if (fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode) || status.st_size > 4 * 1024 * 1024) {
        close(descriptor);
        throw std::runtime_error("Session input must be a regular file of at most 4 MiB");
    }
    std::string text;
    std::array<char, 65536> buffer{};
    for (;;) {
        const auto count = read(descriptor, buffer.data(), buffer.size());
        if (count < 0 && errno == EINTR) continue;
        if (count < 0 || text.size() + static_cast<std::size_t>(count) > 4 * 1024 * 1024) {
            close(descriptor);
            throw std::runtime_error("Cannot read bounded session input");
        }
        if (!count) break;
        text.append(buffer.data(), static_cast<std::size_t>(count));
    }
    close(descriptor);
    return text;
}

/**
 * @brief Snapshot the current accepted photo list for one GREAT revision.
 * @param[in] config Session settings and either a source CSV or photo directory.
 * @return The accepted list plus newly arrived, sorted AI Camera basenames.
 * @throws std::runtime_error If the admission record, photo names, or row limit is invalid.
 * @details Existing rows remain a prefix; preserving their order is required
 * for stable revision identity and deterministic checkpoint reuse.
 */
inline std::string greatInputText(const MapConfig& config) {
    if (config.photoDirectory.empty()) return sessionText(config.sourceCsv);
    std::string text = "USHER_AICAM_V1\n";
    std::map<std::string, bool> admitted;
    if (fs::exists(config.output / "accepted")) {
        const auto record = sessionText(config.output / "accepted");
        if (record.size() < 65 || record[64] != '\n') throw std::runtime_error("Invalid AI Camera admission record");
        text = record.substr(65);
        std::istringstream lines(text);
        std::string name;
        if (!std::getline(lines, name) || name != "USHER_AICAM_V1") throw std::runtime_error("Invalid AI Camera snapshot schema");
        while (std::getline(lines, name)) admitted.emplace(name, true);
    }
    std::vector<std::string> arrivals;
    for (const auto& entry : fs::directory_iterator(config.photoDirectory)) {
        const auto name = entry.path().filename().string();
        if (name.empty() || name.front() != 'a' || entry.path().extension() != ".jpg" || admitted.count(name)) continue;
        if (admitted.size() + arrivals.size() >= 4096) throw std::runtime_error("AI Camera session exceeds 4096 photos");
        arrivals.push_back(name);
    }
    std::sort(arrivals.begin(), arrivals.end());
    for (const auto& name : arrivals) text += name + '\n';
    return text;
}

/**
 * @brief Atomically point `latest` at a completed, owned revision.
 * @param[in] root Session output root.
 * @param[in] revision Validated revision name beneath `revisions/`.
 * @throws std::runtime_error If staging, ownership checks, rename, or directory sync fails.
 */
inline void publishGreatLatest(const fs::path& root, const std::string& revision) {
    const auto pending = root / "latest.partial";
    if (fs::is_symlink(pending)) fs::remove(pending);
    if (fs::exists(pending)) throw std::runtime_error("Unexpected latest revision staging path");
    fs::create_directory_symlink(fs::path("revisions") / revision, pending);
    if (fs::exists(root / "latest") && !fs::is_symlink(root / "latest"))
        throw std::runtime_error("Latest revision path is not an owned symlink");
    fs::rename(pending, root / "latest");
    const int directory = open(root.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory < 0) throw std::runtime_error("Cannot open session publication directory");
    const int status = fsync(directory);
    close(directory);
    if (status != 0) throw std::runtime_error("Cannot synchronize latest revision");
}

/** @brief Validate a digest-based revision name and its preview/final suffix. */
inline bool greatRevisionName(const std::string& name) {
    if (name.size() < 64 || (name.substr(64) != "-preview" && name.substr(64) != "-final")) return false;
    return std::all_of(name.begin(), name.begin() + 64, [](char value) {
        return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
    });
}

/**
 * @brief Admit new photos and optionally publish a GREAT preview/final revision.
 * @param[in] settings Validated low-memory GREAT XYZ configuration.
 * @param[in] ini Configuration whose content participates in session identity.
 * @param[in] final True to require a closed photo list and finalize the session.
 * @param[in] render False to prepare source/checkpoint state without rendering.
 * @return True when input was admitted or a revision was processed; false when
 * no complete input row is available yet.
 * @throws std::runtime_error If session identity, append order, source contents,
 * checkpoints, output coverage, or publication checks fail.
 */
static bool updateGreatSession(const MapConfig& settings, const fs::path& ini, bool final, bool render = true) {
    if (!settings.lowMemory || settings.mode != "great" || settings.outputFormat != "xyz" ||
        !settings.captureManifest.empty())
        throw std::runtime_error("Incremental GREAT requires low_memory GREAT XYZ and CSV or AI Camera photos");
    const auto root = fs::weakly_canonical(fs::absolute(settings.output));
    const auto sourceDirectory = settings.photoDirectory.empty() ? fs::canonical(settings.sourceCsv).parent_path() :
                                                                  fs::canonical(settings.photoDirectory);
    if (root == sourceDirectory)
        throw std::runtime_error("Session output must be separate from the photo directory");
    const auto profileIdentity = settings.photoDirectory.empty() ? std::string() : fileDigest(settings.cameraProfile);
    const auto identity = "PANGEA_GREAT_SESSION_V1\n" + fileDigest(ini) + "\n" +
        (settings.photoDirectory.empty() ? fs::canonical(settings.sourceCsv) : sourceDirectory).generic_string() + "\n" +
        root.generic_string() + "\n" + (profileIdentity.empty() ? "" : profileIdentity + "\n");
    if (fs::exists(root)) {
        if (!fs::exists(root / "identity") || sessionText(root / "identity") != identity)
            throw std::runtime_error("Session identity changed or output is unowned; choose a new output");
    } else {
        fs::create_directories(root);
        saveWorkingText(root / "identity", identity);
    }
    auto text = greatInputText(settings);
    if (text.find('\0') != std::string::npos) throw std::runtime_error("NUL in growing photo CSV");
    const auto newline = text.rfind('\n');
    if (final && (text.empty() || newline != text.size() - 1))
        throw std::runtime_error("Final photo CSV has an unfinished row");
    if (newline == std::string::npos) return false;
    text.resize(newline + 1);
    if (text.size() > 4 * 1024 * 1024 - 65)
        throw std::runtime_error("Growing photo CSV exceeds admission record budget");
    if (std::count(text.begin(), text.end(), '\n') > 4097)
        throw std::runtime_error("Growing photo CSV exceeds 4096 rows");
    std::string accepted, acceptedIdentity;
    if (fs::exists(root / "accepted")) {
        const auto record = sessionText(root / "accepted");
        const auto separator = record.find('\n');
        if (separator != 64) throw std::runtime_error("Invalid session admission record");
        acceptedIdentity = record.substr(0, separator);
        accepted = record.substr(separator + 1);
        if (text.compare(0, accepted.size(), accepted) != 0)
            throw std::runtime_error("Committed photo CSV changed; only append is allowed");
    }
    if (std::count(text.begin(), text.end(), '\n') < 2) return false;
    const auto snapshot = root / "snapshot.csv";
    saveWorkingText(snapshot, text);
    auto config = settings;
    config.sourceDirectory = sourceDirectory;
    config.sourceCsv = snapshot;
    config.originalSourceCsv = settings.sourceCsv;
    const XyzPlan plan(config);
    Orthorectifier input(plan.workingConfig(config), snapshot);
    const auto photoIdentity = input.sourceFingerprint(isStopping);
    if (!profileIdentity.empty() && fileDigest(settings.cameraProfile) != profileIdentity)
        throw std::runtime_error("AI Camera profile changed during admission");
    const auto fingerprint = dataDigest(text.data(), text.size());
    const auto revisionName = fingerprint + (final ? "-final" : "-preview");
    const auto revision = root / "revisions" / revisionName;
    const auto revisionIdentity = "PANGEA_GREAT_REVISION_V1\n" + identity + fingerprint + "\n" + photoIdentity + "\n";
    std::string latest;
    if (fs::is_symlink(root / "latest")) {
        const auto target = fs::read_symlink(root / "latest");
        if (target.parent_path() != fs::path("revisions") || !greatRevisionName(target.filename().string()))
            throw std::runtime_error("Invalid session latest target");
        latest = target.filename().string();
        const auto prefix = "PANGEA_GREAT_REVISION_V1\n" + identity;
        if (sessionText(root / "latest" / "identity").compare(0, prefix.size(), prefix) != 0)
            throw std::runtime_error("Published revision belongs to another session");
        const cv::FileStorage status((root / "latest" / "status.json").string(), cv::FileStorage::READ);
        if (!status.isOpened()) throw std::runtime_error("Cannot verify latest revision status");
        if (static_cast<int>(status["finalized"]) && (!final || text != accepted))
            throw std::runtime_error("GREAT session is finalized; start a new output for another flight");
    }
    if (fs::exists(revision / "identity") && sessionText(revision / "identity") != revisionIdentity)
        throw std::runtime_error("Committed source photo changed; originals and previous revision retained");
    if (!accepted.empty()) {
        saveWorkingText(root / "previous.csv", accepted);
        Orthorectifier previous(plan.workingConfig(config), root / "previous.csv");
        if (previous.sourceFingerprint(isStopping) != acceptedIdentity)
            throw std::runtime_error("Previously committed source photo changed");
    }
    if (fs::exists(root / "revisions")) {
        const auto prefix = "PANGEA_GREAT_REVISION_V1\n" + identity;
        for (const auto& entry : fs::directory_iterator(root / "revisions")) {
            if (entry.path() == revision || entry.path().filename().string() == latest) continue;
            if (entry.is_symlink() || !entry.is_directory() || !greatRevisionName(entry.path().filename().string()) ||
                sessionText(entry.path() / "identity").compare(0, prefix.size(), prefix) != 0)
                throw std::runtime_error("Unowned revision prevents session cleanup");
            fs::remove_all(entry.path());
        }
    }
    fs::create_directories(revision);
    if (!fs::exists(revision / "identity")) saveWorkingText(revision / "identity", revisionIdentity);
    saveWorkingText(revision / "photos.csv", text);
    config.sourceCsv = revision / "photos.csv";
    config.workingDirectory = revision / "work";
    config.preparedSourceDirectory = root / "prepared";
    config.output = revision / "xyz";
    config.incrementalPreview = !final;
    if (!final) {
        config.mosaicFillGaps = false;
        config.xyzCompleteOnly = true;
    }
    fs::create_directories(config.workingDirectory);
    {
        Orthorectifier preparation(plan.workingConfig(config), config.sourceCsv);
        preparation.prepareSources(isStopping);
    }
    saveWorkingText(root / "accepted", photoIdentity + "\n" + text);
    if (!render) {
        saveWorkingText(root / "status.json", "{\"schema\":\"pangea_great_session_v1\",\"state\":\"prepared\","
            "\"finalized\":false,\"source_count\":" + std::to_string(input.sourceCount()) + "}");
        return true;
    }
    exportXyzTiles(config, config.sourceCsv, config.workingDirectory);
    if (input.sourceFingerprint(isStopping) != photoIdentity ||
        (!profileIdentity.empty() && fileDigest(settings.cameraProfile) != profileIdentity))
        throw std::runtime_error("Source changed during GREAT rendering; latest revision retained");
    const cv::FileStorage report((config.workingDirectory / "dataset.json").string(), cv::FileStorage::READ);
    if (!report.isOpened()) throw std::runtime_error("Cannot verify revision report");
    const int tiles = static_cast<int>(report["tile_count"]);
    std::ostringstream status;
    status << "{\"schema\":\"pangea_great_session_v1\",\"provisional\":" << (final ? "false" : "true")
           << ",\"finalized\":" << (final ? "true" : "false") << ",\"source_count\":" << input.sourceCount()
           << ",\"tile_count\":" << tiles << ",\"revision\":" << PhotoExif::jsonString(revisionName) << "}";
    saveWorkingText(revision / "status.json", status.str());
    if (tiles) publishGreatLatest(root, revisionName);
    saveWorkingText(root / "status.json", status.str());
    std::cout << "GREAT " << (final ? "final" : "provisional") << " revision: " << input.sourceCount()
              << " prepared photos, " << tiles << " published tiles; " << root.string() << '\n';
    return true;
}
}
#endif