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
 * @file dipla_worker.cpp
 * @brief Keep IPC responsive while one planner owns the survey and retained route.
 * @details
 * Two threads divide the work by ownership, not by algorithm:
 * - The calling thread validates packets, admits observations, and replies from
 *   small route snapshots. It never reads or mutates the dense GP covariance.
 * - The planner thread consumes the bounded FIFO in order, conditions the GP,
 *   and computes at most one completed route for each requested chunk token.
 * Credit-aware SHOT separately acknowledges assimilation so its owner can bound
 * new measurements without blocking packet validation or dropping evidence.
 *
 * Three identities answer different questions. The session token names the
 * requested route chunk; survey_epoch names the continuing GP experiment;
 * generation identifies a completed route within that chunk. An active START
 * changes the chunk without erasing observations. STOP ends the survey, and
 * the next START advances the epoch so its first observation gets a new GP.
 *
 * @par Why retain routes?
 * A waypoint retrieval is not a waypoint arrival. Only the flight controller
 * knows when its path is complete and when to request another chunk. Letting
 * every pose trigger a new route would move the targets while they are followed.
 * Whole-plan replies copy one generation; legacy point replies pin all eight
 * retrieval slots so a single cycle cannot assemble a patchwork of routes.
 *
 * @par When adjusting timing or capacity
 * Accepted measurements are evidence, not replaceable planning suggestions.
 * Queue overflow fails the survey rather than quietly losing evidence, and
 * freshness is measured from receipt, not from the end of expensive processing.
 * Keep those rules intact when changing cadence, queue size, or reply handling.
 *
 * @warning This is a trusted local SOCK_SEQPACKET ABI, not the UART protocol.
 * Socket calls and planner cancellation do not provide hard real-time bounds;
 * the owning process must still supervise a stalled or failed worker.
 * @see dipla_worker.hpp Public service and descriptor-ownership contract.
 * @see dipla_ipc.h Packet layout, fixed-point units, and reply validity rules.
 */
#include "dipla_worker.hpp"
#include "dipla_planner.hpp"
#include "dipla_ipc.h"
#include "../../src/bounded_ini.h"
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>

namespace dipla {
namespace {
/** @brief Monotonic time keeps clock synchronization or wall-clock edits from renewing stale data. */
using Clock = std::chrono::steady_clock;
constexpr std::size_t observation_queue_capacity = 64;
constexpr std::size_t worker_ini_byte_limit = 8192;
constexpr int minimum_cadence_ms = 100;
constexpr int maximum_cadence_ms = 5000;
constexpr double maximum_anchor_latitude_deg = 80.0;
constexpr double maximum_anchor_longitude_deg = 179.99;
constexpr double earth_radius_m = 6378137.0;
constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
constexpr double field_centre_m = DIPLA_FIELD_EXTENT_M / 2.0;

/**
 * @brief Immutable worker inputs and scheduling.
 * @details Separate worker scheduling from the model's inference
 * settings. Both threads borrow this object until the planner has been joined;
 * no configuration mutex or repeated trigonometry is needed in request handling.
 */
struct Configuration {
    /** @var Configuration::field
     * @brief Synthetic truth CSV path; required, even in comparison mode. */
    /** @var Configuration::model
     * @brief Model fallback or worker override; used only by diffusion. */
    /** Deployment metadata fallback or worker override, paired with model. */
    std::string field, model, metadata;
    /** Post-attempt planning cooldown in milliseconds, not a promised update frequency. */
    int cadence = 1000;
    /** Maximum receipt age of a pose allowed to start planning, in milliseconds. */
    unsigned planning_pose_max_age_ms = 1000;
    double field_size_m = DIPLA_FIELD_EXTENT_M;
    double field_rotation_deg = 0;
    double flight_height_m = DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M;
    double flight_height_tolerance_m = DIPLA_HEIGHT_TOLERANCE_M;

    /**
     * @brief Parse the bounded worker INI before either thread can use it.
    * @details Require the synthetic field explicitly. Reject
     * duplicate and unknown keys so configuration never depends on which value
     * happened to appear last. Optional model paths override entry-point defaults.
     * @param[in] filename Worker configuration file.
     * @param[in] default_model Executable-level model fallback.
     * @param[in] default_metadata Executable-level metadata fallback.
    * @throws std::exception If INI syntax, keys, file size, or cadence are invalid.
     * @note Worker path values are retained as supplied. Do not assume this
     * parser rebases them against the INI directory like model-settings discovery.
     */
    Configuration(const std::string& filename, const std::string& default_model, const std::string& default_metadata)
        : model(default_model), metadata(default_metadata) {
        /** @brief Callback context borrowed only during synchronous INI parsing. */
        struct Fields {
            /** Object under construction; never exposed to the planner yet. */
            Configuration& configuration;
            /** One bit per known key, separating absent values from defaults. */
            unsigned seen = 0;
        } fields{*this};
        const auto handler = [](void* context, const char* key, const char* value) {
            auto& parsed = *static_cast<Fields*>(context);
            auto& config = parsed.configuration;
            // The key order also defines the presence-mask bits below. Keep
            // that pairing together when extending the configuration grammar.
            const char* const keys[] = {"field", "model", "metadata", "cadence_ms",
                                       "planning_pose_max_age_ms", "field_size_m", "field_rotation_deg",
                                       "flight_height_m", "flight_height_tolerance_m"};
            size_t index = 0;
            while (index < std::size(keys) && strcmp(key, keys[index])) ++index;
            if (index == std::size(keys) || (parsed.seen & (1U << index))) return false;
            parsed.seen |= 1U << index;
            switch (index) {
                case 0: config.field = value; return true;
                case 1: config.model = value; return true;
                case 2: config.metadata = value; return true;
                case 3: {
                    uint64_t cadence_value = 0;
                    if (!ini_uint(value, &cadence_value) || cadence_value > maximum_cadence_ms) return false;
                    config.cadence = static_cast<int>(cadence_value);
                    return true;
                }
                case 4: {
                    uint64_t age = 0;
                    if (!ini_uint(value, &age) || !age || age > DIPLA_POINT_MAX_AGE_MS) return false;
                    config.planning_pose_max_age_ms = static_cast<unsigned>(age);
                    return true;
                }
                case 5: return ini_real(value, &config.field_size_m);
                case 6: return ini_real(value, &config.field_rotation_deg);
                case 7: return ini_real(value, &config.flight_height_m);
                case 8: return ini_real(value, &config.flight_height_tolerance_m);
                default: return false;
            }
        };
        // One extra byte accommodates the bounded reader's terminator, not a
        // larger settings file. Reuse the common parser instead of a second INI dialect.
        char contents[worker_ini_byte_limit + 1];
        size_t length = 0;
        const unsigned required = 1U << 0;
        if (!ini_read_file(filename.c_str(), contents, sizeof(contents), &length) ||
            !ini_parse(contents, length, handler, &fields) || (fields.seen & required) != required)
            throw std::runtime_error("Invalid diplacam configuration: required field, unique known keys, maximum 8192 bytes");
        if (cadence < minimum_cadence_ms || cadence > maximum_cadence_ms || field.empty())
            throw std::runtime_error("Invalid field or cadence (100..5000 ms)");
        if (field_size_m < 1 || field_size_m > DIPLA_FIELD_EXTENT_M ||
            field_rotation_deg < -180 || field_rotation_deg > 180 ||
            flight_height_m < DIPLA_FLIGHT_HEIGHT_MIN_MM / DIPLA_MM_PER_M ||
            flight_height_m > DIPLA_FLIGHT_HEIGHT_MAX_MM / DIPLA_MM_PER_M ||
            flight_height_tolerance_m <= 0 || flight_height_tolerance_m >= flight_height_m)
            throw std::runtime_error("Invalid field size, rotation or flight height/tolerance");
        if (field_size_m != DIPLA_FIELD_EXTENT_M &&
            ((flight_height_m - flight_height_tolerance_m) * DIPLA_FIELD_EXTENT_M / field_size_m < DIPLA_MODEL_Z_MIN_M ||
             (flight_height_m + flight_height_tolerance_m) * DIPLA_FIELD_EXTENT_M / field_size_m > DIPLA_MODEL_Z_MAX_M))
            throw std::runtime_error("Rescaled flight height interval must remain within the model's 10..40 m domain");
    }
};

/** @brief Socket-owned geographic frame, immutable throughout one survey. */
struct SurveyFrame {
    double latitude; /**< Actual anchor latitude in degrees. */
    double longitude; /**< Actual anchor longitude in degrees. */
    double north_scale = earth_radius_m * radians_per_degree; /**< Metres per latitude degree. */
    double east_scale; /**< Metres per longitude degree at the anchor. */
    double physical_scale;
    double rotation_cosine, rotation_sine;

    /** @brief Capture the first admitted position. @param request Validated anchor pose. */
        SurveyFrame(const dipla_request& request, const Configuration& config)
        : latitude(request.lat / DIPLA_GEO_UNITS_PER_DEGREE),
          longitude(request.lon / DIPLA_GEO_UNITS_PER_DEGREE),
                    east_scale(north_scale * std::cos(latitude * radians_per_degree)),
                    physical_scale(config.field_size_m / DIPLA_FIELD_EXTENT_M),
                    rotation_cosine(std::cos(config.field_rotation_deg * radians_per_degree)),
                    rotation_sine(std::sin(config.field_rotation_deg * radians_per_degree)) {}

    /** @brief Map a pose into the centred field. @param request Geographic pose. @return XYZ in metres. */
    std::array<double, 3> position(const dipla_request& request) const {
        const double east = (request.lon / DIPLA_GEO_UNITS_PER_DEGREE - longitude) * east_scale;
        const double north = (request.lat / DIPLA_GEO_UNITS_PER_DEGREE - latitude) * north_scale;
        return {{field_centre_m + (east * rotation_cosine + north * rotation_sine) / physical_scale,
                 field_centre_m + (-east * rotation_sine + north * rotation_cosine) / physical_scale,
                 request.agl_q8 / DIPLA_AGL_UNITS_PER_M / physical_scale}};
    }

    std::array<double, 2> physical_offset(const std::array<double, 3>& point) const {
        const double field_x = (point[0] - field_centre_m) * physical_scale;
        const double field_y = (point[1] - field_centre_m) * physical_scale;
        return {{field_x * rotation_cosine - field_y * rotation_sine,
                 field_x * rotation_sine + field_y * rotation_cosine}};
    }
};

/**
 * @brief Shared state exchanged between the socket loop and planner thread.
 * @details The mutex protects operational fields during concurrent use. Copy a
 * small observation or route while locked, then release the mutex before GP,
 * inference, or socket work. The condition variable announces work; its predicate,
 * not the notification count, decides whether there is anything to consume.
 * @invariant count never exceeds observations.size(); first identifies the next
 * FIFO item, and each queued item retains its original receipt time and epoch.
 * @warning failed is latched for this worker's lifetime. START is a chunk request,
 * not a magic eraser for failed evidence; recovery requires a new worker.
 */
struct Shared {
    /** Protects state and pairs with `changed` for planner wakeups. */
    std::mutex mutex;
    /** Wakes the planner when queued work or terminal state may have changed. */
    std::condition_variable changed;
    /** @var Shared::stopping
     * @brief Terminal shutdown request, observed before taking another FIFO item. */
    /** Latched failure; invalidates publication even if route bytes still exist. */
    bool stopping = false, failed = false;
    /** @var Shared::session
     * @brief Active chunk token, or zero when STOP has ended the survey. */
    /** Published route identity within the active chunk; zero means no usable route. */
    uint32_t session = 0, generation = 0;
    /** GP lifetime identity; advances only on START from the stopped state. */
    std::uint64_t survey_epoch = 0;
    /**
     * @brief One admitted pose with the context needed after delayed processing.
     * @details A chunk transition may retain this item, but an epoch transition
     * must not let an old survey's measurement become new-survey evidence.
     */
    struct Observation {
        /** Original pose and request identity; not replaced by a later request. */
        dipla_request request{};
        /** Position converted in the frame that admitted this observation. */
        std::array<double, 3> position{};
        /** Local admission time, never refreshed merely because processing was slow. */
        Clock::time_point received{};
        /** Survey identity at admission, independent of the request's chunk token. */
        std::uint64_t epoch = 0;
    };
    /** Bounded evidence FIFO; dropping or coalescing entries would change GP history. */
    std::array<Observation, observation_queue_capacity> observations{};
    /** @var Shared::first
     * @brief Ring index of the oldest admitted observation. */
    /** Number of occupied ring entries, excluding any item already owned by the planner. */
    std::size_t first = 0, count = 0;
    /** Receipt-time observation lease; later assimilation may renew it without replacing route. */
    Clock::time_point plan_time{};
    /** Most recently completed route, copied under the mutex. */
    Waypoints route{};

    /**
     * @brief Admit a pose exactly once without silently replacing older evidence.
     * @details Overload invalidates the route immediately. Overwriting the oldest
     * item or keeping only the latest pose would make the GP claim a measurement
     * history different from the one the socket owner successfully submitted.
     * @param[in] request Validated, increasing observation request for the active chunk.
    * @param[in] position Pose in the survey's immutable local frame.
     * @param[in] received Original local receipt time for freshness accounting.
     * @return True after FIFO admission; false after latching overflow failure.
     * @pre The caller holds mutex and has checked the active session and failed flag.
     * @post A waiting planner is notified on either admission or overflow.
     * @note The caller must still send a correlated invalid reply on overflow;
     * a failed survey is not a reason to leave its requester waiting in silence.
     */
    bool enqueue(const dipla_request& request, const std::array<double, 3>& position, Clock::time_point received) {
        if (count == observations.size()) {
            failed = true;
            generation = 0;
            std::cerr << "diplacam observation queue overflow; survey stopped\n";
            changed.notify_one();
            return false;
        }
        // Append at the tail rather than shifting entries. The small fixed ring
        // bounds queue storage; it does not bound the GP or ONNX Runtime's allocations.
        observations[(first + count) % observations.size()] = {request, position, received, survey_epoch};
        ++count;
        changed.notify_one();
        return true;
    }
};

/**
 * @brief Check geographic representation, local field bounds and demo height.
 * @param request Original integer-valued IPC pose.
 * @param position Pose converted to local metres.
 * @return True if the pose is compatible with the fixed demonstration domain.
 */
bool valid_pose(const dipla_request& request, const std::array<double, 3>& position, const Configuration& config) {
    return request.lat >= -DIPLA_LATITUDE_LIMIT && request.lat <= DIPLA_LATITUDE_LIMIT &&
           request.lon >= -DIPLA_LONGITUDE_LIMIT && request.lon <= DIPLA_LONGITUDE_LIMIT &&
           (request.lat != 0 || request.lon != 0) &&
           position[0] >= 0 && position[0] <= DIPLA_FIELD_EXTENT_M &&
           position[1] >= 0 && position[1] <= DIPLA_FIELD_EXTENT_M &&
           std::abs(request.agl_q8 / DIPLA_AGL_UNITS_PER_M - config.flight_height_m) <= config.flight_height_tolerance_m;
}

/**
 * @brief Run the sole planner thread until shutdown or an unrecoverable error.
 * @details Assimilates each queued observation once. A new chunk is planned
 * only from a pose within the configured startup age; delayed measurements still update
 * the GP without becoming fresh planning input. Expensive work runs unlocked;
 * publication rechecks the chunk token so a completed old job cannot overwrite
 * the current chunk. The flight controller alone requests the next chunk.
 * @param[in,out] shared Synchronized evidence FIFO, lifecycle, and publication state.
 * @param[in] config Immutable worker configuration, alive until this thread exits.
 * @param[in] cmaes Explicit comparison-only selector; never an inference-failure fallback.
 * @param[in] comparison Comparison search bounds and cooperative deadline.
 * @param[in] network_options Descriptor and whole-DDIM execution policy.
 * @param[in] experiment Immutable GP, sensor, seed and publication-margin policy.
 * @pre The service keeps all referenced inputs alive until joining this thread.
 * @note Standard exceptions latch failure and clear route validity. A retained
 * route must not conceal that its GP or inference owner has failed.
 * @warning CMA-ES polls session-aware cancellation. Diffusion uses its network
 * deadline and checks session validity at commit, not between each step here.
 */
void plan(int descriptor, Shared& shared, const Configuration& config, bool cmaes, const CmaesOptions& comparison,
          const NetworkOptions& network_options, const ExperimentOptions& experiment) {
    try {
        // Load once, not once per observation or chunk. Comparison mode must
        // remain usable without opening the diffusion graph or ONNX session.
        std::unique_ptr<Network> network;
        if (!cmaes) network = std::make_unique<Network>(config.model, config.metadata, network_options);
        std::cerr << "diplacam planner=" << (cmaes ? "cmaes comparison_only=true onnx=false" : "diffusion onnx=true") << '\n';
        std::unique_ptr<Survey> survey;
        // This stream belongs to diffusion only; Survey owns a separate sensor
        // noise stream. Changing inference cadence must not change sensor noise.
        std::mt19937_64 generator(experiment.diffusion_seed);
        std::normal_distribution<double> normal;
        // Wire observation numbers restart at chunk boundaries, but GP evidence
        // numbers must increase throughout the survey. Keep a private counter.
        uint32_t planned_session = 0, observation_number = 0;
        std::uint64_t previous_epoch = 0;
        auto next = Clock::now();
        for (;;) {
            std::unique_lock<std::mutex> lock(shared.mutex);
            // Predicate-based waiting handles both spurious wakeups and notices
            // delivered before we started waiting. No queued pose, no busy loop.
            shared.changed.wait(lock, [&] { return shared.stopping || shared.failed || shared.count; });
            if (shared.stopping || shared.failed) return;
            // Take one item by value so the socket thread can immediately reuse
            // its ring slot while the planner works on this private copy.
            const auto observation = shared.observations[shared.first];
            shared.first = (shared.first + 1) % shared.observations.size();
            --shared.count;
            const auto session = shared.session;
            const auto survey_epoch = observation.epoch;
            const auto request = observation.request;
            const auto pose_time = observation.received;
            if (!session || survey_epoch != shared.survey_epoch) continue;
            // Planning touches large GP state and may run for hundreds of
            // milliseconds; never hold the IPC mutex across that work.
            lock.unlock();
            const auto update_started = Clock::now();
            if (survey_epoch != previous_epoch) {
                // Only a new survey epoch resets knowledge, not a new route
                // chunk. Release the old dense covariance before allocating its
                // replacement; briefly keeping both would need another ~51.6 MiB.
                survey.reset();
                survey = std::make_unique<Survey>(config.field, experiment);
                previous_epoch = survey_epoch;
                observation_number = 0;
                planned_session = 0;
                next = Clock::time_point{};
                std::cerr << "diplacam field_fnv1a=" << survey->field_digest()
                          << " physical_flight_height_m=" << config.flight_height_m
                          << " predicted_z_discarded=true\n";
            }
            const auto position = observation.position;
            // Wraparound would make fresh evidence look like a duplicate to
            // Survey::observe. Exhaustion fails explicitly instead of losing updates.
            if (observation_number == UINT32_MAX) throw std::runtime_error("Observation sequence exhausted");
            const double sensor_height = config.field_size_m == DIPLA_FIELD_EXTENT_M &&
                config.flight_height_m == DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M
                ? DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M : position[2];
            survey->observe(position[0], position[1], ++observation_number, sensor_height);
            if (request.operation == DIPLA_IPC_CREDIT_SHOT) {
                const dipla_reply acknowledgement{DIPLA_IPC_VERSION, request.session, request.number,
                                                  0, 0, 0, 0, 0, DIPLA_IPC_ASSIMILATED_METRIC};
                ssize_t sent;
                do { sent = send(descriptor, &acknowledgement, sizeof(acknowledgement), MSG_NOSIGNAL | MSG_DONTWAIT); }
                while (sent < 0 && errno == EINTR);
                if (sent != sizeof(acknowledgement)) throw std::runtime_error("Observation acknowledgement failed");
            }
            if (observation_number == 1)
                std::cerr << "diplacam initial_gp_update_ms="
                          << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - update_started).count() << '\n';
            // Assimilate first, then decide whether this item may START planning.
            // Old-chunk FIFO items still belong to this survey, and delayed poses
            // are still measurements; neither is a fresh planning authorization.
            // planned_session retains the route while later observations refine GP.
            if (request.session != session || planned_session == session || Clock::now() < next ||
                Clock::now() - pose_time > std::chrono::milliseconds(config.planning_pose_max_age_ms)) {
                lock.lock();
                // Refresh a retained route's observation lease only after this
                // chunk's pose has reached the GP. Preserve its receipt time:
                // finishing a late update cannot make old evidence young again.
                if (request.session == shared.session && survey_epoch == shared.survey_epoch &&
                    shared.generation && !shared.stopping)
                    shared.plan_time = pose_time;
                continue;
            }
            std::optional<Waypoints> route;
            const auto inference_started = Clock::now();
            if (cmaes) {
                route = survey->compare(position, comparison, [&] {
                    std::lock_guard<std::mutex> guard(shared.mutex);
                    return shared.stopping || shared.session != session || shared.survey_epoch != survey_epoch;
                });
            } else {
                Noise noise{};
                for (double& value : noise) value = normal(generator);
                route = network->sample(survey->belief(), position, noise);
            }
            // Cool down after the attempt, not from its start. This is a minimum
            // idle budget, not a fixed-rate timer; another eligible pose is needed
            // to start future work, and FIFO assimilation continues meanwhile.
            next = Clock::now() + std::chrono::milliseconds(config.cadence);
            if (!route) {
                std::cerr << "diplacam CMA-ES comparison interrupted before a complete route; no new points\n";
                continue;
            }
            // Both planners share the same application flight padding. Keep it
            // outside Network::sample so raw DDIM output stays reference-testable.
            apply_flight_margin(*route, experiment.flight_margin_cells);
            lock.lock();
            // A stop/start may have happened while planning. Discard stale
            // output rather than exposing points from the previous session.
            if (session == shared.session && survey_epoch == shared.survey_epoch && !shared.stopping && !shared.failed) {
                // Legacy metric packs three slot bits below generation into a
                // signed int32. Cap generation at 28 bits rather than wrapping
                // route identity or setting the metric's sign bit.
                if (shared.generation == DIPLA_MAX_GENERATION) throw std::runtime_error("Plan generation exhausted");
                shared.route = *route;
                ++shared.generation;
                planned_session = session;
                std::cerr << "diplacam chunk=" << session << " observations=" << observation_number
                          << " posterior_variance=" << survey->total_variance()
                          << " inference_ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - inference_started).count() << '\n';
                // Publish route, generation, and lease as one locked state change.
                // Planning time consumes freshness; it does not earn a new lease.
                shared.plan_time = pose_time;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "diplacam worker: " << error.what() << '\n';
        // The socket owner may keep receiving long enough to answer invalid.
        // Invalidate even a previously good route; stale success must not hide failure.
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.failed = true;
        shared.generation = 0;
    }
}
}

/*
 * Service implementation notes; the public Doxygen contract lives in the header.
 *
 * Keep session transitions and publication snapshots under the same mutex, but
 * keep blocking socket I/O outside it. The planner can then condition its GP even
 * while the owner is slow to receive. This does not impose a socket timeout;
 * the parent still owns transport supervision and closes the borrowed descriptor.
 *
 * PLAN returns one complete bank. SHOT returns a slot from a pinned bank and
 * caches that exact reply for retries. Both read the same completed route; neither
 * advances aircraft navigation. Slot seven is not a tiny finish-line detector.
 *
 * Owners must give each new chunk a distinct token while old work can still be
 * in flight. Publication rejects a changed token, not a history of reused tokens.
 */
int serve(int descriptor, const std::string& config_path, const std::string& model, const std::string& metadata,
          bool cmaes, const CmaesOptions& comparison, const NetworkOptions& network_options,
          const ExperimentOptions& experiment) {
    experiment.validate();
    comparison.validate();
    Configuration config(config_path, model, metadata);
    if (cmaes && (config.field_size_m != DIPLA_FIELD_EXTENT_M ||
                  config.flight_height_m != DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M))
        throw std::runtime_error("Resized or nondefault-height worker profiles require the diffusion planner");
    Shared shared;
    // Start only after config is validated. References remain valid until join;
    // neither the worker nor its exception handler takes ownership of the socket.
    std::thread worker(plan, descriptor, std::ref(shared), std::cref(config), cmaes, std::cref(comparison),
                       std::cref(network_options), std::cref(experiment));
    // These retrieval/cache fields belong to the socket thread alone. The GP
    // never sees a slot cursor, which prevents retrieval from becoming navigation.
    Waypoints pinned{};
    uint32_t slot = 0, generation = 0, last_number = 0;
    bool last_uses_credit = false;
    dipla_reply cached{};
    auto last_request = Clock::time_point{}, pinned_time = Clock::time_point{};
    bool transport_failed = false;
    std::optional<SurveyFrame> frame;
    const auto admit = [&](const dipla_request& request, const std::array<double, 3>& position, Clock::time_point received) {
        if (!shared.enqueue(request, position, received)) return false;
        if (!frame) {
            frame.emplace(request, config);
            std::cerr << "diplacam anchor_lat_e7=" << request.lat << " anchor_lon_e7=" << request.lon
                      << " anchor_shot=" << request.number << " field_centre_m=" << field_centre_m
                      << " field_size_m=" << config.field_size_m << " field_rotation_deg=" << config.field_rotation_deg
                      << " physical_agl_m=" << request.agl_q8 / DIPLA_AGL_UNITS_PER_M
                      << " model_agl_m=" << position[2] << '\n';
        }
        return true;
    };
    for (;;) {
        dipla_request request{};
        // SOCK_SEQPACKET preserves boundaries. MSG_TRUNC reports an oversized
        // packet's full length so its plausible prefix cannot pass the size check.
        // This is not a byte-stream parser: reconnect rather than resynchronize
        // after a malformed packet or an incompatible layout version.
        const auto count = recv(descriptor, &request, sizeof(request), MSG_TRUNC);
        if (count < 0 && errno == EINTR) continue;
        if (!count) break;
        if (count != sizeof(request) || request.version != DIPLA_IPC_VERSION || !request.session) {
            transport_failed = true;
            break;
        }
        std::unique_lock<std::mutex> lock(shared.mutex);
        if (request.operation == DIPLA_IPC_START || request.operation == DIPLA_IPC_STOP) {
            // A repeated START is idempotent; a stale STOP must not cancel a
            // newer chunk. Active START requests another route while retaining
            // the same survey epoch and its accepted, still-pending observations.
            if (request.operation == DIPLA_IPC_STOP && shared.session != request.session) continue;
            if (request.operation == DIPLA_IPC_START) {
                if (shared.session == request.session) continue;
                if (!shared.session) {
                    // A reused epoch could mislabel queued evidence. Even this
                    // practically remote counter limit must fail instead of wrap.
                    if (shared.survey_epoch == UINT64_MAX) { transport_failed = true; break; }
                    ++shared.survey_epoch;
                    frame.reset();
                }
            }
            shared.session = request.operation == DIPLA_IPC_START ? request.session : 0;
            // Invalidate publication immediately; old bytes may stay in route,
            // but generation zero prevents treating them as this chunk's result.
            shared.generation = 0;
            if (!shared.session) {
                shared.first = shared.count = 0;
                frame.reset();
            }
            // Every chunk begins a new retrieval cycle and wire-number history.
            // Only STOP discards pending evidence; chunk handoff must not lose it.
            slot = generation = last_number = 0;
            last_uses_credit = false;
            cached = {};
            shared.changed.notify_one();
            continue;
        }
        const auto position = frame ? frame->position(request) :
            std::array<double, 3>{{field_centre_m, field_centre_m,
                                  request.agl_q8 / DIPLA_AGL_UNITS_PER_M * DIPLA_FIELD_EXTENT_M / config.field_size_m}};
        const bool pose_valid = valid_pose(request, position, config) &&
            (frame || (std::abs(request.lat / DIPLA_GEO_UNITS_PER_DEGREE) <= maximum_anchor_latitude_deg &&
                       std::abs(request.lon / DIPLA_GEO_UNITS_PER_DEGREE) <= maximum_anchor_longitude_deg));
        if (request.operation == DIPLA_IPC_PLAN && request.session == shared.session && request.number) {
            const auto now = Clock::now();
            // Check geographic range as well as local XY, and require the fixed
            // 2 m experiment's altitude tolerance. This validates the input pose,
            // not arrival at any planned waypoint or suitability of model Z.
            // Even an unready or rejected request gets its own correlated INVALID
            // reply. Zero initialization ensures no leftover route can leak as data.
            dipla_plan_reply reply{};
            reply.version = DIPLA_IPC_VERSION;
            reply.session = request.session;
            reply.number = request.number;
            if (request.number > last_number && pose_valid && !shared.failed) {
                // Admission is asynchronous: the route returned below, if any,
                // predates assimilation of this pose. Do not call it a new plan.
                // Overflow clears generation under this lock, suppressing validity.
                (void)admit(request, position, now);
                last_number = request.number;
                last_uses_credit = false;
                if (shared.generation && now - shared.plan_time <= std::chrono::milliseconds(DIPLA_POINT_MAX_AGE_MS)) {
                    // Copy all eight points under the lock so one reply cannot
                    // mix generations even if the planner completes concurrently.
                    reply.valid = 1;
                    reply.generation = shared.generation;
                    reply.age_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - shared.plan_time).count());
                    reply.count = points;
                    for (size_t index = 0; index < shared.route.size(); ++index) {
                        const auto& point = shared.route[index];
                        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
                            point[0] < 0 || point[0] > DIPLA_FIELD_EXTENT_M ||
                            point[1] < 0 || point[1] > DIPLA_FIELD_EXTENT_M) {
                            reply.valid = reply.count = 0;
                            break;
                        }
                        // Bounds precede narrowing to integer millimetres. The
                        // repeated field check defends this publication boundary;
                        // the shared flight-margin helper already padded the route.
                        reply.points[index].east_mm = static_cast<int32_t>(std::llround(point[0] * DIPLA_MM_PER_M));
                        reply.points[index].north_mm = static_cast<int32_t>(std::llround(point[1] * DIPLA_MM_PER_M));
                        // Predicted model Z is intentionally not an altitude command.
                        reply.points[index].height_mm = static_cast<int32_t>(std::llround(config.flight_height_m * DIPLA_MM_PER_M));
                    }
                }
            }
            // Unlike SHOT, PLAN does not replay cached data for a duplicate
            // number: it returns correlated INVALID without another observation.
            lock.unlock();
            // Socket I/O must not hold the planner mutex: a blocked owner may
            // lose this worker, but must not freeze GP progress under the lock.
            ssize_t sent;
            // A disconnected owner is an error, not SIGPIPE termination. Retry
            // interrupted calls only; a short packet send is a broken contract,
            // not permission to send a second fragment as a separate message.
            do { sent = send(descriptor, &reply, sizeof(reply), MSG_NOSIGNAL | MSG_DONTWAIT); } while (sent < 0 && errno == EINTR);
            if (sent != sizeof(reply)) { transport_failed = true; break; }
            continue;
        }
        // Unknown operations and inactive/mismatched requests never enter the GP.
        // SHOT follows the one-point contract below, not PLAN's whole-bank format.
           if ((request.operation != DIPLA_IPC_SHOT && request.operation != DIPLA_IPC_CREDIT_SHOT &&
               request.operation != DIPLA_IPC_POINT) ||
            request.session != shared.session || !request.number) continue;
        const auto now = Clock::now();
        dipla_reply reply{DIPLA_IPC_VERSION, request.session, request.number, 0, 0, 0, 0, 0, 0};
        bool observation_admitted = false;
        if (request.number == last_number) {
            // Replay the original reply, including an originally INVALID one.
            // Retrying is neither another independent measurement nor permission
            // to consume the next target or upgrade an earlier reply to success.
            if (cached.session == request.session && cached.number == request.number) reply = cached;
            // Failure and expiry override cached success; caching is not a lease extension.
            if (shared.failed || now - pinned_time > std::chrono::milliseconds(DIPLA_POINT_MAX_AGE_MS)) reply.valid = 0;
        }
        else if (request.number > last_number && pose_valid && !shared.failed) {
            if (request.operation != DIPLA_IPC_POINT) observation_admitted = admit(request, position, now);
            // A long retrieval gap restarts at P1. It says nothing about aircraft
            // arrival: the slot cursor belongs to transport, not the flight plan.
            if (now - last_request > std::chrono::milliseconds(DIPLA_CYCLE_TIMEOUT_MS)) slot = 0;
            if (!slot && shared.generation && now - shared.plan_time <= std::chrono::milliseconds(DIPLA_POINT_MAX_AGE_MS)) {
                // Capture route, identity, and lease together ONLY at cycle start.
                // Later assimilation may refresh shared.plan_time, but cannot
                // rejuvenate a bank already being delivered partway through.
                pinned = shared.route;
                generation = shared.generation;
                pinned_time = shared.plan_time;
            }
            if (!shared.failed && generation && now - pinned_time <= std::chrono::milliseconds(DIPLA_POINT_MAX_AGE_MS)) {
                // Invert the anchored frame consistently: north -> latitude, east ->
                // longitude. Keep FC altitude as supplied and declare fixed AGL;
                // neither the sampler's Z nor an invented altitude datum belongs here.
                const auto offset = frame->physical_offset(pinned[slot]);
                const double latitude = frame->latitude + offset[1] / frame->north_scale;
                const double longitude = frame->longitude + offset[0] / frame->east_scale;
                reply.valid = 1;
                reply.lat = static_cast<int32_t>(std::llround(latitude * DIPLA_GEO_UNITS_PER_DEGREE));
                reply.lon = static_cast<int32_t>(std::llround(longitude * DIPLA_GEO_UNITS_PER_DEGREE));
                reply.alt_mm = request.alt_mm;
                reply.agl_mm = static_cast<int32_t>(std::llround(config.flight_height_m * DIPLA_MM_PER_M));
                // Internal slots 0..7 map to P1..P8. Generation distinguishes the
                // bank; advancing the slot does not tell the aircraft where to fly next.
                reply.metric = static_cast<int32_t>((generation << DIPLA_SLOT_BITS) | slot);
                slot = (slot + 1) % points;
            } else slot = generation = 0;
        } else slot = generation = 0;
        // Remember every newer SHOT reply, including rejection. Reusing its number
        // must not turn an invalid attempt into fresh evidence after a pose change.
        // Older numbers never replace this high-water mark or its replay cache.
        const bool rejected_credit = request.operation == DIPLA_IPC_CREDIT_SHOT &&
            ((request.number > last_number && !observation_admitted) ||
             (request.number == last_number && !last_uses_credit));
        if (request.number > last_number) {
            last_number = request.number;
            last_uses_credit = request.operation == DIPLA_IPC_CREDIT_SHOT;
            cached = reply;
            last_request = now;
        }
        lock.unlock();
        ssize_t sent;
        do { sent = send(descriptor, &reply, sizeof(reply), MSG_NOSIGNAL | MSG_DONTWAIT); } while (sent < 0 && errno == EINTR);
        if (sent != sizeof(reply)) { transport_failed = true; break; }
        if (rejected_credit) {
            const dipla_reply acknowledgement{DIPLA_IPC_VERSION, request.session, request.number,
                                              0, 0, 0, 0, 0, DIPLA_IPC_ASSIMILATED_METRIC};
            do { sent = send(descriptor, &acknowledgement, sizeof(acknowledgement), MSG_NOSIGNAL | MSG_DONTWAIT); }
            while (sent < 0 && errno == EINTR);
            if (sent != sizeof(acknowledgement)) { transport_failed = true; break; }
        }
    }
    // Wake an idle planner as well as one checking cooperative cancellation.
    // Join before config/shared/reference arguments leave scope; a detached GP
    // owner would otherwise outlive the very state that keeps it synchronized.
    {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.stopping = true;
        shared.changed.notify_one();
    }
    worker.join();
    // After join there is no concurrent writer, so reading the failure latch
    // needs no lock. Peer closure alone is orderly; malformed transport is not.
    return transport_failed || shared.failed ? 1 : 0;
}
}
