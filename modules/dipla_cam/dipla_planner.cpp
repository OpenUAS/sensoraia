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

/* Additional author attribution: Copyright 2026 Akshat Jain */

/**
 * @file dipla_planner.cpp
 * @brief Turn a synthetic survey's GP belief into eight model or comparison points.
 * @details
 * Three responsibilities deliberately meet here without becoming one algorithm:
 * - Survey owns the hidden field and Gaussian-process (GP) posterior. Only a
 *   sensor footprint may reveal truth; planners receive the resulting belief.
 * - Network adapts that belief to the exported ONNX denoiser and performs the
 *   complete deterministic DDIM reverse process, returning raw XYZ in metres.
 * - apply_flight_margin() prepares either planner's route for publication,
 *   without changing the sampler's numerically testable model contract.
 *
 * The flight controller, not this file, decides when a route has been flown.
 * Keeping inference, flight preparation, and mission progress separate lets us
 * compare the sampler with the Python reference without importing flight policy
 * into the maths. There is one neural network here, not a second flight network.
 *
 * @par When adapting a model
 * Grid geometry, axis order, world scaling, tensor bindings, normalizations,
 * and the diffusion schedule form one contract. Update and test them together;
 * a tensor with the right shape can still describe entirely the wrong world.
 *
 * @warning Reused network tensors and mutable survey state require serialized
 * ownership. Fixed tensor buffers do not make ONNX Runtime or the GP heap-free;
 * the dense covariance alone occupies about 51.6 MiB.
 * @see dipla_planner.hpp Public ownership, input, and result contracts.
 */
#include "dipla_planner.hpp"
#include <chrono>
#include "dipla_gp_belief_planner.hpp"
#include <jsonbourne.h>
#include "../../src/onnx_runtime.h"
#include "../../src/onnx_session.h"
#include "../../src/inference_model.h"
#include "../../src/bounded_ini.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace dipla {
/**
 * @details Delegate runtime discovery to the shared loader so Diplacam uses the
 * same library-selection and process-lifetime rules as other ONNX consumers.
 * Keeping an error buffer here turns the C API's bounded diagnostic into a C++
 * exception before a Network can acquire a session from an unavailable runtime.
 * @see Network::Network
 */
void initialize_runtime(const std::string& onnx_lib_path) {
    std::array<char, 16384> error{};
    if (!onnx_runtime_open(onnx_lib_path.c_str(), error.data(), error.size()))
        throw std::runtime_error(error.data());
}

namespace {
constexpr std::size_t metadata_byte_limit = 16 * 1024;
constexpr std::size_t metadata_arena_bytes = 64 * 1024;
constexpr std::size_t runtime_error_capacity = 16 * 1024;
constexpr std::size_t field_byte_limit = 2 * 1024 * 1024;
constexpr std::size_t field_row_byte_limit = 256;
constexpr double field_alignment_tolerance_m = 1e-9;
constexpr std::uint64_t fnv_offset_basis = UINT64_C(14695981039346656037);
constexpr std::uint64_t fnv_prime = UINT64_C(1099511628211);
constexpr double xy_half_extent_m = DIPLA_FIELD_EXTENT_M / 2.0;
constexpr double model_z_half_range_m = (DIPLA_MODEL_Z_MAX_M - DIPLA_MODEL_Z_MIN_M) / 2.0;

/** @brief One named training-time affine normalization. */
struct Normalization {
    double center = 0; /**< Training center. */
    double scale = 1; /**< Positive training scale. */
};

bool inside_field(double east, double north) {
    return std::isfinite(east) && std::isfinite(north) && east >= 0 && east <= DIPLA_FIELD_EXTENT_M &&
           north >= 0 && north <= DIPLA_FIELD_EXTENT_M;
}

std::size_t cell_index(std::size_t row, std::size_t column) { return row * DIPLA_GRID_SIDE + column; }

/**
 * @brief Find a unique required member of a metadata object.
 * @details Length-aware comparison handles decoded keys without accepting
 * embedded-NUL prefixes or ambiguous duplicate members. Different JSON tools
 * can choose different winners for duplicate keys; rejecting them avoids that
 * disagreement at the model boundary rather than guessing which export wins.
 * @param[in] tree JSON object containing the requested key.
 * @param[in] key Required member name, compared in full.
 * @return Required member view backed by the caller's parser storage.
 * @throws std::runtime_error If the object or member is absent, mistyped, or duplicated.
 * @warning Do not retain the returned reference beyond the parser arena's lifetime.
 */
const jsonbourne_val& metadata_member(const jsonbourne_val& tree, std::string_view key) {
    if (tree.type != JSONBOURNE_TYPE_OBJ) throw std::runtime_error("Expected metadata object");
    const auto& object = *static_cast<const jsonbourne_obj*>(tree.ptr);
    const jsonbourne_val* found = nullptr;
    for (const auto* member = object.first; member; member = member->next) {
        if (std::string_view(member->key->str, member->key->len) != key) continue;
        if (found) throw std::runtime_error("Duplicate metadata member: " + std::string(key));
        found = member->val;
    }
    if (!found) throw std::runtime_error("Missing metadata member: " + std::string(key));
    return *found;
}

/**
 * @brief Convert a JSON number without locale dependence or implicit coercion.
 * @details Metadata must mean the same thing on the development PC and MORA.
 * Parsing the entire numeric token avoids locale surprises and silently
 * accepting a usable-looking prefix, a quoted number, or a nonfinite value.
 * @param[in] node Required numeric value backed by live parser storage.
 * @return Finite double, with the complete token consumed.
 * @throws std::runtime_error If the type, conversion, or range is invalid.
 */
double metadata_number(const jsonbourne_val& node) {
    if (node.type != JSONBOURNE_TYPE_NUM) throw std::runtime_error("Expected metadata number");
    const auto& number = *static_cast<const jsonbourne_num*>(node.ptr);
    double value = 0;
    const auto result = std::from_chars(number.str, number.str + number.len, value);
    if (result.ec != std::errc{} || result.ptr != number.str + number.len || !std::isfinite(value))
        throw std::runtime_error("Invalid metadata number");
    return value;
}

/**
 * @brief Extract a finite numeric object member.
 * @details Reuse the same lookup and conversion rules for world geometry,
 * normalization, and scheduling; none should quietly accept weaker metadata.
 * @param[in] tree Metadata object.
 * @param[in] key Required member name.
 * @return Checked numeric value.
 * @throws std::runtime_error If lookup or finite-number conversion fails.
 * @see metadata_member
 * @see metadata_number
 */
double finite_number(const jsonbourne_val& tree, std::string_view key) {
    return metadata_number(metadata_member(tree, key));
}

/**
 * @brief Extract an integer token without accepting fractions or exponents.
 * @details Dimensions and step counts are discrete contract fields. Requiring
 * an actual integer token makes accidental exporter changes visible instead
 * of rounding or coercing them into something that merely fits a tensor.
 * @param[in] tree Metadata object.
 * @param[in] key Required member name.
 * @return Checked integer value.
 * @throws std::runtime_error If the member is not a representable integer token.
 */
int metadata_integer(const jsonbourne_val& tree, std::string_view key) {
    const auto& node = metadata_member(tree, key);
    if (node.type != JSONBOURNE_TYPE_NUM) throw std::runtime_error("Expected metadata integer");
    const auto& number = *static_cast<const jsonbourne_num*>(node.ptr);
    int value = 0;
    const auto result = std::from_chars(number.str, number.str + number.len, value);
    if (result.ec != std::errc{} || result.ptr != number.str + number.len)
        throw std::runtime_error("Invalid metadata integer: " + std::string(key));
    return value;
}

}

/**
 * @brief Runtime-owned ONNX session and reusable input/output storage.
 * @details Bind once and reuse across all twenty denoising calls and later
 * plans. This avoids rebuilding application-side tensors in the hot loop;
 * it does not promise allocation-free execution inside ONNX Runtime.
 * @invariant The session borrows member buffers whose addresses remain stable
 * until the session is closed by the destructor.
 * @warning A State is single-owner mutable workspace, not a concurrent sampler.
 */
struct Network::State {
    /** @brief Own the paired encoder and cached denoiser plus their bound tensors. */
    struct ConditioningCache {
        onnx_session encoder{}, denoiser{};
        inference_model encoder_model{}, denoiser_model{};
        std::array<std::vector<float>, ONNX_TENSOR_LIMIT> values;

        /** @brief Validate the split model contract and open both runtime sessions.
         * @param directory Directory containing split graphs and their descriptors.
         * @param source Source graph path used to validate the cache identity.
         * @param inputs Canonical model inputs against which the split bindings are checked.
         * @param outputs Canonical model outputs against which the split bindings are checked.
         * @param options Runtime options shared by both sessions.
         * @throws std::runtime_error On identity, binding or runtime initialization failure.
         * @details Check that the artifacts belong together before binding shared
         * buffers. Close the encoder explicitly if the second session cannot open. */
        ConditioningCache(const std::string& directory, const std::string& source,
                          const onnx_tensor* inputs, const onnx_tensor* outputs,
                          const onnx_session_options& options) {
            const std::filesystem::path root(directory);
            std::array<char, runtime_error_capacity> error{};
            struct Identities {
                std::array<std::string, 3> hashes;
                unsigned seen = 0;
            } identities;
            const auto identify = [](void* context, const char* key, const char* value) {
                auto& parsed = *static_cast<Identities*>(context);
                const std::array<std::string_view, 3> keys{{"source_sha256", "encoder_descriptor_sha256", "denoiser_descriptor_sha256"}};
                const auto found = std::find(keys.begin(), keys.end(), key);
                if (found == keys.end()) return false;
                const auto index = static_cast<std::size_t>(found - keys.begin());
                if (parsed.seen & (1U << index)) return false;
                parsed.seen |= 1U << index;
                parsed.hashes[index] = value;
                return parsed.hashes[index].size() == 64 &&
                    std::all_of(parsed.hashes[index].begin(), parsed.hashes[index].end(),
                                [](char digit) { return (digit >= '0' && digit <= '9') || (digit >= 'a' && digit <= 'f'); });
            };
            std::array<char, 2049> manifest{};
            std::size_t length = 0;
            if (!ini_read_file((root / "bundle.ini").c_str(), manifest.data(), manifest.size(), &length) ||
                !ini_parse(manifest.data(), length, identify, &identities) || identities.seen != 7)
                throw std::runtime_error("Invalid conditioning cache bundle identity");
            const std::array<std::filesystem::path, 3> identified{{source, root / "encoder.ini", root / "denoiser.ini"}};
            for (std::size_t index = 0; index < identified.size(); ++index) {
                std::array<char, 65> digest{};
                if (!inference_file_sha256(identified[index].c_str(), digest.data()) || digest.data() != identities.hashes[index])
                    throw std::runtime_error("Conditioning cache source/descriptor hash mismatch");
            }
            if (!inference_model_read(&encoder_model, (root / "encoder.ini").c_str(), (root / "encoder.onnx").c_str(), error.data(), error.size()) ||
                !inference_model_read(&denoiser_model, (root / "denoiser.ini").c_str(), (root / "denoiser.onnx").c_str(), error.data(), error.size()))
                throw std::runtime_error(error.data());
            if (std::string(encoder_model.adapter) != "dipla_ddim_cache_v1" ||
                std::string(denoiser_model.adapter) != "dipla_ddim_cache_v1" || encoder_model.input_count != 3 ||
                !encoder_model.output_count || encoder_model.output_count > ONNX_TENSOR_LIMIT - 2 ||
                denoiser_model.input_count != encoder_model.output_count + 2 || denoiser_model.output_count != 1)
                throw std::runtime_error("Unsupported conditioning cache tensor contract");
            const auto bind = [](onnx_tensor& target, const onnx_tensor& actual) {
                if (target.type != actual.type || target.rank != actual.rank || target.bytes != actual.bytes)
                    throw std::runtime_error("Conditioning cache tensor mismatch");
                for (std::size_t axis = 0; axis < target.rank; ++axis)
                    if (target.dimensions[axis] != actual.dimensions[axis])
                        throw std::runtime_error("Conditioning cache shape mismatch");
                target.data = actual.data;
            };
            for (std::size_t index = 0; index < encoder_model.input_count; ++index) {
                auto& target = encoder_model.inputs[index];
                const onnx_tensor* actual = nullptr;
                for (std::size_t candidate = 2; candidate < 5; ++candidate)
                    if (std::string(target.name) == inputs[candidate].name) actual = &inputs[candidate];
                if (!actual) throw std::runtime_error("Unknown conditioning input");
                bind(target, *actual);
            }
            std::size_t bytes = 0;
            for (std::size_t index = 0; index < encoder_model.output_count; ++index) {
                auto& output = encoder_model.outputs[index];
                if (output.type != ONNX_FLOAT32 || output.bytes > 4U * 1024U * 1024U - bytes)
                    throw std::runtime_error("Conditioning cache exceeds 4 MiB float32 budget");
                bytes += output.bytes;
                values[index].resize(output.bytes / sizeof(float));
                output.data = values[index].data();
            }
            for (std::size_t index = 0; index < denoiser_model.input_count; ++index) {
                auto& target = denoiser_model.inputs[index];
                const onnx_tensor* actual = nullptr;
                for (std::size_t candidate = 0; candidate < 2; ++candidate)
                    if (std::string(target.name) == inputs[candidate].name) actual = &inputs[candidate];
                for (std::size_t candidate = 0; candidate < encoder_model.output_count; ++candidate)
                    if (std::string(target.name) == encoder_model.outputs[candidate].name) actual = &encoder_model.outputs[candidate];
                if (!actual) throw std::runtime_error("Unknown cached denoiser input");
                bind(target, *actual);
            }
            if (std::string(denoiser_model.outputs[0].name) != outputs[0].name)
                throw std::runtime_error("Unexpected cached denoiser output");
            bind(denoiser_model.outputs[0], outputs[0]);
            if (!onnx_session_open(&encoder, (root / "encoder.onnx").c_str(), encoder_model.inputs, encoder_model.input_count,
                                   encoder_model.outputs, encoder_model.output_count, &options, error.data(), error.size()))
                throw std::runtime_error(error.data());
            if (!onnx_session_open(&denoiser, (root / "denoiser.onnx").c_str(), denoiser_model.inputs, denoiser_model.input_count,
                                   denoiser_model.outputs, denoiser_model.output_count, &options, error.data(), error.size())) {
                onnx_session_close(&encoder);
                throw std::runtime_error(error.data());
            }
        }

        /** @brief Release sessions before their borrowed tensor storage is destroyed. */
        ~ConditioningCache() {
            onnx_session_close(&denoiser);
            onnx_session_close(&encoder);
        }
    };
    std::unique_ptr<ConditioningCache> conditioning_cache;
    /** Shared session borrowing this state's fixed buffers. */
    onnx_session session{};
    /** Whole-plan cooperative deadline; zero disables the local timer. */
    unsigned timeout_ms = 0;
    /** Float32 graph input: eight X values, then eight Y, then eight Z, not interleaved XYZ. */
    std::array<float, DIPLA_COORDINATE_COUNT * points> candidate{};
    /** Noise prediction written by the ONNX graph for each reverse step. */
    std::array<float, DIPLA_COORDINATE_COUNT * points> predicted{};
    /** NCHW map input: mean, variance, marker; each channel uses row * 51 + column. */
    std::array<float, DIPLA_MAP_CHANNEL_COUNT * cells> maps{};
    /** Normalized current pose tensor, ordered east/north/altitude. */
    std::array<float, 3> position{};
    /** Normalized scalar total-variance tensor. */
    std::array<float, 1> total{};
    /** Current reverse-diffusion step. */
    std::array<int64_t, 1> timestep{};
    /** Trained cumulative-alpha schedule used by the deterministic DDIM update. */
    std::array<double, DIPLA_DIFFUSION_STEPS> alpha{};
    /** Center/scale pairs for mean, variance, and log1p(total variance), in that order. */
    Normalization mean_normalization, variance_normalization, total_normalization;

    /**
      * @brief Validate the export contract before binding a reusable ONNX session.
      * @details Reject mismatched geometry, schedules, and tensors at startup,
      * when there is still a useful file error to report, rather than after a
      * plausible-looking but mis-scaled route reaches the application.
      * @param[in] model ONNX graph path.
      * @param[in] metadata Deployment JSON describing training-time preprocessing.
      * @param[in] policy Optional integrity descriptor, thread count, and deadline policy.
      * @pre initialize_runtime() has successfully selected a compatible runtime.
      * @throws std::exception If reading, allocation, validation, or session creation fails.
      * @note Parser views are temporary. Copy only checked numeric values into
      * members; nothing in the session may borrow the constructor's JSON arena.
     */
    State(const std::string& model, const std::string& metadata, const NetworkOptions& policy)
        : timeout_ms(policy.timeout_ms) {
        std::ifstream input(metadata, std::ios::binary);
        if (!input) throw std::runtime_error("Cannot open metadata: " + metadata);
        // Read one byte beyond the accepted 16 KiB to detect oversize input
        // without loading an arbitrary file. The arena separately bounds the
        // decoded tree; a small JSON file can still contain many tiny nodes.
        std::array<char, metadata_byte_limit + 1> contents{};
        input.read(contents.data(), contents.size());
        if (input.bad()) throw std::runtime_error("Cannot read metadata: " + metadata);
        const auto size = static_cast<std::size_t>(input.gcount());
        if (size > metadata_byte_limit) throw std::runtime_error("Metadata too large");
        // JSON nodes need proper alignment, not merely enough bytes. The
        // module builds JSONbourne with a separate 16-container depth limit.
        alignas(std::max_align_t) std::array<unsigned char, metadata_arena_bytes> storage;
        jsonbourne_read_err parsed{};
        const auto* tree = jsonbourne_read_buf(contents.data(), size, JSONBOURNE_READ_NOFLAG,
                                           nullptr, storage.data(), storage.size(), nullptr, &parsed);
        if (!tree && parsed.code == JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION)
            throw std::runtime_error("Metadata JSON exceeds 64 KiB storage limit");
        if (!tree && parsed.code == JSONBOURNE_READ_ERROR_DEPTH)
            throw std::runtime_error("Metadata JSON exceeds 16-level nesting limit");
        if (!tree) throw std::runtime_error("Invalid metadata JSON at byte " + std::to_string(parsed.pos));
        const auto& world = metadata_member(*tree, "world");
        const auto& diffusion = metadata_member(*tree, "diffusion");
        /** @todo Centralize the supported world and schedule contract. Changing
         * these checks alone is not model support: update the public sizes,
         * tensor shapes, grid indexing, sampler loop, and coordinate transforms
         * together, then rerun the reference cases. Until then, reject drift. */
        if (metadata_integer(world, "grid") != DIPLA_GRID_SIDE || finite_number(world, "xy_scale_m") != DIPLA_FIELD_EXTENT_M ||
            finite_number(world, "z_min_m") != DIPLA_MODEL_Z_MIN_M || finite_number(world, "z_max_m") != DIPLA_MODEL_Z_MAX_M ||
            metadata_integer(diffusion, "T") != DIPLA_DIFFUSION_STEPS || metadata_integer(diffusion, "num_waypoints") != points)
            throw std::runtime_error("Unsupported model world/schedule contract");
        // Centers may be signed, but every scale must be positive: a zero
        // divisor breaks normalization and a negative one reverses its meaning.
        struct Transform { const char* center; const char* scale; Normalization& destination; };
        const Transform transforms[] = {
            {"mean_center", "mean_scale", mean_normalization},
            {"var_center", "var_scale", variance_normalization},
            {"total_variance_center", "total_variance_scale", total_normalization}
        };
        const auto& scales = metadata_member(*tree, "normalization");
        for (const auto& transform : transforms) {
            transform.destination.center = finite_number(scales, transform.center);
            transform.destination.scale = finite_number(scales, transform.scale);
            if (transform.destination.scale <= 0) throw std::runtime_error("Invalid normalization scale");
        }
        const auto& schedule_node = metadata_member(diffusion, "alphas_cumprod");
        // The denoiser predicts noise for this schedule; changing or reordering
        // alpha levels is an algorithm change, not an interchangeable setting.
        if (schedule_node.type != JSONBOURNE_TYPE_ARR) throw std::runtime_error("Expected diffusion schedule array");
        const auto& schedule = *static_cast<const jsonbourne_arr*>(schedule_node.ptr);
        if (schedule.len != alpha.size())
            throw std::runtime_error("Incomplete diffusion schedule");
        double previous = 1.0;
        const auto* entry = schedule.first;
        for (std::size_t index = 0; index < alpha.size(); ++index) {
            const double value = metadata_number(*entry->val);
            if (!std::isfinite(value) || value <= 0 || value >= previous)
                throw std::runtime_error("Invalid diffusion schedule");
            alpha[index] = value;
            previous = value;
            entry = entry->next;
        }
        // Names, element types, ranks, and axis order are the export ABI. A
        // correctly sized buffer is not enough if X and waypoint axes swap.
        const onnx_tensor inputs[] = {
            {"x", ONNX_FLOAT32, 3, {1,DIPLA_COORDINATE_COUNT,points}, candidate.data(), sizeof(candidate)},
            {"t", ONNX_INT64, 1, {1}, timestep.data(), sizeof(timestep)},
            /** @todo Derive this shape from the same supported-grid contract
             * as maps and Belief; do not independently make only the binding dynamic. */
            {"meanvarmarker_map", ONNX_FLOAT32, 4, {1,DIPLA_MAP_CHANNEL_COUNT,DIPLA_GRID_SIDE,DIPLA_GRID_SIDE}, maps.data(), sizeof(maps)},
            {"current_position", ONNX_FLOAT32, 2, {1,DIPLA_COORDINATE_COUNT}, position.data(), sizeof(position)},
            {"total_variance_condition", ONNX_FLOAT32, 2, {1,1}, total.data(), sizeof(total)}
        };
        const onnx_tensor outputs[] = {
            {"noise_pred", ONNX_FLOAT32, 3, {1,DIPLA_COORDINATE_COUNT,points}, predicted.data(), sizeof(predicted)}
        };
        std::array<char, runtime_error_capacity> error{};
        if (!policy.descriptor.empty()) {
            // JSON describes the maths; the optional descriptor also binds
            // graph bytes and the adapter/tensor ABI. Check both so a swapped
            // export cannot borrow a familiar filename and pass unnoticed.
            // Integrity checking is not authentication of the model's author.
            inference_model bundle{};
            if (!inference_model_read(&bundle, policy.descriptor.c_str(), model.c_str(), error.data(), error.size()))
                throw std::runtime_error(error.data());
            if (std::string(bundle.adapter) != "dipla_ddim_v1" || bundle.input_count != std::size(inputs) || bundle.output_count != std::size(outputs))
                throw std::runtime_error("Unsupported Diplacam model descriptor adapter");
            const auto matches = [](const onnx_tensor& declared, const onnx_tensor& expected) {
                if (std::string(declared.name) != expected.name || declared.type != expected.type ||
                    declared.rank != expected.rank || declared.bytes != expected.bytes) return false;
                for (std::size_t axis = 0; axis < declared.rank; ++axis)
                    if (declared.dimensions[axis] != expected.dimensions[axis]) return false;
                return true;
            };
            for (std::size_t index = 0; index < std::size(inputs); ++index)
                if (!matches(bundle.inputs[index], inputs[index])) throw std::runtime_error("Diplacam descriptor input mismatch");
            if (!matches(bundle.outputs[0], outputs[0])) throw std::runtime_error("Diplacam descriptor output mismatch");
        }
        const onnx_session_options options{policy.threads, nullptr};
        if (!policy.conditioning_cache.empty()) {
            conditioning_cache = std::make_unique<ConditioningCache>(policy.conditioning_cache, model, inputs, outputs, options);
        } else if (!onnx_session_open(&session, model.c_str(), inputs, std::size(inputs), outputs, std::size(outputs), &options,
                               error.data(), error.size())) throw std::runtime_error(error.data());
    }

    /**
     * @brief Close the session while every borrowed member buffer is still alive.
     * @details Destructor bodies run before member destruction. Explicit closure
     * here preserves that order even though session is declared before its buffers.
     */
    ~State() {
        conditioning_cache.reset();
        onnx_session_close(&session);
    }
};

/**
 * @details Build the entire private state before exposing a usable Network.
 * The pimpl keeps runtime and tensor implementation types out of the public API.
 */
Network::Network(const std::string& model, const std::string& metadata, const NetworkOptions& options)
    : state_(std::make_unique<State>(model, metadata, options)) {}
/** @details Destroy State through its owner so the session closes before its buffers disappear. */
Network::~Network() = default;

/**
 * @details
 * This is the complete sampler, not one neural-network pass. Conditioning
 * inputs stay fixed while twenty noise predictions reconstruct one route.
 * The caller supplies initial noise so numerical tests can compare identical
 * inputs without depending on C++ and NumPy generating identical random streams.
 *
 * @par Precision matters
 * Match the reference's float32 graph boundary and double-precision DDIM
 * arithmetic. Moving casts or simplifying apparently equivalent expressions
 * can change the trajectory over twenty steps; check reference parity first.
 *
 * @note The deadline is cooperative and covers the whole plan. It is not a
 * hard real-time guarantee or a fresh allowance for each network call.
 * @see apply_flight_margin Application-owned XY padding, intentionally absent here.
 */
Waypoints Network::sample(const Belief& belief, const std::array<double, 3>& position, const Noise& noise) {
    auto& state = *state_;
    // One absolute budget covers preprocessing and every reverse step. Resetting
    // it inside the loop would permit a much longer plan than the owner requested.
    const std::uint64_t deadline = state.timeout_ms ? onnx_monotonic_us() + std::uint64_t(state.timeout_ms) * 1000 : 0;
    for (double value : position) if (!std::isfinite(value)) throw std::runtime_error("Nonfinite position");
    if (!inside_field(position[0], position[1]))
        throw std::runtime_error("Position outside field");
    // The third map channel is a one-hot pose marker; clearing all channels
    // first prevents a previous sample's marker leaking into this inference.
    std::fill(state.maps.begin(), state.maps.end(), 0.0f);
    double total = 0;
    // The model sees normalized float32 cell channels, but total variance is
    // summed from the original doubles before its separate log1p transform.
    // Preserve this boundary to match the reference, not just its algebra.
    for (std::size_t cell = 0; cell < cells; ++cell) {
        if (!std::isfinite(belief.mean[cell]) || !std::isfinite(belief.variance[cell]) || belief.variance[cell] < 0)
            throw std::runtime_error("Invalid belief cell");
        state.maps[cell] = (static_cast<float>(belief.mean[cell]) - static_cast<float>(state.mean_normalization.center)) /
                   static_cast<float>(state.mean_normalization.scale);
        state.maps[cells + cell] = (static_cast<float>(belief.variance[cell]) - static_cast<float>(state.variance_normalization.center)) /
                       static_cast<float>(state.variance_normalization.scale);
        total += belief.variance[cell];
        if (!std::isfinite(state.maps[cell]) || !std::isfinite(state.maps[cells + cell]))
            throw std::runtime_error("Belief normalization overflow");
    }
    // East selects a column and north a row: the marker is [y, x], not [x, y].
    // nearbyint matches NumPy's ties-to-even rounding under the default FP
    // rounding mode; clamping keeps boundary poses inside the 51-by-51 map.
    const auto column = static_cast<std::size_t>(std::clamp(std::nearbyint(position[0] / DIPLA_GRID_STEP_M), 0.0, double(DIPLA_GRID_SIDE - 1U)));
    const auto row = static_cast<std::size_t>(std::clamp(std::nearbyint(position[1] / DIPLA_GRID_STEP_M), 0.0, double(DIPLA_GRID_SIDE - 1U)));
    state.maps[2 * cells + cell_index(row, column)] = 1;
    // Pass actual altitude, even in the 2 m demo outside the trained 10..40 m
    // band. Pretending to be at 10 m would misdescribe the aircraft to the model;
    // preserving this input does not establish model quality at low altitude.
    /** @todo Share XY and Z normalization bounds with metadata validation and
     * output denormalization. These are training scales, not flight-height settings. */
    state.position = {{static_cast<float>(position[0] / xy_half_extent_m - 1.0), static_cast<float>(position[1] / xy_half_extent_m - 1.0),
                       static_cast<float>((position[2] - DIPLA_MODEL_Z_MIN_M) / model_z_half_range_m - 1.0)}};
    state.total[0] = static_cast<float>((std::log1p(total) - state.total_normalization.center) / state.total_normalization.scale);
    if (!std::isfinite(state.total[0])) throw std::runtime_error("Variance normalization overflow");
    if (state.conditioning_cache) {
        std::array<char, 1024> error{};
        if (!onnx_session_run(&state.conditioning_cache->encoder, deadline, error.data(), error.size()))
            throw std::runtime_error(error.data());
    }
    Noise current = noise;
    for (double value : current) if (!std::isfinite(value)) throw std::runtime_error("Nonfinite noise");
    // The graph predicts noise, not clean coordinates. Each reverse DDIM step
    // estimates x0, clamps that estimate to the trained normalized range, then
    // advances to the previous cumulative-alpha level without adding new noise
    // (DDIM eta = 0). Clipping x0 is part of the reference algorithm, not the
    // flight margin; removing it changes all subsequent reverse steps.
    for (int step = static_cast<int>(state.alpha.size()) - 1; step >= 0; --step) {
        for (std::size_t coordinate = 0; coordinate < current.size(); ++coordinate) {
            state.candidate[coordinate] = static_cast<float>(current[coordinate]);
            if (!std::isfinite(state.candidate[coordinate])) throw std::runtime_error("Sampler overflow");
        }
        state.timestep[0] = step;
        std::array<char, 1024> error{};
        auto* session = state.conditioning_cache ? &state.conditioning_cache->denoiser : &state.session;
        if (!onnx_session_run(session, deadline, error.data(), error.size()))
            throw std::runtime_error(error.data());
        const double alpha = state.alpha[static_cast<std::size_t>(step)];
        for (std::size_t coordinate = 0; coordinate < current.size(); ++coordinate) {
            const double predicted = state.predicted[coordinate];
            if (!std::isfinite(predicted)) throw std::runtime_error("Nonfinite network output");
            const double clean = std::clamp((current[coordinate] - std::sqrt(1.0 - alpha) * predicted) /
                                             std::sqrt(alpha), -1.0, 1.0);
            // Step zero has no earlier alpha level: the clean estimate is
            // already the result. Do not index alpha[-1] for a final update.
            current[coordinate] = step == 0 ? clean :
                std::sqrt(state.alpha[static_cast<std::size_t>(step - 1)]) * clean +
                std::sqrt(1.0 - state.alpha[static_cast<std::size_t>(step - 1)]) * predicted;
        }
    }
    Waypoints result{};
    for (std::size_t waypoint = 0; waypoint < points; ++waypoint) {
        // Repack axis-major model output into waypoint-major application XYZ.
        // Keep raw XY in 0..100 m and model Z in 10..40 m for reference parity;
        // the publication boundary chooses flight XY padding and fixed height.
        /** @todo Derive these inverse transforms from the same supported-world
         * contract as input normalization. Changing just one direction rescales routes. */
        result[waypoint] = {{(current[waypoint] + 1.0) * xy_half_extent_m,
                     (current[points + waypoint] + 1.0) * xy_half_extent_m,
                     (current[2 * points + waypoint] + 1.0) * model_z_half_range_m + DIPLA_MODEL_Z_MIN_M}};
    }
    return result;
}

void ExperimentOptions::validate() const {
    if (flight_margin_cells * 2ULL >= DIPLA_GRID_SIDE - 1U ||
        !std::isfinite(gp_prior_mean) || !std::isfinite(gp_kernel_variance) || gp_kernel_variance <= 0 ||
        !std::isfinite(gp_lengthscale_m) || gp_lengthscale_m <= 0 ||
        !std::isfinite(sensor_fov_deg) || sensor_fov_deg <= 0 || sensor_fov_deg >= 180 ||
        !std::isfinite(sensor_noise_variance) || sensor_noise_variance <= 0)
        throw std::runtime_error("Invalid Diplacam experiment settings");
}

void CmaesOptions::validate() const {
    if (budget_ms < 1 || budget_ms > max_budget_ms || maxfevals < population_size ||
        maxfevals > max_evaluations || seed < 1 || !std::isfinite(utility_threshold) ||
        !std::isfinite(confidence_beta) || confidence_beta < 0 ||
        !std::isfinite(step_xy_m) || step_xy_m <= 0)
        throw std::runtime_error("Invalid CMA-ES comparison settings");
}

/**
 * @details Both planners pass here once before publication. The inset is an
 * application policy, not model normalization or a change to the raw sampler.
 * Validate the entire route, including Z, before changing any coordinate.
 * @see Network::sample
 */
void apply_flight_margin(Waypoints& route, unsigned margin_cells) {
    if (margin_cells * 2ULL >= DIPLA_GRID_SIDE - 1U) throw std::runtime_error("Invalid flight margin");
    const double inset = margin_cells * DIPLA_GRID_STEP_M;
    for (const auto& point : route)
        for (double coordinate : point)
            if (!std::isfinite(coordinate)) throw std::runtime_error("Nonfinite flight waypoint");
    for (auto& point : route) {
        point[0] = std::clamp(point[0], inset, DIPLA_FIELD_EXTENT_M - inset);
        point[1] = std::clamp(point[1], inset, DIPLA_FIELD_EXTENT_M - inset);
    }
}

/**
 * @brief Private storage for the synthetic field and its GP posterior.
 * @details Keep truth beside the synthetic observation source, not in the
 * network's inputs: the planner should learn through measurements, not peek at
 * the answer sheet. The mean and full covariance evolve together in place.
 * @invariant Only accepted increasing observation numbers advance replay state.
 * @warning The covariance is dense. Additional copies are substantial allocations
 * on MORA's shared 512 MB board, even when the public Belief stays small.
 */
struct Survey::State {
    /** Immutable validated experiment snapshot; never reloaded mid-survey. */
    const ExperimentOptions options;
    /** Fixed 51-by-51 grid geometry. */
    aipp::Grid grid{0.0, DIPLA_FIELD_EXTENT_M, 0.0, DIPLA_FIELD_EXTENT_M, DIPLA_GRID_STEP_M};
    /** Dense prior/posterior covariance; one matrix is about 51.6 MiB. */
    aipp::Matrix covariance;
    /** Configured prior/posterior mean, independent of the comparison threshold. */
    aipp::Vector mean;
    /** Private synthetic ground truth used only by the observation operator. */
    aipp::Vector truth{aipp::Vector::Zero(cells)};
    /** Highest accepted observation identifier. */
    std::uint32_t last_number = 0;
    /** Independent configured sensor stream, never advanced by diffusion sampling. */
    std::mt19937_64 sensor_generator;
    /** FNV-1a over data-row bytes with LF separators; excludes the CSV header. */
    std::uint64_t digest = fnv_offset_basis;
    /**
      * @brief Load the complete synthetic lattice without inventing missing truth.
      * @details Numeric rows outside the deployed even-coordinate grid are
      * ignored after parsing. Selected cells must appear exactly once; silently
      * averaging duplicates or filling holes would change the experiment.
      * @param[in] path CSV with complete coverage of the selected 51-by-51 grid.
    * @param[in] experiment Immutable GP and synthetic sensor policy.
      * @throws std::exception For file-system, allocation, reading, or data-validation failures.
      * @note The 2 MiB file and 256-character row limits bound accepted input,
      * not the dense covariance or every transient allocation made by streams.
     */
    explicit State(const std::string& path, const ExperimentOptions& experiment)
        : options(experiment), sensor_generator(options.sensor_noise_seed) {
        options.validate();
        if (std::filesystem::file_size(path) > field_byte_limit) throw std::runtime_error("Field CSV too large");
        std::ifstream input(path);
        std::string line;
        if (!std::getline(input, line) || (line != "x,y,count" && line != "x,y,count\r"))
            throw std::runtime_error("Field must be numeric x,y,count CSV, not an LFS pointer");
        std::array<bool, cells> seen{};
        std::size_t rows = 0;
        while (std::getline(input, line)) {
            if (line.size() > field_row_byte_limit) throw std::runtime_error("Field CSV row too long");
            // Hash before replacing commas, including ignored rows and any CR.
            // getline removes LF, so restore a logical LF for every data row.
            // This is a provenance aid, not a byte-exact whole-file hash: the
            // header is excluded and a missing final LF is normalized away.
            for (unsigned char byte : line) { digest ^= byte; digest *= fnv_prime; }
            digest ^= '\n'; digest *= fnv_prime;
            if (line.empty() || line == "\r") continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream values(line);
            double east = 0, north = 0, count = 0;
            std::string trailing;
            if (!(values >> east >> north >> count) || (values >> trailing) ||
                !std::isfinite(east) || !std::isfinite(north) || !std::isfinite(count))
                throw std::runtime_error("Invalid field CSV row");
            // Research maps may contain a finer one-metre grid; only exact
            // even-coordinate samples belong to the deployed two-metre lattice.
            if (!inside_field(east, north) ||
                std::abs(std::remainder(east, DIPLA_GRID_STEP_M)) > field_alignment_tolerance_m ||
                std::abs(std::remainder(north, DIPLA_GRID_STEP_M)) > field_alignment_tolerance_m) continue;
            // Match Belief and the network maps: north is the row, east the
            // column. Transposing here gives a valid-looking but rotated survey.
            const auto cell = cell_index(static_cast<std::size_t>(std::llround(north / DIPLA_GRID_STEP_M)),
                                         static_cast<std::size_t>(std::llround(east / DIPLA_GRID_STEP_M)));
            if (seen[cell]) throw std::runtime_error("Duplicate field grid cell");
            seen[cell] = true;
            truth[static_cast<int>(cell)] = count;
            ++rows;
        }
        if (!input.eof() || rows != cells) throw std::runtime_error("Field must cover all 2601 grid cells");
        covariance = aipp::initialize_covariance(grid, options.gp_kernel_variance, options.gp_lengthscale_m);
        mean = aipp::Vector::Constant(cells, options.gp_prior_mean);
    }
};

/**
 * @details A new Survey starts a fresh belief and sensor random stream. Keep
 * the same instance across route chunks; reconstruct only for a new survey,
 * otherwise previously acquired knowledge disappears along with the covariance.
 */
Survey::Survey(const std::string& field, const ExperimentOptions& options)
    : state_(std::make_unique<State>(field, options)) {}
/** @details Release private truth and the dense posterior together through their owning State. */
Survey::~Survey() = default;

/**
 * @details Measurement generation and GP conditioning must describe the same
 * sensor: observe the footprint, add Gaussian noise with its declared variance,
 * and use that variance as the update's likelihood. Adding noise only to the
 * belief, or conditioning on exact truth, would be a different experiment.
 * @note The default remains the original 2 m experiment. Resized worker frames
 * supply the same model-domain height used for network pose conditioning.
 * @warning Updates are in place, not transactional. A GP arithmetic failure
 * throws; the owner must treat that survey as failed, not retry as if untouched.
 * @see Survey::belief
 */
bool Survey::observe(double east, double north, std::uint32_t number, double height) {
    auto& state = *state_;
    // A retried pose is not another independent sensor reading: conditioning
    // twice would create false certainty even though the field value is unchanged.
    // Reject before drawing noise as well: retries must not shift the sensor
    // random stream and thereby change every later accepted measurement.
    if (!number || number <= state.last_number) return false;
    if (!inside_field(east, north) || !std::isfinite(height) || height <= 0 || height > DIPLA_MODEL_Z_MAX_M)
        throw std::runtime_error("Observation outside field");
    const auto sensor = aipp::build_sensor(state.grid, {east, north, height},
                                         state.options.sensor_fov_deg, state.options.sensor_noise_variance);
    // The research altitude-noise formula is not extrapolated below its training
    // band, where it would become negative at the 2 m demo. The configured
    // positive variance drives both sampled noise and GP likelihood.
    auto measured = aipp::observe(sensor, state.truth);
    const auto sensor_noise = aipp::sample_sensor_noise(sensor, state.sensor_generator);
    for (int index = 0; index < measured.size(); ++index) measured[index] += sensor_noise[index];
    // Truth is consumed only through the footprint operator; planning receives
    // the posterior rather than privileged access to unmeasured field cells.
    aipp::condition(state.covariance, &state.mean, sensor, &measured);
    if (!state.mean.allFinite() || !state.covariance.allFinite()) throw std::runtime_error("GP update failed");
    // Consume the identifier only after a successful finite update. Even an
    // empty footprint is consumed, so retrying it cannot later become new evidence.
    state.last_number = number;
    return !sensor.rows.empty();
}

/*
 * This is an explicitly selected alternative to diffusion, never an
 * automatic rescue for a failed neural plan. Use the same configured sensor
 * and fixed 2 m height so the comparison changes the planner,
 * not the measurement model. The supplied altitude is checked for compatibility
 * before planning at exactly 2 m.
 *
 * The underlying greedy search copies covariance for hypothetical updates;
 * the observed posterior must remain untouched. Budget that extra dense matrix.
 * A completed greedy seed may survive a refinement deadline; an incomplete seed
 * or caller cancellation must not become a partial published route.
 *
 * Deadline and cancellation are cooperative polling, not hard preemption.
 * Serialize this call with observations on the same Survey, despite its const API.
 * Pass successful results through apply_flight_margin before publication.
 */
std::optional<Waypoints> Survey::compare(const std::array<double, 3>& position,
                                       const CmaesOptions& comparison,
                                       const std::function<bool()>& cancelled) const {
    for (double value : position) if (!std::isfinite(value)) throw std::runtime_error("Nonfinite comparison position");
    comparison.validate();
    const double height_m = DIPLA_DEMO_HEIGHT_MM / DIPLA_MM_PER_M;
    if (position[0] < 0 || position[0] > DIPLA_FIELD_EXTENT_M ||
        position[1] < 0 || position[1] > DIPLA_FIELD_EXTENT_M ||
        std::abs(position[2] - height_m) > DIPLA_HEIGHT_TOLERANCE_M)
        throw std::runtime_error("Invalid CMA-ES comparison position");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(comparison.budget_ms);
    aipp::PlannerOptions options;
    options.zmin = options.zmax = height_m;
    options.observation_variance = state_->options.sensor_noise_variance;
    options.angle_of_view = state_->options.sensor_fov_deg;
    options.margin_cells = state_->options.flight_margin_cells;
    options.horizon = static_cast<int>(points);
    options.popsize = CmaesOptions::population_size;
    options.threshold = comparison.utility_threshold;
    options.beta = comparison.confidence_beta;
    options.step_xy = comparison.step_xy_m;
    options.maxfevals = comparison.maxfevals;
    // Round up to enough population generations to use the evaluation allowance;
    // the evaluation cap remains authoritative, rather than rounding the budget down.
    options.maxiter = (comparison.maxfevals + options.popsize - 1) / options.popsize;
    options.seed = comparison.seed;
    // The library optimizer is cooperative. This closure gives both its
    // deadline and caller cancellation the same inexpensive polling point.
    options.keep_planning = [&] {
        return std::chrono::steady_clock::now() < deadline && (!cancelled || !cancelled());
    };
    const auto controls = aipp::plan(state_->grid, state_->mean, state_->covariance,
                                    {position[0], position[1], height_m}, options);
    if (controls.size() != points || (cancelled && cancelled())) return std::nullopt;
    Waypoints route{};
    for (std::size_t slot = 0; slot < points; ++slot) {
        if (!controls[slot].allFinite()) throw std::runtime_error("Nonfinite CMA-ES waypoint");
        route[slot] = {{controls[slot].x(), controls[slot].y(), height_m}};
    }
    return route;
}

/**
 * @details Publish a value snapshot, not aliases into the mutable GP. The
 * denoiser needs per-cell means and marginal variances, whereas conditioning
 * needs the correlations in the full covariance; keep those responsibilities
 * separate rather than expanding every model request into a matrix copy.
 * @note Clamp negative diagonal round-off only in the exported summary. This
 * does not repair the internal covariance or validate its positive semidefiniteness.
 */
Belief Survey::belief() const {
    Belief result;
    // Publish only marginal variance. Copying dense covariance for every request
    // would move about 51.6 MiB and expose mutable GP internals unnecessarily.
    for (std::size_t cell = 0; cell < cells; ++cell) {
        result.mean[cell] = state_->mean[static_cast<int>(cell)];
        result.variance[cell] = std::max(0.0, state_->covariance(static_cast<int>(cell), static_cast<int>(cell)));
    }
    return result;
}

/**
 * @details The trace sums uncertainty over cells, not the uncertainty of their
 * sum (which would include cross-covariances). Keep it consistent with the
 * model's total-variance conditioning; it is not a calibrated route-quality score.
 * @note This diagnostic reads the raw trace, unlike belief()'s clipped marginals.
 */
double Survey::total_variance() const { return state_->covariance.trace(); }

/**
 * @details Identify the loaded data rows without exposing the hidden field.
 * FNV-1a is a lightweight diagnostic fingerprint, not collision-resistant
 * integrity checking or authentication. See State's loader for newline handling.
 */
std::uint64_t Survey::field_digest() const { return state_->digest; }
}
