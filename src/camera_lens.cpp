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
 * @file camera_lens.cpp
 * @brief Bounded native lens and exposure correction for USHER JPEGs.
 * @details Validates a versioned profile, reads the clean source JPEG and
 * flight EXIF, renders in memory-bounded strips, and atomically publishes a
 * derivative with provenance. Default profiles are illustrative, not measured
 * calibration.
 */

// clang-format off
#include <stddef.h>
#include <stdio.h>
#include <jpeglib.h>
#include "jpeg_marker_guard.h"
// clang-format on
#include "exifite.h"
#include "image_exif.h"
extern "C" {
#include "sha256fst.h"
}

#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "version.h"
#include "opencv_json.hpp"
#include "opencv_parallel.hpp"

#if defined(__GNUC__) && !defined(__clang__) && defined(__x86_64__)
#define CAMERA_LENS_SIMD __attribute__((target_clones("avx2,fma,default")))
#else
#define CAMERA_LENS_SIMD
#endif

/** Internal camera/lens implementation, kept together so the projection path is
 * navigable. */
namespace camera_lens_detail {
using Json = usher_cv_json::Value;
/** Angle conversion shared by pose and calibration calculations. */
constexpr double pi = 3.14159265358979323846;
/** Binary-size unit used by file and memory bounds. */
constexpr size_t mib = 1024 * 1024;
/** JPEG APP1 segment data maximum (the 16-bit marker length reserves two
 * bytes). */
constexpr size_t maximum_exif_payload_bytes = 65533;
/** Reject pathological dimensions before allocating decoder or OpenCV buffers.
 */
constexpr unsigned int maximum_image_dimension = 4096;
/** MIMIC fixture guard: 8 megapixels bounds the full-frame software path. */
constexpr uint64_t maximum_mock_pixels = 8388608;
/** Fixed cache residency avoids turning tile access order into unbounded RAM
 * growth while keeping wide fisheye strip working sets (~35 tiles) resident. */
constexpr size_t tile_cache_entries = 48;
/** Fisheye/Brown inversion must round-trip within a tenth of one pixel. */
constexpr double maximum_reprojection_error_pixels = 0.1;
/** OpenCV's iterative Brown inversion has a hard work ceiling. */
constexpr int lens_inverse_iterations = 30;
/** Forward-model correction passes and target after OpenCV inversion. */
constexpr int lens_polish_passes = 8;
constexpr double lens_polish_tolerance_px = 0.001;
/** First exact-inverse node spacing; halved until interpolation is verified. */
constexpr int ray_grid_initial_step = 16;
/** Maximum verified ray interpolation error, in output pixels. */
constexpr double ray_interpolation_tolerance_px = 0.02;
/** Maximum ground motion between neighbouring exposure samples, in pixels. */
constexpr double temporal_sample_spacing_px = 0.5;
struct FileCloser {
  /** @brief Close the stream owned by a File handle.
   * @param file Non-null stream handed off by the unique pointer.
   * @details Scope cleanup cannot report close errors; output paths that need
   * to detect write failures must check them before relinquishing ownership. */
  void operator()(FILE* file) const { fclose(file); }
};
using File = std::unique_ptr<FILE, FileCloser>;

/** @brief Route a failed precondition to the CLI's diagnostic handler.
 * @param valid Condition to enforce.
 * @param message Diagnostic copied into the exception.
 * @throws std::runtime_error If @p valid is false.
 * @details Use exceptions so the job's scoped resources unwind on failure. */
void require(bool valid, const char* message) {
  if (!valid) throw std::runtime_error(message);
}

/** @copydoc require(bool, const char*) */
void require(bool valid, const std::string& message) {
  if (!valid) throw std::runtime_error(message);
}

/** Validate one finite numeric profile/geometry field and return its value.
 * @param value JSON field.
 * @param name Field name used in diagnostics.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @return Validated number. */
double number(const Json& value, const std::string& name, double minimum,
              double maximum) {
  require(value.is_number(), name + " must be numeric");
  const double result = value.get<double>();
  require(std::isfinite(result) && result >= minimum && result <= maximum,
          name + " outside permitted range");
  return result;
}

/** Open only a bounded regular file; O_NONBLOCK prevents special-file stalls.
 * @param path Input path.
 * @param maximum Maximum permitted size in bytes.
 * @return Owning stream, or throws a descriptive error. */
File open_regular(const std::string& path, size_t maximum) {
  const int descriptor = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  require(descriptor >= 0, "cannot open input: " + path);
  struct stat status{};
  if (fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode) ||
      status.st_size < 0 || static_cast<size_t>(status.st_size) > maximum) {
    close(descriptor);
    throw std::runtime_error("input must be a bounded regular file: " + path);
  }
  File input(fdopen(descriptor, "rb"));
  if (!input) {
    close(descriptor);
    throw std::runtime_error("cannot open input stream: " + path);
  }
  return input;
}

/** Fill profile defaults, then validate schema, known keys, types and ranges.
 * @param path USHER_LENS_PROFILE_V1 JSON file.
 * @return Validated profile with all optional defaults materialized. */
Json read_profile(const char* path) {
  File input = open_regular(path, 16384);
  std::array<char, 16385> buffer{};
  const size_t length = fread(buffer.data(), 1, buffer.size(), input.get());
  require(length <= 16384 && !ferror(input.get()),
          "cannot read bounded lens profile");
  const Json values = Json::parse(buffer.begin(), buffer.begin() + length);
  // Defaults keep the worker runnable for examples; they are not optical
  // measurements.
  Json profile = Json::object({{"schema", "USHER_LENS_PROFILE_V1"},
                  {"name", "custom"},
                  {"calibration", "illustrative_not_measured"},
                  {"model", "brown"},
                  {"horizontal_fov_deg", 66.3},
                  {"vertical_fov_deg", 52.3},
                  {"diagonal_fov_deg", 160.0},
                  {"distortion", Json::array({0.0, 0.0, 0.0, 0.0, 0.0})},
                  {"chromatic_aberration", 0.0005},
                  {"exposure_s", 0.002},
                  {"exposure_ev", 0.0},
                  {"f_number", 1.79},
                  {"focal_length_mm", nullptr},
                  {"shutter", "rolling"},
                  {"readout_s", 0.02},
                  {"readout_direction", "top_to_bottom"},
                  {"motion_scale", 1.0},
                  {"angular_rate_deg_s", Json::array({0.0, 0.0, 0.0})},
                  {"vibration_deg", 0.03},
                  {"vibration_hz", 80.0},
                  {"vibration_phase_deg", 0.0},
                  {"samples", 7},
                  {"height_override_m", nullptr},
                  {"max_tiles", 121},
                  {"jpeg_quality", 95},
                  {"threads", 4},
                  {"memory_budget_mb", 192},
                  // Sensor/ISP stage: zero disables each effect, so older
                  // profiles render exactly as before.
                  {"vignetting", 0.0},
                  {"optical_blur_px", 0.0},
                  {"sensor_full_well_e", 0.0},
                  {"sensor_read_noise_e", 0.0},
                  {"isp_sharpen", 0.0}});
  profile.set("output_width", nullptr);
  profile.set("output_height", nullptr);
  require(values.is_object() &&
              values.value("schema", "") == "USHER_LENS_PROFILE_V1",
          "invalid profile schema");
  for (const auto& entry : values.items()) {
    require(profile.contains(entry.first),
            "unknown lens parameter: " + entry.first);
    profile.set(entry.first, entry.second);
  }
  for (const char* key : {"name", "calibration"}) {
    const std::string text = profile[key].get<std::string>();
    require(!text.empty() && text.size() <= 160,
            std::string("invalid ") + key);
  }
  require(profile["model"] == "brown" || profile["model"] == "fisheye",
          "invalid lens model");
  require(profile["shutter"] == "global" || profile["shutter"] == "rolling",
          "invalid shutter");
  require(profile["readout_direction"] == "top_to_bottom" ||
              profile["readout_direction"] == "bottom_to_top",
          "invalid readout direction");
  struct Range {
    const char* key;
    double minimum, maximum;
  };
  // These bounds keep projective math finite and cap exposure/distortion work
  // to supported inputs.
  for (const Range& range :
       std::initializer_list<Range>{{"horizontal_fov_deg", 1, 170},
                                    {"vertical_fov_deg", 1, 170},
                                    {"diagonal_fov_deg", 1, 170},
                                    {"chromatic_aberration", 0, 0.01},
                                    {"exposure_s", 0.000001, 1},
                                    {"exposure_ev", -8, 8},
                                    {"f_number", 0.5, 64},
                                    {"readout_s", 0, 0.2},
                                    {"motion_scale", 0, 10},
                                    {"vibration_deg", 0, 5},
                                    {"vibration_hz", 0, 1000},
                                    {"vibration_phase_deg", -360, 360},
                                    {"vignetting", 0, 0.95},
                                    {"optical_blur_px", 0, 5},
                                    {"sensor_full_well_e", 0, 1000000},
                                    {"sensor_read_noise_e", 0, 100},
                                    {"isp_sharpen", 0, 3}}) {
    profile.set(range.key,
          number(profile[range.key], range.key, range.minimum, range.maximum));
  }
  // Integer limits bound tile fan-out, thread creation, and explicit
  // working-set requests.
  for (const Range& range :
       std::initializer_list<Range>{{"samples", 1, 65},
                                    {"max_tiles", 1, 225},
                                    {"jpeg_quality", 1, 100},
                                    {"threads", 1, 32},
                                    {"memory_budget_mb", 96, 1024}}) {
    number(profile[range.key], range.key, range.minimum, range.maximum);
    require(profile[range.key].is_number_integer(),
            std::string(range.key) + " must be an integer");
  }
  require(profile["samples"].get<int>() % 2 == 1, "samples must be odd");
  require(profile["output_width"].is_null() == profile["output_height"].is_null(),
          "output_width and output_height must be set together");
  if (!profile["output_width"].is_null()) {
    for (const char* key : {"output_width", "output_height"}) {
      number(profile[key], key, 2, maximum_image_dimension);
      require(profile[key].is_number_integer(), std::string(key) + " must be an integer");
    }
    require(profile["output_width"].get<uint64_t>() * profile["output_height"].get<uint64_t>() <=
                maximum_mock_pixels, "output exceeds pixel limit");
  }
  require(
      profile["shutter"] != "rolling" || profile["readout_s"].get<double>() > 0,
      "rolling readout must be positive");
  const double full_well = profile["sensor_full_well_e"].get<double>();
  require(full_well == 0 || full_well >= 10,
          "sensor_full_well_e must be 0 (noise off) or at least 10");
  require(full_well > 0 || profile["sensor_read_noise_e"].get<double>() == 0,
          "sensor_read_noise_e requires sensor_full_well_e");
  if (!profile["height_override_m"].is_null())
    number(profile["height_override_m"], "height_override_m", 0.1, 10000);
  if (!profile["focal_length_mm"].is_null())
    number(profile["focal_length_mm"], "focal_length_mm", 0.1, 1000);
  for (const char* key : {"distortion", "angular_rate_deg_s"}) {
    const bool distortion = std::string(key) == "distortion";
    const size_t count =
        distortion ? (profile["model"] == "fisheye" ? 4 : 5) : 3;
    const double limit = distortion ? 2 : 360;
    require(profile[key].is_array() && profile[key].size() == count,
            std::string("invalid ") + key + " coefficients");
    for (size_t index = 0; index < profile[key].size(); ++index)
      number(profile[key].at(index), key, -limit, limit);
  }
  return profile;
}

/** libjpeg error state with a recovery point so malformed input becomes an
 * exception. */
struct JpegError {
  jpeg_error_mgr manager{};
  jmp_buf recovery;
  void (*emit)(j_common_ptr, int) = nullptr;
};

/** @brief Return a fatal libjpeg error to the active codec recovery point.
 * @param state Codec state carrying the installed JpegError.
 * @details The recovery branch converts the failure to an exception; this
 * C callback itself never unwinds the C++ stack. */
void jpeg_failure(j_common_ptr state) {
  longjmp(reinterpret_cast<JpegError*>(state->err)->recovery, 1);
}

/** @brief Enforce JPEG marker bounds before normal message handling.
 * @param state Codec state carrying the installed JpegError.
 * @param level libjpeg diagnostic level.
 * @details Forward accepted messages to the saved handler so warning
 * accounting remains available to the decoder. */
void jpeg_message(j_common_ptr state, int level) {
  usher_jpeg_validate_marker(state);
  reinterpret_cast<JpegError*>(state->err)->emit(state, level);
}

/** @brief Own decoder state across libjpeg's nonlocal error return. */
struct Decoder {
  jpeg_decompress_struct state{};
  JpegError error{};
  bool created = false;
  cv::Mat image;
  /** @brief Destroy the decoder if creation was attempted.
   * @details The creation flag is armed before the libjpeg call so partial
   * initialization can be cleaned up on the recovery path. */
  ~Decoder() {
    if (created) jpeg_destroy_decompress(&state);
  }
};

/** @brief Encoder facts of a decoded JPEG, used to calibrate profiles. */
struct JpegHeader {
  int width = 0, height = 0;
  std::array<unsigned, 64> luma_quant{};  ///< Natural order, 0 when absent.
  std::string subsampling = "other";
};

/** Decode one regular JPEG after applying source/tile dimensions and memory
 * checks.
 * @param path JPEG path.
 * @param tile True for a cache tile (must be exactly 256x256); false for a
 * source frame.
 * @param budget Maximum permitted working bytes.
 * @param header Optional destination for encoder facts used by --measure.
 * @param decode_pixels False to validate header/dimensions/budget only (used
 * when rendering from a map cache without pinhole source sampling).
 * @return Contiguous RGB pixels, or empty when @p decode_pixels is false. */
cv::Mat read_jpeg(const std::string& path, bool tile, size_t budget,
                  JpegHeader* header = nullptr, bool decode_pixels = true) {
  File input = open_regular(path, (tile ? 4 : 64) * mib);
  auto decoder = std::make_unique<Decoder>();
  decoder->state.err = jpeg_std_error(&decoder->error.manager);
  decoder->error.emit = decoder->error.manager.emit_message;
  decoder->error.manager.emit_message = jpeg_message;
  decoder->error.manager.error_exit = jpeg_failure;
  if (setjmp(decoder->error.recovery))
    throw std::runtime_error("invalid JPEG: " + path);
  decoder->created = true;
  jpeg_create_decompress(&decoder->state);
  jpeg_stdio_src(&decoder->state, input.get());
  require(jpeg_read_header(&decoder->state, TRUE) == JPEG_HEADER_OK,
          "invalid JPEG header");
  const size_t width = decoder->state.image_width,
               height = decoder->state.image_height;
  require(width >= 2 && height >= 2 && width <= maximum_image_dimension &&
              height <= maximum_image_dimension &&
              width * height <= maximum_mock_pixels &&
              (!tile || (width == 256 && height == 256)),
          "unsupported JPEG dimensions: " + path);
  require(32 * mib + width * height * 12 <= budget,
          "input exceeds memory budget before decode");
  if (header) {
    header->width = static_cast<int>(width);
    header->height = static_cast<int>(height);
    const JQUANT_TBL* table = decoder->state.quant_tbl_ptrs[0];
    for (size_t index = 0; table && index < header->luma_quant.size(); ++index)
      header->luma_quant[index] = table->quantval[index];
    if (decoder->state.num_components == 3 &&
        decoder->state.jpeg_color_space == JCS_YCbCr) {
      const auto* components = decoder->state.comp_info;
      if (components[1].h_samp_factor == 1 && components[1].v_samp_factor == 1 &&
          components[2].h_samp_factor == 1 && components[2].v_samp_factor == 1) {
        const int horizontal = components[0].h_samp_factor,
                  vertical = components[0].v_samp_factor;
        header->subsampling = horizontal == 2 && vertical == 2 ? "4:2:0"
                              : horizontal == 2 && vertical == 1 ? "4:2:2"
                              : horizontal == 1 && vertical == 1 ? "4:4:4"
                                                                : "other";
      }
    }
  }
  if (!decode_pixels) {
    require(decoder->error.manager.num_warnings == 0, "damaged JPEG: " + path);
    return {};
  }
  decoder->state.out_color_space = JCS_RGB;
  jpeg_start_decompress(&decoder->state);
  decoder->image.create(static_cast<int>(height), static_cast<int>(width),
                        CV_8UC3);
  while (decoder->state.output_scanline < height) {
    JSAMPROW row = decoder->image.ptr(decoder->state.output_scanline);
    require(jpeg_read_scanlines(&decoder->state, &row, 1) == 1,
            "incomplete JPEG scanline");
  }
  jpeg_finish_decompress(&decoder->state);
  require(decoder->error.manager.num_warnings == 0, "damaged JPEG: " + path);
  return decoder->image;
}

/** @brief Own encoder state independently of the output stream. */
struct Encoder {
  jpeg_compress_struct state{};
  JpegError error{};
  bool created = false;
  /** @brief Release the libjpeg compressor when armed.
   * @details The caller separately owns the stream and decides whether to
   * publish the temporary output. */
  ~Encoder() {
    if (created) jpeg_destroy_compress(&state);
  }
};

/** @brief Own an unpublished derivative and its temporary sibling path. */
struct TemporaryOutput {
  std::string path;
  File file;
  /** @brief Open an exclusive temporary file beside the requested output.
   * @param output Final derivative path.
   * @throws std::runtime_error If temporary-file creation or stream setup fails.
   * @details A sibling path permits rename-based publication on the same
   * filesystem. Clean up the descriptor and file if wrapping it fails. */
  explicit TemporaryOutput(const std::string& output) {
    std::string pattern = output + ".tmp-XXXXXX";
    const int descriptor = mkstemp(pattern.data());
    require(descriptor >= 0, "cannot create temporary derivative");
    path = pattern;
    file.reset(fdopen(descriptor, "wb"));
    if (!file) {
      close(descriptor);
      unlink(path.c_str());
      throw std::runtime_error("cannot open temporary derivative");
    }
  }
  /** @brief Remove an unpublished temporary pathname.
   * @details A successful rename has already moved the file; cleanup targets
   * only the temporary name, never the published derivative. */
  ~TemporaryOutput() {
    if (!path.empty()) unlink(path.c_str());
  }
};

/** Write a JPEG derivative with EXIF into a same-directory temporary then
 * rename it.
 * @param output Destination, which is never the clean input.
 * @param image RGB derivative.
 * @param quality Profile JPEG quality.
 * @param metadata EXIF payload to retain and extend. */
void write_jpeg(const std::string& output, const cv::Mat& image, int quality,
                const exif_document* metadata) {
  std::array<unsigned char, EXIF_MAX_PAYLOAD> payload{};
  size_t length = 0;
  require(exif_serialize(metadata, payload.data(), payload.size(), &length) == 0,
          "EXIF record exceeds JPEG APP1 capacity");
  TemporaryOutput temporary(output);
  auto encoder = std::make_unique<Encoder>();
  encoder->state.err = jpeg_std_error(&encoder->error.manager);
  encoder->error.manager.error_exit = jpeg_failure;
  if (setjmp(encoder->error.recovery))
    throw std::runtime_error("JPEG encoding failed");
  encoder->created = true;
  jpeg_create_compress(&encoder->state);
  jpeg_stdio_dest(&encoder->state, temporary.file.get());
  encoder->state.image_width = image.cols;
  encoder->state.image_height = image.rows;
  encoder->state.input_components = 3;
  encoder->state.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder->state);
  jpeg_set_quality(&encoder->state, quality, TRUE);
  jpeg_start_compress(&encoder->state, TRUE);
  jpeg_write_marker(&encoder->state, JPEG_APP0 + 1, payload.data(), static_cast<unsigned>(length));
  while (encoder->state.next_scanline < static_cast<unsigned>(image.rows)) {
    JSAMPROW row =
        const_cast<JSAMPROW>(image.ptr(encoder->state.next_scanline));
    require(jpeg_write_scanlines(&encoder->state, &row, 1) == 1,
            "cannot encode scanline");
  }
  jpeg_finish_compress(&encoder->state);
  require(fflush(temporary.file.get()) == 0 &&
              fsync(fileno(temporary.file.get())) == 0,
          "cannot flush derivative");
  require(fclose(temporary.file.release()) == 0, "cannot close derivative");
  require(rename(temporary.path.c_str(), output.c_str()) == 0,
          "cannot publish derivative");
  temporary.path.clear();
}

/** Read EXIF text while preserving the eight-byte UserComment encoding prefix.
 * @param metadata Parsed EXIF object.
 * @param ifd EXIF directory.
 * @param tag Tag identifier.
 * @return Bounded text, or empty when absent. */
std::string tag_text(const exif_document* metadata, exif_directory ifd, uint16_t tag) {
  exif_value entry{};
  if (!exif_document_get(metadata, ifd, tag, &entry)) return {};
  require(entry.size <= maximum_exif_payload_bytes &&
              (entry.format == EXIF_ASCII || entry.format == EXIF_UNDEFINED), "invalid EXIF text");
  std::string result(reinterpret_cast<const char*>(entry.data), entry.size);
  const size_t minimum =
      tag == EXIF_USER_COMMENT ? std::min<size_t>(8, result.size()) : 0;
  while (result.size() > minimum && result.back() == '\0') result.pop_back();
  return result;
}

/** Replace or create an EXIF entry with zero-initialized storage.
 * @param metadata EXIF object to update.
 * @param ifd Destination directory.
 * @param tag EXIF tag identifier.
 * @param format EXIF storage format.
 * @param components Number of values.
 * @return Borrowed entry owned by @p metadata; throws if bounded allocation
 * fails. */
unsigned char* replace_tag(exif_document* metadata, exif_directory ifd, uint16_t tag,
                           exif_format format, size_t components) {
  require(components <= UINT32_MAX, "invalid EXIF component count");
  unsigned char* entry = exif_document_reserve(metadata, ifd, tag, format,
                                               static_cast<uint32_t>(components));
  require(entry != nullptr, "EXIF tag exceeds bounded storage");
  return entry;
}

/** Replace an ASCII or UNDEFINED EXIF text tag and include its terminating
 * byte.
 * @param metadata EXIF object to update.
 * @param ifd Destination directory.
 * @param tag EXIF tag identifier.
 * @param text Text bytes.
 * @param undefined Use EXIF_FORMAT_UNDEFINED (needed for UserComment). */
void set_text(exif_document* metadata, exif_directory ifd, uint16_t tag,
              const std::string& text, bool undefined = false) {
    unsigned char* entry = replace_tag(
      metadata, ifd, tag, undefined ? EXIF_UNDEFINED : EXIF_ASCII,
      text.size() + 1);
  for (size_t index = 0; index < text.size(); ++index)
    entry[index] = static_cast<unsigned char>(text[index]);
}

/** Store a floating measurement as a bounded EXIF rational.
 * @param metadata EXIF object to update.
 * @param tag EXIF measurement tag.
 * @param value Measurement value.
 * @param denominator Decimal scale used to preserve useful precision.
 * @param signed_value Select signed rational format for exposure compensation.
 */
void set_rational(exif_document* metadata, uint16_t tag, double value,
                  unsigned denominator, bool signed_value = false) {
    require(denominator > 0 && std::isfinite(value), "invalid EXIF rational");
    const double scaled = std::round(value * denominator);
    require(std::isfinite(scaled) &&
          (signed_value ? scaled >= INT32_MIN && scaled <= INT32_MAX && denominator <= INT32_MAX
                : scaled >= 0 && scaled <= UINT32_MAX), "EXIF rational out of range");
    unsigned char* entry = replace_tag(
      metadata, EXIF_CAMERA, tag,
      signed_value ? EXIF_SRATIONAL : EXIF_RATIONAL, 1);
  if (signed_value) {
    exif_encode_long(entry, static_cast<uint32_t>(static_cast<int32_t>(scaled)));
  } else {
    exif_encode_long(entry, static_cast<uint32_t>(scaled));
  }
  exif_encode_long(entry + 4, denominator);
}

/** Validate USHER shot tags and extract the source pose and camera geometry.
 * @param metadata EXIF from the clean input JPEG.
 * @param[out] geometry MIMIC source geometry, or null for an explicit
 * real-camera profile.
 * @return Validated pose values in JSON form. */
Json read_shot(const exif_document* metadata, Json& geometry) {
  geometry =
      Json::parse(tag_text(metadata, EXIF_PRIMARY, EXIF_DOCUMENT_NAME));
  const bool xyz_geometry = geometry.value("tile_scheme", "") == "XYZ";
  // Older MIMIC EXIF used GOOGLE as a cache-layout token, not imagery provenance.
  const bool legacy_cache_geometry = geometry.value("provider", "") == "GOOGLE";
  require(geometry.value("schema", "") == "USHER_MOCK_MAP_V1" &&
              (xyz_geometry || legacy_cache_geometry) &&
              geometry.value("zoom", 0) == 18 &&
              geometry.value("projection", "") == "pinhole_flat_ground",
          "input must be an unmodified USHER MIMIC JPEG");
  // Normalize older cache-key metadata before writing derived-image EXIF.
  Json normalized_geometry = Json::object();
  for (const auto& field : geometry.items()) {
    if (field.first != "provider")
      normalized_geometry.set(field.first, field.second);
  }
  normalized_geometry.set("tile_scheme", "XYZ");
  geometry = std::move(normalized_geometry);
  require(tag_text(metadata, EXIF_CAMERA, EXIF_USER_COMMENT)
                  .find("USHER_LENS_V1") == std::string::npos,
          "lens effects already applied");
    exif_value orientation{};
    uint32_t orientation_value = 0;
    require(!exif_document_get(metadata, EXIF_PRIMARY, EXIF_ORIENTATION, &orientation) ||
                (orientation.format == EXIF_SHORT && orientation.count == 1 &&
                 exif_value_unsigned(&orientation, 0, &orientation_value) && orientation_value == 1),
      "non-upright EXIF orientation is unsupported");
  std::istringstream description(
      tag_text(metadata, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION));
  std::string field;
  std::getline(description, field, ';');
  require(field.find("Paparazzi USHER ") == 0, "missing USHER flight metadata");
  std::map<std::string, std::string> fields;
  while (std::getline(description, field, ';')) {
    const size_t start = field.find_first_not_of(" \t\r\n");
    const size_t end = field.find_last_not_of(" \t\r\n");
    require(start != std::string::npos, "empty flight metadata field");
    field = field.substr(start, end - start + 1);
    const size_t separator = field.find('=');
    require(separator != std::string::npos &&
                fields
                    .emplace(field.substr(0, separator),
                             field.substr(separator + 1))
                    .second,
            "malformed or duplicate flight metadata field");
  }
  Json shot = Json::object();
  for (const char* key :
       {"lat_deg", "lon_deg", "roll_deg", "pitch_deg", "yaw_deg", "speed_m_s",
        "course_deg", "groundalt_m"}) {
    const std::string original = "original_" + std::string(key);
    const std::string& text = fields.at(
        (std::string(key) == "lat_deg" || std::string(key) == "lon_deg") &&
                fields.count(original)
            ? original
            : key);
    size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    require(consumed == text.size(), "invalid flight number");
    shot.set(key, number(value, key, -1e7, 1e7));
  }
  number(shot["lat_deg"], "latitude", -85.05112878, 85.05112878);
  number(shot["lon_deg"], "longitude", -180, 180);
  number(shot["speed_m_s"], "speed", 0, 1000);
  number(shot["groundalt_m"], "AGL", 0.001, 10000);
  for (const char* key : {"horizontal_fov_deg", "vertical_fov_deg"})
    number(geometry.at(key), key, 0.001, 178.999);
  return shot;
}

/** Apply the profile budget and additional conservative host-memory/thread
 * limits. */
struct Resources {
  size_t budget;
  int threads;
  /** @brief Resolve requested resources against current host limits.
   * @param profile Validated memory budget and thread settings.
   * @details Restrict small hosts to one thread and at most one third of
   * total memory. Available-memory evidence may lower the budget further to
   * leave headroom for other processes. */
  explicit Resources(const Json& profile)
      : budget(profile["memory_budget_mb"].get<size_t>() * mib),
        threads(profile["threads"].get<int>()) {
    std::ifstream input("/proc/meminfo");
    std::string line;
    while (std::getline(input, line)) {
      std::istringstream fields(line);
      std::string key;
      size_t value;
      if (!(fields >> key >> value)) continue;
      if (key == "MemTotal:" && value <= 768 * 1024) {
        budget = std::min(budget, value * 1024 / 3);
        threads = 1;
      } else if (key == "MemAvailable:") {
        budget =
            std::min(budget, value > 128 * 1024 ? value * 1024 - 128 * mib : 0);
      }
    }
  }
};

/** Inclusive tile bounds; @c right and @c bottom identify actual covered tiles.
 */
struct Bounds {
  int64_t left, top, right, bottom;
  /** @brief Validate an inclusive tile rectangle and count its cells.
   * @param limit Maximum permitted tile count.
   * @return Number of tiles including both boundary rows and columns.
   * @throws std::runtime_error If the extent, count, or Mercator row range is invalid.
   * @details Bound each span before multiplying. Columns may cross the
   * antimeridian and are wrapped at lookup; rows cannot wrap past the poles. */
  size_t count(int limit) const {
    require(right >= left && bottom >= top && right - left < limit &&
                bottom - top < limit,
            "lens tile rectangle exceeds max_tiles");
    const size_t count = (right - left + 1) * (bottom - top + 1);
    require(count <= static_cast<size_t>(limit),
            "lens tile rectangle exceeds max_tiles");
    require(top >= 0 && bottom < (1 << 18),
            "view beyond Web Mercator coverage");
    return count;
  }
};

/** @brief 8-bit sRGB to linear-light lookup, built once. */
const std::array<float, 256>& srgb_decoding() {
  static const std::array<float, 256> table = [] {
    std::array<float, 256> values{};
    for (size_t index = 0; index < values.size(); ++index) {
      const float encoded = static_cast<float>(index) / 255.0f;
      values[index] = encoded <= 0.04045f
                          ? encoded / 12.92f
                          : std::pow((encoded + 0.055f) / 1.055f, 2.4f);
    }
    return values;
  }();
  return table;
}

/** Small LRU for source tiles; the fixed entry cap bounds decoded tile memory.
 */
class TileCache {
  struct Entry {
    int64_t column, row;
    cv::Mat linear;
  };
  std::list<Entry> entries;
  std::vector<std::pair<int64_t, int64_t>> verified;
  std::string root;
  size_t budget;

 public:
  /** @brief Configure a cache with no resident tiles.
   * @param path Root of the local map cache.
   * @param limit Working-byte budget passed to the JPEG decoder.
   * @details Load lazily so views that do not overlap need not share an
   * eagerly decoded mosaic. */
  TileCache(const std::string& path, size_t limit)
      : root(path), budget(limit) {}
  /** @brief Ensure a tile in the frame's bounding box exists and is valid
   * without re-decoding tiles already verified earlier in the same render. */
  void verify(int64_t column, int64_t row) {
    const int64_t wrapped = (column % (1 << 18) + (1 << 18)) % (1 << 18);
    for (const auto& seen : verified)
      if (seen.first == wrapped && seen.second == row) return;
    tile(column, row);
  }
  /** @brief Fetch a linearized float32 RGB tile from the local zoom-18 cache.
   * @param column XYZ column, which may be outside the world range before wrapping.
   * @param row Validated XYZ row; not wrapped.
   * @return Reference-counted 256x256 CV_32FC3 pixels; callers must treat them
   * as read-only.
   * @throws std::runtime_error If the required tile cannot be read or decoded.
   * @details Normalize the column before lookup so opposite antimeridian
   * representations share an entry. Decoding and sRGB-to-linear conversion
   * happen once per unique tile rather than per strip/channel. */
  cv::Mat tile(int64_t column, int64_t row) {
    column = (column % (1 << 18) + (1 << 18)) % (1 << 18);
    for (auto entry = entries.begin(); entry != entries.end(); ++entry) {
      if (entry->column == column && entry->row == row) {
        entries.splice(entries.begin(), entries, entry);
        return entries.front().linear;
      }
    }
    const cv::Mat encoded =
        read_jpeg(root + "/GOOGLE/18/" + std::to_string(column) + "/" +
                      std::to_string(row) + ".jpeg",
                  true, budget);
    cv::Mat linear(256, 256, CV_32FC3);
    const float* decode = srgb_decoding().data();
    const unsigned char* source = encoded.data;
    float* destination = linear.ptr<float>();
    for (int index = 0; index < 256 * 256 * 3; ++index)
      destination[index] = decode[source[index]];
    bool recorded = false;
    for (const auto& seen : verified)
      if (seen.first == column && seen.second == row) recorded = true;
    if (!recorded) verified.emplace_back(column, row);
    entries.push_front({column, row, linear});
    if (entries.size() > tile_cache_entries) entries.pop_back();
    return linear;
  }
};

/** Camera intrinsics and Brown/fisheye distortion coefficients for ray
 * generation. */
struct Lens {
  Json profile;
  double focal_x, focal_y;
  cv::Mat distortion;
  bool fisheye;
  /** @brief Build intrinsics for the selected distortion model.
   * @param width Output columns in pixels.
   * @param height Output rows in pixels.
   * @param values Validated model, field of view, and distortion coefficients.
   * @details Brown intrinsics use the two rectilinear fields of view. Fisheye
   * intrinsics derive a shared focal length from the distorted diagonal ray. */
  Lens(int width, int height, const Json& values)
      : profile(values), fisheye(values["model"] == "fisheye") {
    const double center_x = (width - 1) * 0.5, center_y = (height - 1) * 0.5;
    std::vector<double> coefficients =
        values["distortion"].get<std::vector<double>>();
    distortion = cv::Mat(coefficients).clone();
    if (fisheye) {
      const double angle = values["diagonal_fov_deg"].get<double>() * pi / 360;
      double polynomial = 1, power = angle * angle;
      for (double coefficient : coefficients) {
        polynomial += coefficient * power;
        power *= angle * angle;
      }
      focal_x = focal_y = std::hypot(center_x, center_y) / (angle * polynomial);
    } else {
      focal_x = center_x /
                std::tan(values["horizontal_fov_deg"].get<double>() * pi / 360);
      focal_y = center_y /
                std::tan(values["vertical_fov_deg"].get<double>() * pi / 360);
    }
    require(std::isfinite(focal_x) && focal_x > 0 && focal_y > 0,
            "invalid lens focal scale");
  }
  /** @brief Camera matrix for one channel focal scale. */
  cv::Matx33d intrinsic(int width, int height, double scale) const {
    return cv::Matx33d(focal_x * scale, 0, (width - 1) * 0.5, 0,
                       focal_y * scale, (height - 1) * 0.5, 0, 0, 1);
  }
  /** Forward lens model: normalized rays to distorted pixel coordinates.
   * @param rays N x 1 CV_32FC2 normalized camera rays.
   * @param width Full image width.
   * @param height Full image height.
   * @param scale Per-channel focal scaling.
   * @return N x 1 CV_32FC2 pixel coordinates. */
  cv::Mat project(const cv::Mat& rays, int width, int height,
                  double scale) const {
    const cv::Matx33d camera = intrinsic(width, height, scale);
    cv::Mat result;
    if (fisheye) {
      cv::fisheye::distortPoints(rays, result, camera, distortion);
      return result;
    }
    cv::Mat objects(rays.rows, 1, CV_32FC3);
    for (int index = 0; index < rays.rows; ++index) {
      const cv::Vec2f point = rays.at<cv::Vec2f>(index);
      objects.at<cv::Vec3f>(index) = cv::Vec3f(point[0], point[1], 1);
    }
    cv::projectPoints(objects, cv::Vec3d(), cv::Vec3d(), camera, distortion,
                      result, cv::noArray());
    return result;
  }
  /** Exactly invert distortion for arbitrary pixels, then forward-project to
   * reject non-convergent rays.
   * @param pixels N x 1 CV_32FC2 pixel coordinates.
   * @param width Full image width.
   * @param height Full image height.
   * @param scale Per-channel focal scaling for chromatic-aberration simulation.
   * @return N x 1 normalized camera rays. */
  cv::Mat undistort(const cv::Mat& pixels, int width, int height,
                    double scale) const {
    const cv::Matx33d intrinsic = this->intrinsic(width, height, scale);
    cv::Mat normalized, projected;
    auto project = [&](const cv::Mat& rays) {
      return this->project(rays, width, height, scale);
    };
    if (fisheye) {
      cv::fisheye::undistortPoints(pixels, normalized, intrinsic, distortion);
    } else {
      cv::undistortPoints(
          pixels, normalized, intrinsic, distortion, cv::noArray(),
          cv::noArray(),
          cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                           lens_inverse_iterations, 1e-8));
    }
    projected = project(normalized);
    // OpenCV's fixed-point inverse can stop several hundredths of a pixel
    // short under strong distortion; Newton-polish it against the forward
    // model with a numeric Jacobian, keeping only improving passes.
    double residual = cv::checkRange(normalized) && cv::checkRange(projected)
                          ? cv::norm(projected, pixels, cv::NORM_INF)
                          : INFINITY;
    constexpr float delta = 1e-4f;
    for (int pass = 0; pass < lens_polish_passes &&
                       residual > lens_polish_tolerance_px &&
                       std::isfinite(residual);
         ++pass) {
      const cv::Mat shift_x = normalized + cv::Scalar(delta, 0),
                    shift_y = normalized + cv::Scalar(0, delta);
      const cv::Mat moved_x = project(shift_x), moved_y = project(shift_y);
      cv::Mat candidate = normalized.clone();
      for (int index = 0; index < pixels.rows; ++index) {
        const cv::Vec2f base = projected.at<cv::Vec2f>(index);
        const cv::Vec2f column_x = (moved_x.at<cv::Vec2f>(index) - base) / delta;
        const cv::Vec2f column_y = (moved_y.at<cv::Vec2f>(index) - base) / delta;
        const cv::Vec2f error = pixels.at<cv::Vec2f>(index) - base;
        const float determinant =
            column_x[0] * column_y[1] - column_y[0] * column_x[1];
        if (!(std::abs(determinant) > 1e-12f)) continue;
        candidate.at<cv::Vec2f>(index) += cv::Vec2f(
            (column_y[1] * error[0] - column_y[0] * error[1]) / determinant,
            (column_x[0] * error[1] - column_x[1] * error[0]) / determinant);
      }
      const cv::Mat candidate_projected = project(candidate);
      if (!cv::checkRange(candidate) || !cv::checkRange(candidate_projected))
        break;
      const double candidate_residual =
          cv::norm(candidate_projected, pixels, cv::NORM_INF);
      if (candidate_residual >= residual) break;
      normalized = candidate;
      projected = candidate_projected;
      residual = candidate_residual;
    }
    require(cv::checkRange(normalized) && cv::checkRange(projected) &&
                cv::norm(projected, pixels, cv::NORM_INF) <=
                    maximum_reprojection_error_pixels,
            "lens inversion did not converge");
    return normalized;
  }
  /** Normalized rays for one strip; see RayField for the accuracy contract.
   * @param width Full image width.
   * @param height Full image height.
   * @param first First row represented by this strip.
   * @param rows Strip row count.
   * @param scale Per-channel focal scaling.
   * @return rows x width CV_32FC2 normalized rays. */
  cv::Mat rays(int width, int height, int first, int rows, double scale) const;
};

/** Full-frame ray field: exact inverse distortion on a coarse node grid,
 * bilinear interpolation between nodes.
 * @details Brown and fisheye inversion is iterative and dominated rendering
 * time when evaluated per pixel and per colour. The distortion field is
 * smooth, so the node step is halved until every interpolated cell-centre
 * ray forward-projects within ray_interpolation_tolerance_px of its pixel.
 * Step 1 is the exact per-pixel inverse. */
class RayField {
  std::vector<float> node_x, node_y;
  cv::Mat nodes;
  std::vector<int> cell_x;
  std::vector<float> weight_x;
  int columns;

  static std::vector<float> axis(int size, int step) {
    std::vector<float> values;
    for (int value = 0; value < size - 1; value += step) values.push_back(value);
    values.push_back(static_cast<float>(size - 1));
    return values;
  }
  static void locate(const std::vector<float>& axis, float value, int& cell,
                     float& weight) {
    cell = static_cast<int>(
        std::upper_bound(axis.begin(), axis.end() - 1, value) - axis.begin()) - 1;
    cell = std::clamp(cell, 0, static_cast<int>(axis.size()) - 2);
    weight = (value - axis[cell]) / (axis[cell + 1] - axis[cell]);
  }
  // Nodes are unit view directions: bounded and smooth up to 90 degrees,
  // unlike tangent-plane rays, so wide fisheye corners interpolate well.
  cv::Vec2f blend(int cell_row, float weight_row, int cell_column,
                  float weight_column) const {
    const cv::Vec3f* top = nodes.ptr<cv::Vec3f>(cell_row);
    const cv::Vec3f* bottom = nodes.ptr<cv::Vec3f>(cell_row + 1);
    const cv::Vec3f upper = top[cell_column] * (1 - weight_column) +
                            top[cell_column + 1] * weight_column;
    const cv::Vec3f lower = bottom[cell_column] * (1 - weight_column) +
                            bottom[cell_column + 1] * weight_column;
    const cv::Vec3f direction = upper * (1 - weight_row) + lower * weight_row;
    return cv::Vec2f(direction[0] / direction[2], direction[1] / direction[2]);
  }

 public:
  int step = 1;
  double error_px = 0;
  /** @brief Build and verify the ray field for one channel focal scale.
   * @throws std::runtime_error If exact inversion fails at any node/check. */
  RayField(const Lens& lens, int width, int height, double scale,
           size_t budget = static_cast<size_t>(-1))
      : columns(width) {
    for (step = ray_grid_initial_step;; step /= 2) {
      node_x = axis(width, step);
      node_y = axis(height, step);
      // Include inverse-polishing temporaries before allocating a dense grid.
      require(node_x.size() * node_y.size() <= budget / 256,
              "ray grid exceeds memory budget");
      cv::Mat pixels(static_cast<int>(node_x.size() * node_y.size()), 1, CV_32FC2);
      for (size_t row = 0; row < node_y.size(); ++row)
        for (size_t column = 0; column < node_x.size(); ++column)
          pixels.at<cv::Vec2f>(static_cast<int>(row * node_x.size() + column)) =
              cv::Vec2f(node_x[column], node_y[row]);
      const cv::Mat rays = lens.undistort(pixels, width, height, scale);
      nodes.create(static_cast<int>(node_y.size()),
                   static_cast<int>(node_x.size()), CV_32FC3);
      for (int index = 0; index < rays.rows; ++index) {
        const cv::Vec2f ray = rays.at<cv::Vec2f>(index);
        nodes.at<cv::Vec3f>(index / nodes.cols, index % nodes.cols) =
            cv::Vec3f(ray[0], ray[1], 1) / std::sqrt(1 + ray.dot(ray));
      }
      error_px = 0;
      if (step == 1) break;
      const int cells_x = static_cast<int>(node_x.size()) - 1,
                cells_y = static_cast<int>(node_y.size()) - 1;
      cv::Mat centres(cells_x * cells_y, 1, CV_32FC2);
      for (int row = 0; row < cells_y; ++row)
        for (int column = 0; column < cells_x; ++column)
          centres.at<cv::Vec2f>(row * cells_x + column) =
              cv::Vec2f((node_x[column] + node_x[column + 1]) * 0.5f,
                        (node_y[row] + node_y[row + 1]) * 0.5f);
      cv::Mat blended(centres.rows, 1, CV_32FC2);
      for (int row = 0; row < cells_y; ++row)
        for (int column = 0; column < cells_x; ++column)
          blended.at<cv::Vec2f>(row * cells_x + column) =
              blend(row, 0.5f, column, 0.5f);
      const cv::Mat reprojected = lens.project(blended, width, height, scale);
      error_px = cv::checkRange(reprojected)
                     ? cv::norm(reprojected, centres, cv::NORM_INF)
                     : INFINITY;
      if (error_px <= ray_interpolation_tolerance_px) break;
    }
    cell_x.resize(width);
    weight_x.resize(width);
    for (int column = 0; column < width; ++column)
      locate(node_x, static_cast<float>(column), cell_x[column], weight_x[column]);
  }
  /** @brief Resident bytes of the node grid and column lookup. */
  size_t bytes() const {
    return nodes.total() * nodes.elemSize() +
           cell_x.size() * sizeof(int) + weight_x.size() * sizeof(float) +
           (node_x.size() + node_y.size()) * sizeof(float);
  }
  /** @brief Interpolate rays at horizontal grid nodes for one full-frame row.
   * @details Along each cell [node_x[c], node_x[c + 1]], flat-ground
   * projection is a fractional-linear function with positive denominator and
   * therefore monotone, so row extrema occur at these node columns. */
  int node_columns() const { return nodes.cols; }
  void row_nodes_into(int full_row, float* rx, float* ry) const {
    int cell_row;
    float weight_row;
    locate(node_y, static_cast<float>(full_row), cell_row, weight_row);
    const cv::Vec3f* top = nodes.ptr<cv::Vec3f>(cell_row);
    const cv::Vec3f* bottom = nodes.ptr<cv::Vec3f>(cell_row + 1);
    for (int col = 0; col < nodes.cols; ++col) {
      const cv::Vec3f direction =
          top[col] * (1.0f - weight_row) + bottom[col] * weight_row;
      const float inv_z = 1.0f / direction[2];
      rx[col] = direction[0] * inv_z;
      ry[col] = direction[1] * inv_z;
    }
  }
  /** @brief Interpolate rays for one full-frame row into a caller buffer. */
  void row_into(int full_row, cv::Vec2f* destination) const {
    int cell_row;
    float weight_row;
    locate(node_y, static_cast<float>(full_row), cell_row, weight_row);
    for (int column = 0; column < columns; ++column)
      destination[column] =
          blend(cell_row, weight_row, cell_x[column], weight_x[column]);
  }
  /** @brief Interpolate rays for full-frame rows [first, first + rows). */
  cv::Mat strip(int first, int rows) const {
    cv::Mat result(rows, columns, CV_32FC2);
    cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
      for (int row = range.start; row < range.end; ++row)
        row_into(first + row, result.ptr<cv::Vec2f>(row));
    });
    return result;
  }
  /** @brief Interpolate one ray at a sub-pixel position. */
  cv::Vec2f at(float x, float y) const {
    int cell_row, cell_column;
    float weight_row, weight_column;
    locate(node_y, y, cell_row, weight_row);
    locate(node_x, x, cell_column, weight_column);
    return blend(cell_row, weight_row, cell_column, weight_column);
  }
};

cv::Mat Lens::rays(int width, int height, int first, int rows,
                   double scale) const {
  return RayField(*this, width, height, scale).strip(first, rows);
}

/** Flight pose and profile motion/exposure settings used for ground projection.
 */
struct Pose {
  double altitude, speed, course;
  std::array<double, 3> angles, rates;
  double readout, exposure, vibration, frequency, phase;
  /** @brief Resolve the shot geometry and time-dependent motion model.
   * @param shot Validated flight pose and ground motion.
   * @param profile Validated lens motion, height, and shutter settings.
   * @details Global shutter removes the row readout term. Reverse rolling
   * readout negates it, preserving a common midpoint time for both directions. */
  Pose(const Json& shot, const Json& profile) {
    altitude = profile["height_override_m"].is_null()
                   ? shot["groundalt_m"].get<double>()
                   : profile["height_override_m"].get<double>();
    speed =
        shot["speed_m_s"].get<double>() * profile["motion_scale"].get<double>();
    course = shot["course_deg"].get<double>() * pi / 180;
    angles = {shot["roll_deg"].get<double>(), shot["pitch_deg"].get<double>(),
          shot["yaw_deg"].get<double>()};
    rates = profile["angular_rate_deg_s"].get<std::array<double, 3>>();
    readout = profile["shutter"] == "rolling"
                  ? profile["readout_s"].get<double>()
                  : 0;
    if (profile["readout_direction"] == "bottom_to_top") readout = -readout;
    exposure = profile["exposure_s"].get<double>();
    vibration = profile["vibration_deg"].get<double>();
    frequency = profile["vibration_hz"].get<double>();
    phase = profile["vibration_phase_deg"].get<double>() * pi / 180;
  }
  /** @brief Check whether camera geometry varies across exposure samples.
   * @return True without translation, angular rate, or time-varying vibration.
   * @details Fixed attitude is permitted; only motion over the exposure
   * requires evaluating distinct sample times. */
  bool stationary() const {
    return speed == 0 && rates == std::array<double, 3>{0, 0, 0} &&
           (vibration == 0 || frequency == 0);
  }
  /** Intersect each ray with a flat ground plane at its row/exposure time.
   * @param rays Normalized strip rays.
   * @param first First full-image row in @p rays.
   * @param height Full image height used for rolling-shutter timing.
   * @param sample Exposure-time offset from the center of the shot.
   * @return East and north displacement matrices in meters. */
  std::pair<cv::Mat, cv::Mat> ground(const cv::Mat& rays, int first, int height,
                                     double sample) const {
    cv::Mat east(rays.size(), CV_32F), north(rays.size(), CV_32F);
    std::atomic<bool> horizon{false};
    cv::parallel_for_(cv::Range(0, rays.rows), [&](const cv::Range& range) {
    for (int row = range.start; row < range.end; ++row) {
      const float instant =
          static_cast<float>(
              (static_cast<double>(first + row) / (height - 1) - 0.5) *
              readout) +
          static_cast<float>(sample);
      const double cycle = 2 * pi * frequency * instant + phase;
      const double roll =
          (angles[0] + rates[0] * instant + vibration * std::sin(cycle)) * pi /
          180;
      const double pitch = (angles[1] + rates[1] * instant +
                            vibration * std::sin(cycle + pi / 2)) *
                           pi / 180;
      const double yaw = (angles[2] + rates[2] * instant) * pi / 180;
      const float cos_roll = std::cos(roll), sin_roll = std::sin(roll),
                  cos_pitch = std::cos(pitch), sin_pitch = std::sin(pitch);
      const float cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
      const float north_forward = cos_yaw * cos_pitch,
                  north_right =
                      cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
      const float north_down =
          cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
      const float east_forward = sin_yaw * cos_pitch,
                  east_right =
                      sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
      const float east_down =
          sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
      const float down_right = cos_pitch * sin_roll,
                  down_down = cos_pitch * cos_roll;
      const float translation_north = speed * instant * std::cos(course),
                  translation_east = speed * instant * std::sin(course);
      for (int column = 0; column < rays.cols; ++column) {
        const cv::Vec2f point = rays.at<cv::Vec2f>(row, column);
        const float forward = -point[1], right = point[0];
        const float down =
            -sin_pitch * forward + down_right * right + down_down;
        if (!(down > 1e-4f)) {
          horizon = true;
          return;
        }
        const float scale = altitude / down;
        north.at<float>(row, column) =
            (north_forward * forward + north_right * right + north_down) *
                scale +
            translation_north;
        east.at<float>(row, column) =
            (east_forward * forward + east_right * right + east_down) * scale +
            translation_east;
      }
    }
    });
    require(!horizon, "lens/exposure rays reach the horizon");
    require(cv::checkRange(east) && cv::checkRange(north),
            "non-finite ground projection");
    return {east, north};
  }
};

struct Rendered {
  cv::Mat image;
  Json evidence;
};

/** Temporal exposure discretization chosen from the actual blur extent. */
struct TemporalSampling {
  int samples;
  double blur_px;
};

/** @brief Choose the fewest odd exposure samples that resolve motion blur.
 * @param field Reference-channel ray field.
 * @param pose Shot motion, vibration and shutter model.
 * @param width Output width.
 * @param height Output height.
 * @param maximum Profile sample ceiling (odd).
 * @return Sample count whose neighbouring-sample ground spacing is at most
 * temporal_sample_spacing_px at every probe, plus the largest blur path.
 * @details Probe the corners, edge centres and centre of the frame, follow
 * each ray's ground intersection across the exposure (including vibration
 * reversals) and convert path length to pixels with the local ground
 * sampling distance. Sub-pixel blur therefore costs few samples, while
 * long exposures keep every sample the profile allows. */
TemporalSampling temporal_sampling(const RayField& field, const Pose& pose,
                                   int width, int height, int maximum) {
  if (pose.stationary()) return {1, 0};
  const int steps = std::max(
      32, static_cast<int>(std::ceil(
              pose.exposure * (pose.vibration > 0 ? pose.frequency : 0) * 8)));
  double blur = 0;
  for (float y : {0.0f, (height - 1) * 0.5f, height - 2.0f}) {
    for (float x : {0.0f, (width - 1) * 0.5f, width - 2.0f}) {
      cv::Mat probe(1, 3, CV_32FC2);
      probe.at<cv::Vec2f>(0, 0) = field.at(x, y);
      probe.at<cv::Vec2f>(0, 1) = field.at(x + 1, y);
      probe.at<cv::Vec2f>(0, 2) = field.at(x, y + 1);
      const int row = static_cast<int>(y);
      const auto centre = pose.ground(probe, row, height, 0);
      const cv::Mat& east = centre.first;
      const cv::Mat& north = centre.second;
      const double gsd = std::min(
          std::hypot(east.at<float>(0, 1) - east.at<float>(0, 0),
                     north.at<float>(0, 1) - north.at<float>(0, 0)),
          std::hypot(east.at<float>(0, 2) - east.at<float>(0, 0),
                     north.at<float>(0, 2) - north.at<float>(0, 0)));
      require(std::isfinite(gsd) && gsd > 0, "invalid ground sampling distance");
      const cv::Mat ray = probe.colRange(0, 1);
      auto previous = pose.ground(ray, row, height, -0.5 * pose.exposure);
      double path = 0;
      for (int step = 1; step <= steps; ++step) {
        auto next = pose.ground(ray, row, height,
                                (static_cast<double>(step) / steps - 0.5) * pose.exposure);
        path += std::hypot(next.first.at<float>(0, 0) - previous.first.at<float>(0, 0),
                           next.second.at<float>(0, 0) - previous.second.at<float>(0, 0));
        previous = std::move(next);
      }
      blur = std::max(blur, path / gsd);
    }
  }
  int samples = static_cast<int>(
                    std::min<double>(maximum, std::ceil(blur / temporal_sample_spacing_px))) + 1;
  if (samples % 2 == 0) ++samples;
  return {std::clamp(samples, 1, maximum), blur};
}

/** @brief Linear-light to 8-bit sRGB lookup, built once.
 * @details 65,536 linear levels keep every round-trip of an 8-bit sRGB code
 * exact while replacing a per-pixel pow() call. */
const std::array<unsigned char, 65536>& srgb_encoding() {
  static const std::array<unsigned char, 65536> table = [] {
    auto encode_one = [](size_t index) -> unsigned char {
      const double linear = index / 65535.0;
      const double encoded = linear <= 0.0031308
                                 ? 12.92 * linear
                                 : 1.055 * std::pow(linear, 1 / 2.4) - 0.055;
      return cv::saturate_cast<unsigned char>(encoded * 255);
    };
    std::array<unsigned char, 65536> values{};
    size_t start = 0;
    for (int code = 0; code < 255; ++code) {
      const double midpoint = (code + 0.5) / 255.0;
      const double approx_linear =
          midpoint <= 0.04045 ? midpoint / 12.92
                              : std::pow((midpoint + 0.055) / 1.055, 2.4);
      size_t boundary = std::clamp<size_t>(
          static_cast<size_t>(approx_linear * 65535.0), start, values.size());
      while (boundary > start && encode_one(boundary - 1) > code) --boundary;
      while (boundary < values.size() && encode_one(boundary) <= code)
        ++boundary;
      for (size_t index = start; index < boundary; ++index)
        values[index] = static_cast<unsigned char>(code);
      start = boundary;
    }
    for (size_t index = start; index < values.size(); ++index)
      values[index] = 255;
    return values;
  }();
  return table;
}

/** @brief Reflect-101 index clamp matching OpenCV's BORDER_REFLECT_101. */
inline int reflect_101(int index, int size) {
  if (size <= 1) return 0;
  while (index < 0 || index >= size)
    index = index < 0 ? -index : 2 * size - 2 - index;
  return index;
}

/** @brief Parallel separable Gaussian blur on a contiguous CV_32FC3 frame. */
CAMERA_LENS_SIMD void blur_linear_rgb(cv::Mat& frame, double sigma) {
  const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0)));
  const int kernel_size = 2 * radius + 1;
  std::vector<float> weights(kernel_size);
  float weight_sum = 0;
  for (int index = -radius; index <= radius; ++index) {
    weights[index + radius] = std::exp(
        -0.5f * static_cast<float>(index * index) /
        static_cast<float>(sigma * sigma));
    weight_sum += weights[index + radius];
  }
  for (float& weight : weights) weight /= weight_sum;
  const int width = frame.cols, height = frame.rows;
  cv::Mat horizontal(height, width, CV_32FC3);
  const float* kernel = weights.data();
  const int left_limit = std::min(radius, width),
            right_start = std::max(left_limit, width - radius);
  cv::parallel_for_(cv::Range(0, height), [&](const cv::Range& range) {
    for (int row = range.start; row < range.end; ++row) {
      const float* source = frame.ptr<float>(row);
      float* destination = horizontal.ptr<float>(row);
      for (int column = 0; column < left_limit; ++column) {
        float red = 0, green = 0, blue = 0;
        for (int tap = -radius; tap <= radius; ++tap) {
          const int sample = reflect_101(column + tap, width) * 3;
          const float weight = kernel[tap + radius];
          red += source[sample] * weight;
          green += source[sample + 1] * weight;
          blue += source[sample + 2] * weight;
        }
        destination[column * 3] = red;
        destination[column * 3 + 1] = green;
        destination[column * 3 + 2] = blue;
      }
      for (int column = left_limit; column < right_start; ++column) {
        float red = 0, green = 0, blue = 0;
        const float* window = source + (column - radius) * 3;
        for (int tap = 0; tap < kernel_size; ++tap) {
          const float weight = kernel[tap];
          red += window[tap * 3] * weight;
          green += window[tap * 3 + 1] * weight;
          blue += window[tap * 3 + 2] * weight;
        }
        destination[column * 3] = red;
        destination[column * 3 + 1] = green;
        destination[column * 3 + 2] = blue;
      }
      for (int column = right_start; column < width; ++column) {
        float red = 0, green = 0, blue = 0;
        for (int tap = -radius; tap <= radius; ++tap) {
          const int sample = reflect_101(column + tap, width) * 3;
          const float weight = kernel[tap + radius];
          red += source[sample] * weight;
          green += source[sample + 1] * weight;
          blue += source[sample + 2] * weight;
        }
        destination[column * 3] = red;
        destination[column * 3 + 1] = green;
        destination[column * 3 + 2] = blue;
      }
    }
  });
  cv::parallel_for_(cv::Range(0, height), [&](const cv::Range& range) {
    for (int row = range.start; row < range.end; ++row) {
      float* destination = frame.ptr<float>(row);
      const int values = width * 3;
      for (int index = 0; index < values; ++index) destination[index] = 0;
      for (int tap = -radius; tap <= radius; ++tap) {
        const float weight = kernel[tap + radius];
        const float* source =
            horizontal.ptr<float>(reflect_101(row + tap, height));
        for (int index = 0; index < values; ++index)
          destination[index] += source[index] * weight;
      }
    }
  });
}

/** @brief Stateless zero-mean, unit-variance Irwin-Hall normal sample.
 * @details Keyed directly by (row_seed, channel_index) so noise is strictly
 * deterministic, unbiased, and bit-identical across any thread count. */
inline float stateless_normal(uint64_t row_seed, int index) {
  uint64_t state =
      row_seed + static_cast<uint64_t>(index) * 0x9e3779b97f4a7c15ULL;
  state = (state ^ (state >> 30)) * 0xbf58476d1ce4e5b9ULL;
  state = (state ^ (state >> 27)) * 0x94d049bb133111ebULL;
  const uint64_t first = state ^ (state >> 31);
  uint64_t second = state * 0x2545f4914f6cdd1dULL;
  second ^= second >> 29;
  const uint32_t sum =
      static_cast<uint32_t>(first & 1023ULL) +
      static_cast<uint32_t>((first >> 10) & 1023ULL) +
      static_cast<uint32_t>((first >> 20) & 1023ULL) +
      static_cast<uint32_t>((first >> 30) & 1023ULL) +
      static_cast<uint32_t>((first >> 40) & 1023ULL) +
      static_cast<uint32_t>((first >> 50) & 1023ULL) +
      static_cast<uint32_t>(second & 1023ULL) +
      static_cast<uint32_t>((second >> 10) & 1023ULL) +
      static_cast<uint32_t>((second >> 20) & 1023ULL) +
      static_cast<uint32_t>((second >> 30) & 1023ULL) +
      static_cast<uint32_t>((second >> 40) & 1023ULL) +
      static_cast<uint32_t>((second >> 50) & 1023ULL);
  return (static_cast<int>(sum) - 6138) * (1.0f / 1024.0f);
}

/** @brief Parallel shot/read noise + sRGB encoding for a row range. */
CAMERA_LENS_SIMD void encode_sensor_rows(const cv::Mat& linear, cv::Mat& output,
                                         int start_row, int end_row,
                                         float well, float read_variance,
                                         uint64_t seed,
                                         const unsigned char* encode) {
  const int values_per_row = linear.cols * 3;
  for (int row = start_row; row < end_row; ++row) {
    const float* values = linear.ptr<float>(row);
    unsigned char* destination = output.ptr(row);
    if (well > 0) {
      const uint64_t row_seed =
          seed ^ (static_cast<uint64_t>(row) * 0xd1342543de82ef95ULL);
      for (int index = 0; index < values_per_row; ++index) {
        const float electrons = std::max(values[index], 0.0f) * well;
        const float noisy =
            electrons >= well
                ? 1.0f
                : (electrons + stateless_normal(row_seed, index) *
                                   std::sqrt(electrons + read_variance)) /
                      well;
        destination[index] = encode[static_cast<size_t>(
            std::clamp(noisy, 0.0f, 1.0f) * 65535.0f + 0.5f)];
      }
    } else {
      for (int index = 0; index < values_per_row; ++index)
        destination[index] = encode[static_cast<size_t>(
            std::clamp(values[index], 0.0f, 1.0f) * 65535.0f + 0.5f)];
    }
  }
}

/** @brief Parallel luma-only unsharp mask (sigma = 1 px) preserving chroma.
 * @details Adding the luma high-pass delta equally to R, G, and B shifts Y
 * while leaving YCrCb chroma differences (R - Y, B - Y) unchanged, avoiding
 * full-frame colour-space splits and merges. */
CAMERA_LENS_SIMD void sharpen_luma_rgb(cv::Mat& output, double sharpen) {
  const int width = output.cols, height = output.rows;
  cv::Mat luma(height, width, CV_8U), horizontal(height, width, CV_16U);
  static constexpr int weights[7] = {5, 55, 248, 408, 248, 55, 5};
  const int left_limit = std::min(3, width),
            right_start = std::max(left_limit, width - 3);
  cv::parallel_for_(cv::Range(0, height), [&](const cv::Range& range) {
    for (int row = range.start; row < range.end; ++row) {
      const unsigned char* rgb = output.ptr(row);
      unsigned char* y = luma.ptr(row);
      for (int column = 0; column < width; ++column)
        y[column] = static_cast<unsigned char>(
            (77 * rgb[column * 3] + 150 * rgb[column * 3 + 1] +
             29 * rgb[column * 3 + 2] + 128) >>
            8);
      uint16_t* filtered = horizontal.ptr<uint16_t>(row);
      for (int column = 0; column < left_limit; ++column) {
        int sum = 0;
        for (int tap = -3; tap <= 3; ++tap)
          sum += y[reflect_101(column + tap, width)] * weights[tap + 3];
        filtered[column] = static_cast<uint16_t>((sum + 16) >> 5);
      }
      for (int column = left_limit; column < right_start; ++column) {
        const int sum = y[column - 3] * 5 + y[column - 2] * 55 +
                        y[column - 1] * 248 + y[column] * 408 +
                        y[column + 1] * 248 + y[column + 2] * 55 +
                        y[column + 3] * 5;
        filtered[column] = static_cast<uint16_t>((sum + 16) >> 5);
      }
      for (int column = right_start; column < width; ++column) {
        int sum = 0;
        for (int tap = -3; tap <= 3; ++tap)
          sum += y[reflect_101(column + tap, width)] * weights[tap + 3];
        filtered[column] = static_cast<uint16_t>((sum + 16) >> 5);
      }
    }
  });
  const int amount_q8 = static_cast<int>(std::nearbyint(sharpen * 256.0));
  cv::parallel_for_(cv::Range(0, height), [&](const cv::Range& range) {
    for (int row = range.start; row < range.end; ++row) {
      const uint16_t* taps[7];
      for (int tap = -3; tap <= 3; ++tap)
        taps[tap + 3] = horizontal.ptr<uint16_t>(reflect_101(row + tap, height));
      const unsigned char* y = luma.ptr(row);
      unsigned char* rgb = output.ptr(row);
      for (int column = 0; column < width; ++column) {
        const int blurred_q5 =
            (taps[0][column] * 5 + taps[1][column] * 55 +
             taps[2][column] * 248 + taps[3][column] * 408 +
             taps[4][column] * 248 + taps[5][column] * 55 +
             taps[6][column] * 5 + 512) >>
            10;
        const int delta =
            (((static_cast<int>(y[column]) << 5) - blurred_q5) * amount_q8 +
             4096) >>
            13;
        rgb[column * 3] = cv::saturate_cast<unsigned char>(rgb[column * 3] + delta);
        rgb[column * 3 + 1] =
            cv::saturate_cast<unsigned char>(rgb[column * 3 + 1] + delta);
        rgb[column * 3 + 2] =
            cv::saturate_cast<unsigned char>(rgb[column * 3 + 2] + delta);
      }
    }
  });
}

/** Sensor and ISP stage applied after optics and exposure integration.
 * @details Order follows a real capture: optical PSF on linear light, photon
 * shot and read noise on the integrated signal with full-well clipping,
 * sRGB encoding, then ISP luma sharpening before JPEG compression. The
 * parameters are effective values that reproduce the saved JPEG, so they
 * include the camera ISP's denoising; calibrate them with --measure. */
struct Development {
  double blur, full_well, read_noise, sharpen;
  uint64_t seed;
  /** @brief Read effect settings and derive a reproducible per-shot seed. */
  Development(const Json& profile, const Json& shot)
      : blur(profile["optical_blur_px"].get<double>()),
        full_well(profile["sensor_full_well_e"].get<double>()),
        read_noise(profile["sensor_read_noise_e"].get<double>()),
        sharpen(profile["isp_sharpen"].get<double>()),
        seed(0) {
    Json noise_profile = profile;
    noise_profile.set("threads", nullptr);
    noise_profile.set("memory_budget_mb", nullptr);
    const std::string identity = noise_profile.dump_sorted() + shot.dump_sorted();
    std::array<char, SHA256_HEX_SIZE> hash{};
    sha256_hex(identity.data(), identity.size(), hash.data());
    seed = std::stoull(std::string(hash.data(), 16), nullptr, 16);
  }
  /** @brief Extra full-frame bytes per pixel used by enabled effects. */
  size_t bytes_per_pixel() const {
    return (blur > 0 ? 12 : 0) + (sharpen > 0 ? 3 : 0);
  }
  /** @brief Develop a linear RGB frame into the saved 8-bit RGB image. */
  cv::Mat develop(cv::Mat& linear) const {
    if (blur > 0) blur_linear_rgb(linear, blur);
    cv::Mat output(linear.size(), CV_8UC3);
    const unsigned char* encode = srgb_encoding().data();
    const float well = static_cast<float>(full_well),
                read_variance = static_cast<float>(read_noise * read_noise);
    cv::parallel_for_(cv::Range(0, linear.rows), [&](const cv::Range& range) {
      encode_sensor_rows(linear, output, range.start, range.end, well,
                         read_variance, seed, encode);
    });
    if (sharpen > 0) sharpen_luma_rgb(output, sharpen);
    return output;
  }
};

/** Precomputed row-and-sample ground projection (east/north in metres). */
struct RowSamplePose {
  float east_x, east_y, east_z;
  float north_x, north_y, north_z;
  float down_x, down_y, down_z;
  float translation_east, translation_north;
};

/** Precomputed Clean-JPEG pinhole reprojection for cache-free renders. */
struct SourcePinhole {
  double f_north, f_east, f_bias;
  double r_north, r_east, r_bias;
  double d_north, d_east, d_bias;
  double focal_x, focal_y, center_x, center_y;
  int cols, rows;
};

/** @brief Bilinearly sample one channel from a 2D pointer grid of 256x256
 * linear float32 RGB tiles without assembling a strip mosaic. */
inline float sample_tile_channel(const float* const* tile_ptrs, int grid_width,
                                 int max_x, int max_y, float map_x, float map_y,
                                 int channel) {
  const int sx = std::clamp(static_cast<int>(std::floor(map_x * 32.0f + 0.5f)),
                            0, max_x * 32);
  const int sy = std::clamp(static_cast<int>(std::floor(map_y * 32.0f + 0.5f)),
                            0, max_y * 32);
  const int ix = sx >> 5, iy = sy >> 5;
  const float fx = static_cast<float>(sx & 31) * (1.0f / 32.0f),
              fy = static_cast<float>(sy & 31) * (1.0f / 32.0f);
  const int tx = ix >> 8, ty = iy >> 8, lx = ix & 255, ly = iy & 255;
  if (lx < 255 && ly < 255) {
    const float* top =
        tile_ptrs[ty * grid_width + tx] + (ly * 256 + lx) * 3 + channel;
    const float* bottom = top + 256 * 3;
    const float upper = top[0] + fx * (top[3] - top[0]);
    const float lower = bottom[0] + fx * (bottom[3] - bottom[0]);
    return upper + fy * (lower - upper);
  }
  const int ix1 = std::min(ix + 1, max_x), iy1 = std::min(iy + 1, max_y);
  const float v00 = tile_ptrs[(iy >> 8) * grid_width + (ix >> 8)]
                             [((iy & 255) * 256 + (ix & 255)) * 3 + channel];
  const float v01 = tile_ptrs[(iy >> 8) * grid_width + (ix1 >> 8)]
                             [((iy & 255) * 256 + (ix1 & 255)) * 3 + channel];
  const float v10 = tile_ptrs[(iy1 >> 8) * grid_width + (ix >> 8)]
                             [((iy1 & 255) * 256 + (ix & 255)) * 3 + channel];
  const float v11 = tile_ptrs[(iy1 >> 8) * grid_width + (ix1 >> 8)]
                             [((iy1 & 255) * 256 + (ix1 & 255)) * 3 + channel];
  const float upper = v00 + fx * (v01 - v00);
  const float lower = v10 + fx * (v11 - v10);
  return upper + fy * (lower - upper);
}

/** @brief Bilinearly sample all 3 RGB channels at identical coordinates from a
 * 2D pointer grid of 256x256 linear float32 tiles. */
inline void sample_tile_rgb(const float* const* tile_ptrs, int grid_width,
                            int max_x, int max_y, float map_x, float map_y,
                            float& red, float& green, float& blue) {
  const int sx = std::clamp(static_cast<int>(std::floor(map_x * 32.0f + 0.5f)),
                            0, max_x * 32);
  const int sy = std::clamp(static_cast<int>(std::floor(map_y * 32.0f + 0.5f)),
                            0, max_y * 32);
  const int ix = sx >> 5, iy = sy >> 5;
  const float fx = static_cast<float>(sx & 31) * (1.0f / 32.0f),
              fy = static_cast<float>(sy & 31) * (1.0f / 32.0f);
  const int tx = ix >> 8, ty = iy >> 8, lx = ix & 255, ly = iy & 255;
  if (lx < 255 && ly < 255) {
    const float* top = tile_ptrs[ty * grid_width + tx] + (ly * 256 + lx) * 3;
    const float* bottom = top + 256 * 3;
    red += (top[0] + fx * (top[3] - top[0])) +
           fy * ((bottom[0] + fx * (bottom[3] - bottom[0])) -
                 (top[0] + fx * (top[3] - top[0])));
    green += (top[1] + fx * (top[4] - top[1])) +
             fy * ((bottom[1] + fx * (bottom[4] - bottom[1])) -
                   (top[1] + fx * (top[4] - top[1])));
    blue += (top[2] + fx * (top[5] - top[2])) +
            fy * ((bottom[2] + fx * (bottom[5] - bottom[2])) -
                  (top[2] + fx * (top[5] - top[2])));
    return;
  }
  const int ix1 = std::min(ix + 1, max_x), iy1 = std::min(iy + 1, max_y);
  const float* p00 =
      tile_ptrs[(iy >> 8) * grid_width + (ix >> 8)] +
      ((iy & 255) * 256 + (ix & 255)) * 3;
  const float* p01 =
      tile_ptrs[(iy >> 8) * grid_width + (ix1 >> 8)] +
      ((iy & 255) * 256 + (ix1 & 255)) * 3;
  const float* p10 =
      tile_ptrs[(iy1 >> 8) * grid_width + (ix >> 8)] +
      ((iy1 & 255) * 256 + (ix & 255)) * 3;
  const float* p11 =
      tile_ptrs[(iy1 >> 8) * grid_width + (ix1 >> 8)] +
      ((iy1 & 255) * 256 + (ix1 & 255)) * 3;
  red += (p00[0] + fx * (p01[0] - p00[0])) +
         fy * ((p10[0] + fx * (p11[0] - p10[0])) -
               (p00[0] + fx * (p01[0] - p00[0])));
  green += (p00[1] + fx * (p01[1] - p00[1])) +
           fy * ((p10[1] + fx * (p11[1] - p10[1])) -
                 (p00[1] + fx * (p01[1] - p00[1])));
  blue += (p00[2] + fx * (p01[2] - p00[2])) +
          fy * ((p10[2] + fx * (p11[2] - p10[2])) -
                (p00[2] + fx * (p01[2] - p00[2])));
}

/** @brief Bilinearly sample a linear CV_32FC3 pinhole source image matching
 * OpenCV INTER_LINEAR + BORDER_CONSTANT(0). */
inline float sample_pinhole_channel(const cv::Mat& source, float map_x,
                                    float map_y, int channel) {
  const int sx = static_cast<int>(std::floor(map_x * 32.0f + 0.5f)),
            sy = static_cast<int>(std::floor(map_y * 32.0f + 0.5f));
  const int ix = sx >> 5, iy = sy >> 5;
  const float fx = static_cast<float>(sx & 31) * (1.0f / 32.0f),
              fy = static_cast<float>(sy & 31) * (1.0f / 32.0f);
  if (ix >= 0 && ix + 1 < source.cols && iy >= 0 && iy + 1 < source.rows) {
    const float* top = source.ptr<float>(iy) + ix * 3 + channel;
    const float* bottom = source.ptr<float>(iy + 1) + ix * 3 + channel;
    const float upper = top[0] + fx * (top[3] - top[0]);
    const float lower = bottom[0] + fx * (bottom[3] - bottom[0]);
    return upper + fy * (lower - upper);
  }
  auto fetch = [&](int x, int y) -> float {
    if (x < 0 || x >= source.cols || y < 0 || y >= source.rows) return 0.0f;
    return source.ptr<float>(y)[x * 3 + channel];
  };
  const float v00 = fetch(ix, iy),
              v01 = fx > 0.0f ? fetch(ix + 1, iy) : 0.0f,
              v10 = fy > 0.0f ? fetch(ix, iy + 1) : 0.0f,
              v11 = (fx > 0.0f && fy > 0.0f) ? fetch(ix + 1, iy + 1) : 0.0f;
  const float upper = v00 + fx * (v01 - v00);
  const float lower = v10 + fx * (v11 - v10);
  return upper + fy * (lower - upper);
}

/** @brief SIMD-friendly strip ground-bounds and horizon check. */
CAMERA_LENS_SIMD void compute_strip_bounds_rows(
    const std::vector<std::unique_ptr<RayField>>& fields, int channels,
    const RowSamplePose* poses, int samples, int first, int start_row,
    int end_row, double* min_e, double* max_e, double* min_n, double* max_n,
    std::atomic<bool>& horizon, std::atomic<bool>& non_finite) {
  int max_nodes = 0;
  for (int channel = 0; channel < channels; ++channel)
    max_nodes = std::max(max_nodes, fields[channel]->node_columns());
  std::vector<float> rx_row(max_nodes), ry_row(max_nodes);
  for (int r = start_row; r < end_row; ++r) {
    if (horizon.load(std::memory_order_relaxed) ||
        non_finite.load(std::memory_order_relaxed))
      return;
    const int full_row = first + r;
    const RowSamplePose* row_poses =
        poses + static_cast<size_t>(full_row) * samples;
    float re_min = INFINITY, re_max = -INFINITY, rn_min = INFINITY,
          rn_max = -INFINITY;
    for (int channel = 0; channel < channels; ++channel) {
      const int count = fields[channel]->node_columns();
      fields[channel]->row_nodes_into(full_row, rx_row.data(), ry_row.data());
      const float* rx = rx_row.data();
      const float* ry = ry_row.data();
      for (int sample = 0; sample < samples; ++sample) {
        const RowSamplePose& p = row_poses[sample];
        float min_down = INFINITY;
        for (int column = 0; column < count; ++column)
          min_down =
              std::min(min_down, p.down_x * rx[column] + p.down_y * ry[column] +
                                     p.down_z);
        if (!(min_down > 1e-4f)) {
          horizon.store(true, std::memory_order_relaxed);
          return;
        }
        for (int column = 0; column < count; ++column) {
          const float scale =
              1.0f / (p.down_x * rx[column] + p.down_y * ry[column] + p.down_z);
          const float east =
              (p.east_x * rx[column] + p.east_y * ry[column] + p.east_z) *
                  scale +
              p.translation_east;
          const float north =
              (p.north_x * rx[column] + p.north_y * ry[column] + p.north_z) *
                  scale +
              p.translation_north;
          re_min = east < re_min ? east : re_min;
          re_max = east > re_max ? east : re_max;
          rn_min = north < rn_min ? north : rn_min;
          rn_max = north > rn_max ? north : rn_max;
        }
      }
    }
    if (!std::isfinite(re_min) || !std::isfinite(re_max) ||
        !std::isfinite(rn_min) || !std::isfinite(rn_max)) {
      non_finite.store(true, std::memory_order_relaxed);
      return;
    }
    min_e[r] = re_min;
    max_e[r] = re_max;
    min_n[r] = rn_min;
    max_n[r] = rn_max;
  }
}

/** @brief Fused parallel strip kernel for map-cache rendering. */
CAMERA_LENS_SIMD void sample_cache_rows(
    const std::vector<std::unique_ptr<RayField>>& fields, int channels,
    const RowSamplePose* poses, int samples, int first, int start_row,
    int end_row, int width, float inv_map_scale, float base_offset_x,
    float base_offset_y, const float* const* tile_ptrs, int grid_width,
    int grid_height, float gain, float vignette_scale, cv::Mat& linear) {
  const int max_x = grid_width * 256 - 1, max_y = grid_height * 256 - 1;
  std::vector<cv::Vec2f> row_rays_0(width), row_rays_1, row_rays_2;
  if (channels == 3) {
    row_rays_1.resize(width);
    row_rays_2.resize(width);
  }
  for (int row = start_row; row < end_row; ++row) {
    const int full_row = first + row;
    const RowSamplePose* row_poses =
        poses + static_cast<size_t>(full_row) * samples;
    float* destination = linear.ptr<float>(full_row);
    fields[0]->row_into(full_row, row_rays_0.data());
    if (channels == 1) {
      for (int column = 0; column < width; ++column) {
        const float rx = row_rays_0[column][0], ry = row_rays_0[column][1];
        float red = 0, green = 0, blue = 0;
        for (int sample = 0; sample < samples; ++sample) {
          const RowSamplePose& p = row_poses[sample];
          const float scale =
              1.0f / (p.down_x * rx + p.down_y * ry + p.down_z);
          const float east =
              (p.east_x * rx + p.east_y * ry + p.east_z) * scale +
              p.translation_east;
          const float north =
              (p.north_x * rx + p.north_y * ry + p.north_z) * scale +
              p.translation_north;
          const float mx = base_offset_x + east * inv_map_scale;
          const float my = base_offset_y - north * inv_map_scale;
          sample_tile_rgb(tile_ptrs, grid_width, max_x, max_y, mx, my, red,
                          green, blue);
        }
        const float cos2 = 1.0f / (1.0f + rx * rx + ry * ry);
        const float factor =
            gain * (1.0f - vignette_scale * (1.0f - cos2 * cos2));
        destination[column * 3] = red * factor;
        destination[column * 3 + 1] = green * factor;
        destination[column * 3 + 2] = blue * factor;
      }
    } else {
      fields[1]->row_into(full_row, row_rays_1.data());
      fields[2]->row_into(full_row, row_rays_2.data());
      const cv::Vec2f* channel_rays[3] = {row_rays_0.data(), row_rays_1.data(),
                                          row_rays_2.data()};
      for (int column = 0; column < width; ++column) {
        for (int channel = 0; channel < 3; ++channel) {
          const float rx = channel_rays[channel][column][0],
                      ry = channel_rays[channel][column][1];
          float sum = 0;
          for (int sample = 0; sample < samples; ++sample) {
            const RowSamplePose& p = row_poses[sample];
            const float scale =
                1.0f / (p.down_x * rx + p.down_y * ry + p.down_z);
            const float east =
                (p.east_x * rx + p.east_y * ry + p.east_z) * scale +
                p.translation_east;
            const float north =
                (p.north_x * rx + p.north_y * ry + p.north_z) * scale +
                p.translation_north;
            const float mx = base_offset_x + east * inv_map_scale;
            const float my = base_offset_y - north * inv_map_scale;
            sum += sample_tile_channel(tile_ptrs, grid_width, max_x, max_y, mx,
                                       my, channel);
          }
          const float cos2 = 1.0f / (1.0f + rx * rx + ry * ry);
          destination[column * 3 + channel] =
              sum * (gain * (1.0f - vignette_scale * (1.0f - cos2 * cos2)));
        }
      }
    }
  }
}

/** Render bounded image strips and return the assumptions/resource evidence
 * used.
 * @param input Unmodified clean RGB source (may be empty when @p cache_path is
 * set and @p source_size supplies the input dimensions).
 * @param shot Validated flight pose.
 * @param geometry Source-camera geometry.
 * @param profile Validated lens/effect parameters.
 * @param cache_path Optional offline tile-cache directory.
 * @param resources Active worker resource policy.
 * @param source_size Optional clean-input dimensions when @p input was
 * header-probed without scanline decompression.
 * @return Corrected pixels and quantitative provenance fields. */
Rendered render(const cv::Mat& input, const Json& shot, const Json& geometry,
                const Json& profile, const std::string& cache_path,
                const Resources& resources, cv::Size source_size = {}) {
  const int source_width = input.empty() ? source_size.width : input.cols;
  const int source_height = input.empty() ? source_size.height : input.rows;
  const int width = profile["output_width"].is_null()
                        ? source_width
                        : profile["output_width"].get<int>();
  const int height = profile["output_height"].is_null()
                         ? source_height
                         : profile["output_height"].get<int>();
  const Lens lens(width, height, profile);
  const Pose pose(shot, profile);
  const bool cache = !cache_path.empty();
  require(cache || !input.empty(), "missing decoded clean input");
  require(cache || profile["height_override_m"].is_null(),
          "height override requires a map cache");
  const double aberration = profile["chromatic_aberration"].get<double>();
  const int channels = aberration == 0 ? 1 : 3;
  const Development development(profile, shot);
  const size_t input_pixels =
      static_cast<size_t>(source_width) * static_cast<size_t>(source_height);
  const size_t frame_bytes =
      32 * mib + input_pixels * (cache ? (input.empty() ? 0 : 3) : 15) +
      static_cast<size_t>(width) * height *
          (3 + 12 + development.bytes_per_pixel());
  require(frame_bytes <= resources.budget,
          "lens working set exceeds memory budget");
  usher_cv_parallel::ensure_installed();
  cv::setNumThreads(resources.threads);
  std::vector<std::unique_ptr<RayField>> fields(channels);
  size_t field_bytes = 0;
  if (channels == 1) {
    require(field_bytes < resources.budget - frame_bytes,
            "ray grid exceeds memory budget");
    fields[0] = std::make_unique<RayField>(lens, width, height, 1.0,
                                           resources.budget - frame_bytes);
    field_bytes = fields[0]->bytes();
  } else {
    const size_t per_field_budget = (resources.budget - frame_bytes) / 3;
    std::atomic<bool> failed{false};
    std::string failure;
    cv::parallel_for_(cv::Range(0, 3), [&](const cv::Range& range) {
      for (int channel = range.start; channel < range.end; ++channel) {
        try {
          const double scale =
              channel == 0 ? 1 + aberration : (channel == 2 ? 1 - aberration : 1);
          fields[channel] = std::make_unique<RayField>(
              lens, width, height, scale, per_field_budget);
        } catch (const std::exception& error) {
          if (!failed.exchange(true)) failure = error.what();
        }
      }
    });
    if (failed) throw std::runtime_error(failure);
    for (const auto& field : fields) field_bytes += field->bytes();
  }
  const RayField& reference = *fields[channels == 1 ? 0 : 1];
  const TemporalSampling sampling = temporal_sampling(
      reference, pose, width, height, profile["samples"].get<int>());
  const int samples = sampling.samples;
  const int strip_rows = std::min(32, std::max(1, 524288 / (width * samples)));
  const size_t pose_bytes =
      static_cast<size_t>(height) * samples * sizeof(RowSamplePose);
  const size_t base_bytes =
      frame_bytes + field_bytes + pose_bytes +
      static_cast<size_t>(resources.threads) * width * channels * sizeof(cv::Vec2f);
  require(base_bytes <= resources.budget,
          "lens working set exceeds memory budget");
  size_t peak_bytes = base_bytes;

  // Precompute time-dependent camera rotation and ground translation once
  // across (row, sample); all chromatic-aberration channels share this table.
  std::vector<RowSamplePose> poses(static_cast<size_t>(height) * samples);
  const float altitude_f = static_cast<float>(pose.altitude);
  const float cos_course = static_cast<float>(std::cos(pose.course)),
              sin_course = static_cast<float>(std::sin(pose.course));
  for (int row = 0; row < height; ++row) {
    for (int sample = 0; sample < samples; ++sample) {
      const double sample_offset =
          ((sample + 0.5) / samples - 0.5) * pose.exposure;
      const float instant =
          static_cast<float>(
              (static_cast<double>(row) / (height - 1) - 0.5) * pose.readout) +
          static_cast<float>(sample_offset);
      const double cycle = 2 * pi * pose.frequency * instant + pose.phase;
      const double roll = (pose.angles[0] + pose.rates[0] * instant +
                           pose.vibration * std::sin(cycle)) *
                          pi / 180;
      const double pitch = (pose.angles[1] + pose.rates[1] * instant +
                            pose.vibration * std::sin(cycle + pi / 2)) *
                           pi / 180;
      const double yaw = (pose.angles[2] + pose.rates[2] * instant) * pi / 180;
      const float cr = std::cos(roll), sr = std::sin(roll),
                  cp = std::cos(pitch), sp = std::sin(pitch),
                  cy = std::cos(yaw), sy = std::sin(yaw);
      const float nf = cy * cp, nr = cy * sp * sr - sy * cr,
                  nd = cy * sp * cr + sy * sr;
      const float ef = sy * cp, er = sy * sp * sr + cy * cr,
                  ed = sy * sp * cr - cy * sr;
      RowSamplePose& entry = poses[static_cast<size_t>(row) * samples + sample];
      // forward = -ray_y, right = ray_x
      entry.east_x = er * altitude_f;
      entry.east_y = -ef * altitude_f;
      entry.east_z = ed * altitude_f;
      entry.north_x = nr * altitude_f;
      entry.north_y = -nf * altitude_f;
      entry.north_z = nd * altitude_f;
      entry.down_x = cp * sr;
      entry.down_y = sp;
      entry.down_z = cp * cr;
      entry.translation_north = static_cast<float>(pose.speed) * instant * cos_course;
      entry.translation_east = static_cast<float>(pose.speed) * instant * sin_course;
    }
  }

  cv::Mat linear(height, width, CV_32FC3);
  // Residual relative illumination after ISP shading correction: cos^4
  // falloff shape, scaled so the image corner loses exactly `vignetting`.
  const cv::Vec2f corner_ray = reference.at(0, 0);
  const double corner_cos2 = 1 / (1 + corner_ray.dot(corner_ray));
  const double corner_falloff = 1 - corner_cos2 * corner_cos2;
  const float vignette_scale =
      corner_falloff > 1e-9
          ? static_cast<float>(profile["vignetting"].get<double>() / corner_falloff)
          : 0.0f;
  const float gain =
      static_cast<float>(std::pow(2.0, profile["exposure_ev"].get<double>()) / samples);

  Bounds bounds{};
  bool have_bounds = false;
  if (cache) {
    const double latitude = shot["lat_deg"].get<double>() * pi / 180;
    const double world = 256.0 * (1 << 18);
    const double center_x = (shot["lon_deg"].get<double>() + 180) / 360 * world;
    const double center_y =
        (1 - std::asinh(std::tan(latitude)) / pi) * world / 2;
    const double map_scale = 2 * pi * 6378137 * std::cos(latitude) / world;
    const float inv_map_scale = static_cast<float>(1.0 / map_scale);
    const int64_t base_tile_x = static_cast<int64_t>(std::floor(center_x / 256.0));
    const int64_t base_tile_y = static_cast<int64_t>(std::floor(center_y / 256.0));
    const float local_center_x = static_cast<float>(center_x - base_tile_x * 256.0);
    const float local_center_y = static_cast<float>(center_y - base_tile_y * 256.0);
    const int max_tiles = profile["max_tiles"].get<int>();
    TileCache tiles(cache_path, resources.budget);

    for (int first = 0; first < height; first += strip_rows) {
      const int rows = std::min(strip_rows, height - first);
      std::vector<double> min_e(rows, INFINITY), max_e(rows, -INFINITY),
          min_n(rows, INFINITY), max_n(rows, -INFINITY);
      std::atomic<bool> horizon{false}, non_finite{false};
      cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
        compute_strip_bounds_rows(fields, channels, poses.data(), samples,
                                  first, range.start, range.end, min_e.data(),
                                  max_e.data(), min_n.data(), max_n.data(),
                                  horizon, non_finite);
      });
      require(!horizon, "lens/exposure rays reach the horizon");
      require(!non_finite, "non-finite ground projection");
      double minimum_east = INFINITY, maximum_east = -INFINITY,
             minimum_north = INFINITY, maximum_north = -INFINITY;
      for (int r = 0; r < rows; ++r) {
        minimum_east = std::min(minimum_east, min_e[r]);
        maximum_east = std::max(maximum_east, max_e[r]);
        minimum_north = std::min(minimum_north, min_n[r]);
        maximum_north = std::max(maximum_north, max_n[r]);
      }
      const Bounds region = {
          static_cast<int64_t>(
              std::floor((center_x + minimum_east / map_scale) / 256)),
          static_cast<int64_t>(
              std::floor((center_y - maximum_north / map_scale) / 256)),
          static_cast<int64_t>(
              std::floor((center_x + maximum_east / map_scale + 1) / 256)),
          static_cast<int64_t>(
              std::floor((center_y - minimum_north / map_scale + 1) / 256))};
      bounds = have_bounds ? Bounds{std::min(bounds.left, region.left),
                                    std::min(bounds.top, region.top),
                                    std::max(bounds.right, region.right),
                                    std::max(bounds.bottom, region.bottom)}
                           : region;
      have_bounds = true;
      const size_t total_tile_count = bounds.count(max_tiles);
      const size_t strip_tile_count = region.count(max_tiles);
      const size_t resident_tiles =
          std::max(strip_tile_count,
                   std::min(total_tile_count, tile_cache_entries));
      const size_t required =
          base_bytes + resident_tiles * 256 * 256 * sizeof(cv::Vec3f) +
          strip_tile_count * (sizeof(cv::Mat) + sizeof(const float*));
      require(required <= resources.budget,
              "lens tile strip exceeds memory budget");
      peak_bytes = std::max(peak_bytes, required);

      const int grid_width = static_cast<int>(region.right - region.left + 1);
      const int grid_height = static_cast<int>(region.bottom - region.top + 1);
      std::vector<cv::Mat> strip_tiles(
          static_cast<size_t>(grid_width) * grid_height);
      std::vector<const float*> tile_ptrs(strip_tiles.size());
      for (int ty = 0; ty < grid_height; ++ty) {
        for (int tx = 0; tx < grid_width; ++tx) {
          const size_t slot = static_cast<size_t>(ty) * grid_width + tx;
          strip_tiles[slot] = tiles.tile(region.left + tx, region.top + ty);
          tile_ptrs[slot] = strip_tiles[slot].ptr<float>();
        }
      }
      const float base_offset_x =
          local_center_x + static_cast<float>((base_tile_x - region.left) * 256);
      const float base_offset_y =
          local_center_y + static_cast<float>((base_tile_y - region.top) * 256);
      cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
        sample_cache_rows(fields, channels, poses.data(), samples, first,
                          range.start, range.end, width, inv_map_scale,
                          base_offset_x, base_offset_y, tile_ptrs.data(),
                          grid_width, grid_height, gain, vignette_scale,
                          linear);
      });
    }
    for (int64_t row = bounds.top; row <= bounds.bottom; ++row)
      for (int64_t column = bounds.left; column <= bounds.right; ++column)
        tiles.verify(column, row);
  } else {
    cv::Mat linear_source(input.rows, input.cols, CV_32FC3);
    const float* decode = srgb_decoding().data();
    cv::parallel_for_(cv::Range(0, input.rows), [&](const cv::Range& range) {
      for (int row = range.start; row < range.end; ++row) {
        const unsigned char* src = input.ptr(row);
        float* dst = linear_source.ptr<float>(row);
        for (int index = 0; index < input.cols * 3; ++index)
          dst[index] = decode[src[index]];
      }
    });
    const double roll = shot["roll_deg"].get<double>() * pi / 180,
                 pitch = shot["pitch_deg"].get<double>() * pi / 180,
                 yaw = shot["yaw_deg"].get<double>() * pi / 180,
                 shot_altitude = shot["groundalt_m"].get<double>();
    const double cr = std::cos(roll), sr = std::sin(roll),
                 cp = std::cos(pitch), sp = std::sin(pitch),
                 cy = std::cos(yaw), sy = std::sin(yaw);
    const SourcePinhole pinhole{
        cy * cp,
        sy * cp,
        -sp * shot_altitude,
        cy * sp * sr - sy * cr,
        sy * sp * sr + cy * cr,
        cp * sr * shot_altitude,
        cy * sp * cr + sy * sr,
        sy * sp * cr - cy * sr,
        cp * cr * shot_altitude,
        (input.cols - 1) /
            (2 * std::tan(geometry["horizontal_fov_deg"].get<double>() * pi / 360)),
        (input.rows - 1) /
            (2 * std::tan(geometry["vertical_fov_deg"].get<double>() * pi / 360)),
        (input.cols - 1) * 0.5,
        (input.rows - 1) * 0.5,
        input.cols,
        input.rows};

    for (int first = 0; first < height; first += strip_rows) {
      const int rows = std::min(strip_rows, height - first);
      std::atomic<int> error_code{0};
      cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
        std::vector<cv::Vec2f> row_rays_0(width), row_rays_1, row_rays_2;
        if (channels == 3) {
          row_rays_1.resize(width);
          row_rays_2.resize(width);
        }
        for (int r = range.start; r < range.end; ++r) {
          if (error_code.load(std::memory_order_relaxed) != 0) return;
          const int full_row = first + r;
          const RowSamplePose* row_poses =
              poses.data() + static_cast<size_t>(full_row) * samples;
          fields[0]->row_into(full_row, row_rays_0.data());
          if (channels == 3) {
            fields[1]->row_into(full_row, row_rays_1.data());
            fields[2]->row_into(full_row, row_rays_2.data());
          }
          const cv::Vec2f* channel_rays[3] = {
              row_rays_0.data(),
              channels == 3 ? row_rays_1.data() : row_rays_0.data(),
              channels == 3 ? row_rays_2.data() : row_rays_0.data()};
          float* destination = linear.ptr<float>(full_row);
          for (int column = 0; column < width; ++column) {
            for (int c = 0; c < channels; ++c) {
              const float rx = channel_rays[c][column][0],
                          ry = channel_rays[c][column][1];
              float acc0 = 0, acc1 = 0, acc2 = 0;
              for (int sample = 0; sample < samples; ++sample) {
                const RowSamplePose& p = row_poses[sample];
                const float down_ray = p.down_x * rx + p.down_y * ry + p.down_z;
                if (!(down_ray > 1e-4f)) {
                  int expected = 0;
                  error_code.compare_exchange_strong(expected, 1);
                  return;
                }
                const float scale = 1.0f / down_ray;
                const float east =
                    (p.east_x * rx + p.east_y * ry + p.east_z) * scale +
                    p.translation_east;
                const float north =
                    (p.north_x * rx + p.north_y * ry + p.north_z) * scale +
                    p.translation_north;
                if (!std::isfinite(east) || !std::isfinite(north)) {
                  int expected = 0;
                  error_code.compare_exchange_strong(expected, 2);
                  return;
                }
                const double forward = pinhole.f_north * north +
                                       pinhole.f_east * east + pinhole.f_bias;
                const double right = pinhole.r_north * north +
                                     pinhole.r_east * east + pinhole.r_bias;
                const double down = pinhole.d_north * north +
                                    pinhole.d_east * east + pinhole.d_bias;
                if (!(down > 1e-4)) {
                  int expected = 0;
                  error_code.compare_exchange_strong(expected, 3);
                  return;
                }
                const float mx = static_cast<float>(
                    right / down * pinhole.focal_x + pinhole.center_x);
                const float my = static_cast<float>(
                    -forward / down * pinhole.focal_y + pinhole.center_y);
                if (!(mx >= -0.001f && mx <= pinhole.cols - 1 + 0.001f &&
                      my >= -0.001f && my <= pinhole.rows - 1 + 0.001f)) {
                  int expected = 0;
                  error_code.compare_exchange_strong(expected, 4);
                  return;
                }
                if (channels == 1) {
                  acc0 += sample_pinhole_channel(linear_source, mx, my, 0);
                  acc1 += sample_pinhole_channel(linear_source, mx, my, 1);
                  acc2 += sample_pinhole_channel(linear_source, mx, my, 2);
                } else {
                  acc0 += sample_pinhole_channel(linear_source, mx, my, c);
                }
              }
              const float cos2 = 1.0f / (1.0f + rx * rx + ry * ry);
              const float factor =
                  gain * (1.0f - vignette_scale * (1.0f - cos2 * cos2));
              if (channels == 1) {
                destination[column * 3] = acc0 * factor;
                destination[column * 3 + 1] = acc1 * factor;
                destination[column * 3 + 2] = acc2 * factor;
              } else {
                destination[column * 3 + c] = acc0 * factor;
              }
            }
          }
        }
      });
      const int code = error_code.load();
      require(code != 1, "lens/exposure rays reach the horizon");
      require(code != 2, "non-finite ground projection");
      require(code != 3, "view is behind source camera");
      require(code != 4, "view exceeds source image; supply --cache");
    }
  }
  const cv::Mat output = development.develop(linear);
  const double relative_course =
      pose.course - shot["yaw_deg"].get<double>() * pi / 180;
  const double blur = std::hypot(lens.focal_x * std::sin(relative_course),
                                 lens.focal_y * std::cos(relative_course)) *
                      pose.speed * pose.exposure / pose.altitude;
  return {
      output,
      Json::object(
          {{"used_agl_m", pose.altitude},
           {"shot_agl_m", shot["groundalt_m"]},
           {"renderer", "native_opencv_bounded_strips_v2"},
           {"effective_threads", resources.threads},
           {"effective_samples", samples},
           {"configured_max_samples", profile["samples"]},
           {"exposure_blur_px_max", sampling.blur_px},
           {"ray_grid_step_px", reference.step},
           {"ray_interpolation_error_px", reference.error_px},
           {"sensor_model", development.full_well > 0
                                ? "gaussian_shot_and_read_noise"
                                : "noise_free"},
           {"noise_seed", std::to_string(development.seed)},
           {"strip_rows", strip_rows},
           {"memory_budget_bytes", resources.budget},
           {"estimated_peak_bytes", peak_bytes},
           {"center_translation_blur_px_level_estimate", blur},
           {"source", cache ? "cache" : "pinhole_jpeg"},
           {"tile_bounds_unwrapped",
            cache ? Json::array(
                        {bounds.left, bounds.top, bounds.right, bounds.bottom})
                  : Json(nullptr)},
           {"effective_readout_s", std::abs(pose.readout)},
           {"motion_model",
            "constant_ground_velocity_and_configured_euler_rates"},
           {"radiometry", "srgb_linearized_relative_EV_not_sensor_raw"},
           {"vibration_model",
            "configured_sinusoidal_roll_pitch_not_measured"},
           {"time_reference", "shot_pose_at_center_row_mid_exposure"}})};
}

/** Apply one profile and preserve the clean source while writing a tagged
 * derivative.
 * @param input Original clean JPEG.
 * @param output New derivative path.
 * @param profile Validated profile.
 * @param cache Optional tile-cache directory.
 * @return EXIF evidence plus run-time/resource measurements. */
Json emulate(const std::string& input, const std::string& output,
             const Json& profile, const std::string& cache) {
  struct stat original{}, destination{};
  require(stat(input.c_str(), &original) == 0, "missing clean input");
  require(stat(output.c_str(), &destination) != 0 ||
              original.st_dev != destination.st_dev ||
              original.st_ino != destination.st_ino,
          "output must differ from clean source");
  const auto started = std::chrono::steady_clock::now();
  const Resources resources(profile);
  JpegHeader source_header{};
  cv::Mat image = read_jpeg(input, false, resources.budget, &source_header,
                            cache.empty());
  auto metadata = std::make_unique<exif_document>();
  std::array<unsigned char, EXIF_MAX_PAYLOAD> payload;
  size_t payload_size = 0;
  exif_view view{};
  require(exif_read_jpeg(input.c_str(), payload.data(), payload.size(),
                         &payload_size) == 0 &&
              exif_parse(&view, payload.data(), payload_size) == 0 &&
              image_exif_import(metadata.get(), &view) == 0,
          "missing or invalid EXIF metadata");
  Json geometry;
  const Json shot = read_shot(metadata.get(), geometry);
  Rendered result =
      render(image, shot, geometry, profile, cache, resources,
             cv::Size(source_header.width, source_header.height));
  const std::string serialized = profile.dump_sorted();
  std::array<char, SHA256_HEX_SIZE> hash{};
  sha256_hex(serialized.data(), serialized.size(), hash.data());
  Json record = Json::object({{"schema", "USHER_LENS_V1"},   {"simulated", true},
                 {"profile", profile},          {"profile_sha256", hash.data()},
                 {"source_geometry", geometry}, {"opencv", CV_VERSION}});
  record.update(result.evidence);
  const std::string record_text = record.dump(-1, ' ', true);
  set_text(metadata.get(), EXIF_PRIMARY, EXIF_DOCUMENT_NAME, record_text);
  set_text(metadata.get(), EXIF_PRIMARY, EXIF_SOFTWARE,
           "Paparazzi USHER native simulated lens / OpenCV");
  set_text(metadata.get(), EXIF_CAMERA, EXIF_LENS_MODEL,
           "SIMULATED " + profile["name"].get<std::string>());
  set_rational(metadata.get(), EXIF_EXPOSURE_TIME, profile["exposure_s"].get<double>(),
               1000000);
  set_rational(metadata.get(), EXIF_FNUMBER, profile["f_number"].get<double>(), 100);
  set_rational(metadata.get(), EXIF_EXPOSURE_BIAS,
               profile["exposure_ev"].get<double>(), 1000, true);
  if (!profile["focal_length_mm"].is_null())
    set_rational(metadata.get(), EXIF_FOCAL_LENGTH,
                 profile["focal_length_mm"].get<double>(), 1000);
  else {
        require(exif_document_remove(metadata.get(), EXIF_CAMERA, EXIF_FOCAL_LENGTH) == 0,
          "cannot remove focal length");
  }
  for (const auto& dimension :
         {std::make_pair(EXIF_PIXEL_X_DIMENSION, result.image.cols),
        std::make_pair(EXIF_PIXEL_Y_DIMENSION, result.image.rows)}) {
      unsigned char* entry = replace_tag(metadata.get(), EXIF_CAMERA,
                         dimension.first, EXIF_LONG, 1);
      exif_encode_long(entry, static_cast<uint32_t>(dimension.second));
  }
  std::string comment =
        tag_text(metadata.get(), EXIF_CAMERA, EXIF_USER_COMMENT);
  const std::string prefix("ASCII\0\0\0", 8);
  if (comment.empty()) comment = prefix;
  require(comment.compare(0, prefix.size(), prefix) == 0,
          "expected ASCII USHER UserComment");
  set_text(metadata.get(), EXIF_CAMERA, EXIF_USER_COMMENT,
           comment + "\nUSHER_LENS_V1\n" + record_text, true);
    write_jpeg(output, result.image, profile["jpeg_quality"].get<int>(), metadata.get());
    record.set("elapsed_s",
         std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
           .count());
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    std::istringstream fields(line);
    std::string key;
    size_t value;
    if (fields >> key >> value && key == "VmHWM:")
        record.set("process_peak_rss_kib", value);
  }
      record.set("output", output);
  return record;
}
}  // namespace camera_lens_detail

using namespace camera_lens_detail;

/** @brief Estimate the IJG quality whose scaled luma table best matches.
 * @param quant Natural-order luma quantization table.
 * @return {quality, mean absolute table error}; error 0 is an exact IJG
 * table, larger values mean a vendor table and an approximate quality. */
std::pair<int, double> ijg_quality(const std::array<unsigned, 64>& quant) {
  static const std::array<int, 64> standard = {
      16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
      14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
      18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
      49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};
  std::pair<int, double> best{0, INFINITY};
  for (int quality = 1; quality <= 100; ++quality) {
    const int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
    double error = 0;
    for (size_t index = 0; index < standard.size(); ++index)
      error += std::abs(
          std::clamp((standard[index] * scale + 50) / 100, 1, 255) -
          static_cast<int>(quant[index]));
    if (error / 64 < best.second) best = {quality, error / 64};
  }
  return best;
}

/** @brief Measure image statistics that a lens profile should reproduce.
 * @param path JPEG saved by a real or simulated camera.
 * @return Encoder, exposure, noise, sharpness, shading and colour facts.
 * @details Compare medians of real flight photos with medians of simulated
 * photos of similar terrain; single-image values are scene dependent. */
Json measure(const std::string& path) {
  JpegHeader header;
  const cv::Mat image = read_jpeg(path, false, 1024 * mib, &header);
  cv::Mat luma8, luma, gradient_x, gradient_y, magnitude, residual, laplacian;
  cv::cvtColor(image, luma8, cv::COLOR_RGB2GRAY);
  luma8.convertTo(luma, CV_32F);
  // Immerkaer (1996) noise estimate restricted to the flattest 30 percent of
  // unsaturated pixels so that scene texture is not mistaken for noise.
  float laplacian_difference[9] = {1, -2, 1, -2, 4, -2, 1, -2, 1};
  const cv::Mat kernel(3, 3, CV_32F, laplacian_difference);
  cv::filter2D(luma, residual, CV_32F, kernel);
  cv::Sobel(luma, gradient_x, CV_32F, 1, 0);
  cv::Sobel(luma, gradient_y, CV_32F, 0, 1);
  cv::magnitude(gradient_x, gradient_y, magnitude);
  std::vector<float> flat;
  flat.reserve(luma.total());
  for (int row = 1; row + 1 < luma.rows; ++row)
    for (int column = 1; column + 1 < luma.cols; ++column) {
      const unsigned char value = luma8.at<unsigned char>(row, column);
      if (value > 2 && value < 253) flat.push_back(magnitude.at<float>(row, column));
    }
  require(!flat.empty(), "image has no unsaturated pixels: " + path);
  auto cut = flat.begin() + flat.size() * 3 / 10;
  std::nth_element(flat.begin(), cut, flat.end());
  const float threshold = *cut;
  double absolute = 0;
  size_t count = 0;
  for (int row = 1; row + 1 < luma.rows; ++row)
    for (int column = 1; column + 1 < luma.cols; ++column) {
      const unsigned char value = luma8.at<unsigned char>(row, column);
      if (value > 2 && value < 253 &&
          magnitude.at<float>(row, column) <= threshold) {
        absolute += std::abs(residual.at<float>(row, column));
        ++count;
      }
    }
  cv::Laplacian(luma, laplacian, CV_32F);
  cv::Scalar mean, deviation;
  cv::meanStdDev(laplacian, mean, deviation);
  const double sharpness = deviation[0] * deviation[0];
  int histogram[256] = {};
  for (size_t index = 0; index < luma8.total(); ++index)
    ++histogram[luma8.data[index]];
  auto percentile = [&](double fraction) {
    size_t total = 0;
    for (int value = 0; value < 256; ++value)
      if ((total += histogram[value]) >= fraction * luma8.total()) return value;
    return 255;
  };
  const int patch_w = std::max(1, image.cols / 10),
            patch_h = std::max(1, image.rows / 10);
  auto patch = [&](int x, int y) {
    return cv::mean(luma(cv::Rect(x, y, patch_w, patch_h)))[0];
  };
  const double corners =
      (patch(0, 0) + patch(image.cols - patch_w, 0) +
       patch(0, image.rows - patch_h) +
       patch(image.cols - patch_w, image.rows - patch_h)) / 4;
  const double centre =
      patch((image.cols - patch_w) / 2, (image.rows - patch_h) / 2);
  const cv::Scalar colour = cv::mean(image);
  const auto quality = ijg_quality(header.luma_quant);
  return Json::object(
      {{"file", path},
       {"width", image.cols},
       {"height", image.rows},
       {"jpeg_quality_estimate", quality.first},
       {"jpeg_table_error", quality.second},
       {"chroma_subsampling", header.subsampling},
       {"noise_sigma", std::sqrt(pi / 2) * absolute / (6.0 * count)},
       {"laplacian_variance", sharpness},
       {"luma_mean", cv::mean(luma)[0]},
       {"luma_p1", percentile(0.01)},
       {"luma_p99", percentile(0.99)},
       {"corner_centre_ratio", centre > 0 ? Json(corners / centre) : Json(nullptr)},
       {"red_green_ratio", colour[1] > 0 ? Json(colour[0] / colour[1]) : Json(nullptr)},
       {"blue_green_ratio", colour[1] > 0 ? Json(colour[2] / colour[1]) : Json(nullptr)}});
}

/** @brief Measure files and append per-key medians of numeric results. */
Json measure_all(const std::vector<std::string>& paths) {
  Json files = Json::array(), median = Json::object();
  std::map<std::string, std::vector<double>> values;
  for (const auto& path : paths) {
    Json result = measure(path);
    for (const char* key :
         {"width", "height", "jpeg_quality_estimate", "noise_sigma",
          "laplacian_variance", "luma_mean", "luma_p1", "luma_p99",
          "corner_centre_ratio", "red_green_ratio", "blue_green_ratio"})
      if (!result[key].is_null()) values[key].push_back(result[key].get<double>());
    files.push_back(result);
  }
  for (auto& [key, list] : values) {
    std::sort(list.begin(), list.end());
    const size_t middle = list.size() / 2;
    median.set(key, list.size() % 2 ? list[middle]
                                    : (list[middle - 1] + list[middle]) / 2);
  }
  return Json::object({{"schema", "USHER_LENS_MEASURE_V1"},
                       {"count", static_cast<int>(paths.size())},
                       {"median", median},
                       {"files", files}});
}

/** @brief Validate a lens profile or generate a simulated camera derivative.
 * @param argc Argument count including the program name.
 * @param argv Command-line argument vector.
 * @return Zero for success, or one after reporting an exception.
 * @details Version and validation requests bypass rendering. Render jobs
 * take a nonblocking executable lock so separate jobs cannot multiply the
 * working set managed by the per-job memory budget. */
int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--version") {
      std::cout << CAMERA_LENS_BUILD_VERSION << '\n';
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "--validate-profile") {
      std::cout << read_profile(argv[2]).dump() << '\n';
      return 0;
    }
    const usher_cv_parallel::ScopedBackend opencv_parallel_backend;
    if (argc >= 3 && std::string(argv[1]) == "--measure") {
      std::cout << measure_all(std::vector<std::string>(argv + 2, argv + argc))
                       .dump(-1, ' ', true)
                << '\n';
      return 0;
    }
    File job_lock(fopen("/proc/self/exe", "rb"));
    require(job_lock && flock(fileno(job_lock.get()), LOCK_EX | LOCK_NB) == 0,
            "busy or unavailable native lens job lock");
    std::string profile, cache;
    std::vector<std::string> positional;
    for (int argument = 1; argument < argc; ++argument) {
      const std::string value(argv[argument]);
      if ((value == "--profile" || value == "--cache") && argument + 1 < argc) {
        (value == "--profile" ? profile : cache) = argv[++argument];
      } else {
        require(value.empty() || value[0] != '-',
                "unknown or incomplete lens option");
        positional.push_back(value);
      }
    }
    require(!profile.empty() && positional.size() == 2,
            "usage: camera_lens --version | --validate-profile FILE | "
            "--measure JPEG... | --profile FILE [--cache DIR] INPUT "
            "OUTPUT");
    std::cout << emulate(positional[0], positional[1],
                         read_profile(profile.c_str()), cache)
                     .dump(-1, ' ', true)
              << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "CAMERA_LENS: " << error.what() << '\n';
    return 1;
  }
}