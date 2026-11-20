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

/** @brief Request POSIX.1-2008 stream/descriptor APIs before system headers. */
#define _POSIX_C_SOURCE 200809L

/**
 * @file lwir_calibrate.cpp
 * @brief Native host tool for LWIR lens and camera-mount calibration.
 * @details Lens mode estimates pinhole intrinsics and five Brown-Conrady
 * distortion coefficients from varied checkerboard photographs. Mount mode
 * holds those intrinsics fixed and estimates camera-to-body rotation from a
 * board physically aligned with the aircraft datum. Keeping these stages
 * separate avoids interpreting lens distortion as a mounting error.
 *
 * OpenCV supplies detection, geometry and drawing; bundled libjpeg handles
 * JPEG I/O so no OpenCV imgcodecs module or additional codec dependency is
 * needed. Original pixels are neither resized nor EXIF-oriented before fitting:
 * coefficients must describe the same native coordinates used by LWIRcam.
 * Only inspection previews are enlarged and annotated.
 *
 * Each run produces a runtime-readable profile, a diagnostic report and
 * inspection previews. Failed runs remove the candidate profile but retain
 * available diagnostic evidence. Outputs are intended for a trusted workshop
 * directory, not concurrent writers competing for the same filenames.
 * @warning Numerical convergence is not physical verification. Every exported
 * profile remains unverified; corner orientation, board alignment and independent
 * measurements must be checked before relying on projected coordinates.
 * @see fit_lens
 * @see fit_mount
 */
#include "version.h"

#include <cstddef>
#include <cstdio>
#include <jpeglib.h>
#include "jpeg_marker_guard.h"
#include <setjmp.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv_parallel.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

/** @brief Filesystem operations use paths rather than shell command strings. */
namespace fs = std::filesystem;

/** @brief Compressed-input ceiling, checked before libjpeg starts decoding. */
constexpr size_t maximum_jpeg_bytes = 128U * 1024U * 1024U;
/** @brief Decoded-pixel ceiling; compressed file size alone cannot bound memory. */
constexpr uint64_t maximum_image_pixels = 32U * 1024U * 1024U;
/** @brief Per-axis limit also bounds the preview encoder's scanline scratch. */
constexpr unsigned maximum_image_dimension = 8192U;
/** @brief Accepted-image-list cap; this is not a total fitting-memory budget. */
constexpr size_t maximum_directory_images = 10000U;
/** @brief Minimum complete fitting views; count alone cannot ensure varied poses. */
constexpr size_t minimum_fit_views = 15U;
/** @brief Minimum complete hold-out views when an independent set is requested. */
constexpr size_t minimum_check_views = 5U;
/** @brief Convert the workshop's measured millimetre pitch to solver metres. */
constexpr double metres_per_millimetre = 0.001;
/** @brief Enlarge small thermal images for inspection, subject to pixel limits. */
constexpr double preferred_preview_scale = 3.0;
/** @brief Inspection JPEG quality; previews never feed back into the fit. */
constexpr int preview_jpeg_quality = 95;
/** @brief Runtime schema key kept false regardless of the numerical residual. */
constexpr const char* profile_verified_key = "verified";

/** @brief Distinguish invalid invocation from an unsuccessful calibration run. */
struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/** @brief Close C streams during normal return and C++ exception unwinding. */
struct FileCloser {
  /** @brief Release an owned stream without throwing.
   * @param file Stream to close, or nullptr for an empty owner.
   * @note Writers check flushing separately; this deleter cannot report errors.
   */
  void operator()(FILE* file) const noexcept
  {
    if (file != nullptr) {
      std::fclose(file);
    }
  }
};

/** @brief Single owner for a FILE stream, including error paths. */
using File = std::unique_ptr<FILE, FileCloser>;

/** @brief libjpeg-compatible error manager and its active C recovery target.
 * @details The manager must be first: libjpeg exposes only its base pointer to
 * the callback. Keeping codec state outside the setjmp stack frame lets C++
 * ownership clean it up after the callback returns control via longjmp.
 */
struct JpegError {
  jpeg_error_mgr manager{}; ///< Base error manager passed to libjpeg.
  jmp_buf recovery{}; ///< Recovery point active only during the codec operation.
  void (*emit)(j_common_ptr, int) = nullptr;
};

/** @brief Return a fatal libjpeg error to the enclosing decode/encode operation.
 * @param state Codec whose error manager belongs to a JpegError instance.
 * @pre That instance has an active setjmp recovery point on the current thread.
 * @details libjpeg normally terminates the process on a fatal error. This C-only
 * jump returns to a boundary that can throw a descriptive C++ exception instead;
 * no exception crosses libjpeg's C call frames.
 */
void jpeg_failure(j_common_ptr state)
{
  auto* error = reinterpret_cast<JpegError*>(state->err);
  longjmp(error->recovery, 1);
}

void jpeg_message(j_common_ptr state, int level)
{
  usher_jpeg_validate_marker(state);
  reinterpret_cast<JpegError*>(state->err)->emit(state, level);
}

/** @brief Own decoder state and its output across libjpeg's C error recovery. */
struct Decoder {
  jpeg_decompress_struct state{}; ///< libjpeg decompressor and allocation state.
  JpegError error{}; ///< Recovery target used by jpeg_failure().
  bool created = false; ///< Destroy only after codec creation has completed.
  cv::Mat gray; ///< Ref-counted native grayscale pixels, not a display transform.

  /** @brief Release codec allocations after successful creation. */
  ~Decoder()
  {
    if (created) {
      jpeg_destroy_decompress(&state);
    }
  }
};

/** @brief Own preview-compression state across normal and recovered returns. */
struct Encoder {
  jpeg_compress_struct state{}; ///< libjpeg compressor and allocation state.
  JpegError error{}; ///< Recovery target used by jpeg_failure().
  bool created = false; ///< Prevent destruction of a not-yet-created compressor.

  /** @brief Release codec allocations after successful creation. */
  ~Encoder()
  {
    if (created) {
      jpeg_destroy_compress(&state);
    }
  }
};

/** @brief Validated invocation; dimensions count inner corners, not squares. */
struct Options {
  /** @brief Independent stages with different physical evidence requirements.
   * @details lens fits intrinsics/distortion from multiple views; mount fits
   * rotation using fixed intrinsics and an aircraft-aligned board.
   */
  enum class Mode { lens, mount };
  Mode mode = Mode::lens; ///< Selected calibration stage.
  int columns = 0; ///< Inner corners along the board's first coordinate axis.
  int rows = 0; ///< Inner corners along the board's second coordinate axis.
  double square_mm = 0.0; ///< Measured adjacent-corner pitch in millimetres.
  fs::path output; ///< New candidate profile; existing evidence is not replaced.
  fs::path images; ///< Lens-fitting JPEG directory.
  fs::path check; ///< Optional separate lens hold-out JPEG directory.
  fs::path image; ///< Mount photograph in the lens profile's native dimensions.
  fs::path lens; ///< Intrinsic profile held fixed during mount fitting.
  bool reverse_corners = false; ///< Explicitly reverse mount detection order.
};

/** @brief Related artifacts share one stem so evidence stays with its profile. */
struct OutputPaths {
  fs::path profile; ///< Candidate calibration consumed by LWIRcam.
  fs::path previews; ///< Directory of annotated inspection-only JPEGs.
  fs::path report; ///< Human-readable fit/check diagnostics and failure details.
};

/** @brief Intrinsics read without inheriting mounting or verification claims. */
struct LensProfile {
  cv::Size image_size; ///< Native pixel dimensions for which coefficients apply.
  cv::Mat camera_matrix; ///< Double-precision 3x3 pinhole matrix, focal units pixels.
  cv::Mat distortion; ///< Double-precision k1, k2, p1, p2, k3 in OpenCV order.
};

/** @brief Ordered diagnostic lines retained for success and failure reports. */
using Report = std::vector<std::string>;

/** @brief Stop a run before invalid evidence can propagate to its profile.
 * @param condition Invariant required for the next operation.
 * @param message Diagnostic explaining the failed invariant.
 * @throws std::runtime_error If condition is false.
 */
void require(bool condition, const std::string& message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

/** @brief Parse an entire finite numeric argument, never a valid prefix alone.
 * @param text Argument token to parse.
 * @param option Option name used in the failure diagnostic.
 * @return Finite value; its physical range is checked by parse_options().
 * @throws UsageError If trailing text or a nonfinite value is present.
 * @throws std::invalid_argument If no number can be parsed.
 * @throws std::out_of_range If conversion exceeds the representable range.
 */
double parse_double(const std::string& text, const std::string& option)
{
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw UsageError(option + " requires one finite number");
  }
  return value;
}

/** @brief Parse a bounded nonnegative integer before narrowing to int.
 * @param text Complete decimal argument token.
 * @param option Option name used in the failure diagnostic.
 * @return Parsed value in 0..1000000; board limits are checked separately.
 * @throws UsageError If trailing text or an out-of-policy value is present.
 * @throws std::invalid_argument If no integer can be parsed.
 * @throws std::out_of_range If conversion overflows long.
 */
int parse_integer(const std::string& text, const std::string& option)
{
  size_t consumed = 0;
  const long value = std::stol(text, &consumed, 10);
  if (consumed != text.size() || value < 0 || value > 1000000) {
    throw UsageError(option + " requires a nonnegative integer");
  }
  return static_cast<int>(value);
}

/** @brief Describe the two workshop stages without opening images or devices.
 * @param program Invocation name displayed in command examples.
 */
void print_usage(const char* program)
{
  std::cout << LWIR_CALIBRATE_BUILD_VERSION << "\n"
            << "Usage:\n"
            << "  " << program << " lens --images DIR [--check DIR] --columns N --rows N\n"
            << "      --square-mm MM --output PROFILE.yml\n"
            << "  " << program << " mount --image PHOTO.jpg --lens PROFILE.yml\n"
            << "      --columns N --rows N --square-mm MM --output PROFILE.yml\n"
            << "      [--reverse-corners]\n"
            << "Only original JPG/JPEG calibration photographs are accepted.\n";
}

/** @brief Validate stage-specific arguments before creating any output paths.
 * @param argc Number of command-line tokens, including the executable name.
 * @param argv Command-line tokens supplied to main().
 * @return Complete invocation with compatible options and bounded board geometry.
 * @throws UsageError If options are missing, repeated, unknown or incompatible.
 * @throws std::exception On numeric conversion failures.
 * @details Rejecting duplicates prevents a silently overridden pitch or board
 * size from changing the physical interpretation of an otherwise plausible fit.
 */
Options parse_options(int argc, char** argv)
{
  if (argc < 2) {
    throw UsageError("select lens or mount mode");
  }
  Options options;
  const std::string mode(argv[1]);
  if (mode == "lens") {
    options.mode = Options::Mode::lens;
  } else if (mode == "mount") {
    options.mode = Options::Mode::mount;
  } else {
    throw UsageError("mode must be lens or mount");
  }

  std::set<std::string> seen;
  for (int index = 2; index < argc; ++index) {
    const std::string key(argv[index]);
    if (!seen.insert(key).second) {
      throw UsageError("option repeated: " + key);
    }
    if (key == "--reverse-corners") {
      options.reverse_corners = true;
      continue;
    }
    if (index + 1 >= argc) {
      throw UsageError("missing value for " + key);
    }
    const std::string value(argv[++index]);
    if (key == "--columns") {
      options.columns = parse_integer(value, key);
    } else if (key == "--rows") {
      options.rows = parse_integer(value, key);
    } else if (key == "--square-mm") {
      options.square_mm = parse_double(value, key);
    } else if (key == "--output") {
      options.output = value;
    } else if (key == "--images") {
      options.images = value;
    } else if (key == "--check") {
      options.check = value;
    } else if (key == "--image") {
      options.image = value;
    } else if (key == "--lens") {
      options.lens = value;
    } else {
      throw UsageError("unknown option: " + key);
    }
  }

  if (options.columns < 3 || options.columns > 64 || options.rows < 3 || options.rows > 64
      || options.square_mm <= 0.0 || options.square_mm > 10000.0 || options.output.empty()) {
    throw UsageError("use 3..64 inner corners per dimension, a positive square pitch, and --output");
  }
  if (options.mode == Options::Mode::lens && (options.images.empty() || !options.image.empty()
                                               || !options.lens.empty() || options.reverse_corners)) {
    throw UsageError("lens mode requires --images and does not accept mount-only options");
  }
  if (options.mode == Options::Mode::mount && (options.image.empty() || options.lens.empty()
                                                || !options.images.empty() || !options.check.empty())) {
    throw UsageError("mount mode requires --image and --lens, not --images or --check");
  }
  return options;
}

/** @brief Open a nonempty, size-bounded regular file for JPEG decoding.
 * @param path Input photograph; the descriptor is checked after opening.
 * @return Owned binary stream positioned at its beginning.
 * @throws std::runtime_error If opening or descriptor validation fails.
 * @note This is a trusted-directory workflow, not a no-follow or nonblocking
 * open: descriptor validation does not provide hostile-path isolation.
 */
File open_regular_jpeg(const fs::path& path)
{
  const std::string native_path = path.string();
  File input(std::fopen(native_path.c_str(), "rb"));
  require(input != nullptr, "cannot open JPEG: " + native_path);
  struct stat status {};
  require(::fstat(::fileno(input.get()), &status) == 0 && S_ISREG(status.st_mode)
              && status.st_size > 0 && static_cast<uint64_t>(status.st_size) <= maximum_jpeg_bytes,
          "JPEG must be a non-empty regular file below 128 MiB: " + native_path);
  return input;
}

/** @brief Decode native grayscale pixels using the already-bundled JPEG codec.
 * @param path Original JPG/JPEG photograph.
 * @return Owned/ref-counted CV_8UC1 image in the JPEG's stored orientation.
 * @throws std::runtime_error If file bounds, dimensions or codec checks fail.
 * @details Grayscale is sufficient for checkerboard geometry. Dimension checks
 * precede image allocation, and codec warnings reject damaged-but-recoverable
 * inputs rather than allowing reconstructed pixels to bias a calibration.
 */
cv::Mat read_jpeg_gray(const fs::path& path)
{
  File input = open_regular_jpeg(path);
  auto decoder = std::make_unique<Decoder>();
  decoder->state.err = jpeg_std_error(&decoder->error.manager);
  decoder->error.emit = decoder->error.manager.emit_message;
  decoder->error.manager.emit_message = jpeg_message;
  decoder->error.manager.error_exit = jpeg_failure;
  if (setjmp(decoder->error.recovery)) {
    throw std::runtime_error("invalid or damaged JPEG: " + path.string());
  }
  decoder->created = true;
  jpeg_create_decompress(&decoder->state);
  jpeg_stdio_src(&decoder->state, input.get());
  require(jpeg_read_header(&decoder->state, TRUE) == JPEG_HEADER_OK,
          "invalid JPEG header: " + path.string());
  const uint64_t width = decoder->state.image_width;
  const uint64_t height = decoder->state.image_height;
  require(width >= 2 && height >= 2 && width <= maximum_image_dimension
              && height <= maximum_image_dimension && width * height <= maximum_image_pixels,
          "unsupported JPEG dimensions: " + path.string());
  decoder->state.out_color_space = JCS_GRAYSCALE;
  jpeg_start_decompress(&decoder->state);
  decoder->gray.create(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
  while (decoder->state.output_scanline < decoder->state.output_height) {
    JSAMPROW row = decoder->gray.ptr(decoder->state.output_scanline);
    require(jpeg_read_scanlines(&decoder->state, &row, 1) == 1,
            "incomplete JPEG scanline: " + path.string());
  }
  jpeg_finish_decompress(&decoder->state);
  require(decoder->error.manager.num_warnings == 0, "damaged JPEG: " + path.string());
  return decoder->gray;
}

/** @brief Encode an inspection preview without linking OpenCV imgcodecs.
 * @param path Destination inside this run's prepared preview directory.
 * @param bgr Nonempty CV_8UC3 image within the configured size ceilings.
 * @throws std::runtime_error On invalid dimensions or write/codec failure.
 * @details Convert BGR to RGB one bounded row at a time, avoiding another
 * full-image color copy. Previews are lossy display artifacts, never fit inputs.
 * @note A fatal codec error unlinks its partial preview; this is not an atomic
 * publication or crash-durability guarantee.
 */
void write_jpeg_bgr(const fs::path& path, const cv::Mat& bgr)
{
  require(!bgr.empty() && bgr.type() == CV_8UC3
              && bgr.cols <= static_cast<int>(maximum_image_dimension)
              && bgr.rows <= static_cast<int>(maximum_image_dimension)
              && static_cast<uint64_t>(bgr.cols) * static_cast<uint64_t>(bgr.rows) <= maximum_image_pixels,
          "preview must be an 8-bit BGR image within the configured dimension limit");
  const std::string native_path = path.string();
  File output(std::fopen(native_path.c_str(), "wb"));
  require(output != nullptr, "cannot create JPEG preview: " + native_path);
  auto encoder = std::make_unique<Encoder>();
  encoder->state.err = jpeg_std_error(&encoder->error.manager);
  encoder->error.manager.error_exit = jpeg_failure;
  std::array<JSAMPLE, maximum_image_dimension * 3U> rgb_row{};
  if (setjmp(encoder->error.recovery)) {
    ::unlink(native_path.c_str());
    throw std::runtime_error("JPEG preview encoding failed: " + native_path);
  }
  encoder->created = true;
  jpeg_create_compress(&encoder->state);
  jpeg_stdio_dest(&encoder->state, output.get());
  encoder->state.image_width = static_cast<JDIMENSION>(bgr.cols);
  encoder->state.image_height = static_cast<JDIMENSION>(bgr.rows);
  encoder->state.input_components = 3;
  encoder->state.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder->state);
  jpeg_set_quality(&encoder->state, preview_jpeg_quality, TRUE);
  jpeg_start_compress(&encoder->state, TRUE);
  while (encoder->state.next_scanline < encoder->state.image_height) {
    const unsigned int y = encoder->state.next_scanline;
    const cv::Vec3b* source = bgr.ptr<cv::Vec3b>(static_cast<int>(y));
    for (int x = 0; x < bgr.cols; ++x) {
      rgb_row[static_cast<size_t>(x) * 3U] = source[x][2];
      rgb_row[static_cast<size_t>(x) * 3U + 1U] = source[x][1];
      rgb_row[static_cast<size_t>(x) * 3U + 2U] = source[x][0];
    }
    JSAMPROW row = rgb_row.data();
    require(jpeg_write_scanlines(&encoder->state, &row, 1) == 1,
            "cannot write JPEG preview scanline");
  }
  jpeg_finish_compress(&encoder->state);
  require(std::fflush(output.get()) == 0, "cannot flush JPEG preview: " + native_path);
}

/** @brief Collect supported photographs in stable lexical order.
 * @param directory Nonrecursive input directory.
 * @return Nonempty sorted list of regular .jpg/.jpeg files, case-insensitive.
 * @throws std::runtime_error If enumeration fails, finds no JPEGs or exceeds
 * the accepted-image cap.
 * @details Stable ordering keeps report/preview indexing reproducible for the
 * same filenames. Extensions select candidates; libjpeg validates their content.
 */
std::vector<fs::path> jpg_paths(const fs::path& directory)
{
  std::error_code error;
  require(fs::is_directory(directory, error) && !error,
          "not an image directory: " + directory.string());
  std::vector<fs::path> paths;
  for (fs::directory_iterator it(directory, error), end; !error && it != end; it.increment(error)) {
    if (paths.size() >= maximum_directory_images) {
      throw std::runtime_error("image directory exceeds the 10000-file limit: " + directory.string());
    }
    if (!it->is_regular_file(error) || error) {
      error.clear();
      continue;
    }
    std::string extension = it->path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (extension == ".jpg" || extension == ".jpeg") {
      paths.push_back(it->path());
    }
  }
  require(!error, "cannot enumerate image directory: " + directory.string());
  std::sort(paths.begin(), paths.end());
  require(!paths.empty(), "no JPG/JPEG images in " + directory.string());
  return paths;
}

/** @brief Build the planar metric coordinates matching detector corner order.
 * @param columns Inner corners per row, already validated by parse_options().
 * @param rows Inner-corner rows, already validated by parse_options().
 * @param square_mm Measured adjacent-corner pitch in millimetres.
 * @return Row-major points in metres with origin at corner zero and z=0.
 * @details Consistent metric units make pose translations meaningful; resizing
 * an image or confusing square count with corner count changes the model.
 */
std::vector<cv::Point3f> board_points(int columns, int rows, double square_mm)
{
  const float pitch_m = static_cast<float>(square_mm * metres_per_millimetre);
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<size_t>(columns) * static_cast<size_t>(rows));
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      points.emplace_back(column * pitch_m, row * pitch_m, 0.0F);
    }
  }
  return points;
}

/** @brief Detect a complete board with OpenCV's subpixel sector-based method.
 * @param gray Original native grayscale image, not an enlarged preview.
 * @param options Validated inner-corner dimensions.
 * @param[out] corners Detected subpixel positions; consume only on success.
 * @return True when a complete board of the requested dimensions is found.
 * @details Normalization and exhaustive search help low-contrast thermal
 * patterns. Detection order still cannot establish physical aircraft axes.
 */
bool detect_board(const cv::Mat& gray, const Options& options, std::vector<cv::Point2f>& corners)
{
  const cv::Size pattern(options.columns, options.rows);
  return cv::findChessboardCornersSB(gray, pattern, corners,
      cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
}

/** @brief Measure per-corner Euclidean reprojection RMS in original pixels.
 * @param object_points Board coordinates in the frame used for pose fitting.
 * @param image_points Corresponding observed native pixel positions.
 * @param rotation Rodrigues object-to-camera rotation vector.
 * @param translation Object-to-camera translation in board units (metres here).
 * @param camera_matrix Pinhole intrinsics with focal lengths in pixels.
 * @param distortion OpenCV distortion coefficients used by the fitted model.
 * @return Square root of summed squared 2D errors divided by corner count.
 * @pre Correspondence arrays have equal nonzero length.
 * @note This residual measures image-model agreement, not ground-position or
 * temperature accuracy, board alignment or correctness of physical axis labels.
 */
double reprojection_rms(const std::vector<cv::Point3f>& object_points,
                        const std::vector<cv::Point2f>& image_points,
                        const cv::Mat& rotation, const cv::Mat& translation,
                        const cv::Mat& camera_matrix, const cv::Mat& distortion)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rotation, translation, camera_matrix, distortion, projected);
  double squared_error = 0.0;
  for (size_t index = 0; index < projected.size(); ++index) {
    const cv::Point2f delta = projected[index] - image_points[index];
    squared_error += delta.dot(delta);
  }
  return std::sqrt(squared_error / static_cast<double>(projected.size()));
}

/** @brief Save inspectable corner evidence separately from fitting pixels.
 * @param gray Original native grayscale image.
 * @param corners Complete detected pattern in native pixel coordinates.
 * @param options Validated board dimensions.
 * @param path New preview JPEG path.
 * @param numbered Label corners 0, 1 and columns to expose both board directions.
 * @details Nearest-neighbor enlargement keeps thermal pixel structure visible.
 * Both side and pixel limits constrain the preferred scale. Numbered neighbors
 * let an operator check detector order against the physically aligned board.
 * @throws std::runtime_error If corner indexing or preview encoding fails.
 */
void write_preview(const cv::Mat& gray, const std::vector<cv::Point2f>& corners,
                   const Options& options, const fs::path& path, bool numbered)
{
  cv::Mat canvas;
  cv::cvtColor(gray, canvas, cv::COLOR_GRAY2BGR);
  cv::drawChessboardCorners(canvas, cv::Size(options.columns, options.rows), corners, true);
  const double dimension_scale = std::min(
      static_cast<double>(maximum_image_dimension) / canvas.cols,
      static_cast<double>(maximum_image_dimension) / canvas.rows);
  const double pixel_scale = std::sqrt(static_cast<double>(maximum_image_pixels) / canvas.total());
  const double scale = std::min(preferred_preview_scale, std::min(dimension_scale, pixel_scale));
  const cv::Size preview_size(std::max(1, cvRound(canvas.cols * scale)),
                              std::max(1, cvRound(canvas.rows * scale)));
  cv::resize(canvas, canvas, preview_size, 0.0, 0.0, cv::INTER_NEAREST);
  if (numbered) {
    for (const int index : {0, 1, options.columns}) {
      require(static_cast<size_t>(index) < corners.size(), "internal corner index outside detected pattern");
      const cv::Point point(cvRound(corners[index].x * preview_size.width / gray.cols),
                cvRound(corners[index].y * preview_size.height / gray.rows));
      const std::string label = std::to_string(index);
      cv::putText(canvas, label, point, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
      cv::putText(canvas, label, point, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
  }
  write_jpeg_bgr(path, canvas);
}

/** @brief Keep live console diagnostics and the saved evidence in the same order.
 * @param[in,out] report Accumulated diagnostic lines.
 * @param line Message to retain and print.
 */
void report_line(Report& report, const std::string& line)
{
  report.push_back(line);
  std::cout << line << '\n';
}

/** @brief Write retained evidence as a plain-text workshop report.
 * @param path Report path belonging to this run.
 * @param report Ordered diagnostics, including any terminal failure message.
 * @throws std::runtime_error If opening or writing the stream fails.
 * @note Used best-effort during failure cleanup; not an atomic journal.
 */
void write_report(const fs::path& path, const Report& report)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  require(output.good(), "cannot write calibration report: " + path.string());
  for (const std::string& line : report) {
    output << line << '\n';
  }
  require(output.good(), "incomplete calibration report: " + path.string());
}

/** @brief Refuse existing artifacts before creating the run's directories.
 * @param options Invocation containing the requested new profile path.
 * @return Profile, report and preview paths after directory creation succeeds.
 * @throws std::runtime_error If any artifact already exists or cannot be checked.
 * @throws fs::filesystem_error If directory creation fails.
 * @details Checking all three artifacts preserves evidence from earlier runs,
 * including failed ones. It is a workshop overwrite guard, not an exclusive
 * filesystem reservation against another process racing the same pathname.
 */
OutputPaths prepare_outputs(const Options& options)
{
  OutputPaths paths{options.output, options.output.parent_path() /
      (options.output.stem().string() + "-previews"), options.output};
  paths.report.replace_extension(".report.txt");
  std::error_code error;
  require(!fs::exists(paths.profile, error) && !error && !fs::exists(paths.previews, error)
              && !error && !fs::exists(paths.report, error) && !error,
          "output, report or preview path already exists; choose a new output name");
  fs::path parent = paths.profile.parent_path();
  if (parent.empty()) {
    parent = ".";
  }
  fs::create_directories(parent);
  fs::create_directory(paths.previews);
  return paths;
}

/** @brief Record enough setup context to interpret the residuals and previews.
 * @param options Board dimensions and measured pitch used for this run.
 * @param[in,out] report Destination for software, pattern and verification notes.
 */
void append_report_header(const Options& options, Report& report)
{
  std::ostringstream header;
  header << "LWIR workshop calibration; OpenCV " << CV_VERSION;
  report.push_back(header.str());
  std::ostringstream pattern;
  pattern << "Pattern " << options.columns << 'x' << options.rows
          << " inner corners, pitch " << options.square_mm << " mm";
  report.push_back(pattern.str());
  report.emplace_back("verified=0; inspect previews and perform independent physical checks.");
}

/** @brief Reject nonfinite coefficients before persisting a candidate profile.
 * @param camera_matrix Fitted or imported intrinsics.
 * @param distortion Fitted or imported distortion coefficients.
 * @param mounting Row-major camera-to-body rotation coefficients.
 * @throws std::runtime_error If an array fails its numeric check.
 * @note This is a numeric sanity check, not a physical calibration assessment.
 */
void validate_outputs(const cv::Mat& camera_matrix, const cv::Mat& distortion,
                      const std::array<double, 9>& mounting)
{
  require(cv::checkRange(camera_matrix) && cv::checkRange(distortion),
          "fit produced non-finite camera coefficients");
  require(std::all_of(mounting.begin(), mounting.end(),
                      [](double value) { return std::isfinite(value); }),
          "fit produced non-finite mounting coefficients");
}

/** @brief Export coefficients in the format consumed by native LWIRcam.
 * @param path Candidate profile path approved by prepare_outputs().
 * @param size Native image dimensions to which the calibration applies.
 * @param matrix Double-precision 3x3 intrinsic matrix.
 * @param distortion At least five doubles in k1, k2, p1, p2, k3 order.
 * @param mounting Nine row-major camera-to-body rotation coefficients.
 * @throws std::runtime_error If numeric validation or output completion fails.
 * @details Seventeen significant digits preserve double precision on round-trip.
 * The rotation stays on one line because the bounded runtime loader parses one
 * record per line; a general serializer may wrap long fitted coefficients.
 * Verification is always false, including when refining a previously used lens.
 * @note Output completion is checked, but publication is not atomic or fsynced.
 */
void write_profile(const fs::path& path, cv::Size size, const cv::Mat& matrix,
                   const cv::Mat& distortion, const std::array<double, 9>& mounting)
{
  validate_outputs(matrix, distortion, mounting);
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  require(output.is_open(), "cannot write calibration profile: " + path.string());
  output << "%YAML:1.0\n---\n" << std::setprecision(17);
  output << "image_width: " << size.width << '\n';
  output << "image_height: " << size.height << '\n';
  output << "fx: " << matrix.at<double>(0, 0) << '\n';
  output << "fy: " << matrix.at<double>(1, 1) << '\n';
  output << "cx: " << matrix.at<double>(0, 2) << '\n';
  output << "cy: " << matrix.at<double>(1, 2) << '\n';
  const cv::Mat flat_distortion = distortion.reshape(1, 1);
  static const std::array<const char*, 5> keys{{"k1", "k2", "p1", "p2", "k3"}};
  for (int index = 0; index < 5; ++index) {
    output << keys[static_cast<size_t>(index)] << ": "
           << flat_distortion.at<double>(0, index) << '\n';
  }
  output << profile_verified_key << ": 0\ncamera_to_body: [";
  for (size_t index = 0; index < mounting.size(); ++index) {
    output << (index == 0 ? " " : ", ") << mounting[index];
  }
  output << " ]\n";
  output.close();
  require(output.good(), "cannot finish calibration profile: " + path.string());
  require(fs::file_size(path) > 0, "calibration profile is empty: " + path.string());
}

/** @brief Import only the fixed intrinsics needed by the mounting solver.
 * @param path Existing profile with numeric dimensions and lens coefficients.
 * @return Double-precision intrinsics/distortion and their native pixel size.
 * @throws std::runtime_error If required fields, dimensions or focal lengths fail
 * validation.
 * @throws cv::Exception If OpenCV cannot parse the profile.
 * @details Mount fitting must not refit the lens from a single board view or
 * inherit a prior mounting/verification claim. Numeric nodes are required so a
 * quoted number cannot silently become a missing or default coefficient.
 */
LensProfile read_lens_profile(const fs::path& path)
{
  cv::FileStorage input(path.string(), cv::FileStorage::READ);
  require(input.isOpened(), "cannot open lens profile: " + path.string());
  const auto read_number = [&input, &path](const char* key) {
    const cv::FileNode node = input[key];
    require(!node.empty() && (node.isInt() || node.isReal()),
            std::string("missing or nonnumeric lens key: ") + key + " in " + path.string());
    double value = 0.0;
    node >> value;
    require(std::isfinite(value), std::string("non-finite lens key: ") + key);
    return value;
  };
  const double width_value = read_number("image_width");
  const double height_value = read_number("image_height");
  require(width_value >= 1 && width_value <= maximum_image_dimension
              && height_value >= 1 && height_value <= maximum_image_dimension
              && std::floor(width_value) == width_value && std::floor(height_value) == height_value,
          "invalid lens-profile dimensions");
  LensProfile profile;
  profile.image_size = cv::Size(static_cast<int>(width_value), static_cast<int>(height_value));
  profile.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  profile.camera_matrix.at<double>(0, 0) = read_number("fx");
  profile.camera_matrix.at<double>(1, 1) = read_number("fy");
  profile.camera_matrix.at<double>(0, 2) = read_number("cx");
  profile.camera_matrix.at<double>(1, 2) = read_number("cy");
  require(profile.camera_matrix.at<double>(0, 0) > 0 && profile.camera_matrix.at<double>(1, 1) > 0,
          "lens focal lengths must be positive");
  profile.distortion = cv::Mat(1, 5, CV_64F);
  static const std::array<const char*, 5> keys{{"k1", "k2", "p1", "p2", "k3"}};
  for (int index = 0; index < 5; ++index) {
    profile.distortion.at<double>(0, index) = read_number(keys[static_cast<size_t>(index)]);
  }
  return profile;
}

/** @brief Fit lens geometry and report residuals on separate check photographs.
 * @param options Validated lens-stage invocation and measured board pitch.
 * @param outputs Fresh artifact paths prepared for this run.
 * @param[in,out] report Fitting/check diagnostics retained for operator review.
 * @throws std::runtime_error If inputs disagree, too few complete boards remain,
 * a check pose fails or output cannot be written.
 * @throws cv::Exception If OpenCV detection or optimization fails.
 * @details Only corner correspondences are retained between images, not all
 * decoded photographs. OpenCV estimates intrinsics, five distortion coefficients
 * and a separate pose for each accepted view. Hold-out images estimate poses
 * using those fixed lens coefficients, so they are not additional fitting data.
 * @warning View count and low residuals do not prove useful image coverage or
 * correct board dimensions. Exported mounting is nominal, not measured here.
 */
void fit_lens(const Options& options, const OutputPaths& outputs, Report& report)
{
  const std::vector<cv::Point3f> board = board_points(options.columns, options.rows, options.square_mm);
  const std::vector<fs::path> paths = jpg_paths(options.images);
  std::vector<std::vector<cv::Point3f>> object_views;
  std::vector<std::vector<cv::Point2f>> image_views;
  std::vector<fs::path> accepted_paths;
  cv::Size image_size;
  for (size_t index = 0; index < paths.size(); ++index) {
    const cv::Mat gray = read_jpeg_gray(paths[index]);
    const cv::Size current_size(gray.cols, gray.rows);
    if (image_size.empty()) {
      image_size = current_size;
    } else {
      require(image_size == current_size,
              "mixed image dimensions at " + paths[index].string() + "; do not resize to force a match");
    }
    std::vector<cv::Point2f> corners;
    if (!detect_board(gray, options, corners)) {
      report_line(report, "SKIPPED no complete pattern: " + paths[index].string());
      continue;
    }
    object_views.push_back(board);
    image_views.push_back(corners);
    accepted_paths.push_back(paths[index]);
    std::ostringstream preview_name;
    preview_name << "fit-" << std::setw(3) << std::setfill('0') << index << ".jpg";
    write_preview(gray, corners, options, outputs.previews / preview_name.str(), false);
  }
  require(object_views.size() >= minimum_fit_views,
          "only " + std::to_string(object_views.size()) + " complete patterns; need at least 15 varied views");

  cv::Mat camera_matrix;
  cv::Mat distortion;
  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  const double rms = cv::calibrateCamera(object_views, image_views, image_size,
      camera_matrix, distortion, rotations, translations);
  require(std::isfinite(rms), "lens fit produced non-finite RMS");
  {
    std::ostringstream line;
    line << std::fixed << std::setprecision(6) << "Fitting RMS: " << rms
         << " pixels; retained views: " << accepted_paths.size();
    report_line(report, line.str());
  }
  for (size_t index = 0; index < accepted_paths.size(); ++index) {
    std::ostringstream line;
    line << std::fixed << std::setprecision(6) << "FIT "
         << reprojection_rms(board, image_views[index], rotations[index], translations[index],
                             camera_matrix, distortion)
         << " px: " << accepted_paths[index].string();
    report_line(report, line.str());
  }

  if (!options.check.empty()) {
    // Resolve path aliases so a symlink to a fitting image is not a hold-out.
    // This is path separation, not content deduplication of copied photographs.
    std::set<fs::path> fit_files;
    for (const fs::path& path : paths) {
      fit_files.insert(fs::weakly_canonical(path));
    }
    size_t check_count = 0;
    const std::vector<fs::path> check_paths = jpg_paths(options.check);
    for (size_t index = 0; index < check_paths.size(); ++index) {
      require(fit_files.count(fs::weakly_canonical(check_paths[index])) == 0,
              "check images must be separate from fitting images: " + check_paths[index].string());
      const cv::Mat gray = read_jpeg_gray(check_paths[index]);
      require(cv::Size(gray.cols, gray.rows) == image_size,
              "check-image dimensions differ: " + check_paths[index].string());
      std::vector<cv::Point2f> corners;
      if (!detect_board(gray, options, corners)) {
        report_line(report, "SKIPPED check pattern: " + check_paths[index].string());
        continue;
      }
      cv::Mat rotation;
      cv::Mat translation;
      require(cv::solvePnP(board, corners, camera_matrix, distortion, rotation, translation),
              "cannot estimate check-board pose: " + check_paths[index].string());
      std::ostringstream line;
      line << std::fixed << std::setprecision(6) << "CHECK "
           << reprojection_rms(board, corners, rotation, translation, camera_matrix, distortion)
           << " px: " << check_paths[index].string();
      report_line(report, line.str());
      ++check_count;
      std::ostringstream preview_name;
      preview_name << "check-" << std::setw(3) << std::setfill('0') << index << ".jpg";
      write_preview(gray, corners, options, outputs.previews / preview_name.str(), false);
    }
    require(check_count >= minimum_check_views,
            "only " + std::to_string(check_count) + " complete check patterns; supply at least 5");
  } else {
    report_line(report, "WARNING: no independent check set supplied");
  }
  report_line(report, "Mounting is NOMINAL ONLY; physical axis measurement is still required.");
  const std::array<double, 9> nominal_mounting{{0, -1, 0, 1, 0, 0, 0, 0, 1}};
  write_profile(outputs.profile, image_size, camera_matrix, distortion, nominal_mounting);
}

/** @brief Estimate camera-to-body rotation using an aircraft-aligned board.
 * @param options Mount-stage invocation with fixed lens profile and photograph.
 * @param outputs Fresh artifact paths prepared for this run.
 * @param[in,out] report Residual, axis-direction and inspection diagnostics.
 * @throws std::runtime_error If dimensions, board detection, pose cheirality or
 * profile publication checks fail.
 * @throws cv::Exception If OpenCV parsing, detection or pose solving fails.
 * @details Camera axes are right/down/forward; aircraft body axes are
 * forward/right/down. The board's increasing columns must point body-right and
 * increasing rows tailward, in a plane parallel to the body datum. The solver's
 * body-to-camera rotation is inverted for downstream camera-ray projection.
 * Translation is a board-pose nuisance variable, not the camera/GPS lever arm.
 * @warning A symmetric checkerboard can give a plausible residual with a wrong
 * physical orientation. The numbered preview and datum alignment require human
 * confirmation; positive board depth alone cannot verify the mount.
 */
void fit_mount(const Options& options, const OutputPaths& outputs, Report& report)
{
  const LensProfile lens = read_lens_profile(options.lens);
  const cv::Mat gray = read_jpeg_gray(options.image);
  require(cv::Size(gray.cols, gray.rows) == lens.image_size,
          "mount image does not match lens-profile dimensions");
  std::vector<cv::Point2f> corners;
  require(detect_board(gray, options, corners),
          "no complete checker pattern; improve contrast, focus or framing");
    // Reversal is an explicit operator choice, never inferred from a low RMS.
  if (options.reverse_corners) {
    std::reverse(corners.begin(), corners.end());
  }
  write_preview(gray, corners, options, outputs.previews / "mount-order.jpg", true);

  const std::vector<cv::Point3f> board = board_points(options.columns, options.rows, options.square_mm);
  std::vector<cv::Point3f> body_points;
  body_points.reserve(board.size());
  // Board x grows rightward and board y tailward: body (forward,right) = (-y,x).
  // The arbitrary corner origin affects fitted translation, not the rotation.
  for (const cv::Point3f& point : board) {
    body_points.emplace_back(-point.y, point.x, point.z);
  }
  cv::Mat rotation;
  cv::Mat translation;
  require(cv::solvePnP(body_points, corners, lens.camera_matrix, lens.distortion, rotation, translation),
          "mounting pose did not converge");
  cv::Mat body_to_camera_mat;
  cv::Rodrigues(rotation, body_to_camera_mat);
  cv::Matx33d body_to_camera;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      body_to_camera(row, column) = body_to_camera_mat.at<double>(row, column);
    }
  }
  // Reject the physically impossible branch with any board point behind the lens.
  for (const cv::Point3f& point : body_points) {
    const cv::Vec3d camera_point = body_to_camera * cv::Vec3d(point.x, point.y, point.z)
                                   + cv::Vec3d(translation.at<double>(0), translation.at<double>(1),
                                               translation.at<double>(2));
    require(camera_point[2] > 0.0, "board pose lies behind the camera; check corner order");
  }
  // solvePnP maps object/body coordinates into camera coordinates; rays need
  // the opposite direction. For a Rodrigues rotation, transpose is the inverse.
  const cv::Matx33d camera_to_body = body_to_camera.t();
  std::array<double, 9> mounting{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      mounting[static_cast<size_t>(3 * row + column)] = camera_to_body(row, column);
    }
  }
  {
    std::ostringstream line;
    line << std::fixed << std::setprecision(6) << "Mount reprojection RMS: "
         << reprojection_rms(body_points, corners, rotation, translation,
                             lens.camera_matrix, lens.distortion) << " pixels";
    report_line(report, line.str());
  }
  report_line(report, "ASSUMPTION: board columns point body-right; rows point toward tail; board is level with body datum.");
  report_line(report, "CONFIRM preview labels 0, 1 and " + std::to_string(options.columns)
                      + " against the physical board; no automatic verification.");
  {
    const cv::Vec3d optical_axis(camera_to_body(0, 2), camera_to_body(1, 2), camera_to_body(2, 2));
    std::ostringstream line;
    line << "Optical axis in body forward/right/down: [" << optical_axis[0] << ", "
         << optical_axis[1] << ", " << optical_axis[2] << "]";
    report_line(report, line.str());
  }
  write_profile(outputs.profile, lens.image_size, lens.camera_matrix, lens.distortion, mounting);
}

}  // namespace

/** @brief Run one workshop stage with distinct invocation and fitting failures.
 * @param argc Number of command-line arguments, including the executable name.
 * @param argv Command-line arguments; help/version require no input files.
 * @retval 0 Requested information or calibration artifacts were produced.
 * @retval 1 Calibration or I/O failed; available diagnostic evidence is retained.
 * @retval 2 Invocation or numeric argument parsing failed before output setup.
 * @details Failed processing removes the candidate profile so a partial result
 * cannot masquerade as a usable calibration. Reports/previews stay available for
 * diagnosis, and report-writing failure never hides the original error.
 */
int main(int argc, char** argv)
{
  if (argc == 2 && (std::string(argv[1]) == "--version" || std::string(argv[1]) == "-V")) {
    std::cout << LWIR_CALIBRATE_BUILD_VERSION << '\n';
    return 0;
  }
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_usage(argv[0]);
    return 0;
  }

  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const UsageError& error) {
    std::cerr << error.what() << '\n';
    print_usage(argv[0]);
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }

  OutputPaths outputs;
  Report report;
  try {
    const usher_cv_parallel::ScopedBackend opencv_parallel_backend;
    outputs = prepare_outputs(options);
    append_report_header(options, report);
    if (options.mode == Options::Mode::lens) {
      fit_lens(options, outputs, report);
    } else {
      fit_mount(options, outputs, report);
    }
    report_line(report, "Not automatically verified. Inspect evidence before using coordinates.");
    write_report(outputs.report, report);
    std::cout << "Profile: " << outputs.profile << "\nReport: " << outputs.report
              << "\nPreviews: " << outputs.previews << '\n';
    return 0;
  } catch (const cv::Exception& error) {
    report.push_back(std::string("STOP: ") + error.what());
  } catch (const std::exception& error) {
    report.push_back(std::string("STOP: ") + error.what());
  }

  std::error_code remove_error;
  // outputs is assigned only after prepare_outputs succeeds. An existing profile
  // rejected by the overwrite guard must never become a cleanup target.
  if (!outputs.profile.empty()) {
    fs::remove(outputs.profile, remove_error);
  }
  if (!outputs.report.empty()) {
    try {
      write_report(outputs.report, report);
    } catch (...) {
    }
  }
  if (!report.empty()) {
    std::cerr << report.back() << '\n';
  }
  if (!outputs.report.empty()) {
    std::cerr << "Details: " << outputs.report << "\nChoose a new output filename on retry.\n";
  }
  return 1;
}