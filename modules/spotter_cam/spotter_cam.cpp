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
 * @file spotter_cam.cpp
 * @brief Object-model decoder, detection sidecar writer and detector entry point.
 * @details Camera/DMA/JPEG ownership belongs to imx500_camera.cpp. This application
 * interprets retained frame metadata only as object boxes; it contains no LOOM
 * feature processing, reference matching or waypoint policy. A structurally
 * valid empty detection result is useful evidence and differs from missing or
 * malformed inference; keep that distinction when extending the model contract.
 */

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

#include "version.h"
#include "inference_worker.h"
#include "spotter_targets.h"
#include "camera_profile.h"

#ifndef SENSORAIA_INFERENCE_ONLY
#include "imx500_camera.hpp"
#include "imx500_tensor.h"
#include "inference_capture.hpp"

/** Internal implementation details documented here to make the camera and model
 * contracts navigable. */
namespace spottercam {

/** Physical IMX500 pixel-array dimensions expected by the model's coordinate
 * transform. */
constexpr unsigned int sensor_width = imx500::sensor_width;
/** @brief Sensor-space height, separate from model and encoded-image dimensions. */
constexpr unsigned int sensor_height = imx500::sensor_height;
/** Input dimensions of the object RPK's absolute-coordinate detection head. */
constexpr unsigned int model_input_width = 320;
/** @brief This detection head's input height; LOOM does not reuse its box semantics. */
constexpr unsigned int model_input_height = 320;
/** The model emits boxes, scores, classes, and a detection count in this order.
 */
constexpr uint32_t expected_output_tensor_count = 4;
constexpr uint32_t bbox_values_per_candidate = 4; /**< Left/top/right/bottom values in each candidate. */
constexpr uint32_t score_values_before_candidates = 4; /**< Scores follow all box coordinate values. */
constexpr uint32_t class_values_before_candidates = 5; /**< Classes follow coordinates and scores. */
constexpr uint32_t values_per_candidate_with_score_and_class = 6; /**< Count follows the three candidate arrays. */
/** Maximum candidate count permitted by the model's fixed tensor scratch bound.
 */
constexpr uint32_t maximum_tensor_elements = IMX500_TENSOR_ELEMENT_LIMIT;
/** Maximum tensor rank documented by libcamera's Raspberry Pi IMX500 controls.
 */
constexpr uint32_t maximum_tensor_dimensions = IMX500_TENSOR_MAX_RANK;
/** Metadata layout: fixed header, tensor count, then fixed-size tensor records.
 */
constexpr size_t tensor_info_header_size = IMX500_TENSOR_HEADER_BYTES;
constexpr size_t tensor_count_offset =
    IMX500_TENSOR_COUNT_OFFSET; /**< Four-byte tensor count follows the header. */
constexpr size_t first_tensor_record_offset =
    IMX500_TENSOR_FIRST_RECORD_OFFSET; /**< First record follows count. */
constexpr size_t tensor_record_size =
    IMX500_TENSOR_RECORD_BYTES; /**< Fixed fields followed by sixteen 16-bit dimensions. */
constexpr size_t tensor_record_dimensions_offset =
    IMX500_TENSOR_DIMENSIONS_OFFSET; /**< Length and rank occupy the first eight bytes. */
constexpr size_t tensor_record_dimension_bytes =
    IMX500_TENSOR_DIMENSION_BYTES; /**< Each shape entry is an unsigned 16-bit value. */
constexpr size_t tensor_record_rank_offset =
    IMX500_TENSOR_RANK_OFFSET; /**< Rank follows the four-byte element count. */
constexpr size_t tensor_record_length_offset =
    IMX500_TENSOR_LENGTH_OFFSET; /**< Element count starts each record. */
/** Full IMX500 metadata structure size (16 records, each with up to 16
 * dimensions). */
constexpr size_t tensor_info_size =
    IMX500_TENSOR_INFO_BYTES;
/** Current single-class model contract: class zero, confidence 0.6, at most
 * five boxes. */
constexpr size_t maximum_detections = 5;
constexpr uint32_t object_class_index = 0; /**< Class selected by this detector's current model policy. */
constexpr float object_confidence_threshold = 0.6f; /**< Reject low-confidence boxes without invalidating the whole head. */
/** @brief One accepted object in final still-image coordinates.
 * @details The crop projection has already happened; consumers must not apply
 * the sensor-to-JPEG transform to these pixel coordinates a second time. */
struct BasicDetection {
  float confidence = 0; /**< Model confidence in [0, 1]. */
  /** @var spottercam::BasicDetection::x
   * @brief Left edge in JPEG pixels. */
  /** @var spottercam::BasicDetection::y
   * @brief Top edge in JPEG pixels. */
  /** @var spottercam::BasicDetection::width
   * @brief Positive accepted box width in JPEG pixels. */
  int x = 0, y = 0, width = 0, height = 0; /**< Positive accepted box height in JPEG pixels. */
};

/** Per-frame detector output; invalid metadata is distinct from a valid empty
 * result. */
struct BasicResult {
  bool valid =
      false; /**< True only when the complete tensor contract validates. */
  size_t count = 0; /**< Number of accepted object boxes, capped at five. */
  std::array<BasicDetection, maximum_detections>
      objects{}; /**< Bounded frame results. */
};

/** Decode one little-endian 32-bit integer from the driver's tensor-info byte
 * array.
 * @param data At least four readable bytes.
 * @return The unsigned value represented by @p data[0..3]. */
uint32_t read_u32(const uint8_t* data) {
  return uint32_t(data[0]) | uint32_t(data[1]) << 8 | uint32_t(data[2]) << 16 |
         uint32_t(data[3]) << 24;
}

/** Validate the model's four output tensors and project object boxes into the
 * JPEG.
 * @param tensor Interleaved boxes followed by scores, classes, and the
 * detection count.
 * @param info Driver CnnOutputTensorInfo bytes for this same camera request.
 * @param crop Sensor crop reported in that request's ScalerCrop metadata.
 * @param image Configured still-stream dimensions.
 * @return Invalid by default; valid with zero or more boxes only after full
 * validation.
 * @details This is intentionally model-specific. Raw neural arrays cannot be
 * treated as boxes until shape, integral count and coordinate assumptions hold. */
BasicResult decode(imx500::Span<const float> tensor,
                   imx500::Span<const uint8_t> info,
                   libcamera::Rectangle crop, libcamera::Size image) {
  BasicResult result;
  if (!info.data() || !tensor.data() || info.size() < tensor_info_size ||
      tensor.size() > maximum_tensor_elements ||
      read_u32(info.data() + tensor_count_offset) !=
          expected_output_tensor_count ||
      crop.x < 0 || crop.y < 0 || !crop.width || !crop.height ||
      crop.width > sensor_width || crop.height > sensor_height ||
      uint64_t(crop.x) + crop.width > sensor_width ||
      uint64_t(crop.y) + crop.height > sensor_height || !image.width ||
      !image.height || image.width > sensor_width ||
      image.height > sensor_height)
    return result;
  std::array<uint32_t, expected_output_tensor_count> lengths{};
  /* Validate every shape before indexing flattened values. Divide before
   * multiplying so a corrupt extent cannot overflow its bounded element budget. */
  for (size_t index = 0; index < lengths.size(); ++index) {
    const uint8_t* record =
        info.data() + first_tensor_record_offset + index * tensor_record_size;
    lengths[index] = read_u32(record + tensor_record_length_offset);
    uint32_t dimensions = read_u32(record + tensor_record_rank_offset);
    if (dimensions == 0 || dimensions > maximum_tensor_dimensions ||
        lengths[index] > maximum_tensor_elements)
      return result;
    uint32_t product = 1;
    for (uint32_t dimension = 0; dimension < dimensions; ++dimension) {
      size_t dimension_offset = tensor_record_dimensions_offset +
                                dimension * tensor_record_dimension_bytes;
      uint32_t size = uint32_t(record[dimension_offset]) |
                      uint32_t(record[dimension_offset + 1]) << 8;
      if (size == 0 || product > maximum_tensor_elements / size) return result;
      product *= size;
    }
    if (product != lengths[index]) return result;
  }
  uint32_t slots = lengths[0] / bbox_values_per_candidate;
  if (slots == 0 || lengths[0] % bbox_values_per_candidate ||
      lengths[1] != slots || lengths[2] != slots || lengths[3] != 1 ||
      tensor.size() != values_per_candidate_with_score_and_class * slots + 1)
    return result;
  // Output layout stores four box coordinates, then scores, classes, and one
  // total count.
  float count = tensor[values_per_candidate_with_score_and_class * slots];
  if (!std::isfinite(count) || count < 0 || count > slots ||
      std::floor(count) != count)
    return result;
  result.valid = true;
  /* Invalid individual candidates are omitted; a valid head with no accepted
   * candidates still produces a valid empty result, not analysis_failed. */
  for (size_t index = 0; index < static_cast<size_t>(count); ++index) {
    size_t box_offset = bbox_values_per_candidate * index;
    float score = tensor[score_values_before_candidates * slots + index];
    float category = tensor[class_values_before_candidates * slots + index];
    float left = tensor[box_offset], top = tensor[box_offset + 1];
    float right = tensor[box_offset + 2], bottom = tensor[box_offset + 3];
    if (category != object_class_index || !std::isfinite(score) ||
        score < object_confidence_threshold || score > 1 ||
        !std::isfinite(left) || !std::isfinite(top) || !std::isfinite(right) ||
        !std::isfinite(bottom) || left < 0 || top < 0 || right <= left ||
        bottom <= top || right > model_input_width ||
        bottom > model_input_height)
      continue;
    size_t position = 0;
    while (position < result.count && result.objects[position].confidence >= score) ++position;
    if (position == maximum_detections) continue;
    // Normalize model coordinates, map to sensor pixels, remove ScalerCrop
    // origin, then scale to the JPEG.
    auto project = [](float coordinate, unsigned int sensor, int start,
                      unsigned int extent, unsigned int output) {
      double offset = std::clamp(
          double(coordinate) / model_input_width * (sensor - 1) - start, 0.0,
          double(extent));
      return int(std::lround(offset * output / extent));
    };
    int x0 = project(left, sensor_width, crop.x, crop.width, image.width);
    int x1 = project(right, sensor_width, crop.x, crop.width, image.width);
    int y0 = project(top, sensor_height, crop.y, crop.height, image.height);
    int y1 = project(bottom, sensor_height, crop.y, crop.height, image.height);
    if (x1 <= x0 || y1 <= y0) continue;
    if (result.count < maximum_detections) ++result.count;
    for (size_t moved = result.count - 1; moved > position; --moved)
      result.objects[moved] = result.objects[moved - 1];
    result.objects[position] = {score, x0, y0, x1 - x0, y1 - y0};
  }
  return result;
}


/** @brief Interpret only the detector model's outputs from this retained exposure.
 * @param frame Borrowed same-exposure tensors, crop and configured image size.
 * @return Invalid without crop evidence, otherwise the validated detector result.
 * @details A guessed full-sensor crop could put correct boxes in the wrong JPEG
 * position, so missing geometry is a failed analysis rather than a default. */
BasicResult frame_result(const imx500::FrameView &frame)
{
  if (!frame.crop_valid) {
    return {};
  }
  return decode(frame.tensor, frame.tensor_info, frame.crop, frame.image);
}

/** @brief Qualify startup from a valid detector head, including zero detections.
 * @param frame Retained exposure supplied by the neutral camera worker.
 * @return True when this model's decoder contract holds; not necessarily a hit. */
bool detector_ready(const imx500::FrameView &frame)
{
  return frame_result(frame).valid;
}

/** @brief Publish detector results, never LOOM features or navigation evidence.
 * @param frame Borrowed exposure whose image/tensor identity is retained by a lease.
 * @param filename New analysis destination selected by the camera publication owner.
 * @return True after exclusive-create, flush, sync and close; false retains no partial output.
 * @details Readiness can describe an earlier warm-up frame. Decode this frame
 * again rather than reusing a previous result for newly captured pixels. */
bool write_detections(const imx500::FrameView &frame, const char *filename)
{
  const BasicResult result = frame_result(frame);
  if (!result.valid) {
    return false;
  }
  FILE *output = std::fopen(filename, "wx");
  if (!output) {
    return false;
  }
  std::fprintf(output,
               "schema=USHER_OBJECTS_V2\nsequence=%llu\nsensor_timestamp_ns=%llu\n"
               "image_width=%u\nimage_height=%u\ncount=%zu\n",
               static_cast<unsigned long long>(frame.sequence),
               static_cast<unsigned long long>(frame.sensor_timestamp_ns),
               frame.image.width, frame.image.height, result.count);
  for (size_t index = 0; index < result.count; ++index) {
    const auto &box = result.objects[index];
    std::fprintf(output,
                 "detection.%zu.name=vehicle\ndetection.%zu.confidence=%.6f\n"
                 "detection.%zu.x=%d\ndetection.%zu.y=%d\ndetection.%zu.width=%d\ndetection.%zu.height=%d\n",
                 index, index, double(box.confidence), index, box.x, index, box.y, index, box.width, index, box.height);
  }
  bool written = std::fflush(output) == 0 && !std::ferror(output) && fsync(fileno(output)) == 0;
  if (std::fclose(output) != 0) {
    written = false;
  }
  if (!written) {
    unlink(filename);
  }
  return written;
}

}  // namespace spottercam

using namespace spottercam;
#endif

#ifndef SPOTTER_CAM_TEST
namespace {
/** @brief Assignment-level gates, deliberately separate from immutable model preprocessing. */
struct OnnxDetectorOptions {
  double threshold = 0.6;
  unsigned category = 0, limit = 5, seen = 0;
};

/** @brief Reject duplicate assignment controls rather than letting INI field order decide detection policy. */
bool detector_setting(void *context, const char *key, const char *value)
{
  auto &options = *static_cast<OnnxDetectorOptions *>(context);
  unsigned bit;
  if (!std::strcmp(key, "score_threshold")) bit = 1;
  else if (!std::strcmp(key, "class_index")) bit = 2;
  else if (!std::strcmp(key, "max_detections")) bit = 4;
  else return false;
  if (options.seen & bit) return false;
  options.seen |= bit;
  if (bit == 1) return ini_real(value, &options.threshold) && options.threshold >= 0 && options.threshold <= 1;
  uint64_t number = 0;
  if (!ini_uint(value, &number) || number > (bit == 2 ? 65535U : 64U) || (bit == 4 && !number)) return false;
  if (bit == 2) options.category = static_cast<unsigned>(number);
  else options.limit = static_cast<unsigned>(number);
  return true;
}

/** @brief Read the declared numeric representation without assuming all exporters use float class/count tensors. */
bool tensor_number(const onnx_tensor &tensor, size_t index, double &number)
{
  const size_t element = tensor.type == ONNX_INT64 ? 8 : 4;
  if (index >= tensor.bytes / element) return false;
  if (tensor.type == ONNX_FLOAT32) number = static_cast<const float *>(tensor.data)[index];
  else if (tensor.type == ONNX_INT64) number = static_cast<double>(static_cast<const int64_t *>(tensor.data)[index]);
  else if (tensor.type == ONNX_INT32) number = static_cast<const int32_t *>(tensor.data)[index];
  else return false;
  return std::isfinite(number);
}

/** @brief Qualify output layout and label policy before readiness, not on the first aircraft request. */
bool validate_onnx_detector(void *context, const inference_context *network, char *error, size_t capacity)
{
  const auto &options = *static_cast<OnnxDetectorOptions *>(context);
  const auto &model = network->model;
  char label[64];
  if (model.input_count != 1 || !model.image.channels ||
      !inference_model_label(&model, options.category, label, sizeof(label))) {
    std::snprintf(error, capacity, "Invalid spotter_boxes_xyxy_v1 tensor or label contract");
    return false;
  }
  if (model.output_count == 1) {
    const auto &output = model.outputs[0];
    const int64_t channels = output.rank == 3 ? output.dimensions[1] : 0;
    const int64_t slots = output.rank == 3 ? output.dimensions[2] : 0;
    if (output.type != ONNX_FLOAT32 || output.rank != 3 || output.dimensions[0] != 1 ||
        channels < 5 || channels > 68 || slots <= 0 || slots > 8400 ||
        options.category >= static_cast<unsigned>(channels - 4) ||
        output.bytes != static_cast<size_t>(channels) * static_cast<size_t>(slots) * sizeof(float)) {
      std::snprintf(error, capacity, "Invalid spotter_boxes_xyxy_v1 tensor or label contract");
      return false;
    }
    return true;
  }
  const auto integer_size = [](const onnx_tensor &tensor) -> size_t {
    return tensor.type == ONNX_INT64 ? 8 : tensor.type == ONNX_INT32 || tensor.type == ONNX_FLOAT32 ? 4 : 0;
  };
  const size_t slots = model.outputs[0].bytes / (4 * sizeof(float));
  const size_t classes = integer_size(model.outputs[2]), count = integer_size(model.outputs[3]);
  if (model.output_count != 4 ||
      model.outputs[0].type != ONNX_FLOAT32 || model.outputs[0].bytes % (4 * sizeof(float)) ||
      !slots || slots > 4096 || model.outputs[1].type != ONNX_FLOAT32 || model.outputs[1].bytes != slots * sizeof(float) ||
      !classes || model.outputs[2].bytes != slots * classes || !count || model.outputs[3].bytes != count) {
    std::snprintf(error, capacity, "Invalid spotter_boxes_xyxy_v1 tensor or label contract");
    return false;
  }
  return true;
}

struct DetectionCandidate {
  double score;
  int x, y, width, height;
  double center_x, center_y;
};

static double box_iou(const DetectionCandidate &first, const DetectionCandidate &second)
{
  const int left = std::max(first.x, second.x);
  const int top = std::max(first.y, second.y);
  const int right = std::min(first.x + first.width, second.x + second.width);
  const int bottom = std::min(first.y + first.height, second.y + second.height);
  if (right <= left || bottom <= top) return 0.0;
  const double intersection = static_cast<double>(right - left) * static_cast<double>(bottom - top);
  const double first_area = static_cast<double>(first.width) * static_cast<double>(first.height);
  const double second_area = static_cast<double>(second.width) * static_cast<double>(second.height);
  const double union_area = first_area + second_area - intersection;
  return union_area > 0.0 ? intersection / union_area : 0.0;
}

static void insert_detection(DetectionCandidate (&detections)[64], size_t &accepted,
                             unsigned limit, const DetectionCandidate &candidate, bool suppress_overlap)
{
  if (suppress_overlap) {
    for (size_t index = 0; index < accepted; ++index) {
      if (detections[index].score >= candidate.score && box_iou(detections[index], candidate) > 0.45) return;
    }
    size_t retained = 0;
    for (size_t index = 0; index < accepted; ++index) {
      if (detections[index].score >= candidate.score || box_iou(detections[index], candidate) <= 0.45) {
        detections[retained++] = detections[index];
      }
    }
    accepted = retained;
  }
  size_t position = 0;
  while (position < accepted && detections[position].score >= candidate.score) ++position;
  if (position >= limit) return;
  if (accepted < limit) ++accepted;
  for (size_t moved = accepted - 1; moved > position; --moved) detections[moved] = detections[moved - 1];
  detections[position] = candidate;
}

/** @brief Rank bounded boxes and publish source/model identity so saved-picture results cannot attach to another JPEG. */
bool write_onnx_detections(void *context, const inference_context *network,
                            const char *image, uint32_t sequence, char *error, size_t capacity)
{
  const auto &options = *static_cast<OnnxDetectorOptions *>(context);
  const auto &model = network->model;
  char label[64];
  if (!inference_model_label(&model, options.category, label, sizeof(label))) return false;
  DetectionCandidate detections[64]{};
  size_t accepted = 0;
  if (model.input_count == 1 && model.output_count == 1 && model.image.channels &&
      model.outputs[0].type == ONNX_FLOAT32 && model.outputs[0].rank == 3 &&
      model.outputs[0].dimensions[0] == 1 && model.outputs[0].dimensions[1] >= 5 &&
      model.outputs[0].dimensions[1] <= 68 && model.outputs[0].dimensions[2] > 0 &&
      model.outputs[0].dimensions[2] <= 8400 &&
      options.category < static_cast<unsigned>(model.outputs[0].dimensions[1] - 4)) {
    const size_t channels = static_cast<size_t>(model.outputs[0].dimensions[1]);
    const size_t slots = static_cast<size_t>(model.outputs[0].dimensions[2]);
    if (model.outputs[0].bytes != channels * slots * sizeof(float)) {
      std::snprintf(error, capacity, "Invalid ONNX detection count or score shape");
      return false;
    }
    const auto *values = static_cast<const float *>(model.outputs[0].data);
    const double min_x = network->transform.left;
    const double max_x = network->transform.left + network->transform.content_width;
    const double min_y = network->transform.top;
    const double max_y = network->transform.top + network->transform.content_height;
    const double max_box_width = 0.30 * network->transform.input_width;
    const double max_box_height = 0.30 * network->transform.input_height;
    for (size_t index = 0; index < slots; ++index) {
      const double cx = values[index], cy = values[slots + index];
      const double width = values[2 * slots + index], height = values[3 * slots + index];
      const double score = values[(4 + options.category) * slots + index];
      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) ||
          !std::isfinite(height) || !std::isfinite(score) ||
          score < options.threshold || score > 1.0 || width <= 0.0 || height <= 0.0 ||
          width > max_box_width || height > max_box_height ||
          cx < min_x || cx > max_x || cy < min_y || cy > max_y) continue;
      bool best_class = true;
      for (size_t category = 0; category + 4 < channels; ++category) {
        const double other = values[(4 + category) * slots + index];
        if (!std::isfinite(other) || other > score) { best_class = false; break; }
      }
      if (!best_class) continue;
      const double left = std::clamp(cx - 0.5 * width, min_x, max_x);
      const double top = std::clamp(cy - 0.5 * height, min_y, max_y);
      const double right = std::clamp(cx + 0.5 * width, min_x, max_x);
      const double bottom = std::clamp(cy + 0.5 * height, min_y, max_y);
      double x0, y0, x1, y1, center_x, center_y;
      if (left >= right || top >= bottom ||
          !inference_image_point(&network->transform, left, top, &x0, &y0) ||
          !inference_image_point(&network->transform, right, bottom, &x1, &y1) ||
          !inference_image_point(&network->transform, cx, cy, &center_x, &center_y)) continue;
      const int start_x = static_cast<int>(std::lround(x0)), start_y = static_cast<int>(std::lround(y0));
      const int end_x = static_cast<int>(std::lround(x1)), end_y = static_cast<int>(std::lround(y1));
      if (end_x <= start_x || end_y <= start_y) continue;
      insert_detection(detections, accepted, options.limit,
                       {score, start_x, start_y, end_x - start_x, end_y - start_y,
                        std::clamp(center_x, 0.0, double(network->transform.source_width - 1)),
                        std::clamp(center_y, 0.0, double(network->transform.source_height - 1))}, true);
    }
  } else {
    if (model.input_count != 1 || model.output_count != 4 || !model.image.channels ||
        model.outputs[0].type != ONNX_FLOAT32 || model.outputs[1].type != ONNX_FLOAT32 ||
        model.outputs[0].bytes % (4 * sizeof(float))) {
      std::snprintf(error, capacity, "spotter_boxes_xyxy_v1 requires boxes, scores, classes and count");
      return false;
    }
    const size_t slots = model.outputs[0].bytes / (4 * sizeof(float));
    double count = 0;
    if (!slots || slots > 4096 || model.outputs[1].bytes != slots * sizeof(float) ||
        !tensor_number(model.outputs[3], 0, count) || count < 0 || count > slots || std::floor(count) != count) {
      std::snprintf(error, capacity, "Invalid ONNX detection count or score shape");
      return false;
    }
    const auto *boxes = static_cast<const float *>(model.outputs[0].data);
    for (size_t index = 0; index < static_cast<size_t>(count); ++index) {
      double score = 0, category = 0;
      if (!tensor_number(model.outputs[1], index, score) || !tensor_number(model.outputs[2], index, category) ||
          category != options.category || score < options.threshold || score > 1) continue;
      const double left = boxes[4 * index], top = boxes[4 * index + 1];
      const double right = boxes[4 * index + 2], bottom = boxes[4 * index + 3];
      double x0, y0, x1, y1;
      if (left >= right || top >= bottom ||
          !inference_image_point(&network->transform, left, top, &x0, &y0) ||
          !inference_image_point(&network->transform, right, bottom, &x1, &y1)) continue;
      const int start_x = static_cast<int>(std::lround(x0)), start_y = static_cast<int>(std::lround(y0));
      const int end_x = static_cast<int>(std::lround(x1)), end_y = static_cast<int>(std::lround(y1));
      if (end_x <= start_x || end_y <= start_y) continue;
      insert_detection(detections, accepted, options.limit,
                       {score, start_x, start_y, end_x - start_x, end_y - start_y,
                        std::clamp((x0 + x1) / 2, 0.0, double(network->transform.source_width - 1)),
                        std::clamp((y0 + y1) / 2, 0.0, double(network->transform.source_height - 1))}, false);
    }
  }
  char filename[INFERENCE_PATH_CAPACITY], source_digest[65];
  const size_t length = std::strlen(image);
  if (length < 4 || std::strcmp(image + length - 4, ".jpg") ||
      !inference_file_sha256(image, source_digest)) {
    std::snprintf(error, capacity, "Cannot identify inference source JPEG");
    return false;
  }
  const int size = std::snprintf(filename, sizeof(filename), "%.*s.objects.ini", static_cast<int>(length - 4), image);
  if (size < 0 || static_cast<size_t>(size) >= sizeof(filename)) return false;
  FILE *output = std::fopen(filename, "wx");
  if (!output) { std::snprintf(error, capacity, "Cannot exclusively create %s", filename); return false; }
  std::fprintf(output, "schema=USHER_OBJECTS_V2\nsequence=%llu\nsensor_timestamp_ns=%llu\n"
               "image_width=%u\nimage_height=%u\ncount=%zu\n"
               "model_sha256=%s\nmodel_descriptor_sha256=%s\nimage_sha256=%s\n",
               static_cast<unsigned long long>(network->capture_identity ? network->frame_sequence : sequence),
               static_cast<unsigned long long>(network->sensor_timestamp_ns),
               network->transform.source_width, network->transform.source_height, accepted,
               model.graph_sha256, model.descriptor_sha256, source_digest);
  for (size_t index = 0; index < accepted; ++index) {
    const auto &box = detections[index];
    std::fprintf(output, "detection.%zu.name=%s\ndetection.%zu.confidence=%.6f\n"
                 "detection.%zu.x=%d\ndetection.%zu.y=%d\ndetection.%zu.width=%d\ndetection.%zu.height=%d\n",
                 index, label, index, box.score, index, box.x, index, box.y, index, box.width, index, box.height);
          std::fprintf(output, "detection.%zu.center_x=%.12g\ndetection.%zu.center_y=%.12g\n",
                 index, box.center_x, index, box.center_y);
  }
  bool written = std::fflush(output) == 0 && !std::ferror(output) && fsync(fileno(output)) == 0;
  if (std::fclose(output)) written = false;
  if (!written) { unlink(filename); std::snprintf(error, capacity, "Cannot publish ONNX detections"); }
  return written;
}
}

/** @brief Select simulation input using the common bounded settings parser.
 * @details Only an absent model permits fabricated spotting. A present ONNX
 * file is qualified by the normal shared analysis worker, never by this probe. */
static int simulation_mode(int argc, char **argv)
{
  if (argc != 2 && (argc != 4 || std::strcmp(argv[2], "--settings"))) return 2;
  char filename[INFERENCE_PATH_CAPACITY], error[2048];
  const char *path = argc == 4 ? argv[3] : nullptr;
  if (!path) {
    if (!inference_settings_find("spottercam", "spotter_cam", filename, sizeof(filename))) {
      std::puts("RANDOM settings_not_found");
      return 0;
    }
    path = filename;
  }
  inference_settings settings;
  OnnxDetectorOptions options;
  inference_settings_init(&settings, INFERENCE_AUTO);
  if (!inference_settings_read(&settings, path, detector_setting, &options, error, sizeof(error))) {
    std::fprintf(stderr, "spottercam: %s\n", error);
    return 1;
  }
  if (!settings.model[0]) {
    std::puts("RANDOM model_not_set");
    return 0;
  }
  const char *suffix = std::strrchr(settings.model, '.');
  if (settings.backend == INFERENCE_IMX500 && suffix && !std::strcmp(suffix, ".rpk")) {
    const char *slash = std::strrchr(settings.model, '/');
    if (slash) {
      static const char *const candidates[] = {"vehicle_yolo11n_int8.onnx", "vehicle_yolo11n_fp32.onnx"};
      const int directory_length = static_cast<int>(slash - settings.model);
      for (const char *candidate_name : candidates) {
        char candidate_path[INFERENCE_PATH_CAPACITY];
        const int written = std::snprintf(candidate_path, sizeof(candidate_path), "%.*s/%s",
                                          directory_length, settings.model, candidate_name);
        struct stat candidate_status;
        if (written > 0 && static_cast<size_t>(written) < sizeof(candidate_path) &&
            !stat(candidate_path, &candidate_status) && S_ISREG(candidate_status.st_mode) &&
            !access(candidate_path, R_OK)) {
          std::puts("ONNX");
          return 0;
        }
      }
    }
    std::puts("RANDOM rpk_requires_ai_camera");
    return 0;
  }
  if (settings.backend != INFERENCE_ONNX || !suffix || std::strcmp(suffix, ".onnx")) {
    std::fprintf(stderr, "spottercam: simulation requires an automatic or ONNX .onnx model\n");
    return 1;
  }
  struct stat status;
  if (stat(settings.model, &status)) {
    if (errno == ENOENT || errno == ENOTDIR) {
      std::puts("RANDOM onnx_model_missing");
      return 0;
    }
    std::fprintf(stderr, "spottercam: cannot inspect ONNX model: %s\n", std::strerror(errno));
    return 1;
  }
  if (!S_ISREG(status.st_mode) || access(settings.model, R_OK)) {
    std::fprintf(stderr, "spottercam: ONNX model is not a readable regular file\n");
    return 1;
  }
  std::puts("ONNX");
  return 0;
}

/** @brief Project one delivered-image point with the shared camera/coverage geometry.
 * @details Emits east/north offsets, not a new navigation command. The caller
 * supplies the observed pose and retains ownership of the geographic datum. */
static int project_ground(int argc, char **argv)
{
  if (argc != 13) return 2;
  double values[10];
  for (unsigned index = 0; index < 10; ++index)
    if (!ini_real(argv[index + 3], &values[index])) return 2;
  usher_camera_profile profile;
  if (!usher_camera_profile_read(argv[2], &profile)) {
    std::fprintf(stderr, "spottercam: invalid ground-projection camera profile\n");
    return 1;
  }
  if (values[8] < 2 || values[8] > 4096 || values[9] < 2 || values[9] > 4096 ||
      std::floor(values[8]) != values[8] || std::floor(values[9]) != values[9]) return 2;
  auto &camera = profile.calibration;
  const double scale_x = values[8] / camera.width, scale_y = values[9] / camera.height;
  camera.fx *= scale_x;
  camera.fy *= scale_y;
  camera.cx = (camera.cx + 0.5) * scale_x - 0.5;
  camera.cy = (camera.cy + 0.5) * scale_y - 0.5;
  camera.width = static_cast<unsigned>(values[8]);
  camera.height = static_cast<unsigned>(values[9]);
  const coverage_pose pose{values[0], values[1], values[2], values[3], values[4], values[5], 0};
  coverage_point offset;
  if (!coverage_project_pixel(&camera, &pose, values[6], values[7], &offset)) {
    std::fprintf(stderr, "spottercam: image point cannot reach the configured ground plane\n");
    return 1;
  }
  std::printf("GROUND %.9f %.9f\n", offset.east, offset.north);
  return 0;
}

/** @brief Publish a proposed slot and commit only after the transport confirms it.
 * @details Coordinates retain the USHER ellipsoid reference; the transport owns
 * conversion to its waypoint altitude datum. An absent or negative ACK leaves
 * the native ranking unchanged. */
struct TargetTransaction {
  long long sequence = 0;
  bool channel_failed = false;
};

/** @brief Read one bounded printable protocol line.
 * @param[out] line Buffer for the line, newline and terminator.
 * @param capacity Total buffer bytes.
 * @param acknowledgement True to impose a deadline on the read.
 * @return One for a complete line, zero for clean EOF, or -1 on failure.
 * @details Keep one deadline across all bytes of an acknowledgement so a
 * partially responsive peer cannot prolong a pending transaction indefinitely. */
static int target_line(char *line, size_t capacity, bool acknowledgement)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(4000);
  size_t length = 0;
  while (length + 1 < capacity) {
    if (acknowledgement) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      if (remaining <= 0) return -1;
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int ready = poll(&input, 1, static_cast<int>(remaining));
      if (ready < 0 && errno == EINTR) continue;
      if (ready <= 0) return -1;
    }
    const int character = std::fgetc(stdin);
    if (character == EOF) return length ? -1 : 0;
    if (character != '\n' && (character < 32 || character >= 127)) return -1;
    line[length++] = static_cast<char>(character);
    if (character == '\n') { line[length] = '\0'; return 1; }
  }
  return -1;
}

/** @brief Propose a target update and require its matching acknowledgement.
 * @param[in,out] context Active TargetTransaction holding the correlation sequence.
 * @param slot Proposed target slot.
 * @param target Borrowed ranked target observation.
 * @return True only if the peer acknowledged this transaction.
 * @details Flush before reading the reply. Distinguish a missing or malformed
 * line from a complete nonmatching response so the caller can detect channel failure. */
static bool publish_target(void *context, unsigned slot, const spotter_target *target)
{
  auto &transaction = *static_cast<TargetTransaction *>(context);
  const auto sequence = transaction.sequence;
  if (std::printf("MOVE %lld %u %d %d %d %u\n", sequence, slot,
                  target->latitude, target->longitude, target->altitude_mm,
                  static_cast<unsigned>(target->score)) < 0 || std::fflush(stdout)) return false;
  char reply[80], expected[80];
  std::snprintf(expected, sizeof(expected), "ACK %lld\n", sequence);
  const int received = target_line(reply, sizeof(reply), true);
  if (received != 1) transaction.channel_failed = true;
  return received == 1 && !std::strcmp(reply, expected);
}

/** @brief Run a bounded, model-independent geographic target stream.
 * @return Zero on orderly EOF, two for malformed/out-of-order input, one for I/O failure.
 * @details OBS sequence latitude longitude ellipsoid_mm score proposes one
 * observation. MOVE awaits ACK sequence; OK commits, SKIP retains existing slots.
 * Restarting the worker clears all five slots without moving any FC waypoint. */
static int target_server()
{
  if (std::setvbuf(stdin, nullptr, _IONBF, 0)) return 1;
  spotter_targets targets{};
  long long previous = 0;
  TargetTransaction transaction;
  char line[192];
  std::puts("READY SENSORAIA_TARGETS_V1");
  if (std::fflush(stdout)) return 1;
  int line_status;
  while ((line_status = target_line(line, sizeof(line), false)) > 0) {
    if (!std::strchr(line, '\n') || std::strncmp(line, "OBS ", 4)) return 2;
    char *cursor = line + 4;
    long long values[5]{};
    for (auto &value : values) {
      char *end = nullptr;
      errno = 0;
      value = std::strtoll(cursor, &end, 10);
      if (errno || end == cursor || (*end != ' ' && *end != '\n')) return 2;
      cursor = end;
      while (*cursor == ' ') ++cursor;
    }
    if (std::strcmp(cursor, "\n") || values[0] <= previous || values[0] > UINT32_MAX ||
        values[1] < -900000000 || values[1] > 900000000 ||
        values[2] < -1800000000 || values[2] > 1800000000 ||
        values[3] < INT32_MIN || values[3] > INT32_MAX || values[4] < 0 || values[4] > 1000) return 2;
    previous = values[0];
    transaction.sequence = previous;
    const spotter_target observation{static_cast<int32_t>(values[1]), static_cast<int32_t>(values[2]),
      static_cast<int32_t>(values[3]), static_cast<uint16_t>(values[4]), true};
    const int slot = spotter_targets_offer(&targets, &observation, publish_target, &transaction);
    if (transaction.channel_failed) return 1;
    if (slot < 0) std::printf("SKIP %lld\n", previous);
    else std::printf("OK %lld %d\n", previous, slot);
    if (std::fflush(stdout)) return 1;
  }
  return line_status < 0 ? 2 : std::ferror(stdin) ? 1 : 0;
}

/** @brief Select detector-specific policy, then run neutral capture mechanics.
 * @param argc Command-line token count.
 * @param argv Version or probe/capture/serve arguments.
 * @return Zero on success, two for syntax errors, one for worker failure.
 * @details The explicit .objects.ini callback keeps object interpretation out
 * of the shared camera and out of LOOM's independently linked executable. */
int main(int argc, char **argv)
{
  if (argc == 2 && !std::strcmp(argv[1], "--target-server")) return target_server();
  if (argc >= 2 && !std::strcmp(argv[1], "--simulation-mode")) return simulation_mode(argc, argv);
  if (argc >= 2 && !std::strcmp(argv[1], "--project-ground")) return project_ground(argc, argv);
  if (argc == 2 && !std::strcmp(argv[1], "--help")) {
    std::puts(SPOTTERCAM_BUILD_VERSION);
    std::puts("Usage: spottercam --analyze FILE.jpg | --analyze-server | --check-model [--settings FILE]\n"
  #ifndef SENSORAIA_INFERENCE_ONLY
              "       spottercam --capture --settings FILE /absolute/photo.jpg\n"
              "       spottercam --serve --settings FILE WIDTH HEIGHT\n"
          "       spottercam --photo-only [--probe | --capture /absolute/photo.jpg | --serve WIDTH HEIGHT]\n"
  #endif
              "       spottercam --target-server\n"
              "       spottercam --simulation-mode [--settings FILE]\n"
              "       spottercam --project-ground PROFILE LAT LON AGL ROLL PITCH YAW X Y WIDTH HEIGHT\n"
              "       spottercam --probe MODEL.rpk | --version\n"
              "Picture analysis accepts --sequence N; settings select the model/backend.");
    return 0;
  }
  OnnxDetectorOptions options;
  const inference_image_processor onnx_processor{"spottercam", "spotter_cam", "spotter_boxes_xyxy_v1",
                                                 detector_setting, &options, write_onnx_detections, validate_onnx_detector, ".objects.ini"};
  const int analyzed = inference_worker_cli(argc, argv, &onnx_processor);
  if (analyzed >= 0) return analyzed;
  if (argc == 2 && std::strcmp(argv[1], "--version") == 0) {
    std::puts(SPOTTERCAM_BUILD_VERSION);
    return 0;
  }
#ifdef SENSORAIA_INFERENCE_ONLY
  std::fprintf(stderr, "Usage: spottercam --analyze FILE.jpg | --analyze-server | --check-model [--settings FILE]\n");
  return 2;
#else
  if (argc > 1 && !std::strcmp(argv[1], "--photo-only")) {
    return imx500::run_photo_worker(argc - 1, argv + 1, "spottercam");
  }
  const imx500::FrameProcessor processor{".objects.ini", detector_ready, write_detections};
  const int configured = inference_capture_cli(argc, argv, onnx_processor, processor);
  if (configured >= 0) return configured;
  if (argc < 3 || (std::strcmp(argv[1], "--probe") != 0
                  && std::strcmp(argv[1], "--serve") != 0
                  && std::strcmp(argv[1], "--capture") != 0)) {
    std::fprintf(stderr,
                 "Usage: spottercam --probe MODEL.rpk\n"
                 "       spottercam --serve MODEL.rpk WIDTH HEIGHT\n"
                 "       spottercam --capture MODEL.rpk /absolute/photo.jpg\n"
                 "       spottercam --analyze FILE.jpg --settings FILE [--sequence N]\n"
                 "       spottercam --analyze-server --settings FILE\n"
                 "       spottercam --check-model --settings FILE\n"
                 "       spottercam --version\n");
    return 2;
  }
  return imx500::run_worker(argc, argv, processor, "spottercam");
#endif
}
#endif
