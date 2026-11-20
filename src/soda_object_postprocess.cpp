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
 * @file soda_object_postprocess.cpp
 * @brief EXIF-driven object-hit post-processing for SODA.
 * @details This component does not run inference. It reads USHER's object
 * record, creates sibling review copies and a padded crop, and appends one CSV
 * row. The original flight JPEG is never changed.
 */
#include "soda_object_postprocess.h"

#include <fcntl.h>
// clang-format off
#include <stddef.h>
#include <stdio.h>
#include <jpeglib.h>
// clang-format on
#include "exifite.h"
#include <setjmp.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <string>
#include <vector>

#include "path_utils.h"
#include "jpeg_marker_guard.h"

/** Parsing, drawing, and file-writing helpers used by the public SODA entry
 * point. */
namespace soda_object_postprocess_detail {

/** Matches image_exif.c's write_tagged_comment(): an 8-byte ASCII-charset
 * prefix precedes the text payload in EXIF_TAG_USER_COMMENT, and the tagged
 * record itself is introduced by
 * "\n<TAG>\n". The entry is zero-filled before being written and sized one byte
 * past the visible text (image_exif.c's replace_tag()/write_tagged_comment()),
 * so a trailing NUL is present in practice, but entry->size (not strlen()) is
 * used regardless, to stay correct even if that trailing byte is ever not zero.
 */
constexpr size_t kAsciiCommentPrefixSize = 8;
constexpr char kObjectDetectionMarker[] = "\nAICAM_OBJECTS_V1\n";
constexpr size_t kObjectLabelCapacity = 64;
/** Same default as the now-removed Python cropper, kept identical so a ported
 * call site produces the same crop given the same box. */
constexpr double kCropMargin = 0.18;
/** Matches the outline width the now-removed Python draw_hits() used (PIL's
 * ImageDraw.rectangle(..., width=4)). There is no text label here -- see
 * draw_box_outline()'s comment for why. */
constexpr int kBoxOutlineThickness = 4;
/** Fixed copy buffer avoids allocating a second full-resolution JPEG-sized
 * block. */
constexpr size_t kCopyChunkBytes = 65536;
/** Bounded detection row; rows stay comfortably below append atomicity limits.
 */
constexpr size_t kCsvRowCapacity = 512;
/** Keep crop output quality consistent with the previous SODA crop path. */
constexpr int kCropJpegQuality = 95;

struct ObjectDetection {
  int count = 0; /**< Total object detections reported for this shot. */
  std::string
      label; /**< Bounded model-provided label of the selected object. */
  double confidence = 0.0; /**< Confidence of the selected best box. */
  /**< Selected box in original JPEG coordinates. */
  int box_x = 0, box_y = 0, box_w = 0, box_h = 0;
};

/** Half-open [left,right) x [top,bottom) crop rectangle. */
struct CropBox {
  int left, top, right, bottom;
};

/** RGB pixels and dimensions decoded once for annotation and crop output. */
struct DecodedImage {
  std::vector<unsigned char> pixels;
  unsigned width = 0;
  unsigned height = 0;
};

/** libjpeg error state with local recovery for malformed input streams. */
struct JpegErrorContext {
  struct jpeg_error_mgr manager;
  jmp_buf recovery;
  void (*emit)(j_common_ptr, int);
};

/** Redirect libjpeg fatal errors to the caller's setjmp cleanup path.
 * @param jpeg_info Active libjpeg common state. */
void jpeg_error_exit(j_common_ptr jpeg_info) {
  JpegErrorContext* error = reinterpret_cast<JpegErrorContext*>(jpeg_info->err);
  longjmp(error->recovery, 1);
}

/** @brief Check JPEG marker bounds before forwarding a codec diagnostic.
 * @param state Codec state carrying the installed JpegErrorContext.
 * @param level libjpeg diagnostic level.
 * @details Retain libjpeg's normal warning accounting for accepted input. */
void jpeg_message(j_common_ptr state, int level) {
  usher_jpeg_validate_marker(state);
  reinterpret_cast<JpegErrorContext*>(state->err)->emit(state, level);
}

/** @brief Parse an already-decoded EXIF_TAG_USER_COMMENT payload for an
 * AICAM_OBJECTS_V1 hit. Kept separate from the EXIF/file I/O in
 * read_object_detection() so the parsing logic is unit-testable against
 * synthetic buffers without needing a real JPEG on disk.
 * @param comment_text Complete EXIF UserComment payload after charset prefix.
 * @param[out] out Parsed detection fields on success.
 * @return true only for a genuine hit; false when the marker is absent (every
 * non-spottercam image) or the summary after it is "status=ok; count=0" /
 * "status=analysis_failed; ...", both of which fail to match the count>=1
 * pattern below. */
bool parse_object_summary(const std::string& comment_text,
                          ObjectDetection* out) {
  size_t marker_pos = comment_text.find(kObjectDetectionMarker);
  if (marker_pos == std::string::npos) {
    return false;
  }
  std::string summary =
      comment_text.substr(marker_pos + std::strlen(kObjectDetectionMarker));

  int count = 0, box_x = 0, box_y = 0, box_w = 0, box_h = 0;
  double confidence = 0.0;
  char label[kObjectLabelCapacity];
  if (std::sscanf(
          summary.c_str(),
          "status=ok; count=%d; label=%63[^;]; confidence=%lf; box=%d,%d,%d,%d",
          &count, label, &confidence, &box_x, &box_y, &box_w, &box_h) != 7 ||
      count < 1 || !std::isfinite(confidence) || confidence < 0.0 ||
      confidence > 1.0 || box_x < 0 || box_y < 0 || box_w <= 0 || box_h <= 0) {
    return false;
  }

  out->count = count;
  out->label = label;
  out->confidence = confidence;
  out->box_x = box_x;
  out->box_y = box_y;
  out->box_w = box_w;
  out->box_h = box_h;
  return true;
}

/** @brief Read the AICAM_OBJECTS_V1 record usher's
 * image_exif_write_object_detections() writes into EXIF_TAG_USER_COMMENT, if
 * present and reporting count >= 1.
 * @return true only for a genuine hit; false for a read/parse failure, a shot
 * without the marker (every non-spottercam image), or a "status=ok;
 * count=0"/"analysis_failed" shot. */
bool read_object_detection(const char* image_path, ObjectDetection* out) {
  unsigned char payload[EXIF_MAX_PAYLOAD];
  size_t size = 0;
  exif_view view{};
  exif_value entry{};
  if (exif_read_jpeg(image_path, payload, sizeof(payload), &size) != 0 ||
      exif_parse(&view, payload, size) != 0 ||
      !exif_get(&view, EXIF_CAMERA, EXIF_USER_COMMENT, &entry) ||
      entry.format != EXIF_UNDEFINED || entry.size <= kAsciiCommentPrefixSize ||
      std::memcmp(entry.data, "ASCII\0\0\0", kAsciiCommentPrefixSize) != 0) return false;
  std::string text(
      reinterpret_cast<const char*>(entry.data) + kAsciiCommentPrefixSize,
      entry.size - kAsciiCommentPrefixSize);

  return parse_object_summary(text, out);
}

/** Resolve a sibling output directory and ensure it is writable.
 * @param image_path Source photo path.
 * @param name Sibling directory name.
 * @param[out] out Resolved directory path.
 * @return True when the directory exists and is usable. */
bool sibling_directory(const char* image_path, const char* name,
                       std::string* out) {
  std::string path(image_path);
  size_t slash = path.find_last_of('/');
  std::string base_dir =
      (slash == std::string::npos) ? "." : path.substr(0, slash);
  *out = base_dir + "/" + name;
  return usher_ensure_directory(out->c_str()) == 0;
}

/** Return the final path component for display and CSV output.
 * @param image_path Source photo path.
 * @return Basename without directory components. */
std::string basename_of(const char* image_path) {
  std::string path(image_path);
  size_t slash = path.find_last_of('/');
  return (slash == std::string::npos) ? path : path.substr(slash + 1);
}

/** Copy every byte of @p source_path into a newly created/truncated @p
 * dest_path.
 * @param source_path Source JPEG.
 * @param dest_path Destination copy.
 * @return True only for a complete copy; removes partial output on failure.
 * @details A standalone copy loop, deliberately not shared with
 * spotter_cam_pipe.c's own copy_file() -- that function is file-local there,
 * and linking spotter_cam_pipe.o into soda would couple this build to usher's
 * object graph for ~20 lines of code. */
bool copy_file_bytes(const char* source_path, const char* dest_path) {
  int source_fd = open(source_path, O_RDONLY);
  if (source_fd < 0) {
    return false;
  }
  int dest_fd = open(dest_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (dest_fd < 0) {
    close(source_fd);
    return false;
  }

  char buffer[kCopyChunkBytes];
  ssize_t bytes_read;
  bool ok = true;
  while ((bytes_read = read(source_fd, buffer, sizeof(buffer))) > 0) {
    ssize_t written_total = 0;
    while (written_total < bytes_read) {
      ssize_t written = write(dest_fd, buffer + written_total,
                              static_cast<size_t>(bytes_read - written_total));
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        ok = false;
        break;
      }
      written_total += written;
    }
    if (!ok) {
      break;
    }
  }
  if (bytes_read < 0) {
    ok = false;
  }

  close(source_fd);
  if (close(dest_fd) != 0) {
    ok = false;
  }
  if (!ok) {
    unlink(dest_path);
  }
  return ok;
}

/** Add @p margin times the box size on each side, then clamp to the image
 * bounds, guarding a degenerate zero-area result so it's always safely
 * croppable.
 * @param box_x Detection left pixel.
 * @param box_y Detection top pixel.
 * @param box_w Detection width.
 * @param box_h Detection height.
 * @param img_w Source-image width.
 * @param img_h Source-image height.
 * @param margin Fraction of box size added on every side.
 * @return Clamped half-open crop with nonzero area. */
CropBox compute_crop_box(int box_x, int box_y, int box_w, int box_h,
                         unsigned img_w, unsigned img_h, double margin) {
  double pad_x = box_w * margin;
  double pad_y = box_h * margin;
  int left = std::max(0, static_cast<int>(std::lround(box_x - pad_x)));
  int top = std::max(0, static_cast<int>(std::lround(box_y - pad_y)));
  int right = std::min(static_cast<int>(img_w),
                       static_cast<int>(std::lround(box_x + box_w + pad_x)));
  int bottom = std::min(static_cast<int>(img_h),
                        static_cast<int>(std::lround(box_y + box_h + pad_y)));
  right = std::max(right, left + 1);
  bottom = std::max(bottom, top + 1);
  return CropBox{left, top, right, bottom};
}

/** Decode a JPEG to a contiguous row-major RGB buffer.
 * @param path Input image path.
 * @param[out] image Decoded pixels and image dimensions.
 * @return True only for a complete valid JPEG decode.
 * @details Same libjpeg pattern as image_mock_transform.c's decode_jpeg(),
 * duplicated rather than shared since that file's decode/encode helpers are
 * file-local (static) and out of scope to refactor for this change. */
bool decode_jpeg_rgb(const char* path, DecodedImage* image) {
  if (path == nullptr || image == nullptr) return false;
  FILE* input = fopen(path, "rb");
  if (input == nullptr) {
    return false;
  }

  struct jpeg_decompress_struct* decoder =
      static_cast<struct jpeg_decompress_struct*>(calloc(1, sizeof(*decoder)));
  if (decoder == nullptr) {
    fclose(input);
    return false;
  }

  JpegErrorContext error;
  decoder->err = jpeg_std_error(&error.manager);
  error.emit = error.manager.emit_message;
  error.manager.emit_message = jpeg_message;
  error.manager.error_exit = jpeg_error_exit;
  if (setjmp(error.recovery)) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }

  jpeg_create_decompress(decoder);
  jpeg_stdio_src(decoder, input);
  if (jpeg_read_header(decoder, TRUE) != JPEG_HEADER_OK ||
      !decoder->image_width || !decoder->image_height ||
      decoder->image_width > 8192 || decoder->image_height > 8192 ||
      static_cast<uint64_t>(decoder->image_width) * decoder->image_height > 32U * 1024U * 1024U) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }
  decoder->out_color_space = JCS_RGB;
  jpeg_start_decompress(decoder);

  image->width = decoder->output_width;
  image->height = decoder->output_height;
  if (decoder->output_components != 3 || image->width < 1 ||
      image->height < 1) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }

  size_t row_size = static_cast<size_t>(image->width) * 3;
  try {
    image->pixels.resize(row_size * image->height);
  } catch (const std::bad_alloc&) {
    jpeg_destroy_decompress(decoder);
    free(decoder);
    fclose(input);
    return false;
  }

  while (decoder->output_scanline < decoder->output_height) {
    JSAMPROW row[1] = {
        &image->pixels[static_cast<size_t>(decoder->output_scanline) *
                       row_size]};
    if (jpeg_read_scanlines(decoder, row, 1) != 1) {
      jpeg_destroy_decompress(decoder);
      free(decoder);
      fclose(input);
      return false;
    }
  }

  jpeg_finish_decompress(decoder);
  jpeg_destroy_decompress(decoder);
  free(decoder);
  return fclose(input) == 0;
}

/** Draw a green rectangle outline in place on a decoded RGB buffer.
 * @param image Mutable decoded image.
 * @param box_x Box left coordinate.
 * @param box_y Box top coordinate.
 * @param box_w Box width.
 * @param box_h Box height.
 * @param thickness Outline width in pixels.
 * @details The native capture and metadata stages never draw on the frame saved
 * to the configured photo directory; that file stays a clean flight-record
 * image. SODA is the only place that burns a box in, onto its own copies
 * (objects_captured/ and the crop), using the box this process already read
 * back from EXIF. This draws only the outline, not the "label 0.NN" confidence
 * text -- replicating PIL's text rendering in raw C++ would need a
 * font-rendering dependency for a label that's already in the console log and
 * the CSV row. */
void draw_box_outline(DecodedImage* image, int box_x, int box_y, int box_w,
                      int box_h, int thickness) {
  int left = box_x;
  int top = box_y;
  int right = box_x + box_w;
  int bottom = box_y + box_h;
  int img_w = static_cast<int>(image->width);
  int img_h = static_cast<int>(image->height);

  auto set_pixel = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= img_w || y >= img_h) {
      return;
    }
    size_t offset =
        (static_cast<size_t>(y) * image->width + static_cast<size_t>(x)) * 3;
    image->pixels[offset] = 0;
    image->pixels[offset + 1] = 255;
    image->pixels[offset + 2] = 0;
  };

  for (int stroke = 0; stroke < thickness; ++stroke) {
    for (int x = left; x < right; ++x) {
      set_pixel(x, top + stroke);
      set_pixel(x, bottom - 1 - stroke);
    }
    for (int y = top; y < bottom; ++y) {
      set_pixel(left + stroke, y);
      set_pixel(right - 1 - stroke, y);
    }
  }
}

/** Encode the selected region of an already-decoded frame to a new JPEG file.
 * @param path Destination JPEG path.
 * @param image Full-frame RGB source.
 * @param box Half-open source rectangle; rows are referenced directly without a
 * crop copy.
 * @return True when all rows encode and the output closes successfully.
 * @details No separate crop buffer is allocated: since the decoded buffer is
 * row-major RGB at the *full* image's stride, each output scanline's row
 * pointer can point directly into it at the crop's left/top offset -- the
 * encoder only reads `crop_w*3` bytes from wherever that pointer starts.
 * Quality 95, matching image_mock_transform.c's encode_jpeg(). */
bool encode_cropped_jpeg(const char* path, const DecodedImage& image,
                         const CropBox& box) {
  int crop_w = box.right - box.left;
  int crop_h = box.bottom - box.top;
  if (crop_w <= 0 || crop_h <= 0) {
    return false;
  }

  FILE* output = fopen(path, "wb");
  if (output == nullptr) {
    return false;
  }

  struct jpeg_compress_struct* encoder =
      static_cast<struct jpeg_compress_struct*>(calloc(1, sizeof(*encoder)));
  if (encoder == nullptr) {
    fclose(output);
    return false;
  }

  JpegErrorContext error;
  encoder->err = jpeg_std_error(&error.manager);
  error.manager.error_exit = jpeg_error_exit;
  if (setjmp(error.recovery)) {
    jpeg_destroy_compress(encoder);
    free(encoder);
    fclose(output);
    return false;
  }

  jpeg_create_compress(encoder);
  jpeg_stdio_dest(encoder, output);
  encoder->image_width = static_cast<JDIMENSION>(crop_w);
  encoder->image_height = static_cast<JDIMENSION>(crop_h);
  encoder->input_components = 3;
  encoder->in_color_space = JCS_RGB;
  jpeg_set_defaults(encoder);
  jpeg_set_quality(encoder, kCropJpegQuality, TRUE);
  jpeg_start_compress(encoder, TRUE);

  size_t full_row_size = static_cast<size_t>(image.width) * 3;
  while (encoder->next_scanline < encoder->image_height) {
    size_t source_row = static_cast<size_t>(box.top) + encoder->next_scanline;
    const unsigned char* row_start =
        &image.pixels[source_row * full_row_size +
                      static_cast<size_t>(box.left) * 3];
    JSAMPROW row[1] = {const_cast<JSAMPROW>(row_start)};
    if (jpeg_write_scanlines(encoder, row, 1) != 1) {
      jpeg_destroy_compress(encoder);
      free(encoder);
      fclose(output);
      return false;
    }
  }

  jpeg_finish_compress(encoder);
  jpeg_destroy_compress(encoder);
  free(encoder);
  return fclose(output) == 0;
}

/** Write one complete buffer in a single syscall, retrying only an EINTR that
 * transferred nothing. Callers rely on the single-syscall property for O_APPEND
 * atomicity -- see append_detection_csv().
 * @param fd Open output descriptor.
 * @param data Complete row or header-plus-row payload.
 * @return True only if the syscall wrote the entire buffer. */
bool write_all_atomic(int fd, const std::string& data) {
  ssize_t written;
  do {
    written = write(fd, data.data(), data.size());
  } while (written < 0 && errno == EINTR);
  return written == static_cast<ssize_t>(data.size());
}

/** Escape one bounded model label as a CSV field. */
std::string csv_field(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
  std::string escaped = "\"";
  for (char character : value) {
    if (character == '\"') escaped += '\"';
    escaped += character;
  }
  escaped += '\"';
  return escaped;
}

/** Append one CSV row for @p detection to @p csv_path, safe against concurrent
 * SODA processes racing on the same file (usher can run multiple worker
 * threads' finish_image() close together, each spawning its own SODA child).
 * @details Header-or-not is decided with O_CREAT|O_EXCL: whichever process wins
 * that race writes header+row together in a single write(); every loser falls
 * back to a plain O_APPEND open and writes only its own row. Each row (and the
 * header+row for the winner) goes out as exactly one write() call, which POSIX
 * guarantees is atomic against other O_APPEND writers on the same file as long
 * as it stays under PIPE_BUF -- comfortably true for these rows.
 * @param csv_path Shared CSV path.
 * @param image_basename Source JPEG basename.
 * @param detection EXIF-derived hit summary.
 * @return True when exactly one complete row/header+row is written. */
bool append_detection_csv(const std::string& csv_path,
                          const std::string& image_basename,
                          const ObjectDetection& detection) {
  std::string label = csv_field(detection.label);
  char row_buffer[kCsvRowCapacity];
  int row_length = std::snprintf(
      row_buffer, sizeof(row_buffer), "%s,%s,%d,%.4f,%d,%d,%d,%d\n",
      image_basename.c_str(), label.c_str(), detection.count,
      detection.confidence, detection.box_x, detection.box_y, detection.box_w,
      detection.box_h);
  if (row_length <= 0 ||
      static_cast<size_t>(row_length) >= sizeof(row_buffer)) {
    return false;
  }
  std::string row(row_buffer, static_cast<size_t>(row_length));

  int fd = open(csv_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (fd >= 0) {
    static const char kHeader[] =
        "image,label,count,confidence,box_x,box_y,box_w,box_h\n";
    bool ok = write_all_atomic(fd, std::string(kHeader) + row);
    return close(fd) == 0 && ok;
  }
  if (errno != EEXIST) {
    return false;
  }

  fd = open(csv_path.c_str(), O_WRONLY | O_APPEND);
  if (fd < 0) {
    return false;
  }
  bool ok = write_all_atomic(fd, row);
  return close(fd) == 0 && ok;
}

}  // namespace soda_object_postprocess_detail

using namespace soda_object_postprocess_detail;

/** Create SODA review derivatives only when the source EXIF describes an object
 * hit.
 * @return True for no-op/no-hit or complete processing; false when a detected
 * hit could not be fully copied, cropped, or recorded. */
bool soda_process_object_detection(const char* image_path) {
  if (image_path == nullptr || image_path[0] == '\0') {
    return true;
  }

  ObjectDetection detection;
  if (!read_object_detection(image_path, &detection)) {
    return true;
  }

  std::string basename = basename_of(image_path);
  bool ok = true;

  // Decode once and draw the box once; both the objects_captured/ copy and the
  // crop are regions of this same annotated buffer, so the box appears
  // consistently in both without decoding or drawing twice. The original file
  // at image_path is never touched -- it stays the plain capture saved by the
  // native camera backend.
  DecodedImage decoded;
  bool decoded_ok = decode_jpeg_rgb(image_path, &decoded);
  if (decoded_ok) {
    draw_box_outline(&decoded, detection.box_x, detection.box_y,
                     detection.box_w, detection.box_h, kBoxOutlineThickness);
  } else {
    std::cerr << "SODA: failed to decode " << image_path << " for annotation"
              << std::endl;
  }

  std::string hits_dir;
  if (sibling_directory(image_path, "objects_captured", &hits_dir)) {
    std::string dest_path = hits_dir + "/" + basename;
    bool saved;
    if (decoded_ok) {
      CropBox full_frame{0, 0, static_cast<int>(decoded.width),
                         static_cast<int>(decoded.height)};
      saved = encode_cropped_jpeg(dest_path.c_str(), decoded, full_frame);
    } else {
      // Annotation failed -- still land an (unannotated) copy so the hit isn't
      // lost entirely.
      saved = copy_file_bytes(image_path, dest_path.c_str());
    }
    if (!saved) {
      std::cerr << "SODA: failed to save annotated copy of " << image_path
                << " into " << hits_dir << std::endl;
      ok = false;
    }
    std::string csv_path = hits_dir + "/detections.csv";
    if (!append_detection_csv(csv_path, basename, detection)) {
      std::cerr << "SODA: failed to record detection in " << csv_path
                << std::endl;
      ok = false;
    }
  } else {
    std::cerr << "SODA: failed to prepare objects_captured directory for "
              << image_path << std::endl;
    ok = false;
  }

  std::string crop_dir;
  if (sibling_directory(image_path, "tight_crop_objects", &crop_dir)) {
    if (decoded_ok) {
      CropBox box = compute_crop_box(
          detection.box_x, detection.box_y, detection.box_w, detection.box_h,
          decoded.width, decoded.height, kCropMargin);
      std::string stem = basename;
      size_t dot = stem.find_last_of('.');
      if (dot != std::string::npos) {
        stem.erase(dot);
      }
      std::string crop_path = crop_dir + "/" + stem + "_object01.jpg";
      if (!encode_cropped_jpeg(crop_path.c_str(), decoded, box)) {
        std::cerr << "SODA: failed to save crop for " << image_path
                  << std::endl;
        ok = false;
      }
    } else {
      std::cerr << "SODA: failed to crop " << image_path
                << " (decode already failed)" << std::endl;
      ok = false;
    }
  } else {
    std::cerr << "SODA: failed to prepare tight_crop_objects directory for "
              << image_path << std::endl;
    ok = false;
  }

  std::cerr << "SODA: " << detection.label
            << " detected (count=" << detection.count
            << ", conf=" << detection.confidence << "); post-processed "
            << image_path << std::endl;

  return ok;
}
