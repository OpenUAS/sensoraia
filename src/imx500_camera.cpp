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

/** @file imx500_camera.cpp
 * @brief Shared IMX500 capture, frame ownership and JPEG publication mechanics.
 * @details Camera ownership and model interpretation deliberately have separate
 * lifetimes. Retaining one request keeps its pixels, crop, tensors and timestamps
 * together without another image queue. Application callbacks decide what those
 * tensors mean; this implementation only captures and publishes their evidence.
 * The owner serializes camera operations and lease destruction; libcamera's
 * completion callback uses a mutex only for the handoff, not for image processing. */
#include "imx500_camera.hpp"
#include "usher_frame_timing.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits.h>
#include <mutex>
#include <setjmp.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <jpeglib.h>
#include <libcamera/control_ids.h>
#include <libcamera/libcamera.h>
#include <libcamera/property_ids.h>

namespace imx500 {
namespace {

constexpr unsigned camera_buffer_count = 3; /**< Requested reusable frame count; avoids an extra image queue. */
constexpr unsigned maximum_camera_buffer_count = 4; /**< Reject driver adjustments that exceed the buffer budget. */
constexpr unsigned jpeg_luma_lines_per_mcu = 16; /**< YUV420 JPEG consumes one sixteen-line luma group. */
constexpr unsigned jpeg_chroma_lines_per_mcu = 8; /**< Half-resolution chroma accompanies that luma group. */
constexpr size_t maximum_mapped_plane_bytes = 32 * 1024 * 1024; /**< Bound unexpectedly large DMA plane mappings. */
constexpr int64_t camera_frame_duration_us = 125000; /**< Existing 8 FPS cadence, not an adaptive model policy. */
constexpr int camera_request_timeout_seconds = 5; /**< Bound a missing fresh-frame callback. */
constexpr int tensor_startup_timeout_seconds = 70; /**< Allow first-use firmware upload and inference warm-up. */
constexpr unsigned capture_width_alignment = 16; /**< Luma stride/capture alignment expected by the worker. */
constexpr unsigned capture_height_alignment = 2; /**< YUV420 needs paired luma rows for chroma sampling. */
constexpr uint32_t imx500_network_firmware_control = 0x00982901; /**< Driver control accepting an open RPK descriptor. */
constexpr uint32_t imx500_inference_roi_control = 0x00982900; /**< Driver control selecting sensor-space inference ROI. */

/** @brief Local C encoder recovery state; no camera lease is unwound by longjmp. */
struct JpegError {
  jpeg_error_mgr manager; /**< libjpeg callbacks initialized before compressor creation. */
  jmp_buf jump; /**< Recovery stays inside encode_frame_jpeg(), where partial output is removed. */
};

/** @brief Redirect libjpeg's fatal callback to the encoder's local cleanup path.
 * @param codec Active codec carrying our error manager.
 * @details libjpeg uses a C callback, not C++ exceptions. Keep the jump target
 * inside the encoder so it never skips a model processor or frame-lease destructor. */
void jpeg_failure(j_common_ptr codec)
{
  longjmp(reinterpret_cast<JpegError *>(codec->err)->jump, 1);
}

/** @brief Find a correctly typed array without assuming a vendor control ID number.
 * @param metadata Metadata belonging to the retained camera request.
 * @param name Driver-advertised control name.
 * @param type Required element type before interpreting its bytes.
 * @return Borrowed matching value, or NULL for absent/mismatched metadata.
 * @details Rejecting scalar and differently typed controls prevents a superficially
 * present field from being reinterpreted as a neural-output array. */
const libcamera::ControlValue *named_metadata(const libcamera::ControlList &metadata,
                                             const char *name, libcamera::ControlType type)
{
  if (!metadata.idMap()) {
    return nullptr;
  }
  for (const auto &entry : metadata) {
    const auto identifier = metadata.idMap()->find(entry.first);
    if (identifier != metadata.idMap()->end() && identifier->second->name() == name
        && entry.second.type() == type && entry.second.isArray()) {
      return &entry.second;
    }
  }
  return nullptr;
}

/** @brief Balance CPU/DMA ownership for the completed frame's borrowed planes.
 * @details Several planes may share one DMA-BUF descriptor. Map plane ranges
 * separately, but start/end CPU synchronization only once per shared descriptor. */
class MappedFrame {
public:
  /** @brief Map a successful completed buffer and validate its accessible plane ranges.
   * @param buffer Borrowed frame buffer retained by the surrounding request lease.
   * @throws std::runtime_error for invalid layout, page size, synchronization or mapping.
   * @details Constructor failure releases only mappings/syncs already acquired;
   * the outer lease remains responsible for returning the camera request. */
  explicit MappedFrame(libcamera::FrameBuffer *buffer)
  {
    if (buffer->planes().empty() || buffer->planes().size() > mappings_.size()
        || buffer->metadata().planes().size() != buffer->planes().size()
        || buffer->metadata().status != libcamera::FrameMetadata::FrameSuccess) {
      std::fprintf(stderr, "invalid camera buffer: planes=%zu metadata_planes=%zu status=%d\n",
                   buffer->planes().size(), buffer->metadata().planes().size(),
                   static_cast<int>(buffer->metadata().status));
      throw std::runtime_error("invalid camera buffer");
    }
    try {
      const long page = sysconf(_SC_PAGESIZE);
      if (page <= 0) {
        throw std::runtime_error("invalid page size");
      }
      for (size_t index = 0; index < buffer->planes().size(); ++index) {
        const auto &plane = buffer->planes()[index];
        auto &mapping = mappings_[index];
        mapping.fd = plane.fd.get();
        bool duplicate = false;
        for (size_t previous = 0; previous < index; ++previous) {
          duplicate = duplicate || mappings_[previous].fd == mapping.fd;
        }
        /* Plane count is not descriptor count: synchronizing a shared DMA-BUF
         * once keeps START/END balanced even when Y, Cb and Cr share storage. */
        if (!duplicate) {
          dma_buf_sync sync{DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
          if (ioctl(mapping.fd, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
            throw std::runtime_error("DMA read sync failed");
          }
          mapping.synced = true;
        }
        const size_t offset = plane.offset % size_t(page);
        /* mmap starts at a page boundary; expose only the plane's adjusted start.
         * Allocation capacity is not evidence that all those bytes were produced. */
        mapping.length = size_t(plane.length) + offset;
        if (!plane.length || mapping.length > maximum_mapped_plane_bytes) {
          throw std::runtime_error("camera plane too large");
        }
        mapping.base = mmap(nullptr, mapping.length, PROT_READ, MAP_SHARED,
                            mapping.fd, plane.offset - offset);
        if (mapping.base == MAP_FAILED) {
          throw std::runtime_error("camera buffer mmap failed");
        }
        data[index] = static_cast<const uint8_t *>(mapping.base) + offset;
        lengths[index] = std::min(plane.length, buffer->metadata().planes()[index].bytesused);
      }
    } catch (...) {
      release();
      throw;
    }
  }

  /** @brief End scoped CPU access before the camera request can be reused. */
  ~MappedFrame() { release(); }
  /** @brief Copies cannot share responsibility for DMA synchronization. */
  MappedFrame(const MappedFrame &) = delete;
  /** @brief Mapping ownership is never reassigned implicitly. */
  MappedFrame &operator=(const MappedFrame &) = delete;

  std::array<const uint8_t *, 3> data{}; /**< Borrowed Y/Cb/Cr addresses, valid only while mapped. */
  std::array<size_t, 3> lengths{}; /**< Produced readable bytes, capped by each plane allocation. */

private:
  /** @brief Release mapped ranges and balance each successful CPU-access START.
   * @details This also serves the constructor's failure path. Cleanup cannot
   * throw; a failed end-sync is reported rather than masking an earlier exception. */
  void release() noexcept
  {
    for (auto &mapping : mappings_) {
      if (mapping.base != MAP_FAILED) {
        munmap(mapping.base, mapping.length);
      }
      if (mapping.synced) {
        dma_buf_sync sync{DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ};
        if (ioctl(mapping.fd, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
          std::perror("DMA read sync end");
        }
      }
    }
  }

  /** @brief Ownership facts needed to unwind a partially mapped frame. */
  struct Mapping {
    void *base = MAP_FAILED; /**< mmap address before the plane's page-offset adjustment. */
    size_t length = 0; /**< Mapped bytes, including the page-prefix adjustment. */
    int fd = -1; /**< Borrowed DMA-BUF descriptor; this class never closes it. */
    bool synced = false; /**< This record owns the matching END for a successful START. */
  };
  std::array<Mapping, 3> mappings_{}; /**< Fixed maximum plane count, independent of image size. */
};

/** @brief Reject output names that cannot fit bounded sidecars or the line protocol.
 * @param filename Accessible NUL-terminated JPEG destination, or NULL.
 * @return True for a bounded absolute .jpg path without CR/LF characters.
 * @details Leave room for temporary/sidecar suffixes before allocating any
 * path strings. Absolute names keep publication independent of child working directories. */
bool valid_path(const char *filename)
{
  if (!filename) {
    return false;
  }
  const size_t length = strnlen(filename, PATH_MAX);
  return length >= 5 && length <= PATH_MAX - 32 && filename[0] == '/'
         && std::strcmp(filename + length - 4, ".jpg") == 0
         && std::strpbrk(filename, "\r\n") == nullptr;
}

bool valid_yuv420_layout(unsigned width, unsigned height, unsigned stride)
{
  return width && width <= sensor_width && height && height <= sensor_height
         && width % 2 == 0 && height % 2 == 0 && stride % 2 == 0
         && stride >= ((width + capture_width_alignment - 1) / capture_width_alignment)
                      * capture_width_alignment;
}

} // namespace

/** @details The lease owns the right to recycle the request, not the DMA memory.
 * Keeping that distinction avoids both full-frame copies and premature reuse. */
FrameLease::FrameLease(FrameView view, void *owner, Release release) noexcept
    : view_(view), owner_(owner), release_(release)
{
}

/** @brief Return a held request exactly once, including during exception unwinding. */
FrameLease::~FrameLease()
{
  /* All processing exits use the same release operation, including exceptions.
   * A moved-from lease has no callback left and therefore cannot recycle twice. */
  if (release_) {
    release_(owner_);
  }
}

/** @details Moving transfers the recycling obligation; it does not extend the
 * lifetime of the camera state or make an owned copy of the borrowed pixels. */
FrameLease::FrameLease(FrameLease &&other) noexcept
    : view_(other.view_), owner_(other.owner_), release_(other.release_)
{
  /* Transfer the resource, not just its metadata; empty the old view as a useful
   * guard against accidentally reading bytes through a consumed lease. */
  other.owner_ = nullptr;
  other.release_ = nullptr;
  other.view_ = FrameView{};
}

/** @brief Encode one YUV420 frame with optional same-exposure EXIF metadata.
 * @param path New output path; exclusive creation never replaces an existing file.
 * @param planes Borrowed Y, Cb and Cr starts, readable throughout encoding.
 * @param width Positive even image width in pixels.
 * @param height Positive even image height in pixels.
 * @param stride Even luma byte stride; chroma uses half this stride.
 * @param view Capture evidence to embed, or NULL for a pixels-only JPEG.
 * @param processor Optional application callback for extending the EXIF document.
 * @return True after encoding, flushing, file sync and close; false on metadata
 * rejection or I/O/codec failure. Partial files created here are removed.
 * @pre The caller has validated dimensions, strides and accessible plane lengths.
 * @details Metadata is prepared before opening the output or entering libjpeg's
 * C error boundary, so a throwing embed callback leaves no partial JPEG behind. */
static bool encode_frame_jpeg(const char *path, const std::array<const uint8_t *, 3> &planes,
                              unsigned width, unsigned height, unsigned stride,
                              const FrameView *view, const FrameProcessor *processor)
{
  unsigned char capture_exif[EXIF_MAX_PAYLOAD];
  size_t capture_size = 0;
  if (view) {
    const struct usher_frame_timing timing = {
      view->sequence, view->sensor_timestamp_ns, view->monotonic_us, view->time_error_us,
      view->exposure_us, view->frame_duration_us
    };
    exif_document document;
    if (!usher_frame_document(&timing, width, height, &document)
      || (processor && processor->embed && !processor->embed(*view, &document))
      || exif_serialize(&document, capture_exif, sizeof(capture_exif), &capture_size)) return false;
  }
  FILE *file = std::fopen(path, "wbx");
  if (!file) {
    return false;
  }
  jpeg_compress_struct codec{};
  JpegError error{};
  codec.err = jpeg_std_error(&error.manager);
  error.manager.error_exit = jpeg_failure;
  /* Recover inside the encoder: libjpeg's longjmp must not bypass the outer
   * frame lease or DMA mapping destructors that keep the camera queue usable. */
  if (setjmp(error.jump)) {
    jpeg_destroy_compress(&codec);
    std::fclose(file);
    unlink(path);
    return false;
  }
  jpeg_create_compress(&codec);
  jpeg_stdio_dest(&codec, file);
  codec.image_width = width;
  codec.image_height = height;
  codec.input_components = 3;
  codec.in_color_space = JCS_YCbCr;
  jpeg_set_defaults(&codec);
  /* Keep YUV420 in its native planar representation. An RGB staging image would
   * add full-frame memory and conversion work without improving capture identity. */
  codec.raw_data_in = TRUE;
  jpeg_set_quality(&codec, 90, TRUE);
  if (processor && !processor->sidecar_suffix && !processor->write && !processor->embed)
    codec.dct_method = JDCT_FLOAT;
  jpeg_start_compress(&codec, TRUE);
  if (capture_size) jpeg_write_marker(&codec, JPEG_APP0 + 1, capture_exif, static_cast<unsigned>(capture_size));
  while (codec.next_scanline < height) {
    JSAMPROW rows[3][jpeg_luma_lines_per_mcu]{};
    JSAMPARRAY components[3]{rows[0], rows[1], rows[2]};
    for (unsigned row = 0; row < jpeg_luma_lines_per_mcu; ++row) {
      /* JPEG consumes whole MCU groups; repeat the final valid row at the edge
       * rather than reading padding outside the caller-validated plane span. */
      rows[0][row] = const_cast<JSAMPROW>(
          planes[0] + std::min(codec.next_scanline + row, height - 1) * stride);
      if (row < jpeg_chroma_lines_per_mcu) {
        for (size_t component = 1; component < 3; ++component) {
          rows[component][row] = const_cast<JSAMPROW>(
              planes[component]
              + std::min(codec.next_scanline / 2 + row, height / 2 - 1) * (stride / 2));
        }
      }
    }
    jpeg_write_raw_data(&codec, components, jpeg_luma_lines_per_mcu);
  }
  jpeg_finish_compress(&codec);
  jpeg_destroy_compress(&codec);
  /* Only a fully flushed and synced file may be published by the caller.
   * This sync covers file contents, not the parent directory's durability. */
  bool written = std::fflush(file) == 0 && fsync(fileno(file)) == 0;
  if (std::fclose(file) != 0) {
    written = false;
  }
  if (!written) {
    unlink(path);
  }
  return written;
}

/** @details Use the same encoder without capture/application metadata so
 * pixels-only callers retain identical YUV layout and output-cleanup behavior. */
bool encode_jpeg(const char *path, const std::array<const uint8_t *, 3> &planes,
                  unsigned width, unsigned height, unsigned stride)
{
  if (!path || !path[0] || !planes[0] || !planes[1] || !planes[2]
      || !valid_yuv420_layout(width, height, stride)) return false;
  return encode_frame_jpeg(path, planes, width, height, stride, nullptr, nullptr);
}

/** @brief Capture resources and the callback-to-owner handoff, not model results.
 * @details The mutex protects callback-visible readiness state. Control calls and
 * lease release remain serialized by the owner, as required by the public API. */
struct Camera::Impl {
  libcamera::CameraManager manager; /**< Owns camera discovery/event processing. */
  std::shared_ptr<libcamera::Camera> camera; /**< The sole selected sensor, acquired exclusively. */
  std::unique_ptr<libcamera::CameraConfiguration> configuration; /**< Validated stream geometry/layout. */
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator; /**< Owns CMA-backed buffers, not CPU copies. */
  std::vector<std::unique_ptr<libcamera::Request>> requests; /**< One reusable request per allocated buffer. */
  std::array<int64_t, 2> frame_duration{camera_frame_duration_us, camera_frame_duration_us}; /**< Fixed cadence range. */
  std::mutex mutex; /**< Protects callback readiness, failure and waiting-request state. */
  std::condition_variable ready; /**< Wakes a waiter for a fresh frame or queue failure. */
  libcamera::Request *completed = nullptr; /**< Callback-held request awaiting owner handoff. */
  libcamera::Request *leased = nullptr; /**< Request held until its sole lease releases it. */
  int64_t minimum_timestamp = 0; /**< Earliest eligible sensor timestamp for the current request. */
  bool wanted = false; /**< A capture call is awaiting one post-request exposure. */
  bool failed = false; /**< Queue failure prevents treating capture as healthy. */
  bool acquired = false; /**< Cleanup owes camera release only after acquisition succeeds. */
  bool started = false; /**< Cleanup owes stream stop only after startup succeeds. */

  /** @brief Unwind partial/full startup in libcamera's resource-lifetime order. */
  ~Impl()
  {
    /* Stop callbacks before destroying their requests and DMA buffers. The
     * success flags also make failed-open cleanup safe at each setup stage. */
    if (started) {
      camera->stop();
    }
    requests.clear();
    allocator.reset();
    configuration.reset();
    if (acquired) {
      camera->release();
    }
    camera.reset();
    manager.stop();
  }

  /** @brief Return an unneeded or fully consumed request to the bounded camera queue.
   * @param request Completed request with no remaining borrowed consumers.
   * @details A requeue failure wakes the owner as a failure; it must not wait for
   * a callback that can no longer arrive or silently reuse stale evidence. */
  void recycle(libcamera::Request *request)
  {
    request->reuse(libcamera::Request::ReuseBuffers);
    if (camera->queueRequest(request) != 0) {
      std::lock_guard<std::mutex> lock(mutex);
      failed = true;
      ready.notify_one();
    }
  }

  /** @brief Recycle the sole leased request without throwing through its destructor.
   * @param owner Capture implementation that outlives the lease.
   * @details Clear ownership before requeueing. Unexpected exceptions cannot be
   * hidden as a successful release, so this nonthrowing boundary terminates. */
  static void release_lease(void *owner) noexcept
  {
    auto &state = *static_cast<Impl *>(owner);
    libcamera::Request *request = state.leased;
    state.leased = nullptr;
    try {
      if (request) {
        state.recycle(request);
      }
    } catch (...) {
      std::terminate();
    }
  }

  /** @brief Hold one fresh requested exposure; immediately recycle other callbacks.
   * @param request Completed or cancelled libcamera request.
   * @details The timestamp gate is set when capture() starts. A previously queued
   * frame is not automatically the photograph requested by the current SHOT. */
  void complete(libcamera::Request *request)
  {
    /* Cancellation is part of shutdown, not a fresh frame to hand off or requeue. */
    if (request->status() == libcamera::Request::RequestCancelled) {
      return;
    }
    const auto &buffers = request->buffers();
    const auto buffer = buffers.find(configuration->at(0).stream());
    const auto status = buffer != buffers.end() && buffer->second
                        ? detail::frame_status(buffer->second->metadata().status)
                        : detail::FrameStatus::failed;
    if (status == detail::FrameStatus::startup) {
      recycle(request);
      return;
    }
    if (status == detail::FrameStatus::failed) {
      std::lock_guard<std::mutex> lock(mutex);
      failed = true;
      wanted = false;
      ready.notify_one();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      const auto timestamp = request->metadata().get(libcamera::controls::SensorTimestamp);
      if (wanted && timestamp && *timestamp >= minimum_timestamp) {
        completed = request;
        wanted = false;
        ready.notify_one();
        return;
      }
    }
    recycle(request);
  }

  /** @brief Load an RPK or clear a retained network through the driver control.
   * @param model Accessible readable model path, or NULL to select image-only capture.
   * @throws std::runtime_error for ambiguous/missing devices or failed controls.
  * @details The driver's INT32_MAX control value clears a retained volatile network.
  * Full-array inference ROI is set only when loading a model. */
  static void configure_model(const char *model)
  {
    std::string device;
    for (const auto &entry : std::filesystem::directory_iterator("/sys/class/video4linux")) {
      if (entry.path().filename().string().find("v4l-subdev") != 0) {
        continue;
      }
      std::error_code error;
      const auto driver = std::filesystem::canonical(entry.path() / "device/driver", error);
      if (!error && driver.filename() == "imx500") {
        if (!device.empty()) {
          throw std::runtime_error("ambiguous IMX500 subdevice");
        }
        device = "/dev/" + entry.path().filename().string();
      }
    }
    const int sensor = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (sensor < 0) {
      throw std::runtime_error("cannot open IMX500 subdevice");
    }
    v4l2_control firmware{};
    firmware.id = imx500_network_firmware_control;
    if (!model) {
      firmware.value = INT32_MAX;
      const int status = ioctl(sensor, VIDIOC_S_CTRL, &firmware);
      close(sensor);
      if (status != 0) throw std::runtime_error("IMX500 network unload failed");
      return;
    }
    const int network = ::open(model, O_RDONLY | O_CLOEXEC);
    if (network < 0) {
      close(sensor);
      throw std::runtime_error("cannot open network.rpk");
    }
    /* Pass an open descriptor, not model bytes copied into another userspace
     * buffer; keep it open until the synchronous driver control completes. */
    firmware.value = network;
    int status = ioctl(sensor, VIDIOC_S_CTRL, &firmware);
    std::array<uint32_t, 4> region{0, 0, sensor_width, sensor_height};
    v4l2_ext_control roi{};
    roi.id = imx500_inference_roi_control;
    roi.size = sizeof(region);
    roi.p_u32 = region.data();
    v4l2_ext_controls controls{};
    controls.count = 1;
    controls.controls = &roi;
    if (status == 0) {
      status = ioctl(sensor, VIDIOC_S_EXT_CTRLS, &controls);
    }
    close(network);
    close(sensor);
    if (status != 0) {
      throw std::runtime_error("IMX500 firmware/ROI control failed");
    }
  }
};

/** @brief Allocate unopened camera state; discovery and acquisition wait for open(). */
Camera::Camera() : impl_(std::make_unique<Impl>()) {}
/** @brief Stop callbacks, free requests/buffers, then release the camera.
 * @pre All leases and pixel visitors have finished; they borrow this state. */
Camera::~Camera() = default;

/** @details Exactly one eligible camera is required so discovery order cannot
 * silently choose a different sensor. Partial startup remains owned by Impl and
 * is unwound when the Camera is destroyed. */
void Camera::open(const char *model, unsigned width, unsigned height, bool photo_only)
{
  auto &state = *impl_;
  if (state.acquired || state.started) {
    throw std::runtime_error("camera already open");
  }
  if (photo_only && model) throw std::invalid_argument("photo-only capture cannot load a model");
  if (state.manager.start() != 0) {
    throw std::runtime_error("libcamera startup failed");
  }
  for (const auto &candidate : state.manager.cameras()) {
    const auto name = candidate->properties().get(libcamera::properties::Model);
    if ((!model && !photo_only) || (name && name->find("imx500") != std::string::npos)) {
      if (state.camera) {
        throw std::runtime_error("capture requires exactly one eligible camera");
      }
      state.camera = candidate;
    }
  }
  if (!state.camera || state.camera->acquire() != 0) {
    throw std::runtime_error("IMX500 unavailable or busy");
  }
  state.acquired = true;
  /* The SDK profile selects this switch from its span API. In the supported
   * newer SDK, PixelArrayArea also replaces PixelArraySize; compare dimensions
   * in both cases because the firmware ROI below assumes the full IMX500 array. */
#ifdef SENSORAIA_LIBCAMERA_STD_SPAN
  const auto sensor = state.camera->properties().get(libcamera::properties::PixelArrayArea);
  const bool expected_sensor = sensor && sensor->size() == libcamera::Size(sensor_width, sensor_height);
#else
  const auto sensor = state.camera->properties().get(libcamera::properties::PixelArraySize);
  const bool expected_sensor = sensor && *sensor == libcamera::Size(sensor_width, sensor_height);
#endif
  if ((model || photo_only) && !expected_sensor) {
    throw std::runtime_error("unexpected IMX500 geometry");
  }
  if (model || photo_only) Impl::configure_model(model);
  state.configuration = state.camera->generateConfiguration({libcamera::StreamRole::StillCapture});
  if (!state.configuration || state.configuration->empty()) {
    throw std::runtime_error("no still stream");
  }
  auto &stream = state.configuration->at(0);
  stream.size = {width, height};
  stream.pixelFormat = libcamera::formats::YUV420;
  stream.colorSpace = libcamera::ColorSpace::Sycc;
  state.configuration->orientation = libcamera::Orientation::Rotate0;
  stream.bufferCount = camera_buffer_count;
  /* libcamera can adjust a configuration. Reject changes that invalidate plane
   * arithmetic, downstream geometry or our intended CMA-buffer budget. */
  if (state.configuration->validate() == libcamera::CameraConfiguration::Invalid
      || state.configuration->orientation != libcamera::Orientation::Rotate0
      || stream.size != libcamera::Size(width, height)
      || stream.pixelFormat != libcamera::formats::YUV420
      || stream.colorSpace != libcamera::ColorSpace::Sycc || stream.stride % 2
      || stream.stride < ((width + capture_width_alignment - 1) / capture_width_alignment)
                            * capture_width_alignment
      || stream.bufferCount == 0
      || stream.bufferCount > maximum_camera_buffer_count
      || state.camera->configure(state.configuration.get()) != 0) {
    throw std::runtime_error("unsupported still configuration");
  }
  state.allocator = std::make_unique<libcamera::FrameBufferAllocator>(state.camera);
  if (state.allocator->allocate(stream.stream()) < 0) {
    throw std::runtime_error("camera buffer allocation failed");
  }
  for (const auto &buffer : state.allocator->buffers(stream.stream())) {
    /* Reuse requests/buffers rather than allocating another full-resolution
     * frame per trigger. The lease, not a copied image, preserves identity. */
    auto request = state.camera->createRequest();
    if (!request || request->addBuffer(stream.stream(), buffer.get()) != 0) {
      throw std::runtime_error("camera request allocation failed");
    }
    state.requests.push_back(std::move(request));
  }
  if (state.requests.empty()) throw std::runtime_error("camera allocated no capture buffers");
  state.camera->requestCompleted.connect(&state, &Impl::complete);
  libcamera::ControlList controls(state.camera->controls());
  if (!photo_only) {
    controls.set(libcamera::controls::FrameDurationLimits,
                 Span<const int64_t, 2>(state.frame_duration));
  }
  /* Install the callback before starting, then queue every allocated request.
   * Requests that are not wanted by capture() keep circulating without copies. */
  if (state.camera->start(&controls) != 0) {
    throw std::runtime_error("camera start failed");
  }
  state.started = true;
  for (auto &request : state.requests) {
    state.recycle(request.get());
  }
}

/** @details A queued frame can predate a SHOT command. Compare sensor timestamps
 * in the BOOTTIME domain instead of treating callback arrival as exposure time.
 * Tensor validity and interpretation remain the application's responsibility. */
FrameLease Camera::capture()
{
  auto &state = *impl_;
  std::unique_lock<std::mutex> lock(state.mutex);
  if (!state.started || state.leased) {
    throw std::runtime_error("camera not started or previous frame still leased");
  }
  timespec now{};
  if (clock_gettime(CLOCK_BOOTTIME, &now) != 0) {
    throw std::runtime_error("capture clock failed");
  }
  state.minimum_timestamp = int64_t(now.tv_sec) * 1000000000 + now.tv_nsec;
  /* Only the callback can satisfy this wait. It may retain a fresh request,
   * while older completed frames return immediately to the capture queue. */
  state.wanted = true;
  if (!state.ready.wait_for(lock, std::chrono::seconds(camera_request_timeout_seconds),
                            [&state] { return state.completed || state.failed; })) {
    state.wanted = false;
    throw std::runtime_error("camera frame timeout");
  }
  if (state.failed) {
    throw std::runtime_error("camera request or frame failed");
  }
  /* Move the request from callback handoff to lease ownership under the lock.
   * Release the lock before metadata work so other completions can circulate. */
  state.leased = state.completed;
  state.completed = nullptr;
  lock.unlock();
  /* Establish cleanup before inspecting optional metadata: an exception during
   * extraction must still return the request exactly once. */
  FrameLease lease(FrameView{}, impl_.get(), Impl::release_lease);
  const auto *request = state.leased;
  FrameView &view = lease.view_;
  /* Every borrowed tensor/crop/time comes from this same retained request.
   * Missing controls remain explicit sentinels, not invented capture evidence. */
  const auto *tensor = named_metadata(request->metadata(), "CnnOutputTensor", libcamera::ControlTypeFloat);
  const auto *info = named_metadata(request->metadata(), "CnnOutputTensorInfo", libcamera::ControlTypeByte);
  if (tensor) {
    view.tensor = tensor->get<Span<const float>>();
  }
  if (info) {
    view.tensor_info = info->get<Span<const uint8_t>>();
  }
  const auto crop = request->metadata().get(libcamera::controls::ScalerCrop);
  if (crop) {
    view.crop = *crop;
    view.crop_valid = true;
  }
  view.image = state.configuration->at(0).size;
  view.sequence = request->sequence();
  const auto sensor = request->metadata().get(libcamera::controls::SensorTimestamp);
  view.sensor_timestamp_ns = sensor && *sensor > 0 ? static_cast<uint64_t>(*sensor) : 0;
  const auto exposure = request->metadata().get(libcamera::controls::ExposureTime);
  const auto duration = request->metadata().get(libcamera::controls::FrameDuration);
  view.exposure_us = exposure ? *exposure : -1;
  view.frame_duration_us = duration ? *duration : -1;
  /* Estimate observation timing once while this exposure is retained. JPEG
   * encoding/publication can happen later and must not become the capture time. */
  const struct usher_frame_timing timing = usher_frame_timing_estimate(
      view.sequence, view.sensor_timestamp_ns, view.exposure_us, view.frame_duration_us);
  view.monotonic_us = timing.monotonic_us;
  view.time_error_us = timing.time_error_us;
  view.timing_sampled = true;
  return lease;
}

/** @details Encode first, then attempt sidecar output, then link the complete JPEG
 * into place. The JPEG is the final publication step, but JPEG and sidecar are
 * not one atomic transaction; callbacks must remove their own partial outputs.
 * @throws std::exception Filesystem or processor exceptions propagate after cleanup. */
bool publish_frame(const FrameView &view, const std::array<const uint8_t *, 3> &planes,
                    unsigned stride, const char *filename, const FrameProcessor &processor)
{
  if (!valid_path(filename) || bool(processor.sidecar_suffix) != bool(processor.write)
      || !planes[0] || !planes[1] || !planes[2]
      || !valid_yuv420_layout(view.image.width, view.image.height, stride)) {
    return false;
  }
  const std::string path(filename);
  const std::string sidecar = processor.sidecar_suffix ? path.substr(0, path.size() - 4) + processor.sidecar_suffix : "";
  const std::string temporary = path + ".capture.tmp";
  /* Existence checks avoid ordinary collisions; exclusive creation/link are
   * still the authority if another writer races after these checks. */
  if (std::filesystem::exists(path) || (!sidecar.empty() && std::filesystem::exists(sidecar))) {
    return false;
  }
  FrameView evidence = view;
  /* Live capture already sampled timing. Other callers may supply only raw
   * sensor metadata; fill the EXIF estimate without mutating their shared view. */
  if (!evidence.timing_sampled) {
    const struct usher_frame_timing timing = usher_frame_timing_estimate(
      view.sequence, view.sensor_timestamp_ns, view.exposure_us, view.frame_duration_us);
    evidence.monotonic_us = timing.monotonic_us;
    evidence.time_error_us = timing.time_error_us;
  }
  if (!encode_frame_jpeg(temporary.c_str(), planes, view.image.width, view.image.height, stride, &evidence, &processor)) {
    return false;
  }
  bool analysis_written = false;
  try {
    /* A false analysis result means no usable sidecar, not unusable pixels.
     * Timing is tied to the temporary JPEG inode retained by the final hard link. */
    if (processor.write && !sidecar.empty()) analysis_written = processor.write(view, sidecar.c_str());
    /* A hard link exposes only the completed inode and refuses an existing
     * destination, unlike a replacing rename. Both names stay in one directory. */
    const bool published = link(temporary.c_str(), path.c_str()) == 0;
    unlink(temporary.c_str());
    if (!published) {
      /* Remove only successful artifacts created here. A failed callback owes
       * its own cleanup and must not give us ownership of somebody else's file. */
      if (analysis_written) {
        unlink(sidecar.c_str());
      }
    }
    return published;
  } catch (...) {
    /* The callback contract covers partial analysis output; our exception path
     * owns only the JPEG temporary and sidecars whose creation was reported. */
    unlink(temporary.c_str());
    if (analysis_written) {
      unlink(sidecar.c_str());
    }
    throw;
  }
}

/** @details with_pixels() scopes CPU/DMA access around publication; the caller
 * still owns the frame lease and decides when the request returns to the queue.
 * @throws std::exception Mapping, filesystem or processor failures may propagate. */
bool Camera::save(const FrameLease &frame, const char *filename, const FrameProcessor &processor)
{
  struct SavePolicy { const char *filename; const FrameProcessor *processor; } policy{filename, &processor};
  return with_pixels(frame, [](const FrameView &view, const std::array<const uint8_t *, 3> &planes,
                               unsigned stride, void *context) {
    const auto &save = *static_cast<SavePolicy *>(context);
    return publish_frame(view, planes, stride, save.filename, *save.processor);
  }, &policy);
}

/** @details Plane allocation sizes alone do not prove that a frame produced enough
 * bytes. Check bytesused before exposing chroma, and unmap/end CPU access before
 * returning or unwinding to the code that can recycle the request.
 * @throws std::exception DMA/mapping failures and visitor exceptions may propagate. */
bool Camera::with_pixels(const FrameLease &frame,
                          bool (*visitor)(const FrameView &, const std::array<const uint8_t *, 3> &,
                                          unsigned, void *), void *context)
{
  auto &state = *impl_;
  if (!visitor || frame.owner_ != impl_.get() || frame.release_ == nullptr || !state.leased) {
    return false;
  }
  auto *request = state.leased;
  const auto &stream = state.configuration->at(0);
  auto *buffer = request->buffers().at(stream.stream());
  MappedFrame mapped(buffer);
  /* Some drivers use one contiguous YUV allocation, others three planes.
   * Validate produced byte counts before deriving chroma starts or encoding. */
  auto planes = mapped.data;
  const size_t luma = size_t(stream.stride) * stream.size.height;
  const size_t chroma = luma / 4;
  if (buffer->planes().size() == 1 && mapped.lengths[0] >= luma + 2 * chroma) {
    planes[1] = planes[0] + luma;
    planes[2] = planes[1] + chroma;
  } else if (buffer->planes().size() != 3 || mapped.lengths[0] < luma
             || mapped.lengths[1] < chroma || mapped.lengths[2] < chroma) {
    return false;
  }
  return visitor(frame.view(), planes, stream.stride, context);
}

/** @retval 0 Probe/capture succeeded, or the serving input ended cleanly.
 * @retval 1 Runtime, output or protocol-stream I/O failure.
 * @retval 2 Invalid arguments, dimensions, processor contract or command syntax.
 * @details A per-SHOT save failure emits ERROR without retrying the exposure.
 * An exception terminates the worker after reporting the reason on stderr. */
int run_worker(int argc, char **argv, const FrameProcessor &processor, const char *application)
{
  if (argc < 3) {
    return 2;
  }
  const bool photo_only = std::strcmp(argv[2], "--photo-only") == 0;
  if (!photo_only && (!processor.validate || ((!processor.write || !processor.sidecar_suffix) && !processor.embed))) {
    return 2;
  }
  const FrameProcessor photo_processor{nullptr, nullptr, nullptr};
  const FrameProcessor &selected_processor = photo_only ? photo_processor : processor;
  const char *timing_setting = std::getenv("USHER_AI_CAM_TIMING");
  const bool report_timing = timing_setting && std::strcmp(timing_setting, "1") == 0;
  const bool probe = std::strcmp(argv[1], "--probe") == 0;
  const bool serve = std::strcmp(argv[1], "--serve") == 0;
  const bool capture = std::strcmp(argv[1], "--capture") == 0;
  if ((!probe && !serve && !capture) || (probe && argc != 3)
      || (serve && argc != 5) || (capture && argc != 4)) {
    return 2;
  }
  try {
    unsigned width = 2028;
    unsigned height = 1520;
    if (serve) {
      char *end_width = nullptr;
      char *end_height = nullptr;
      errno = 0;
      const unsigned long parsed_width = std::strtoul(argv[3], &end_width, 10);
      const unsigned long parsed_height = std::strtoul(argv[4], &end_height, 10);
      if (errno || end_width == argv[3] || end_height == argv[4] || *end_width || *end_height
          || parsed_width > sensor_width || parsed_height > sensor_height
          || !valid_capture_dimensions(static_cast<unsigned>(parsed_width), static_cast<unsigned>(parsed_height))) {
        return 2;
      }
      width = static_cast<unsigned>(parsed_width);
      height = static_cast<unsigned>(parsed_height);
    }
    Camera camera;
    camera.open(photo_only ? nullptr : argv[2], width, height, photo_only);
    /* RPK upload and neural warm-up can outlast normal frame acquisition.
     * Discard startup leases until this application's tensor contract passes;
     * an available camera frame alone is not enough to announce readiness. */
    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::seconds(tensor_startup_timeout_seconds);
    bool ready = false;
    while (std::chrono::steady_clock::now() < deadline) {
      auto frame = camera.capture();
      if (photo_only && !detail::photo_only_frame(frame.view()))
        throw std::runtime_error("unexpected IMX500 inference metadata in photo-only mode");
      ready = photo_only || selected_processor.validate(frame.view());
      if (ready) {
        break;
      }
    }
    if (!ready) {
      throw std::runtime_error("no fresh valid IMX500 tensor before startup deadline");
    }
    if (probe) {
      std::puts(photo_only ? "camera frame received" : "IMX500 frame and output tensor received");
      return 0;
    }
    if (capture) {
      /* The readiness frame is not the requested photograph: take another
       * post-call exposure instead of publishing a frame consumed by warm-up. */
      auto frame = camera.capture();
      if (photo_only && !detail::photo_only_frame(frame.view()))
        throw std::runtime_error("unexpected IMX500 inference metadata in photo-only mode");
      return camera.save(frame, argv[3], selected_processor) ? 0 : 1;
    }
    std::puts("READY");
    /* READY means this processor accepted a fresh tensor contract. It never
     * means that LOOM found a navigable match or that objects were detected. */
    if (std::fflush(stdout) != 0) {
      return 1;
    }
    char command[PATH_MAX + 16];
    while (std::fgets(command, sizeof(command), stdin)) {
      const size_t length = std::strlen(command);
      /* Missing newline also catches an overlong/truncated command. Reject it
       * rather than treating the remainder as another independent SHOT. */
      if (length == 0 || command[length - 1] != '\n' || std::strncmp(command, "SHOT ", 5) != 0) {
        return 2;
      }
      command[length - 1] = '\0';
      /* Each command takes a new scoped lease. ERROR is reported for that
       * operation; the protocol never automatically repeats its exposure. */
      std::chrono::steady_clock::time_point capture_started{}, capture_completed{};
      if (report_timing) capture_started = std::chrono::steady_clock::now();
      auto frame = camera.capture();
      if (photo_only && !detail::photo_only_frame(frame.view()))
        throw std::runtime_error("unexpected IMX500 inference metadata in photo-only mode");
      if (report_timing) capture_completed = std::chrono::steady_clock::now();
      const bool saved = camera.save(frame, command + 5, selected_processor);
      if (report_timing) {
        const auto save_completed = std::chrono::steady_clock::now();
        std::fprintf(stderr, "%s TIMING: capture_ms=%.3f save_ms=%.3f exposure_us=%lld frame_us=%lld\n",
                     application,
                     std::chrono::duration<double, std::milli>(capture_completed - capture_started).count(),
                     std::chrono::duration<double, std::milli>(save_completed - capture_completed).count(),
                     static_cast<long long>(frame.view().exposure_us),
                     static_cast<long long>(frame.view().frame_duration_us));
      }
      std::puts(saved ? "OK" : "ERROR");
      if (std::fflush(stdout) != 0) {
        return 1;
      }
    }
    return std::ferror(stdin) ? 1 : 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "%s: %s\n", application, error.what());
    return 1;
  }
}

int run_photo_worker(int argc, char **argv, const char *application)
{
  if (argc < 2 || argc > 4) return 2;
  char photo_only[] = "--photo-only";
  char *arguments[] = {argv[0], argv[1], photo_only,
                       argc > 2 ? argv[2] : nullptr,
                       argc > 3 ? argv[3] : nullptr, nullptr};
  const FrameProcessor processor{nullptr, nullptr, nullptr};
  return run_worker(argc + 1, arguments, processor, application);
}

} // namespace imx500