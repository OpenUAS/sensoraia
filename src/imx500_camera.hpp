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

/** @file imx500_camera.hpp
 * @brief Neutral IMX500 frame ownership, JPEG publication and worker protocol.
 * @details Model interpretation belongs to the caller. Tensor views are borrowed
 * from one retained camera request and may not outlive its FrameLease. */
#ifndef SENSORAIA_IMX500_CAMERA_HPP
#define SENSORAIA_IMX500_CAMERA_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/base/span.h>
#include "exifite.h"

namespace imx500 {

namespace detail {
enum class FrameStatus { ready, startup, failed };
/** @brief Map libcamera metadata to this worker's frame disposition.
 * @param status Driver-reported frame status.
 * @return Ready, startup, or failed.
 * @details Startup frames are not usable captures, but unlike failures
 * they are expected while the camera pipeline settles. */
constexpr FrameStatus frame_status(libcamera::FrameMetadata::Status status)
{
  return status == libcamera::FrameMetadata::FrameSuccess ? FrameStatus::ready
       : status == libcamera::FrameMetadata::FrameStartup ? FrameStatus::startup
       : FrameStatus::failed;
}
}

#ifdef SENSORAIA_LIBCAMERA_STD_SPAN
template <typename Element, std::size_t Extent = std::dynamic_extent>
using Span = std::span<Element, Extent>;
#else
using libcamera::Span;
#endif

/** @brief Physical IMX500 array width, used to reject an unexpected sensor geometry. */
constexpr unsigned sensor_width = 4056;
/** @brief Physical IMX500 array height; model input dimensions are separate policy. */
constexpr unsigned sensor_height = 3040;

/** @brief Check that capture dimensions meet the worker's raster policy.
 * @param width Requested columns.
 * @param height Requested rows.
 * @return True for an even raster within the physical array and minimum dimensions.
 * @details Check here before driver setup; this does not guarantee that the
 * driver will accept a specific format or stream configuration. */
constexpr bool valid_capture_dimensions(unsigned width, unsigned height)
{
  return width >= 16 && width <= sensor_width && height >= 2 && height <= sensor_height
         && width % 2 == 0 && height % 2 == 0;
}

/** @brief Borrowed same-exposure evidence, without object or navigation semantics.
 * @details Views avoid copying neural outputs into another queue. Keep the
 * associated lease alive while reading any tensor or interpreting its crop. */
struct FrameView {
  Span<const float> tensor; /**< Borrowed output values from the retained request. */
  Span<const uint8_t> tensor_info; /**< Matching driver shape metadata, not model semantics. */
  libcamera::Rectangle crop{}; /**< Sensor-space crop used by consumers such as box projection. */
  libcamera::Size image{}; /**< Configured JPEG dimensions, not the neural model input size. */
  uint64_t sequence = 0; /**< Camera request sequence; deliberately distinct from an FC shot number. */
  uint64_t sensor_timestamp_ns = 0; /**< Sensor timestamp in its camera clock domain; zero when absent. */
  int64_t exposure_us = -1; /**< Reported exposure duration; negative means unavailable. */
  int64_t frame_duration_us = -1; /**< Reported frame duration; negative means unavailable. */
  uint64_t monotonic_us = 0;
  uint64_t time_error_us = 100001;
  bool timing_sampled = false;
  bool crop_valid = false; /**< Distinguishes a reported crop from zero-initialized placeholder geometry. */
};

namespace detail {
/** @brief Confirm that a frame carries no neural output.
 * @param frame Borrowed view of the completed request.
 * @return True only if both tensor data and shape metadata are absent.
 * @details Check both views so partially exposed inference metadata is not
 * mistaken for a deliberately photo-only request. */
inline bool photo_only_frame(const FrameView &frame)
{
  return frame.tensor.empty() && frame.tensor_info.empty();
}
}

/** @brief Stack-owned model-specific operations; no dynamic plugin allocation.
 * @details validate qualifies readiness. write validates and publishes its own
 * analysis file; false or an exception must leave no partial analysis file.
 * Keep this contract small: capture mechanics should not learn detector boxes,
 * LOOM feature semantics or route-selection policy. */
struct FrameProcessor {
  const char *sidecar_suffix; /**< Trusted application-owned suffix with a lifetime covering the call. */
  bool (*validate)(const FrameView &frame); /**< Qualify model-specific readiness without retaining borrowed data. */
  bool (*write)(const FrameView &frame, const char *filename); /**< Exclusive-create analysis publication; failure must clean up its own partial file. */
  bool (*embed)(const FrameView &frame, exif_document *document) = nullptr;
};

/** @brief Exactly-once release of a borrowed frame, including exception exits.
 * @details The capture owner must outlive its lease. Only one lease per Camera
 * may exist. Move transfers ownership; copies and move assignment are disabled. */
class FrameLease {
public:
  /** @brief Nonthrowing release callback; a destructor must never unwind through it. */
  using Release = void (*)(void *) noexcept;

  /** @brief Associate borrowed evidence with exactly one release operation.
   * @param view Evidence whose backing storage is retained by @p owner.
   * @param owner Borrowed capture-owner context, alive until release completes.
   * @param release Callback to recycle the held request, or NULL for an unowned view.
   * @details Camera constructs leases before interpreting metadata so failures
   * during extraction also release the request. */
  FrameLease(FrameView view, void *owner, Release release) noexcept;
  /** @brief Release once on every scope exit, including a processing exception. */
  ~FrameLease();
  /** @brief Transfer release responsibility and empty the moved-from view.
   * @param other Lease to consume; it will no longer recycle the request. */
  FrameLease(FrameLease &&other) noexcept;
  /** @brief Copies are forbidden because two destructors cannot own one request. */
  FrameLease(const FrameLease &) = delete;
  /** @brief Copy assignment cannot duplicate capture ownership. */
  FrameLease &operator=(const FrameLease &) = delete;
  /** @brief Assignment is disabled; move construction is the supported transfer. */
  FrameLease &operator=(FrameLease &&) = delete;

  /** @brief Read evidence only while this unmoved lease remains alive.
   * @return Borrowed immutable view; its tensor spans have the lease's lifetime. */
  const FrameView &view() const noexcept { return view_; }

private:
  friend class Camera;
  FrameView view_; /**< Borrowed metadata, never a second image/tensor allocation. */
  void *owner_; /**< Context whose lifetime must exceed this lease. */
  Release release_; /**< Sole remaining release operation, NULL after a move. */
};

/** @brief One exclusive IMX500 owner with bounded reusable YUV420 buffers.
 * @pre Control operations and lease destruction are serialized by their owner;
 * the camera's callback thread is synchronized internally. */
class Camera {
public:
  /** @brief Construct unopened camera state; construction does not acquire the sensor. */
  Camera();
  /** @brief Stop streaming before freeing requests/buffers and releasing the camera.
   * @pre Every frame lease has already been destroyed. */
  ~Camera();
  /** @brief Camera owners cannot be copied. */
  Camera(const Camera &) = delete;
  /** @brief Assignment cannot replace an acquired sensor owner. */
  Camera &operator=(const Camera &) = delete;

  /** @brief Acquire one eligible camera and configure bounded capture, uploading an RPK only when requested.
   * @param model Readable RPK for IMX500 inference, or NULL for plain pixels.
   * @param width Requested JPEG width, not model-input width.
   * @param height Requested JPEG height, not model-input height.
    * @param photo_only Select IMX500 image-only capture and leave frame duration to libcamera.
   * @throws std::runtime_error for unavailable/ambiguous hardware or rejected configuration.
   * @details Configuration adjustments that change our expected geometry, YUV420
   * layout or buffer budget are rejected rather than silently changing downstream assumptions. */
  void open(const char *model, unsigned width = 2028, unsigned height = 1520,
            bool photo_only = false);
  /** @brief Lease a fresh post-request exposure and its own tensor metadata.
   * @return Move-only lease; destroy it before the next capture or camera shutdown.
   * @throws std::runtime_error for missing startup, a still-live prior lease,
   * queue failure, unavailable clock or frame timeout.
   * @details Freshness uses the sensor/BOOTTIME domain, not JPEG download time.
   * Model-specific validation remains the processor's responsibility. */
  FrameLease capture();
  /** @brief Map a leased frame and publish its JPEG plus available evidence.
   * @param frame Lease obtained from this camera, still alive and unmoved.
   * @param filename New absolute JPEG destination; existing images are not overwritten.
   * @param processor Application-owned sidecar policy for this same exposure.
   * @return True after JPEG publication, false for rejected layout/path or write failure.
   * @details Mapping is scoped separately from the lease: CPU DMA ownership ends
   * before the request can be recycled. Missing analysis does not discard good pixels.
   * @warning Success is not proof of a useful neural match, geolocation accuracy,
   * complete multi-file crash atomicity or parent-directory power-loss durability. */
  bool save(const FrameLease &frame, const char *filename, const FrameProcessor &processor);
  /** @brief Grant scoped CPU pixel access without transferring DMA/frame ownership.
   * @param frame Live lease acquired from this camera.
   * @param visitor Callback consuming validated YUV420 planes and stride before mappings are released.
   * @param context Borrowed callback state, not retained by the camera.
   * @return Callback result, or false for invalid ownership/layout/visitor.
   * @pre The visitor must not retain pixel pointers or release the lease during the call.
   * @details Prepare a model-sized tensor here, then release the lease before expensive inference. */
  bool with_pixels(const FrameLease &frame,
                    bool (*visitor)(const FrameView &, const std::array<const uint8_t *, 3> &,
                                    unsigned, void *), void *context);

private:
  struct Impl; /**< Camera/request implementation hidden from model-specific callers. */
  std::unique_ptr<Impl> impl_; /**< One owned capture state; libcamera has its own allocation policy. */
};

/** @brief Encode validated YUV420 planes directly, without full-frame RGB staging.
 * @param path New output path; exclusive creation refuses overwrites.
 * @param planes Accessible Y, Cb and Cr plane starts, immutable during encoding.
 * @param width Positive even image width in pixels.
 * @param height Positive even image height in pixels.
 * @param stride Even luma byte stride, at least the width rounded up to 16; chroma uses half.
 * @return True after encoding, flush, file sync and close; false removes partial output.
 * @pre The caller has checked plane lengths and image/stride layout. This low-level
 * encoder does not accept arbitrary unchecked external plane pointers. */
bool encode_jpeg(const char *path, const std::array<const uint8_t *, 3> &planes,
                  unsigned width, unsigned height, unsigned stride);

/** @brief Publish caller-validated YUV420 planes and same-exposure evidence.
 * @param view Borrowed evidence belonging to the supplied pixels.
 * @param planes Accessible YUV420 planes retained by the caller.
 * @param stride Luma byte stride; chroma uses half this stride.
 * @param filename New absolute JPEG destination.
 * @param processor Application's trusted sidecar suffix and writer callbacks.
 * @return True when the JPEG is published, regardless of missing analysis;
 * false when publication fails. File/processor exceptions may also propagate.
 * @pre Plane spans cover the declared image/stride and remain immutable for this
 * call. Capture retains its frame lease while invoking this routine.
 * @details Analysis failure retains a usable photograph. Publication failure
 * removes successfully created sidecars from this operation, never an existing image.
 * @warning JPEG and sidecars are not one filesystem transaction. Consumers must
 * check file/exposure identity, and directory power-loss durability is not guaranteed. */
bool publish_frame(const FrameView &frame, const std::array<const uint8_t *, 3> &planes,
                    unsigned stride, const char *filename, const FrameProcessor &processor);

/** @brief Run the shared probe/capture/READY-SHOT-OK worker with supplied model policy.
 * @param argc Argument count for a capture mode, excluding saved-photo matcher modes.
 * @param argv Mode, RPK and mode-specific dimensions/destination tokens.
 * @param processor Readiness and sidecar callbacks; never interpreted as navigation here.
 * @param application Diagnostic prefix selected by the independent entry point.
 * @return Zero for success, two for syntax/configuration misuse, one for runtime failure.
 * @details The worker announces READY after a fresh frame and, for detection,
 * model-specific validation. Every SHOT leases a new exposure without replay.
 * Standard output is protocol-only while diagnostics go to standard error. */
int run_worker(int argc, char **argv, const FrameProcessor &processor, const char *application);

/** @brief Run photo-only probe, capture or serve without model loading or inference.
 * @param argc Argument count for the photo-only command.
 * @param argv Command and mode-specific dimensions or destination.
 * @param application Diagnostic prefix.
 * @return Zero for success, two for invalid arguments, one for runtime failure. */
int run_photo_worker(int argc, char **argv, const char *application);

} // namespace imx500
#endif