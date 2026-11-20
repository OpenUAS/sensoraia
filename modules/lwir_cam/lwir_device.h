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

#ifndef SENSORAIA_LWIR_DEVICE_H
#define SENSORAIA_LWIR_DEVICE_H
/** @file
 * @brief Tiny1C/TDK acquisition and Tiny1C state, without image or mission logic.
 *
 * @details
 * This interface is the only boundary between LWIRcam and the camera hardware.
 * It delivers raw camera-layout frames plus timing evidence; image conversion,
 * calibration, geolocation and mission policy live elsewhere.
 *
 * @par Supported cameras
 * - Tiny1-C (USB 0bda:5840): combined YUYV transport of 256x384 pixels, i.e.
 *   a 256x192 image plane followed by a 256x192 temperature plane holding
 *   Kelvin*64 little-endian values; 196608 bytes per frame at about 25 fps.
 *   Its separate 256x192 image-only UVC mode is not combined data and is not used.
 * - TDK (USB 5474:5411): 640x512 image-only YUYV; no temperature plane.
 *
 * @par Backend
 * Capture runs in lwir_device_v4l2.c directly on the Linux uvcvideo kernel
 * driver with V4L2 MMAP buffers and usbfs control requests for Tiny1-C state.
 *
 * @par Ownership model
 * Every object is caller-owned storage; the backend performs no heap
 * allocation. A session owns one V4L2 descriptor (held under an exclusive
 * flock) and up to #LWIR_VIDEO_BUFFERS kernel buffers mapped into the process.
 * Mapped buffers are only borrowed between dequeue and requeue; callers never
 * see their addresses and receive copies instead.
 *
 * @par Threading
 * A session is not thread-safe: one thread must own all calls on it. The only
 * cross-context input is the borrowed sig_atomic_t cancellation flag, which
 * may be set from a signal handler or another thread and is polled at least
 * every 25 ms while waiting.
 *
 * @par Lifecycle invariants
 * - Zero-initialize, lwir_device_open(), acquire/discard/state calls, then
 *   lwir_device_close(). Close before reopening.
 * - A fatal kernel or protocol error poisons the session (EIO) until it is
 *   closed and reopened; no partial recovery is attempted.
 * - The eight-frame startup qualification runs once per USB power instance,
 *   remembered across reopen and process restart, and is never repeated for
 *   later requests, gaps, invalid frames or reconnection of the same instance.
 */
#include "lwir_frame.h"
#include "lwir_state.h"
#include <signal.h>

/** @brief Number of V4L2 MMAP buffers requested from uvcvideo.
 * @details Four keeps one buffer in user space for inspection while the
 * kernel still has spare buffers to fill, so a short inspection does not drop
 * the next frame. The driver may grant fewer; at least two are required.
 */
#define LWIR_VIDEO_BUFFERS 4U

/** @brief One kernel capture buffer mapped into this process. */
struct lwir_video_buffer {
    void *address; /**< Start of the shared mapping; only valid while @c mapped. */
    size_t length; /**< Mapped length reported by VIDIOC_QUERYBUF; needed for munmap. */
    /** @brief Ownership flag for the mapping.
     * @details Stays true after a failed munmap so a later close can retry and
     * a reopen is refused instead of leaking or double-mapping memory.
     */
    bool mapped;
};

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bounded camera policy; Tiny1C requires full 256x192 image and temperature planes.
 * @details Zero dimensions select native geometry; Tiny1C's 256x192 image-only UVC mode is not combined data.
 * The same structure is used for open and for each acquisition: open writes
 * the negotiated geometry and layout back, and acquisition requires them to
 * still match the session.
 */
struct lwir_device_options {
    /** Optional USB serial string that must match exactly; NULL accepts any serial.
     * Must fit in lwir_device::serial including its terminator. */
    const char *serial;
    /** Optional explicit V4L2 node (for example /dev/video2). NULL scans
     * /dev/video0..63 and fails with EEXIST when more than one camera matches. */
    const char *device_path;
    /** @c width / @c height: per-plane native geometry, or zero for "native"
     * before open; written on open success.
     * @c warmup_ms: extra delay after the request starts, applied only on the
     * first startup of a USB power instance; must be below @c timeout_ms.
     * @c timeout_ms: whole-call budget in milliseconds, 1..120000. */
    unsigned width, height, warmup_ms, timeout_ms;
    /** @c scene_check: also require lwir_frame_plausible(); combined layout only,
     * so enabling it for image-only sessions rejects every frame.
     * @c settle: require consecutive frames to agree per lwir_frame_settled()
     * before accepting, at the cost of at least one extra frame period. */
    bool scene_check, settle;
    /** Optional measurement region validated against the native geometry at open; NULL skips it. */
    const struct lwir_measurement *measurement;
    /** Borrowed cancellation flag; non-zero aborts waits with EINTR. NULL disables cancellation. */
    const volatile sig_atomic_t *cancelled;
    /** Output of open: the detected camera's frame layout. */
    enum lwir_frame_layout layout;
    /** Fail open with ENOTSUP when only an image-only camera is present. */
    bool requires_thermal;
};

/** @brief Caller-owned session, initialized to zero before opening.
 * @details Handles belong only to the driver. Keeping their storage here avoids
 * a Sensoraia allocation; callers use the functions below, not backend handles directly.
 */
struct lwir_device {
    const volatile sig_atomic_t *cancelled; /**< Borrowed from the options at open; may be NULL. */
    char serial[64]; /**< USB serial read from sysfs, empty when the device reports none. */
    enum lwir_frame_layout layout; /**< Combined Tiny1-C or image-only TDK, fixed by USB identity. */
    int video_fd; /**< V4L2 descriptor holding the exclusive flock; -1 once closed. */
    /** @c video_streaming is set just before STREAMON so close always issues
     * STREAMOFF, even after a partially failed start. @c video_have_sequence
     * says whether @c video_last_sequence holds a real kernel sequence yet. */
    bool video_open, video_streaming, video_have_sequence;
    /** Sticky poison after a fatal kernel/protocol error; every later call
     * fails with EIO until close and reopen. */
    bool video_failed;
    /** @c video_count: buffers granted by REQBUFS (2..#LWIR_VIDEO_BUFFERS).
     * @c video_width / @c video_height: per-plane geometry, not transport height. */
    unsigned video_count, video_width, video_height;
    uint32_t video_last_sequence; /**< Last raw 32-bit V4L2 sequence number seen. */
    /** 64-bit extension of the kernel sequence; skipped kernel numbers also
     * advance it, so gaps stay visible to consumers. */
    uint64_t video_sequence;
    uint64_t video_good_arrival_us; /**< Host MONOTONIC arrival of the last qualifying frame. */
    /** Consecutive good frames, saturating at eight. Once eight is reached it
     * latches for the rest of the USB power instance. */
    unsigned video_good_frames;
    /** True when this USB power instance already completed startup
     * qualification, either now or according to the runtime marker. */
    bool video_power_ready;
    /** "bootid:sysfs-inode:busnum:devnum"; changes whenever the camera
     * re-enumerates, which is the only observable power boundary on Linux. */
    char video_power_key[160];
    char video_power_port[64]; /**< Physical USB port name (for example 1-1.2) used to name the marker. */
    /** USB bus and address used to reach the same device through usbfs for
     * vendor state/control requests. */
    unsigned video_usb_bus, video_usb_address;
    size_t video_bytes; /**< Expected bytesused per frame (sizeimage) for the layout. */
    struct lwir_video_buffer video_buffers[LWIR_VIDEO_BUFFERS]; /**< Mappings; only the first @c video_count are used. */
};

/** @brief Host observation plus optional kernel clock evidence, never automatic exposure qualification. */
struct lwir_device_observation {
    /** @c sequence: session-extended frame sequence (see lwir_device::video_sequence).
     * @c arrival_us: host CLOCK_MONOTONIC time when the frame was dequeued, in
     * microseconds; it includes USB delivery delay after the sensor frame start. */
    uint64_t sequence, arrival_us;
    /** Kernel buffer timestamp in CLOCK_MONOTONIC microseconds (uvcvideo takes
     * it at start of frame); zero when the buffer clock is not MONOTONIC. */
    uint64_t kernel_monotonic_us;
    /** Raw V4L2 timestamp type and source flags, so consumers can judge what
     * @c kernel_monotonic_us actually measures. */
    unsigned kernel_timestamp_flags;
};

/** @brief Open/start a bounded camera-specific combined or image-only YUY2 stream.
 * @param options Requested policy; native dimensions are written on success.
 * @param device Empty caller-owned session, closed on failure.
 * @return True on success; errno describes failure. Close before reopening.
 * @par Errors
 * EINVAL invalid options; EBUSY session still owns a descriptor or mapping;
 * ENODEV no matching camera; EEXIST more than one matching camera during a
 * scan; EWOULDBLOCK another process holds the camera; ENOTSUP unsupported
 * format or image-only camera when thermal data is required; ETIMEDOUT or
 * EINTR deadline or cancellation; other values come from the kernel.
 */
bool lwir_device_open(struct lwir_device_options *options, struct lwir_device *device);
/** @brief Copy a qualified, fresh camera-layout frame into caller-owned storage.
 * @param device Open session with an optional borrowed cancellation flag.
 * @param options Native geometry, warmup and qualification policy.
 * @param destination Writable frame buffer, retained through the call.
 * @param capacity Accessible bytes; at least lwir_frame_bytes(width, height).
 * @param observation Sequence and host arrival, written only on success.
 * @return True on acceptance; ignore destination on failure and never auto-retry publication.
 * @details Native image/observation buffers must not alias each other, the session
 * or mapped camera buffers. Fatal kernel ownership failures require close/reopen.
 * Startup qualification is retained for the USB instance across reopen/restart;
 * no subsequent request, gap or invalid frame imposes another eight-frame wait.
 * Freshness: when the kernel timestamp is CLOCK_MONOTONIC, the accepted frame
 * started at or after this call began, so it never predates the trigger.
 * @par Errors
 * EINVAL bad arguments or aliasing; EIO poisoned session; ETIMEDOUT deadline;
 * EINTR cancellation; EPROTO/EOVERFLOW inconsistent kernel data or clock.
 */
bool lwir_device_acquire(struct lwir_device *device, const struct lwir_device_options *options,
                         uint8_t *destination, size_t capacity, struct lwir_device_observation *observation);
/** @brief Drain one pending frame without retaining the borrowed UVC buffer.
 * @param device Open session.
 * @param received Whether a frame was present, written on success.
 * @return True on successful polling, even when no frame is present.
 * @details Non-blocking. A drained frame still counts toward startup
 * qualification, so idle draining can complete it before the first request.
 */
bool lwir_device_discard(struct lwir_device *device, bool *received);
/** @brief Read bounded optional state after acquisition, never writing device settings.
 * @param device Open session with its cancellation flag.
 * @param state Caller-owned result with per-field failure statuses.
 * @details The snapshot is not frame-atomic and cannot establish gain at exposure.
 * Image-only TDK sessions return a not-requested snapshot without USB commands.
 */
void lwir_device_read_state(struct lwir_device *device, struct lwir_device_state *state);
/** @brief Submit one Tiny1-C vendor control write (or FFC action) as a standalone operation.
 * @param device Open combined-layout session; image-only cameras fail with ENOTSUP.
 * @param request Validated control kind and value.
 * @param result Caller-owned reply, always initialized.
 * @return True when the camera acknowledged the request.
 * @details Uses vendor control transfers through usbfs while uvcvideo keeps
 * streaming. Normal capture never calls this.
 * @warning The write is submitted once and never replayed automatically. A
 * failure (including a close error after submission) does not prove the
 * camera ignored it; read the value back rather than blindly retrying.
 * @par Errors
 * EINVAL invalid request; ENOTSUP image-only camera; ENODEV identity changed;
 * EINTR cancelled; ETIMEDOUT camera did not answer; EIO other failures.
 */
bool lwir_device_control(struct lwir_device *device, const struct lwir_control_request *request,
                          struct lwir_control_result *result);
/** @brief Read the current value of one Tiny1-C control without writing it.
 * @param device Open combined-layout session.
 * @param kind Control to read; LWIR_CONTROL_FFC is an action and is rejected (EINVAL).
 * @param result Caller-owned reply, always initialized.
 * @return True when the value was read; errors as for lwir_device_control().
 */
bool lwir_device_control_read(struct lwir_device *device, enum lwir_control_kind kind,
                               struct lwir_control_result *result);
/** @brief Close an initialized session; safe repeatedly and preserves errno.
 * @details Native failed unmaps remain owned for a subsequent close and prevent
 * reopening. Linux descriptor-close errors are never retried on a reused number.
 */
void lwir_device_close(struct lwir_device *device);

#ifdef __cplusplus
}
#endif
#endif