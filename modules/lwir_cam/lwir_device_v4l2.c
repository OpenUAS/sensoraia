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

/** @file
 * @brief Native Linux UVC acquisition with bounded mapped-buffer ownership.
 *
 * @details
 * Implements lwir_device.h directly on the kernel uvcvideo driver through
 * V4L2 MMAP buffers and Linux usbfs control transfers.
 *
 * @par Cameras
 * - Tiny1-C, USB 0bda:5840: YUYV transport 256x384 = 256x192 image plane
 *   followed by a 256x192 temperature plane (Kelvin*64, little-endian),
 *   196608 bytes per frame at 25 fps (#LWIR_COMBINED).
 * - TDK, USB 5474:5411: 640x512 YUYV image only (#LWIR_YUYV_IMAGE).
 *
 * @par How the pieces fit
 * 1. Discovery (video_probe(), video_identity()) maps a /dev/videoN node to
 *    its USB device through sysfs, checks vendor/product/serial and takes an
 *    exclusive non-blocking flock so no second process (for example a control
 *    command while USHER captures) can share the camera.
 * 2. Setup (video_setup()) negotiates the exact native format and maps
 *    #LWIR_VIDEO_BUFFERS MMAP buffers, then starts streaming.
 * 3. Acquisition (lwir_device_acquire()) polls in 25 ms slices, dequeues a
 *    buffer, validates it, copies it if accepted and always requeues it.
 * 4. Startup qualification is persisted per USB power instance by a runtime
 *    marker (video_power_marker()).
 * 5. Optional Tiny1-C state/control uses vendor control transfers over usbfs
 *    (video_state_open(), video_state_transfer()) beside the running stream.
 *
 * @par Buffer ownership
 * A mapped buffer belongs to user space only between VIDIOC_DQBUF and
 * VIDIOC_QBUF. Everything needed from it (pixels, sequence, timestamp, flags)
 * is copied or snapshotted before requeue: after QBUF the kernel may refill
 * the memory and the driver may rewrite buffer metadata at any moment. This
 * ordering was required by a real hardware fault, not by theory.
 *
 * @par Failure model
 * Fatal kernel or protocol errors set lwir_device::video_failed, poisoning
 * the session until close and reopen; partial recovery of a V4L2 queue in an
 * unknown state is not attempted. A failed munmap keeps ownership so a later
 * close retries it and open refuses with EBUSY. close() is never retried
 * because Linux releases the descriptor number even on error, and another
 * thread may already have reused it.
 *
 * @par Startup qualification
 * The first power-up of the camera needs eight consecutive valid frames.
 * Linux exposes no sensor power-cycle counter, so a fresh USB enumeration is
 * the observable power boundary: the marker
 * /run/lock/sensoraia-lwir-UID/usb-PORT.ready stores
 * "bootid:sysfs-inode:busnum:devnum". Qualification is never repeated for
 * later requests, sequence gaps, bad frames, reopen or worker restart within
 * the same instance. Marker failures are explicit errors rather than a silent
 * fallback to another warmup.
 *
 * @par Freshness and latency
 * A frame is accepted only when its kernel CLOCK_MONOTONIC timestamp (taken
 * by uvcvideo at start of frame) is not earlier than the request, so the
 * photo never predates the trigger. Measured cost is about 8.6 ms average
 * wait for the next frame start plus about 36.1 ms USB delivery; that is the
 * sensor/USB limit, not overhead of this code. For non-MONOTONIC clocks the
 * already-queued buffers are drained once instead.
 *
 * @par Policies
 * - No heap allocation; all state is in the caller's struct lwir_device.
 * - No memcpy by project policy: bounded byte loops are used. Compilers
 *   vectorize them; -fno-tree-loop-distribute-patterns stops them from being
 *   turned back into memcpy calls.
 * - Interrupted system calls are retried a bounded number of times so a
 *   signal storm cannot spin forever.
 *
 * @par Threading
 * Single-threaded per session. Only the borrowed cancellation flag is read
 * asynchronously.
 */
#include "lwir_device.h"

#include <errno.h>
#include <time.h>
#include <linux/videodev2.h>
#include <linux/usbdevice_fs.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

/**
 * @brief Check that the format the driver actually granted is exactly the native camera format.
 * @details VIDIOC_S_FMT may silently adjust any requested field, so every
 * field that affects buffer interpretation is compared after negotiation:
 * geometry, YUYV, progressive field order, tightly packed rows and the exact
 * frame size. For the combined layout the transport height is twice the plane
 * height because the temperature plane follows the image plane.
 * @param options Caller policy; non-zero width/height must equal the native
 *        geometry and an optional measurement region must fit it.
 * @param layout Layout implied by the USB identity.
 * @param format Format returned by the driver.
 * @return True only for an exact match; false otherwise (errno untouched).
 */
static bool video_format_valid(const struct lwir_device_options *options,
                                enum lwir_frame_layout layout, const struct v4l2_format *format)
{
    if (!options || !format || format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE
        || (layout != LWIR_COMBINED && layout != LWIR_YUYV_IMAGE)) return false;
    const unsigned width = layout == LWIR_COMBINED ? 256U : 640U;
    const unsigned height = layout == LWIR_COMBINED ? 192U : 512U;
    /* lwir_frame_bytes() sizes both planes (4 bytes/pixel); image-only YUYV carries half of that. */
    const size_t bytes = lwir_frame_bytes(width, height) / (layout == LWIR_COMBINED ? 1U : 2U);
    const struct v4l2_pix_format *pixels = &format->fmt.pix;
    return (!options->width || (options->width == width && options->height == height))
        && pixels->width == width && pixels->height == height * (layout == LWIR_COMBINED ? 2U : 1U)
        && pixels->pixelformat == V4L2_PIX_FMT_YUYV && pixels->field == V4L2_FIELD_NONE
        && pixels->bytesperline == width * 2U && pixels->sizeimage == bytes
        && (!options->measurement || lwir_measurement_valid(width, height, options->measurement));
}

/**
 * @brief Poll the borrowed cancellation flag.
 * @param device Session whose optional flag is read.
 * @return True when a flag is attached and set.
 */
static bool video_cancelled(const struct lwir_device *device)
{
    return device->cancelled && *device->cancelled;
}

/**
 * @brief Read CLOCK_MONOTONIC in microseconds.
 * @details MONOTONIC is the clock uvcvideo uses for buffer timestamps, so
 * host and kernel times are directly comparable. The conversion is checked
 * for malformed values and 64-bit overflow instead of trusting the kernel.
 * @param microseconds Output, written only on success.
 * @return True on success.
 * @par Errors
 * errno from clock_gettime(); EPROTO malformed timespec; EOVERFLOW unrepresentable.
 */
static bool video_now(uint64_t *microseconds)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now)) return false;
    if (now.tv_sec < 0 || now.tv_nsec < 0 || now.tv_nsec >= 1000000000L) { errno = EPROTO; return false; }
    /* Division form of the bound avoids computing the overflowing product. */
    if ((uint64_t)now.tv_sec > (UINT64_MAX - (uint64_t)now.tv_nsec / 1000U) / 1000000U) { errno = EOVERFLOW; return false; }
    *microseconds = (uint64_t)now.tv_sec * 1000000U + (uint64_t)now.tv_nsec / 1000U;
    return true;
}

/**
 * @brief Combined cancellation and deadline check placed between setup steps.
 * @details Setup issues several ioctls that can each take noticeable time on
 * USB; checking between them keeps open within its timeout budget.
 * @param device Session whose cancellation flag is consulted.
 * @param deadline Absolute CLOCK_MONOTONIC limit in microseconds.
 * @return True to continue.
 * @par Errors
 * EINTR cancelled; ETIMEDOUT deadline reached; clock errors from video_now().
 */
static bool video_checkpoint(const struct lwir_device *device, uint64_t deadline)
{
    if (video_cancelled(device)) { errno = EINTR; return false; }
    uint64_t now;
    if (!video_now(&now)) return false;
    if (now >= deadline) { errno = ETIMEDOUT; return false; }
    return true;
}

/**
 * @brief ioctl() with a bounded retry on EINTR.
 * @details Signals (including the one that sets the cancellation flag) may
 * interrupt V4L2 calls. Retrying keeps transient interruptions harmless,
 * while the eight-attempt cap guarantees a signal storm still returns so the
 * caller can notice cancellation.
 * @param descriptor Open V4L2 descriptor.
 * @param request V4L2 request code.
 * @param argument Request-specific structure, updated by the kernel.
 * @return ioctl() result; -1 with errno set on failure (EINTR after the cap).
 */
static int video_ioctl(int descriptor, unsigned long request, void *argument)
{
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
        const int result = ioctl(descriptor, request, argument);
        if (result >= 0 || errno != EINTR) return result;
    }
    errno = EINTR;
    return -1;
}

/**
 * @brief Read one short sysfs/procfs attribute as a validated single-line string.
 * @details Reads until EOF into the caller buffer. If the buffer fills, one
 * further byte is attempted: any data there means the value was truncated and
 * the whole read is rejected, because a cut-off identity string could match
 * the wrong device. Trailing newline characters are stripped, a terminator
 * must still fit, and control characters are refused so the value is safe to
 * embed in the marker file and log lines.
 * @param directory sysfs directory.
 * @param name Attribute file name inside @p directory.
 * @param text Output buffer; NUL-terminated only on success.
 * @param capacity Size of @p text, at least 2.
 * @return True for a non-empty, complete, printable value. errno is
 *         meaningful only for open/read failures; validation failures leave it unspecified.
 */
static bool video_text(const char *directory, const char *name, char *text, size_t capacity)
{
    char path[PATH_MAX];
    int count = snprintf(path, sizeof(path), "%s/%s", directory, name);
    if (count < 0 || (size_t)count >= sizeof(path) || capacity < 2) return false;
    const int descriptor = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor < 0) return false;
    size_t used = 0;
    bool valid = true;
    unsigned interruptions = 0;
    while (used < capacity) {
        ssize_t received = read(descriptor, text + used, capacity - used);
        if (received < 0 && errno == EINTR && ++interruptions < 8) continue;
        if (received < 0) { valid = false; break; }
        if (!received) break;
        used += (size_t)received;
    }
    /* A full buffer is ambiguous: probe for one more byte to tell "exact fit" from truncation. */
    if (valid && used == capacity) {
        char extra;
        if (read(descriptor, &extra, 1) != 0) valid = false;
    }
    if (close(descriptor)) valid = false;
    if (!valid || !used) return false;
    while (used && (text[used - 1] == '\n' || text[used - 1] == '\r')) --used;
    /* Without a trailing newline to strip, an exact fit leaves no room for the terminator. */
    if (used >= capacity) return false;
    for (size_t index = 0; index < used; ++index)
        if ((unsigned char)text[index] < 32 || text[index] == 127) return false;
    text[used] = '\0';
    return used != 0;
}

/**
 * @brief Resolve a V4L2 character device to its USB camera identity through sysfs.
 * @details
 * 1. /sys/dev/char/MAJOR:MINOR/device points at the USB interface that owns
 *    the video node; it is canonicalized with realpath().
 * 2. Walk up at most 16 directories to the first ancestor exposing
 *    idVendor/idProduct, which is the USB device itself. The first such
 *    ancestor decides: an unsupported vendor/product is rejected rather than
 *    walking further up to a hub.
 * 3. The last path component is the physical port name (digits, '-' and
 *    '.'), which is stable across re-enumeration and so names the marker.
 * 4. Build the power key from boot_id, the sysfs directory inode, busnum and
 *    devnum. A new boot, a new enumeration (new sysfs node, new address) or a
 *    different bus all change the key; that is how a sensor power cycle is
 *    detected without any counter from the camera.
 *
 * Bus and address are range-checked because they later form the usbfs path
 * /dev/bus/usb/BBB/DDD and its expected device number (USB addresses are
 * 1..127).
 * @param node stat() of the video node; must be a character device.
 * @param layout Output: layout implied by vendor/product.
 * @param serial Output: USB serial or empty string.
 * @param power_key Output: power-instance key (see above).
 * @param power_port Output: USB port name.
 * @param usb_bus Output: USB bus number.
 * @param usb_address Output: USB device address.
 * @return True when the node belongs to a supported camera.
 * @par Errors
 * ENODEV not a supported camera or no USB ancestor; EPROTO malformed bus or
 * address; EOVERFLOW key too long; other errno from realpath/stat/sysfs reads.
 * @note Outputs may be partially written on failure.
 */
static bool video_identity(const struct stat *node, enum lwir_frame_layout *layout, char serial[64],
                            char power_key[160], char power_port[64], unsigned *usb_bus, unsigned *usb_address)
{
    if (!S_ISCHR(node->st_mode)) { errno = ENODEV; return false; }
    char path[PATH_MAX], resolved[PATH_MAX];
    int length = snprintf(path, sizeof(path), "/sys/dev/char/%u:%u/device", major(node->st_rdev), minor(node->st_rdev));
    if (length < 0 || (size_t)length >= sizeof(path) || !realpath(path, resolved)) return false;
    for (unsigned parent = 0; parent < 16; ++parent) {
        char vendor[16], product[16];
        if (video_text(resolved, "idVendor", vendor, sizeof(vendor)) && video_text(resolved, "idProduct", product, sizeof(product))) {
            if (!strcmp(vendor, "0bda") && !strcmp(product, "5840")) *layout = LWIR_COMBINED;
            else if (!strcmp(vendor, "5474") && !strcmp(product, "5411")) *layout = LWIR_YUYV_IMAGE;
            else { errno = ENODEV; return false; }
            serial[0] = '\0';
            /* Serial is optional; a camera without one can still be selected without --serial. */
            if (!video_text(resolved, "serial", serial, 64)) serial[0] = '\0';
            char boot[64], bus[16], address[16];
            struct stat identity;
            const char *port = strrchr(resolved, '/');
            if (!port || !port[1] || strlen(port + 1) >= 64
                || strspn(port + 1, "0123456789-.") != strlen(port + 1)
                || stat(resolved, &identity) || !S_ISDIR(identity.st_mode)
                || !video_text("/proc/sys/kernel/random", "boot_id", boot, sizeof(boot))
                || !video_text(resolved, "busnum", bus, sizeof(bus))
                || !video_text(resolved, "devnum", address, sizeof(address))) return false;
            if (!bus[0] || !address[0] || strspn(bus, "0123456789") != strlen(bus)
                || strspn(address, "0123456789") != strlen(address)) { errno = EPROTO; return false; }
            errno = 0;
            const unsigned long bus_number = strtoul(bus, NULL, 10), device_number = strtoul(address, NULL, 10);
            if (errno || !bus_number || bus_number > 999 || !device_number || device_number > 127) { errno = EPROTO; return false; }
            *usb_bus = (unsigned)bus_number; *usb_address = (unsigned)device_number;
            /* The sysfs inode differs per enumeration even when bus/address happen to be reused. */
            const int count = snprintf(power_key, 160, "%s:%llu:%s:%s", boot,
                (unsigned long long)identity.st_ino, bus, address);
            if (count < 0 || count >= 160) { errno = EOVERFLOW; return false; }
            /* Inclusive bound copies the terminator; length was checked against 64 above. */
            for (size_t index = 0; index <= strlen(port + 1); ++index) power_port[index] = port[index + 1];
            return true;
        }
        char *separator = strrchr(resolved, '/');
        if (!separator || separator == resolved) break;
        *separator = '\0';
    }
    errno = ENODEV;
    return false;
}

/**
 * @brief Test one V4L2 node and, if it is a usable camera, open and lock it.
 * @details
 * Identity is established from sysfs before opening, so unrelated video
 * devices are never opened. After open, fstat() must report the same inode
 * and device numbers as the earlier lstat(); otherwise the node was replaced
 * in between and the descriptor might belong to a different device. The
 * driver must be uvcvideo with capture and streaming capability.
 *
 * flock(LOCK_EX | LOCK_NB) gives exclusive ownership across processes:
 * a second LWIRcam instance, or a control command issued while USHER is
 * capturing, fails immediately instead of fighting over the stream.
 * The lock is released automatically when the descriptor is closed.
 *
 * The return value has three states so the scan in lwir_device_open() can
 * skip non-matching nodes but stop on a matching camera it cannot use: a busy
 * or permission-denied camera must be reported, not silently ignored.
 * @param path Candidate node (symlinks resolved first).
 * @param options Selection policy (serial filter, cancellation flag).
 * @param candidate Output session, overwritten only on a match.
 * @return 1 matched, opened and locked; 0 not a candidate (errno says why);
 *         -1 hard failure on a matching node (errno from open/ioctl/flock).
 */
static int video_probe(const char *path, const struct lwir_device_options *options, struct lwir_device *candidate)
{
    char resolved[PATH_MAX], serial[64] = {0}, power_key[160] = {0}, power_port[64] = {0};
    struct stat node, opened;
    enum lwir_frame_layout layout;
    unsigned usb_bus, usb_address;
    if (!realpath(path, resolved) || lstat(resolved, &node)
        || !video_identity(&node, &layout, serial, power_key, power_port, &usb_bus, &usb_address)) return 0;
    if (options->serial && strcmp(options->serial, serial)) { errno = ENODEV; return 0; }
    const int descriptor = open(resolved, O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor < 0) return -1;
    int error = 0;
    struct v4l2_capability capability = {0};
    if (fstat(descriptor, &opened) || !S_ISCHR(opened.st_mode) || opened.st_dev != node.st_dev || opened.st_rdev != node.st_rdev
        || opened.st_ino != node.st_ino) error = ENODEV;
    else if (video_ioctl(descriptor, VIDIOC_QUERYCAP, &capability)) error = errno;
    /* device_caps describes this node; capabilities covers the whole physical device when DEVICE_CAPS is set. */
    const uint32_t capabilities = capability.capabilities & V4L2_CAP_DEVICE_CAPS ? capability.device_caps : capability.capabilities;
    const bool capture = !error && !strncmp((const char *)capability.driver, "uvcvideo", sizeof(capability.driver))
        && (capabilities & V4L2_CAP_VIDEO_CAPTURE) && (capabilities & V4L2_CAP_STREAMING);
    if (capture && flock(descriptor, LOCK_EX | LOCK_NB)) error = errno;
    if (error || !capture) {
        /* The descriptor was never published, so a close error cannot affect any other owner. */
        close(descriptor);
        errno = error ? error : ENOTSUP;
        return error ? -1 : 0;
    }
    *candidate = (struct lwir_device){.video_fd = descriptor, .video_open = true,
        .layout = layout, .cancelled = options->cancelled, .video_usb_bus = usb_bus, .video_usb_address = usb_address};
    for (size_t index = 0; index < sizeof(candidate->serial); ++index) candidate->serial[index] = serial[index];
    for (size_t index = 0; index < sizeof(candidate->video_power_key); ++index) candidate->video_power_key[index] = power_key[index];
    for (size_t index = 0; index < sizeof(candidate->video_power_port); ++index) candidate->video_power_port[index] = power_port[index];
    return 1;
}

/**
 * @brief Read or publish the per-USB-instance startup qualification marker.
 * @details
 * The marker /run/lock/sensoraia-lwir-UID/usb-PORT.ready holds the power key
 * of the USB instance that already completed eight-frame qualification. /run
 * is normally tmpfs, so a reboot clears it; boot_id inside the key covers the
 * case where it does not.
 *
 * Security and integrity checks: the directory must be a real directory owned
 * by the effective user with no group/other access, and the marker a regular
 * single-link file with the same ownership. All opens use O_NOFOLLOW and
 * *at() calls relative to the verified directory descriptor, so a symlink or
 * hard link planted by another user cannot redirect or spoof the marker.
 *
 * Read mode (@p complete false): a missing marker means "not yet qualified";
 * a marker with a different key (an earlier power instance) likewise. A
 * matching key restores eight good frames so no wait is imposed after reopen
 * or worker restart.
 *
 * Write mode (@p complete true): the key is written to usb-PORT.pending,
 * created with O_EXCL, and renamed over the .ready file. rename is atomic, so
 * a concurrent reader or a crash sees either the old or the new complete
 * marker, never a partial key.
 *
 * Any failure is returned as an error instead of being treated as "not
 * ready": a silent fallback would quietly impose a new warmup on a camera that
 * does not need one and hide a broken runtime directory.
 * @param device Session supplying the power key and port; @c video_power_ready
 *        and possibly @c video_good_frames are updated.
 * @param complete False to read, true to publish completion.
 * @return True on success (including "no marker" in read mode).
 * @par Errors
 * EINVAL missing key/port; EACCES unsafe ownership, mode or link count;
 * EOVERFLOW name too long; EPROTO embedded NUL; EIO unexpected EOF/short
 * write; other errno from mkdir/open/read/write/rename/close.
 */
static bool video_power_marker(struct lwir_device *device, bool complete)
{
    char directory[128], filename[96], temporary[96];
    int count = snprintf(directory, sizeof(directory), "/run/lock/sensoraia-lwir-%lu", (unsigned long)geteuid());
    if (count < 0 || (size_t)count >= sizeof(directory) || !device->video_power_key[0] || !device->video_power_port[0]) { errno = EINVAL; return false; }
    if (mkdir(directory, 0700) && errno != EEXIST) return false;
    const int folder = open(directory, O_RDONLY | O_DIRECTORY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (folder < 0) return false;
    struct stat identity;
    bool valid = fstat(folder, &identity) == 0 && S_ISDIR(identity.st_mode)
        && identity.st_uid == geteuid() && !(identity.st_mode & 0077);
    if (!valid) { close(folder); errno = EACCES; return false; }
    count = snprintf(filename, sizeof(filename), "usb-%s.ready", device->video_power_port);
    int pending = snprintf(temporary, sizeof(temporary), "usb-%s.pending", device->video_power_port);
    if (count < 0 || (size_t)count >= sizeof(filename) || pending < 0 || (size_t)pending >= sizeof(temporary)) {
        close(folder); errno = EOVERFLOW; return false;
    }
    if (!complete) {
        const int file = openat(folder, filename, O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
        if (file < 0) {
            const int error = errno;
            close(folder);
            if (error == ENOENT) { device->video_power_ready = false; return true; }
            errno = error; return false;
        }
        char key[160] = {0};
        valid = fstat(file, &identity) == 0 && S_ISREG(identity.st_mode) && identity.st_uid == geteuid()
            && identity.st_nlink == 1 && !(identity.st_mode & 0077) && identity.st_size > 0 && identity.st_size < (off_t)sizeof(key);
        if (!valid) errno = EACCES;
        if (valid) {
            size_t used = 0;
            unsigned interruptions = 0;
            while (used < (size_t)identity.st_size) {
                const ssize_t received = read(file, key + used, (size_t)identity.st_size - used);
                if (received < 0 && errno == EINTR && ++interruptions < 8) continue;
                if (received <= 0) { valid = false; if (!received) errno = EIO; break; }
                used += (size_t)received;
            }
            if (valid && memchr(key, '\0', used)) { valid = false; errno = EPROTO; }
        }
        /* Keep the first failure's errno; close errors override it only if they occur. */
        int error = errno;
        if (close(file)) { valid = false; error = errno; }
        if (close(folder)) { valid = false; error = errno; }
        if (!valid) { errno = error ? error : EIO; return false; }
        device->video_power_ready = !strcmp(key, device->video_power_key);
        /* Seed the latched counter so acquisition sees the instance as qualified immediately. */
        if (device->video_power_ready) device->video_good_frames = 8;
        return true;
    }
    /* A leftover .pending from an interrupted earlier write would make O_EXCL fail forever. */
    if (unlinkat(folder, temporary, 0) && errno != ENOENT) { const int error = errno; close(folder); errno = error; return false; }
    const int file = openat(folder, temporary, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (file < 0) { const int error = errno; close(folder); errno = error; return false; }
    const size_t length = strlen(device->video_power_key);
    size_t written = 0;
    unsigned interruptions = 0;
    while (written < length) {
        const ssize_t result = write(file, device->video_power_key + written, length - written);
        if (result < 0 && errno == EINTR && ++interruptions < 8) continue;
        if (result <= 0) { valid = false; if (!result) errno = EIO; break; }
        written += (size_t)result;
    }
    int error = errno;
    /* Close before rename so a write-back error reported at close cannot publish a bad marker. */
    if (close(file)) { valid = false; error = errno; }
    if (valid && renameat(folder, temporary, folder, filename)) { valid = false; error = errno; }
    if (!valid) unlinkat(folder, temporary, 0);
    if (close(folder)) { valid = false; error = errno; }
    if (valid) device->video_power_ready = true;
    else errno = error;
    return valid;
}

/**
 * @brief Release a session in the reverse order of setup.
 * @details
 * 1. STREAMOFF returns all buffers from the driver queue to user space.
 * 2. munmap each mapping. A failed unmap keeps its entry marked as mapped and
 *    the session struct is not cleared, so a later close retries it and
 *    lwir_device_open() refuses with EBUSY rather than losing track of
 *    memory the process still maps.
 * 3. REQBUFS with count 0 frees the kernel buffers when the driver allows it.
 * 4. close() the descriptor exactly once. Its state is cleared first and a
 *    close error is only logged: Linux frees the number even on error, so
 *    retrying could close an unrelated descriptor opened by another thread.
 *
 * Errors of STREAMOFF/REQBUFS are deliberately ignored; there is no better
 * recovery than continuing teardown, and closing the descriptor ends the
 * stream regardless.
 * @param device Session to release; NULL and already-closed sessions are accepted.
 * @note errno is preserved so callers can close on an error path without
 *       losing the original cause.
 */
void lwir_device_close(struct lwir_device *device)
{
    if (!device) return;
    const int error = errno;
    bool mappings_remain = false;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (device->video_open) {
        if (device->video_streaming) video_ioctl(device->video_fd, VIDIOC_STREAMOFF, &type);
    }
    for (unsigned index = 0; index < LWIR_VIDEO_BUFFERS; ++index) {
        struct lwir_video_buffer *buffer = &device->video_buffers[index];
        if (!buffer->mapped) continue;
        if (munmap(buffer->address, buffer->length) == 0) *buffer = (struct lwir_video_buffer){0};
        else {
            mappings_remain = true;
            fprintf(stderr, "LWIR: mapping cleanup failed; ownership retained: %s\n", strerror(errno));
        }
    }
    if (device->video_open) {
        if (device->video_count) {
            struct v4l2_requestbuffers buffers = {.type = type, .memory = V4L2_MEMORY_MMAP};
            video_ioctl(device->video_fd, VIDIOC_REQBUFS, &buffers);
        }
        const int descriptor = device->video_fd;
        device->video_open = false;
        device->video_streaming = false;
        device->video_fd = -1;
        if (close(descriptor)) fprintf(stderr, "LWIR: camera close reported an error: %s\n", strerror(errno));
    }
    if (!mappings_remain) *device = (struct lwir_device){0};
    errno = error;
}

/**
 * @brief Negotiate the native format, map the capture buffers and start streaming.
 * @details
 * 1. Request the native YUYV format; for the combined layout the transport
 *    height is doubled to carry the temperature plane.
 * 2. Verify the granted format exactly (video_format_valid()).
 * 3. REQBUFS for #LWIR_VIDEO_BUFFERS MMAP buffers. The driver may grant a
 *    different count; it is recorded before validation so close frees the
 *    kernel buffers even when the count is rejected.
 * 4. For each buffer: QUERYBUF, check its size against both the frame and
 *    #LWIR_MAX_FRAME_BYTES, mmap it, record the mapping immediately (so close
 *    can unmap it on any later failure), and queue it to the driver.
 * 5. STREAMON. lwir_device::video_streaming is set beforehand so close issues
 *    STREAMOFF even if STREAMON failed half-way.
 *
 * Cancellation/deadline checkpoints sit between ioctls because each one may
 * block on USB. On failure the partially built state is left for
 * lwir_device_close(), which is the single teardown path.
 * @param device Probed, locked session.
 * @param options Policy; on success width, height and layout are written back.
 * @param deadline Absolute CLOCK_MONOTONIC limit in microseconds.
 * @return True when streaming.
 * @par Errors
 * ENOTSUP thermal data required from an image-only camera, unexpected format
 * or buffer count; EPROTO inconsistent QUERYBUF; EFAULT NULL mapping;
 * EINTR/ETIMEDOUT checkpoints; other errno from ioctl/mmap.
 */
static bool video_setup(struct lwir_device *device, struct lwir_device_options *options, uint64_t deadline)
{
    const unsigned width = device->layout == LWIR_COMBINED ? 256U : 640U;
    const unsigned height = device->layout == LWIR_COMBINED ? 192U : 512U;
    if (device->layout == LWIR_YUYV_IMAGE && options->requires_thermal) { errno = ENOTSUP; return false; }
    if (!video_checkpoint(device, deadline)) return false;
    struct v4l2_format format = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .fmt.pix = {
        .width = width, .height = height * (device->layout == LWIR_COMBINED ? 2U : 1U),
        .pixelformat = V4L2_PIX_FMT_YUYV, .field = V4L2_FIELD_NONE}};
    if (video_ioctl(device->video_fd, VIDIOC_S_FMT, &format)) return false;
    if (!video_format_valid(options, device->layout, &format)) { errno = ENOTSUP; return false; }
    device->video_width = width; device->video_height = height;
    device->video_bytes = format.fmt.pix.sizeimage;
    struct v4l2_requestbuffers buffers = {.count = LWIR_VIDEO_BUFFERS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    if (video_ioctl(device->video_fd, VIDIOC_REQBUFS, &buffers)) return false;
    device->video_count = buffers.count;
    if (buffers.count < 2 || buffers.count > LWIR_VIDEO_BUFFERS) { errno = ENOTSUP; return false; }
    for (unsigned index = 0; index < buffers.count; ++index) {
        if (!video_checkpoint(device, deadline)) return false;
        struct v4l2_buffer buffer = {.type = buffers.type, .memory = buffers.memory, .index = index};
        if (video_ioctl(device->video_fd, VIDIOC_QUERYBUF, &buffer)) return false;
        if (buffer.type != buffers.type || buffer.memory != buffers.memory || buffer.index != index
            || buffer.length < device->video_bytes || buffer.length > LWIR_MAX_FRAME_BYTES) {
            errno = EPROTO; return false;
        }
        void *address = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, device->video_fd, buffer.m.offset);
        if (address == MAP_FAILED) return false;
        /* Record before the NULL check: even a NULL mapping must still be unmapped by close. */
        device->video_buffers[index] = (struct lwir_video_buffer){address, buffer.length, true};
        if (!address) { errno = EFAULT; return false; }
        if (!video_checkpoint(device, deadline)) return false;
        if (video_ioctl(device->video_fd, VIDIOC_QBUF, &buffer)) return false;
    }
    if (!video_checkpoint(device, deadline)) return false;
    enum v4l2_buf_type type = buffers.type;
    /* Set first: a STREAMON that fails may still have changed driver state that STREAMOFF must undo. */
    device->video_streaming = true;
    if (video_ioctl(device->video_fd, VIDIOC_STREAMON, &type)) return false;
    if (!video_checkpoint(device, deadline)) return false;
    options->width = width; options->height = height; options->layout = device->layout;
    fprintf(stderr, "LWIR: native V4L2 YUYV %ux%u transport, %ux%u %s\n", width, format.fmt.pix.height,
        width, height, device->layout == LWIR_COMBINED ? "image and temperature planes" : "image only");
    fprintf(stderr, "LWIR: %s\n", device->video_power_ready
        ? "USB instance already started; no frame-count startup wait"
        : "first startup for this USB instance; qualifying eight frames once");
    return true;
}

/**
 * @brief Find, lock and start exactly one supported camera within the timeout.
 * @details
 * 1. Validate options and refuse while the session still owns anything,
 *    including mappings kept after a failed munmap (EBUSY).
 * 2. Compute one absolute deadline covering discovery and setup.
 * 3. With an explicit device path, probe only that node. Otherwise probe
 *    /dev/video0..63. A second matching camera is an error (EEXIST) rather
 *    than "first wins": with two cameras attached the choice would depend on
 *    enumeration order, so the caller must disambiguate with a serial or path.
 *    Each match holds its own flock until the scan decides.
 * 4. Read the startup marker, then set up streaming.
 *
 * Every failure after a successful probe closes the session, so the caller
 * never has to clean up after a false return.
 * @return True when streaming; errno set otherwise (see lwir_device.h).
 */
bool lwir_device_open(struct lwir_device_options *options, struct lwir_device *device)
{
    if (!options || !device || !options->timeout_ms || options->timeout_ms > 120000 || options->warmup_ms >= options->timeout_ms
        || ((options->width || options->height) && !lwir_frame_bytes(options->width, options->height))
        || (options->serial && strnlen(options->serial, sizeof(device->serial)) >= sizeof(device->serial))) {
        errno = EINVAL; return false;
    }
    if (device->video_open) { errno = EBUSY; return false; }
    for (unsigned index = 0; index < LWIR_VIDEO_BUFFERS; ++index)
        if (device->video_buffers[index].mapped) { errno = EBUSY; return false; }
    device->cancelled = options->cancelled;
    uint64_t started;
    if (!video_now(&started)) return false;
    const uint64_t duration = (uint64_t)options->timeout_ms * 1000U;
    if (started > UINT64_MAX - duration) { errno = EOVERFLOW; return false; }
    const uint64_t deadline = started + duration;
    if (!video_checkpoint(device, deadline)) return false;
    if (options->device_path) {
        if (!options->device_path[0]) { errno = EINVAL; return false; }
        if (video_probe(options->device_path, options, device) != 1) return false;
    } else {
        for (unsigned index = 0; index < 64; ++index) {
            if (!video_checkpoint(device, deadline)) { lwir_device_close(device); return false; }
            char path[64];
            snprintf(path, sizeof(path), "/dev/video%u", index);
            struct lwir_device candidate = {0};
            const int result = video_probe(path, options, &candidate);
            if (result < 0) { lwir_device_close(device); return false; }
            if (result == 1) {
                /* Ambiguity: release both locks so neither camera stays held by a failed open. */
                if (device->video_open) {
                    lwir_device_close(&candidate); lwir_device_close(device);
                    errno = EEXIST; return false;
                }
                *device = candidate;
            }
        }
        if (!device->video_open) { errno = ENODEV; return false; }
    }
    if (!video_power_marker(device, false) || !video_setup(device, options, deadline)) { lwir_device_close(device); return false; }
    return true;
}

/**
 * @brief Extend the kernel's 32-bit frame sequence to a monotonic 64-bit count.
 * @details The forward distance is computed in unsigned 32-bit arithmetic,
 * so wrap-around of the kernel counter is handled naturally. A zero distance
 * (repeated number) or a distance above half the range (interpreted as going
 * backwards) is rejected as inconsistent. Dropped frames advance the 64-bit
 * count by the full distance so gaps stay visible in observations, and are
 * reported as non-continuous to the qualification logic.
 * @param device Session sequence state, updated on success.
 * @param sequence Raw v4l2_buffer::sequence.
 * @param continuous Output: true for the first frame or an exact successor.
 * @return False for a duplicate, backwards or overflowing sequence (state unchanged).
 * @note The first frame is numbered raw + 1. The reason for the offset is not
 *       documented in the code; it at least keeps the extended value non-zero.
 */
static bool video_sequence(struct lwir_device *device, uint32_t sequence, bool *continuous)
{
    *continuous = !device->video_have_sequence || sequence == device->video_last_sequence + 1U;
    if (!device->video_have_sequence) device->video_sequence = (uint64_t)sequence + 1;
    else {
        const uint32_t distance = sequence - device->video_last_sequence;
        if (!distance || distance > UINT32_MAX / 2U || device->video_sequence > UINT64_MAX - distance) return false;
        device->video_sequence += distance;
    }
    device->video_last_sequence = sequence;
    device->video_have_sequence = true;
    return true;
}

/**
 * @brief Poison the session after a fatal kernel or protocol error.
 * @details After an unexpected DQBUF/QBUF/poll failure the driver queue is
 * in an unknown state (a buffer may be lost or still owned by user space).
 * Rather than guess, every later operation fails with EIO until the caller
 * closes and reopens, which rebuilds the queue from scratch.
 * @param device Session to poison.
 * @param error errno value to report.
 * @return Always false, for direct use in return statements.
 */
static bool video_fail(struct lwir_device *device, int error)
{
    device->video_failed = true;
    errno = error;
    return false;
}

/**
 * @brief Verify session invariants before touching the stream.
 * @details Re-derives the expected geometry and frame size from the layout and
 * checks every mapping. This guards against use of a zeroed, closed, partially
 * opened or caller-corrupted session, which would otherwise lead to reads
 * outside a mapping.
 * @param device Session to check; NULL accepted.
 * @return True when streaming and consistent.
 * @par Errors
 * EINVAL not open/streaming or inconsistent; EIO poisoned by video_fail().
 */
static bool video_ready(const struct lwir_device *device)
{
    if (!device || !device->video_open || !device->video_streaming || device->video_fd < 0
        || device->video_count < 2 || device->video_count > LWIR_VIDEO_BUFFERS
        || (device->layout != LWIR_COMBINED && device->layout != LWIR_YUYV_IMAGE)) { errno = EINVAL; return false; }
    if (device->video_failed) { errno = EIO; return false; }
    const unsigned width = device->layout == LWIR_COMBINED ? 256U : 640U;
    const unsigned height = device->layout == LWIR_COMBINED ? 192U : 512U;
    const size_t bytes = lwir_frame_bytes(width, height) / (device->layout == LWIR_COMBINED ? 1U : 2U);
    if (device->video_width != width || device->video_height != height || device->video_bytes != bytes) { errno = EINVAL; return false; }
    for (unsigned index = 0; index < device->video_count; ++index) {
        const struct lwir_video_buffer *buffer = &device->video_buffers[index];
        if (!buffer->mapped || !buffer->address || buffer->address == MAP_FAILED
            || buffer->length < bytes || buffer->length > LWIR_MAX_FRAME_BYTES) { errno = EINVAL; return false; }
    }
    return true;
}

/**
 * @brief Reject caller memory that overlaps any mapped camera buffer.
 * @details A destination inside a mapping would let the copy read and write
 * the same kernel-owned memory, and the kernel may rewrite it after requeue.
 * Address arithmetic is done on uintptr_t with explicit wrap checks, so a
 * range that wraps around the address space is rejected instead of passing
 * the overlap test by overflow.
 * @param device Session whose first @c video_count mappings are checked.
 * @param destination Start of the caller range.
 * @param bytes Length of the caller range.
 * @return True when the range is well-formed and disjoint from every mapping.
 */
static bool video_destination_valid(const struct lwir_device *device, const void *destination, size_t bytes)
{
    const uintptr_t start = (uintptr_t)destination;
    if (start > UINTPTR_MAX - bytes) return false;
    for (unsigned index = 0; index < device->video_count; ++index) {
        const uintptr_t mapped = (uintptr_t)device->video_buffers[index].address;
        const size_t length = device->video_buffers[index].length;
        /* Half-open interval overlap test, after ruling out a wrapping mapping end. */
        if (mapped > UINTPTR_MAX - length || (start < mapped + length && mapped < start + bytes)) return false;
    }
    return true;
}

/**
 * @brief Test that two byte ranges do not overlap.
 * @details Compares the lengths against the gap between the start addresses
 * instead of computing end addresses, so no addition can overflow. Used to
 * keep the frame, the observation and the session struct apart: an overlap
 * would let a frame copy corrupt the session or the observation.
 * @param first Start of the first range.
 * @param first_bytes Length of the first range.
 * @param second Start of the second range.
 * @param second_bytes Length of the second range.
 * @return True when the ranges are disjoint (touching ranges are disjoint).
 */
static bool video_ranges_separate(const void *first, size_t first_bytes, const void *second, size_t second_bytes)
{
    const uintptr_t first_start = (uintptr_t)first, second_start = (uintptr_t)second;
    return first_start <= second_start ? first_bytes <= second_start - first_start : second_bytes <= first_start - second_start;
}

/**
 * @brief Return a dequeued buffer to the driver.
 * @details A fresh v4l2_buffer is used rather than the dequeued one, so the
 * caller's copy of sequence/timestamp/flags is not touched by the ioctl.
 * Failure is fatal: the buffer has left the ring and the stream would
 * silently run with fewer buffers.
 * @param device Session.
 * @param index Buffer index previously dequeued.
 * @return True on success; false after video_fail().
 */
static bool video_requeue(struct lwir_device *device, unsigned index)
{
    struct v4l2_buffer buffer = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = index};
    if (video_ioctl(device->video_fd, VIDIOC_QBUF, &buffer)) return video_fail(device, errno);
    return true;
}

/**
 * @brief Non-blocking VIDIOC_DQBUF with validation of what the kernel returned.
 * @details EAGAIN (nothing ready on the O_NONBLOCK descriptor) and EINTR after
 * the bounded retries are reported as "no frame", not errors. An index outside
 * the mapped set is fatal without requeue, since an unknown index cannot be
 * returned safely. A wrong type or memory model is requeued first, so the
 * ring stays complete, and then treated as fatal.
 * @param device Session.
 * @param buffer Output: dequeued buffer metadata, valid when @p received.
 * @param received Output: whether a buffer is now owned by the caller.
 * @return True on success (with or without a frame); false after video_fail().
 * @warning When @p received is true the caller owns the buffer and must call
 *          video_requeue() on every path.
 */
static bool video_dequeue(struct lwir_device *device, struct v4l2_buffer *buffer, bool *received)
{
    *buffer = (struct v4l2_buffer){.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    *received = false;
    if (video_ioctl(device->video_fd, VIDIOC_DQBUF, buffer)) {
        if (errno == EAGAIN || errno == EINTR) return true;
        return video_fail(device, errno);
    }
    if (buffer->index >= device->video_count || buffer->index >= LWIR_VIDEO_BUFFERS
        || !device->video_buffers[buffer->index].mapped) return video_fail(device, EPROTO);
    if (buffer->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || buffer->memory != V4L2_MEMORY_MMAP) {
        if (!video_requeue(device, buffer->index)) return false;
        return video_fail(device, EPROTO);
    }
    *received = true;
    return true;
}

/**
 * @brief Check that a MONOTONIC kernel timestamp lies within [start, now].
 * @details uvcvideo stamps buffers at start of frame. Requiring the stamp to
 * be at or after the request start guarantees the image was not exposed
 * before the trigger. A stamp after @p now is impossible on one clock and is
 * rejected as inconsistent. Buffers whose clock is not MONOTONIC cannot be
 * compared and pass here; lwir_device_acquire() compensates by draining the
 * already-queued buffers once.
 * @param buffer Dequeued buffer.
 * @param start Earliest acceptable time, CLOCK_MONOTONIC microseconds.
 * @param now Host time at dequeue, CLOCK_MONOTONIC microseconds.
 * @return True when fresh or when freshness cannot be judged.
 */
static bool video_timestamp_fresh(const struct v4l2_buffer *buffer, uint64_t start, uint64_t now)
{
    if ((buffer->flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) return true;
    if (buffer->timestamp.tv_sec < 0 || buffer->timestamp.tv_usec < 0 || buffer->timestamp.tv_usec >= 1000000
        || (uint64_t)buffer->timestamp.tv_sec > (UINT64_MAX - (uint64_t)buffer->timestamp.tv_usec) / 1000000U) return false;
    const uint64_t timestamp = (uint64_t)buffer->timestamp.tv_sec * 1000000U + (uint64_t)buffer->timestamp.tv_usec;
    return timestamp >= start && timestamp <= now;
}

/**
 * @brief Structural and content check of a dequeued frame, independent of the request.
 * @details Rejects driver-flagged errors, short or oversized payloads, and
 * stamps older than max(@p start, now - 1 s). The one-second window lets
 * frames that arrived before a request still count toward startup
 * qualification while excluding stale backlog. For the combined layout,
 * lwir_frame_capture_valid() rejects the neutral fill frames (all byte pairs
 * 00,80) the Tiny1-C emits at transitions, and an invalid thermal plane.
 * @param device Session (layout, geometry, mappings).
 * @param buffer Dequeued buffer, still owned by the caller.
 * @param start Additional lower time bound; 0 for none.
 * @param now Host time at dequeue.
 * @return True when the frame is usable for qualification.
 */
static bool video_frame_qualified(const struct lwir_device *device, const struct v4l2_buffer *buffer,
                                   uint64_t start, uint64_t now)
{
    const uint64_t recent = now > 1000000U ? now - 1000000U : 0;
    if ((buffer->flags & V4L2_BUF_FLAG_ERROR) || buffer->bytesused != device->video_bytes
        || buffer->length < buffer->bytesused || buffer->length > device->video_buffers[buffer->index].length
        || !video_timestamp_fresh(buffer, start > recent ? start : recent, now)) return false;
    return device->layout != LWIR_COMBINED || lwir_frame_capture_valid(
        device->video_buffers[buffer->index].address, buffer->bytesused, device->video_width, device->video_height);
}

/**
 * @brief Advance the eight-consecutive-good-frames startup counter.
 * @details Once eight is reached the counter latches for the rest of the USB
 * power instance: later bad frames or gaps must not impose another startup
 * wait. Before that, an invalid frame resets to zero and a sequence gap
 * restarts the count with the current frame as the first good one.
 * @param device Session counter and last-good arrival time.
 * @param valid Result of video_frame_qualified() and the sequence check.
 * @param continuous Whether the frame directly follows the previous one.
 * @param now Host arrival time of the frame.
 */
static void video_qualification(struct lwir_device *device, bool valid, bool continuous, uint64_t now)
{
    if (device->video_good_frames >= 8) return;
    if (!valid) { device->video_good_frames = 0; return; }
    if (!continuous) device->video_good_frames = 0;
    if (device->video_good_frames < 8) ++device->video_good_frames;
    device->video_good_arrival_us = now;
}

/**
 * @brief Wait for, validate and copy the first acceptable frame that started after this call.
 * @details
 * Loop structure, each iteration bounded by a 25 ms poll() slice so that
 * cancellation and the deadline are noticed promptly:
 * 1. poll() for a filled buffer; POLLERR/POLLHUP/POLLNVAL (for example an
 *    unplugged camera) poison the session.
 * 2. Dequeue one buffer and read the host clock immediately.
 * 3. Grade the frame in increasing strictness:
 *    - @c usable: structurally valid, recent, correct sequence; feeds the
 *      startup qualification counter even when it predates the request.
 *    - @c valid: additionally started at or after the request (freshness).
 *    - @c scene_valid: additionally passes the optional scene check.
 * 4. For non-MONOTONIC clocks freshness is unknowable, so the first good
 *    frame triggers a one-time drain of all already-queued buffers; the next
 *    frame is then certainly produced after the request.
 * 5. Accept when the instance is qualified (eight good frames), optional
 *    settling holds, and on first startup only, @c warmup_ms has elapsed.
 * 6. Copy what is needed and snapshot sequence/timestamp/flags, then requeue.
 *    The order matters: after QBUF the kernel may overwrite both pixels and
 *    metadata.
 * 7. Persist the startup marker once qualification and warmup are complete.
 *
 * Settling uses @p destination as the previous-image store: the image plane
 * of each scene-valid frame is copied there and the next frame is compared
 * against it, so no extra buffer is needed. The temperature plane is copied
 * only for the accepted frame.
 * @return True with a fresh frame in @p destination.
 * @par Errors
 * EINVAL arguments/aliasing; EIO poisoned session or poll error; ETIMEDOUT
 * deadline, including completion that would land after it; EINTR cancelled;
 * kernel errno from failed ioctls (session then poisoned).
 * @warning On failure @p destination may contain partial or settling data and
 *          must be ignored.
 */
bool lwir_device_acquire(struct lwir_device *device, const struct lwir_device_options *options,
                         uint8_t *destination, size_t capacity, struct lwir_device_observation *observation)
{
    if (!options || !destination || !observation) { errno = EINVAL; return false; }
    if (!video_ready(device)) return false;
    if (capacity < device->video_bytes || !video_destination_valid(device, destination, device->video_bytes)
        || !video_destination_valid(device, observation, sizeof(*observation))
        || !video_ranges_separate(destination, device->video_bytes, observation, sizeof(*observation))
        || !video_ranges_separate(destination, device->video_bytes, device, sizeof(*device))
        || !video_ranges_separate(observation, sizeof(*observation), device, sizeof(*device)) || options->width != device->video_width
        || options->height != device->video_height || !options->timeout_ms || options->timeout_ms > 120000
        || options->warmup_ms >= options->timeout_ms) { errno = EINVAL; return false; }
    uint64_t start;
    if (!video_now(&start)) return false;
    const uint64_t duration = (uint64_t)options->timeout_ms * 1000U;
    if (start > UINT64_MAX - duration) { errno = EOVERFLOW; return false; }
    const uint64_t deadline = start + duration;
    /* Only the first startup of a USB instance may wait for warmup; it also makes the non-MONOTONIC drain redundant. */
    const bool first_startup = !device->video_power_ready;
    bool request_queue_drained = first_startup;
    bool have_previous_image = false;
    unsigned skipped = 0;
    uint64_t last_arrival = start;
    const size_t image_bytes = device->layout == LWIR_COMBINED ? device->video_bytes / 2 : device->video_bytes;
    while (!video_cancelled(device)) {
        uint64_t now;
        if (!video_now(&now)) return false;
        if (now < start || now >= deadline) { errno = ETIMEDOUT; return false; }
        /* Round up so a sub-millisecond remainder still polls instead of spinning with timeout 0. */
        const uint64_t remaining = (deadline - now + 999U) / 1000U;
        struct pollfd descriptor = {.fd = device->video_fd, .events = POLLIN};
        const int ready = poll(&descriptor, 1, remaining > 25 ? 25 : (int)remaining);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) return video_fail(device, errno);
        if (!ready) continue;
        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) return video_fail(device, EIO);
        if (!(descriptor.revents & POLLIN)) continue;
        if (video_cancelled(device)) break;
        struct v4l2_buffer buffer;
        bool received;
        if (!video_dequeue(device, &buffer, &received)) return false;
        if (!received) continue;
        /* From here the buffer is owned: every path below must requeue it before returning. */
        const bool clock_read = video_now(&now);
        const int clock_error = errno;
        const bool clock_valid = clock_read && now >= start && now < deadline;
        bool continuous = false;
        const bool sequence_valid = video_sequence(device, buffer.sequence, &continuous);
        /* usable ignores the request start so pre-request frames still count toward qualification. */
        const bool usable = clock_valid && sequence_valid && video_frame_qualified(device, &buffer, 0, now);
        const bool valid = usable && video_timestamp_fresh(&buffer, start, now);
        const uint8_t *source = device->video_buffers[buffer.index].address;
        /* The scene check exists only for combined data, so requesting it on image-only rejects every frame. */
        const bool scene_valid = valid && (!options->scene_check
            || (device->layout == LWIR_COMBINED && lwir_frame_plausible(source, buffer.bytesused, options->width, options->height)));
        if (!scene_valid && skipped < UINT_MAX) ++skipped;
        const bool monotonic = (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        video_qualification(device, usable, continuous, now);
        /* Unknown clock: this frame may predate the request. Discard it and the queued backlog once,
         * bounded by the buffer count so a fast stream cannot keep this loop busy. */
        if (scene_valid && !monotonic && !request_queue_drained) {
            if (!video_requeue(device, buffer.index)) return false;
            for (unsigned index = 0; index < device->video_count; ++index) {
                bool pending;
                if (!lwir_device_discard(device, &pending)) return false;
                if (!pending) break;
            }
            request_queue_drained = true;
            continue;
        }
        bool accepted = false;
        if (!scene_valid) have_previous_image = false;
        else {
            /* destination holds the previous scene-valid image plane; compare before overwriting it.
             * A gap or >1 s pause makes that comparison meaningless, so settling restarts. */
            const bool settled = have_previous_image && continuous && now - last_arrival <= 1000000U
                && (!options->settle || lwir_frame_settled(source, destination, options->width, options->height));
            have_previous_image = true;
            const bool qualified = device->video_good_frames >= 8;
            const bool settled_now = !options->settle || settled;
            accepted = qualified && settled_now && (!first_startup || now - start >= (uint64_t)options->warmup_ms * 1000U);
            /* Bounded byte loops instead of memcpy (project policy); they still vectorize. */
            if (options->settle || accepted) for (size_t index = 0; index < image_bytes; ++index) destination[index] = source[index];
            if (accepted && device->layout == LWIR_COMBINED)
                for (size_t index = image_bytes; index < device->video_bytes; ++index) destination[index] = source[index];
        }
        last_arrival = now;
        struct lwir_device_observation selected = {0};
        /* Snapshot metadata before QBUF: the driver may clear or rewrite it once requeued. */
        if (accepted) {
            selected = (struct lwir_device_observation){.sequence = device->video_sequence, .arrival_us = now,
                .kernel_monotonic_us = monotonic ? (uint64_t)buffer.timestamp.tv_sec * 1000000U + (uint64_t)buffer.timestamp.tv_usec : 0,
                .kernel_timestamp_flags = buffer.flags & (V4L2_BUF_FLAG_TIMESTAMP_MASK | V4L2_BUF_FLAG_TSTAMP_SRC_MASK)};
        }
        if (!video_requeue(device, buffer.index)) return false;
        /* Clock failure is reported only now, after the buffer is safely back in the ring. */
        if (!clock_valid) { errno = clock_read ? ETIMEDOUT : clock_error; return false; }
        /* A marker that cannot be written is fatal rather than silently forcing a future warmup. */
        if (!device->video_power_ready && device->video_good_frames >= 8
            && now - start >= (uint64_t)options->warmup_ms * 1000U && !video_power_marker(device, true)) return video_fail(device, errno);
        if (accepted) {
            /* Copy and marker I/O take time; never hand back a result past the caller's budget. */
            uint64_t finished;
            if (!video_now(&finished) || finished < now || finished >= deadline) { errno = ETIMEDOUT; return false; }
            if (video_cancelled(device)) break;
            if (skipped) fprintf(stderr, "LWIR: skipped %u unusable/stale frames before capture; startup wait was %s\n",
                skipped, first_startup ? "initial-only" : "disabled");
            *observation = selected;
            return true;
        }
    }
    errno = EINTR;
    return false;
}

/**
 * @brief Dequeue and immediately requeue at most one pending frame.
 * @details Used to empty the queue (for example the non-MONOTONIC drain in
 * lwir_device_acquire()) and to keep the stream moving between requests.
 * The frame still feeds startup qualification with the one-second recency
 * window, and the marker is published as soon as eight good frames are seen;
 * no warmup delay applies here because there is no request start to measure from.
 * @return True on success, including when no frame was pending.
 * @par Errors
 * EINVAL bad arguments or session; EIO poisoned; EINTR cancelled; kernel or
 * clock errno (session then poisoned).
 */
bool lwir_device_discard(struct lwir_device *device, bool *received)
{
    if (!received) { errno = EINVAL; return false; }
    if (!video_ready(device)) return false;
    if (video_cancelled(device)) { errno = EINTR; return false; }
    struct v4l2_buffer buffer;
    if (!video_dequeue(device, &buffer, received)) return false;
    if (!*received) return true;
    bool continuous;
    const bool sequence_valid = video_sequence(device, buffer.sequence, &continuous);
    uint64_t now;
    const bool clock_valid = video_now(&now);
    if (!clock_valid) {
        /* Return the buffer first so the poisoned session still holds a complete ring for close. */
        const int error = errno;
        if (!video_requeue(device, buffer.index)) return false;
        return video_fail(device, error);
    }
    video_qualification(device, sequence_valid && video_frame_qualified(device, &buffer, 0, now), continuous, now);
    if (!video_requeue(device, buffer.index)) return false;
    if (!device->video_power_ready && device->video_good_frames >= 8 && !video_power_marker(device, true)) return video_fail(device, errno);
    return true;
}

/** @brief Context handed to the lwir_state.c protocol through struct lwir_state_io. */
struct video_state_transport {
    struct lwir_device *device; /**< Session supplying the cancellation flag. */
    int descriptor; /**< usbfs descriptor, open only for the duration of one operation. */
};

/**
 * @brief One Tiny1-C vendor mailbox control transfer over usbfs.
 * @details Reads use bmRequestType 0xc1 / bRequest 0x44 and writes 0x41 /
 * 0x45 (vendor type, interface recipient), with wValue 0x78 and the mailbox
 * address in wIndex. Linux usbfs permits vendor-type control requests without
 * claiming the interface, so uvcvideo is never detached and streaming
 * continues undisturbed. Length (64 bytes) and timeout (50 ms) caps keep a
 * misbehaving camera from blocking the caller for long.
 * @param context struct video_state_transport.
 * @param input True for device-to-host (read), false for host-to-device (write).
 * @param address Mailbox address placed in wIndex.
 * @param data Transfer buffer of @p length bytes.
 * @param length 1..64 bytes.
 * @param timeout 1..50 milliseconds.
 * @return Bytes transferred, or a negative code interpreted by lwir_state.c:
 *         -10 cancelled/interrupted, -7 timeout, -2 invalid argument, -1 other
 *         I/O error. These values follow the numbering of the retired libusb
 *         backend so lwir_state.c stays backend-neutral.
 */
static int video_state_transfer(void *context, bool input, uint16_t address, uint8_t *data,
                                 unsigned length, unsigned timeout)
{
    const struct video_state_transport *transport = context;
    if (video_cancelled(transport->device)) return -10;
    if (!data || !length || length > 64 || !timeout || timeout > 50) return -2;
    struct usbdevfs_ctrltransfer control = {.bRequestType = input ? 0xc1 : 0x41,
        .bRequest = input ? 0x44 : 0x45, .wValue = 0x78, .wIndex = address,
        .wLength = (uint16_t)length, .timeout = timeout, .data = data};
    const int result = ioctl(transport->descriptor, USBDEVFS_CONTROL, &control);
    if (result >= 0) return result;
    return errno == ETIMEDOUT ? -7 : errno == EINTR ? -10 : -1;
}

/**
 * @brief Clock callback for lwir_state.c.
 * @param context Unused.
 * @return CLOCK_MONOTONIC microseconds, or UINT64_MAX if the clock fails so
 *         that any deadline comparison in the protocol treats it as expired.
 */
static uint64_t video_state_now(void *context)
{
    (void)context;
    uint64_t now;
    return video_now(&now) ? now : UINT64_MAX;
}

/**
 * @brief Short cancellable delay between mailbox polls for lwir_state.c.
 * @details nanosleep() is resumed with the remaining time after signals, but
 * cancellation is checked on every interruption so a cancel request ends the
 * pause immediately. The 10 ms cap keeps cancellation latency small.
 * @param context struct video_state_transport.
 * @param milliseconds 1..10.
 * @return True when the full delay elapsed without cancellation.
 */
static bool video_state_pause(void *context, unsigned milliseconds)
{
    const struct video_state_transport *transport = context;
    if (!milliseconds || milliseconds > 10 || video_cancelled(transport->device)) return false;
    struct timespec remaining = {.tv_nsec = (long)milliseconds * 1000000L};
    while (nanosleep(&remaining, &remaining))
        if (errno != EINTR || video_cancelled(transport->device)) return false;
    return !video_cancelled(transport->device);
}

/**
 * @brief Confirm the streaming camera is still the same USB power instance.
 * @details Re-derives the identity from the open V4L2 descriptor and compares
 * power key, bus and address with those recorded at open. If the camera
 * re-enumerated meanwhile, the stored bus/address may now name a different
 * device, and vendor writes must never reach it.
 * @param device Open session.
 * @return True when unchanged and combined-layout.
 * @par Errors
 * ENODEV identity changed or unavailable; errno from fstat().
 */
static bool video_state_identity(struct lwir_device *device)
{
    struct stat node;
    enum lwir_frame_layout layout;
    char serial[64], key[160], port[64];
    unsigned bus, address;
    if (fstat(device->video_fd, &node)) return false;
    if (!video_identity(&node, &layout, serial, key, port, &bus, &address)) { errno = ENODEV; return false; }
    if (layout != LWIR_COMBINED || strcmp(key, device->video_power_key)
        || bus != device->video_usb_bus || address != device->video_usb_address) { errno = ENODEV; return false; }
    return true;
}

/**
 * @brief Open the usbfs node of the streaming Tiny1-C for one state/control operation.
 * @details
 * 1. Require a healthy combined-layout session and verify identity.
 * 2. Open /dev/bus/usb/BBB/DDD.
 * 3. Check the opened node is the expected usbfs character device: major 189,
 *    minor (bus - 1) * 128 + address - 1.
 * 4. Verify identity again after open, closing the window in which the camera
 *    could re-enumerate and another device take over the address.
 *
 * The node is opened per operation and closed on every path by the caller,
 * so no usbfs handle outlives a single request.
 * @param device Open, healthy session.
 * @return usbfs descriptor, or -1 with errno set.
 * @par Errors
 * EINVAL/EIO session not ready; ENOTSUP image-only camera; EINTR cancelled;
 * ENODEV identity mismatch; EOVERFLOW path too long; errno from open/fstat.
 */
static int video_state_open(struct lwir_device *device)
{
    if (!video_ready(device)) return -1;
    if (device->layout != LWIR_COMBINED) { errno = ENOTSUP; return -1; }
    if (video_cancelled(device)) { errno = EINTR; return -1; }
    if (!video_state_identity(device)) return -1;
    char path[64];
    const int length = snprintf(path, sizeof(path), "/dev/bus/usb/%03u/%03u", device->video_usb_bus, device->video_usb_address);
    if (length <= 0 || (size_t)length >= sizeof(path)) { errno = EOVERFLOW; return -1; }
    const int descriptor = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor < 0) return -1;
    struct stat node;
    const dev_t expected = makedev(189, (device->video_usb_bus - 1U) * 128U + device->video_usb_address - 1U);
    int error = 0;
    if (fstat(descriptor, &node)) error = errno;
    else if (!S_ISCHR(node.st_mode) || node.st_rdev != expected) error = ENODEV;
    else if (!video_state_identity(device)) error = errno;
    if (error) { close(descriptor); errno = error; return -1; }
    return descriptor;
}

/**
 * @brief Shared implementation of control write and read-back.
 * @details Validates the request, opens usbfs, runs the lwir_state.c
 * mailbox protocol once, and closes usbfs. The reply status is mapped to
 * errno for callers that only check errno. A close error turns the result
 * into a failure even though a write may already have been submitted;
 * nothing is replayed, so the caller should read back instead of retrying.
 * @param device Open combined-layout session.
 * @param request Control kind and value.
 * @param result Output reply; reset to LWIR_STATE_INVALID first.
 * @param read_only True for a read; FFC is an action and has no read form.
 * @return True on success.
 * @par Errors
 * EINVAL invalid request; errno from video_state_open(); EINTR cancelled;
 * ETIMEDOUT camera timeout; EIO any other protocol failure.
 */
static bool video_control(struct lwir_device *device, const struct lwir_control_request *request,
                            struct lwir_control_result *result, bool read_only)
{
    if (!result) { errno = EINVAL; return false; }
    *result = (struct lwir_control_result){.reply = {.status = LWIR_STATE_INVALID}};
    if (!lwir_control_valid(request) || (read_only && request->kind == LWIR_CONTROL_FFC)) { errno = EINVAL; return false; }
    const int descriptor = video_state_open(device);
    if (descriptor < 0) {
        result->reply.status = errno == EINTR ? LWIR_STATE_CANCELLED : errno == ENOTSUP ? LWIR_STATE_UNSUPPORTED : LWIR_STATE_IO_ERROR;
        result->reply.transport_error = -errno;
        return false;
    }
    struct video_state_transport transport = {.device = device, .descriptor = descriptor};
    const struct lwir_state_io io = {&transport, video_state_transfer, video_state_now, video_state_pause};
    bool success = read_only ? lwir_state_control_read(&io, request->kind, result) : lwir_state_control(&io, request, result);
    if (close(descriptor)) success = false;
    if (!success) errno = result->reply.status == LWIR_STATE_CANCELLED ? EINTR :
        result->reply.status == LWIR_STATE_TIMEOUT ? ETIMEDOUT : EIO;
    return success;
}

/**
 * @brief Public control write; see lwir_device.h.
 * @return True when acknowledged.
 */
bool lwir_device_control(struct lwir_device *device, const struct lwir_control_request *request,
                          struct lwir_control_result *result)
{
    return video_control(device, request, result, false);
}

/**
 * @brief Public control read; see lwir_device.h.
 * @details Builds a request only so the shared validation path can be reused.
 * @return True when the value was read.
 */
bool lwir_device_control_read(struct lwir_device *device, enum lwir_control_kind kind,
                               struct lwir_control_result *result)
{
    /* Placeholder value only; presumably needed to satisfy lwir_control_valid() for emissivity. Nothing is written. */
    const struct lwir_control_request request = {.kind = kind, .value = kind == LWIR_CONTROL_EMISSIVITY ? 1 : 0};
    return video_control(device, &request, result, true);
}

/**
 * @brief Best-effort read-only snapshot of Tiny1-C state; never writes settings.
 * @details All fields start as skipped and the first as an I/O error, so a
 * failure to open usbfs is reported on a field instead of looking like an
 * empty but successful read. lwir_state_read() overwrites what it reaches.
 * The usbfs close result is ignored because nothing was written. Image-only
 * sessions (or NULL) return a zero, not-requested snapshot without any USB
 * traffic.
 * @note errno is preserved; outcomes are reported only through @p state.
 * @note The snapshot is taken after the frame and is not frame-atomic.
 */
void lwir_device_read_state(struct lwir_device *device, struct lwir_device_state *state)
{
    if (!state) return;
    const int error = errno;
    *state = (struct lwir_device_state){0};
    if (device && device->layout == LWIR_COMBINED) {
        state->requested = true;
        video_now(&state->started_us);
        state->finished_us = state->started_us;
        for (unsigned index = 0; index < LWIR_STATE_FIELDS; ++index) state->values[index].status = LWIR_STATE_SKIPPED;
        state->values[0].status = LWIR_STATE_IO_ERROR;
        struct video_state_transport transport = {.device = device, .descriptor = video_state_open(device)};
        if (transport.descriptor >= 0) {
            const struct lwir_state_io io = {&transport, video_state_transfer, video_state_now, video_state_pause};
            lwir_state_read(&io, state);
            close(transport.descriptor);
        } else {
            state->values[0].transport_error = -errno;
        }
        if (video_cancelled(device)) state->values[0].status = LWIR_STATE_CANCELLED;
    }
    errno = error;
}