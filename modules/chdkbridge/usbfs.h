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
 * @file usbfs.h
 * @brief Minimal header-only Linux usbfs access for one CHDKbridge camera.
 * @details Discovers devices through sysfs, parses the active configuration,
 * owns one interface and performs synchronous bulk transfers through
 * /dev/bus/usb ioctls. The wrapper allocates no heap storage and creates
 * no threads.
 * @warning Serialize all access to a handle, including close. A bulk call
 * owns its submitted URB until completion or descriptor teardown; another
 * caller must not reap it or close and reuse the descriptor.
 */
#ifndef CHDKBRIDGE_USBFS_H
#define CHDKBRIDGE_USBFS_H

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/usbdevice_fs.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <time.h>
#include <unistd.h>

#define USBFS_SYSFS_ROOT "/sys/bus/usb/devices"
#define USBFS_DEVICE_ROOT "/dev/bus/usb"

/** @brief Result codes; negative values are errors, as in libusb. */
enum {
  USBFS_SUCCESS = 0, USBFS_ERROR_IO = -1, USBFS_ERROR_INVALID_PARAM = -2,
  USBFS_ERROR_ACCESS = -3, USBFS_ERROR_NO_DEVICE = -4, USBFS_ERROR_NOT_FOUND = -5,
  USBFS_ERROR_BUSY = -6, USBFS_ERROR_TIMEOUT = -7, USBFS_ERROR_OVERFLOW = -8,
  USBFS_ERROR_PIPE = -9, USBFS_ERROR_INTERRUPTED = -10, USBFS_ERROR_NO_MEM = -11,
  USBFS_ERROR_OTHER = -99
};

enum {
  USBFS_ENDPOINT_IN = 0x80, USBFS_TRANSFER_TYPE_MASK = 0x03,
  USBFS_TRANSFER_TYPE_BULK = 2, USBFS_TRANSFER_TYPE_INTERRUPT = 3,
  USBFS_MAX_ENDPOINTS = 30, USBFS_MAX_PORTS = 7,
  USBFS_DESCRIPTOR_CAPACITY = 16384,
  USBFS_LEGACY_TRANSFER = 16384, /**< Per-URB limit without NO_PACKET_SIZE_LIM. */
  USBFS_DISCARD_MS = 1000, /**< Grace for a cancelled URB before the fd is dropped. */
  USBFS_DEVICE_MAJOR = 189
};

/** @brief One enumerated USB device; a value copy, valid after enumeration ends. */
struct usbfs_device {
  char name[32];            /**< sysfs name, e.g. "1-5.1". */
  unsigned bus, address;
  uint16_t vendor, product;
  uint8_t ports[USBFS_MAX_PORTS];
  int depth;                /**< Port count, or -1 when deeper than ports[]. */
};

struct usbfs_endpoint {
  uint8_t bEndpointAddress, bmAttributes;
  uint16_t wMaxPacketSize;
};

struct usbfs_interface {
  uint8_t bInterfaceNumber, bAlternateSetting, bInterfaceClass;
  uint8_t bInterfaceSubClass, bInterfaceProtocol, bNumEndpoints;
  struct usbfs_endpoint endpoint[USBFS_MAX_ENDPOINTS];
};

struct usbfs_devices {
  DIR *directory;
};

/** @brief Open usbfs file; fd is -1 when closed or dropped after a lost URB. */
struct usbfs_handle {
  int fd;
  uint32_t capabilities;
  const volatile sig_atomic_t *cancel; /**< Optional; nonzero discards a pending bulk URB early. */
};

/** @brief Get the diagnostic name of a usbfs result.
 * @param error Result from this wrapper.
 * @return Borrowed static name, with OTHER for unknown values.
 * @details Keep the diagnostic vocabulary stable independently of errno. */
static inline const char *usbfs_error_name(int error)
{
  switch (error) {
    case USBFS_SUCCESS: return "USB_SUCCESS";
    case USBFS_ERROR_IO: return "USB_ERROR_IO";
    case USBFS_ERROR_INVALID_PARAM: return "USB_ERROR_INVALID_PARAM";
    case USBFS_ERROR_ACCESS: return "USB_ERROR_ACCESS";
    case USBFS_ERROR_NO_DEVICE: return "USB_ERROR_NO_DEVICE";
    case USBFS_ERROR_NOT_FOUND: return "USB_ERROR_NOT_FOUND";
    case USBFS_ERROR_BUSY: return "USB_ERROR_BUSY";
    case USBFS_ERROR_TIMEOUT: return "USB_ERROR_TIMEOUT";
    case USBFS_ERROR_OVERFLOW: return "USB_ERROR_OVERFLOW";
    case USBFS_ERROR_PIPE: return "USB_ERROR_PIPE";
    case USBFS_ERROR_INTERRUPTED: return "USB_ERROR_INTERRUPTED";
    case USBFS_ERROR_NO_MEM: return "USB_ERROR_NO_MEM";
    default: return "USB_ERROR_OTHER";
  }
}

/** @brief True when sysfs device listing and usbfs nodes are reachable. */
static inline bool usbfs_available(void)
{
  return access(USBFS_SYSFS_ROOT, R_OK | X_OK) == 0 && access(USBFS_DEVICE_ROOT, R_OK | X_OK) == 0;
}

/** @brief Read one bounded unsigned numeric sysfs attribute.
 * @param device Sysfs device directory name.
 * @param attribute Attribute basename.
 * @param base Number base, 10 or 16 for supported callers.
 * @param maximum Largest acceptable value.
 * @param[out] value Result written only on success.
 * @return True if the attribute could be read and parsed in range.
 * @details Allow the newline used by sysfs, but reject signs and
 * nonnumeric prefixes before conversion. */
static inline bool usbfs_read_number(const char *device, const char *attribute, int base,
                                     unsigned long maximum, unsigned long *value)
{
  char path[128], text[32];
  int length = snprintf(path, sizeof(path), USBFS_SYSFS_ROOT "/%s/%s", device, attribute);
  if (length < 0 || (size_t)length >= sizeof(path)) return false;
  int fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  ssize_t count;
  do count = read(fd, text, sizeof(text) - 1); while (count < 0 && errno == EINTR);
  close(fd);
  if (count <= 0) return false;
  text[count] = '\0';
  bool digit = (text[0] >= '0' && text[0] <= '9') ||
               (base == 16 && ((text[0] >= 'a' && text[0] <= 'f') || (text[0] >= 'A' && text[0] <= 'F')));
  if (!digit) return false;
  char *end;
  errno = 0;
  unsigned long parsed = strtoul(text, &end, base);
  if (errno || (*end && *end != '\n') || parsed > maximum) return false;
  *value = parsed;
  return true;
}

/** @brief Parse "BUS-PORT[.PORT...]"; rejects root hubs and interface entries. */
static inline bool usbfs_parse_name(const char *name, struct usbfs_device *device)
{
  int length = snprintf(device->name, sizeof(device->name), "%s", name);
  if (length < 0 || (size_t)length >= sizeof(device->name) || name[0] < '1' || name[0] > '9') return false;
  char *end;
  unsigned long bus = strtoul(name, &end, 10);
  if (*end != '-' || bus > 255) return false;
  device->bus = (unsigned)bus;
  unsigned depth = 0;
  for (const char *cursor = end + 1;; cursor = end + 1) {
    if (*cursor < '1' || *cursor > '9') return false;
    unsigned long port = strtoul(cursor, &end, 10);
    if (port > 255 || depth > 255) return false;
    if (depth < USBFS_MAX_PORTS) device->ports[depth] = (uint8_t)port;
    ++depth;
    if (!*end) break;
    if (*end != '.') return false;
  }
  device->depth = depth <= USBFS_MAX_PORTS ? (int)depth : -1;
  return true;
}

/** @brief Start a streaming sysfs enumeration.
 * @param[out] devices Iterator receiving the directory stream.
 * @return USBFS_SUCCESS or a negative access/I/O error.
 * @details Scan at iteration time rather than allocating a device list. */
static inline int usbfs_devices_open(struct usbfs_devices *devices)
{
  devices->directory = opendir(USBFS_SYSFS_ROOT);
  if (devices->directory) return USBFS_SUCCESS;
  return errno == EACCES ? USBFS_ERROR_ACCESS : USBFS_ERROR_IO;
}

/** @return 1 with a device, 0 at the end, or a negative error. Vanishing devices are skipped. */
static inline int usbfs_devices_next(struct usbfs_devices *devices, struct usbfs_device *device)
{
  for (;;) {
    errno = 0;
    struct dirent *entry = readdir(devices->directory);
    if (!entry) return errno ? USBFS_ERROR_IO : 0;
    struct usbfs_device candidate = {0};
    unsigned long bus, address, vendor, product;
    if (!usbfs_parse_name(entry->d_name, &candidate) ||
        !usbfs_read_number(candidate.name, "busnum", 10, 255, &bus) ||
        !usbfs_read_number(candidate.name, "devnum", 10, 127, &address) ||
        !usbfs_read_number(candidate.name, "idVendor", 16, 0xffff, &vendor) ||
        !usbfs_read_number(candidate.name, "idProduct", 16, 0xffff, &product) ||
        bus != candidate.bus || !address) continue;
    candidate.address = (unsigned)address;
    candidate.vendor = (uint16_t)vendor;
    candidate.product = (uint16_t)product;
    *device = candidate;
    return 1;
  }
}

/** @brief Release the sysfs enumeration stream.
 * @param[in,out] devices Initialized iterator.
 * @details Clear the stream pointer so repeated cleanup is safe. */
static inline void usbfs_devices_close(struct usbfs_devices *devices)
{
  if (devices->directory) closedir(devices->directory);
  devices->directory = NULL;
}

/**
 * @brief Locate configuration @p value in raw sysfs descriptors.
 * @details sysfs stores the device descriptor followed by every configuration
 * in bus (little-endian) order; a short final configuration is clamped.
 */
static inline int usbfs_select_config(const uint8_t *descriptors, size_t length, unsigned value,
                                      const uint8_t **config, size_t *config_length)
{
  if (length < 18 || descriptors[0] != 18 || descriptors[1] != 1) return USBFS_ERROR_IO;
  for (size_t offset = 18; length - offset >= 9;) {
    const uint8_t *header = descriptors + offset;
    size_t total = (size_t)header[2] | (size_t)header[3] << 8;
    if (header[0] < 9 || header[1] != 2 || total < header[0]) return USBFS_ERROR_IO;
    if (total > length - offset) total = length - offset;
    if (header[5] == value) {
      *config = header;
      *config_length = total;
      return USBFS_SUCCESS;
    }
    offset += total;
  }
  return USBFS_ERROR_NOT_FOUND;
}

/** @brief Read the active configuration into @p buffer; @p config points inside it. */
static inline int usbfs_active_config(const struct usbfs_device *device, uint8_t *buffer, size_t capacity,
                                      const uint8_t **config, size_t *config_length)
{
  unsigned long value;
  if (!usbfs_read_number(device->name, "bConfigurationValue", 10, 255, &value) || !value)
    return USBFS_ERROR_NOT_FOUND;
  char path[128];
  int written = snprintf(path, sizeof(path), USBFS_SYSFS_ROOT "/%s/descriptors", device->name);
  if (written < 0 || (size_t)written >= sizeof(path)) return USBFS_ERROR_INVALID_PARAM;
  int fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return errno == ENOENT ? USBFS_ERROR_NO_DEVICE : USBFS_ERROR_IO;
  size_t used = 0;
  int result = USBFS_SUCCESS;
  for (;;) {
    uint8_t probe;
    uint8_t *target = used < capacity ? buffer + used : &probe;
    ssize_t count = read(fd, target, used < capacity ? capacity - used : 1);
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) result = USBFS_ERROR_IO;
    else if (count > 0 && used >= capacity) result = USBFS_ERROR_OVERFLOW;
    if (count <= 0 || result) break;
    used += (size_t)count;
  }
  close(fd);
  return result ? result : usbfs_select_config(buffer, used, (unsigned)value, config, config_length);
}

/**
 * @brief Return the next complete interface/alternate setting in a configuration.
 * @param config Borrowed descriptor block for the active configuration.
 * @param length Readable bytes in @p config.
 * @param offset Start with 0; advanced past the returned interface.
 * @param[out] interface Parsed interface; use only when the function returns true.
 * @return True when a complete interface was found.
 * @details Interfaces whose endpoint descriptors disagree with bNumEndpoints
 * are skipped. Parsing stops at the first malformed descriptor header.
 */
static inline bool usbfs_next_interface(const uint8_t *config, size_t length, size_t *offset,
                                        struct usbfs_interface *interface)
{
  size_t cursor = *offset ? *offset : (length ? config[0] : 0);
  bool open = false;
  unsigned endpoints = 0;
  while (cursor < length && length - cursor >= 2) {
    const uint8_t *descriptor = config + cursor;
    if (descriptor[0] < 2 || descriptor[0] > length - cursor) break;
    if (descriptor[1] == 4 && descriptor[0] >= 9) {
      if (open && endpoints == interface->bNumEndpoints) {
        *offset = cursor;
        return true;
      }
      *interface = (struct usbfs_interface){
        .bInterfaceNumber = descriptor[2], .bAlternateSetting = descriptor[3],
        .bNumEndpoints = descriptor[4], .bInterfaceClass = descriptor[5],
        .bInterfaceSubClass = descriptor[6], .bInterfaceProtocol = descriptor[7]};
      open = descriptor[4] <= USBFS_MAX_ENDPOINTS;
      endpoints = 0;
    } else if (descriptor[1] == 5 && descriptor[0] >= 7 && open) {
      if (endpoints == interface->bNumEndpoints) {
        open = false;
      } else {
        interface->endpoint[endpoints++] = (struct usbfs_endpoint){
          .bEndpointAddress = descriptor[2], .bmAttributes = descriptor[3],
          .wMaxPacketSize = (uint16_t)(descriptor[4] | descriptor[5] << 8)};
      }
    }
    cursor += descriptor[0];
  }
  *offset = length ? length : 1;
  return open && endpoints == interface->bNumEndpoints;
}

/** @brief Compare a previously discovered USB identity with current sysfs.
 * @param device Device snapshot with sysfs name, bus, address and IDs.
 * @return True if the checked attributes still match.
 * @details Recheck after opening because disconnects and re-enumeration
 * can invalidate a previously discovered bus address. */
static inline bool usbfs_same_device(const struct usbfs_device *device)
{
  unsigned long bus, address, vendor, product;
  return usbfs_read_number(device->name, "busnum", 10, 255, &bus) && bus == device->bus &&
         usbfs_read_number(device->name, "devnum", 10, 127, &address) && address == device->address &&
         usbfs_read_number(device->name, "idVendor", 16, 0xffff, &vendor) && vendor == device->vendor &&
         usbfs_read_number(device->name, "idProduct", 16, 0xffff, &product) && product == device->product;
}

/** @brief Open the device node, rejecting a node now owned by a different device. */
static inline int usbfs_open(const struct usbfs_device *device, struct usbfs_handle *handle)
{
  handle->fd = -1;
  handle->capabilities = 0;
  if (!device->bus || device->bus > 255 || !device->address || device->address > 127)
    return USBFS_ERROR_INVALID_PARAM;
  char path[64];
  snprintf(path, sizeof(path), USBFS_DEVICE_ROOT "/%03u/%03u", device->bus, device->address);
  int fd;
  do fd = open(path, O_RDWR | O_CLOEXEC); while (fd < 0 && errno == EINTR);
  if (fd < 0) {
    if (errno == EACCES || errno == EPERM) return USBFS_ERROR_ACCESS;
    return errno == ENOENT || errno == ENODEV || errno == ENXIO ? USBFS_ERROR_NO_DEVICE : USBFS_ERROR_IO;
  }
  struct stat status;
  if (fstat(fd, &status) != 0 || !S_ISCHR(status.st_mode) || major(status.st_rdev) != USBFS_DEVICE_MAJOR ||
      minor(status.st_rdev) != (device->bus - 1) * 128 + device->address - 1 || !usbfs_same_device(device)) {
    close(fd);
    return USBFS_ERROR_NO_DEVICE;
  }
  uint32_t capabilities = 0;
  if (ioctl(fd, USBDEVFS_GET_CAPABILITIES, &capabilities) != 0) capabilities = 0;
  handle->fd = fd;
  handle->capabilities = capabilities;
  return USBFS_SUCCESS;
}

/** @brief Close the node; the kernel releases claimed interfaces and kills pending URBs. */
static inline void usbfs_close(struct usbfs_handle *handle)
{
  if (handle->fd >= 0) close(handle->fd);
  handle->fd = -1;
}

/** @brief Run a synchronous interface ioctl and translate its result.
 * @param handle Serialized open device handle.
 * @param request Linux usbfs ioctl number.
 * @param argument Request-specific argument storage.
 * @return USBFS_SUCCESS or a negative wrapper error.
 * @details Retry interruptions, preserving the distinction between busy,
 * disconnected and inaccessible devices for the calling policy. */
static inline int usbfs_interface_ioctl(struct usbfs_handle *handle, unsigned long request, void *argument)
{
  if (handle->fd < 0) return USBFS_ERROR_NO_DEVICE;
  int result;
  do result = ioctl(handle->fd, request, argument); while (result < 0 && errno == EINTR);
  if (result == 0) return USBFS_SUCCESS;
  switch (errno) {
    case EBUSY: return USBFS_ERROR_BUSY;
    case ENOENT: case EINVAL: return USBFS_ERROR_NOT_FOUND;
    case ENODEV: return USBFS_ERROR_NO_DEVICE;
    case EACCES: case EPERM: return USBFS_ERROR_ACCESS;
    default: return USBFS_ERROR_OTHER;
  }
}

/** @brief Claim without detaching another owner; EBUSY is reported to the caller. */
static inline int usbfs_claim_interface(struct usbfs_handle *handle, int interface)
{
  unsigned number = (unsigned)interface;
  return interface < 0 ? USBFS_ERROR_INVALID_PARAM : usbfs_interface_ioctl(handle, USBDEVFS_CLAIMINTERFACE, &number);
}

/** @brief Release the numbered interface on an open device.
 * @param handle Serialized device handle.
 * @param interface Nonnegative USB interface number.
 * @return USBFS_SUCCESS or a negative wrapper error.
 * @details Release the claim without closing the device descriptor. */
static inline int usbfs_release_interface(struct usbfs_handle *handle, int interface)
{
  unsigned number = (unsigned)interface;
  return interface < 0 ? USBFS_ERROR_INVALID_PARAM : usbfs_interface_ioctl(handle, USBDEVFS_RELEASEINTERFACE, &number);
}

/** @brief Select an alternate setting for an interface.
 * @param handle Serialized device handle.
 * @param interface Nonnegative USB interface number.
 * @param alternate Nonnegative alternate-setting number.
 * @return USBFS_SUCCESS or a negative wrapper error.
 * @details The caller must pair the setting with its matching endpoint descriptors. */
static inline int usbfs_set_alternate(struct usbfs_handle *handle, int interface, int alternate)
{
  if (interface < 0 || alternate < 0) return USBFS_ERROR_INVALID_PARAM;
  struct usbdevfs_setinterface setting = {.interface = (unsigned)interface, .altsetting = (unsigned)alternate};
  return usbfs_interface_ioctl(handle, USBDEVFS_SETINTERFACE, &setting);
}

/**
 * @brief Synchronous control transfer on endpoint zero.
 * @param handle Serialized device handle.
 * @param request_type USB bmRequestType, including direction.
 * @param request USB bRequest.
 * @param value USB wValue.
 * @param index USB wIndex.
 * @param[in,out] data Caller buffer; may be NULL only for a zero-length transfer.
 * @param length Size of the data stage in bytes.
 * @param timeout Nonzero kernel transfer timeout in milliseconds.
 * @return Bytes transferred (>= 0) or a negative USBFS error.
 * @details A short successful transfer can be meaningful; callers validate
 * the returned length against the particular request's contract.
 */
static inline int usbfs_control_transfer(struct usbfs_handle *handle, uint8_t request_type, uint8_t request,
                                         uint16_t value, uint16_t index, unsigned char *data,
                                         uint16_t length, unsigned timeout)
{
  if (!handle || handle->fd < 0) return USBFS_ERROR_NO_DEVICE;
  if ((!data && length) || !timeout) return USBFS_ERROR_INVALID_PARAM;
  struct usbdevfs_ctrltransfer transfer = {.bRequestType = request_type, .bRequest = request,
    .wValue = value, .wIndex = index, .wLength = length, .timeout = timeout, .data = data};
  int result;
  do result = ioctl(handle->fd, USBDEVFS_CONTROL, &transfer); while (result < 0 && errno == EINTR);
  if (result >= 0) return result > length ? USBFS_ERROR_IO : result;
  switch (errno) {
    case ENODEV: case ESHUTDOWN: return USBFS_ERROR_NO_DEVICE;
    case ETIMEDOUT: return USBFS_ERROR_TIMEOUT;
    case EPIPE: return USBFS_ERROR_PIPE;
    case EACCES: case EPERM: return USBFS_ERROR_ACCESS;
    default: return USBFS_ERROR_IO;
  }
}

/** @brief Clear a halted endpoint of a claimed interface; data toggles are reset by the kernel. */
static inline int usbfs_clear_halt(struct usbfs_handle *handle, unsigned char endpoint)
{
  unsigned number = endpoint;
  return usbfs_interface_ioctl(handle, USBDEVFS_CLEAR_HALT, &number);
}

/** @brief Read the clock used for bulk transfer deadlines.
 * @return Monotonic milliseconds, or zero on failure.
 * @details Keep a single time base across submit, wait and discard. */
static inline uint64_t usbfs_now_ms(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}

/** @brief Abandon the fd; closing it is the only way the kernel guarantees the URB is gone. */
static inline int usbfs_drop(struct usbfs_handle *handle, int result)
{
  usbfs_close(handle);
  return result;
}

/**
 * @brief Submit one bulk URB and wait until it is reaped or @p deadline passes.
 * @details On expiry the URB is discarded and reaped, preserving any bytes
 * already received. Data reaches @p data only when the URB is reaped, so a URB
 * that cannot be reaped within the discard grace is made harmless by closing.
 */
static inline int usbfs_bulk_urb(struct usbfs_handle *handle, unsigned char endpoint, unsigned char *data,
                                 int length, int *actual, uint64_t deadline)
{
  struct usbdevfs_urb urb = {.type = USBDEVFS_URB_TYPE_BULK, .endpoint = endpoint,
                             .buffer = data, .buffer_length = length};
  *actual = 0;
  int result;
  do result = ioctl(handle->fd, USBDEVFS_SUBMITURB, &urb); while (result < 0 && errno == EINTR);
  if (result < 0) {
    if (errno == ENODEV || errno == ESHUTDOWN) return USBFS_ERROR_NO_DEVICE;
    if (errno == ENOMEM) return USBFS_ERROR_NO_MEM;
    return errno == EPIPE ? USBFS_ERROR_PIPE : USBFS_ERROR_IO;
  }
  bool discarded = false, cancelled = false;
  uint64_t limit = deadline;
  for (;;) {
    void *reaped = NULL;
    if (ioctl(handle->fd, USBDEVFS_REAPURBNDELAY, &reaped) == 0) {
      if (reaped != &urb) return usbfs_drop(handle, USBFS_ERROR_IO);
      break;
    }
    int error = errno;
    if (error == EINTR) continue;
    if (error != EAGAIN) return usbfs_drop(handle, error == ENODEV ? USBFS_ERROR_NO_DEVICE : USBFS_ERROR_IO);
    uint64_t now = usbfs_now_ms();
    if (!now) return usbfs_drop(handle, USBFS_ERROR_IO);
    if (!discarded && handle->cancel && *handle->cancel) {
      (void)ioctl(handle->fd, USBDEVFS_DISCARDURB, &urb);
      discarded = cancelled = true;
      limit = now + USBFS_DISCARD_MS;
      continue;
    }
    if (now >= limit) {
      if (discarded) return usbfs_drop(handle, cancelled ? USBFS_ERROR_INTERRUPTED : USBFS_ERROR_TIMEOUT);
      (void)ioctl(handle->fd, USBDEVFS_DISCARDURB, &urb);
      discarded = true;
      limit = now + USBFS_DISCARD_MS;
      continue;
    }
    struct pollfd descriptor = {.fd = handle->fd, .events = POLLOUT};
    uint64_t wait = limit - now;
    if (poll(&descriptor, 1, wait > 1000 ? 1000 : (int)wait) < 0 && errno != EINTR)
      return usbfs_drop(handle, USBFS_ERROR_IO);
  }
  if (urb.actual_length < 0 || urb.actual_length > length) return USBFS_ERROR_IO;
  *actual = urb.actual_length;
  switch (urb.status) {
    case 0: case -EREMOTEIO: return USBFS_SUCCESS;
    case -ENOENT: case -ECONNRESET:
      return cancelled ? USBFS_ERROR_INTERRUPTED : discarded ? USBFS_ERROR_TIMEOUT : USBFS_ERROR_INTERRUPTED;
    case -ENODEV: case -ESHUTDOWN: return USBFS_ERROR_NO_DEVICE;
    case -EPIPE: return USBFS_ERROR_PIPE;
    case -EOVERFLOW: return USBFS_ERROR_OVERFLOW;
    default: return USBFS_ERROR_IO;
  }
}

/**
 * @brief Synchronous bulk transfer with libusb_bulk_transfer() semantics.
 * @param handle Serialized device handle holding the interface claim.
 * @param endpoint Bulk endpoint address including the direction bit.
 * @param[in,out] data Buffer for @p length bytes.
 * @param length Requested bytes, nonnegative.
 * @param[out] transferred Required byte count; may be nonzero on failure.
 * @param timeout Nonzero total timeout in milliseconds.
 * @return USBFS_SUCCESS or a negative wrapper error.
 * @details IN transfers are capped at one URB; OUT transfers are split only on
 * kernels without NO_PACKET_SIZE_LIM, sharing one deadline.
 */
static inline int usbfs_bulk_transfer(struct usbfs_handle *handle, unsigned char endpoint, unsigned char *data,
                                      int length, int *transferred, unsigned timeout)
{
  *transferred = 0;
  if (!handle || handle->fd < 0) return USBFS_ERROR_NO_DEVICE;
  if (length < 0 || (!data && length) || !timeout) return USBFS_ERROR_INVALID_PARAM;
  uint64_t now = usbfs_now_ms();
  if (!now) return USBFS_ERROR_IO;
  uint64_t deadline = now + timeout;
  int limit = handle->capabilities & USBDEVFS_CAP_NO_PACKET_SIZE_LIM ? INT_MAX : USBFS_LEGACY_TRANSFER;
  if (endpoint & USBFS_ENDPOINT_IN)
    return usbfs_bulk_urb(handle, endpoint, data, length < limit ? length : limit, transferred, deadline);
  do {
    int chunk = length - *transferred < limit ? length - *transferred : limit;
    int actual;
    int result = usbfs_bulk_urb(handle, endpoint, data + *transferred, chunk, &actual, deadline);
    *transferred += actual;
    if (result || actual != chunk) return result;
  } while (*transferred < length);
  return USBFS_SUCCESS;
}

#endif
