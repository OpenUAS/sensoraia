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
 * @file exifite.c
 * @brief Allocation-free EXIF parsing, serialization and bounded JPEG streaming.
 * @details The implementation keeps three jobs separate: validate borrowed TIFF
 * bytes, collect caller-owned values, then stream JPEG markers around a serialized
 * APP1 payload. That separation lets us check each boundary without allocating
 * image-sized buffers or interpreting manufacturer-specific pointer structures.
 * @see exifite.h for public contracts and storage ownership.
 */

#include "exifite.h"

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/** @brief Standard primary-IFD link to the camera/EXIF directory. */
#define EXIF_CAMERA_POINTER 0x8769U
/** @brief Standard primary-IFD link to the GPS directory. */
#define EXIF_GPS_POINTER 0x8825U
/** @brief Bounds validation work and its stack-based duplicate-tag table.
 * @details Input may contain more fields than our writer, but cannot demand
 * unbounded memory or quadratic validation work from the embedded board. */
#define EXIF_MAX_INPUT_ENTRIES 256U

/** @brief Fixed TIFF layout widths, expressed separately from JPEG framing. */
#define TIFF_HEADER_SIZE 8U
/** @brief Bytes occupied by one TIFF directory entry. */
#define TIFF_ENTRY_SIZE 12U
/** @brief Maximum TIFF value length stored directly in a directory entry. */
#define TIFF_INLINE_SIZE 4U
/** @brief Bytes occupied by a TIFF directory's entry count. */
#define TIFF_DIRECTORY_COUNT_SIZE 2U
/** @brief Bytes occupied by the next-directory link after an entry table. */
#define TIFF_DIRECTORY_NEXT_SIZE 4U
/** @brief Limit on JPEG header segments inspected before reaching image data. */
#define JPEG_MAX_HEADER_SEGMENTS 1024U
/** @brief Limit on ordinary JPEG header segment bytes inspected. */
#define JPEG_MAX_HEADER_BYTES (4U * 1024U * 1024U)
/** @brief Read-ahead capacity independent of image dimensions. */
#define JPEG_STREAM_CAPACITY 8192U

/** @brief APP1 identifier; its terminators are part of the on-disk signature. */
static const unsigned char exif_signature[] = {'E', 'x', 'i', 'f', 0, 0};

/** @brief Use one predictable error for invalid arguments and malformed values.
 * @return Always -1, after setting errno to EINVAL.
 */
static int invalid(void)
{
  errno = EINVAL;
  return -1;
}

/** @brief Read two already-checked bytes without alignment or host-endian assumptions.
 * @param[in] data At least two accessible bytes.
 * @param[in] little True for little-endian TIFF, false for big-endian TIFF.
 * @return Decoded unsigned 16-bit value.
 */
static uint16_t read_short(const unsigned char *data, bool little)
{
  return little ? (uint16_t)((uint16_t)data[0] | (uint16_t)data[1] << 8)
                : (uint16_t)((uint16_t)data[0] << 8 | data[1]);
}

/** @brief Read four checked bytes; casts precede shifts to avoid signed promotion.
 * @param[in] data At least four accessible bytes.
 * @param[in] little Byte order declared by the input TIFF header.
 * @return Decoded unsigned 32-bit value.
 */
static uint32_t read_long(const unsigned char *data, bool little)
{
  return little ? (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 |
                      (uint32_t)data[3] << 24
                : (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 |
                      (uint32_t)data[3];
}

void exif_encode_short(unsigned char *data, uint16_t value)
{
  /* Reserved storage may be unaligned; byte stores also keep host byte order irrelevant. */
  data[0] = (unsigned char)value;
  data[1] = (unsigned char)(value >> 8);
}

void exif_encode_long(unsigned char *data, uint32_t value)
{
  for (size_t index = 0; index < 4; ++index) {
    data[index] = (unsigned char)(value >> (index * 8));
  }
}

/** @brief Map TIFF's numbered representations to their component byte widths.
 * @param[in] format On-disk TIFF type identifier.
 * @return Component width, or zero for an unsupported type.
 * @details The table includes ordinary TIFF types beyond our named enum values
 * so unknown but structurally valid input fields can be checked and then ignored.
 */
static size_t format_size(uint16_t format)
{
  static const unsigned char sizes[] = {0, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8};
  return format < sizeof(sizes) ? sizes[format] : 0;
}

/** @brief Check a byte span without forming a potentially overflowing end offset.
 * @param[in] total Accessible bytes in the enclosing storage.
 * @param[in] offset Proposed starting offset.
 * @param[in] length Proposed span length.
 * @return True when the entire span fits.
 * @details Check the start first; only then is the subtraction safe. Keep this
 * order when extending the parser with another field or pointer check.
 */
static bool range(size_t total, size_t offset, size_t length)
{
  return offset <= total && length <= total - offset;
}

/** @brief Detect caller-buffer aliasing without adding lengths to addresses.
 * @param[in] first First storage address.
 * @param[in] first_size First storage length in bytes.
 * @param[in] second Second storage address.
 * @param[in] second_size Second storage length in bytes.
 * @return True for overlapping nonempty spans.
 * @details Address differences avoid wraparound at the end of an address space.
 * This uses the flat uintptr_t address representation of our Linux targets.
 */
static bool overlaps(const void *first, size_t first_size, const void *second, size_t second_size)
{
  if (!first_size || !second_size) {
    return false;
  }
  const uintptr_t first_address = (uintptr_t)first;
  const uintptr_t second_address = (uintptr_t)second;
  return first_address <= second_address ? second_address - first_address < first_size
                                         : first_address - second_address < second_size;
}

/** @brief Resolve one TIFF entry into checked, borrowed value bytes.
 * @param[in] view TIFF storage and declared byte order.
 * @param[in] offset TIFF-relative offset of the twelve-byte directory entry.
 * @param[out] value Field descriptor; use only when validation succeeds.
 * @return True for a supported, fully accessible representation.
 * @details TIFF stores values of at most four bytes directly in the entry;
 * larger values use an offset relative to the TIFF header. Count validation
 * precedes multiplication, so an untrusted component count cannot overflow.
 */
static bool entry_value(const struct exif_view *view, size_t offset, struct exif_value *value)
{
  if (!range(view->size, offset, TIFF_ENTRY_SIZE)) {
    return false;
  }
  const unsigned char *entry = view->tiff + offset;
  const uint16_t format = read_short(entry + 2, view->little_endian);
  const size_t unit = format_size(format);
  const uint32_t count = read_long(entry + 4, view->little_endian);
  if (!unit || !count || count > view->size / unit) {
    return false;
  }
  const size_t size = unit * count;
  /* The last four entry bytes are either the value itself or its TIFF-relative offset. */
  const size_t location =
      size <= TIFF_INLINE_SIZE ? offset + 8 : read_long(entry + 8, view->little_endian);
  if ((size > TIFF_INLINE_SIZE && location < TIFF_HEADER_SIZE) ||
      !range(view->size, location, size)) {
    return false;
  }
  *value = (struct exif_value){.data = view->tiff + location,
                               .size = size,
                               .count = count,
                               .format = format,
                               .little_endian = view->little_endian};
  if (format == EXIF_ASCII && value->data[size - 1] != 0) {
    return false;
  }
  if (format == EXIF_RATIONAL || format == EXIF_SRATIONAL) {
    for (size_t index = 0; index < count; ++index) {
      if (read_long(value->data + index * 8 + 4, view->little_endian) == 0) {
        return false;
      }
    }
  }
  return true;
}

/** @brief Validate one bounded IFD and discover the two supported child links.
 * @param[in,out] view Candidate view; primary-directory links fill child offsets.
 * @param[in] directory Directory identity to validate.
 * @return True for an absent directory or a valid directory table and its values.
 * @details Duplicate tags are ambiguous, so reject them rather than let readers
 * disagree over which value wins. A small bounded scan avoids heap-backed sets
 * and does not assume that camera entries arrive sorted.
 */
static bool validate_directory(struct exif_view *view, enum exif_directory directory)
{
  const size_t offset = view->directories[directory];
  if (!offset) {
    return true;
  }
  if (offset < TIFF_HEADER_SIZE || !range(view->size, offset, TIFF_DIRECTORY_COUNT_SIZE)) {
    return false;
  }
  const size_t count = read_short(view->tiff + offset, view->little_endian);
  if (count > EXIF_MAX_INPUT_ENTRIES ||
      !range(view->size, offset + TIFF_DIRECTORY_COUNT_SIZE,
             count * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE)) {
    return false;
  }
  uint16_t tags[EXIF_MAX_INPUT_ENTRIES];
  for (size_t index = 0; index < count; ++index) {
    const size_t entry = offset + TIFF_DIRECTORY_COUNT_SIZE + index * TIFF_ENTRY_SIZE;
    const uint16_t tag = read_short(view->tiff + entry, view->little_endian);
    for (size_t previous = 0; previous < index; ++previous) {
      if (tags[previous] == tag) {
        return false;
      }
    }
    tags[index] = tag;
    struct exif_value value;
    if (!entry_value(view, entry, &value)) {
      return false;
    }
    if (tag == EXIF_CAMERA_POINTER || tag == EXIF_GPS_POINTER) {
      /* Only primary-IFD child links are followed; nested/vendor graphs stay out of scope. */
      if (directory != EXIF_PRIMARY || value.format != EXIF_LONG || value.count != 1) {
        return false;
      }
      const uint32_t target = read_long(value.data, view->little_endian);
      if (target < TIFF_HEADER_SIZE) {
        return false;
      }
      view->directories[tag == EXIF_CAMERA_POINTER ? EXIF_CAMERA : EXIF_GPS] = target;
    }
  }
  return true;
}

int exif_parse(struct exif_view *view, const unsigned char *payload, size_t size)
{
  if (!view) {
    return invalid();
  }
  *view = (struct exif_view){0};
  if (!payload || size < sizeof(exif_signature) + TIFF_HEADER_SIZE || size > EXIF_MAX_PAYLOAD ||
      memcmp(payload, exif_signature, sizeof(exif_signature)) != 0) {
    return invalid();
  }
  const unsigned char *tiff = payload + sizeof(exif_signature);
  const bool little = tiff[0] == 'I' && tiff[1] == 'I';
  if ((!little && !(tiff[0] == 'M' && tiff[1] == 'M')) || read_short(tiff + 2, little) != 42) {
    return invalid();
  }
  /* Publish only after every supported table is checked; failure leaves no partial view. */
  struct exif_view candidate = {.tiff = tiff,
                                .size = size - sizeof(exif_signature),
                                .directories = {read_long(tiff + 4, little), 0, 0},
                                .little_endian = little};
  if (candidate.directories[EXIF_PRIMARY] < TIFF_HEADER_SIZE) {
    return invalid();
  }
  for (size_t index = 0; index < EXIF_DIRECTORY_COUNT; ++index) {
    if (!validate_directory(&candidate, (enum exif_directory)index)) {
      return invalid();
    }
  }
  /* Distinct directory identities must not reinterpret overlapping table bytes. */
  for (size_t index = 0; index < EXIF_DIRECTORY_COUNT; ++index) {
    if (!candidate.directories[index]) {
      continue;
    }
    const size_t count = read_short(tiff + candidate.directories[index], little);
    const size_t extent =
        TIFF_DIRECTORY_COUNT_SIZE + count * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE;
    for (size_t other = 0; other < index; ++other) {
      if (!candidate.directories[other]) {
        continue;
      }
      const size_t other_count = read_short(tiff + candidate.directories[other], little);
      const size_t other_extent =
          TIFF_DIRECTORY_COUNT_SIZE + other_count * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE;
      if (candidate.directories[index] < candidate.directories[other] + other_extent &&
          candidate.directories[other] < candidate.directories[index] + extent) {
        return invalid();
      }
    }
  }
  *view = candidate;
  return 0;
}

bool exif_get(const struct exif_view *view, enum exif_directory directory, uint16_t tag,
              struct exif_value *value)
{
  if (!value) {
    return false;
  }
  *value = (struct exif_value){0};
  if (!view || !view->tiff || (unsigned)directory >= EXIF_DIRECTORY_COUNT) {
    return false;
  }
  const size_t offset = view->directories[directory];
  if (!offset || !range(view->size, offset, TIFF_DIRECTORY_COUNT_SIZE)) {
    return false;
  }
  const size_t count = read_short(view->tiff + offset, view->little_endian);
  if (count > EXIF_MAX_INPUT_ENTRIES ||
      !range(view->size, offset + TIFF_DIRECTORY_COUNT_SIZE,
             count * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE)) {
    return false;
  }
  for (size_t index = 0; index < count; ++index) {
    const size_t entry = offset + TIFF_DIRECTORY_COUNT_SIZE + index * TIFF_ENTRY_SIZE;
    if (read_short(view->tiff + entry, view->little_endian) == tag) {
      return entry_value(view, entry, value);
    }
  }
  return false;
}

bool exif_value_unsigned(const struct exif_value *value, size_t index, uint32_t *result)
{
  if (!value || !value->data || !result || index >= value->count) {
    return false;
  }
  switch (value->format) {
    case EXIF_BYTE:
      if (index >= value->size) {
        return false;
      }
      *result = value->data[index];
      break;

    case EXIF_SHORT:
      if (index >= value->size / 2) {
        return false;
      }
      *result = read_short(value->data + index * 2, value->little_endian);
      break;

    case EXIF_LONG:
      if (index >= value->size / 4) {
        return false;
      }
      *result = read_long(value->data + index * 4, value->little_endian);
      break;

    default:
      return false;
  }
  return true;
}

/** @brief Interpret a TIFF signed component without an implementation-defined cast.
 * @param[in] value Unsigned bit representation of a 32-bit signed integer.
 * @return Its signed mathematical value, exactly representable as a double.
 * @details Subtracting 2^32 handles negative values, including INT32_MIN, without
 * signed overflow. This keeps signed rational handling portable between compilers.
 */
static double signed_long(uint32_t value)
{
  return value <= INT32_MAX ? (double)value : (double)value - 4294967296.0;
}

bool exif_value_rational(const struct exif_value *value, size_t index, double *result)
{
  if (!value || !value->data || !result || index >= value->count || index >= value->size / 8 ||
      (value->format != EXIF_RATIONAL && value->format != EXIF_SRATIONAL)) {
    return false;
  }
  const uint32_t numerator = read_long(value->data + index * 8, value->little_endian);
  const uint32_t denominator = read_long(value->data + index * 8 + 4, value->little_endian);
  if (!denominator) {
    return false;
  }
  *result = value->format == EXIF_SRATIONAL ? signed_long(numerator) / signed_long(denominator)
                                            : (double)numerator / denominator;
  return true;
}

size_t exif_text_length(const struct exif_value *value)
{
  if (!value || !value->data || (value->format != EXIF_ASCII && value->format != EXIF_UNDEFINED)) {
    return 0;
  }
  size_t size = value->size;
  /* Trim only the end: UserComment's encoding prefix contains meaningful embedded NULs. */
  while (size && value->data[size - 1] == 0) {
    --size;
  }
  return size;
}

void exif_document_init(struct exif_document *document)
{
  if (!document) {
    return;
  }
  document->count = 0;
  document->used = 0;
  /* These fixed defaults fit an empty document by construction. Clear only the
   * active bookkeeping; reservations initialize the bytes we actually expose. */
  unsigned char *data =
      exif_document_reserve(document, EXIF_PRIMARY, EXIF_ORIENTATION, EXIF_SHORT, 1);
  exif_encode_short(data, 1);
  (void)exif_document_rational(document, EXIF_PRIMARY, EXIF_X_RESOLUTION, 72, 1);
  (void)exif_document_rational(document, EXIF_PRIMARY, EXIF_Y_RESOLUTION, 72, 1);
  data = exif_document_reserve(document, EXIF_PRIMARY, EXIF_RESOLUTION_UNIT, EXIF_SHORT, 1);
  exif_encode_short(data, 2);
  data = exif_document_reserve(document, EXIF_CAMERA, EXIF_VERSION, EXIF_UNDEFINED, 4);
  data[0] = '0';
  data[1] = '2';
  data[2] = '3';
  data[3] = '2';
  data = exif_document_reserve(document, EXIF_CAMERA, EXIF_FLASHPIX_VERSION, EXIF_UNDEFINED, 4);
  data[0] = '0';
  data[1] = '1';
  data[2] = '0';
  data[3] = '0';
  data = exif_document_reserve(document, EXIF_CAMERA, EXIF_COMPONENTS_CONFIGURATION, EXIF_UNDEFINED,
                               4);
  data[0] = 1;
  data[1] = 2;
  data[2] = 3;
  data = exif_document_reserve(document, EXIF_CAMERA, EXIF_COLOR_SPACE, EXIF_SHORT, 1);
  exif_encode_short(data, 1);
}

bool exif_document_get(const struct exif_document *document, enum exif_directory directory,
                       uint16_t tag, struct exif_value *value)
{
  if (!value) {
    return false;
  }
  *value = (struct exif_value){0};
  if (!document || document->count > EXIF_MAX_ENTRIES || document->used > sizeof(document->data) ||
      (unsigned)directory >= EXIF_DIRECTORY_COUNT) {
    return false;
  }
  for (size_t index = 0; index < document->count; ++index) {
    const struct exif_item *entry = &document->entries[index];
    if (entry->directory == directory && entry->tag == tag &&
        range(document->used, entry->offset, entry->size)) {
      *value = (struct exif_value){.data = document->data + entry->offset,
                                   .size = entry->size,
                                   .count = entry->count,
                                   .format = entry->format,
                                   .little_endian = true};
      return true;
    }
  }
  return false;
}

int exif_document_remove(struct exif_document *document, enum exif_directory directory,
                         uint16_t tag)
{
  if (!document || document->count > EXIF_MAX_ENTRIES || document->used > sizeof(document->data) ||
      (unsigned)directory >= EXIF_DIRECTORY_COUNT) {
    return invalid();
  }
  for (size_t index = 0; index < document->count; ++index) {
    const struct exif_item entry = document->entries[index];
    if (entry.directory != directory || entry.tag != tag) {
      continue;
    }
    if (!range(document->used, entry.offset, entry.size)) {
      return invalid();
    }
    /* Forward copying is safe because each source byte lies above its destination.
     * Reclaiming holes lets repeated metadata updates reuse the same fixed buffer. */
    for (size_t position = entry.offset; position < document->used - entry.size; ++position) {
      document->data[position] = document->data[position + entry.size];
    }
    document->used -= entry.size;
    for (size_t other = 0; other < document->count; ++other) {
      if (document->entries[other].offset > entry.offset) {
        document->entries[other].offset -= entry.size;
      }
    }
    for (size_t other = index; other + 1 < document->count; ++other) {
      document->entries[other] = document->entries[other + 1];
    }
    --document->count;
    break;
  }
  return 0;
}

unsigned char *exif_document_reserve(struct exif_document *document, enum exif_directory directory,
                                     uint16_t tag, enum exif_format format, uint32_t count)
{
  const size_t unit = format_size((uint16_t)format);
  if (!document || (unsigned)directory >= EXIF_DIRECTORY_COUNT || !unit || !count ||
      count > EXIF_MAX_PAYLOAD / unit || tag == EXIF_CAMERA_POINTER || tag == EXIF_GPS_POINTER ||
      document->count > EXIF_MAX_ENTRIES || document->used > sizeof(document->data)) {
    invalid();
    return NULL;
  }
  const size_t size = unit * count;
  struct exif_value previous;
  const bool replacing = exif_document_get(document, directory, tag, &previous);
  /* Account for the old value before removing it: a full document can still
   * replace a field, and a capacity error must not erase the old value. */
  if ((!replacing && document->count == EXIF_MAX_ENTRIES) ||
      size > sizeof(document->data) - document->used + (replacing ? previous.size : 0)) {
    errno = ENOSPC;
    return NULL;
  }
  if (exif_document_remove(document, directory, tag) != 0) {
    return NULL;
  }
  unsigned char *data = document->data + document->used;
  document->entries[document->count++] = (struct exif_item){.offset = (uint32_t)document->used,
                                                            .size = (uint32_t)size,
                                                            .count = count,
                                                            .tag = tag,
                                                            .format = (uint16_t)format,
                                                            .directory = (unsigned char)directory};
  document->used += size;
  memset(data, 0, size);
  return data;
}

int exif_document_ascii(struct exif_document *document, enum exif_directory directory, uint16_t tag,
                        const char *text)
{
  if (!document || !text || overlaps(document, sizeof(*document), text, 1)) {
    return invalid();
  }
  /* Never scan into the document and then compact that same storage while copying. */
  size_t length = 0;
  while (length < EXIF_MAX_PAYLOAD && text[length]) {
    ++length;
  }
  if (length == EXIF_MAX_PAYLOAD) {
    errno = ENOSPC;
    return -1;
  }
  if (overlaps(document, sizeof(*document), text, length + 1)) {
    return invalid();
  }
  unsigned char *data =
      exif_document_reserve(document, directory, tag, EXIF_ASCII, (uint32_t)length + 1);
  if (!data) {
    return -1;
  }
  for (size_t index = 0; index < length; ++index) {
    data[index] = (unsigned char)text[index];
  }
  return 0;
}

int exif_document_rational(struct exif_document *document, enum exif_directory directory,
                           uint16_t tag, uint32_t numerator, uint32_t denominator)
{
  if (!denominator) {
    return invalid();
  }
  unsigned char *data = exif_document_reserve(document, directory, tag, EXIF_RATIONAL, 1);
  if (!data) {
    return -1;
  }
  exif_encode_long(data, numerator);
  exif_encode_long(data + 4, denominator);
  return 0;
}

/** @brief Select standard fields retained by the default import policy.
 * @param[in] directory Directory containing the field.
 * @param[in] tag Directory-local tag identifier.
 * @return True when the field belongs in an imported writer document.
 * @details Byte-for-byte copying is not enough for vendor records with private
 * offsets. An explicit allowlist keeps imports small and avoids pretending to
 * preserve structures we neither interpret nor need. Defaults are rebuilt separately.
 */
static bool supported(enum exif_directory directory, uint16_t tag)
{
  if (directory == EXIF_PRIMARY) {
    return tag == EXIF_DOCUMENT_NAME || tag == EXIF_IMAGE_DESCRIPTION || tag == EXIF_ORIENTATION ||
           tag == EXIF_SOFTWARE;
  }
  if (directory == EXIF_CAMERA) {
    return tag == EXIF_EXPOSURE_TIME || tag == EXIF_FNUMBER || tag == EXIF_EXPOSURE_BIAS ||
           tag == EXIF_FOCAL_LENGTH || tag == EXIF_USER_COMMENT || tag == EXIF_PIXEL_X_DIMENSION ||
           tag == EXIF_PIXEL_Y_DIMENSION || tag == EXIF_LENS_MODEL;
  }
  return tag <= EXIF_GPS_ALTITUDE || (tag >= EXIF_GPS_SPEED_REF && tag <= EXIF_GPS_DIRECTION);
}

/** @brief Copy supported fields from one directory into writer storage.
 * @param[in,out] document Initialized destination document.
 * @param[in] view Source bytes and input byte order.
 * @param[in] directory Directory whose fields should be imported.
 * @param[in] select Optional caller selection for additional fields.
 * @param[in] context Borrowed selection state.
 * @return True on success, false when the caller must reset the partial import.
 * @details Keeping rollback in the public wrapper makes this routine responsible
 * only for traversal and byte-order conversion, not document lifecycle.
 */
static bool import_directory(struct exif_document *document, const struct exif_view *view,
                             enum exif_directory directory, exif_tag_selector select, void *context)
{
  const size_t offset = view->directories[directory];
  if (offset == 0) {
    return true;
  }
  if (!range(view->size, offset, TIFF_DIRECTORY_COUNT_SIZE)) {
    return false;
  }

  const size_t count = read_short(view->tiff + offset, view->little_endian);
  if (count > EXIF_MAX_INPUT_ENTRIES ||
      !range(view->size, offset + TIFF_DIRECTORY_COUNT_SIZE,
             count * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE)) {
    return false;
  }

  for (size_t index = 0; index < count; ++index) {
    const size_t location = offset + TIFF_DIRECTORY_COUNT_SIZE + index * TIFF_ENTRY_SIZE;
    const uint16_t tag = read_short(view->tiff + location, view->little_endian);
    if (!supported(directory, tag) && (!select || !select(context, directory, tag))) {
      continue;
    }

    struct exif_value value;
    if (!entry_value(view, location, &value)) {
      return false;
    }

    unsigned char *data = exif_document_reserve(document, directory, tag,
                                                (enum exif_format)value.format, value.count);
    if (data == NULL) {
      return false;
    }

    size_t unit = format_size(value.format);
    /* A rational contains two 32-bit integers. Reverse each integer, not the
     * whole eight-byte pair, or numerator and denominator would trade places. */
    if (value.format == EXIF_RATIONAL || value.format == EXIF_SRATIONAL) {
      unit = 4;
    }
    for (size_t position = 0; position < value.size; ++position) {
      const size_t source = !value.little_endian && unit > 1
                                ? position / unit * unit + unit - 1 - position % unit
                                : position;
      data[position] = value.data[source];
    }
  }
  return true;
}

int exif_document_import_custom(struct exif_document *document, const struct exif_view *view,
                                exif_tag_selector select, void *context)
{
  if (!document || !view || !view->tiff || view->size > EXIF_MAX_PAYLOAD - sizeof(exif_signature) ||
      overlaps(document, sizeof(*document), view->tiff, view->size) ||
      overlaps(document, sizeof(*document), view, sizeof(*view))) {
    return invalid();
  }
  exif_document_init(document);
  for (size_t directory = 0; directory < EXIF_DIRECTORY_COUNT; ++directory) {
    if (!import_directory(document, view, (enum exif_directory)directory, select, context)) {
      /* A failed import must not leave a plausible-looking half-copied record. */
      exif_document_init(document);
      return invalid();
    }
  }
  return 0;
}

int exif_document_import(struct exif_document *document, const struct exif_view *view)
{
  return exif_document_import_custom(document, view, NULL, NULL);
}

/** @brief Validate a writer field before calculating offsets or touching output.
 * @param[in] document Source document whose storage has already been bounded.
 * @param[in] entry Field descriptor to check against that storage.
 * @return True when the representation can be serialized safely.
 */
static bool valid_document_entry(const struct exif_document *document,
                                 const struct exif_item *entry)
{
  const size_t unit = format_size(entry->format);
  if (entry->directory >= EXIF_DIRECTORY_COUNT || unit == 0 || entry->count == 0 ||
      entry->count > EXIF_MAX_PAYLOAD / unit || entry->size != unit * entry->count ||
      !range(document->used, entry->offset, entry->size) || entry->tag == EXIF_CAMERA_POINTER ||
      entry->tag == EXIF_GPS_POINTER) {
    return false;
  }

  const unsigned char *data = document->data + entry->offset;
  if (entry->format == EXIF_ASCII && data[entry->size - 1] != 0) {
    return false;
  }
  if (entry->format == EXIF_RATIONAL || entry->format == EXIF_SRATIONAL) {
    for (size_t component = 0; component < entry->count; ++component) {
      if (read_long(data + component * 8 + 4, true) == 0) {
        return false;
      }
    }
  }
  return true;
}

/** @brief Compare the wire ordering of two directory-local fields.
 * @param[in] first Candidate field.
 * @param[in] second Field currently ahead of the candidate.
 * @return True when the candidate belongs before the existing field.
 */
static bool entry_precedes(const struct exif_item *first, const struct exif_item *second)
{
  return first->directory < second->directory ||
         (first->directory == second->directory && first->tag < second->tag);
}

int exif_serialize(const struct exif_document *document, unsigned char *payload, size_t capacity,
                   size_t *size)
{
  /* Check the size-output alias before clearing it; it could otherwise corrupt
   * the document we are about to validate. Ordinary failures report zero bytes. */
  if (!size || (document && overlaps(document, sizeof(*document), size, sizeof(*size))) ||
      (payload && overlaps(payload, capacity, size, sizeof(*size)))) {
    return invalid();
  }
  *size = 0;
  if (!document || !payload || document->count > EXIF_MAX_ENTRIES ||
      document->used > sizeof(document->data) ||
      overlaps(document, sizeof(*document), payload, capacity)) {
    return invalid();
  }
  size_t counts[EXIF_DIRECTORY_COUNT] = {0};
  size_t order[EXIF_MAX_ENTRIES];
  /* First pass: validate and sort only indices. The caller's document stays
   * unchanged, and the small entry budget makes insertion sort inexpensive. */
  for (size_t index = 0; index < document->count; ++index) {
    const struct exif_item *entry = &document->entries[index];
    if (!valid_document_entry(document, entry)) {
      return invalid();
    }
    ++counts[entry->directory];
    size_t position = index;
    while (position && entry_precedes(entry, &document->entries[order[position - 1]])) {
      order[position] = order[position - 1];
      --position;
    }
    order[position] = index;
    if (position && document->entries[order[position - 1]].directory == entry->directory &&
        document->entries[order[position - 1]].tag == entry->tag) {
      return invalid();
    }
  }
  counts[EXIF_PRIMARY] += (counts[EXIF_CAMERA] ? 1U : 0U) + (counts[EXIF_GPS] ? 1U : 0U);
  /* Reserve the TIFF header and all directory tables before placing value bytes.
   * Directory links are generated here, never accepted as caller-provided values. */
  size_t offsets[EXIF_DIRECTORY_COUNT] = {TIFF_HEADER_SIZE, 0, 0};
  size_t extent = TIFF_HEADER_SIZE;
  for (size_t directory = 0; directory < EXIF_DIRECTORY_COUNT; ++directory) {
    if (directory && !counts[directory]) {
      continue;
    }
    offsets[directory] = extent;
    extent +=
        TIFF_DIRECTORY_COUNT_SIZE + counts[directory] * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE;
  }

  for (size_t index = 0; index < document->count; ++index) {
    if (document->entries[index].size > TIFF_INLINE_SIZE) {
      extent += document->entries[index].size + (document->entries[index].size & 1U);
    }
  }
  if (extent > EXIF_MAX_PAYLOAD - sizeof(exif_signature) ||
      capacity < extent + sizeof(exif_signature)) {
    errno = ENOSPC;
    return -1;
  }
  /* Only now touch output. Zeroing supplies next-IFD terminators, unused inline
   * value bytes and padding without leaking old workspace contents into photos. */
  memset(payload, 0, extent + sizeof(exif_signature));
  for (size_t index = 0; index < sizeof(exif_signature); ++index) {
    payload[index] = exif_signature[index];
  }
  unsigned char *tiff = payload + sizeof(exif_signature);
  tiff[0] = 'I';
  tiff[1] = 'I';
  exif_encode_short(tiff + 2, 42);
  exif_encode_long(tiff + 4, TIFF_HEADER_SIZE);
  size_t cursors[EXIF_DIRECTORY_COUNT];
  size_t tail = TIFF_HEADER_SIZE;
  for (size_t directory = 0; directory < EXIF_DIRECTORY_COUNT; ++directory) {
    cursors[directory] = offsets[directory] + TIFF_DIRECTORY_COUNT_SIZE;
    if (directory && !counts[directory]) {
      continue;
    }
    exif_encode_short(tiff + offsets[directory], (uint16_t)counts[directory]);
    tail +=
        TIFF_DIRECTORY_COUNT_SIZE + counts[directory] * TIFF_ENTRY_SIZE + TIFF_DIRECTORY_NEXT_SIZE;
  }
  for (size_t index = 0; index < document->count; ++index) {
    const struct exif_item *entry = &document->entries[order[index]];
    unsigned char *output = tiff + cursors[entry->directory];
    cursors[entry->directory] += TIFF_ENTRY_SIZE;
    exif_encode_short(output, entry->tag);
    exif_encode_short(output + 2, entry->format);
    exif_encode_long(output + 4, entry->count);
    unsigned char *destination = output + 8;
    if (entry->size > TIFF_INLINE_SIZE) {
      /* External values start on two-byte boundaries; padding is already zero. */
      exif_encode_long(output + 8, (uint32_t)tail);
      destination = tiff + tail;
      tail += entry->size + (entry->size & 1U);
    }
    for (size_t position = 0; position < entry->size; ++position) {
      destination[position] = document->data[entry->offset + position];
    }
  }

  for (size_t directory = EXIF_CAMERA; directory < EXIF_DIRECTORY_COUNT; ++directory) {
    if (!counts[directory]) {
      continue;
    }
    unsigned char *entry = tiff + cursors[EXIF_PRIMARY];
    cursors[EXIF_PRIMARY] += TIFF_ENTRY_SIZE;
    exif_encode_short(entry, directory == EXIF_CAMERA ? EXIF_CAMERA_POINTER : EXIF_GPS_POINTER);
    exif_encode_short(entry + 2, EXIF_LONG);
    exif_encode_long(entry + 4, 1);
    exif_encode_long(entry + 8, (uint32_t)offsets[directory]);
  }
  /* Child pointers were appended after ordinary entries. Sort the primary table
   * again so those generated tags obey TIFF ordering even for custom high tags. */
  for (size_t index = 1; index < counts[EXIF_PRIMARY]; ++index) {
    unsigned char saved[TIFF_ENTRY_SIZE];
    const size_t origin =
        offsets[EXIF_PRIMARY] + TIFF_DIRECTORY_COUNT_SIZE + index * TIFF_ENTRY_SIZE;
    for (size_t byte = 0; byte < TIFF_ENTRY_SIZE; ++byte) {
      saved[byte] = tiff[origin + byte];
    }
    size_t position = index;
    while (position && read_short(tiff + offsets[EXIF_PRIMARY] + TIFF_DIRECTORY_COUNT_SIZE +
                                      (position - 1) * TIFF_ENTRY_SIZE,
                                  true) > read_short(saved, true)) {
      const size_t target =
          offsets[EXIF_PRIMARY] + TIFF_DIRECTORY_COUNT_SIZE + position * TIFF_ENTRY_SIZE;
      for (size_t byte = 0; byte < TIFF_ENTRY_SIZE; ++byte) {
        tiff[target + byte] = tiff[target - TIFF_ENTRY_SIZE + byte];
      }
      --position;
    }
    for (size_t byte = 0; byte < TIFF_ENTRY_SIZE; ++byte) {
      tiff[offsets[EXIF_PRIMARY] + TIFF_DIRECTORY_COUNT_SIZE + position * TIFF_ENTRY_SIZE + byte] =
          saved[byte];
    }
  }

  *size = extent + sizeof(exif_signature);
  return 0;
}

/** @brief Read an entire declared span or report truncation.
 * @param[in,out] input Readable stream at the span's first byte.
 * @param[out] buffer Storage for at least @p size bytes.
 * @param[in] size Exact byte count required.
 * @return True only when every requested byte was read.
 */
static bool read_exact(FILE *input, unsigned char *buffer, size_t size)
{
  return fread(buffer, 1, size, input) == size;
}

/** @brief Consume unwanted bytes without trusting a seek past end-of-file.
 * @param[in,out] input Readable stream positioned before the unwanted span.
 * @param[in] size Bytes that must actually exist.
 * @return True after consuming the span, false for a short read.
 * @details A seek can succeed beyond EOF; reading bounded chunks makes a truncated
 * JPEG segment fail instead of hiding the missing bytes from the header parser.
 */
static bool skip_exact(FILE *input, size_t size)
{
  unsigned char buffer[1024];
  while (size) {
    const size_t chunk = size < sizeof(buffer) ? size : sizeof(buffer);
    if (!read_exact(input, buffer, chunk)) {
      return false;
    }
    size -= chunk;
  }
  return true;
}

/** @brief Extract the unique EXIF payload from a caller-owned JPEG stream.
 * @param[in,out] input Readable stream at start-of-image; not closed here.
 * @param[out] payload Destination for the selected APP1 payload.
 * @param[in] capacity Writable payload capacity in bytes.
 * @param[out] size Selected payload length; the public wrapper clears it on failure.
 * @return Zero on success, otherwise -1 with errno describing the parse failure.
 * @details Ownership stays with the wrapper so every parse exit can be a plain
 * return, while file closing and error preservation remain in one place.
 */
static int read_exif_stream(FILE *input, unsigned char *payload, size_t capacity, size_t *size)
{
  unsigned char header[2];
  bool found = false;
  if (!read_exact(input, header, 2) || header[0] != 0xff || header[1] != 0xd8) {
    return invalid();
  }

  size_t inspected = 2;
  /* Metadata lives before the first scan. Bound header work rather than walking
   * an arbitrarily large file when a malformed producer never supplies a scan. */
  for (size_t segments = 0; segments < JPEG_MAX_HEADER_SEGMENTS; ++segments) {
    const int prefix = fgetc(input);
    if (prefix != 0xff) return invalid();
    int marker = fgetc(input);
    if (marker == EOF) {
      return invalid();
    }
    size_t fill = 0;
    while (marker == 0xff) {
      if (++fill > EXIF_MAX_PAYLOAD) {
        return invalid();
      }
      marker = fgetc(input);
    }

    if (marker == EOF || marker == 0 || marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7)) {
      return invalid();
    }
    if (marker == 0xd9) {
      if (found) {
        return 0;
      }
      errno = ENODATA;
      return -1;
    }

    if (marker == 1) {
      continue;
    }
    if (!read_exact(input, header, 2)) {
      return invalid();
    }
    const size_t length = (size_t)header[0] * 256 + header[1];
    if (length < 2) {
      return invalid();
    }
    inspected += length + 2;
    if (inspected > JPEG_MAX_HEADER_BYTES) {
      errno = EFBIG;
      return -1;
    }
    const size_t remaining = length - 2;
    unsigned char signature[sizeof(exif_signature)];
    if (marker == 0xe1 && remaining >= sizeof(signature)) {
      if (!read_exact(input, signature, sizeof(signature))) {
        return invalid();
      }
      if (memcmp(signature, exif_signature, sizeof(signature)) == 0) {
        /* A second EXIF record is ambiguous; keep callers from silently mixing records. */
        if (found) {
          return invalid();
        }
        if (remaining > capacity) {
          errno = ENOSPC;
          return -1;
        }
        for (size_t index = 0; index < sizeof(signature); ++index) {
          payload[index] = signature[index];
        }
        if (!read_exact(input, payload + sizeof(signature), remaining - sizeof(signature))) {
          return invalid();
        }
        *size = remaining;
        found = true;
      } else if (!skip_exact(input, remaining - sizeof(signature))) {
        return invalid();
      }
    } else if (!skip_exact(input, remaining)) {
      return invalid();
    }
    if (marker == 0xda) {
      if (remaining < 4) {
        return invalid();
      }
      if (found) {
        return 0;
      }
      errno = ENODATA;
      return -1;
    }
  }
  return invalid();
}

int exif_read_stream(FILE *input, unsigned char *payload, size_t capacity, size_t *size)
{
  if (!input || !payload || !size || overlaps(payload, capacity, size, sizeof(*size))) return invalid();
  *size = 0;
  const int result = read_exif_stream(input, payload, capacity, size);
  if (result) *size = 0;
  return result;
}

int exif_read_jpeg(const char *filename, unsigned char *payload, size_t capacity, size_t *size)
{
  if (size != NULL) {
    *size = 0;
  }
  if (filename == NULL || payload == NULL || size == NULL) {
    return invalid();
  }

  FILE *input = fopen(filename, "rb");
  if (input == NULL) {
    return -1;
  }

  int result = exif_read_stream(input, payload, capacity, size);
  if (result != 0 && ferror(input)) {
    errno = EIO;
  }
  /* Preserve the parse/read error across successful cleanup, but report a real
   * close failure rather than returning a payload whose I/O did not finish cleanly. */
  const int saved_errno = errno;
  if (fclose(input) != 0) {
    result = -1;
  } else if (result != 0) {
    errno = saved_errno;
  }
  if (result != 0) {
    *size = 0;
  }
  return result;
}

/** @brief Original-file interval occupied by one discarded EXIF segment. */
struct jpeg_removal {
  off_t start; /**< First marker fill byte, inclusive. */
  off_t end;   /**< First byte after the payload, exclusive. */
};

/** @brief Bounded edit list produced by the same parser used for streaming rewrites.
 * @details Only offsets are retained, never image data. Rejecting an excessive
 * number of EXIF segments before mutation prevents input-controlled storage growth.
 */
struct jpeg_edit_plan {
  off_t insertion; /**< Replacement location after leading JFIF/APP0 segments. */
  size_t count;    /**< Number of ordered, nonoverlapping removal intervals. */
  struct jpeg_removal removals[JPEG_MAX_HEADER_SEGMENTS]; /**< Original EXIF spans. */
};

/** @brief One bounded read-ahead buffer shared by JPEG inspection and rewriting.
 * @details Cursor-based consumption lets entropy runs be copied in chunks rather
 * than one stdio call per image byte. NULL output makes preflight write-free.
 */
struct jpeg_stream {
  FILE *input;     /**< Caller-owned source consumed through the read-ahead buffer. */
  FILE *output;    /**< Destination, or NULL for inspection and validation-only walks. */
  struct jpeg_edit_plan *plan; /**< Optional edit offsets collected without output. */
  size_t position; /**< Cursor of the next unread byte in the buffer. */
  size_t size;     /**< Number of valid bytes currently in the buffer. */
  int error;
  unsigned char
      buffer[JPEG_STREAM_CAPACITY]; /**< Fixed workspace, independent of image dimensions. */
};

/** @brief Refill only after buffered bytes have been consumed.
 * @param[in,out] stream Read-ahead state.
 * @return True when at least one byte remains, false on EOF or read error.
 * @details Callers distinguish EOF from I/O failure with ferror() where needed.
 */
static bool stream_fill(struct jpeg_stream *stream)
{
  if (stream->position < stream->size) {
    return true;
  }
  if (ferror(stream->input) || feof(stream->input)) return false;
  stream->size = fread(stream->buffer, 1, sizeof(stream->buffer), stream->input);
  stream->position = 0;
  return stream->size != 0;
}

/** @brief Read a marker/header byte while preserving buffered cursor ownership.
 * @param[in,out] stream Read-ahead state.
 * @return Next unsigned byte as an int, or EOF when no byte is available.
 */
static int stream_byte(struct jpeg_stream *stream)
{
  return stream_fill(stream) ? stream->buffer[stream->position++] : EOF;
}

/** @brief Share byte emission between real rewrites and write-free validation.
 * @param[in,out] stream Parser state with optional output.
 * @param[in] data Bytes to emit only when an output exists.
 * @param[in] size Accessible byte count.
 * @return True on complete emission or validation-only traversal.
 */
static bool stream_write(struct jpeg_stream *stream, const unsigned char *data, size_t size)
{
  return stream->output == NULL || fwrite(data, 1, size, stream->output) == size;
}

/** @brief Emit one framing byte, or do nothing during preflight.
 * @param[in,out] stream Parser state with optional output.
 * @param[in] byte Byte value to emit.
 * @return True on successful emission or validation-only traversal.
 */
static bool stream_put_byte(struct jpeg_stream *stream, int byte)
{
  return stream->output == NULL || fputc(byte, stream->output) != EOF;
}

/** @brief Recover the logical offset rather than the read-ahead descriptor offset.
 * @param[in] stream Parser state for a seekable regular file.
 * @return Offset of the next unconsumed byte, or -1 on a positioning error.
 */
static off_t stream_offset(const struct jpeg_stream *stream)
{
  const off_t position = ftello(stream->input);
  return position < 0 ? -1 : position - (off_t)(stream->size - stream->position);
}

/** @brief Copy or discard exactly one declared span in bounded chunks.
 * @param[in,out] stream Input cursor and optional writable output stream.
 * @param[in] size Bytes to consume; short input is an error.
 * @param[in] emit True to write bytes, false to discard an old EXIF segment.
 * @return True after consuming the full span, false on read/write failure.
 * @details Both paths consume existing read-ahead first, so changing from marker
 * parsing to bulk copying never loses or reads the same bytes twice.
 */
static bool stream_copy(struct jpeg_stream *stream, size_t size, bool emit)
{
  while (size) {
    if (!stream_fill(stream)) {
      return false;
    }
    size_t chunk = stream->size - stream->position;
    if (chunk > size) {
      chunk = size;
    }
    if (emit && !stream_write(stream, stream->buffer + stream->position, chunk)) {
      return false;
    }
    stream->position += chunk;
    size -= chunk;
  }
  return true;
}

/** @brief Emit a marker together with its original fill-byte count.
 * @param[in,out] stream State containing a writable output stream.
 * @param[in] count Number of leading 0xff bytes, including the marker prefix.
 * @param[in] marker Marker code, or stuffed zero when copying scan data.
 * @return True when all bytes were written.
 * @details Retaining legal fill bytes avoids unnecessary changes to unrelated
 * JPEG framing while replacement EXIF gets its own single-prefix marker.
 */
static bool marker_prefix(struct jpeg_stream *stream, size_t count, int marker)
{
  if (stream->output == NULL) {
    return true;
  }
  for (size_t index = 0; index < count; ++index) {
    if (fputc(0xff, stream->output) == EOF) {
      return false;
    }
  }
  return fputc(marker, stream->output) != EOF;
}

/** @brief Find a structural marker, optionally copying preceding entropy bytes.
 * @param[in,out] stream Buffered cursor; entropy mode requires a writable output.
 * @param[in] entropy True inside a scan, false between marker segments.
 * @param[out] prefix Marker-prefix/fill count for the caller to retain or discard.
 * @param[out] marker Structural marker code, not yet emitted.
 * @return True on a marker, false on malformed framing or I/O failure.
 * @details Inside scans, 0xff/0x00 is a stuffed data byte and restart markers
 * do not end the scan. Handle these here so the rewrite loop sees only markers
 * whose segments it must inspect. Prefix reads may cross buffer boundaries.
 */
static bool stream_marker(struct jpeg_stream *stream, bool entropy, size_t *prefix, int *marker)
{
  for (;;) {
    if (entropy) {
      if (!stream_fill(stream)) {
        return false;
      }
      const size_t start = stream->position;
      while (stream->position < stream->size && stream->buffer[stream->position] != 0xff) {
        ++stream->position;
      }
      const size_t length = stream->position - start;
      if (!stream_write(stream, stream->buffer + start, length)) {
        return false;
      }
      if (stream->position == stream->size) {
        continue;
      }
    }
    if (stream_byte(stream) != 0xff) {
      return false;
    }
    *prefix = 1;
    do {
      *marker = stream_byte(stream);
      if (*marker == 0xff && ++*prefix > EXIF_MAX_PAYLOAD) {
        return false;
      }
    } while (*marker == 0xff);
    if (*marker == EOF) {
      return false;
    }
    if (entropy && (*marker == 0 || *marker == 1 || (*marker >= 0xd0 && *marker <= 0xd7))) {
      if (!marker_prefix(stream, *prefix, *marker)) {
        return false;
      }
      continue;
    }
    return *marker != 0;
  }
}

/** @brief Recognize SOF markers while excluding other codes in the same range.
 * @param[in] marker JPEG marker code without its 0xff prefix.
 * @return True for a start-of-frame marker.
 * @details DHT, JPG and DAC share the 0xc0..0xcf range but do not carry dimensions.
 */
static bool frame_marker(int marker)
{
  return marker >= 0xc0 && marker <= 0xcf && marker != 0xc4 && marker != 0xc8 && marker != 0xcc;
}

/** @brief Read a short fixed-size header across read-ahead buffer boundaries.
 * @param[in,out] stream Buffered source state.
 * @param[out] data Storage for at least the requested byte count.
 * @param[in] size Exact byte count to read.
 * @return True when the complete header was available.
 */
static bool stream_read(struct jpeg_stream *stream, unsigned char *data, size_t size)
{
  for (size_t index = 0; index < size; ++index) {
    const int byte = stream_byte(stream);
    if (byte == EOF) {
      return false;
    }
    data[index] = (unsigned char)byte;
  }
  return true;
}

/** @brief Read and validate the common six-byte start-of-frame header.
 * @param[in,out] stream Source cursor immediately after the segment length.
 * @param[in] payload_size Full frame payload length, including component descriptors.
 * @param[out] header Precision, height, width and component count bytes.
 * @return True for the bounded frame representation supported by this module.
 * @details Inspection and rewriting use the same checks so they cannot disagree
 * about which dimensions or component counts are safe to publish.
 */
static bool stream_frame_header(struct jpeg_stream *stream, size_t payload_size,
                                unsigned char header[6])
{
  if (payload_size < 6 || !stream_read(stream, header, 6)) {
    return false;
  }

  const uint32_t height = (uint32_t)header[1] * 256 + header[2];
  const uint32_t width = (uint32_t)header[3] * 256 + header[4];
  const unsigned components = header[5];

  return header[0] > 0 && header[0] <= 16 && width > 0 && height > 0 && components > 0 &&
         components <= 4 && payload_size == 6U + 3U * components;
}

/** @brief Emit a segment prefix and its two-byte big-endian JPEG length.
 * @param[in,out] stream State containing a writable output stream.
 * @param[in] prefix Number of prefix/fill bytes to preserve.
 * @param[in] marker Segment marker code.
 * @param[in] payload_size Segment bytes excluding the two length bytes.
 * @return True when the complete framing was written.
 * @pre Payload length is already bounded by the JPEG segment limit.
 */
static bool stream_segment_header(struct jpeg_stream *stream, size_t prefix, int marker,
                                  size_t payload_size)
{
  const size_t length = payload_size + 2;
  return marker_prefix(stream, prefix, marker) &&
      stream_put_byte(stream, (int)(length >> 8)) &&
      stream_put_byte(stream, (int)(length & 255));
}

int exif_jpeg_dimensions(FILE *input, uint32_t *width, uint32_t *height, unsigned *components)
{
  if (!input || !width || !height || !components) {
    return invalid();
  }
  *width = 0;
  *height = 0;
  *components = 0;
  /* Read the frame's own dimensions; pre-existing EXIF may describe another image. */
  struct jpeg_stream stream = {.input = input};
  if (stream_byte(&stream) != 0xff || stream_byte(&stream) != 0xd8) {
    return invalid();
  }
  size_t inspected = 2;
  for (size_t segment = 0; segment < JPEG_MAX_HEADER_SEGMENTS; ++segment) {
    int marker;
    size_t prefix;
    if (!stream_marker(&stream, false, &prefix, &marker) || marker == 0xda || marker == 0xd9 ||
        marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7)) {
      return invalid();
    }
    inspected += prefix + 1;
    if (marker == 1) {
      continue;
    }
    const int high = stream_byte(&stream);
    const int low = stream_byte(&stream);
    if (high == EOF || low == EOF) {
      return invalid();
    }
    const size_t length = (size_t)high * 256 + (unsigned)low;
    if (length < 2) {
      return invalid();
    }
    inspected += length;
    if (inspected > JPEG_MAX_HEADER_BYTES) {
      errno = EFBIG;
      return -1;
    }
    size_t remaining = length - 2;
    if (frame_marker(marker)) {
      unsigned char header[6];
      if (!stream_frame_header(&stream, remaining, header) ||
          !stream_copy(&stream, remaining - 6, false)) {
        return invalid();
      }
      const uint32_t frame_width = (uint32_t)header[3] * 256 + header[4];
      const uint32_t frame_height = (uint32_t)header[1] * 256 + header[2];
      *width = frame_width;
      *height = frame_height;
      *components = header[5];
      return 0;
    }
    if (!stream_copy(&stream, remaining, false)) {
      return invalid();
    }
  }
  errno = EFBIG;
  return -1;
}

/** @brief Rewrite JPEG framing using caller-owned streams and validated EXIF.
 * @param[in,out] stream Buffered source and destination state.
 * @param[in] payload Validated replacement APP1 bytes.
 * @param[in] size Replacement payload length.
 * @return True after a complete frame/scan/EOI stream, false on framing or I/O failure.
 * @details This routine owns no resources. Early returns keep the scan-state
 * transitions visible; the public wrapper supplies one error-classification path.
 */
static bool rewrite_jpeg_stream(struct jpeg_stream *stream, const unsigned char *payload,
                                size_t size)
{
  bool exif_written = false;
  bool inside_scan = false;
  bool frame_seen = false;
  bool scan_seen = false;

  if (stream_byte(stream) != 0xff || stream_byte(stream) != 0xd8 ||
      !marker_prefix(stream, 1, 0xd8)) {
    return false;
  }

  for (;;) {
    size_t prefix;
    int marker;
    if (!stream_marker(stream, inside_scan, &prefix, &marker)) {
      return false;
    }
    inside_scan = false;
    if (marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7)) {
      return false;
    }

    off_t marker_start = 0;
    if (stream->plan != NULL) {
      const off_t position = stream_offset(stream);
      if (position < 0 || (uintmax_t)position < prefix + 1) {
        return false;
      }
      marker_start = position - (off_t)(prefix + 1);
    }

    if (!exif_written && marker != 0xe0) {
      if (stream->plan != NULL) {
        stream->plan->insertion = marker_start;
      }
      /* Leave leading APP0/JFIF segments first, then publish exactly one EXIF APP1. */
      if (!stream_segment_header(stream, 1, 0xe1, size) ||
          !stream_write(stream, payload, size)) {
        return false;
      }
      exif_written = true;
    }

    if (marker == 0xd9) {
      /* Reaching EOF is not enough: require a frame, a scan and an actual EOI.
       * Bytes after EOI may carry another application's data, so retain them. */
      if (!frame_seen || !scan_seen || !marker_prefix(stream, prefix, marker)) {
        return false;
      }
      while (stream_fill(stream)) {
        if (!stream_copy(stream, stream->size - stream->position, true)) {
          return false;
        }
      }
      return !ferror(stream->input) && (stream->output == NULL || !ferror(stream->output));
    }

    if (marker == 1) {
      if (!marker_prefix(stream, prefix, marker)) {
        return false;
      }
      continue;
    }

    const int high = stream_byte(stream);
    const int low = stream_byte(stream);
    if (high == EOF || low == EOF) {
      return false;
    }
    const size_t length = (size_t)high * 256 + (unsigned)low;
    if (length < 2) {
      return false;
    }

    const size_t remaining = length - 2;
    unsigned char signature[6];
    size_t inspected = 0;
    if (marker == 0xe1 && remaining >= sizeof(signature)) {
      if (!stream_read(stream, signature, sizeof(signature))) {
        return false;
      }
      inspected = sizeof(signature);
      if (memcmp(signature, exif_signature, sizeof(exif_signature)) == 0) {
        /* APP1 also carries XMP. Drop only the exact EXIF signature, not every APP1. */
        if (!stream_copy(stream, remaining - inspected, false)) {
          return false;
        }
        if (stream->plan != NULL) {
          struct jpeg_edit_plan *plan = stream->plan;
          if (plan->count == JPEG_MAX_HEADER_SEGMENTS) {
            stream->error = EFBIG;
            errno = EFBIG;
            return false;
          }
          const off_t end = stream_offset(stream);
          if (end < marker_start) {
            return false;
          }
          plan->removals[plan->count++] = (struct jpeg_removal){marker_start, end};
        }
        continue;
      }
    }

    if (!stream_segment_header(stream, prefix, marker, remaining) ||
      (inspected != 0 && !stream_write(stream, signature, inspected))) {
      return false;
    }

    if (frame_marker(marker)) {
      /* This is framing validation, not a JPEG decoder. One bounded frame covers
       * our camera files; hierarchical and delayed-height streams are out of scope. */
      if (frame_seen || !stream_frame_header(stream, remaining, signature) ||
          !stream_write(stream, signature, sizeof(signature))) {
        return false;
      }
      inspected = sizeof(signature);
      frame_seen = true;
    }

    if (marker == 0xda) {
      const int components = stream_byte(stream);
      if (!frame_seen || components < 1 || components > 4 ||
          remaining != 4U + 2U * (unsigned)components || !stream_put_byte(stream, components)) {
        return false;
      }
      inspected = 1;
      inside_scan = true;
      scan_seen = true;
    }

    /* The next iteration may return another SOS, as in a progressive JPEG. */
    if (!stream_copy(stream, remaining - inspected, true)) {
      return false;
    }
  }
}

int exif_rewrite_jpeg(FILE *input, FILE *output, const unsigned char *payload, size_t size)
{
  struct exif_view view;
  /* Validate replacement metadata before touching the output. Original EXIF
   * bodies need only correct segment framing: they are being discarded anyway. */
  if (input == NULL || output == NULL || input == output || exif_parse(&view, payload, size) != 0) {
    return invalid();
  }

  struct jpeg_stream stream = {.input = input, .output = output};
  if (!rewrite_jpeg_stream(&stream, payload, size)) {
    errno = ferror(input) || ferror(output) ? EIO : EINVAL;
    return -1;
  }
  return 0;
}

/** @brief Move one retained interval without overwriting its unread source bytes.
 * @param[in] descriptor Open read/write regular file.
 * @param[in] start Original interval start, inclusive.
 * @param[in] end Original interval end, exclusive.
 * @param[in] displacement Signed destination offset relative to the source.
 * @return Zero on success, otherwise -1 with errno set.
 * @details Growth copies backward; shrinkage copies forward. Positional I/O avoids
 * stream seeks per block. Short transfers and EINTR are handled without assuming
 * that one system call completes the block. A zero-byte transfer is an I/O error.
 */
static int relocate_jpeg_range(int descriptor, off_t start, off_t end, off_t displacement)
{
  if (start < 0 || end < start) {
    return invalid();
  }
  unsigned char buffer[JPEG_STREAM_CAPACITY];
  while (start < end) {
    const uintmax_t remaining = (uintmax_t)end - (uintmax_t)start;
    const size_t chunk = remaining > sizeof(buffer) ? sizeof(buffer) : (size_t)remaining;
    const off_t source = displacement > 0 ? end - (off_t)chunk : start;
    size_t transferred = 0;
    while (transferred < chunk) {
      const size_t requested = chunk - transferred;
        if (requested > sizeof(buffer) || transferred >= sizeof(buffer) ||
          requested > sizeof(buffer) - transferred) {
        errno = EIO;
        return -1;
      }
      const ssize_t count = pread(descriptor, buffer + transferred, requested,
                                  source + (off_t)transferred);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0 || (size_t)count > requested) {
        if (count >= 0) errno = EIO;
        return -1;
      }
      transferred += (size_t)count;
    }
    transferred = 0;
    while (transferred < chunk) {
      const size_t requested = chunk - transferred;
        if (requested > sizeof(buffer) || transferred >= sizeof(buffer) ||
          requested > sizeof(buffer) - transferred) {
        errno = EIO;
        return -1;
      }
      const ssize_t count = pwrite(descriptor, buffer + transferred, requested,
                                   source + displacement + (off_t)transferred);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0 || (size_t)count > requested) {
        if (count >= 0) errno = EIO;
        return -1;
      }
      transferred += (size_t)count;
    }
    if (displacement > 0) {
      end -= (off_t)chunk;
    } else {
      start += (off_t)chunk;
    }
  }
  return 0;
}

/** @brief Move only the spans whose final positions differ from their originals.
 * @param[in] descriptor Open read/write regular file.
 * @param[in] plan Validated ordered removal intervals and insertion location.
 * @param[in] original_size Original file size, including bytes after EOI.
 * @param[in] inserted_size Replacement segment size, including marker and length.
 * @param[in] backwards True for right-shifted spans, false for left-shifted spans.
 * @return Zero on success, otherwise -1 with errno set.
 * @details After the one insertion, displacement only decreases as old EXIF is
 * removed. Right-shifted spans therefore precede left-shifted spans. Copying the
 * former from the end and the latter from the start protects every unread span,
 * even when EXIF occurs between progressive scans and displacement changes sign.
 * Zero-displacement spans need no writes; this is the cheap repeated-update path.
 */
static int relocate_jpeg_spans(int descriptor, const struct jpeg_edit_plan *plan,
                               off_t original_size, off_t inserted_size, bool backwards)
{
  off_t removed_size = 0;
  if (backwards) {
    for (size_t index = 0; index < plan->count; ++index) {
      removed_size += plan->removals[index].end - plan->removals[index].start;
    }
  }
  off_t start = plan->insertion;
  off_t end = original_size;
  for (size_t step = 0; step <= plan->count; ++step) {
    const size_t index = backwards ? plan->count - step : step;
    if (backwards) {
      start = index == 0 ? plan->insertion : plan->removals[index - 1].end;
    } else {
      end = index == plan->count ? original_size : plan->removals[index].start;
    }
    const off_t displacement = inserted_size - removed_size;
    if (((backwards && displacement > 0) || (!backwards && displacement < 0)) &&
        relocate_jpeg_range(descriptor, start, end, displacement) != 0) {
      return -1;
    }
    if (index < plan->count && !backwards) {
      removed_size += plan->removals[index].end - plan->removals[index].start;
      start = plan->removals[index].end;
    } else if (index > 0 && backwards) {
      removed_size -= plan->removals[index - 1].end - plan->removals[index - 1].start;
      end = plan->removals[index - 1].start;
    }
  }
  return 0;
}

int exif_update_jpeg(FILE *file, const unsigned char *payload, size_t size)
{
  struct exif_view view;
  if (file == NULL || exif_parse(&view, payload, size) != 0) {
    return invalid();
  }
  struct stat original;
  if (fstat(fileno(file), &original) != 0) {
    return -1;
  }
  if (!S_ISREG(original.st_mode) || original.st_size < 0 || ftello(file) != 0) {
    return invalid();
  }
  struct jpeg_edit_plan plan = {0};
  struct jpeg_stream stream = {.input = file, .plan = &plan};
  errno = 0;
  if (!rewrite_jpeg_stream(&stream, payload, size)) {
    errno = stream.error ? stream.error : ferror(file) ? EIO : EINVAL;
    return -1;
  }
  struct stat current;
  if (fstat(fileno(file), &current) != 0) {
    return -1;
  }
  if (original.st_size != current.st_size || original.st_mtim.tv_sec != current.st_mtim.tv_sec ||
      original.st_mtim.tv_nsec != current.st_mtim.tv_nsec || ftello(file) != original.st_size) {
    errno = ESTALE;
    return -1;
  }
  off_t removed_size = 0;
  for (size_t index = 0; index < plan.count; ++index) {
    removed_size += plan.removals[index].end - plan.removals[index].start;
  }
  const off_t retained_size = original.st_size - removed_size;
  const uintmax_t offset_max = (UINTMAX_C(1) << (sizeof(off_t) * CHAR_BIT - 1)) - 1;
  const off_t inserted_size = (off_t)(size + 4);
  if ((uintmax_t)retained_size > offset_max - (uintmax_t)inserted_size) {
    errno = EOVERFLOW;
    return -1;
  }
  const off_t final_size = retained_size + inserted_size;
  if (fseeko(file, 0, SEEK_SET) != 0) {
    return -1;
  }
  if (final_size > original.st_size && ftruncate(fileno(file), final_size) != 0) {
    return -1;
  }
  if (relocate_jpeg_spans(fileno(file), &plan, original.st_size, inserted_size, true) != 0 ||
      relocate_jpeg_spans(fileno(file), &plan, original.st_size, inserted_size, false) != 0) {
    return -1;
  }
  const unsigned char header[] = {0xff, 0xe1, (unsigned char)((size + 2) >> 8),
                                (unsigned char)(size + 2)};
  if (fseeko(file, plan.insertion, SEEK_SET) != 0 ||
      fwrite(header, 1, sizeof(header), file) != sizeof(header) ||
      fwrite(payload, 1, size, file) != size || fflush(file) != 0) {
    return -1;
  }
  return final_size < original.st_size ? ftruncate(fileno(file), final_size) : 0;
}