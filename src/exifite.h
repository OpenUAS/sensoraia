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
 * @file exifite.h
 * @brief Bounded C11 EXIF metadata and JPEG streaming interfaces.
 * @details EXIFite covers Sensoraia's metadata without a general-purpose TIFF
 * object graph. Callers own the storage so memory use stays predictable on MORA.
 * Readers borrow bytes; writers collect little-endian values before producing
 * one JPEG APP1 payload. No function decodes or recompresses image pixels.
 * @see exifite.c
 */

#ifndef USHER_EXIFITE_H
#define USHER_EXIFITE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup exifite EXIFite Metadata
 * @brief Small, caller-owned EXIF documents and borrowed metadata views.
 * @details Use separate storage for concurrent operations. The core has no
 * mutable global state; sharing a document or stream still needs caller-side
 * synchronization. File publication and path ownership belong to image_exif.c.
 * @{
 */

/** @brief APP1 payload ceiling: the JPEG length includes its own two bytes.
 * @details This includes the six-byte EXIF signature, not the JPEG marker. */
#define EXIF_MAX_PAYLOAD 65533U
/** @brief Writer entry budget, including the standard initialization tags.
 * @details A small fixed table keeps sorting and lookup bounded without a heap. */
#define EXIF_MAX_ENTRIES 48U

/** @brief Directory identities, not on-disk tag numbers or offsets.
 * @details TIFF calls a directory an IFD (Image File Directory). Tag numbers
 * are meaningful within their directory, so always keep the pair together. */
enum exif_directory { EXIF_PRIMARY, EXIF_CAMERA, EXIF_GPS, EXIF_DIRECTORY_COUNT };

/** @brief TIFF field-type numbers used by Sensoraia's metadata.
 * @details These are wire-format identifiers; do not renumber them. RATIONAL
 * and SRATIONAL each store a numerator and denominator, not a floating value. */
enum exif_format {
  EXIF_BYTE = 1,
  EXIF_ASCII = 2,
  EXIF_SHORT = 3,
  EXIF_LONG = 4,
  EXIF_RATIONAL = 5,
  EXIF_UNDEFINED = 7,
  EXIF_SLONG = 9,
  EXIF_SRATIONAL = 10
};

/** @brief Standard tag identifiers used by the current applications.
 * @details GPS identifiers deliberately reuse small numbers: the directory
 * argument distinguishes GPS tags from primary and camera tags. */
enum exif_tag {
  EXIF_DOCUMENT_NAME = 0x010d,
  EXIF_IMAGE_DESCRIPTION = 0x010e,
  EXIF_ORIENTATION = 0x0112,
  EXIF_X_RESOLUTION = 0x011a,
  EXIF_Y_RESOLUTION = 0x011b,
  EXIF_RESOLUTION_UNIT = 0x0128,
  EXIF_SOFTWARE = 0x0131,
  EXIF_EXPOSURE_TIME = 0x829a,
  EXIF_FNUMBER = 0x829d,
  EXIF_ISO_SPEED = 0x8827,
  EXIF_VERSION = 0x9000,
  EXIF_COMPONENTS_CONFIGURATION = 0x9101,
  EXIF_EXPOSURE_BIAS = 0x9204,
  EXIF_FOCAL_LENGTH = 0x920a,
  EXIF_USER_COMMENT = 0x9286,
  EXIF_FLASHPIX_VERSION = 0xa000,
  EXIF_COLOR_SPACE = 0xa001,
  EXIF_PIXEL_X_DIMENSION = 0xa002,
  EXIF_PIXEL_Y_DIMENSION = 0xa003,
  EXIF_BODY_SERIAL_NUMBER = 0xa431,
  EXIF_LENS_MODEL = 0xa434,

  EXIF_GPS_VERSION = 0,
  EXIF_GPS_LATITUDE_REF = 1,
  EXIF_GPS_LATITUDE = 2,
  EXIF_GPS_LONGITUDE_REF = 3,
  EXIF_GPS_LONGITUDE = 4,
  EXIF_GPS_ALTITUDE_REF = 5,
  EXIF_GPS_ALTITUDE = 6,
  EXIF_GPS_SPEED_REF = 12,
  EXIF_GPS_SPEED = 13,
  EXIF_GPS_TRACK_REF = 14,
  EXIF_GPS_TRACK = 15,
  EXIF_GPS_DIRECTION_REF = 16,
  EXIF_GPS_DIRECTION = 17
};

/** @brief Borrowed bytes plus enough type information for safe interpretation.
 * @details This does not own or convert the value. Prefer the typed accessors
 * over casting the bytes; camera files may be unaligned or use another byte order. */
struct exif_value {
  const unsigned char *data; /**< Borrowed field bytes, including any text terminator. */
  size_t size;               /**< Accessible byte count, distinct from the component count. */
  uint32_t count;            /**< Number of TIFF components; one rational occupies eight bytes. */
  uint16_t format;           /**< TIFF field type; check it before interpreting text or numbers. */
  bool little_endian;        /**< Byte order of multi-byte components in @ref exif_value::data. */
};

/** @brief Borrowed view of a validated APP1 payload.
 * @details Keep the source bytes alive and unchanged while using this view.
 * Offsets are relative to the TIFF header, not the JPEG or EXIF signature;
 * keeping that origin explicit prevents a common six-byte addressing mistake. */
struct exif_view {
  const unsigned char *tiff; /**< TIFF header immediately after the EXIF signature. */
  size_t size;               /**< Accessible TIFF bytes; excludes the six-byte EXIF signature. */
  uint32_t directories[EXIF_DIRECTORY_COUNT]; /**< TIFF-relative offsets; zero means absent. */
  bool little_endian;                         /**< Byte order declared by the TIFF header. */
};

/** @brief Writer bookkeeping for one field inside the document's value storage.
 * @details Offsets, rather than pointers, let removal compact the storage and
 * serialization reorder entries without building an allocated object graph.
 * @warning Treat these fields as implementation bookkeeping, not an editing API. */
struct exif_item {
  uint32_t offset;         /**< Starting byte within the document's packed value storage. */
  uint32_t size;           /**< Value byte length used when checking and reclaiming storage. */
  uint32_t count;          /**< Number of TIFF components, not bytes. */
  uint16_t tag;            /**< Tag number interpreted within the stored directory. */
  uint16_t format;         /**< TIFF representation determining each component's width. */
  unsigned char directory; /**< One of the exif_directory identities. */
};

/** @brief Caller-owned bounded writer with no heap allocation.
 * @details Initialize before use. Large documents are usually best kept in a
 * caller workspace rather than a deeply nested stack frame. All stored numeric
 * values are little-endian, giving serialization a single predictable path.
 * @warning Any mutation may invalidate previously borrowed field pointers. */
struct exif_document {
  struct exif_item entries
      [EXIF_MAX_ENTRIES]; /**< Active fields occupy the first @ref exif_document::count slots. */
  size_t count;           /**< Number of active entries in the fixed entry table. */
  size_t used;            /**< Occupied packed-value bytes, excluding directory overhead. */
  unsigned char
      data[EXIF_MAX_PAYLOAD]; /**< Packed values; serialized directory overhead is separate. */
};

/** @brief Validate an EXIF APP1 payload and create a borrowed view.
 * @param[out] view Destination view; cleared on failure when non-NULL.
 * @param[in] payload Complete APP1 payload starting with the six-byte EXIF signature.
 * @param[in] size Accessible payload bytes, at most EXIF_MAX_PAYLOAD.
 * @return Zero on success, otherwise -1 with errno set to EINVAL.
 * @details Validation happens before exposing a usable view. This rejects
 * invalid offsets, duplicate tags, overlapping directory tables, malformed
 * ASCII and zero rational denominators. Thumbnail and vendor pointers are not
 * followed: they add complexity without serving Sensoraia's metadata needs.
 * @pre The destination view must not overlap the source payload.
 * @note Structural validity does not establish the physical meaning of a tag.
 * @see exif_get
 */
int exif_parse(struct exif_view *view, const unsigned char *payload, size_t size);
/** @brief Look up a directory-local tag without copying its bytes.
 * @param[in] view View returned by exif_parse(), with unchanged backing storage.
 * @param[in] directory Directory in which to look for the tag.
 * @param[in] tag Standard tag identifier within that directory.
 * @param[out] value Borrowed field descriptor; use it only when true is returned.
 * @return True when a valid value is available, otherwise false.
 * @details Lookup accepts unsorted input directories because external camera
 * metadata need not share our writer's ordering. Missing tags are normal.
 */
bool exif_get(const struct exif_view *view, enum exif_directory directory, uint16_t tag,
              struct exif_value *value);
/** @brief Decode one BYTE, SHORT or LONG component without an unaligned load.
 * @param[in] value Borrowed field descriptor from a lookup.
 * @param[in] index Zero-based component index, not a byte offset.
 * @param[out] result Decoded unsigned value; unchanged on failure.
 * @return True for a supported, in-bounds component, otherwise false.
 */
bool exif_value_unsigned(const struct exif_value *value, size_t index, uint32_t *result);
/** @brief Decode one unsigned or signed rational component as a double.
 * @param[in] value RATIONAL or SRATIONAL field descriptor.
 * @param[in] index Zero-based rational index; each rational occupies eight bytes.
 * @param[out] result Numerator divided by denominator; unchanged on failure.
 * @return True for a valid component with a nonzero denominator, otherwise false.
 * @details Signed conversion handles the full 32-bit range without relying on
 * implementation-defined unsigned-to-signed integer casts.
 */
bool exif_value_rational(const struct exif_value *value, size_t index, double *result);
/** @brief Count text bytes after trimming trailing NUL padding only.
 * @param[in] value ASCII or UNDEFINED field descriptor.
 * @return Trimmed byte length, or zero for absent/unsupported values.
 * @details Embedded NULs and the UserComment encoding prefix are retained.
 * This is a byte-length helper, not a character decoder or strlen replacement.
 */
size_t exif_text_length(const struct exif_value *value);

/** @brief Start a fresh upright sRGB/Y-Cb-Cr document with EXIF 2.32 defaults.
 * @param[out] document Caller-owned writer storage; NULL is ignored.
 * @details Standard housekeeping tags help ordinary EXIF readers interpret the
 * result. Initialization deliberately invents no camera pose, capture time or
 * image dimensions. Set actual dimensions and component configuration from the
 * JPEG header before publication; grayscale needs a one-component configuration.
 * @warning Reinitialization discards previous tags and invalidates borrowed values.
 */
void exif_document_init(struct exif_document *document);
/** @brief Borrow a field from the current writable document.
 * @param[in] document Initialized document that remains unchanged during the read.
 * @param[in] directory Directory containing the desired tag.
 * @param[in] tag Directory-local tag identifier.
 * @param[out] value Borrowed little-endian field descriptor.
 * @return True when the field is available, otherwise false.
 * @warning Copy needed source bytes elsewhere before any document mutation.
 */
bool exif_document_get(const struct exif_document *document, enum exif_directory directory,
                       uint16_t tag, struct exif_value *value);
/** @brief Rebuild a writable document from supported Sensoraia fields.
 * @param[out] document Caller-owned destination; previous contents are replaced.
 * @param[in] view Validated borrowed view with nonoverlapping backing storage.
 * @return Zero on success, otherwise -1 with errno set to EINVAL.
 * @details Import starts from standard defaults, copies only the supported tag
 * set and normalizes numeric byte order. It intentionally drops thumbnails,
 * MakerNotes and unknown fields instead of promising unsafe vendor preservation.
 * @note Invalid arguments leave the document untouched. If copying has begun
 * and fails, the document is reset to defaults, not left partially imported.
 */
int exif_document_import(struct exif_document *document, const struct exif_view *view);
/** @brief Caller policy for additional, directory-local fields to import.
 * @details Called only for fields outside the standard import set. The callback
 * must not mutate the view or destination; context is borrowed for the call. */
typedef bool (*exif_tag_selector)(void *context, enum exif_directory directory, uint16_t tag);
/** @brief Import standard fields plus additional tags selected by the caller.
 * @param document Destination with the same ownership/rollback rules as exif_document_import.
 * @param view Validated source with nonoverlapping backing storage.
 * @param select Optional selector for additional fields; NULL uses standard policy only.
 * @param context Borrowed selector state, or NULL.
 * @return Zero on success, otherwise -1; an interrupted import resets to defaults.
 * @details Numeric values are normalized to little-endian; opaque bytes are
 * unchanged. The caller owns custom tag meaning and relocation safety. Do not
 * select vendor pointer structures or MakerNotes containing hidden TIFF offsets.
 * There is no registration, global policy or retained callback state. */
int exif_document_import_custom(struct exif_document *document, const struct exif_view *view,
                                exif_tag_selector select, void *context);
/** @brief Remove a tag and reclaim its packed value storage.
 * @param[in,out] document Initialized writable document.
 * @param[in] directory Directory containing the tag.
 * @param[in] tag Directory-local identifier to remove.
 * @return Zero on removal or an absent tag, otherwise -1 with errno set to EINVAL.
 * @details Compaction avoids storage growth when applications repeatedly replace
 * provenance records. Treat earlier borrowed pointers as invalid afterwards.
 */
int exif_document_remove(struct exif_document *document, enum exif_directory directory,
                         uint16_t tag);
/** @brief Create or replace a field with zeroed, bounded value storage.
 * @param[in,out] document Initialized writable document.
 * @param[in] directory Destination directory.
 * @param[in] tag Directory-local tag; structural directory pointers are reserved.
 * @param[in] format TIFF representation for the value.
 * @param[in] count Positive component count, not a byte length.
 * @return Writable value bytes, or NULL with errno set to EINVAL or ENOSPC.
 * @details Capacity is checked before removing the old field, so an ordinary
 * capacity failure retains it. Fill numeric components in little-endian order,
 * terminate ASCII and supply nonzero rational denominators before serialization.
 * @warning Reservation can move existing values. Do not use old borrowed bytes
 * as a source afterwards. The returned pointer is borrowed until the next mutation.
 * @see exif_encode_short
 * @see exif_encode_long
 */
unsigned char *exif_document_reserve(struct exif_document *document, enum exif_directory directory,
                                     uint16_t tag, enum exif_format format, uint32_t count);
/** @brief Replace an ASCII field, including its terminating NUL byte.
 * @param[in,out] document Initialized writable document.
 * @param[in] directory Destination directory.
 * @param[in] tag Directory-local ASCII tag identifier.
 * @param[in] text Accessible NUL-terminated text outside the document storage.
 * @return Zero on success, otherwise -1 with errno set to EINVAL or ENOSPC.
 * @details Scanning and copying are bounded by the APP1 limit. This stores bytes;
 * callers are responsible for supplying text appropriate to an ASCII EXIF tag.
 */
int exif_document_ascii(struct exif_document *document, enum exif_directory directory, uint16_t tag,
                        const char *text);
/** @brief Store an unsigned measurement as an exact numerator/denominator pair.
 * @param[in,out] document Initialized writable document.
 * @param[in] directory Destination directory.
 * @param[in] tag Directory-local RATIONAL tag identifier.
 * @param[in] numerator Unsigned numerator in the tag's chosen units.
 * @param[in] denominator Nonzero scale, such as 1000 for millimeter precision.
 * @return Zero on success, otherwise -1 with errno set to EINVAL or ENOSPC.
 * @details Keeping the integer pair avoids an unnecessary floating-point round
 * trip. Use reservation plus integer encoders for signed rational fields.
 */
int exif_document_rational(struct exif_document *document, enum exif_directory directory,
                           uint16_t tag, uint32_t numerator, uint32_t denominator);
/** @brief Encode a 16-bit value in the writer's little-endian representation.
 * @param[out] data At least two writable bytes, normally reserved field storage.
 * @param[in] value Unsigned value to encode.
 * @pre The caller has already checked the pointer and available capacity.
 * @details Byte stores work on unaligned addresses and on either host byte order.
 */
void exif_encode_short(unsigned char *data, uint16_t value);
/** @brief Encode a 32-bit value in the writer's little-endian representation.
 * @param[out] data At least four writable bytes, normally reserved field storage.
 * @param[in] value Unsigned value or the bit representation of a signed component.
 * @pre The caller has already checked the pointer and available capacity.
 * @details Signed rational callers may convert int32_t to uint32_t before encoding;
 * that conversion preserves the required modulo-2^32 representation.
 */
void exif_encode_long(unsigned char *data, uint32_t value);
/** @brief Serialize a document into one complete EXIF APP1 payload.
 * @param[in] document Initialized, fully populated writable document.
 * @param[out] payload Writable storage outside the document, starting at the EXIF signature.
 * @param[in] capacity Available bytes in @p payload.
 * @param[out] size Published payload length; zero on failure with valid, distinct storage.
 * @return Zero on success, otherwise -1 with errno set to EINVAL or ENOSPC.
 * @details Validation and sizing finish before output bytes are written. Directory
 * overhead and alignment must fit APP1 too, not just the document's value storage.
 * Input entries are not reordered; a bounded index table controls wire ordering.
 * @pre The size output must not overlap the document or payload. Invalid alias
 * arguments are rejected without clearing that output, to avoid corrupting input.
 */
int exif_serialize(const struct exif_document *document, unsigned char *payload, size_t capacity,
                   size_t *size);
/** @brief Read the single EXIF APP1 segment before the first JPEG scan.
 * @param[in] filename NUL-terminated path to a readable JPEG.
 * @param[out] payload Caller-owned writable payload buffer.
 * @param[in] capacity Available bytes in the payload buffer.
 * @param[out] size Payload length on success, otherwise zero when non-NULL.
 * @return Zero on success, otherwise -1 with errno describing the failure.
 * @details Metadata lookup stops before compressed image data to keep reads
 * cheap. Duplicate EXIF is rejected rather than guessing which record is authoritative.
 * Header work is bounded by marker and segment-byte limits.
 * @pre The size output must not overlap the payload buffer.
 * @note ENODATA means no EXIF; ENOSPC means insufficient output space. Structural
 * errors use EINVAL and excessive segment bytes use EFBIG. File errors are also possible.
 * @warning This extracts bytes, not a validated TIFF view or a fully validated JPEG.
 * Call exif_parse() before interpreting the payload; ignore its contents on failure.
 */
int exif_read_jpeg(const char *filename, unsigned char *payload, size_t capacity, size_t *size);

/** @brief Read capture EXIF from an already-open JPEG without reopening its path.
 * @param input Borrowed readable stream positioned at SOI; position changes.
 * @param payload Caller-owned bounded output, nonoverlapping with size.
 * @param capacity Available payload bytes.
 * @param size Payload length on success; zero on failure when writable.
 * @return Zero on success; -1 with errno, including ENODATA for absent EXIF.
 * @details The caller owns rewind, file-identity checks and closing. */
int exif_read_stream(FILE *input, unsigned char *payload, size_t capacity, size_t *size);
/** @brief Inspect the frame header without allocating or decoding image pixels.
 * @param[in,out] input Readable stream positioned at the JPEG start-of-image marker.
 * @param[out] width Actual frame width in pixels, zero on failure with valid arguments.
 * @param[out] height Actual frame height in pixels, zero on failure with valid arguments.
 * @param[out] components Actual frame component count, from one through four.
 * @return Zero on success, otherwise -1 with errno set to EINVAL or EFBIG.
 * @details Frame dimensions, not old EXIF tags, describe the compressed image.
 * Buffered reads may advance past the inspected header; rewind before reuse.
 * @pre Use distinct output variables; all must be valid writable pointers.
 * @note No scan data is decoded. Delayed-height frames are outside this interface.
 */
int exif_jpeg_dimensions(FILE *input, uint32_t *width, uint32_t *height, unsigned *components);
/** @brief Stream a JPEG with one new EXIF segment instead of its old EXIF records.
 * @param[in,out] input Readable stream positioned at JPEG start-of-image.
 * @param[in,out] output Distinct writable stream for the replacement JPEG.
 * @param[in] payload Complete, valid EXIF APP1 payload outside both stream buffers.
 * @param[in] size Accessible payload byte count, at most EXIF_MAX_PAYLOAD.
 * @return Zero on successful streaming, otherwise -1 with errno set to EINVAL or EIO.
 * @details Unrelated segments, marker fill bytes and compressed bytes are retained.
 * Framing is checked through end-of-image without a full-image buffer, so entropy
 * decoding, pixel validity and camera-specific metadata are intentionally separate.
 * @pre Input and output must refer to different underlying files as well as different streams.
 * @warning Failure can leave partial output: discard it, never publish it. Success
 * does not flush, sync, close or rename the destination; publication belongs to the caller.
 * @see image_exif_write_complete
 */
int exif_rewrite_jpeg(FILE *input, FILE *output, const unsigned char *payload, size_t size);

/** @brief Replace EXIF in a regular JPEG file without a temporary file.
 * @param[in,out] file Read/write binary stream positioned at byte zero.
 * @param[in] payload Complete valid EXIF APP1 payload outside the stream buffers.
 * @param[in] size Accessible payload byte count, at most EXIF_MAX_PAYLOAD.
 * @return Zero on success, otherwise -1 with errno set.
 * @details The entire framing is validated before mutation. Bounded block moves
 * retain compressed bytes, unrelated markers and trailing data. Equal-size spans
 * are not rewritten. No JPEG-sized memory or disk copy is created.
 * @pre The caller exclusively owns the file and any hard-linked aliases.
 * @warning Not atomic: write failures or interruption after mutation starts can
 * leave a damaged JPEG. Success flushes but does not sync or close the file.
 */
int exif_update_jpeg(FILE *file, const unsigned char *payload, size_t size);

/** @} */

#ifdef __cplusplus
}
#endif
#endif