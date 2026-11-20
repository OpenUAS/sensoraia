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
 * @file jsonbourne.h
 * @brief JSON reader, writer and DOM utilities for Linux C11 with a C++11-compatible API.
 *
 * @details
 * Strict parsing validates RFC 8259 grammar and Unicode scalar values in UTF-8.
 * The input need not be terminated, aligned or padded and is never modified.
 * Decoded strings and unchanged numeric tokens are owned by the returned DOM.
 * Duplicate names and their order are preserved; jsonbourne_obj_getn_unique() rejects
 * ambiguous lookups. Embedded NULs are supported through explicit byte lengths.
 *
 * Parsing and cloning make one allocation, released with free() when the default
 * allocator is used. Custom allocators must provide malloc-compatible alignment
 * and own their corresponding release policy. jsonbourne_read_buf(), jsonbourne_read_measure(),
 * jsonbourne_read_validate() and the *_buf writers do not allocate. Input, DOM and output
 * buffers must not overlap and must remain stable throughout each operation.
 * Allocator callbacks must not mutate input or DOM data. Independent calls share
 * no mutable state and may run concurrently.
 *
 * JSONBOURNE_READER_DEPTH_LIMIT bounds reader nesting and fixed traversal storage.
 * JSONBOURNE_WRITER_DEPTH_LIMIT bounds writer/copy traversal and defaults to the
 * reader limit. Both are positive hard limits, initially 999; zero does not
 * mean unlimited. Embedded consumers should select smaller application-specific
 * limits when compiling jsonbourne.c. Runtime limits additionally bound input,
 * total DOM bytes, decoded string bytes and value count before allocation.
 * SIMD is automatic for baseline x86-64 SSE2 and AArch64 NEON; define
 * JSONBOURNE_DISABLE_SIMD=1 for a scalar build. Every vector load is within the
 * supplied input extent. Normal -O2/-O3 suffices; no fast-math or SSSE3 is needed.
 *
 * Writers validate accessible DOM structures, UTF-8, numbers, list lengths and
 * cycles. All output sizes INCLUDE the terminating NUL, matching the original
 * API. Numbers keep their decimal spelling and precision. Relaxed hexadecimal
 * tokens are converted exactly, up to JSONBOURNE_MAX_HEX_DIGITS significant hex
 * digits; larger conversions fail rather than overflow. The legacy relaxed
 * nonfinite writer policy maps NaN to 0 and infinities to signed DBL_MAX text.
 * Compile jsonbourne.c once with the desired build options and link its object.
 * Public names follow Yuan conventions, but the DOM layout, flag bits and
 * writer lengths are JSONbourne-specific rather than Yuan ABI compatible.
 * Pretty-print indentation/newlines must contain JSON whitespace only.
 *
 * @par Why tokens and lengths are preserved
 * Keeping decimal text avoids rounding large integers through binary floating
 * point. Explicit byte lengths distinguish embedded NULs from terminators, and
 * preserving duplicate keys lets the application decide whether ambiguity is
 * acceptable instead of silently discarding input.
 *
 * @pre Input, DOM nodes, linked entries and output storage must refer to valid,
 * accessible memory for their advertised lifetimes and extents. Structural
 * validation detects malformed lists and cycles, not arbitrary invalid pointers.
 * @note Select compile-time options when building jsonbourne.c; defining them
 * only in a client does not change the linked implementation's storage limits.
 * @see jsonbourne_read_buf(), jsonbourne_read_limits_default(), jsonbourne_version()
 */

#ifndef JSONBOURNE_H
#define JSONBOURNE_H

#include <stddef.h>
#include <stdint.h>

/**
 * @defgroup jsonbourne_config Version and Build Configuration
 * @brief Options applied when compiling the implementation translation unit.
 * @{
 */
/** @brief Major API version component. */
#define JSONBOURNE_VERSION_MAJOR 1
/** @brief Minor API version component. */
#define JSONBOURNE_VERSION_MINOR 0
/** @brief Patch version component. */
#define JSONBOURNE_VERSION_PATCH 1
/** @brief Packed major/minor/patch version for compile-time comparisons. */
#define JSONBOURNE_VERSION_HEX                                                                     \
  ((JSONBOURNE_VERSION_MAJOR << 16) | (JSONBOURNE_VERSION_MINOR << 8) | JSONBOURNE_VERSION_PATCH)
/** @brief Human-readable header version; query jsonbourne_version() for the linked object. */
#define JSONBOURNE_VERSION_STRING "1.0.1"

#ifndef JSONBOURNE_READER_DEPTH_LIMIT
/**
 * @brief Maximum simultaneous open reader containers; must be positive.
 * @details A fixed stack makes nesting memory predictable without recursive
 * calls or growth allocations. Smaller application-specific bounds save stack
 * space even when a particular input has little nesting.
 * @see jsonbourne_read_limits::max_depth
 */
#define JSONBOURNE_READER_DEPTH_LIMIT 999
#endif
#ifndef JSONBOURNE_WRITER_DEPTH_LIMIT
/**
 * @brief Positive writer/copy nesting bound, defaulting to the reader bound.
 * @details Separate limits allow a program to inspect deeper input than it is
 * willing to traverse for output or copying, without unbounded stack growth.
 */
#define JSONBOURNE_WRITER_DEPTH_LIMIT JSONBOURNE_READER_DEPTH_LIMIT
#endif
#if JSONBOURNE_READER_DEPTH_LIMIT < 1 || JSONBOURNE_WRITER_DEPTH_LIMIT < 1
#error JSONbourne reader and writer depth limits must be positive
#endif
#ifndef JSONBOURNE_MAX_HEX_DIGITS
/**
 * @brief Maximum significant hexadecimal digits converted to decimal output.
 * @details Bounds exact conversion work and its fixed scratch array; accepted
 * settings are 16 through 16384. Leading zeroes do not consume this limit.
 */
#define JSONBOURNE_MAX_HEX_DIGITS 1024
#endif
#if JSONBOURNE_MAX_HEX_DIGITS < 16 || JSONBOURNE_MAX_HEX_DIGITS > 16384
#error JSONBOURNE_MAX_HEX_DIGITS must be between 16 and 16384
#endif
#ifndef JSONBOURNE_DISABLE_SIMD
/** @brief Set to 1 to exercise the scalar path or omit architecture-specific intrinsics. */
#define JSONBOURNE_DISABLE_SIMD 0
#endif
#ifndef JSONBOURNE_API
/** @brief Optional declaration annotation for a consumer's visibility/export policy. */
#define JSONBOURNE_API
#endif
/** @} */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup jsonbourne_dom DOM Types and Read Options
 * @brief Length-aware views, ordered containers, diagnostics and resource limits.
 * @{
 */
/** @brief Read-option bitmask; combine named options with bitwise OR. */
typedef size_t jsonbourne_read_flag;
/**
 * @brief Independent reader options and convenience presets.
 * @details Presets are unions of feature bits, not distinct parser modes.
 * Use the names rather than persisting raw masks so code can follow flag remaps.
 */
enum jsonbourne_read_flag_e {
  JSONBOURNE_READ_NOFLAG = 0,                       /**< Strict RFC 8259 input. */
  JSONBOURNE_READ_ALLOW_TRAILING_COMMAS = 0x0002,    /**< Permit a comma before container closure. */
  JSONBOURNE_READ_ALLOW_UNQUOTED_KEY = 0x0004,       /**< Permit nonempty ASCII identifier-like keys. */
  JSONBOURNE_READ_ALLOW_GLOBAL_OBJECT = 0x0008,      /**< Permit an object without outer braces. */
  JSONBOURNE_READ_ALLOW_EQUALS = 0x0010,             /**< Permit equals in place of a member colon. */
  JSONBOURNE_READ_ALLOW_NO_COMMAS = 0x0020,          /**< Permit adjacent members/elements without commas. */
  JSONBOURNE_READ_ALLOW_COMMENTS = 0x0040,           /**< Skip C-style block and line comments. */
  JSONBOURNE_READ_RESERVED = 0x0080,                 /**< Accepted reserved bit with no syntax effect. */
  JSONBOURNE_READ_TRACK_LOCATION = 0x0100,           /**< Allocate location extensions for values and keys. */
  JSONBOURNE_READ_ALLOW_SINGLE_QUOTED_STR = 0x0200,  /**< Permit single quotes and escaped apostrophes. */
  JSONBOURNE_READ_ALLOW_HEX_NUM = 0x0400,            /**< Permit hexadecimal integer tokens. */
  JSONBOURNE_READ_ALLOW_PLUS_SIGN = 0x0800,          /**< Permit an explicit positive number sign. */
  JSONBOURNE_READ_ALLOW_DECIMAL_POINT = 0x1000,      /**< Permit .5 and 1. forms. */
  JSONBOURNE_READ_ALLOW_INF_AND_NAN = 0x2000,        /**< Permit signed Infinity and NaN tokens. */
  JSONBOURNE_READ_ALLOW_MULTILINE_STR = 0x4000,      /**< Permit raw CR/LF and escaped line continuations. */
  JSONBOURNE_READ_SIMPLIFIED = 0x003e,               /**< Combine the five object/spacing conveniences. */
  JSONBOURNE_READ_JSON5 = 0x7e46                     /**< JSON5-style conveniences, not full conformance. */
  /* bitmask combining nine parsing options
  0x0002 | 0x0004 | 0x0040 | 0x0200 | 0x0400 |
  0x0800 | 0x1000 | 0x2000 | 0x4000 = 0x7e46
  */
};

/** @brief Payload tag; literals need no separately allocated payload. */
typedef enum jsonbourne_type {
  JSONBOURNE_TYPE_STR,   /**< Payload is jsonbourne_str. */
  JSONBOURNE_TYPE_NUM,   /**< Payload is jsonbourne_num. */
  JSONBOURNE_TYPE_OBJ,   /**< Payload is jsonbourne_obj. */
  JSONBOURNE_TYPE_ARR,   /**< Payload is jsonbourne_arr. */
  JSONBOURNE_TYPE_TRUE,  /**< JSON true; payload is NULL for parsed values. */
  JSONBOURNE_TYPE_FALSE, /**< JSON false; payload is NULL for parsed values. */
  JSONBOURNE_TYPE_NULL   /**< JSON null; payload is NULL for parsed values. */
} jsonbourne_type;

/** @brief Decoded UTF-8 bytes; len, not strlen(), determines the string's extent. */
typedef struct jsonbourne_str {
  const char *str; /**< DOM-owned bytes; parsed strings have an additional trailing NUL. */
  size_t len;      /**< Decoded byte count, excluding the trailing NUL. */
} jsonbourne_str;

/**
 * @brief String view followed by optional source coordinates.
 * @pre Only cast a parsed string/key to this type when TRACK_LOCATION was enabled.
 * @details Keeping the base first preserves ordinary string access while making
 * location storage opt-in rather than charging every node for coordinates.
 */
typedef struct jsonbourne_str_loc {
  struct jsonbourne_str str; /**< Base view; must remain the first member. */
  size_t pos;               /**< Zero-based source byte offset. */
  size_t line;              /**< One-based line number; LF advances the line. */
  size_t col;               /**< Zero-based byte column, not a Unicode character count. */
} jsonbourne_str_loc;

/** @brief Original number spelling, retained so reading never rounds through a double. */
typedef struct jsonbourne_num {
  const char *str; /**< DOM-owned token bytes with an additional trailing NUL after parsing. */
  size_t len;      /**< Token byte count, excluding the trailing NUL. */
} jsonbourne_num;

/** @brief Forward name for values referenced by recursive container entries. */
typedef struct jsonbourne_val jsonbourne_val;

/** @brief Forward name allowing an object entry to link to its own type. */
typedef struct jsonbourne_obj_entry jsonbourne_obj_entry;
/** @brief One object member; linked order and duplicate decoded keys are retained. */
struct jsonbourne_obj_entry {
  jsonbourne_str *key;         /**< Decoded member name. */
  jsonbourne_val *val;         /**< Member value. */
  jsonbourne_obj_entry *next;  /**< Next member, or NULL for the last member. */
};

/** @brief Ordered object members; len must exactly match the linked-list length. */
typedef struct jsonbourne_obj {
  jsonbourne_obj_entry *first; /**< First member; NULL for an empty object. */
  size_t len;                 /**< Number of members, including duplicates. */
} jsonbourne_obj;

/** @brief Forward name allowing an array entry to link to its own type. */
typedef struct jsonbourne_arr_entry jsonbourne_arr_entry;
/** @brief One array element in source order. */
struct jsonbourne_arr_entry {
  jsonbourne_val *val;         /**< Element value. */
  jsonbourne_arr_entry *next;  /**< Next element, or NULL for the last element. */
};

/** @brief Ordered array elements; explicit length supports bounded traversal. */
typedef struct jsonbourne_arr {
  jsonbourne_arr_entry *first; /**< First element; NULL for an empty array. */
  size_t len;                 /**< Exact number of linked elements. */
} jsonbourne_arr;

/**
 * @brief Tagged DOM value; children and text borrow the root allocation's lifetime.
 * @note Release an allocated tree once at its root, never by freeing individual
 * payloads or entries. Caller-buffer roots must not be passed to free().
 */
struct jsonbourne_val {
  void *ptr;    /**< Payload selected by type; parsed literals use NULL. */
  size_t type;  /**< One of the jsonbourne_type enumerators. */
};

/**
 * @brief Value view followed by coordinates recorded only with TRACK_LOCATION.
 * @pre Only use this extension for parsed values created with location tracking.
 */
typedef struct jsonbourne_val_loc {
  struct jsonbourne_val val; /**< Base value; must remain the first member. */
  size_t pos;               /**< Zero-based source byte offset before the value. */
  size_t line;              /**< One-based source line. */
  size_t col;               /**< Zero-based source byte column. */
} jsonbourne_val_loc;

/** @brief Stable storage type for a diagnostic code, separate from display text. */
typedef size_t jsonbourne_read_code;
/** @brief Reader failures; zero denotes success and leaves error coordinates zeroed. */
enum jsonbourne_read_code_e {
  JSONBOURNE_READ_SUCCESS = 0,               /**< No read failure. */
  JSONBOURNE_READ_ERROR_EXPECTED_SEPARATOR,  /**< Missing comma/closure or disallowed trailing comma. */
  JSONBOURNE_READ_ERROR_EXPECTED_COLON,      /**< Missing permitted key/value separator. */
  JSONBOURNE_READ_ERROR_EXPECTED_KEY,        /**< Missing or invalid object member name. */
  JSONBOURNE_READ_ERROR_INVALID_ESCAPE,      /**< Invalid escape or UTF-16 surrogate pairing. */
  JSONBOURNE_READ_ERROR_INVALID_NUMBER,      /**< Invalid number spelling or token boundary. */
  JSONBOURNE_READ_ERROR_LITERAL,             /**< Invalid true, false or null spelling/boundary. */
  JSONBOURNE_READ_ERROR_UNEXPECTED_END,      /**< Input ended before the construct was complete. */
  JSONBOURNE_READ_ERROR_INVALID_STRING,      /**< Disallowed unescaped string control byte. */
  JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION,   /**< Allocation failed or caller storage is insufficient. */
  JSONBOURNE_READ_ERROR_UNEXPECTED_CONTENT,  /**< Non-trivia bytes follow a complete root value. */
  JSONBOURNE_READ_ERROR_DEPTH,               /**< Container nesting exceeds the effective bound. */
  JSONBOURNE_READ_ERROR_INVALID_COMMENT,     /**< Block comment has no closing delimiter. */
  JSONBOURNE_READ_ERROR_UNKNOWN,             /**< Generic diagnostic and unrecognized-code fallback. */
  JSONBOURNE_READ_ERROR_INVALID_UTF8,        /**< Invalid UTF-8 scalar encoding. */
  JSONBOURNE_READ_ERROR_LIMIT,               /**< Resource bound or checked arithmetic was exceeded. */
  JSONBOURNE_READ_ERROR_INVALID_PARAMETER    /**< Invalid flags, limits, input or storage alignment. */
};

/** @brief Optional read result; coordinates describe the parser's failure cursor. */
typedef struct jsonbourne_read_err {
  jsonbourne_read_code code; /**< Success or one of the JSONBOURNE_READ_ERROR_* codes. */
  size_t pos;               /**< Zero-based failure offset, clamped to the input extent. */
  size_t line;              /**< One-based failure line, or zero on success. */
  size_t col;               /**< Zero-based failure byte column, or zero on success. */
} jsonbourne_read_err;

/**
 * @brief Per-read budgets checked during validation, before a DOM is allocated.
 * @details Limits are literal upper bounds: zero permits none of the resource,
 * not an unlimited amount. Start with jsonbourne_read_limits_default() so a
 * partially initialized structure cannot accidentally prohibit useful input.
 */
typedef struct jsonbourne_read_limits {
  size_t max_depth;     /**< Open containers; cannot exceed the compiled reader limit. */
  size_t max_input_len; /**< Entire input extent in bytes, including trivia. */
  size_t max_memory;    /**< Combined DOM record and text bytes, not total process memory. */
  size_t max_vals;      /**< Root plus nested values; member keys are not values. */
  size_t max_str_len;   /**< Decoded bytes in each string/key, excluding its terminator. */
} jsonbourne_read_limits;

/** @brief Numeric spelling extensions; does not by itself permit Infinity or NaN. */
#define JSONBOURNE_READ_ALLOW_EXT_NUMBER                                                           \
  (JSONBOURNE_READ_ALLOW_HEX_NUM | JSONBOURNE_READ_ALLOW_PLUS_SIGN |                               \
   JSONBOURNE_READ_ALLOW_DECIMAL_POINT)
/** @} */

/**
 * @brief Query the linked implementation rather than the including header.
 * @return Packed major/minor/patch version in JSONBOURNE_VERSION_HEX format.
 * @ingroup jsonbourne_config
 */
JSONBOURNE_API uint32_t jsonbourne_version(void);

/**
 * @defgroup jsonbourne_reader Reading and Validation
 * @brief Strict/relaxed reads, allocation-free checks and caller-owned storage.
 * @{
 */
/**
 * @brief Create default budgets using the linked implementation's reader limit.
 * @return Fully initialized limits with pointer-safe byte bounds and no tighter
 * application policy for value count or individual string length.
 * @note These defaults are not a whole-application memory budget; reduce them
 * for inputs that must fit a particular arena or embedded workload.
 */
JSONBOURNE_API struct jsonbourne_read_limits jsonbourne_read_limits_default(void);

/**
 * @brief Validate input and measure the exact single-allocation DOM size.
 * @param[in] src Input bytes; NULL is permitted only when len is zero.
 * @param[in] len Input byte count, including any trailing trivia.
 * @param[in] flg Named read options combined with bitwise OR.
 * @param[in] limits Optional budgets; NULL selects linked defaults.
 * @param[out] required Optional exact allocation size; zero on failure.
 * @param[out] err Optional diagnostic; populated on success and failure.
 * @retval 1 The complete input is valid within all effective limits.
 * @retval 0 Syntax, encoding, arguments or resource checks failed.
 * @note Does not allocate or retain input pointers. The same parser is used for
 * sizing and materialization so resource checks cannot disagree with allocation.
 */
JSONBOURNE_API int jsonbourne_read_measure(const void *src, size_t len, jsonbourne_read_flag flg,
                                           const jsonbourne_read_limits *limits, size_t *required,
                                           jsonbourne_read_err *err);

/**
 * @brief Validate the entire extent without allocating a DOM or changing input.
 * @param[in] src Input bytes; NULL is permitted only for a zero-length extent.
 * @param[in] len Number of input bytes to inspect.
 * @param[in] flg Named read-option mask.
 * @param[in] limits Optional budgets; NULL selects linked defaults.
 * @param[out] err Optional read diagnostic.
 * @retval 1 Valid complete input within the selected budgets.
 * @retval 0 Invalid input, arguments, encoding or resource usage.
 * @see jsonbourne_read_measure()
 */
JSONBOURNE_API int jsonbourne_read_validate(const void *src, size_t len, jsonbourne_read_flag flg,
                                            const jsonbourne_read_limits *limits,
                                            jsonbourne_read_err *err);

/**
 * @brief Read a DOM into aligned caller storage without heap allocation.
 * @param[in] src Immutable input bytes, disjoint from buf.
 * @param[in] len Input extent in bytes.
 * @param[in] flg Named read-option mask.
 * @param[in] limits Optional budgets; NULL selects linked defaults.
 * @param[out] buf Storage with malloc-compatible alignment; NULL probes capacity.
 * @param[in] cap Available bytes at buf.
 * @param[out] required Optional exact size after successful validation, even if
 * storage is too small; zero if validation fails.
 * @param[out] err Optional diagnostic, including storage/alignment failures.
 * @return Root in buf, or NULL on failure; never free this root separately.
 * @note A missing/short buffer reports MEMORY_ALLOCATION and is not written.
 * The arena must outlive all returned nodes, strings and borrowed views.
 */
JSONBOURNE_API jsonbourne_val *jsonbourne_read_buf(const void *src, size_t len,
                                                   jsonbourne_read_flag flg,
                                                   const jsonbourne_read_limits *limits, void *buf,
                                                   size_t cap, size_t *required,
                                                   jsonbourne_read_err *err);

/**
 * @brief Validate against explicit budgets, then allocate the entire DOM once.
 * @param[in] src Immutable input extent; never retained by the returned tree.
 * @param[in] len Input size in bytes.
 * @param[in] flg Named read-option mask.
 * @param[in] limits Optional budgets; NULL selects linked defaults.
 * @param[in] alc_fn Optional allocator called as alc_fn(alc_ctx, byte_count);
 * NULL uses malloc. Returned storage must have malloc-compatible alignment.
 * @param[in] alc_ctx Opaque allocator context, forwarded unchanged.
 * @param[out] err Optional read diagnostic.
 * @return Owning root, or NULL on failure; default-allocated roots use free().
 * @note Custom allocators own their release policy, including rejected storage;
 * no custom deallocator is available on this read interface.
 */
JSONBOURNE_API jsonbourne_val *jsonbourne_read_opts_with_limits(
    const void *src, size_t len, jsonbourne_read_flag flg, const jsonbourne_read_limits *limits,
    void *(*alc_fn)(void *, size_t), void *alc_ctx, jsonbourne_read_err *err);

/**
 * @brief Read with named options and the linked implementation's default limits.
 * @param[in] src Immutable input bytes.
 * @param[in] len Input byte count.
 * @param[in] flg Named read-option mask.
 * @param[in] alc_fn Optional malloc-aligned allocator; NULL uses malloc.
 * @param[in] alc_ctx Opaque allocation context.
 * @param[out] err Optional diagnostic.
 * @return Root owning one allocation, or NULL on failure.
 * @see jsonbourne_read_opts_with_limits() for custom allocator lifetime rules.
 */
JSONBOURNE_API jsonbourne_val *jsonbourne_read_opts(const void *src, size_t len,
                                                    jsonbourne_read_flag flg,
                                                    void *(*alc_fn)(void *, size_t), void *alc_ctx,
                                                    jsonbourne_read_err *err);

/**
 * @brief Read strict JSON with default limits and malloc-backed ownership.
 * @param[in] src Immutable input bytes; no trailing NUL or padding is required.
 * @param[in] len Exact input byte count.
 * @return Owning root released once with free(), or NULL on failure.
 * @see jsonbourne_read_opts() when a diagnostic or relaxed syntax is needed.
 */
JSONBOURNE_API jsonbourne_val *jsonbourne_read(const void *src, size_t len);
/** @} */

/**
 * @defgroup jsonbourne_writer Writing and Copying
 * @brief Bounded iterative traversal with preflighted allocation/capacity checks.
 * @{
 */
/**
 * @brief Validate a DOM and allocate minified JSON output.
 * @param[in] val Accessible root; must remain stable through both passes.
 * @param[in] alc_fn Optional output allocator; NULL uses malloc.
 * @param[in] free_fn Optional matching failure-release callback. If absent,
 * failed output allocations are released with free().
 * @param[in] alc_ctx Opaque context passed to allocation/release callbacks.
 * @param[out] out_len Optional output bytes INCLUDING NUL; zero on failure.
 * @return NUL-terminated output, or NULL for invalid DOM, limits or allocation.
 * @note Successful output belongs to the caller and is never released here.
 * Custom storage must be free-compatible unless free_fn supplies its release.
 */
JSONBOURNE_API void *jsonbourne_val_write_opts(const jsonbourne_val *val,
                                               void *(*alc_fn)(void *, size_t),
                                               void (*free_fn)(void *, void *), void *alc_ctx,
                                               size_t *out_len);

/**
 * @brief Allocate minified JSON using malloc.
 * @param[in] val Accessible, stable DOM root.
 * @param[out] out_len Optional NUL-inclusive output size; zero on failure.
 * @return Output released with free(), or NULL on failure.
 */
JSONBOURNE_API void *jsonbourne_val_write(const jsonbourne_val *val, size_t *out_len);

/**
 * @brief Validate and allocate pretty output with explicit formatting/ownership.
 * @param[in] val Accessible DOM root, stable throughout both passes.
 * @param[in] indent NUL-terminated JSON whitespace; NULL selects two spaces.
 * @param[in] newline NUL-terminated JSON whitespace; NULL selects LF.
 * @param[in] alc_fn Optional output allocator; NULL uses malloc.
 * @param[in] free_fn Optional failure-release callback; NULL falls back to free().
 * @param[in] alc_ctx Opaque allocation/release context.
 * @param[out] out_len Optional NUL-inclusive size; zero on failure.
 * @return Caller-owned output, or NULL on failure.
 * @note Restricting formatting to JSON whitespace keeps pretty output valid JSON.
 * @see jsonbourne_val_write_opts() for custom allocation release rules.
 */
JSONBOURNE_API void *jsonbourne_val_write_pretty_opts(const jsonbourne_val *val, const char *indent,
                                                      const char *newline,
                                                      void *(*alc_fn)(void *, size_t),
                                                      void (*free_fn)(void *, void *),
                                                      void *alc_ctx, size_t *out_len);

/**
 * @brief Allocate pretty JSON using malloc.
 * @param[in] val Accessible, stable DOM root.
 * @param[in] indent JSON whitespace; NULL selects two spaces.
 * @param[in] newline JSON whitespace; NULL selects LF.
 * @param[out] out_len Optional NUL-inclusive size; zero on failure.
 * @return Output released with free(), or NULL on failure.
 */
JSONBOURNE_API void *jsonbourne_val_write_pretty(const jsonbourne_val *val, const char *indent,
                                                 const char *newline, size_t *out_len);

/**
 * @brief Preflight minified output, then write only if the caller buffer fits.
 * @param[in] val Accessible DOM disjoint from the output buffer.
 * @param[out] buf Destination; NULL measures without writing.
 * @param[in] cap Destination capacity in bytes, including room for NUL.
 * @param[out] out_len Optional required NUL-inclusive size; zero for an invalid DOM.
 * @retval 1 Complete output, including NUL, was written.
 * @retval 0 Validation failed or storage is absent/insufficient.
 * @note A valid size probe returns 0 with the required length. Insufficient
 * storage stays untouched because traversal is measured before the first write.
 */
JSONBOURNE_API int jsonbourne_val_write_buf(const jsonbourne_val *val, char *buf, size_t cap,
                                            size_t *out_len);

/**
 * @brief Preflight pretty output without allocating storage.
 * @param[in] val Accessible DOM, stable and disjoint from buf.
 * @param[in] indent JSON whitespace; NULL selects two spaces.
 * @param[in] newline JSON whitespace; NULL selects LF.
 * @param[out] buf Output destination; NULL probes the required size.
 * @param[in] cap Destination extent including terminator space.
 * @param[out] out_len Optional required NUL-inclusive size; zero on validation failure.
 * @retval 1 Complete pretty output was written.
 * @retval 0 Invalid input/formatting or absent/insufficient storage.
 * @see jsonbourne_val_write_buf() for size-probe and untouched-buffer behavior.
 */
JSONBOURNE_API int jsonbourne_val_write_pretty_buf(const jsonbourne_val *val, const char *indent,
                                                   const char *newline, char *buf, size_t cap,
                                                   size_t *out_len);

/**
 * @brief Deep-copy a validated value into one independent allocation.
 * @param[in] val Accessible source DOM; no source storage is borrowed by the copy.
 * @param[in] alc_fn Optional malloc-aligned allocator; NULL uses malloc.
 * @param[in] alc_ctx Opaque allocation context.
 * @return Owning copy, or NULL for invalid DOM, nesting/size limits or allocation.
 * @note Source coordinates are deliberately not copied: they describe the
 * original document, not the storage or provenance of the new tree.
 * Custom allocators retain their own release policy, including rejected storage.
 */
JSONBOURNE_API jsonbourne_val *
jsonbourne_val_copy_opts(const jsonbourne_val *val, void *(*alc_fn)(void *, size_t), void *alc_ctx);

/**
 * @brief Make an independently owned deep copy using malloc.
 * @param[in] val Accessible source root.
 * @return Copy released once with free(), or NULL on failure.
 * @see jsonbourne_val_copy_opts()
 */
JSONBOURNE_API jsonbourne_val *jsonbourne_val_copy(const jsonbourne_val *val);
/** @} */

/**
 * @defgroup jsonbourne_access Access and Conversion
 * @brief Borrowed views, unambiguous lookups and exact integer conversions.
 * @{
 */
/**
 * @brief Borrow a string payload without allocating or copying.
 * @param[in] val Value to inspect; NULL is accepted.
 * @return String view, or NULL for a different tag/NULL value.
 * @note The view shares the source root/arena's lifetime.
 */
JSONBOURNE_API jsonbourne_str *jsonbourne_get_str(jsonbourne_val *val);

/**
 * @brief Borrow the original number-token view.
 * @param[in] val Value to inspect; NULL is accepted.
 * @return Number view, or NULL for a different tag/NULL value.
 * @note No floating-point conversion or ownership transfer occurs.
 */
JSONBOURNE_API jsonbourne_num *jsonbourne_get_num(jsonbourne_val *val);

/**
 * @brief Borrow the ordered object payload.
 * @param[in] val Value to inspect; NULL is accepted.
 * @return Object view, or NULL for a different tag/NULL value.
 */
JSONBOURNE_API jsonbourne_obj *jsonbourne_get_obj(jsonbourne_val *val);

/**
 * @brief Borrow the ordered array payload.
 * @param[in] val Value to inspect; NULL is accepted.
 * @return Array view, or NULL for a different tag/NULL value.
 */
JSONBOURNE_API jsonbourne_arr *jsonbourne_get_arr(jsonbourne_val *val);

/**
 * @brief Test the true tag without coercing numbers or strings.
 * @param[in] val Value to inspect; NULL is accepted.
 * @return 1 only for a non-NULL true-tagged value, otherwise 0.
 */
JSONBOURNE_API int jsonbourne_is_true(const jsonbourne_val *val);

/**
 * @brief Test the false tag without truthiness conversion.
 * @param[in] val Value to inspect; NULL is accepted.
 * @return 1 only for a non-NULL false-tagged value, otherwise 0.
 */
JSONBOURNE_API int jsonbourne_is_false(const jsonbourne_val *val);

/**
 * @brief Distinguish JSON null from a missing C pointer.
 * @param[in] val Value to inspect; NULL itself is not a JSON null node.
 * @return 1 only for a non-NULL null-tagged value, otherwise 0.
 */
JSONBOURNE_API int jsonbourne_is_null(const jsonbourne_val *val);

/**
 * @brief Find the first exact decoded member name, including embedded NUL bytes.
 * @param[in] val Accessible object value.
 * @param[in] key Decoded key bytes; NULL is accepted only when len is zero.
 * @param[in] len Exact key byte count, not a terminator-inclusive size.
 * @return Borrowed first-match value, or NULL for no match/invalid traversal.
 * @note Returns immediately on a match; it does not validate the remaining list
 * or reject duplicates. Use jsonbourne_obj_getn_unique() for configuration keys.
 */
JSONBOURNE_API const jsonbourne_val *jsonbourne_obj_getn(const jsonbourne_val *val, const char *key,
                                                         size_t len);

/**
 * @brief Require exactly one decoded matching key instead of silently choosing.
 * @param[in] val Accessible object value.
 * @param[in] key Decoded key bytes; NULL is accepted only when len is zero.
 * @param[in] len Exact key byte count.
 * @return Borrowed value, or NULL for duplicates, no match or malformed traversal.
 * @note Comparing decoded bytes catches aliases such as "a" and "\u0061".
 */
JSONBOURNE_API const jsonbourne_val *jsonbourne_obj_getn_unique(const jsonbourne_val *val,
                                                                const char *key, size_t len);

/**
 * @brief Locate a zero-based array element by bounded linked traversal.
 * @param[in] val Accessible array value.
 * @param[in] idx Zero-based element index.
 * @return Borrowed element, or NULL for invalid input/index/traversal.
 * @note Lookup is linear in idx and does not validate the unvisited suffix.
 */
JSONBOURNE_API const jsonbourne_val *jsonbourne_arr_get(const jsonbourne_val *val, size_t idx);

/**
 * @brief Convert an unsigned decimal integer token exactly, without coercion.
 * @param[in] num Token view; signs, fractions, exponents and leading zeroes are rejected.
 * @param[out] out Required destination, written only on success.
 * @retval 1 The integer fits uint64_t and was stored.
 * @retval 0 Invalid token/arguments or overflow; out remains unchanged.
 * @note Checking before each multiply/add avoids floating-point rounding and
 * unsigned wraparound at UINT64_MAX.
 */
JSONBOURNE_API int jsonbourne_num_to_uint(const jsonbourne_num *num, uint64_t *out);

/**
 * @brief Convert a signed decimal integer token exactly, including INT64_MIN.
 * @param[in] num Decimal integer spelling with an optional minus, but no plus.
 * @param[out] out Required destination, written only on success.
 * @retval 1 The integer fits int64_t and was stored.
 * @retval 0 Invalid token/arguments or overflow; out remains unchanged.
 * @note The negative range has one extra magnitude; conversion avoids negating
 * INT64_MIN or overflowing an intermediate signed value.
 */
JSONBOURNE_API int jsonbourne_num_to_sint(const jsonbourne_num *num, int64_t *out);

/**
 * @brief Resolve an RFC 6901 pointer without allocating decoded path tokens.
 * @param[in] val Accessible root to query.
 * @param[in] pointer Pointer bytes, not URI-fragment notation; NULL only for len zero.
 * @param[in] len Exact pointer byte count; zero selects the root itself.
 * @return Borrowed result, or NULL for invalid syntax, ambiguity or a missing path.
 * @note Object names must match uniquely after ~0/~1 decoding. Array indices
 * are checked decimal integers without leading zeroes; '-' is not an element.
 */
JSONBOURNE_API const jsonbourne_val *jsonbourne_ptr_getn(const jsonbourne_val *val,
                                                         const char *pointer, size_t len);

/**
 * @brief Borrow human-readable diagnostic text without allocating.
 * @param[in] code Read code; unrecognized codes select the generic diagnostic.
 * @return Static NUL-terminated text that must not be modified or freed.
 * @note Branch on numeric codes, not these intentionally editable messages.
 */
JSONBOURNE_API const char *jsonbourne_read_err_desc(jsonbourne_read_code code);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
