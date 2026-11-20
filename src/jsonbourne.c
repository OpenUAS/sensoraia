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
 * @file jsonbourne.c
 * @brief Bounded, two-pass JSON parsing and iterative DOM operations.
 * @details Validation and materialization deliberately share the same grammar.
 * A counting pass establishes exact record/text requirements before one backing
 * allocation is split into aligned records and byte strings. Writers and copies
 * likewise reuse a single walker so their sizing and output passes agree.
 *
 * @par Implementation invariants
 * Cursors never exceed their extents; arithmetic is checked before advancing.
 * A NULL pool/output base means measurement, not allocation failure. Fixed
 * traversal frames replace recursion, and all per-call state remains local.
 * The API still requires accessible, stable input and DOM pointers: structural
 * checks cannot make arbitrary addresses safe to dereference.
 * @see jsonbourne.h for public ownership, argument and return-value contracts.
 */
#include "jsonbourne.h"
#include <stdlib.h>
#include <string.h>

/* Don't like weird error messages? I do, that's my prerogative.
 * Hey, you have the code given, change it as you see fit.
 */

/**
 * @name Reader diagnostic text
 * @brief Display strings kept separate from machine-readable error codes.
 * @details Centralizing text lets wording change without touching control flow.
 * Every macro remains one source line so each message has one editing location.
 * @{
 */
/** @brief Message for successful validation. */
#define JSONBOURNE_READ_SUCCESS_MSG "JSONbourne found zero errors and is now insufferable at parties."
/** @brief Missing container separator/closure diagnostic. */
#define JSONBOURNE_READ_ERROR_EXPECTED_SEPARATOR_MSG "The comma and closing bracket have both rage-quit this syntax rave."
/** @brief Missing key/value separator diagnostic. */
#define JSONBOURNE_READ_ERROR_EXPECTED_COLON_MSG "This key and value eloped without filing the mandatory colon paperwork."
/** @brief Invalid member-name diagnostic. */
#define JSONBOURNE_READ_ERROR_EXPECTED_KEY_MSG "That member name needs quotes, not a fake moustache and a convincing backstory."
/** @brief Invalid escape/surrogate diagnostic. */
#define JSONBOURNE_READ_ERROR_INVALID_ESCAPE_MSG "That string escape tried to clear Unicode customs with a counterfeit surrogate."
/** @brief Invalid numeric-token diagnostic. */
#define JSONBOURNE_READ_ERROR_INVALID_NUMBER_MSG "That alleged number has been banned from every calculator in three dimensions."
/** @brief Invalid true/false/null token diagnostic. */
#define JSONBOURNE_READ_ERROR_LITERAL_MSG "Wishful thinking in a trench coat is still not a JSON value."
/** @brief Premature end-of-input diagnostic. */
#define JSONBOURNE_READ_ERROR_UNEXPECTED_END_MSG "Your input rolled the end credits while the document was still introducing itself."
/** @brief Disallowed string control-byte diagnostic. */
#define JSONBOURNE_READ_ERROR_INVALID_STRING_MSG "An unescaped control character is rearranging the furniture inside your string."
/** @brief Allocation failure or insufficient caller-storage diagnostic. */
#define JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION_MSG "Your DOM ordered a penthouse and the allocator delivered a shoebox with no floor."
/** @brief Non-trivia input remaining after the root diagnostic. */
#define JSONBOURNE_READ_ERROR_UNEXPECTED_CONTENT_MSG "Your document is finished but the trailing input has launched an unsolicited podcast."
/** @brief Nesting-limit diagnostic. */
#define JSONBOURNE_READ_ERROR_DEPTH_MSG "Your containers nested past the depth limit and filed for their own postcode."
/** @brief Unterminated block-comment diagnostic. */
#define JSONBOURNE_READ_ERROR_INVALID_COMMENT_MSG "Your block comment swallowed the entire document while waiting for its missing */."
/** @brief Generic diagnostic, also used for unrecognized codes. */
#define JSONBOURNE_READ_ERROR_UNKNOWN_MSG "Your JSON structure belongs in a modern-art museum, not in this parser."
/** @brief Invalid Unicode scalar encoding diagnostic. */
#define JSONBOURNE_READ_ERROR_INVALID_UTF8_MSG "UTF-8 cannot decode whatever experimental keyboard jazz these bytes are performing."
/** @brief Resource-budget or arithmetic-limit diagnostic. */
#define JSONBOURNE_READ_ERROR_LIMIT_MSG "Your resource or arithmetic limit refuses to finance infinity's latest shopping spree."
/** @brief Invalid API arguments or buffer-layout diagnostic. */
#define JSONBOURNE_READ_ERROR_INVALID_PARAMETER_MSG "Your arguments, flags or buffer layout have turned this call into a constitutional crisis."
/** @} */

/** @brief Derive the accepted bits from named options so flag remaps cannot stale this mask. */
#define JSONBOURNE_READ_FLAG_MASK                                                                  \
  (JSONBOURNE_READ_SIMPLIFIED | JSONBOURNE_READ_JSON5 | JSONBOURNE_READ_RESERVED |                    \
   JSONBOURNE_READ_TRACK_LOCATION)
/** @brief C11 aggregate zero-initializer without byte-copying object representations. */
#define JSONBOURNE_ZERO {0}
/** @brief Required C11 alignment for an arena's typed record region. */
#define JSONBOURNE_ALIGNOF(type) _Alignof(type)

#if !JSONBOURNE_DISABLE_SIMD && defined(__SSE2__)
#include <emmintrin.h>
/** @brief Compile only baseline SSE2 scanning intrinsics on compatible x86 builds. */
#define JSONBOURNE_HAS_SSE2 1
#elif !JSONBOURNE_DISABLE_SIMD && defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
/** @brief Compile AArch64 NEON scanning without imposing extra host CPU flags. */
#define JSONBOURNE_HAS_NEON 1
#endif

uint32_t jsonbourne_version(void) { return JSONBOURNE_VERSION_HEX; }

struct jsonbourne_read_limits jsonbourne_read_limits_default(void) {
  struct jsonbourne_read_limits limits;
  /* Pointer-safe defaults are ceilings, not a promise of available RAM. */
  limits.max_depth = JSONBOURNE_READER_DEPTH_LIMIT;
  limits.max_input_len = (size_t)PTRDIFF_MAX;
  limits.max_memory = (size_t)PTRDIFF_MAX;
  limits.max_vals = SIZE_MAX;
  limits.max_str_len = (size_t)PTRDIFF_MAX;
  return limits;
}

/**
 * @brief Two-region arena shared by measurement and materialization.
 * @details Fixed-size DOM records are kept together so variable-length text
 * cannot misalign the next record. NULL bases count bytes without writing them.
 */
struct jsonbourne_pool {
  unsigned char *val_pool;  /**< Record region, or NULL while measuring. */
  char *str_pool;           /**< Text region, or NULL while measuring. */
  size_t val_len;           /**< Record bytes counted/reserved so far. */
  size_t str_len;           /**< Text bytes counted/reserved, including terminators. */
  size_t val_cap;           /**< Maximum record bytes available in this pass. */
  size_t str_cap;           /**< Maximum text bytes available in this pass. */
  size_t max_memory;        /**< Shared bound across both regions. */
  jsonbourne_read_code code; /**< Arena failure carried separately from grammar errors. */
};

/**
 * @brief Reserve checked space in one arena region or merely account for it.
 * @param[in,out] pool Arena counters, capacities and sticky error state.
 * @param[in] len Bytes required, including any caller-supplied terminator space.
 * @param[in] is_str Nonzero selects text; zero selects typed records.
 * @return Region address, or NULL while measuring/on failure; inspect pool->code
 * to distinguish a successful count from an error.
 * @details Subtractions are guarded before use so neither capacity checks nor
 * the combined-region budget can wrap around size_t.
 */
static inline void *jsonbourne_pool_take(struct jsonbourne_pool *pool, size_t len, int is_str) {
  size_t *used = is_str ? &pool->str_len : &pool->val_len;
  const size_t other = is_str ? pool->val_len : pool->str_len;
  const size_t cap = is_str ? pool->str_cap : pool->val_cap;
  void *out = NULL;
  if (*used > cap || len > cap - *used || other > pool->max_memory ||
      *used > pool->max_memory - other || len > pool->max_memory - other - *used) {
    pool->code = JSONBOURNE_READ_ERROR_LIMIT;
    return NULL;
  }
  if (is_str && pool->str_pool)
    out = pool->str_pool + *used;
  if (!is_str && pool->val_pool)
    out = pool->val_pool + *used;
  *used += len;
  return out;
}

/**
 * @brief Account for or copy a byte extent into the text region.
 * @param[in,out] pool Destination arena or counting state.
 * @param[in] src Accessible source bytes; NULL is permitted only for len zero.
 * @param[in] len Exact byte count, independent of embedded NULs.
 * @return 1 on success, 0 when reservation fails.
 * @note Counting skips source reads; materialization uses a bounded copy loop.
 */
static inline int jsonbourne_pool_copy(struct jsonbourne_pool *pool, const char *src, size_t len) {
  char *dst = (char *)jsonbourne_pool_take(pool, len, 1);
  size_t idx;
  if (pool->code)
    return 0;
  if (dst)
    for (idx = 0; idx < len; ++idx)
      dst[idx] = src[idx];
  return 1;
}

/**
 * @brief Find bytes that need no string escaping or Unicode decoding.
 * @param[in] src Accessible input extent; no padding or alignment is assumed.
 * @param[in] len Available bytes.
 * @param[in] quote Current string delimiter, which terminates the fast path.
 * @return Prefix length containing printable ASCII other than quote/backslash.
 * @details Vector loads require a complete 16-byte block. The scalar tail keeps
 * page-boundary inputs safe; high-bit bytes return to strict UTF-8 validation.
 */
static inline size_t jsonbourne_ascii_span(const unsigned char *src, size_t len,
                                           unsigned char quote) {
  size_t pos = 0;
#if defined(JSONBOURNE_HAS_SSE2)
  const __m128i quotes = _mm_set1_epi8((char)quote);
  const __m128i slashes = _mm_set1_epi8('\\');
  const __m128i controls = _mm_set1_epi8(0x20);
  while (len - pos >= 16) {
    const __m128i block = _mm_loadu_si128((const __m128i *)(const void *)(src + pos));
    /* Signed lanes also classify 0x80..0xff as below 0x20, stopping before UTF-8. */
    const __m128i special =
        _mm_or_si128(_mm_or_si128(_mm_cmpeq_epi8(block, quotes), _mm_cmpeq_epi8(block, slashes)),
                     _mm_cmplt_epi8(block, controls));
    const unsigned mask = (unsigned)_mm_movemask_epi8(special);
    if (mask)
      return pos + (size_t)__builtin_ctz(mask);
    pos += 16;
  }
#elif defined(JSONBOURNE_HAS_NEON)
  while (len - pos >= 16) {
    const uint8x16_t block = vld1q_u8(src + pos);
    const uint8x16_t special =
        vorrq_u8(vorrq_u8(vceqq_u8(block, vdupq_n_u8(quote)), vceqq_u8(block, vdupq_n_u8('\\'))),
                 vorrq_u8(vcltq_u8(block, vdupq_n_u8(0x20)), vcgeq_u8(block, vdupq_n_u8(0x80))));
    if (vmaxvq_u8(special))
      break;
    pos += 16;
  }
#endif
  while (pos < len && src[pos] >= 0x20 && src[pos] < 0x80 && src[pos] != quote && src[pos] != '\\')
    ++pos;
  return pos;
}

/**
 * @brief Recognize only the four JSON whitespace bytes, independently of locale.
 * @param[in] chr Byte to classify.
 * @return Nonzero for space, tab, CR or LF; zero otherwise.
 */
static inline int jsonbourne_char_is_space(unsigned char chr) {
  return chr == ' ' || chr == '\t' || chr == '\r' || chr == '\n';
}

/**
 * @brief Skip a bounded run of JSON whitespace with the available SIMD baseline.
 * @param[in] src Accessible input extent without padding requirements.
 * @param[in] len Bytes available for inspection.
 * @return Whitespace prefix length, never greater than len.
 * @note The first non-whitespace byte is left for the grammar, including UTF-8.
 */
static inline size_t jsonbourne_space_span(const unsigned char *src, size_t len) {
  size_t pos = 0;
#if defined(JSONBOURNE_HAS_SSE2)
  while (len - pos >= 16) {
    const __m128i block = _mm_loadu_si128((const __m128i *)(const void *)(src + pos));
    const __m128i spaces = _mm_or_si128(_mm_or_si128(_mm_cmpeq_epi8(block, _mm_set1_epi8(' ')),
                                                     _mm_cmpeq_epi8(block, _mm_set1_epi8('\t'))),
                                        _mm_or_si128(_mm_cmpeq_epi8(block, _mm_set1_epi8('\r')),
                                                     _mm_cmpeq_epi8(block, _mm_set1_epi8('\n'))));
    const unsigned mask = (unsigned)_mm_movemask_epi8(spaces);
    if (mask != 0xffffU)
      return pos + (size_t)__builtin_ctz((~mask) & 0xffffU);
    pos += 16;
  }
#elif defined(JSONBOURNE_HAS_NEON)
  while (len - pos >= 16) {
    const uint8x16_t block = vld1q_u8(src + pos);
    const uint8x16_t spaces =
        vorrq_u8(vorrq_u8(vceqq_u8(block, vdupq_n_u8(' ')), vceqq_u8(block, vdupq_n_u8('\t'))),
                 vorrq_u8(vceqq_u8(block, vdupq_n_u8('\r')), vceqq_u8(block, vdupq_n_u8('\n'))));
    if (vminvq_u8(spaces) != 255)
      break;
    pos += 16;
  }
#endif
  while (pos < len && jsonbourne_char_is_space(src[pos]))
    ++pos;
  return pos;
}

/**
 * @brief Decode an ASCII hexadecimal digit without locale-sensitive conversion.
 * @param[in] chr Byte to inspect.
 * @return Integer 0 through 15, or -1 if the byte is not a hex digit.
 */
static inline int jsonbourne_hex_to_digit(unsigned char chr) {
  if (chr >= '0' && chr <= '9')
    return chr - '0';
  if (chr >= 'a' && chr <= 'f')
    return chr - 'a' + 10;
  if (chr >= 'A' && chr <= 'F')
    return chr - 'A' + 10;
  return -1;
}

/**
 * @brief Recognize the decimal digits permitted by JSON number grammar.
 * @param[in] chr Byte to inspect.
 * @return Nonzero for ASCII 0 through 9, zero otherwise.
 */
static inline int jsonbourne_char_is_digit(unsigned char chr) { return chr >= '0' && chr <= '9'; }

/**
 * @brief Classify this reader's relaxed, ASCII-only unquoted-key alphabet.
 * @param[in] chr Byte to inspect.
 * @return Nonzero for letters, digits, underscore or dollar sign.
 * @note This convenience alphabet is not a complete ECMAScript IdentifierName
 * implementation; it deliberately does not imply full JSON5 conformance.
 */
static inline int jsonbourne_char_is_id(unsigned char chr) {
  return (chr >= 'a' && chr <= 'z') || (chr >= 'A' && chr <= 'Z') ||
         jsonbourne_char_is_digit(chr) || chr == '_' || chr == '$';
}

/**
 * @brief Validate exactly one UTF-8 scalar without reading beyond the extent.
 * @param[in] src First byte of a possible UTF-8 sequence.
 * @param[in] len Available bytes, possibly fewer than a complete sequence.
 * @return Sequence length 1 through 4, or zero for an invalid/truncated sequence.
 * @details Lead-specific second-byte bounds reject overlong encodings, UTF-16
 * surrogate code points and values above U+10FFFF, not just bad continuation bits.
 */
static inline size_t jsonbourne_utf8_len(const unsigned char *src, size_t len) {
  unsigned char first;
  size_t count;
  size_t idx;
  if (!len)
    return 0;
  first = src[0];
  if (first < 0x80)
    return 1;
  if (first >= 0xc2 && first <= 0xdf)
    count = 2;
  else if (first >= 0xe0 && first <= 0xef)
    count = 3;
  else if (first >= 0xf0 && first <= 0xf4)
    count = 4;
  else
    return 0;
  if (len < count)
    return 0;
  for (idx = 1; idx < count; ++idx)
    if ((src[idx] & 0xc0) != 0x80)
      return 0;
  if ((first == 0xe0 && src[1] < 0xa0) || (first == 0xed && src[1] >= 0xa0) ||
      (first == 0xf0 && src[1] < 0x90) || (first == 0xf4 && src[1] >= 0x90))
    return 0;
  return count;
}

/**
 * @brief Recognize a numeric prefix without converting or rounding its value.
 * @param[in] src Accessible token candidate.
 * @param[in] len Candidate extent, which may also include following input.
 * @param[in] flg Enabled numeric spelling extensions.
 * @param[out] read_len Consumed token bytes on success; unchanged on failure.
 * @return 1 for a valid numeric prefix, 0 for invalid/incomplete grammar.
 * @note The caller must check the following delimiter or require read_len == len;
 * otherwise a valid prefix could conceal an invalid suffix.
 */
static inline int jsonbourne_read_num_token(const unsigned char *src, size_t len,
                                            jsonbourne_read_flag flg, size_t *read_len) {
  size_t pos = 0;
  size_t int_pos;
  size_t int_len;
  size_t frac_len = 0;
  int decimal = 0;
  if (!len)
    return 0;
  if (src[pos] == '-' || src[pos] == '+') {
    if (src[pos] == '+' && !(flg & JSONBOURNE_READ_ALLOW_PLUS_SIGN))
      return 0;
    if (++pos == len)
      return 0;
  }
  if (flg & JSONBOURNE_READ_ALLOW_INF_AND_NAN) {
    if (len - pos >= 8 && memcmp(src + pos, "Infinity", 8) == 0) {
      *read_len = pos + 8;
      return 1;
    }
    if (len - pos >= 3 && memcmp(src + pos, "NaN", 3) == 0) {
      *read_len = pos + 3;
      return 1;
    }
  }
  if ((flg & JSONBOURNE_READ_ALLOW_HEX_NUM) && len - pos >= 2 && src[pos] == '0' &&
      (src[pos + 1] == 'x' || src[pos + 1] == 'X')) {
    pos += 2;
    int_pos = pos;
    while (pos < len && jsonbourne_hex_to_digit(src[pos]) >= 0)
      ++pos;
    if (pos == int_pos)
      return 0;
    *read_len = pos;
    return 1;
  }
  int_pos = pos;
  while (pos < len && jsonbourne_char_is_digit(src[pos]))
    ++pos;
  int_len = pos - int_pos;
  if (int_len > 1 && src[int_pos] == '0')
    return 0;
  if (pos < len && src[pos] == '.') {
    size_t frac_pos = ++pos;
    decimal = 1;
    while (pos < len && jsonbourne_char_is_digit(src[pos]))
      ++pos;
    frac_len = pos - frac_pos;
  }
  if (!int_len && !frac_len)
    return 0;
  if ((!int_len || (decimal && !frac_len)) && !(flg & JSONBOURNE_READ_ALLOW_DECIMAL_POINT))
    return 0;
  if (pos < len && (src[pos] == 'e' || src[pos] == 'E')) {
    size_t exp_pos;
    ++pos;
    if (pos < len && (src[pos] == '+' || src[pos] == '-'))
      ++pos;
    exp_pos = pos;
    while (pos < len && jsonbourne_char_is_digit(src[pos]))
      ++pos;
    if (pos == exp_pos)
      return 0;
  }
  *read_len = pos;
  return 1;
}

/** @brief Per-call parser state; no mutable global state or input modification is needed. */
struct jsonbourne_read_ctx {
  const unsigned char *src;            /**< Immutable input extent. */
  size_t len;                          /**< Total supplied bytes. */
  size_t pos;                          /**< Next grammar byte, always at most len. */
  jsonbourne_read_flag flg;             /**< Validated named-option mask. */
  size_t val_count;                    /**< Root and nested values consumed so far. */
  jsonbourne_read_code code;           /**< First grammar/argument failure. */
  size_t loc_pos;                      /**< Input offset already scanned for locations. */
  size_t line;                         /**< Current one-based LF-delimited line. */
  size_t line_pos;                     /**< Offset immediately after the last scanned LF. */
  struct jsonbourne_read_limits limits; /**< Effective per-call budgets. */
  struct jsonbourne_pool pool;         /**< Counting or materializing arena. */
};

/**
 * @brief One open container, replacing an equivalent recursive parser call.
 * @details state 0 permits an initial member or empty closure; state 1 expects
 * an item after a separator; state 2 follows an item. Keeping 1 distinct from 0
 * is what lets closure distinguish a trailing comma from an empty container.
 */
struct jsonbourne_read_frame {
  void *ctn;                /**< Object/array payload, NULL during measurement. */
  void *tail;               /**< Last entry for constant-time ordered appends. */
  size_t len;               /**< Completed/reserved member or element count. */
  unsigned char type;       /**< Object or array tag. */
  unsigned char state;      /**< Initial, after-separator or after-item state. */
  unsigned char is_global;  /**< Brace-less object closes at end of input. */
};

/**
 * @brief Preserve the first grammar failure while supporting short-circuit returns.
 * @param[in,out] ctx Read context whose code may still indicate success.
 * @param[in] code Failure to record if no earlier grammar error exists.
 * @return Always zero, allowing direct use in a failing parser expression.
 */
static inline int jsonbourne_set_read_error(struct jsonbourne_read_ctx *ctx,
                                            jsonbourne_read_code code) {
  if (!ctx->code)
    ctx->code = code;
  return 0;
}

/**
 * @brief Advance a lazy location cursor instead of rescanning each node's prefix.
 * @param[in,out] ctx Materializing read state with cached line information.
 * @param[in] pos Requested input byte offset; must not precede ctx->loc_pos.
 * @param[out] line One-based line number at pos.
 * @param[out] row Zero-based byte column at pos.
 * @note Monotonic requests keep total location bookkeeping linear in input size.
 */
static inline void jsonbourne_read_locate(struct jsonbourne_read_ctx *ctx, size_t pos, size_t *line,
                                          size_t *row) {
  while (ctx->loc_pos < pos) {
    if (ctx->src[ctx->loc_pos++] == '\n') {
      ++ctx->line;
      ctx->line_pos = ctx->loc_pos;
    }
  }
  *line = ctx->line;
  *row = pos - ctx->line_pos;
}

/**
 * @brief Consume whitespace and enabled comments without swallowing grammar bytes.
 * @param[in,out] ctx Input cursor and enabled syntax options.
 * @return 1 at the next significant byte/end, 0 for an unterminated block comment.
 * @note A failing block comment restores its opening offset so diagnostics point
 * at the construct that needs repair rather than merely at end of input.
 */
static inline int jsonbourne_skip_trivia(struct jsonbourne_read_ctx *ctx) {
  for (;;) {
    ctx->pos += jsonbourne_space_span(ctx->src + ctx->pos, ctx->len - ctx->pos);
    if (!(ctx->flg & JSONBOURNE_READ_ALLOW_COMMENTS) || ctx->len - ctx->pos < 2 ||
        ctx->src[ctx->pos] != '/')
      return 1;
    if (ctx->src[ctx->pos + 1] == '/') {
      ctx->pos += 2;
      while (ctx->pos < ctx->len && ctx->src[ctx->pos] != '\n' && ctx->src[ctx->pos] != '\r')
        ++ctx->pos;
    } else if (ctx->src[ctx->pos + 1] == '*') {
      const size_t start = ctx->pos;
      ctx->pos += 2;
      while (ctx->len - ctx->pos >= 2 &&
             !(ctx->src[ctx->pos] == '*' && ctx->src[ctx->pos + 1] == '/'))
        ++ctx->pos;
      if (ctx->len - ctx->pos < 2) {
        ctx->pos = start;
        return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_COMMENT);
      }
      ctx->pos += 2;
    } else
      return 1;
  }
}

/**
 * @brief Enforce a decoded-string budget before reserving any more text bytes.
 * @param[in,out] ctx Parser and arena state.
 * @param[in] start Text counter at the beginning of the current decoded string.
 * @param[in] bytes Decoded source bytes to append when materializing.
 * @param[in] count Number of bytes, excluding any final terminator.
 * @return 1 if appended/counted, 0 for a string or arena limit failure.
 * @note The budget follows decoded size, so escapes cannot bypass it.
 */
static inline int jsonbourne_read_str_bytes(struct jsonbourne_read_ctx *ctx, size_t start,
                                            const char *bytes, size_t count) {
  if (ctx->pool.str_len - start > ctx->limits.max_str_len ||
      count > ctx->limits.max_str_len - (ctx->pool.str_len - start))
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_LIMIT);
  return jsonbourne_pool_copy(&ctx->pool, bytes, count);
}

/**
 * @brief Consume the four hexadecimal digits following a Unicode escape marker.
 * @param[in,out] ctx Cursor positioned at the first hex digit.
 * @param[out] unit Decoded UTF-16 code unit; undefined as a result on failure.
 * @return 1 for four valid digits, 0 with INVALID_ESCAPE otherwise.
 * @note A code unit is not necessarily a scalar; the string reader pairs surrogates.
 */
static inline int jsonbourne_read_hex4(struct jsonbourne_read_ctx *ctx, uint32_t *unit) {
  size_t idx;
  *unit = 0;
  if (ctx->len - ctx->pos < 4)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
  for (idx = 0; idx < 4; ++idx) {
    const int digit = jsonbourne_hex_to_digit(ctx->src[ctx->pos]);
    if (digit < 0)
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
    *unit = (*unit << 4) | (unsigned)digit;
    ++ctx->pos;
  }
  return 1;
}

/**
 * @brief Encode a validated Unicode scalar into the decoded text arena.
 * @param[in,out] ctx Parser/arena and decoded-string budget.
 * @param[in] start Current string's initial text counter.
 * @param[in] codepoint Scalar already checked for surrogate/range validity.
 * @return 1 if its UTF-8 bytes fit, 0 on an arena/string limit.
 * @pre codepoint is at most U+10FFFF and is not a surrogate code point.
 */
static inline int jsonbourne_read_utf8_encode(struct jsonbourne_read_ctx *ctx, size_t start,
                                              uint32_t codepoint) {
  char bytes[4];
  size_t len;
  if (codepoint < 0x80) {
    bytes[0] = (char)codepoint;
    len = 1;
  } else if (codepoint < 0x800) {
    bytes[0] = (char)(0xc0 | (codepoint >> 6));
    bytes[1] = (char)(0x80 | (codepoint & 0x3f));
    len = 2;
  } else if (codepoint < 0x10000) {
    bytes[0] = (char)(0xe0 | (codepoint >> 12));
    bytes[1] = (char)(0x80 | ((codepoint >> 6) & 0x3f));
    bytes[2] = (char)(0x80 | (codepoint & 0x3f));
    len = 3;
  } else {
    bytes[0] = (char)(0xf0 | (codepoint >> 18));
    bytes[1] = (char)(0x80 | ((codepoint >> 12) & 0x3f));
    bytes[2] = (char)(0x80 | ((codepoint >> 6) & 0x3f));
    bytes[3] = (char)(0x80 | (codepoint & 0x3f));
    len = 4;
  }
  return jsonbourne_read_str_bytes(ctx, start, bytes, len);
}

/**
 * @brief Decode one quoted string or enabled unquoted key through the same two passes.
 * @param[in,out] ctx Cursor at the string/key and arena state.
 * @param[in] key Nonzero permits the unquoted-key path when its option is enabled.
 * @param[out] out String record, or NULL on a successful counting pass.
 * @return 1 for a complete decoded string, 0 with parser/arena failure recorded.
 * @details Escape decoding and raw UTF-8 validation converge on the same decoded
 * byte budget. A separate trailing NUL is stored without counting it in len.
 */
static inline int jsonbourne_read_str(struct jsonbourne_read_ctx *ctx, int key,
                                      jsonbourne_str **out) {
  const size_t pos = ctx->pos;
  const size_t start = ctx->pool.str_len;
  const int has_loc = (ctx->flg & JSONBOURNE_READ_TRACK_LOCATION) != 0;
  jsonbourne_str *str = (jsonbourne_str *)jsonbourne_pool_take(
      &ctx->pool, has_loc ? sizeof(struct jsonbourne_str_loc) : sizeof(struct jsonbourne_str), 0);
  unsigned char quote;
  int complete = 0;
  if (ctx->pool.code)
    return 0;
  if (ctx->pos >= ctx->len)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
  quote = ctx->src[ctx->pos];
  if (quote != '"' && !(quote == '\'' && (ctx->flg & JSONBOURNE_READ_ALLOW_SINGLE_QUOTED_STR))) {
    size_t len;
    if (!key || !(ctx->flg & JSONBOURNE_READ_ALLOW_UNQUOTED_KEY))
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_EXPECTED_KEY);
    while (ctx->pos < ctx->len && jsonbourne_char_is_id(ctx->src[ctx->pos]))
      ++ctx->pos;
    len = ctx->pos - pos;
    if (!len)
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_EXPECTED_KEY);
    if (!jsonbourne_read_str_bytes(ctx, start, (const char *)ctx->src + pos, len))
      return 0;
    complete = 1;
  } else {
    ++ctx->pos;
    while (ctx->pos < ctx->len) {
      const size_t run = jsonbourne_ascii_span(ctx->src + ctx->pos, ctx->len - ctx->pos, quote);
      unsigned char chr;
      if (!jsonbourne_read_str_bytes(ctx, start, (const char *)ctx->src + ctx->pos, run))
        return 0;
      ctx->pos += run;
      if (ctx->pos == ctx->len)
        break;
      chr = ctx->src[ctx->pos++];
      if (chr == quote) {
        complete = 1;
        break;
      }
      if (chr == '\\') {
        char decoded;
        if (ctx->pos == ctx->len)
          return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
        chr = ctx->src[ctx->pos++];
        switch (chr) {
        case '"':
        case '\\':
        case '/':
          decoded = (char)chr;
          break;
        case 'b':
          decoded = '\b';
          break;
        case 'f':
          decoded = '\f';
          break;
        case 'n':
          decoded = '\n';
          break;
        case 'r':
          decoded = '\r';
          break;
        case 't':
          decoded = '\t';
          break;
        case '\'':
          if (!(ctx->flg & JSONBOURNE_READ_ALLOW_SINGLE_QUOTED_STR))
            return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
          decoded = '\'';
          break;
        case 'u': {
          uint32_t codepoint;
          if (!jsonbourne_read_hex4(ctx, &codepoint))
            return 0;
          /* UTF-16 halves are valid only as a pair; neither half may leak into UTF-8. */
          if (codepoint >= 0xd800 && codepoint <= 0xdbff) {
            uint32_t low;
            if (ctx->len - ctx->pos < 2 || ctx->src[ctx->pos] != '\\' ||
                ctx->src[ctx->pos + 1] != 'u')
              return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
            ctx->pos += 2;
            if (!jsonbourne_read_hex4(ctx, &low))
              return 0;
            if (low < 0xdc00 || low > 0xdfff)
              return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
            codepoint = 0x10000 + ((codepoint - 0xd800) << 10) + low - 0xdc00;
          } else if (codepoint >= 0xdc00 && codepoint <= 0xdfff)
            return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
          if (!jsonbourne_read_utf8_encode(ctx, start, codepoint))
            return 0;
          continue;
        }
        case '\n':
        case '\r':
          if (!(ctx->flg & JSONBOURNE_READ_ALLOW_MULTILINE_STR))
            return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
          if (chr == '\r' && ctx->pos < ctx->len && ctx->src[ctx->pos] == '\n')
            ++ctx->pos;
          continue;
        default:
          return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_ESCAPE);
        }
        if (!jsonbourne_read_str_bytes(ctx, start, &decoded, 1))
          return 0;
      } else if (chr >= 0x80) {
        const size_t position = ctx->pos - 1;
        const size_t len = jsonbourne_utf8_len(ctx->src + position, ctx->len - position);
        if (!len) {
          ctx->pos = position;
          return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_UTF8);
        }
        if (!jsonbourne_read_str_bytes(ctx, start, (const char *)ctx->src + position, len))
          return 0;
        ctx->pos = position + len;
      } else {
        const char decoded = (char)chr;
        if ((chr != '\n' && chr != '\r') || !(ctx->flg & JSONBOURNE_READ_ALLOW_MULTILINE_STR))
          return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_STRING);
        if (!jsonbourne_read_str_bytes(ctx, start, &decoded, 1))
          return 0;
      }
    }
  }
  if (!complete)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
  if (!jsonbourne_pool_copy(&ctx->pool, "", 1))
    return 0;
  if (str) {
    str->str = ctx->pool.str_pool + start;
    str->len = ctx->pool.str_len - start - 1;
    if (has_loc) {
      jsonbourne_str_loc *extended = (jsonbourne_str_loc *)str;
      extended->pos = pos;
      jsonbourne_read_locate(ctx, pos, &extended->line, &extended->col);
    }
  }
  *out = str;
  return 1;
}

/**
 * @brief Verify the token boundary separately from the token's own spelling.
 * @param[in] ctx Input extent and comment policy.
 * @param[in] pos Candidate end offset, no greater than ctx->len.
 * @return Nonzero at input end or a permissible token boundary.
 * @details Prevents a valid prefix such as true in truex, or 1 in 1abc, from
 * being admitted as a complete scalar value.
 */
static inline int jsonbourne_is_delimiter(const struct jsonbourne_read_ctx *ctx, size_t pos) {
  unsigned char chr;
  if (pos == ctx->len)
    return 1;
  chr = ctx->src[pos];
  return jsonbourne_char_is_space(chr) || chr == ',' || chr == ']' || chr == '}' ||
         (chr == '/' && (ctx->flg & JSONBOURNE_READ_ALLOW_COMMENTS));
}

/**
 * @brief Consume a scalar or push a new container without a recursive call.
 * @param[in,out] ctx Read cursor, budgets and arena.
 * @param[out] out Result record, or NULL while counting.
 * @param[in,out] stack Fixed storage for open containers.
 * @param[in,out] depth Number of active frames; checked before every push.
 * @param[in] is_global Nonzero requests a brace-less object at the current cursor.
 * @return 1 for a scalar/opened container, 0 for grammar or resource failure.
 * @note Containers are completed by jsonbourne_read_document(), allowing nesting
 * to consume bounded frame storage rather than the C call stack.
 */
static inline int jsonbourne_read_val(struct jsonbourne_read_ctx *ctx, jsonbourne_val **out,
                                      struct jsonbourne_read_frame *stack, size_t *depth,
                                      int is_global) {
  const int has_loc = (ctx->flg & JSONBOURNE_READ_TRACK_LOCATION) != 0;
  jsonbourne_val *val;
  unsigned char chr;
  if (ctx->val_count == ctx->limits.max_vals)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_LIMIT);
  ++ctx->val_count;
  val = (jsonbourne_val *)jsonbourne_pool_take(
      &ctx->pool, has_loc ? sizeof(struct jsonbourne_val_loc) : sizeof(struct jsonbourne_val), 0);
  if (ctx->pool.code)
    return 0;
  if (val) {
    val->ptr = NULL;
    if (has_loc) {
      jsonbourne_val_loc *extended = (jsonbourne_val_loc *)val;
      extended->pos = ctx->pos;
      jsonbourne_read_locate(ctx, ctx->pos, &extended->line, &extended->col);
    }
  }
  *out = val;
  if (ctx->pos == ctx->len && !is_global)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
  chr = ctx->pos < ctx->len ? ctx->src[ctx->pos] : 0;
  if (is_global || chr == '{' || chr == '[') {
    struct jsonbourne_read_frame *frame;
    const int obj = is_global || chr == '{';
    void *ctn;
    if (*depth == ctx->limits.max_depth || *depth == JSONBOURNE_READER_DEPTH_LIMIT)
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_DEPTH);
    ctn = jsonbourne_pool_take(
        &ctx->pool, obj ? sizeof(struct jsonbourne_obj) : sizeof(struct jsonbourne_arr), 0);
    if (ctx->pool.code)
      return 0;
    if (val) {
      val->type = obj ? JSONBOURNE_TYPE_OBJ : JSONBOURNE_TYPE_ARR;
      val->ptr = ctn;
    }
    frame = &stack[(*depth)++];
    frame->ctn = ctn;
    frame->tail = NULL;
    frame->len = 0;
    frame->type = (unsigned char)(obj ? JSONBOURNE_TYPE_OBJ : JSONBOURNE_TYPE_ARR);
    frame->state = 0;
    frame->is_global = (unsigned char)is_global;
    if (ctn) {
      if (obj)
        ((jsonbourne_obj *)ctn)->first = NULL;
      else
        ((jsonbourne_arr *)ctn)->first = NULL;
    }
    if (!is_global)
      ++ctx->pos;
    return 1;
  }
  if (chr == '"' || (chr == '\'' && (ctx->flg & JSONBOURNE_READ_ALLOW_SINGLE_QUOTED_STR))) {
    jsonbourne_str *str;
    if (!jsonbourne_read_str(ctx, 0, &str))
      return 0;
    if (val) {
      val->type = JSONBOURNE_TYPE_STR;
      val->ptr = str;
    }
    return 1;
  }
  if (chr == 't' || chr == 'f' || chr == 'n') {
    const char *literal = chr == 't' ? "true" : chr == 'f' ? "false" : "null";
    const size_t len = chr == 'f' ? 5 : 4;
    if (ctx->len - ctx->pos < len || memcmp(ctx->src + ctx->pos, literal, len) != 0 ||
        !jsonbourne_is_delimiter(ctx, ctx->pos + len))
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_LITERAL);
    ctx->pos += len;
    if (val)
      val->type = chr == 't'   ? JSONBOURNE_TYPE_TRUE
                  : chr == 'f' ? JSONBOURNE_TYPE_FALSE
                               : JSONBOURNE_TYPE_NULL;
    return 1;
  }
  {
    size_t len = 0;
    const size_t start = ctx->pool.str_len;
    jsonbourne_num *num;
    if (!jsonbourne_read_num_token(ctx->src + ctx->pos, ctx->len - ctx->pos, ctx->flg, &len) ||
        !jsonbourne_is_delimiter(ctx, ctx->pos + len))
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_NUMBER);
    num = (jsonbourne_num *)jsonbourne_pool_take(&ctx->pool, sizeof(struct jsonbourne_num), 0);
    if (ctx->pool.code ||
        !jsonbourne_pool_copy(&ctx->pool, (const char *)ctx->src + ctx->pos, len) ||
        !jsonbourne_pool_copy(&ctx->pool, "", 1))
      return 0;
    ctx->pos += len;
    if (val) {
      val->type = JSONBOURNE_TYPE_NUM;
      val->ptr = num;
      num->str = ctx->pool.str_pool + start;
      num->len = len;
    }
    return 1;
  }
}

/**
 * @brief Drive one complete JSON document using explicit container frames.
 * @param[in,out] ctx Initialized counting or materializing read context.
 * @param[out] root Parsed root, or NULL while counting.
 * @return 1 only when the root and all trailing trivia consume the full input.
 * @details The frame state distinguishes empty closure, post-separator closure
 * and a completed item. Sharing this state machine across both passes prevents
 * the allocator's size prediction from using a different grammar.
 */
static inline int jsonbourne_read_document(struct jsonbourne_read_ctx *ctx, jsonbourne_val **root) {
  struct jsonbourne_read_frame stack[JSONBOURNE_READER_DEPTH_LIMIT];
  size_t depth = 0;
  int is_global;
  if (!jsonbourne_skip_trivia(ctx))
    return 0;
  is_global = (ctx->flg & JSONBOURNE_READ_ALLOW_GLOBAL_OBJECT) &&
              (ctx->pos == ctx->len || ctx->src[ctx->pos] != '{');
  if (!jsonbourne_read_val(ctx, root, stack, &depth, is_global))
    return 0;
  while (depth) {
    struct jsonbourne_read_frame *frame = &stack[depth - 1];
    jsonbourne_val *child = NULL;
    jsonbourne_val **dst = &child;
    const unsigned char close = frame->type == JSONBOURNE_TYPE_OBJ ? '}' : ']';
    int closing;
    if (!jsonbourne_skip_trivia(ctx))
      return 0;
    closing = frame->is_global ? ctx->pos == ctx->len
                               : ctx->pos < ctx->len && ctx->src[ctx->pos] == close;
    if (frame->state == 2 && !closing) {
      if (ctx->pos == ctx->len)
        return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
      if (ctx->src[ctx->pos] == ',')
        ++ctx->pos;
      else if (!(ctx->flg & JSONBOURNE_READ_ALLOW_NO_COMMAS))
        return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_EXPECTED_SEPARATOR);
      frame->state = 1;
      if (!jsonbourne_skip_trivia(ctx))
        return 0;
      closing = frame->is_global ? ctx->pos == ctx->len
                                 : ctx->pos < ctx->len && ctx->src[ctx->pos] == close;
    }
    if (closing) {
      if (frame->state == 1 && !(ctx->flg & JSONBOURNE_READ_ALLOW_TRAILING_COMMAS))
        return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_EXPECTED_SEPARATOR);
      if (frame->ctn) {
        if (frame->type == JSONBOURNE_TYPE_OBJ)
          ((jsonbourne_obj *)frame->ctn)->len = frame->len;
        else
          ((jsonbourne_arr *)frame->ctn)->len = frame->len;
      }
      if (!frame->is_global)
        ++ctx->pos;
      --depth;
      continue;
    }
    if (ctx->pos == ctx->len)
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
    if (frame->len == SIZE_MAX)
      return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_LIMIT);
    if (frame->type == JSONBOURNE_TYPE_OBJ) {
      jsonbourne_obj_entry *elem = (jsonbourne_obj_entry *)jsonbourne_pool_take(
          &ctx->pool, sizeof(struct jsonbourne_obj_entry), 0);
      jsonbourne_str *name;
      if (ctx->pool.code || !jsonbourne_read_str(ctx, 1, &name) || !jsonbourne_skip_trivia(ctx))
        return 0;
      if (ctx->pos == ctx->len)
        return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_END);
      if (ctx->src[ctx->pos] != ':' &&
          !(ctx->src[ctx->pos] == '=' && (ctx->flg & JSONBOURNE_READ_ALLOW_EQUALS)))
        return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_EXPECTED_COLON);
      ++ctx->pos;
      if (elem) {
        elem->key = name;
        elem->val = NULL;
        elem->next = NULL;
        if (frame->tail)
          ((jsonbourne_obj_entry *)frame->tail)->next = elem;
        else
          ((jsonbourne_obj *)frame->ctn)->first = elem;
        dst = &elem->val;
      }
      frame->tail = elem;
    } else {
      jsonbourne_arr_entry *elem = (jsonbourne_arr_entry *)jsonbourne_pool_take(
          &ctx->pool, sizeof(struct jsonbourne_arr_entry), 0);
      if (ctx->pool.code)
        return 0;
      if (elem) {
        elem->val = NULL;
        elem->next = NULL;
        if (frame->tail)
          ((jsonbourne_arr_entry *)frame->tail)->next = elem;
        else
          ((jsonbourne_arr *)frame->ctn)->first = elem;
        dst = &elem->val;
      }
      frame->tail = elem;
    }
    ++frame->len;
    frame->state = 2;
    if (!jsonbourne_skip_trivia(ctx) || !jsonbourne_read_val(ctx, dst, stack, &depth, 0))
      return 0;
  }
  if (!jsonbourne_skip_trivia(ctx))
    return 0;
  if (ctx->pos != ctx->len)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_UNEXPECTED_CONTENT);
  return 1;
}

/**
 * @brief Establish cursor/limit invariants before any grammar byte is accessed.
 * @param[out] ctx Fresh context, also safe to report after initialization failure.
 * @param[in] src Input bytes; NULL is valid only with len zero.
 * @param[in] len Input extent in bytes.
 * @param[in] flg Requested named options, checked against accepted bits.
 * @param[in] limits Optional caller budgets; NULL selects compiled defaults.
 * @return 1 for valid initial state, 0 with an argument/resource error recorded.
 * @note PTRDIFF_MAX bounds keep later pointer arithmetic representable; an empty
 * sentinel replaces NULL input so even zero-byte parser paths have a valid base.
 */
static inline int jsonbourne_read_ctx_init(struct jsonbourne_read_ctx *ctx, const void *src,
                                           size_t len, jsonbourne_read_flag flg,
                                           const jsonbourne_read_limits *limits) {
  const struct jsonbourne_read_ctx empty = JSONBOURNE_ZERO;
  *ctx = empty;
  ctx->src = (const unsigned char *)src;
  ctx->len = len;
  ctx->flg = flg;
  ctx->line = 1;
  ctx->limits = limits ? *limits : jsonbourne_read_limits_default();
  ctx->pool.val_cap = SIZE_MAX;
  ctx->pool.str_cap = SIZE_MAX;
  ctx->pool.max_memory =
      ctx->limits.max_memory < (size_t)PTRDIFF_MAX ? ctx->limits.max_memory : (size_t)PTRDIFF_MAX;
  if ((!src && len) || (flg & ~(size_t)JSONBOURNE_READ_FLAG_MASK) ||
      ctx->limits.max_depth > JSONBOURNE_READER_DEPTH_LIMIT)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_PARAMETER);
  if (len > ctx->limits.max_input_len || len > (size_t)PTRDIFF_MAX)
    return jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_LIMIT);
  if (!src)
    ctx->src = (const unsigned char *)"";
  return 1;
}

/**
 * @brief Normalize parser/arena failure into an optional public diagnostic.
 * @param[in] ctx Context after initialization or parsing, successful or failed.
 * @param[out] err Destination, or NULL to skip diagnostic work.
 * @details Coordinates are scanned only when an error must be reported, so reads
 * without TRACK_LOCATION do not maintain line metadata on their successful path.
 */
static inline void jsonbourne_read_report(const struct jsonbourne_read_ctx *ctx,
                                          jsonbourne_read_err *err) {
  size_t idx;
  size_t line_start = 0;
  if (!err)
    return;
  err->code = ctx->code ? ctx->code : ctx->pool.code;
  err->pos = 0;
  err->line = 0;
  err->col = 0;
  if (!err->code)
    return;
  err->pos = ctx->pos <= ctx->len ? ctx->pos : ctx->len;
  err->line = 1;
  for (idx = 0; ctx->src && idx < err->pos; ++idx)
    if (ctx->src[idx] == '\n') {
      ++err->line;
      line_start = idx + 1;
    }
  err->col = err->pos - line_start;
}

/** Validate and return the exact single-allocation size without allocating. */
int jsonbourne_read_measure(const void *src, size_t len, jsonbourne_read_flag flg,
                            const jsonbourne_read_limits *limits, size_t *required,
                            jsonbourne_read_err *err) {
  struct jsonbourne_read_ctx ctx;
  jsonbourne_val *root = NULL;
  int success;
  if (required)
    *required = 0;
  /* NULL pool bases intentionally validate/count without constructing temporary nodes. */
  success = jsonbourne_read_ctx_init(&ctx, src, len, flg, limits) &&
            jsonbourne_read_document(&ctx, &root);
  jsonbourne_read_report(&ctx, err);
  if (success && required)
    *required = ctx.pool.val_len + ctx.pool.str_len;
  return success;
}

int jsonbourne_read_validate(const void *src, size_t len, jsonbourne_read_flag flg,
                             const jsonbourne_read_limits *limits, jsonbourne_read_err *err) {
  return jsonbourne_read_measure(src, len, flg, limits, NULL, err);
}

/**
 * @brief Compare numeric address distances instead of forming overflowing end pointers.
 * @param[in] first First extent's base address.
 * @param[in] first_size First extent in bytes.
 * @param[in] second Second extent's base address.
 * @param[in] second_size Second extent in bytes.
 * @return Nonzero when both nonempty extents overlap, zero otherwise.
 * @note Integer addresses also avoid relational comparison of unrelated C pointers.
 */
static inline int jsonbourne_overlap(const void *first, size_t first_size, const void *second,
                                     size_t second_size) {
  const uintptr_t first_addr = (uintptr_t)first;
  const uintptr_t second_addr = (uintptr_t)second;
  if (!first_size || !second_size)
    return 0;
  return first_addr <= second_addr ? second_addr - first_addr < first_size
                                   : first_addr - second_addr < second_size;
}

/**
 * @brief Replay a successful counting pass into precisely bounded record/text regions.
 * @param[in,out] ctx Successful measurement state; input/options remain unchanged.
 * @param[out] buf Accessible backing allocation, checked for alignment and overlap.
 * @param[in] val_len Measured record-region bytes.
 * @param[in] str_len Measured text-region bytes, including terminators.
 * @param[out] err Optional public read result.
 * @return Root in buf, or NULL on layout/parsing failure.
 * @details Resetting cursor, counters and location cache makes this a fresh grammar
 * pass; exact capacities prevent it from writing beyond the measured allocation.
 */
static inline jsonbourne_val *jsonbourne_read_materialize(struct jsonbourne_read_ctx *ctx,
                                                          void *buf, size_t val_len, size_t str_len,
                                                          jsonbourne_read_err *err) {
  jsonbourne_val *root = NULL;
  if ((uintptr_t)buf % JSONBOURNE_ALIGNOF(struct jsonbourne_val_loc) ||
      jsonbourne_overlap(ctx->src, ctx->len, buf, val_len + str_len)) {
    jsonbourne_set_read_error(ctx, JSONBOURNE_READ_ERROR_INVALID_PARAMETER);
    jsonbourne_read_report(ctx, err);
    return NULL;
  }
  ctx->pos = 0;
  ctx->val_count = 0;
  ctx->loc_pos = 0;
  ctx->line = 1;
  ctx->line_pos = 0;
  ctx->pool.val_pool = (unsigned char *)buf;
  ctx->pool.str_pool = (char *)buf + val_len;
  ctx->pool.val_len = 0;
  ctx->pool.str_len = 0;
  ctx->pool.val_cap = val_len;
  ctx->pool.str_cap = str_len;
  if (!jsonbourne_read_document(ctx, &root))
    root = NULL;
  jsonbourne_read_report(ctx, err);
  return root;
}

/** Parse into aligned caller storage. On insufficient capacity, required is set. */
jsonbourne_val *jsonbourne_read_buf(const void *src, size_t len, jsonbourne_read_flag flg,
                                    const jsonbourne_read_limits *limits, void *buf, size_t cap,
                                    size_t *required, jsonbourne_read_err *err) {
  struct jsonbourne_read_ctx ctx;
  jsonbourne_val *root = NULL;
  size_t total;
  if (required)
    *required = 0;
  if (!jsonbourne_read_ctx_init(&ctx, src, len, flg, limits) ||
      !jsonbourne_read_document(&ctx, &root)) {
    jsonbourne_read_report(&ctx, err);
    return NULL;
  }
  total = ctx.pool.val_len + ctx.pool.str_len;
  if (required)
    *required = total;
  if (!buf || cap < total) {
    jsonbourne_set_read_error(&ctx, JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION);
    jsonbourne_read_report(&ctx, err);
    return NULL;
  }
  return jsonbourne_read_materialize(&ctx, buf, ctx.pool.val_len, ctx.pool.str_len, err);
}

jsonbourne_val *jsonbourne_read_opts_with_limits(const void *src, size_t len,
                                                 jsonbourne_read_flag flg,
                                                 const jsonbourne_read_limits *limits,
                                                 void *(*alc_fn)(void *, size_t), void *alc_ctx,
                                                 jsonbourne_read_err *err) {
  struct jsonbourne_read_ctx ctx;
  jsonbourne_val *root = NULL;
  void *buf;
  if (!jsonbourne_read_ctx_init(&ctx, src, len, flg, limits) ||
      !jsonbourne_read_document(&ctx, &root)) {
    jsonbourne_read_report(&ctx, err);
    return NULL;
  }
  buf = alc_fn ? alc_fn(alc_ctx, ctx.pool.val_len + ctx.pool.str_len)
               : malloc(ctx.pool.val_len + ctx.pool.str_len);
  if (!buf) {
    jsonbourne_set_read_error(&ctx, JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION);
    jsonbourne_read_report(&ctx, err);
    return NULL;
  }
  root = jsonbourne_read_materialize(&ctx, buf, ctx.pool.val_len, ctx.pool.str_len, err);
  /* There is no custom release callback here; custom arenas keep their own lifetime policy. */
  if (!root && !alc_fn)
    free(buf);
  return root;
}

jsonbourne_val *jsonbourne_read_opts(const void *src, size_t len, jsonbourne_read_flag flg,
                                     void *(*alc_fn)(void *, size_t), void *alc_ctx,
                                     jsonbourne_read_err *err) {
  return jsonbourne_read_opts_with_limits(src, len, flg, NULL, alc_fn, alc_ctx, err);
}

jsonbourne_val *jsonbourne_read(const void *src, size_t len) {
  return jsonbourne_read_opts(src, len, JSONBOURNE_READ_NOFLAG, NULL, NULL, NULL);
}

/** @brief Shared traversal protocol so writing and cloning use identical structural checks. */
enum jsonbourne_event {
  JSONBOURNE_EVENT_VAL,  /**< Visit a scalar or open a container at the current depth. */
  JSONBOURNE_EVENT_KEY,  /**< Visit an object key before its child value. */
  JSONBOURNE_EVENT_ELEM, /**< Visit an array slot before its child value. */
  JSONBOURNE_EVENT_END   /**< Close a container after its declared entries are consumed. */
};

/** @brief Suspended container traversal with constant-space sibling-cycle detection. */
struct jsonbourne_walk_frame {
  const jsonbourne_val *val; /**< Container owning this frame. */
  const void *next;          /**< Next unvisited member/element. */
  const void *slow;          /**< One-step Floyd cycle cursor. */
  const void *fast;          /**< Two-step Floyd cycle cursor. */
  size_t remaining;         /**< Declared list entries not yet visited. */
  size_t idx;               /**< Zero-based next entry index, used for separators. */
};

/**
 * @brief Advance either entry type without relying on their layouts being interchangeable.
 * @param[in] elem Accessible entry, or NULL at list end.
 * @param[in] type Object tag selects object entries; array traversal uses array entries.
 * @return Next entry, or NULL.
 */
static inline const void *jsonbourne_next(const void *elem, size_t type) {
  if (!elem)
    return NULL;
  return type == JSONBOURNE_TYPE_OBJ ? (const void *)((const jsonbourne_obj_entry *)elem)->next
                                     : (const void *)((const jsonbourne_arr_entry *)elem)->next;
}

/**
 * @brief Advance Floyd's two cursors without allocating a visited-address set.
 * @param[in,out] slow Cursor advanced by one sibling link.
 * @param[in,out] fast Cursor advanced by two sibling links.
 * @param[in] type Entry layout: object or array.
 * @return 1 if no cycle is encountered on this step, 0 if non-NULL cursors meet.
 * @note All linked entries must still be accessible; this validates topology,
 * not the validity of arbitrary pointer addresses.
 */
static inline int jsonbourne_acyclic_step(const void **slow, const void **fast, size_t type) {
  *slow = jsonbourne_next(*slow, type);
  *fast = jsonbourne_next(jsonbourne_next(*fast, type), type);
  return !*slow || *slow != *fast;
}

/**
 * @brief Validate/traverse an accessible DOM with a fixed explicit stack.
 * @param[in] val Root value to inspect.
 * @param[in] visit Event sink receiving context, event, value, optional key,
 * depth and entry index; returning zero aborts traversal.
 * @param[in,out] ctx Opaque event-sink state.
 * @return 1 for a complete accepted traversal, 0 for malformed structure, nesting
 * exhaustion or a rejecting visitor.
 * @details Exact list lengths and Floyd cursors detect malformed sibling chains.
 * Cycles through value/container pointers cannot grow without bound because
 * every nested container also consumes one checked frame.
 */
static inline int jsonbourne_walk(const jsonbourne_val *val,
                                  int (*visit)(void *, enum jsonbourne_event,
                                               const jsonbourne_val *, const jsonbourne_str *,
                                               size_t, size_t),
                                  void *ctx) {
  struct jsonbourne_walk_frame stack[JSONBOURNE_WRITER_DEPTH_LIMIT];
  size_t depth = 0;
  if (!val)
    return 0;
  for (;;) {
    if (val) {
      if (val->type > JSONBOURNE_TYPE_NULL || (val->type <= JSONBOURNE_TYPE_ARR && !val->ptr))
        return 0;
      if (!visit(ctx, JSONBOURNE_EVENT_VAL, val, NULL, depth, 0))
        return 0;
      if (val->type == JSONBOURNE_TYPE_OBJ || val->type == JSONBOURNE_TYPE_ARR) {
        struct jsonbourne_walk_frame *frame;
        if (depth == JSONBOURNE_WRITER_DEPTH_LIMIT)
          return 0;
        frame = &stack[depth++];
        frame->val = val;
        if (val->type == JSONBOURNE_TYPE_OBJ) {
          const jsonbourne_obj *obj = (const jsonbourne_obj *)val->ptr;
          frame->remaining = obj->len;
          frame->next = obj->first;
        } else {
          const jsonbourne_arr *arr = (const jsonbourne_arr *)val->ptr;
          frame->remaining = arr->len;
          frame->next = arr->first;
        }
        frame->slow = frame->next;
        frame->fast = frame->next;
        frame->idx = 0;
      }
    }
    val = NULL;
    if (!depth)
      return 1;
    {
      struct jsonbourne_walk_frame *frame = &stack[depth - 1];
      if (!frame->remaining) {
        if (frame->next ||
            !visit(ctx, JSONBOURNE_EVENT_END, frame->val, NULL, depth - 1, frame->idx))
          return 0;
        --depth;
      } else {
        const jsonbourne_str *name = NULL;
        if (!frame->next)
          return 0;
        if (frame->val->type == JSONBOURNE_TYPE_OBJ) {
          const jsonbourne_obj_entry *elem = (const jsonbourne_obj_entry *)frame->next;
          name = elem->key;
          val = elem->val;
          if (!name)
            return 0;
        } else
          val = ((const jsonbourne_arr_entry *)frame->next)->val;
        if (!val)
          return 0;
        frame->next = jsonbourne_next(frame->next, frame->val->type);
        if (!jsonbourne_acyclic_step(&frame->slow, &frame->fast, frame->val->type))
          return 0;
        if (!visit(ctx, name ? JSONBOURNE_EVENT_KEY : JSONBOURNE_EVENT_ELEM, frame->val, name,
                   depth, frame->idx))
          return 0;
        --frame->remaining;
        ++frame->idx;
      }
    }
  }
}

/** @brief One output sink used unchanged for measuring and writing JSON text. */
struct jsonbourne_write_ctx {
  char *buf;           /**< NULL while measuring; destination otherwise. */
  size_t cap;          /**< Hard output capacity, including any terminator. */
  size_t len;          /**< Bytes counted/written so far. */
  const char *indent;  /**< Borrowed whitespace indentation string. */
  size_t indent_len;   /**< Cached indentation byte length. */
  const char *newline; /**< Borrowed whitespace line separator. */
  size_t newline_len;  /**< Cached line-separator byte length. */
  int pretty;          /**< Nonzero enables indentation/newlines. */
};

/**
 * @brief Reserve output bytes before optionally copying them.
 * @param[in,out] out Counting/writing sink with len at most cap.
 * @param[in] bytes Accessible source extent when writing.
 * @param[in] len Byte count, not inferred from string termination.
 * @return 1 if bytes fit, 0 without an out-of-bounds write otherwise.
 * @note Capacity subtraction is checked before addition, including size-only passes.
 */
static inline int jsonbourne_write_bytes(struct jsonbourne_write_ctx *out, const char *bytes,
                                         size_t len) {
  size_t idx;
  if (out->len > out->cap || len > out->cap - out->len)
    return 0;
  if (out->buf)
    for (idx = 0; idx < len; ++idx)
      out->buf[out->len + idx] = bytes[idx];
  out->len += len;
  return 1;
}

/**
 * @brief Emit a UTF-8 string with JSON quoting and exact control-byte escaping.
 * @param[in,out] out Measuring/writing output sink.
 * @param[in] str Accessible length-aware string view, possibly containing NULs.
 * @return 1 for valid text that fits, 0 for invalid UTF-8/arguments or capacity.
 * @details Printable ASCII takes the span fast path; all remaining bytes are
 * escaped or validated as complete scalars, never passed through blindly.
 */
static inline int jsonbourne_write_str(struct jsonbourne_write_ctx *out,
                                       const jsonbourne_str *str) {
  static const char digits[] = "0123456789abcdef";
  size_t pos = 0;
  if (!str || (!str->str && str->len) || str->len > (size_t)PTRDIFF_MAX ||
      !jsonbourne_write_bytes(out, "\"", 1))
    return 0;
  while (pos < str->len) {
    const unsigned char *src = (const unsigned char *)str->str;
    const size_t run = jsonbourne_ascii_span(src + pos, str->len - pos, '"');
    unsigned char chr;
    if (!jsonbourne_write_bytes(out, str->str + pos, run))
      return 0;
    pos += run;
    if (pos == str->len)
      break;
    chr = src[pos++];
    if (chr == '"' || chr == '\\') {
      const char escaped[2] = {'\\', (char)chr};
      if (!jsonbourne_write_bytes(out, escaped, 2))
        return 0;
    } else if (chr < 0x20) {
      const char *escape = chr == '\b'   ? "\\b"
                           : chr == '\f' ? "\\f"
                           : chr == '\n' ? "\\n"
                           : chr == '\r' ? "\\r"
                           : chr == '\t' ? "\\t"
                                         : NULL;
      if (escape) {
        if (!jsonbourne_write_bytes(out, escape, 2))
          return 0;
      } else {
        const char escaped[6] = {'\\', 'u', '0', '0', digits[chr >> 4], digits[chr & 15]};
        if (!jsonbourne_write_bytes(out, escaped, 6))
          return 0;
      }
    } else {
      const size_t len = jsonbourne_utf8_len(src + pos - 1, str->len - pos + 1);
      if (!len || !jsonbourne_write_bytes(out, str->str + pos - 1, len))
        return 0;
      pos += len - 1;
    }
  }
  return jsonbourne_write_bytes(out, "\"", 1);
}

/**
 * @brief Convert hexadecimal magnitude text exactly using bounded decimal digits.
 * @param[in,out] out Measuring/writing output sink.
 * @param[in] src Validated hex digits without sign or 0x prefix.
 * @param[in] len Hex digit count, including any leading zeroes.
 * @return 1 for emitted decimal text, 0 on significant-digit/capacity limits.
 * @details Each input digit updates a little-endian decimal accumulator by
 * multiply-by-16 plus carry. This avoids floating-point loss and an unbounded
 * big-integer dependency; two decimal slots per hex digit are a conservative bound.
 */
static inline int jsonbourne_write_hex_num(struct jsonbourne_write_ctx *out, const char *src,
                                           size_t len) {
  unsigned char decimal[JSONBOURNE_MAX_HEX_DIGITS * 2];
  size_t used = 1;
  size_t pos;
  while (len && *src == '0') {
    ++src;
    --len;
  }
  if (len > JSONBOURNE_MAX_HEX_DIGITS)
    return 0;
  decimal[0] = 0;
  for (pos = 0; pos < len; ++pos) {
    unsigned carry = (unsigned)jsonbourne_hex_to_digit((unsigned char)src[pos]);
    size_t idx;
    for (idx = 0; idx < used; ++idx) {
      carry += 16U * decimal[idx];
      decimal[idx] = (unsigned char)(carry % 10U);
      carry /= 10U;
    }
    while (carry) {
      if (used == sizeof(decimal))
        return 0;
      decimal[used++] = (unsigned char)(carry % 10U);
      carry /= 10U;
    }
  }
  while (used) {
    const char digit = (char)('0' + decimal[--used]);
    if (!jsonbourne_write_bytes(out, &digit, 1))
      return 0;
  }
  return 1;
}

/**
 * @brief Preserve strict numeric text or normalize enabled relaxed spellings.
 * @param[in,out] out Measuring/writing output sink.
 * @param[in] num Accessible token view whose full extent must form one number.
 * @return 1 for valid normalized output that fits, 0 otherwise.
 * @details Strict decimals retain every original digit. Relaxed tokens lose a
 * leading plus, gain missing decimal zeroes or use exact hex conversion. The
 * retained nonfinite policy emits zero for NaN and signed DBL_MAX text for infinity.
 */
static inline int jsonbourne_write_num(struct jsonbourne_write_ctx *out,
                                       const jsonbourne_num *num) {
  size_t len = 0;
  size_t pos = 0;
  const unsigned char *src;
  if (!num || !num->str || num->len > (size_t)PTRDIFF_MAX)
    return 0;
  if (jsonbourne_read_num_token((const unsigned char *)num->str, num->len, 0, &len) &&
      len == num->len)
    return jsonbourne_write_bytes(out, num->str, num->len);
  if (!jsonbourne_read_num_token(
          (const unsigned char *)num->str, num->len,
          JSONBOURNE_READ_ALLOW_EXT_NUMBER | JSONBOURNE_READ_ALLOW_INF_AND_NAN, &len) ||
      len != num->len)
    return 0;
  src = (const unsigned char *)num->str;
  if (src[pos] == '+' || src[pos] == '-')
    ++pos;
  if (src[pos] == 'N')
    return jsonbourne_write_bytes(out, "0", 1);
  if (src[0] == '-' && !jsonbourne_write_bytes(out, "-", 1))
    return 0;
  if (src[pos] == 'I')
    return jsonbourne_write_bytes(out, "1.7976931348623158e308", 22);
  if (len - pos >= 2 && src[pos] == '0' && (src[pos + 1] == 'x' || src[pos + 1] == 'X'))
    return jsonbourne_write_hex_num(out, num->str + pos + 2, len - pos - 2);
  if (src[pos] == '.' && !jsonbourne_write_bytes(out, "0", 1))
    return 0;
  for (; pos < len; ++pos) {
    if (!jsonbourne_write_bytes(out, num->str + pos, 1))
      return 0;
    if (src[pos] == '.' && (pos + 1 == len || src[pos + 1] == 'e' || src[pos + 1] == 'E') &&
        !jsonbourne_write_bytes(out, "0", 1))
      return 0;
  }
  return 1;
}

/**
 * @brief Write a configured line break and one indentation unit per nesting level.
 * @param[in,out] out Output sink with already validated formatting strings.
 * @param[in] depth Number of indentation units.
 * @return 1 if no formatting is needed or it fits, 0 on capacity exhaustion.
 * @note The measuring pass counts the same repeated units, including custom lengths.
 */
static inline int jsonbourne_write_indent(struct jsonbourne_write_ctx *out, size_t depth) {
  size_t level;
  if (!out->pretty)
    return 1;
  if (!jsonbourne_write_bytes(out, out->newline, out->newline_len))
    return 0;
  for (level = 0; level < depth; ++level)
    if (!jsonbourne_write_bytes(out, out->indent, out->indent_len))
      return 0;
  return 1;
}

/**
 * @brief Translate shared walker events into JSON punctuation and scalar text.
 * @param[in,out] ctx Output sink cast from the opaque visitor context.
 * @param[in] event Value, key, element or container-end event.
 * @param[in] val Accessible current value/container.
 * @param[in] name Object key for key events, NULL otherwise.
 * @param[in] depth Current nesting/formatting level.
 * @param[in] idx Entry index or completed count, used to avoid trailing separators.
 * @return 1 if output remains valid and fits, 0 to stop traversal.
 */
static inline int jsonbourne_write_visit(void *ctx, enum jsonbourne_event event,
                                         const jsonbourne_val *val, const jsonbourne_str *name,
                                         size_t depth, size_t idx) {
  struct jsonbourne_write_ctx *out = (struct jsonbourne_write_ctx *)ctx;
  if (event == JSONBOURNE_EVENT_KEY || event == JSONBOURNE_EVENT_ELEM) {
    if ((idx && !jsonbourne_write_bytes(out, ",", 1)) || !jsonbourne_write_indent(out, depth))
      return 0;
    return !name || (jsonbourne_write_str(out, name) &&
                     jsonbourne_write_bytes(out, out->pretty ? ": " : ":", out->pretty ? 2 : 1));
  }
  if (event == JSONBOURNE_EVENT_END) {
    if (idx && !jsonbourne_write_indent(out, depth))
      return 0;
    return jsonbourne_write_bytes(out, val->type == JSONBOURNE_TYPE_OBJ ? "}" : "]", 1);
  }
  switch (val->type) {
  case JSONBOURNE_TYPE_STR:
    return jsonbourne_write_str(out, (const jsonbourne_str *)val->ptr);
  case JSONBOURNE_TYPE_NUM:
    return jsonbourne_write_num(out, (const jsonbourne_num *)val->ptr);
  case JSONBOURNE_TYPE_OBJ:
    return jsonbourne_write_bytes(out, "{", 1);
  case JSONBOURNE_TYPE_ARR:
    return jsonbourne_write_bytes(out, "[", 1);
  case JSONBOURNE_TYPE_TRUE:
    return jsonbourne_write_bytes(out, "true", 4);
  case JSONBOURNE_TYPE_FALSE:
    return jsonbourne_write_bytes(out, "false", 5);
  case JSONBOURNE_TYPE_NULL:
    return jsonbourne_write_bytes(out, "null", 4);
  default:
    return 0;
  }
}

/**
 * @brief Initialize a counting sink and validate pretty-print whitespace.
 * @param[out] out Fresh writer state with a pointer-safe maximum count.
 * @param[in] pretty Nonzero enables formatting validation/output.
 * @param[in] indent Whitespace string, or NULL for two spaces.
 * @param[in] newline Whitespace string, or NULL for LF.
 * @return 1 for usable options, 0 for non-JSON whitespace in pretty formatting.
 * @note Arbitrary formatting bytes would turn an otherwise valid DOM into invalid
 * JSON, so they are rejected before output storage is allocated or touched.
 */
static inline int jsonbourne_write_ctx_init(struct jsonbourne_write_ctx *out, int pretty,
                                            const char *indent, const char *newline) {
  size_t idx;
  const struct jsonbourne_write_ctx empty = JSONBOURNE_ZERO;
  *out = empty;
  out->cap = (size_t)PTRDIFF_MAX;
  out->pretty = pretty;
  out->indent = indent ? indent : "  ";
  out->newline = newline ? newline : "\n";
  out->indent_len = strlen(out->indent);
  out->newline_len = strlen(out->newline);
  if (!pretty)
    return 1;
  for (idx = 0; idx < out->indent_len; ++idx)
    if (!jsonbourne_char_is_space((unsigned char)out->indent[idx]))
      return 0;
  for (idx = 0; idx < out->newline_len; ++idx)
    if (!jsonbourne_char_is_space((unsigned char)out->newline[idx]))
      return 0;
  return 1;
}

/**
 * @brief Measure, allocate once and replay the same traversal into exact output storage.
 * @param[in] val Accessible DOM, stable throughout both passes and callbacks.
 * @param[in] pretty Nonzero selects pretty formatting.
 * @param[in] indent Optional indentation string.
 * @param[in] newline Optional line-separator string.
 * @param[in] alc_fn Optional allocation callback; NULL selects malloc.
 * @param[in] free_fn Optional failure release; NULL selects free.
 * @param[in] alc_ctx Opaque callback context.
 * @param[out] out_len Optional NUL-inclusive size, zero unless output succeeds.
 * @return Caller-owned JSON text, or NULL on validation, sizing or allocation failure.
 * @note The final size comparison detects size drift; it does not permit callers
 * or allocators to mutate the DOM between passes.
 */
static inline void *jsonbourne_write_document(const jsonbourne_val *val, int pretty,
                                              const char *indent, const char *newline,
                                              void *(*alc_fn)(void *, size_t),
                                              void (*free_fn)(void *, void *), void *alc_ctx,
                                              size_t *out_len) {
  struct jsonbourne_write_ctx out;
  size_t len;
  char *buf;
  if (out_len)
    *out_len = 0;
  if (!jsonbourne_write_ctx_init(&out, pretty, indent, newline) ||
      !jsonbourne_walk(val, jsonbourne_write_visit, &out) || !jsonbourne_write_bytes(&out, "", 1))
    return NULL;
  len = out.len;
  buf = (char *)(alc_fn ? alc_fn(alc_ctx, len) : malloc(len));
  if (!buf)
    return NULL;
  out.buf = buf;
  out.cap = len;
  out.len = 0;
  if (!jsonbourne_walk(val, jsonbourne_write_visit, &out) || !jsonbourne_write_bytes(&out, "", 1) ||
      out.len != len) {
    if (free_fn)
      free_fn(alc_ctx, buf);
    else
      free(buf);
    return NULL;
  }
  if (out_len)
    *out_len = len;
  return buf;
}

void *jsonbourne_val_write_opts(const jsonbourne_val *val, void *(*alc_fn)(void *, size_t),
                                void (*free_fn)(void *, void *), void *alc_ctx, size_t *out_len) {
  return jsonbourne_write_document(val, 0, NULL, NULL, alc_fn, free_fn, alc_ctx, out_len);
}

void *jsonbourne_val_write(const jsonbourne_val *val, size_t *out_len) {
  return jsonbourne_val_write_opts(val, NULL, NULL, NULL, out_len);
}

void *jsonbourne_val_write_pretty_opts(const jsonbourne_val *val, const char *indent,
                                       const char *newline, void *(*alc_fn)(void *, size_t),
                                       void (*free_fn)(void *, void *), void *alc_ctx,
                                       size_t *out_len) {
  return jsonbourne_write_document(val, 1, indent, newline, alc_fn, free_fn, alc_ctx, out_len);
}

void *jsonbourne_val_write_pretty(const jsonbourne_val *val, const char *indent,
                                  const char *newline, size_t *out_len) {
  return jsonbourne_val_write_pretty_opts(val, indent, newline, NULL, NULL, NULL, out_len);
}

/**
 * @brief Protect a short caller buffer by measuring before any output write.
 * @param[in] val Accessible, stable DOM root.
 * @param[in] pretty Nonzero enables whitespace formatting.
 * @param[in] indent Optional indentation string.
 * @param[in] newline Optional line-separator string.
 * @param[out] buf Destination, or NULL for a size probe.
 * @param[in] cap Destination capacity in bytes.
 * @param[out] out_len Optional required NUL-inclusive size after successful preflight.
 * @return 1 for complete output, 0 for failed preflight or insufficient storage.
 */
static inline int jsonbourne_write_buffer(const jsonbourne_val *val, int pretty, const char *indent,
                                          const char *newline, char *buf, size_t cap,
                                          size_t *out_len) {
  struct jsonbourne_write_ctx out;
  size_t required;
  if (out_len)
    *out_len = 0;
  if (!jsonbourne_write_ctx_init(&out, pretty, indent, newline) ||
      !jsonbourne_walk(val, jsonbourne_write_visit, &out) || !jsonbourne_write_bytes(&out, "", 1))
    return 0;
  required = out.len;
  if (out_len)
    *out_len = required;
  if (!buf || cap < required)
    return 0;
  out.buf = buf;
  out.cap = cap;
  out.len = 0;
  return jsonbourne_walk(val, jsonbourne_write_visit, &out) &&
         jsonbourne_write_bytes(&out, "", 1) && out.len == required;
}

/** Preflight before writing; insufficient buffers are left untouched. */
int jsonbourne_val_write_buf(const jsonbourne_val *val, char *buf, size_t cap, size_t *out_len) {
  return jsonbourne_write_buffer(val, 0, NULL, NULL, buf, cap, out_len);
}

int jsonbourne_val_write_pretty_buf(const jsonbourne_val *val, const char *indent,
                                    const char *newline, char *buf, size_t cap, size_t *out_len) {
  return jsonbourne_write_buffer(val, 1, indent, newline, buf, cap, out_len);
}

/** @brief Per-depth destination append state, separate from source-walker frames. */
struct jsonbourne_copy_frame {
  void *ctn;  /**< Destination object/array payload, NULL while measuring. */
  void *tail; /**< Last copied entry, preserving order without rescanning. */
};

/** @brief Copy visitor state; the arena layout is shared with the parser. */
struct jsonbourne_copy_ctx {
  struct jsonbourne_pool pool; /**< Counting or materializing destination arena. */
  struct jsonbourne_copy_frame frames[JSONBOURNE_WRITER_DEPTH_LIMIT]; /**< Bounded append stack. */
  jsonbourne_val *root;        /**< Copied root after materialization. */
  jsonbourne_val **dst;        /**< Parent/root slot awaiting the next copied value. */
};

/**
 * @brief Validate and duplicate a decoded string instead of borrowing source text.
 * @param[in,out] pool Destination arena or sizing state.
 * @param[in] src Accessible decoded UTF-8 view.
 * @return Copied record, or NULL during successful counting/on failure; inspect pool->code.
 * @note A manually assembled DOM may contain invalid UTF-8 even if parser output
 * cannot, so copying revalidates rather than trusting its provenance.
 */
static inline jsonbourne_str *jsonbourne_copy_str(struct jsonbourne_pool *pool,
                                                  const jsonbourne_str *src) {
  const size_t start = pool->str_len;
  jsonbourne_str *str;
  size_t pos = 0;
  if (!src || (!src->str && src->len) || src->len > (size_t)PTRDIFF_MAX) {
    pool->code = JSONBOURNE_READ_ERROR_INVALID_STRING;
    return NULL;
  }
  while (pos < src->len) {
    const size_t len = jsonbourne_utf8_len((const unsigned char *)src->str + pos, src->len - pos);
    if (!len) {
      pool->code = JSONBOURNE_READ_ERROR_INVALID_UTF8;
      return NULL;
    }
    pos += len;
  }
  str = (jsonbourne_str *)jsonbourne_pool_take(pool, sizeof(struct jsonbourne_str), 0);
  if (pool->code || !jsonbourne_pool_copy(pool, src->str, src->len) ||
      !jsonbourne_pool_copy(pool, "", 1))
    return NULL;
  if (str) {
    str->str = pool->str_pool + start;
    str->len = src->len;
  }
  return str;
}

/**
 * @brief Build copied records from the same validated events used for writing.
 * @param[in,out] ctx Copy visitor state and destination arena.
 * @param[in] event Walker event selecting entry setup or value duplication.
 * @param[in] src Accessible source value/container.
 * @param[in] name Source object key for key events, NULL otherwise.
 * @param[in] depth Source depth, also indexing destination append frames.
 * @param[in] idx Source entry index, unused because destination tails track appends.
 * @return 1 when duplication/counting succeeds, 0 on invalid text/number or limits.
 * @details Entry events choose the next destination slot before the child-value
 * event fills it. Measurement never dereferences those materialization-only slots.
 */
static inline int jsonbourne_copy_visit(void *ctx, enum jsonbourne_event event,
                                        const jsonbourne_val *src, const jsonbourne_str *name,
                                        size_t depth, size_t idx) {
  struct jsonbourne_copy_ctx *copy_ctx = (struct jsonbourne_copy_ctx *)ctx;
  struct jsonbourne_pool *pool = &copy_ctx->pool;
  (void)idx;
  if (event == JSONBOURNE_EVENT_END)
    return 1;
  if (event == JSONBOURNE_EVENT_KEY || event == JSONBOURNE_EVENT_ELEM) {
    struct jsonbourne_copy_frame *frame = &copy_ctx->frames[depth - 1];
    if (name) {
      jsonbourne_obj_entry *elem = (jsonbourne_obj_entry *)jsonbourne_pool_take(
          pool, sizeof(struct jsonbourne_obj_entry), 0);
      jsonbourne_str *str = jsonbourne_copy_str(pool, name);
      if (pool->code)
        return 0;
      if (elem) {
        elem->key = str;
        elem->next = NULL;
        if (frame->tail)
          ((jsonbourne_obj_entry *)frame->tail)->next = elem;
        else
          ((jsonbourne_obj *)frame->ctn)->first = elem;
        frame->tail = elem;
        copy_ctx->dst = &elem->val;
      }
    } else {
      jsonbourne_arr_entry *elem = (jsonbourne_arr_entry *)jsonbourne_pool_take(
          pool, sizeof(struct jsonbourne_arr_entry), 0);
      if (pool->code)
        return 0;
      if (elem) {
        elem->next = NULL;
        if (frame->tail)
          ((jsonbourne_arr_entry *)frame->tail)->next = elem;
        else
          ((jsonbourne_arr *)frame->ctn)->first = elem;
        frame->tail = elem;
        copy_ctx->dst = &elem->val;
      }
    }
    return 1;
  }
  {
    jsonbourne_val *val =
        (jsonbourne_val *)jsonbourne_pool_take(pool, sizeof(struct jsonbourne_val), 0);
    void *payload = NULL;
    if (pool->code)
      return 0;
    if (val) {
      *copy_ctx->dst = val;
      val->type = src->type;
    }
    switch (src->type) {
    case JSONBOURNE_TYPE_STR:
      payload = jsonbourne_copy_str(pool, (const jsonbourne_str *)src->ptr);
      break;
    case JSONBOURNE_TYPE_NUM: {
      const jsonbourne_num *num = (const jsonbourne_num *)src->ptr;
      const size_t start = pool->str_len;
      size_t len = 0;
      jsonbourne_num *copy;
      if (!num->str || num->len > (size_t)PTRDIFF_MAX ||
          !jsonbourne_read_num_token(
              (const unsigned char *)num->str, num->len,
              JSONBOURNE_READ_ALLOW_EXT_NUMBER | JSONBOURNE_READ_ALLOW_INF_AND_NAN, &len) ||
          len != num->len)
        return 0;
      copy = (jsonbourne_num *)jsonbourne_pool_take(pool, sizeof(struct jsonbourne_num), 0);
      if (pool->code || !jsonbourne_pool_copy(pool, num->str, num->len) ||
          !jsonbourne_pool_copy(pool, "", 1))
        return 0;
      if (copy) {
        copy->str = pool->str_pool + start;
        copy->len = num->len;
      }
      payload = copy;
      break;
    }
    case JSONBOURNE_TYPE_OBJ:
      payload = jsonbourne_pool_take(pool, sizeof(struct jsonbourne_obj), 0);
      if (payload) {
        ((jsonbourne_obj *)payload)->first = NULL;
        ((jsonbourne_obj *)payload)->len = ((const jsonbourne_obj *)src->ptr)->len;
      }
      break;
    case JSONBOURNE_TYPE_ARR:
      payload = jsonbourne_pool_take(pool, sizeof(struct jsonbourne_arr), 0);
      if (payload) {
        ((jsonbourne_arr *)payload)->first = NULL;
        ((jsonbourne_arr *)payload)->len = ((const jsonbourne_arr *)src->ptr)->len;
      }
      break;
    default:
      break;
    }
    if (pool->code)
      return 0;
    if (val)
      val->ptr = payload;
    if ((src->type == JSONBOURNE_TYPE_OBJ || src->type == JSONBOURNE_TYPE_ARR) &&
        depth < JSONBOURNE_WRITER_DEPTH_LIMIT) {
      copy_ctx->frames[depth].ctn = payload;
      copy_ctx->frames[depth].tail = NULL;
    }
    return 1;
  }
}

jsonbourne_val *jsonbourne_val_copy_opts(const jsonbourne_val *val, void *(*alc_fn)(void *, size_t),
                                         void *alc_ctx) {
  struct jsonbourne_copy_ctx copy_ctx = JSONBOURNE_ZERO;
  size_t val_len;
  size_t str_len;
  void *buf;
  copy_ctx.pool.val_cap = SIZE_MAX;
  copy_ctx.pool.str_cap = SIZE_MAX;
  copy_ctx.pool.max_memory = (size_t)PTRDIFF_MAX;
  if (!jsonbourne_walk(val, jsonbourne_copy_visit, &copy_ctx))
    return NULL;
  val_len = copy_ctx.pool.val_len;
  str_len = copy_ctx.pool.str_len;
  buf = alc_fn ? alc_fn(alc_ctx, val_len + str_len) : malloc(val_len + str_len);
  if (!buf)
    return NULL;
  if ((uintptr_t)buf % JSONBOURNE_ALIGNOF(struct jsonbourne_val)) {
    if (!alc_fn)
      free(buf);
    return NULL;
  }
  copy_ctx.pool.val_pool = (unsigned char *)buf;
  copy_ctx.pool.str_pool = (char *)buf + val_len;
  /* Replay into independent text/record regions; source locations are intentionally omitted. */
  copy_ctx.pool.val_len = 0;
  copy_ctx.pool.str_len = 0;
  copy_ctx.pool.val_cap = val_len;
  copy_ctx.pool.str_cap = str_len;
  copy_ctx.dst = &copy_ctx.root;
  if (!jsonbourne_walk(val, jsonbourne_copy_visit, &copy_ctx)) {
    if (!alc_fn)
      free(buf);
    return NULL;
  }
  return copy_ctx.root;
}

jsonbourne_val *jsonbourne_val_copy(const jsonbourne_val *val) {
  return jsonbourne_val_copy_opts(val, NULL, NULL);
}

/* These borrowed views inspect tags only; ownership and structural validation stay with callers. */
jsonbourne_str *jsonbourne_get_str(jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_STR ? (jsonbourne_str *)val->ptr : NULL;
}

jsonbourne_num *jsonbourne_get_num(jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_NUM ? (jsonbourne_num *)val->ptr : NULL;
}

jsonbourne_obj *jsonbourne_get_obj(jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_OBJ ? (jsonbourne_obj *)val->ptr : NULL;
}

jsonbourne_arr *jsonbourne_get_arr(jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_ARR ? (jsonbourne_arr *)val->ptr : NULL;
}

int jsonbourne_is_true(const jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_TRUE;
}

int jsonbourne_is_false(const jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_FALSE;
}

int jsonbourne_is_null(const jsonbourne_val *val) {
  return val && val->type == JSONBOURNE_TYPE_NULL;
}

/**
 * @brief Share exact-byte member comparison while keeping ambiguity policy explicit.
 * @param[in] val Accessible object value.
 * @param[in] key Decoded key bytes; NULL only with len zero.
 * @param[in] len Key byte count, including any embedded NULs.
 * @param[in] unique Nonzero scans for exactly one match; zero returns the first.
 * @return Borrowed matching value, or NULL for absent/ambiguous/invalid traversal.
 * @note Unique lookup must inspect the whole declared list: a later escaped
 * spelling can decode to the same key as an earlier member.
 */
static inline const jsonbourne_val *jsonbourne_obj_lookup(const jsonbourne_val *val,
                                                          const char *key, size_t len, int unique) {
  const jsonbourne_obj *obj;
  const jsonbourne_obj_entry *elem;
  const jsonbourne_val *found = NULL;
  const void *slow;
  const void *fast;
  size_t idx;
  if (!val || val->type != JSONBOURNE_TYPE_OBJ || !val->ptr || (!key && len) ||
      len > (size_t)PTRDIFF_MAX)
    return NULL;
  obj = (const jsonbourne_obj *)val->ptr;
  elem = obj->first;
  slow = elem;
  fast = elem;
  for (idx = 0; idx < obj->len; ++idx) {
    if (!elem || !elem->key || (!elem->key->str && elem->key->len))
      return NULL;
    if (elem->key->len == len && (!len || memcmp(elem->key->str, key, len) == 0)) {
      if (found)
        return NULL;
      found = elem->val;
      if (!unique)
        return found;
    }
    elem = elem->next;
    if (!jsonbourne_acyclic_step(&slow, &fast, JSONBOURNE_TYPE_OBJ))
      return NULL;
  }
  return elem ? NULL : found;
}

const jsonbourne_val *jsonbourne_obj_getn(const jsonbourne_val *val, const char *key, size_t len) {
  return jsonbourne_obj_lookup(val, key, len, 0);
}

const jsonbourne_val *jsonbourne_obj_getn_unique(const jsonbourne_val *val, const char *key,
                                                 size_t len) {
  return jsonbourne_obj_lookup(val, key, len, 1);
}

const jsonbourne_val *jsonbourne_arr_get(const jsonbourne_val *val, size_t idx) {
  const jsonbourne_arr *arr;
  const jsonbourne_arr_entry *elem;
  const void *slow;
  const void *fast;
  if (!val || val->type != JSONBOURNE_TYPE_ARR || !val->ptr)
    return NULL;
  arr = (const jsonbourne_arr *)val->ptr;
  if (idx >= arr->len)
    return NULL;
  elem = arr->first;
  slow = elem;
  fast = elem;
  while (idx-- && elem) {
    elem = elem->next;
    if (!jsonbourne_acyclic_step(&slow, &fast, JSONBOURNE_TYPE_ARR))
      return NULL;
  }
  return elem ? elem->val : NULL;
}

/** Convert an integer token exactly; exponent/fraction/coercion are rejected. */
int jsonbourne_num_to_uint(const jsonbourne_num *num, uint64_t *out) {
  uint64_t val = 0;
  size_t idx;
  if (!num || !num->str || !num->len || !out || num->len > (size_t)PTRDIFF_MAX ||
      (num->len > 1 && num->str[0] == '0'))
    return 0;
  for (idx = 0; idx < num->len; ++idx) {
    const unsigned char chr = (unsigned char)num->str[idx];
    const unsigned digit = (unsigned)(chr - '0');
    /* Check the rearranged bound before multiplication, when overflow is still avoidable. */
    if (!jsonbourne_char_is_digit(chr) || val > (UINT64_MAX - digit) / 10U)
      return 0;
    val = val * 10U + digit;
  }
  *out = val;
  return 1;
}

int jsonbourne_num_to_sint(const jsonbourne_num *num, int64_t *out) {
  struct jsonbourne_num magnitude;
  uint64_t val;
  int negative;
  if (!num || !num->str || !num->len || !out)
    return 0;
  negative = num->str[0] == '-';
  magnitude.str = num->str + negative;
  magnitude.len = num->len - (size_t)negative;
  if (!jsonbourne_num_to_uint(&magnitude, &val) || val > (uint64_t)INT64_MAX + (unsigned)negative)
    return 0;
  /* For INT64_MIN, val itself cannot be cast safely; val - 1 fits before negation. */
  *out = negative ? -(int64_t)(val - (val != 0)) - (val != 0) : (int64_t)val;
  return 1;
}

/**
 * @brief Compare a decoded member name with an RFC 6901 token as escapes are read.
 * @param[in] name Accessible decoded key view.
 * @param[in] token Encoded pointer token without its leading slash.
 * @param[in] len Token extent in bytes.
 * @return 1 only for equal decoded bytes and lengths, 0 otherwise.
 * @details Streaming ~0/~1 decoding avoids a temporary key allocation and keeps
 * embedded NUL handling tied to lengths rather than string terminators.
 */
static inline int jsonbourne_ptr_key_eq(const jsonbourne_str *name, const char *token, size_t len) {
  size_t input = 0;
  size_t out = 0;
  if (!name || (!name->str && name->len))
    return 0;
  while (input < len) {
    char chr = token[input++];
    if (chr == '~') {
      if (input == len || (token[input] != '0' && token[input] != '1'))
        return 0;
      chr = token[input++] == '0' ? '~' : '/';
    }
    if (out == name->len || name->str[out++] != chr)
      return 0;
  }
  return out == name->len;
}

/** RFC 6901 lookup. Object tokens must identify exactly one decoded member. */
const jsonbourne_val *jsonbourne_ptr_getn(const jsonbourne_val *val, const char *pointer,
                                          size_t len) {
  size_t pos = 0;
  if ((!pointer && len) || !val || len > (size_t)PTRDIFF_MAX)
    return NULL;
  while (pos < len) {
    size_t start;
    size_t token_len;
    if (pointer[pos++] != '/')
      return NULL;
    start = pos;
    while (pos < len && pointer[pos] != '/') {
      if (pointer[pos] == '~') {
        ++pos;
        if (pos == len || (pointer[pos] != '0' && pointer[pos] != '1'))
          return NULL;
      }
      ++pos;
    }
    token_len = pos - start;
    if (val->type == JSONBOURNE_TYPE_OBJ && val->ptr) {
      const jsonbourne_obj *obj = (const jsonbourne_obj *)val->ptr;
      const jsonbourne_obj_entry *elem = obj->first;
      const jsonbourne_val *found = NULL;
      const void *slow = elem;
      const void *fast = elem;
      size_t idx;
      for (idx = 0; idx < obj->len; ++idx) {
        if (!elem)
          return NULL;
        if (jsonbourne_ptr_key_eq(elem->key, pointer + start, token_len)) {
          if (found)
            return NULL;
          found = elem->val;
        }
        elem = elem->next;
        if (!jsonbourne_acyclic_step(&slow, &fast, JSONBOURNE_TYPE_OBJ))
          return NULL;
      }
      if (elem || !found)
        return NULL;
      val = found;
    } else if (val->type == JSONBOURNE_TYPE_ARR) {
      size_t idx = 0;
      size_t position;
      if (!token_len || (token_len > 1 && pointer[start] == '0'))
        return NULL;
      for (position = start; position < pos; ++position) {
        const unsigned char chr = (unsigned char)pointer[position];
        const unsigned digit = (unsigned)(chr - '0');
        if (!jsonbourne_char_is_digit(chr) || idx > (SIZE_MAX - digit) / 10U)
          return NULL;
        idx = idx * 10U + digit;
      }
      val = jsonbourne_arr_get(val, idx);
      if (!val)
        return NULL;
    } else
      return NULL;
  }
  return val;
}

const char *jsonbourne_read_err_desc(jsonbourne_read_code code) {
  /* Keep text out of parser control flow; unknown codes still receive a static fallback. */
  switch (code) {
  case JSONBOURNE_READ_SUCCESS:
    return JSONBOURNE_READ_SUCCESS_MSG;
  case JSONBOURNE_READ_ERROR_EXPECTED_SEPARATOR:
    return JSONBOURNE_READ_ERROR_EXPECTED_SEPARATOR_MSG;
  case JSONBOURNE_READ_ERROR_EXPECTED_COLON:
    return JSONBOURNE_READ_ERROR_EXPECTED_COLON_MSG;
  case JSONBOURNE_READ_ERROR_EXPECTED_KEY:
    return JSONBOURNE_READ_ERROR_EXPECTED_KEY_MSG;
  case JSONBOURNE_READ_ERROR_INVALID_ESCAPE:
    return JSONBOURNE_READ_ERROR_INVALID_ESCAPE_MSG;
  case JSONBOURNE_READ_ERROR_INVALID_NUMBER:
    return JSONBOURNE_READ_ERROR_INVALID_NUMBER_MSG;
  case JSONBOURNE_READ_ERROR_LITERAL:
    return JSONBOURNE_READ_ERROR_LITERAL_MSG;
  case JSONBOURNE_READ_ERROR_UNEXPECTED_END:
    return JSONBOURNE_READ_ERROR_UNEXPECTED_END_MSG;
  case JSONBOURNE_READ_ERROR_INVALID_STRING:
    return JSONBOURNE_READ_ERROR_INVALID_STRING_MSG;
  case JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION:
    return JSONBOURNE_READ_ERROR_MEMORY_ALLOCATION_MSG;
  case JSONBOURNE_READ_ERROR_UNEXPECTED_CONTENT:
    return JSONBOURNE_READ_ERROR_UNEXPECTED_CONTENT_MSG;
  case JSONBOURNE_READ_ERROR_DEPTH:
    return JSONBOURNE_READ_ERROR_DEPTH_MSG;
  case JSONBOURNE_READ_ERROR_INVALID_COMMENT:
    return JSONBOURNE_READ_ERROR_INVALID_COMMENT_MSG;
  case JSONBOURNE_READ_ERROR_INVALID_UTF8:
    return JSONBOURNE_READ_ERROR_INVALID_UTF8_MSG;
  case JSONBOURNE_READ_ERROR_LIMIT:
    return JSONBOURNE_READ_ERROR_LIMIT_MSG;
  case JSONBOURNE_READ_ERROR_INVALID_PARAMETER:
    return JSONBOURNE_READ_ERROR_INVALID_PARAMETER_MSG;
  default:
    return JSONBOURNE_READ_ERROR_UNKNOWN_MSG;
  }
}
