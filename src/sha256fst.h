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

#ifndef SHA256FST_H
#define SHA256FST_H

/**
 * @file sha256fst.h
 * @brief Allocation-free, portable SHA-256 checksums for memory and streams.
 * @details Use sha256_hex() or sha256_bytes() for a complete memory region.
 * For a stream, initialize a context, append any number of chunks, then call
 * exactly one finalizer. Reinitialize before hashing another message.
 *
 * Context storage belongs to the caller. Separate contexts can be used
 * concurrently; sharing one context requires external synchronization.
 * Inputs need no special alignment. The same digest bytes are produced on
 * x86 and ARM64 without hardware crypto instructions or host-endian casts.
 * @pre Total input length must be less than 2^64 bits, as required by SHA-256.
 * @warning A checksum detects changes but does not authenticate a producer.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Number of input bytes in one SHA-256 compression block. */
#define SHA256_BLOCK_SIZE 64
/** @brief Required hexadecimal output capacity, including the trailing NUL. */
#define SHA256_HEX_SIZE 65
/** @brief Required binary output capacity; binary digests have no terminator. */
#define SHA256_BYTES_SIZE 32

/**
 * @brief Caller-owned incremental checksum state; treat members as read-only.
 * @details The layout is retained for existing C and C++ consumers. Only
 * buffer_counter bytes of buffer are valid before finalization; initialization
 * deliberately does not clear unused buffer storage. No input pointer is kept.
 */
typedef struct sha256 {
    uint32_t state[8]; /**< Chaining words; the finalizer replaces these with the digest. */
    uint8_t buffer[SHA256_BLOCK_SIZE]; /**< Pending partial block or final padding block. */
    uint64_t n_bits; /**< Original message length in bitch, excluding padding. */
    uint8_t buffer_counter; /**< Pending byte count, always below SHA256_BLOCK_SIZE between calls. */
} sha256;

/**
 * @brief Initialize or reset a context for a new message.
 * @param[out] context Writable caller-owned context.
 * @note Initialization is not a secure erasure of previous buffer contents.
 */
void sha256_init(struct sha256 *context);

/**
 * @brief Append one byte to an unfinished message.
 * @param[in,out] context Initialized context that has not been finalized.
 * @param byte Next input byte.
 * @note Prefer sha256_append() for chunks to use its direct-block fast path.
 */
void sha256_append_byte(struct sha256 *context, uint8_t byte);

/**
 * @brief Append an arbitrarily aligned chunk; empty chunks are harmless.
 * @param[in,out] context Initialized context that has not been finalized.
 * @param[in] data Readable input of at least length bytes; may be NULL only when length is zero.
 * @param length Number of bytes to consume.
 * @pre The input region must not overlap context storage.
 * @details Complete blocks are compressed directly from data. Only partial
 * blocks are copied into the bounded context buffer; no input is retained.
 */
void sha256_append(struct sha256 *context, const void *data, size_t length);

/**
 * @brief Finish a message, leaving its eight digest words in context->state.
 * @param[in,out] context Initialized context that has not been finalized.
 * @details This consumes the context. Read the final state if needed, then
 * reinitialize; do not append or call another finalizer on the same message.
 * n_bits retains the original input length and buffer_counter becomes zero.
 */
void sha256_finalize(struct sha256 *context);

/**
 * @brief Finish a message and write its lowercase hexadecimal digest.
 * @param[in,out] context Initialized context that has not been finalized.
 * @param[out] output Writable SHA256_HEX_SIZE-byte region, including the trailing NUL.
 * @pre Output must not overlap context storage.
 * @note This consumes the context, just like sha256_finalize().
 */
void sha256_finalize_hex(struct sha256 *context, char *output);

/**
 * @brief Finish a message and write its standard big-endian digest bytes.
 * @param[in,out] context Initialized context that has not been finalized.
 * @param[out] output Writable SHA256_BYTES_SIZE-byte region, with no trailing NUL.
 * @pre Output must not overlap context storage.
 * @note This consumes the context, just like sha256_finalize().
 */
void sha256_finalize_bytes(struct sha256 *context, void *output);

/**
 * @brief Hash one memory region to a lowercase hexadecimal string.
 * @param[in] data Readable input of at least length bytes; may be NULL only when length is zero.
 * @param length Input byte count.
 * @param[out] output Writable SHA256_HEX_SIZE-byte region, including the trailing NUL.
 * @details Uses a temporary stack context. Output may overlap the input because
 * all input is consumed before the digest is written.
 */
void sha256_hex(const void *data, size_t length, char *output);

/**
 * @brief Hash one memory region to its standard 32-byte binary digest.
 * @param[in] data Readable input of at least length bytes; may be NULL only when length is zero.
 * @param length Input byte count.
 * @param[out] output Writable SHA256_BYTES_SIZE-byte region, with no trailing NUL.
 * @details Uses a temporary stack context; output may overlap the input.
 */
void sha256_bytes(const void *data, size_t length, void *output);
/** @brief Hash an immutable nonempty regular file with bounded streaming scratch.
 * @param filename Input whose size/identity/mtime must remain stable during the read.
 * @param output Writable SHA256_HEX_SIZE bytes, meaningful only after success.
 * @return True after a complete stable read and close, false for I/O or observed mutation.
 * @note Integrity is not authentication; the producer still owns file/path stability. */
bool sha256_file(const char *filename, char output[SHA256_HEX_SIZE]);

#ifdef __cplusplus
}
#endif

#endif