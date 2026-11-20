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
 * @file sha256fst.c
 * @brief Portable SHA-256 block compression, streaming and digest formatting.
 * @details Implements the SHA-256 operations from FIPS 180-4 using unsigned
 * 32-bit arithmetic. Input words and output bytes are explicitly big-endian;
 * neither pointer alignment nor the host byte order affects the result.
 * Complete input blocks bypass the context buffer. The compression schedule
 * occupies a fixed 256-byte local array; no heap or mutable global state is used.
 */

#include "sha256fst.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

/**
 * @brief Rotate a 32-bit word without signed arithmetic.
 * @param value Word to rotate.
 * @param count Rotation distance; every caller uses a constant in [1, 31].
 * @return Rotated word.
 */
static inline uint32_t rotate_right(uint32_t value, unsigned count) {
    return (value >> count) | (value << (32 - count));
}

/**
 * @brief Load a message word safely even when input is unaligned.
 * @param bytes Readable four-byte region, most significant byte first.
 * @return Decoded word.
 * @details Explicit byte loads avoid alignment and strict-aliasing violations.
 * Optimizing compilers can still select native load/byte-reversal instructions.
 */
static inline uint32_t load_big_endian(const uint8_t *bytes) {
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

/**
 * @brief Lowercase sigma-0 for expansion of the message schedule.
 * @param value Earlier schedule word.
 * @return Two rotations XORed with a logical right shift.
 */
static inline uint32_t schedule_sigma_zero(uint32_t value) {
    return rotate_right(value, 7) ^ rotate_right(value, 18) ^ (value >> 3);
}

/**
 * @brief Lowercase sigma-1 for expansion of the message schedule.
 * @param value Earlier schedule word.
 * @return Two rotations XORed with a logical right shift.
 */
static inline uint32_t schedule_sigma_one(uint32_t value) {
    return rotate_right(value, 17) ^ rotate_right(value, 19) ^ (value >> 10);
}

/**
 * @brief Uppercase Sigma-0 for the first working word in a compression round.
 * @param value Current working word A.
 * @return XOR of the three SHA-256 rotations.
 */
static inline uint32_t round_sigma_zero(uint32_t value) {
    return rotate_right(value, 2) ^ rotate_right(value, 13) ^ rotate_right(value, 22);
}

/**
 * @brief Uppercase Sigma-1 for the fifth working word in a compression round.
 * @param value Current working word E.
 * @return XOR of the three SHA-256 rotations.
 */
static inline uint32_t round_sigma_one(uint32_t value) {
    return rotate_right(value, 6) ^ rotate_right(value, 11) ^ rotate_right(value, 25);
}

/**
 * @brief Fold exactly one complete input block into the chaining state.
 * @param[in,out] context Context whose eight chaining words are updated.
 * @param[in] block Readable SHA256_BLOCK_SIZE-byte region of any alignment.
 * @details Does not change the byte count or pending buffer. Working words A-H
 * and the schedule follow the standard directly; uint32_t addition provides
 * the required modulo-2^32 wraparound on both supported architectures.
 */
static void compress_block(struct sha256 *context, const uint8_t *block) {
    /** @brief FIPS 180-4 section 4.2.2 constants, ordered by compression round. */
    static const uint32_t round_constants[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
        0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
        0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
        0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
        0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    };

    uint32_t words[64];
    for (size_t index = 0; index < 16; ++index) {
        words[index] = load_big_endian(block + index * 4);
    }
    for (size_t index = 16; index < 64; ++index) {
        words[index] = schedule_sigma_one(words[index - 2]) + words[index - 7] +
                       schedule_sigma_zero(words[index - 15]) + words[index - 16];
    }

    uint32_t state_a = context->state[0];
    uint32_t state_b = context->state[1];
    uint32_t state_c = context->state[2];
    uint32_t state_d = context->state[3];
    uint32_t state_e = context->state[4];
    uint32_t state_f = context->state[5];
    uint32_t state_g = context->state[6];
    uint32_t state_h = context->state[7];

    for (size_t round = 0; round < 64; ++round) {
        uint32_t choose = (state_e & state_f) ^ (~state_e & state_g);
        uint32_t majority = (state_a & state_b) ^ (state_a & state_c) ^ (state_b & state_c);
        uint32_t first_sum = state_h + round_sigma_one(state_e) + choose +
                             round_constants[round] + words[round];
        uint32_t second_sum = round_sigma_zero(state_a) + majority;

        state_h = state_g;
        state_g = state_f;
        state_f = state_e;
        state_e = state_d + first_sum;
        state_d = state_c;
        state_c = state_b;
        state_b = state_a;
        state_a = first_sum + second_sum;
    }

    context->state[0] += state_a;
    context->state[1] += state_b;
    context->state[2] += state_c;
    context->state[3] += state_d;
    context->state[4] += state_e;
    context->state[5] += state_f;
    context->state[6] += state_g;
    context->state[7] += state_h;
}

void sha256_init(struct sha256 *context) {
    context->state[0] = 0x6a09e667;
    context->state[1] = 0xbb67ae85;
    context->state[2] = 0x3c6ef372;
    context->state[3] = 0xa54ff53a;
    context->state[4] = 0x510e527f;
    context->state[5] = 0x9b05688c;
    context->state[6] = 0x1f83d9ab;
    context->state[7] = 0x5be0cd19;
    context->n_bits = 0;
    context->buffer_counter = 0;
}

void sha256_append_byte(struct sha256 *context, uint8_t byte) {
    context->buffer[context->buffer_counter++] = byte;
    context->n_bits += 8;
    if (context->buffer_counter == SHA256_BLOCK_SIZE) {
        compress_block(context, context->buffer);
        context->buffer_counter = 0;
    }
}

void sha256_append(struct sha256 *context, const void *data, size_t length) {
    if (length == 0) {
        return;
    }
    const uint8_t *bytes = (const uint8_t *)data;
    context->n_bits += (uint64_t)length * 8;

    /* Finish a previous chunk's partial block before reading directly from data. */
    if (context->buffer_counter != 0) {
        size_t available = SHA256_BLOCK_SIZE - context->buffer_counter;
        size_t count = length < available ? length : available;
        for (size_t index = 0; index < count; ++index) {
            context->buffer[context->buffer_counter + index] = bytes[index];
        }
        context->buffer_counter += (uint8_t)count;
        bytes += count;
        length -= count;
        if (context->buffer_counter != SHA256_BLOCK_SIZE) {
            return;
        }
        compress_block(context, context->buffer);
        context->buffer_counter = 0;
    }

    /* Full blocks need no copy; only the final 0-63 bytes must survive this call. */
    while (length >= SHA256_BLOCK_SIZE) {
        compress_block(context, bytes);
        bytes += SHA256_BLOCK_SIZE;
        length -= SHA256_BLOCK_SIZE;
    }
    for (size_t index = 0; index < length; ++index) {
        context->buffer[index] = bytes[index];
    }
    context->buffer_counter = (uint8_t)length;
}

void sha256_finalize(struct sha256 *context) {
    const uint64_t message_bits = context->n_bits;
    size_t count = context->buffer_counter;
    context->buffer[count++] = 0x80;

    /* The eight-byte length must fit entirely in the final block. */
    if (count > SHA256_BLOCK_SIZE - 8) {
        while (count < SHA256_BLOCK_SIZE) {
            context->buffer[count++] = 0;
        }
        compress_block(context, context->buffer);
        count = 0;
    }
    while (count < SHA256_BLOCK_SIZE - 8) {
        context->buffer[count++] = 0;
    }
    for (size_t index = 0; index < 8; ++index) {
        context->buffer[count + index] = (uint8_t)(message_bits >> (56 - index * 8));
    }
    compress_block(context, context->buffer);
    context->buffer_counter = 0;
}

void sha256_finalize_hex(struct sha256 *context, char *output) {
    static const char hexadecimal[] = "0123456789abcdef";
    sha256_finalize(context);
    for (size_t word = 0; word < 8; ++word) {
        for (size_t byte_index = 0; byte_index < 4; ++byte_index) {
            uint8_t byte = (uint8_t)(context->state[word] >> (24 - byte_index * 8));
            output[word * 8 + byte_index * 2] = hexadecimal[byte >> 4];
            output[word * 8 + byte_index * 2 + 1] = hexadecimal[byte & 15];
        }
    }
    output[SHA256_HEX_SIZE - 1] = '\0';
}

void sha256_finalize_bytes(struct sha256 *context, void *output) {
    uint8_t *bytes = (uint8_t *)output;
    sha256_finalize(context);
    for (size_t word = 0; word < 8; ++word) {
        for (size_t byte_index = 0; byte_index < 4; ++byte_index) {
            bytes[word * 4 + byte_index] = (uint8_t)(context->state[word] >> (24 - byte_index * 8));
        }
    }
}

void sha256_hex(const void *data, size_t length, char *output) {
    struct sha256 context;
    sha256_init(&context);
    sha256_append(&context, data, length);
    sha256_finalize_hex(&context, output);
}

void sha256_bytes(const void *data, size_t length, void *output) {
    struct sha256 context;
    sha256_init(&context);
    sha256_append(&context, data, length);
    sha256_finalize_bytes(&context, output);
}

bool sha256_file(const char *filename, char output[SHA256_HEX_SIZE]) {
    if (!filename || !output) return false;
    const int descriptor = open(filename, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    if (descriptor < 0) return false;
    struct stat before, after;
    bool valid = fstat(descriptor, &before) == 0 && S_ISREG(before.st_mode) && before.st_size > 0;
    struct sha256 hash;
    sha256_init(&hash);
    unsigned char buffer[16384];
    off_t consumed = 0;
    while (valid) {
        const ssize_t count = read(descriptor, buffer, sizeof(buffer));
        if (count < 0 && errno == EINTR) continue;
        if (count < 0) valid = false;
        if (count <= 0) break;
        if (count > before.st_size - consumed) { valid = false; break; }
        consumed += count;
        sha256_append(&hash, buffer, (size_t)count);
    }
    valid = valid && consumed == before.st_size && fstat(descriptor, &after) == 0 && before.st_dev == after.st_dev &&
        before.st_ino == after.st_ino && before.st_size == after.st_size &&
        before.st_mtim.tv_sec == after.st_mtim.tv_sec && before.st_mtim.tv_nsec == after.st_mtim.tv_nsec;
    if (close(descriptor)) valid = false;
    if (valid) sha256_finalize_hex(&hash, output);
    return valid;
}