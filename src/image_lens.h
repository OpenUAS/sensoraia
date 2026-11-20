/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USHER_IMAGE_LENS_H
#define USHER_IMAGE_LENS_H

#include <stdbool.h>
#include <stddef.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Run the native renderer with a bounded lifetime and direct argv. The clean
 * input is never changed. Only success selects output; a stale derivative from
 * an earlier failed run must never be consumed. running may be NULL. */
bool image_lens_emulate(const char *application, const char *input,
                        const char *profile, const char *cache,
                        char *output, size_t capacity,
                        const volatile sig_atomic_t *running);

#ifdef __cplusplus
}
#endif
#endif
