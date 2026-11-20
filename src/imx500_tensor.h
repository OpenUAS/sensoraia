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

/** @file imx500_tensor.h
 * @brief Shared IMX500 tensor-info wire layout and bounded decoding policy.
 * @details The object detector and LOOM interpret the same driver metadata.
 * Keeping the offsets here prevents their camera ABI definitions drifting apart;
 * model-specific interpretation remains with each consumer. */
#ifndef SENSORAIA_IMX500_TENSOR_H
#define SENSORAIA_IMX500_TENSOR_H

#include <stdint.h>

/** @brief Driver metadata header preceding the tensor count; not an RPK feature schema. */
#define IMX500_TENSOR_HEADER_BYTES 64U
/** @brief Little-endian 32-bit tensor count starts after the fixed header. */
#define IMX500_TENSOR_COUNT_OFFSET IMX500_TENSOR_HEADER_BYTES
/** @brief First record starts after the count, keeping offset arithmetic in one place. */
#define IMX500_TENSOR_FIRST_RECORD_OFFSET (IMX500_TENSOR_COUNT_OFFSET + sizeof(uint32_t))
/** @brief One record contains length, rank and sixteen 16-bit shape extents. */
#define IMX500_TENSOR_RECORD_BYTES 40U
/** @brief Element count offset within a record, distinct from its byte length. */
#define IMX500_TENSOR_LENGTH_OFFSET 0U
/** @brief Shape rank offset within a record. */
#define IMX500_TENSOR_RANK_OFFSET 4U
/** @brief First shape extent follows the two 32-bit fields. */
#define IMX500_TENSOR_DIMENSIONS_OFFSET 8U
/** @brief Each extent is a little-endian unsigned 16-bit value. */
#define IMX500_TENSOR_DIMENSION_BYTES 2U
/** @brief Fixed shape capacity in the driver ABI; reject greater ranks before indexing. */
#define IMX500_TENSOR_MAX_RANK 16U
/** @brief Metadata reserves sixteen records even when a model uses fewer outputs. */
#define IMX500_TENSOR_MAX_COUNT 16U
/** @brief Full accessible metadata required before following record offsets. */
#define IMX500_TENSOR_INFO_BYTES \
  (IMX500_TENSOR_FIRST_RECORD_OFFSET + IMX500_TENSOR_MAX_COUNT * IMX500_TENSOR_RECORD_BYTES)
/** @brief Sensoraia's accepted element budget, not a claim about every RPK model. */
#define IMX500_TENSOR_ELEMENT_LIMIT 4096U

#endif