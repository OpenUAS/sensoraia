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

#ifndef USHER_SPOTTERCAM_PIPE_H
#define USHER_SPOTTERCAM_PIPE_H

/**
 * @file spotter_cam_pipe.h
 * @brief Native USHER adapter for on-sensor detection or photo-only capture.
 * @details Uses the neutral native-worker supervisor,
 * then interprets only the matching detector sidecar. Capture success and usable
 * analysis are separate outcomes; the last result is retrieved separately for EXIF.
 * @pre Configuration, capture, binding and result retrieval are serialized by
 * the owning backend. This adapter has one parser workspace and one last result.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

struct camera_backend_host;

#define SPOTTER_MAX_DETECTIONS 64U
#define SPOTTER_LABEL_CAPACITY 64U

struct spotter_detection {
	char label[SPOTTER_LABEL_CAPACITY];
	double confidence;
	double center_x, center_y;
	bool precise_center;
	unsigned x, y, width, height;
};

struct spotter_detections {
	uint64_t sequence;
	unsigned width, height, count;
	char image_sha256[65];
	char model_sha256[65], descriptor_sha256[65];
	struct spotter_detection objects[SPOTTER_MAX_DETECTIONS];
};

bool spotter_cam_pipe_read_detections(const char *filename, struct spotter_detections *detections);

/** @brief Borrow USHER's shutdown flag for both serialized native worker owners. */
void spotter_cam_pipe_configure(const struct camera_backend_host *host);

/** @brief Select or inspect the model-free capture mode before worker startup. */
int spotter_cam_pipe_set_photo_only(bool enabled);
bool spotter_cam_pipe_photo_only(void);

/** @brief Start and qualify native persistent capture with the selected analysis policy.
 * @param unused Reserved common backend argument.
 * @return 0 after the selected producer qualifies readiness, otherwise -1.
 * @details Process startup alone is insufficient: the capture owner must report
 * its initial frame and, when enabled, inference contract before accepting photographs. */
int spotter_cam_pipe_init(const char *unused);
/** @brief Set the borrowed output directory before backend initialization.
 * @param directory Caller-owned path retained until configuration changes; NULL
 * restores the build-time default.
 * @pre No capture is using the previous configuration. */
void spotter_cam_pipe_set_photo_directory(const char *directory);
/** @brief Request one JPEG and, when enabled, parse its matching detection sidecar.
 * @param filename Output path buffer, cleared on failure.
 * @param filename_size Output path capacity.
 * @param image_number USHER shot sequence.
 * @return 0 only for a qualified saved image, otherwise -1.
 * @details Missing or malformed analysis leaves the photograph usable and makes
 * the summary report analysis_failed. Clearing the old result prevents a previous
 * hit being attached to the next exposure. */
int spotter_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);
/** @brief Stop the native camera process and clear detector-specific configuration. */
void spotter_cam_pipe_deinit(void);
/** @brief Format the most recent shot's detection result for EXIF writing.
 * @param buffer Caller-owned destination for a short human-readable summary.
 * @param buffer_size Capacity of @p buffer.
 * @details Reflects the last spotter_cam_pipe_shoot() call only; safe to call even
 * when that call failed (reports status=analysis_failed) or no server ever ran. */
void spotter_cam_pipe_last_detection_summary(char *buffer, size_t buffer_size);
/** @brief Reject a detection result whose identity differs from the validated saved frame.
 * @param sequence Camera sequence obtained from that image's validated frame evidence.
 * @param sensor_timestamp_ns Matching sensor timestamp, not the FC shot time.
 * @return True for matching detector identity; false clears the retained analysis.
 * @details INI parsing alone does not bind detections to pixels. Compare both
 * fields after shared frame/file validation and before EXIF replacement changes the inode. */
bool spotter_cam_pipe_bind_frame(uint64_t sequence, uint64_t sensor_timestamp_ns);
/** @brief Copy @p filename into a "detections" subdirectory of the photo directory
 * if the last spotter_cam_pipe_shoot() call found an object.
 * @param filename Path to the just-finished image (ideally already EXIF-tagged by the
 * caller, so the copy carries the same metadata as the original).
 * @details No-op when the last shot found nothing, never completed, or @p filename is
 * NULL/empty; safe to call unconditionally, mirroring spotter_cam_pipe_last_detection_summary(). */
void spotter_cam_pipe_save_detection_copy(const char *filename);
/** @brief Select the detector assignment before either capture or saved-picture analysis starts.
 * @param settings Nonempty borrowed application settings path, alive until both workers stop.
 * @return Zero if accepted; minus one for invalid input or an already active worker. */
int spotter_cam_pipe_set_settings(const char *settings);
/** @brief Analyze a saved simulation/private picture and bind results to its source hash and request.
 * @param filename Saved JPEG, still owned by the producer during analysis.
 * @param number Positive increasing correlation number.
 * @return Zero when disabled or a matching result is accepted, minus one with retained detections cleared.
 * @details A valid empty result remains distinct from failed or mismatched analysis. */
int spotter_cam_pipe_analyze(const char *filename, unsigned number);

#endif
