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
 * @file usher_protocol.h
 * @brief Shared FC/MORA byte-stream UART framing, messages and payload layouts.
 * @details This is the shared FC/MORA contract: commands and replies cross only the
 * UART datastream. Both peers compile these message IDs, payload sizes and layouts;
 * camera implementation, storage and capture scheduling are not part of the wire API.
 * Multi-byte fields use the shared native layout; both peers must be upgraded
 * together. The unions are serialization storage, not portable network byte
 * order. There is no protocol-wide version field; the separate MORA survey
 * extension declares its own version but is not negotiated by the FC.
 *
 * |STX|length|message ID|payload (length-5 bytes)|Checksum A|Checksum B|
 *
 * Length counts the entire frame. The checksum covers length, message ID and payload:
 * @code
 * usher_ck_a = usher_ck_b = length
 * for each byte b in message ID followed by payload
 *     usher_ck_a += b;
 *     usher_ck_b += usher_ck_a;
 * @endcode
 */

#ifndef USHER_TRANSPORT_H
#define USHER_TRANSPORT_H

#include <inttypes.h>
#include <stdbool.h>

/////////////////////////////////////////////////////////////////////
// MESSAGES

#define USHER_SHOT_DATA_SIZE     (4*10)

union dc_shot_union {
  struct {
    int32_t nr;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t phi;
    int32_t theta;
    int32_t psi;
    int32_t vground;
    int32_t course;
    int32_t groundalt;
  } data;
  uint8_t bin[USHER_SHOT_DATA_SIZE];
  int32_t i[10];
};

#define USHER_BUFFER_EMPTY       2

// 0 bytes payload: null

#define USHER_PAYLOAD            3
#define USHER_PAYLOAD_MSG_SIZE   70


// 72 bytes

#define USHER_STATUS             4
#define USHER_STATUS_MSG_SIZE    (4*2)

// 4*2 bytes
union usher_status_union {
  struct usher_status_struct {
    uint16_t cpu;
    uint16_t threads;
    uint16_t shots;
    uint16_t extra;
  } data;
  uint8_t bin[USHER_STATUS_MSG_SIZE];
};

/////////////////////////////////////////////////////////////////////
// CAMERA SELECTION

#define USHER_CAMERA_ALL         0
#define USHER_CAMERA_CHDK        1
#define USHER_CAMERA_AICAM       2
#define USHER_CAMERA_LWIRCAM     3
#define USHER_CAMERA_EARCAM      4
#define USHER_CAMERA_DIPLACAM    5

#define USHER_CAMERA_MASK_NONE   0x00U
#define USHER_CAMERA_MASK_CHDK   0x01U
#define USHER_CAMERA_MASK_AICAM  0x02U
#define USHER_CAMERA_MASK_LWIR   0x04U
#define USHER_CAMERA_MASK_EAR    0x08U
#define USHER_CAMERA_MASK_DIPLA  0x10U
#define USHER_CAMERA_MASK_SUPPORTED 0x1FU
#define USHER_CAMERA_MASK_ALL    0xFFU

#define USHER_SHOOT_MASK          12
/** One trigger number for a mask of up to eight cameras. request_id correlates
 * per-camera feedback and is not a photo filename, cryptographic token or GCS ID. */
#define USHER_SHOOT_MASK_MSG_SIZE  (USHER_SHOT_DATA_SIZE + 8)

union dc_shot_mask_union {
  struct {
    union dc_shot_union shot;
    uint32_t camera_mask;
    uint32_t request_id;
  } data;
  uint8_t bin[USHER_SHOOT_MASK_MSG_SIZE];
};

/** A terminal observation for one requested camera bit. SAVED means original
 * plus required flight metadata were published, not that SODA found an object.
 * RECORDED is accepted non-image work, not an image saved to disk or a valid point.
 * Missing feedback is UNKNOWN: neither successful capture nor failure is proven. */
#define USHER_SHOT_RESULT 17
#define USHER_SHOT_RESULT_MSG_SIZE 16
#define USHER_SHOT_SAVED 1U
#define USHER_SHOT_FAILED 2U
#define USHER_SHOT_SUPERSEDED 3U
#define USHER_SHOT_RECORDED 4U
#define USHER_SHOT_UNSUPPORTED 5U

union usher_shot_result_union {
  struct {
    uint32_t request_id;
    int32_t number;
    uint32_t camera_id;
    uint32_t status;
  } data;
  uint8_t bin[USHER_SHOT_RESULT_MSG_SIZE];
};

/** Session control for one camera. A camera that accumulates evidence over time
 * (acoustic listening, recording, mapping) samples between start and stop; a
 * camera without a session ignores both. A nonzero session_id is allocated by
 * the FC and echoed in the point result; start and stop use the same token. */
#define USHER_STOP_TARGETED           6
#define USHER_STOP_TARGETED_MSG_SIZE  8
#define USHER_START_TARGETED          7
#define USHER_START_TARGETED_MSG_SIZE 8

union usher_camera_session_union {
  struct {
    uint32_t camera_id;
    uint32_t session_id;
  } data;
  uint8_t bin[USHER_START_TARGETED_MSG_SIZE];
};

/** A geographic point one camera deduced from its own session, reported on stop.
 * VALID means the camera produced an estimate, not that the estimate is accurate
 * or that the object was identified. quality is the camera's own confidence and
 * is not comparable between different camera types. */
#define USHER_CAM_POINT_RESULT          18
#define USHER_CAM_POINT_RESULT_MSG_SIZE (4*9)

#define USHER_CAM_POINT_INVALID       0
#define USHER_CAM_POINT_VALID         1

union usher_cam_point_result_union {
  struct {
    uint32_t camera_id;    // 1..8, the camera that deduced this point
    uint32_t session_id;
    uint32_t status;       // USHER_CAM_POINT_VALID or USHER_CAM_POINT_INVALID
    int32_t lat;           // 1e7 deg
    int32_t lon;           // 1e7 deg
    int32_t alt_mm;        // ellipsoid altitude of the point, same reference as dc_shot alt
    int32_t agl_mm;        // observation height above the point
    int32_t quality;       // 0..1000, camera-defined confidence
    int32_t metric;        // camera-defined scalar (EARcam: loudest level in centi-dB)
  } data;
  uint8_t bin[USHER_CAM_POINT_RESULT_MSG_SIZE];
};

/** Repeated point from a saved photo (LOOM_cam uses AIcam ID 2). The positive
 * FC photo number correlates the reply; this is NOT a stopped-session answer.
 * Units/status/quality are the same as CAM_POINT_RESULT. A no-match update is
 * INVALID and must never move a waypoint. Old firmware safely ignores ID 19. */
#define USHER_CAM_POINT_UPDATE 19
#define USHER_CAM_POINT_UPDATE_MSG_SIZE (4*9)
union usher_cam_point_update_union {
  struct {
    uint32_t camera_id;
    uint32_t frame_number;
    uint32_t status;
    int32_t lat, lon, alt_mm, agl_mm, quality, metric;
  } data;
  uint8_t bin[USHER_CAM_POINT_UPDATE_MSG_SIZE];
};

#define USHER_CONTROL_NEGOTIATE 20
#define USHER_CONTROL_HELLO 21
#define USHER_CONTROL_COMMAND 22
#define USHER_CONTROL_RESULT 23
#define USHER_CONTROL_VERSION 1U
#define USHER_CONTROL_NEGOTIATE_SIZE 9U
#define USHER_CONTROL_HELLO_SIZE 41U
#define USHER_CONTROL_COMMAND_SIZE 12U
#define USHER_CONTROL_RESULT_SIZE 14U

#define CAM_CMD_ZOOM_ABSOLUTE 1U
#define CAM_CMD_LENS_RETRACT 2U
#define CAM_CMD_KEEP_AWAKE 3U
#define CAM_CONTROL_CAP(command) (UINT32_C(1) << (command))

#define CAM_CONTROL_NOT_SENT 0U
#define CAM_CONTROL_PENDING 1U
#define CAM_CONTROL_APPLIED 2U
#define CAM_CONTROL_BUSY 3U
#define CAM_CONTROL_UNSUPPORTED 4U
#define CAM_CONTROL_INVALID_VALUE 5U
#define CAM_CONTROL_FAILED 6U
#define CAM_CONTROL_UNKNOWN 7U
#define CAM_CONTROL_PARTIAL 8U

#define CAM_CONTROL_EVIDENCE_NONE 0U
#define CAM_CONTROL_EVIDENCE_VALUE 1U
#define CAM_CONTROL_EVIDENCE_PLAYBACK 2U

/////////////////////////////////////////////////////////////////////
// SENDING

#define USHER_POSE_SAMPLE             8
#define USHER_POSE_SAMPLE_MSG_SIZE    100
#define USHER_POSE_SAMPLE_GPS_PRESENT 1U
#define USHER_POSE_SAMPLE_GPS_HELD 2U
#define USHER_POSE_SAMPLE_GPS_AGE_VALID 4U
#define USHER_POSE_SAMPLE_GPS_AGE_SHIFT 8U
#define USHER_POSE_SAMPLE_GPS_AGE_MASK UINT32_C(0xffffff00)

/** @brief Check the structural consistency of pose-sample GPS flags.
 * @param flags Received wire flags including the encoded age field.
 * @return True if all bits are known and the GPS presence and age dependencies hold.
 * @details Held GPS requires a present fix and valid age. Reject stray age
 * bits rather than treating them as qualified timing evidence. */
static inline bool usher_pose_flags_valid(uint32_t flags)
{
  const uint32_t known = USHER_POSE_SAMPLE_GPS_PRESENT | USHER_POSE_SAMPLE_GPS_HELD
                         | USHER_POSE_SAMPLE_GPS_AGE_VALID | USHER_POSE_SAMPLE_GPS_AGE_MASK;
  return !(flags & ~known)
         && (!(flags & (USHER_POSE_SAMPLE_GPS_HELD | USHER_POSE_SAMPLE_GPS_AGE_VALID))
             || (flags & USHER_POSE_SAMPLE_GPS_PRESENT))
         && (!(flags & (USHER_POSE_SAMPLE_GPS_HELD | USHER_POSE_SAMPLE_GPS_AGE_MASK))
             || (flags & USHER_POSE_SAMPLE_GPS_AGE_VALID));
}

union usher_pose_sample_union {
  struct {
    uint32_t sequence;
    uint32_t sample_begin_us;
    uint32_t sample_end_us;
    union dc_shot_union shot;
    int32_t velocity_north_bfp;
    int32_t velocity_east_bfp;
    int32_t velocity_down_bfp;
    uint32_t gps_tow_ms;
    uint32_t gps_week;
    uint32_t gps_hacc_cm;
    uint32_t gps_vacc_cm;
    uint32_t gps_sacc_cm_s;
    uint32_t gps_fix;
    uint32_t gps_num_sv;
    uint32_t gps_valid_fields;
    uint32_t flags;
  } data;
  uint8_t bin[USHER_POSE_SAMPLE_MSG_SIZE];
};

#define USHER_CLOCK_REQUEST           9
#define USHER_CLOCK_REQUEST_MSG_SIZE  8
#define USHER_CLOCK_REPLY             10
#define USHER_CLOCK_REPLY_MSG_SIZE    16
#define USHER_POSE_CLOCKED            11
#define USHER_POSE_CLOCKED_MSG_SIZE   (USHER_POSE_SAMPLE_MSG_SIZE + 8)

union usher_clock_request_union {
  struct {
    uint32_t token_low;
    uint32_t token_high;
  } data;
  uint8_t bin[USHER_CLOCK_REQUEST_MSG_SIZE];
};

union usher_clock_reply_union {
  struct {
    union usher_clock_request_union request;
    uint32_t receive_us;
    uint32_t transmit_us;
  } data;
  uint8_t bin[USHER_CLOCK_REPLY_MSG_SIZE];
};

union usher_pose_clocked_union {
  struct {
    union usher_pose_sample_union sample;
    union usher_clock_request_union request;
  } data;
  uint8_t bin[USHER_POSE_CLOCKED_MSG_SIZE];
};

/** @name Negotiated survey messages
 * @brief Separate MORA survey extension; ordinary masked capture does not use it.
 * @details A nonzero session token identifies the running USHER peer, not a
 * persistent mission or an authenticated sender. The FC does not currently
 * negotiate or emit this extension; ordinary masked shots use USHER_SHOT_RESULT
 * for per-camera feedback. Preserve explicit sizes when extending structures.
 * @{ */
#define USHER_CAPTURE_ARBITER_VERSION 1U
#define USHER_CAPTURE_ARBITER_HELLO 13
#define USHER_CAPTURE_ARBITER_STATE 14
#define USHER_CAPTURE_ARBITER_SHOOT 15
#define USHER_CAPTURE_ARBITER_RESULT 16
#define USHER_CAPTURE_ARBITER_HELLO_MSG_SIZE 8
#define USHER_CAPTURE_ARBITER_STATE_MSG_SIZE 12
#define USHER_CAPTURE_ARBITER_SHOOT_MSG_SIZE (USHER_SHOT_DATA_SIZE + 12)
#define USHER_CAPTURE_ARBITER_RESULT_MSG_SIZE 24
#define USHER_CAPTURE_ARBITER_SAVED 1U      /**< Original and required metadata published; not proof of an object detection. */
#define USHER_CAPTURE_ARBITER_FAILED 2U     /**< Capture/publication failed; other camera bits can still succeed. */
#define USHER_CAPTURE_ARBITER_EXPIRED 3U    /**< Intent or scope no longer current; no exposure is implied. */
#define USHER_CAPTURE_ARBITER_COVERED 4U    /**< Suppressed using trusted same-camera coverage, not a new photo. */
#define USHER_CAPTURE_ARBITER_SUPERSEDED 5U /**< Waiting capacity reused within this camera; not successful recovery. */

/** @brief USHER-to-FC capability advertisement.
 * @details version selects this contract; session changes between configured
 * USHER processes. Receipt renews the FC lease, not the lifetime of old intentions.
 */
union usher_capture_arbiter_hello_union {
  struct { uint32_t version, session; } data;
  uint8_t bin[USHER_CAPTURE_ARBITER_HELLO_MSG_SIZE];
};
/** @brief FC-to-USHER survey authorization, independent of individual shot requests.
 * @details active is 0 or 1; camera_mask selects allowed sensors for this session.
 * A stop withdraws waiting work without discarding pixels already being saved.
 */
union usher_capture_arbiter_state_union {
  struct { uint32_t session, active, camera_mask; } data;
  uint8_t bin[USHER_CAPTURE_ARBITER_STATE_MSG_SIZE];
};
/** @brief FC-to-USHER spatial observation request, not a physical shutter acknowledgement.
 * @details shot retains the original FC number/pose, camera_mask may select several
 * sensors, and fc_time_us is in the FC's wrapping clock domain. Never compare that
 * timestamp directly with MORA monotonic time. Each camera completes independently.
 */
union usher_capture_arbiter_shoot_union {
  struct { union dc_shot_union shot; uint32_t session, camera_mask, fc_time_us; } data;
  uint8_t bin[USHER_CAPTURE_ARBITER_SHOOT_MSG_SIZE];
};
/** @brief Best-effort USHER-to-FC terminal result for one camera and shot number.
 * @details ready_mask is a snapshot, not a reservation. saved_count is the aggregate
 * successful image count in this USHER session, not a per-camera counter. A full
 * queue or lost link may lose results; never equate absence of telemetry with
 * absence of an original on disk, or treat this as an exactly-once event stream.
 */
union usher_capture_arbiter_result_union {
  struct { uint32_t session; int32_t number; uint32_t camera_id, result, ready_mask, saved_count; } data;
  uint8_t bin[USHER_CAPTURE_ARBITER_RESULT_MSG_SIZE];
};
/** @} */

// Each platform supplies CameraLinkTransmit; this shared header stays OS-independent.

extern uint8_t usher_ck_a, usher_ck_b;

#define STX  0x99

#define UsherSizeOf(_payload) (_payload+5)

#define UsherPutUint8( _byte) {     \
    usher_ck_a += _byte;              \
    usher_ck_b += usher_ck_a;          \
    CameraLinkTransmit(_byte);     \
  }

#define UsherHeader(msg_id, payload_len) {           \
    CameraLinkTransmit(STX);                        \
    uint8_t msg_len = UsherSizeOf( payload_len);       \
    CameraLinkTransmit(msg_len);                    \
    usher_ck_a = msg_len; usher_ck_b = msg_len;         \
    UsherPutUint8(msg_id);                             \
  }

#define UsherTrailer() {               \
    CameraLinkTransmit(usher_ck_a);    \
    CameraLinkTransmit(usher_ck_b);    \
  }

#define UsherPut1ByteByAddr( _byte) {  \
    uint8_t _x = *(_byte);              \
    UsherPutUint8( _x);                  \
  }

/////////////////////////////////////////////////////////////////////
// PARSING

/**
 * @brief Persistent state for incremental USHER UART frame parsing.
 * @details One instance must remain associated with one ordered byte stream. A completed
 * message remains in @c payload until its consumer clears @c msg_received; feeding later
 * bytes before that acknowledgement is counted as an error to avoid silently overwriting it.
 */
struct usher_transport {
  /** Storage for the validated payload of the most recently completed message. */
  uint8_t payload[256];
  /** Count of framing or checksum faults observed while consuming this stream. */
  uint8_t error;
  /** Message identifier from the current or most recently completed frame. */
  uint8_t msg_id;
  /** True when @c payload contains one complete validated message awaiting dispatch. */
  bool    msg_received;
  /** Number of payload bytes expected in the current frame. */
  uint8_t payload_len;
  /** Internal parser state: the next expected position in the frame. */
  uint8_t status;
  /** Internal offset into @c payload while a frame is being assembled. */
  uint8_t payload_idx;
  /** Running Fletcher-style checksum accumulators for the in-progress frame. */
  uint8_t ck_a, ck_b;
};

extern struct usher_transport usher_protocol;

/**
 * @brief Consume one byte from a USHER UART stream.
 * @param t Persistent parser state for that stream.
 * @param c Newly received byte.
 * @details Frames may be arbitrarily split across calls. On a valid complete frame this sets
 * @c t->msg_received and leaves the decoded bytes in @c t->payload. The caller must dispatch
 * the message and clear that flag before supplying bytes for the next frame.
 * @warning This function is not thread-safe. Serialize access to each @p t instance and do not
 * alter its fields while a frame is in progress.
 */
void parse_usher(struct usher_transport *t, uint8_t c);

#endif

