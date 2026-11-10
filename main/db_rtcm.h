/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2026 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

/*
 * Platform independent RTCM3 helpers for carrying RTK corrections inside MAVLink GPS_RTCM_DATA (#233):
 *  - CRC-24Q as used by RTCM3
 *  - A framer that extracts complete, CRC-checked RTCM3 frames from a raw byte stream (e.g. a GNSS base UART)
 *  - A splitter that maps one RTCM3 frame to GPS_RTCM_DATA parts following the MAVLink fragmentation rules
 *
 * No ESP-IDF dependencies, so the module can be unit tested on the host (see test/db_rtcm_test.c).
 */

#ifndef DB_ESP32_DB_RTCM_H
#define DB_ESP32_DB_RTCM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DB_RTCM3_PREAMBLE           0xD3U
#define DB_RTCM3_HEADER_LEN         3U      // preamble + 6 reserved bits + 10 bit payload length
#define DB_RTCM3_CRC_LEN            3U      // CRC-24Q
#define DB_RTCM3_MAX_PAYLOAD_LEN    1023U
#define DB_RTCM3_MAX_FRAME_LEN      (DB_RTCM3_HEADER_LEN + DB_RTCM3_MAX_PAYLOAD_LEN + DB_RTCM3_CRC_LEN)    // 1029

#define DB_RTCM_MAV_DATA_LEN        180U    // GPS_RTCM_DATA.data field length
#define DB_RTCM_MAV_MAX_FRAGMENTS   4U      // 2 bit fragment id in GPS_RTCM_DATA.flags
#define DB_RTCM_MAV_SEQUENCE_MASK   0x1FU   // 5 bit sequence id in GPS_RTCM_DATA.flags
#define DB_RTCM_MAV_MAX_PARTS       6U      // ceil(DB_RTCM3_MAX_FRAME_LEN / DB_RTCM_MAV_DATA_LEN)

/** State of the RTCM3 framer: holds the bytes of the frame currently being received. */
typedef struct {
    uint8_t buffer[DB_RTCM3_MAX_FRAME_LEN];
    uint16_t length;            // bytes currently held in buffer, buffer[0] is always the preamble if length > 0
    uint32_t frames;            // statistics: complete frames with valid CRC
    uint32_t skipped_bytes;     // statistics: bytes discarded while searching for a valid frame
} db_rtcm3_framer_t;

/**
 * Called by db_rtcm3_framer_feed() for every complete RTCM3 frame with a valid CRC.
 *
 * @param frame Complete frame (preamble, length, payload, CRC). Only valid during the call.
 * @param frame_length Number of bytes in frame
 * @param context User pointer passed to db_rtcm3_framer_feed()
 */
typedef void (*db_rtcm3_frame_cb_t)(const uint8_t *frame, uint16_t frame_length, void *context);

/** One GPS_RTCM_DATA message worth of RTCM data, produced by db_rtcm_mav_split(). */
typedef struct {
    uint8_t flags;          // GPS_RTCM_DATA.flags
    uint8_t length;         // GPS_RTCM_DATA.len, 0..180
    const uint8_t *data;    // first data byte inside the source frame, 'length' bytes are valid
} db_rtcm_mav_part_t;

/**
 * Calculates the CRC-24Q checksum used by RTCM3 (polynomial 0x1864CFB, initial value 0, no reflection).
 *
 * @param data Bytes to calculate the checksum over. May be NULL if length is 0.
 * @param length Number of bytes
 * @return 24 bit checksum
 */
uint32_t db_rtcm3_crc24q(const uint8_t *data, size_t length);

/**
 * Returns the 12 bit RTCM message number of a frame (e.g. 1005).
 *
 * @param frame Complete RTCM3 frame
 * @param frame_length Number of bytes in frame
 * @return Message number, 0 if the frame is too short to contain one
 */
uint16_t db_rtcm3_message_type(const uint8_t *frame, uint16_t frame_length);

/**
 * Resets the framer: drops any partially received frame and clears the statistics.
 *
 * @param framer Framer to initialize. Ignored if NULL.
 */
void db_rtcm3_framer_init(db_rtcm3_framer_t *framer);

/**
 * Feeds raw bytes into the framer. Every complete frame with a valid CRC is reported through the callback, in the
 * order it appeared in the stream. Bytes that do not belong to a valid frame (other protocols such as UBX or NMEA,
 * line noise, frames with a wrong CRC) are skipped and counted. Partial frames are kept until the next call.
 *
 * @param framer Framer state
 * @param data Received bytes
 * @param length Number of bytes in data
 * @param callback Called for every complete frame. May be NULL to only count frames.
 * @param context Passed unchanged to the callback
 */
void db_rtcm3_framer_feed(db_rtcm3_framer_t *framer, const uint8_t *data, size_t length,
                          db_rtcm3_frame_cb_t callback, void *context);

/**
 * Splits one RTCM3 frame into GPS_RTCM_DATA parts following the MAVLink fragmentation rules (same as ArduPilot and
 * Mission Planner expect):
 *  - frame <= 180 bytes: one unfragmented part (flags = 0)
 *  - frame <= 720 bytes: up to 4 fragments sharing one 5 bit sequence id. Every fragment but the last is 180 bytes.
 *    If the frame is an exact multiple of 180 bytes and shorter than 720, an empty last fragment marks the end.
 *    The sequence id is advanced afterwards.
 *  - frame > 720 bytes: consecutive unfragmented parts of up to 180 bytes, to be written to the GNSS receiver in order
 *
 * @param frame Complete RTCM3 frame. The parts point into this buffer.
 * @param frame_length Number of bytes in frame, 1..DB_RTCM3_MAX_FRAME_LEN
 * @param sequence In: sequence id to use for a fragmented frame. Out: next sequence id. Only bits 0..4 are used.
 * @param parts Output array with room for DB_RTCM_MAV_MAX_PARTS entries
 * @return Number of parts written, 0 on invalid input (sequence is then unchanged)
 */
uint8_t db_rtcm_mav_split(const uint8_t *frame, uint16_t frame_length, uint8_t *sequence,
                          db_rtcm_mav_part_t parts[DB_RTCM_MAV_MAX_PARTS]);

#endif // DB_ESP32_DB_RTCM_H
