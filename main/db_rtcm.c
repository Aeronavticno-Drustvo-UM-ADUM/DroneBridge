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

#include <string.h>

#include "db_rtcm.h"

#define DB_RTCM3_CRC24Q_POLY        0x1864CFBUL
#define DB_RTCM3_RESERVED_MASK      0xFCU   // upper 6 bits of the second header byte must be zero

/**
 * Calculates the CRC-24Q checksum used by RTCM3 (polynomial 0x1864CFB, initial value 0, no reflection).
 *
 * @param data Bytes to calculate the checksum over. May be NULL if length is 0.
 * @param length Number of bytes
 * @return 24 bit checksum
 */
uint32_t db_rtcm3_crc24q(const uint8_t *data, size_t length) {
    uint32_t crc = 0;
    if (data == NULL) {
        return 0;
    }
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint32_t) data[i] << 16;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc <<= 1;
            if (crc & 0x1000000UL) {
                crc ^= DB_RTCM3_CRC24Q_POLY;
            }
        }
    }
    return crc & 0xFFFFFFUL;
}

/**
 * Returns the 12 bit RTCM message number of a frame (e.g. 1005).
 *
 * @param frame Complete RTCM3 frame
 * @param frame_length Number of bytes in frame
 * @return Message number, 0 if the frame is too short to contain one
 */
uint16_t db_rtcm3_message_type(const uint8_t *frame, uint16_t frame_length) {
    if (frame == NULL || frame_length < DB_RTCM3_HEADER_LEN + 2U + DB_RTCM3_CRC_LEN) {
        return 0;
    }
    return (uint16_t) (((uint16_t) frame[3] << 4) | (frame[4] >> 4));
}

/**
 * Resets the framer: drops any partially received frame and clears the statistics.
 *
 * @param framer Framer to initialize. Ignored if NULL.
 */
void db_rtcm3_framer_init(db_rtcm3_framer_t *framer) {
    if (framer != NULL) {
        memset(framer, 0, sizeof(*framer));
    }
}

/**
 * Removes bytes from the front of the framer buffer.
 *
 * @param framer Framer state
 * @param count Number of bytes to remove, must not exceed framer->length
 */
static void db_rtcm3_framer_drop(db_rtcm3_framer_t *framer, uint16_t count) {
    if (count >= framer->length) {
        framer->length = 0;
        return;
    }
    memmove(framer->buffer, &framer->buffer[count], (size_t) (framer->length - count));
    framer->length = (uint16_t) (framer->length - count);
}

/**
 * Discards bytes from the front of the buffer until the next preamble, starting the search at 'first'.
 * Used to recover after a false frame start (bad reserved bits or a wrong CRC). The discarded bytes are counted.
 *
 * @param framer Framer state
 * @param first Index to start looking for the next preamble. 1 drops the current (bad) frame start.
 */
static void db_rtcm3_framer_skip_to_preamble(db_rtcm3_framer_t *framer, uint16_t first) {
    uint16_t next = first;
    while (next < framer->length && framer->buffer[next] != DB_RTCM3_PREAMBLE) {
        next++;
    }
    framer->skipped_bytes += next;
    db_rtcm3_framer_drop(framer, next);
}

/**
 * Checks the buffered bytes for complete frames. Reports and removes every valid frame and resynchronizes after
 * invalid ones. Returns when more bytes are needed.
 *
 * @param framer Framer state
 * @param callback Called for every complete frame, may be NULL
 * @param context Passed unchanged to the callback
 */
static void db_rtcm3_framer_evaluate(db_rtcm3_framer_t *framer, db_rtcm3_frame_cb_t callback, void *context) {
    while (framer->length > 0) {
        if (framer->buffer[0] != DB_RTCM3_PREAMBLE) {
            db_rtcm3_framer_skip_to_preamble(framer, 0);
            continue;
        }
        if (framer->length < DB_RTCM3_HEADER_LEN) {
            return;     // need the length field
        }
        if ((framer->buffer[1] & DB_RTCM3_RESERVED_MASK) != 0) {
            db_rtcm3_framer_skip_to_preamble(framer, 1);    // not a frame start
            continue;
        }
        const uint16_t payload_length = (uint16_t) (((uint16_t) (framer->buffer[1] & 0x03U) << 8) | framer->buffer[2]);
        const uint16_t frame_length = (uint16_t) (DB_RTCM3_HEADER_LEN + payload_length + DB_RTCM3_CRC_LEN);
        if (framer->length < frame_length) {
            return;     // need more bytes
        }
        const uint16_t crc_pos = (uint16_t) (DB_RTCM3_HEADER_LEN + payload_length);
        const uint32_t crc_received = ((uint32_t) framer->buffer[crc_pos] << 16) |
                                      ((uint32_t) framer->buffer[crc_pos + 1U] << 8) |
                                      (uint32_t) framer->buffer[crc_pos + 2U];
        if (db_rtcm3_crc24q(framer->buffer, crc_pos) == crc_received) {
            framer->frames++;
            if (callback != NULL) {
                callback(framer->buffer, frame_length, context);
            }
            db_rtcm3_framer_drop(framer, frame_length);
        } else {
            db_rtcm3_framer_skip_to_preamble(framer, 1);    // false start or corrupted frame
        }
    }
}

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
                          db_rtcm3_frame_cb_t callback, void *context) {
    if (framer == NULL || data == NULL) {
        return;
    }
    for (size_t i = 0; i < length; i++) {
        if (framer->length == 0 && data[i] != DB_RTCM3_PREAMBLE) {
            framer->skipped_bytes++;    // fast path while idle: nothing but a preamble can start a frame
            continue;
        }
        if (framer->length >= sizeof(framer->buffer)) {
            // Cannot happen with a consistent state (a frame is evaluated once it is complete), recover anyway.
            db_rtcm3_framer_skip_to_preamble(framer, 1);
        }
        framer->buffer[framer->length++] = data[i];
        db_rtcm3_framer_evaluate(framer, callback, context);
    }
}

/**
 * Builds the GPS_RTCM_DATA flags byte of a fragment.
 *
 * @param fragment_id Fragment index 0..3
 * @param sequence Sequence id 0..31
 * @return flags with the 'fragmented' bit set
 */
static uint8_t db_rtcm_mav_fragment_flags(uint8_t fragment_id, uint8_t sequence) {
    return (uint8_t) (0x01U | ((fragment_id & 0x03U) << 1) | ((sequence & DB_RTCM_MAV_SEQUENCE_MASK) << 3));
}

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
                          db_rtcm_mav_part_t parts[DB_RTCM_MAV_MAX_PARTS]) {
    if (frame == NULL || sequence == NULL || parts == NULL || frame_length == 0 ||
        frame_length > DB_RTCM3_MAX_FRAME_LEN) {
        return 0;
    }
    uint8_t count = 0;
    uint16_t offset = 0;
    if (frame_length <= DB_RTCM_MAV_DATA_LEN) {
        parts[0].flags = 0;
        parts[0].length = (uint8_t) frame_length;
        parts[0].data = frame;
        return 1;
    }
    if (frame_length <= DB_RTCM_MAV_DATA_LEN * DB_RTCM_MAV_MAX_FRAGMENTS) {
        const uint8_t seq = (uint8_t) (*sequence & DB_RTCM_MAV_SEQUENCE_MASK);
        while (offset < frame_length) {
            const uint16_t left = (uint16_t) (frame_length - offset);
            const uint8_t chunk = (uint8_t) (left > DB_RTCM_MAV_DATA_LEN ? DB_RTCM_MAV_DATA_LEN : left);
            parts[count].flags = db_rtcm_mav_fragment_flags(count, seq);
            parts[count].length = chunk;
            parts[count].data = &frame[offset];
            offset = (uint16_t) (offset + chunk);
            count++;
        }
        if ((frame_length % DB_RTCM_MAV_DATA_LEN) == 0 && count < DB_RTCM_MAV_MAX_FRAGMENTS) {
            // A receiver only knows a block is complete when a fragment shorter than 180 bytes arrives.
            parts[count].flags = db_rtcm_mav_fragment_flags(count, seq);
            parts[count].length = 0;
            parts[count].data = &frame[frame_length];
            count++;
        }
        *sequence = (uint8_t) ((seq + 1U) & DB_RTCM_MAV_SEQUENCE_MASK);
        return count;
    }
    while (offset < frame_length) {
        const uint16_t left = (uint16_t) (frame_length - offset);
        const uint8_t chunk = (uint8_t) (left > DB_RTCM_MAV_DATA_LEN ? DB_RTCM_MAV_DATA_LEN : left);
        parts[count].flags = 0;
        parts[count].length = chunk;
        parts[count].data = &frame[offset];
        offset = (uint16_t) (offset + chunk);
        count++;
    }
    return count;
}
