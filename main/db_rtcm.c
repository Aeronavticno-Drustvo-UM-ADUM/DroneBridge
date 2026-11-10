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

#define DB_RTCM3_RESERVED_MASK      0xFCU   // upper 6 bits of the second header byte must be zero
#define DB_RTCM3_TYPE_LEN           2U      // payload bytes needed to read the 12 bit message number

/**
 * CRC-24Q lookup table (polynomial 0x1864CFB). Table driven: about 8x less work per byte than bit by bit, which bounds
 * the CPU time the framer can spend on hostile or broken input.
 */
static const uint32_t db_rtcm3_crc24q_table[256] = {
    0x000000UL, 0x864CFBUL, 0x8AD50DUL, 0x0C99F6UL, 0x93E6E1UL, 0x15AA1AUL, 0x1933ECUL, 0x9F7F17UL,
    0xA18139UL, 0x27CDC2UL, 0x2B5434UL, 0xAD18CFUL, 0x3267D8UL, 0xB42B23UL, 0xB8B2D5UL, 0x3EFE2EUL,
    0xC54E89UL, 0x430272UL, 0x4F9B84UL, 0xC9D77FUL, 0x56A868UL, 0xD0E493UL, 0xDC7D65UL, 0x5A319EUL,
    0x64CFB0UL, 0xE2834BUL, 0xEE1ABDUL, 0x685646UL, 0xF72951UL, 0x7165AAUL, 0x7DFC5CUL, 0xFBB0A7UL,
    0x0CD1E9UL, 0x8A9D12UL, 0x8604E4UL, 0x00481FUL, 0x9F3708UL, 0x197BF3UL, 0x15E205UL, 0x93AEFEUL,
    0xAD50D0UL, 0x2B1C2BUL, 0x2785DDUL, 0xA1C926UL, 0x3EB631UL, 0xB8FACAUL, 0xB4633CUL, 0x322FC7UL,
    0xC99F60UL, 0x4FD39BUL, 0x434A6DUL, 0xC50696UL, 0x5A7981UL, 0xDC357AUL, 0xD0AC8CUL, 0x56E077UL,
    0x681E59UL, 0xEE52A2UL, 0xE2CB54UL, 0x6487AFUL, 0xFBF8B8UL, 0x7DB443UL, 0x712DB5UL, 0xF7614EUL,
    0x19A3D2UL, 0x9FEF29UL, 0x9376DFUL, 0x153A24UL, 0x8A4533UL, 0x0C09C8UL, 0x00903EUL, 0x86DCC5UL,
    0xB822EBUL, 0x3E6E10UL, 0x32F7E6UL, 0xB4BB1DUL, 0x2BC40AUL, 0xAD88F1UL, 0xA11107UL, 0x275DFCUL,
    0xDCED5BUL, 0x5AA1A0UL, 0x563856UL, 0xD074ADUL, 0x4F0BBAUL, 0xC94741UL, 0xC5DEB7UL, 0x43924CUL,
    0x7D6C62UL, 0xFB2099UL, 0xF7B96FUL, 0x71F594UL, 0xEE8A83UL, 0x68C678UL, 0x645F8EUL, 0xE21375UL,
    0x15723BUL, 0x933EC0UL, 0x9FA736UL, 0x19EBCDUL, 0x8694DAUL, 0x00D821UL, 0x0C41D7UL, 0x8A0D2CUL,
    0xB4F302UL, 0x32BFF9UL, 0x3E260FUL, 0xB86AF4UL, 0x2715E3UL, 0xA15918UL, 0xADC0EEUL, 0x2B8C15UL,
    0xD03CB2UL, 0x567049UL, 0x5AE9BFUL, 0xDCA544UL, 0x43DA53UL, 0xC596A8UL, 0xC90F5EUL, 0x4F43A5UL,
    0x71BD8BUL, 0xF7F170UL, 0xFB6886UL, 0x7D247DUL, 0xE25B6AUL, 0x641791UL, 0x688E67UL, 0xEEC29CUL,
    0x3347A4UL, 0xB50B5FUL, 0xB992A9UL, 0x3FDE52UL, 0xA0A145UL, 0x26EDBEUL, 0x2A7448UL, 0xAC38B3UL,
    0x92C69DUL, 0x148A66UL, 0x181390UL, 0x9E5F6BUL, 0x01207CUL, 0x876C87UL, 0x8BF571UL, 0x0DB98AUL,
    0xF6092DUL, 0x7045D6UL, 0x7CDC20UL, 0xFA90DBUL, 0x65EFCCUL, 0xE3A337UL, 0xEF3AC1UL, 0x69763AUL,
    0x578814UL, 0xD1C4EFUL, 0xDD5D19UL, 0x5B11E2UL, 0xC46EF5UL, 0x42220EUL, 0x4EBBF8UL, 0xC8F703UL,
    0x3F964DUL, 0xB9DAB6UL, 0xB54340UL, 0x330FBBUL, 0xAC70ACUL, 0x2A3C57UL, 0x26A5A1UL, 0xA0E95AUL,
    0x9E1774UL, 0x185B8FUL, 0x14C279UL, 0x928E82UL, 0x0DF195UL, 0x8BBD6EUL, 0x872498UL, 0x016863UL,
    0xFAD8C4UL, 0x7C943FUL, 0x700DC9UL, 0xF64132UL, 0x693E25UL, 0xEF72DEUL, 0xE3EB28UL, 0x65A7D3UL,
    0x5B59FDUL, 0xDD1506UL, 0xD18CF0UL, 0x57C00BUL, 0xC8BF1CUL, 0x4EF3E7UL, 0x426A11UL, 0xC426EAUL,
    0x2AE476UL, 0xACA88DUL, 0xA0317BUL, 0x267D80UL, 0xB90297UL, 0x3F4E6CUL, 0x33D79AUL, 0xB59B61UL,
    0x8B654FUL, 0x0D29B4UL, 0x01B042UL, 0x87FCB9UL, 0x1883AEUL, 0x9ECF55UL, 0x9256A3UL, 0x141A58UL,
    0xEFAAFFUL, 0x69E604UL, 0x657FF2UL, 0xE33309UL, 0x7C4C1EUL, 0xFA00E5UL, 0xF69913UL, 0x70D5E8UL,
    0x4E2BC6UL, 0xC8673DUL, 0xC4FECBUL, 0x42B230UL, 0xDDCD27UL, 0x5B81DCUL, 0x57182AUL, 0xD154D1UL,
    0x26359FUL, 0xA07964UL, 0xACE092UL, 0x2AAC69UL, 0xB5D37EUL, 0x339F85UL, 0x3F0673UL, 0xB94A88UL,
    0x87B4A6UL, 0x01F85DUL, 0x0D61ABUL, 0x8B2D50UL, 0x145247UL, 0x921EBCUL, 0x9E874AUL, 0x18CBB1UL,
    0xE37B16UL, 0x6537EDUL, 0x69AE1BUL, 0xEFE2E0UL, 0x709DF7UL, 0xF6D10CUL, 0xFA48FAUL, 0x7C0401UL,
    0x42FA2FUL, 0xC4B6D4UL, 0xC82F22UL, 0x4E63D9UL, 0xD11CCEUL, 0x575035UL, 0x5BC9C3UL, 0xDD8538UL,
};

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
        crc = ((crc << 8) ^ db_rtcm3_crc24q_table[((crc >> 16) ^ data[i]) & 0xFFU]) & 0xFFFFFFUL;
    }
    return crc;
}

/**
 * Tells if a message number belongs to the RTCM 3 ranges real GNSS bases send: standard messages 1001-1299 (base
 * position 1005/1006, legacy and MSM observations 1001-1137, GLONASS biases 1230, ...) and proprietary 4001-4095
 * (e.g. u-blox 4072). Used to reject false frame starts after 5 bytes instead of after up to 1029 bytes.
 *
 * @param message_type 12 bit RTCM message number
 * @return true if the number is plausible for a correction stream
 */
bool db_rtcm3_message_type_is_plausible(uint16_t message_type) {
    return (message_type >= 1001U && message_type <= 1299U) || (message_type >= 4001U && message_type <= 4095U);
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
        if (payload_length < DB_RTCM3_TYPE_LEN) {
            db_rtcm3_framer_skip_to_preamble(framer, 1);    // no room for a message number: not a correction message
            continue;
        }
        if (framer->length < DB_RTCM3_HEADER_LEN + DB_RTCM3_TYPE_LEN) {
            return;     // need the message number
        }
        const uint16_t message_type = (uint16_t) (((uint16_t) framer->buffer[3] << 4) | (framer->buffer[4] >> 4));
        if (!db_rtcm3_message_type_is_plausible(message_type)) {
            db_rtcm3_framer_skip_to_preamble(framer, 1);    // reject a false start early instead of after 1029 bytes
            continue;
        }
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
 * Feeds raw bytes into the framer. Every complete frame with a valid CRC and a plausible message number (see
 * db_rtcm3_message_type_is_plausible()) is reported through the callback, in the order it appeared in the stream.
 * Bytes that do not belong to such a frame (other protocols such as UBX or NMEA, line noise, frames with a wrong CRC,
 * frames without a message number) are skipped and counted. Partial frames are kept until the next call.
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
 * Tells if a complete frame with a plausible message number and a valid CRC starts at an index of the buffer.
 *
 * @param framer Framer state
 * @param start Index of a preamble inside the buffer
 * @return true if buffer[start..] holds such a frame completely
 */
static bool db_rtcm3_framer_complete_frame_at(const db_rtcm3_framer_t *framer, uint16_t start) {
    const uint16_t available = (uint16_t) (framer->length - start);
    const uint8_t *frame = &framer->buffer[start];
    if (available < DB_RTCM3_HEADER_LEN + DB_RTCM3_TYPE_LEN || (frame[1] & DB_RTCM3_RESERVED_MASK) != 0) {
        return false;
    }
    const uint16_t payload_length = (uint16_t) (((uint16_t) (frame[1] & 0x03U) << 8) | frame[2]);
    const uint16_t frame_length = (uint16_t) (DB_RTCM3_HEADER_LEN + payload_length + DB_RTCM3_CRC_LEN);
    if (payload_length < DB_RTCM3_TYPE_LEN || available < frame_length ||
        !db_rtcm3_message_type_is_plausible((uint16_t) (((uint16_t) frame[3] << 4) | (frame[4] >> 4)))) {
        return false;
    }
    const uint16_t crc_pos = (uint16_t) (DB_RTCM3_HEADER_LEN + payload_length);
    const uint32_t crc_received = ((uint32_t) frame[crc_pos] << 16) | ((uint32_t) frame[crc_pos + 1U] << 8) |
                                  (uint32_t) frame[crc_pos + 2U];
    return db_rtcm3_crc24q(frame, crc_pos) == crc_received;
}

/**
 * Call when the input has been quiet for a while (e.g. on the serial read timeout). If the framer is still waiting
 * for the rest of a frame, but a complete valid frame already starts later in its buffer, the frame start it is
 * waiting on was a false one (e.g. a corrupted byte that looks like a header claiming a long frame). That false start
 * is dropped, so the real frames behind it are reported now instead of when enough further bytes have arrived. A real
 * frame that is merely paused in the middle never contains a complete valid frame, so it is kept.
 *
 * @param framer Framer state
 * @param callback Called for every complete frame, may be NULL
 * @param context Passed unchanged to the callback
 */
void db_rtcm3_framer_on_idle(db_rtcm3_framer_t *framer, db_rtcm3_frame_cb_t callback, void *context) {
    if (framer == NULL || framer->length < 2U) {
        return;
    }
    for (uint16_t start = 1; start < framer->length; start++) {
        if (framer->buffer[start] == DB_RTCM3_PREAMBLE && db_rtcm3_framer_complete_frame_at(framer, start)) {
            framer->skipped_bytes += start;
            db_rtcm3_framer_drop(framer, start);
            db_rtcm3_framer_evaluate(framer, callback, context);
            return;
        }
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
