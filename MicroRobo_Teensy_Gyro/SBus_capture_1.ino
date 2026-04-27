// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// SBus_capture_1.ino — Non-blocking S.Bus frame parser
// =============================================================================
//
// Parses the Futaba S.Bus protocol on Serial4 (Teensy 4.0, pins 16 RX / 17 TX).
//
// S.Bus protocol (Futaba / FrSky):
//   Baud:       100 000, 8E2, signal inverted (SERIAL_8E2_RXINV)
//   Frame:      25 bytes at ~14 ms (high-speed mode) or ~7 ms (fast mode)
//   Byte 0:     Start byte 0x0F
//   Bytes 1–22: 16 channels × 11 bits, bit-packed LSB-first
//   Byte 23:    Digital channels ch17 (bit 0) and ch18 (bit 1)
//   Byte 24:    End byte 0x00 (flags byte on some variants; treated as end byte here)
//
// The parser is non-blocking.  capture_RX_signals() must be called on every
// loop iteration; it drains the serial buffer one state at a time and returns
// true only when a complete, validated 25-byte frame has been decoded.
//
// INTER-FRAME GAP DETECTION:
//   S.Bus frames are separated by at least 3.5 ms of silence.  The parser
//   waits for GAP_THRESHOLD_US of silence before accepting a start byte,
//   which prevents false-starts mid-stream.
// =============================================================================

#define START_BYTE       0x0F
#define END_BYTE         0x00
#define NUM_BYTES        25
#define GAP_THRESHOLD_US 3000UL  // 3 ms — safely within the 3.5 ms inter-frame gap

// Raw S.Bus byte buffer and decoded digital channels
byte  receivedData[NUM_BYTES];
int   ch17;   // S.Bus digital channel 17 (byte 23, bit 0)
int   ch18;   // S.Bus digital channel 18 (byte 23, bit 1)

// Parser state machine states
#define SBUS_WAIT_GAP   0
#define SBUS_WAIT_START 1
#define SBUS_COLLECTING 2

// ---------------------------------------------------------------------------
// radio_init()  —  call once from setup() before the main loop
// ---------------------------------------------------------------------------
void radio_init() {
    Serial4.setTX(17);
    Serial4.setRX(16);
    Serial4.begin(100000, SERIAL_8E2_RXINV);
    delay(500);
}

// ---------------------------------------------------------------------------
// capture_RX_signals()
//
// Non-blocking S.Bus parser.  Call every loop iteration.
//
// State machine:
//   SBUS_WAIT_GAP   — drain incoming bytes; detect 3 ms silence
//   SBUS_WAIT_START — silence confirmed; wait for start byte 0x0F
//   SBUS_COLLECTING — accumulate 25 bytes; validate end byte; decode
//
// Returns:
//   true  — a complete, valid 25-byte frame was decoded; channels[] updated
//   false — still waiting for a complete frame
// ---------------------------------------------------------------------------
bool capture_RX_signals() {
    static uint8_t       sbus_state   = SBUS_WAIT_GAP;
    static uint8_t       byte_idx     = 0;
    static unsigned long last_byte_us = 0;

    bool fresh_frame = false;

    switch (sbus_state) {

        // -----------------------------------------------------------------------
        // WAIT_GAP: Drain and discard bytes, recording the timestamp of the last
        // byte seen.  Once GAP_THRESHOLD_US of silence has elapsed, the
        // inter-frame gap has been detected and we are ready for the start byte.
        // -----------------------------------------------------------------------
        case SBUS_WAIT_GAP:
            while (Serial4.available()) {
                Serial4.read();
                last_byte_us = micros();
            }
            if (micros() - last_byte_us > GAP_THRESHOLD_US) {
                sbus_state = SBUS_WAIT_START;
            }
            break;

        // -----------------------------------------------------------------------
        // WAIT_START: Gap confirmed.  The next byte must be 0x0F.
        // Anything else means we caught a stray byte after the gap timer fired
        // early — restart gap detection to re-synchronise.
        // -----------------------------------------------------------------------
        case SBUS_WAIT_START:
            if (Serial4.available()) {
                byte b = Serial4.read();
                if (b == START_BYTE) {
                    receivedData[0] = b;
                    byte_idx   = 1;
                    sbus_state = SBUS_COLLECTING;
                } else {
                    last_byte_us = micros();
                    sbus_state   = SBUS_WAIT_GAP;
                }
            }
            break;

        // -----------------------------------------------------------------------
        // COLLECTING: Accumulate bytes until all 25 are received.
        // Validate byte 24 as the end byte before decoding.
        // A corrupt frame is silently discarded; recovery happens on the next gap.
        // -----------------------------------------------------------------------
        case SBUS_COLLECTING:
            while (Serial4.available() && byte_idx < NUM_BYTES) {
                receivedData[byte_idx++] = Serial4.read();
            }
            if (byte_idx == NUM_BYTES) {
                if (receivedData[24] == END_BYTE) {
                    // Decode 16 channels from 22 bytes of bit-packed 11-bit values
                    channels[0]  = (uint16_t)((receivedData[1]      | receivedData[2]  << 8)                          & 0x07FF);
                    channels[1]  = (uint16_t)((receivedData[2]  >> 3 | receivedData[3]  << 5)                          & 0x07FF);
                    channels[2]  = (uint16_t)((receivedData[3]  >> 6 | receivedData[4]  << 2 | receivedData[5]  << 10) & 0x07FF);
                    channels[3]  = (uint16_t)((receivedData[5]  >> 1 | receivedData[6]  << 7)                          & 0x07FF);
                    channels[4]  = (uint16_t)((receivedData[6]  >> 4 | receivedData[7]  << 4)                          & 0x07FF);
                    channels[5]  = (uint16_t)((receivedData[7]  >> 7 | receivedData[8]  << 1 | receivedData[9]  << 9)  & 0x07FF);
                    channels[6]  = (uint16_t)((receivedData[9]  >> 2 | receivedData[10] << 6)                          & 0x07FF);
                    channels[7]  = (uint16_t)((receivedData[10] >> 5 | receivedData[11] << 3)                          & 0x07FF);
                    channels[8]  = (uint16_t)((receivedData[12]      | receivedData[13] << 8)                          & 0x07FF);
                    channels[9]  = (uint16_t)((receivedData[13] >> 3 | receivedData[14] << 5)                          & 0x07FF);
                    channels[10] = (uint16_t)((receivedData[14] >> 6 | receivedData[15] << 2 | receivedData[16] << 10) & 0x07FF);
                    channels[11] = (uint16_t)((receivedData[16] >> 1 | receivedData[17] << 7)                          & 0x07FF);
                    channels[12] = (uint16_t)((receivedData[17] >> 4 | receivedData[18] << 4)                          & 0x07FF);
                    channels[13] = (uint16_t)((receivedData[18] >> 7 | receivedData[19] << 1 | receivedData[20] << 9)  & 0x07FF);
                    channels[14] = (uint16_t)((receivedData[20] >> 2 | receivedData[21] << 6)                          & 0x07FF);
                    channels[15] = (uint16_t)((receivedData[21] >> 5 | receivedData[22] << 3)                          & 0x07FF);

                    // Digital channels ch17 and ch18 are in byte 23, bits 0 and 1
                    ch17 = (receivedData[23] >> 0) & 0x01;
                    ch18 = (receivedData[23] >> 1) & 0x01;

                    fresh_frame = true;
                }
                // Valid or corrupt — return to gap detection for next frame
                last_byte_us = micros();
                byte_idx     = 0;
                sbus_state   = SBUS_WAIT_GAP;
            }
            break;
    }

    return fresh_frame;
}
