// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// BinaryTelemetry.ino — Compact 34-byte binary telemetry transmitter
// =============================================================================
//
// Replaces the multi-line ASCII data_radio_output() with a fixed-width binary
// frame that the Python ground-station parser can decode with a single struct
// unpack.  The frame is small enough to fit comfortably in one XBee 802.15.4
// payload (100-byte maximum).
//
// TARGET:   Teensy 4.0 / Teensyduino / C++17
// OUTPUT:   Data_Radio (Serial3) at 230 400 baud — configured in main sketch
//
// FRAME LAYOUT (36 bytes total):
//   Bytes  0– 1  : Sync pattern   0xAA 0x55  (not covered by CRC)
//   Bytes  2–33  : AirTelemetry_t packed struct  (32 bytes payload)
//   Bytes 30–31  : CRC-16/CCITT-FALSE over struct bytes [0..29]
//                  (i.e. the crc16 field covers the preceding 30 bytes)
//
// ATTITUDE ENCODING:
//   Quaternion X/Y/Z components are transmitted as int16, scaled by 32767.
//   The W component is omitted; the ground station reconstructs it as
//       qw = sqrt(max(0, 1 - qx² - qy² - qz²))
//   This is unambiguous because the firmware always enforces qw >= 0.
//   Resolution: ~0.00003 per LSB (~0.002°).
//
// GPS FRESHNESS FLAG:
//   gps_fresh=1  — this frame coincides with a new GPS PVT fix; the ground
//                  station EKF should apply a position/velocity update.
//   gps_fresh=0  — attitude-only frame; GPS fields hold the last known values.
//
// CALL SITES (see MicroRobo_Teensy_Gyro.ino):
//   send_binary_telemetry(false)  in the new_IMU_aval block  (~112 Hz)
//   send_binary_telemetry(true)   in the (loop_count % 25) block (~4.5 Hz)
//
// COMPILE GUARD:
//   This entire file is compiled only when USE_BINARY_PROTOCOL is defined.
//   Define it in MicroRobo_Teensy_Gyro.ino before including this file.
// =============================================================================

#ifdef USE_BINARY_PROTOCOL

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Port selection.
// Data_Radio is Serial3 at 230 400 baud — configured in main sketch.
// To switch to USB-CDC for bench testing, change to "Serial" and raise
// Serial.begin() to 230400 in setup().
// ---------------------------------------------------------------------------
#define TELEM_PORT  Data_Radio

// ---------------------------------------------------------------------------
// Wire struct — must be exactly 32 bytes.
// #pragma pack(push,1) suppresses any compiler-inserted padding.
// The static_assert below catches accidental size drift at compile time.
//
// Field layout (byte offsets are within the struct, not within the full frame):
//   0  frame_id     uint16  rolling counter, wraps at 65535
//   2  timestamp_ms uint32  millis() at time of assembly
//   6  lat_deg7     int32   WGS-84 latitude  × 1e7  (same as u-blox NAV-PVT)
//  10  lon_deg7     int32   WGS-84 longitude × 1e7
//  14  alt_cm       int16   MSL altitude in cm  (range ±327 m; sufficient for AGL ops)
//  16  vel_n_cms    int16   NED North velocity, cm/s
//  18  vel_e_cms    int16   NED East  velocity, cm/s
//  20  vel_d_cms    int16   NED Down  velocity, cm/s
//  22  quat_x_sc    int16   DMP quaternion X × 32767
//  24  quat_y_sc    int16   DMP quaternion Y × 32767
//  26  quat_z_sc    int16   DMP quaternion Z × 32767
//  28  gps_fix      uint8   bits[7:4]=fix_type  bits[3:0]=HDOP×2 (0–15 clamped)
//  29  gps_fresh    uint8   1=new GPS fix in this frame, 0=attitude-only
//  30  crc16        uint16  CRC-16/CCITT-FALSE over struct bytes [0..29]
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
typedef struct {
    uint16_t frame_id;
    uint32_t timestamp_ms;
    int32_t  lat_deg7;
    int32_t  lon_deg7;
    int16_t  alt_cm;
    int16_t  vel_n_cms;
    int16_t  vel_e_cms;
    int16_t  vel_d_cms;
    int16_t  quat_x_sc;
    int16_t  quat_y_sc;
    int16_t  quat_z_sc;
    uint8_t  gps_fix;
    uint8_t  gps_fresh;
    uint16_t crc16;
} AirTelemetry_t;
#pragma pack(pop)

static_assert(sizeof(AirTelemetry_t) == 32,
    "AirTelemetry_t must be exactly 32 bytes — check #pragma pack");

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE
//   Polynomial : 0x1021
//   Init value : 0xFFFF
//   Input/output reflection: none
//   Test vector: crc16_ccitt_false("123456789", 9) == 0x29B1
// ---------------------------------------------------------------------------
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

// Rolling frame counter, file-scope
static uint16_t s_frame_id = 0;

// ---------------------------------------------------------------------------
// send_binary_telemetry()
//
// Assembles one AirTelemetry_t frame from the current firmware globals, appends
// CRC, and writes the 36-byte frame (2 sync + 32 struct bytes + 2 CRC) to
// TELEM_PORT.
//
// Do NOT call TELEM_PORT.flush() after this — it blocks until the TX FIFO
// drains and will stall the control loop by ~1.5 ms at 230 400 baud.
//
// Parameters:
//   gps_fresh  true  = this call coincides with a new GPS PVT fix
//              false = attitude-only frame (GPS fields hold last known values)
//
// Globals consumed (declared in main sketch and GPS module):
//   gpsData.lat / .lon / .hMSL / .velN / .velE / .velD / .fixType
//   getHAcc_m()          float  horizontal accuracy, metres
//   qx, qy, qz           float  DMP quaternion XYZ (axis-remapped, qw always >= 0)
// ---------------------------------------------------------------------------
void send_binary_telemetry(bool gps_fresh)
{
    AirTelemetry_t pkt;

    pkt.frame_id     = s_frame_id++;
    pkt.timestamp_ms = (uint32_t)millis();

    // GPS position — stored as 1e-7 ° int32, no conversion required
    pkt.lat_deg7 = gpsData.lat;
    pkt.lon_deg7 = gpsData.lon;

    // Altitude: hMSL is mm from GPS driver; wire field is cm
    {
        long alt_cm_raw = (long)(gpsData.hMSL / 10L);
        pkt.alt_cm = (int16_t)constrain(alt_cm_raw, -32768L, 32767L);
    }

    // Velocity NED: GPS driver stores mm/s; getVel*_ms() returns m/s; ×100 = cm/s
    pkt.vel_n_cms = (int16_t)constrain((long)(getVelN_ms() * 100.0f), -32768L, 32767L);
    pkt.vel_e_cms = (int16_t)constrain((long)(getVelE_ms() * 100.0f), -32768L, 32767L);
    pkt.vel_d_cms = (int16_t)constrain((long)(getVelD_ms() * 100.0f), -32768L, 32767L);

    // Quaternion XYZ, scaled to int16.  W is reconstructed on the ground station.
    // qx/qy/qz are axis-remapped globals from read_IMU(); qw is always >= 0.
    pkt.quat_x_sc = (int16_t)constrain((long)(qx * 32767.0f), -32767L, 32767L);
    pkt.quat_y_sc = (int16_t)constrain((long)(qy * 32767.0f), -32767L, 32767L);
    pkt.quat_z_sc = (int16_t)constrain((long)(qz * 32767.0f), -32767L, 32767L);

    // gps_fix byte:
    //   Upper nibble [7:4] = fix_type  (0=no fix, 3=3D, 4=RTK-fixed)
    //   Lower nibble [3:0] = HDOP×2 clamped to 0–15
    //   HDOP approximated as hAcc_m / 3.0 (hAcc ≈ HDOP × σ_UERE, σ_UERE ≈ 3 m)
    {
        float hdop_approx = getHAcc_m() / 3.0f;
        uint8_t hdop_enc  = (uint8_t)min((int)(hdop_approx * 2.0f), 15);
        pkt.gps_fix = ((gpsData.fixType & 0x0F) << 4) | (hdop_enc & 0x0F);
    }
    pkt.gps_fresh = gps_fresh ? 1 : 0;

    // CRC over the 30 payload bytes that precede the crc16 field
    pkt.crc16 = crc16_ccitt_false((const uint8_t *)&pkt,
                                   sizeof(pkt) - sizeof(pkt.crc16));

    // Transmit: 2 sync bytes + 32-byte struct = 34 bytes on the wire
    const uint8_t sync[2] = { 0xAA, 0x55 };
    TELEM_PORT.write(sync, 2);
    TELEM_PORT.write((const uint8_t *)&pkt, sizeof(pkt));
}

// ---------------------------------------------------------------------------
// binary_telemetry_setup()
//
// Call from setup() after serial ports are initialised.
// Validates the CRC-16 implementation against the standard test vector.
// Halts with a rapid LED flash if the algorithm is wrong — catches endianness
// or toolchain issues on the bench before deployment.
// ---------------------------------------------------------------------------
void binary_telemetry_setup(void)
{
    static const uint8_t test_vec[] = "123456789";
    uint16_t result = crc16_ccitt_false(test_vec, 9);
    if (result != 0x29B1) {
        Serial.print("FATAL: CRC16 self-test FAILED! Got 0x");
        Serial.print(result, HEX);
        Serial.println(" expected 0x29B1");
        while (true) {
            digitalWrite(13, HIGH); delay(100);
            digitalWrite(13, LOW);  delay(100);
        }
    }
    Serial.println("BinaryTelemetry: CRC16 self-test PASSED (0x29B1)");
}

#endif // USE_BINARY_PROTOCOL
