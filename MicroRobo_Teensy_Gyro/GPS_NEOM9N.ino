// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// GPS_NEOM9N.ino — u-blox NEO-M9N UBX binary parser
// =============================================================================
//
// Target hardware : u-blox NEO-M9N (M9-generation, firmware SPG 4.04)
//                   Teensy 4.0 on Serial5  (TX=pin 20, RX=pin 21)
//
// Protocol:
//   UBX binary, 38 400 baud, 10 Hz NAV-PVT output.
//   NAV-PVT payload is 92 bytes on the M9N (raised from 84 on earlier M8 silicon).
//   Legacy CFG-MSG / CFG-RATE / CFG-PRT / CFG-CFG are used for configuration;
//   these remain fully supported on SPG 4.04 (UBX-19035940 §3.10).
//
// Multi-constellation:
//   CFG-GNSS enables GPS + GLONASS + Galileo + BeiDou on initialisation.
//   Configuration is saved to flash so it survives power cycles.
//
// Fix validity:
//   Always check BOTH fixType >= 3 AND gnssFixOK (NAV-PVT flags byte bit 0).
//   gnssFixOK reflects whether internal accuracy filters are satisfied;
//   fixType alone can return 3 when gnssFixOK is 0 under poor conditions.
//
// Velocity sources (priority order):
//   NAV-VELNED (preferred, cm/s native, converted to mm/s)
//   NAV-PVT    (fallback, mm/s native, used until VELNED is active)
//
// KNOWN LIMITATION — GPS iTOW not in telemetry frame:
//   iTOW (GPS time-of-week, ms) is parsed from NAV-PVT and stored in
//   gpsData.iTOW, but is NOT transmitted in the 34-byte binary telemetry frame.
//   The ground station therefore cannot detect stale GPS measurements from
//   delayed or retransmitted frames.  Adding iTOW as a telemetry field is the
//   highest-priority frame format revision item.
//   See: https://github.com/lcumbridge/mrs-airborne-tracker/issues/1
//
// CFG-PRT flash-save ordering (root cause note):
//   CFG-PRT (UBX-only output) must be saved to flash immediately after sending,
//   before CFG-GNSS.  CFG-GNSS causes an internal tracking channel
//   reconfiguration that reloads port config from flash; if CFG-PRT was in RAM
//   only, the GPS reverts to NMEA output.  configureUBX() implements the correct
//   sequence: CFG-PRT → CFG-CFG (save) → CFG-GNSS → … → CFG-CFG (save).
//
// DEBUG DIAGNOSTICS:
//   Define GPS_DEBUG_DIAG (below) to compile in byte/message/checksum counters
//   and the gps_print_diag() summary function.  Leave undefined in production.
//   DEBUG_GPS (also below) enables per-message Serial prints inside the parser.
// =============================================================================

// ---------------------------------------------------------------------------
// Compile-time debug switches
// ---------------------------------------------------------------------------
// #define GPS_DEBUG_DIAG   // compile in pipeline counters and gps_print_diag()
#define DEBUG_GPS  false    // per-message verbose prints inside parser (bool, not define)

// ---------------------------------------------------------------------------
// Watchdog: re-send configuration if no valid NAV-PVT for this long
// ---------------------------------------------------------------------------
static uint32_t lastValidPVT  = 0;
static bool     gpsConfigured = false;
#define GPS_WATCHDOG_MS 15000

// ---------------------------------------------------------------------------
// UBX protocol constants
// ---------------------------------------------------------------------------
#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_CLASS_NAV   0x01
#define UBX_CLASS_CFG   0x06
#define UBX_NAV_PVT     0x07   // Position, Velocity, Time (92 bytes on M9N)
#define UBX_NAV_POSLLH  0x02   // Position (Lat/Lon/Height), 28 bytes
#define UBX_NAV_VELNED  0x12   // Velocity NED, 36 bytes

// ---------------------------------------------------------------------------
// GPS serial port
// ---------------------------------------------------------------------------
#define GPS_SERIAL  Serial5    // Teensy 4.0: TX=pin 20, RX=pin 21
#define GPS_BAUD    38400

// ---------------------------------------------------------------------------
// UBX parser circular buffer
// ---------------------------------------------------------------------------
#define GPS_BUFFER_SIZE 1024
volatile uint8_t  gpsBuffer[GPS_BUFFER_SIZE];
volatile uint16_t gpsBufferHead = 0;
volatile uint16_t gpsBufferTail = 0;

// ---------------------------------------------------------------------------
// UBX parser state machine
// ---------------------------------------------------------------------------
enum UBXParseState {
    WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID,
    GET_LENGTH1, GET_LENGTH2, GET_PAYLOAD,
    GET_CHECKSUM1, GET_CHECKSUM2
};

struct UBXMessage {
    uint8_t  msgClass;
    uint8_t  msgID;
    uint16_t length;
    uint8_t  payload[256];  // sufficient for all NAV messages (NAV-PVT = 92 bytes)
    uint8_t  ckA;
    uint8_t  ckB;
    bool     ready;
};

UBXMessage    ubxMsg;
UBXParseState parseState   = WAIT_SYNC1;
uint16_t      payloadIndex = 0;

// Velocity source flag:
//   0 = no velocity yet
//   1 = velocity from NAV-PVT (fallback, mm/s native)
//   2 = velocity from NAV-VELNED (preferred, cm/s converted to mm/s)
volatile uint8_t velN_source = 0;

unsigned long GPS_Loop_time;

// ---------------------------------------------------------------------------
// GPS_DEBUG_DIAG — pipeline diagnostic counters
//
// Useful for tracing parser failures during bringup.  Gate behind
// GPS_DEBUG_DIAG to keep the production binary free of these counters.
//
// To enable:  #define GPS_DEBUG_DIAG at the top of this file.
// To use:     call gps_print_diag() from the main loop (it self-throttles
//             to GPS_DIAG_INTERVAL_MS).
// ---------------------------------------------------------------------------
#ifdef GPS_DEBUG_DIAG

#define GPS_DIAG_INTERVAL_MS 5000

static uint32_t gps_diag_bytes_rx      = 0;
static uint32_t gps_diag_sync1_seen    = 0;
static uint32_t gps_diag_msg_ok        = 0;
static uint32_t gps_diag_ck_fail       = 0;
static uint32_t gps_diag_pvt_calls     = 0;
static uint32_t gps_diag_watchdog_fire = 0;
static uint32_t gps_diag_last_print    = 0;

void gps_print_diag() {
    uint32_t now_ms = millis();
    if (now_ms - gps_diag_last_print < GPS_DIAG_INTERVAL_MS) return;
    gps_diag_last_print = now_ms;

    uint16_t buf_used = (gpsBufferHead >= gpsBufferTail)
                      ? (gpsBufferHead - gpsBufferTail)
                      : (GPS_BUFFER_SIZE - gpsBufferTail + gpsBufferHead);

    Serial.println(F("--- GPS DIAG ---"));
    Serial.print(F("  T+"));           Serial.print(now_ms / 1000);    Serial.println(F("s"));
    Serial.print(F("  bytes_rx:   ")); Serial.println(gps_diag_bytes_rx);
    Serial.print(F("  0xB5_seen:  ")); Serial.println(gps_diag_sync1_seen);
    Serial.print(F("  msg_ok:     ")); Serial.println(gps_diag_msg_ok);
    Serial.print(F("  ck_fail:    ")); Serial.println(gps_diag_ck_fail);
    Serial.print(F("  pvt_calls:  ")); Serial.println(gps_diag_pvt_calls);
    Serial.print(F("  wdog_fires: ")); Serial.println(gps_diag_watchdog_fire);
    Serial.print(F("  buf_used:   ")); Serial.print(buf_used); Serial.println(F(" bytes"));
    Serial.print(F("  fixType:    ")); Serial.print(gpsData.fixType);
    Serial.print(F("  gnssFixOK:  ")); Serial.print(gpsData.gnssFixOK);
    Serial.print(F("  numSV:      ")); Serial.println(gpsData.numSV);
    Serial.print(F("  lat:        ")); Serial.println(gpsData.lat);
    Serial.print(F("  lon:        ")); Serial.println(gpsData.lon);
    Serial.print(F("  lastPVT_age:")); Serial.print((now_ms - lastValidPVT) / 1000);
    Serial.println(F("s ago"));
    Serial.println(F("----------------"));
}

// Convenience macros — compile to nothing when GPS_DEBUG_DIAG is not defined
#define DIAG_BYTES_RX(n)   gps_diag_bytes_rx      += (n)
#define DIAG_SYNC1()       gps_diag_sync1_seen++
#define DIAG_MSG_OK()      gps_diag_msg_ok++
#define DIAG_CK_FAIL()     gps_diag_ck_fail++
#define DIAG_PVT_CALL()    gps_diag_pvt_calls++
#define DIAG_WATCHDOG()    gps_diag_watchdog_fire++

#else  // GPS_DEBUG_DIAG not defined — all counters compile away

#define DIAG_BYTES_RX(n)   ((void)0)
#define DIAG_SYNC1()       ((void)0)
#define DIAG_MSG_OK()      ((void)0)
#define DIAG_CK_FAIL()     ((void)0)
#define DIAG_PVT_CALL()    ((void)0)
#define DIAG_WATCHDOG()    ((void)0)

#endif  // GPS_DEBUG_DIAG

// =============================================================================
// GPS_init()  —  call once from setup()
// =============================================================================
// Initialisation strategy:
//   1. Open GPS_SERIAL at GPS_BAUD and scan for UBX 0xB5 sync bytes (2 s).
//      If found → already configured; call enableNavMessages() and return.
//   2. If not found → scan 9600 / 115200 baud.  If found there → fall through
//      to configureUBX() which switches and saves the baud rate.
//   3. If not found at any baud → log a warning, default to GPS_BAUD, proceed
//      with configureUBX() (handles cold power-up from factory defaults).
//   4. After either path, verify UBX bytes are flowing.  If not, configureUBX()
//      is run one additional time as a recovery measure.
// =============================================================================
void GPS_init() {
    Serial.println(F("GPS_init: starting (NEO-M9N)..."));

    GPS_SERIAL.begin(GPS_BAUD);
    delay(300);
    while (GPS_SERIAL.available()) GPS_SERIAL.read();
    delay(100);

    Serial.print(F("GPS_init: scanning for UBX at ")); Serial.print(GPS_BAUD);
    Serial.print(F(" baud... "));

    uint32_t t0 = millis();
    uint32_t syncCount = 0;
    bool foundAtExpected = false;
    while (millis() - t0 < 2000) {
        if (GPS_SERIAL.available() && GPS_SERIAL.read() == 0xB5) {
            if (++syncCount >= 3) { foundAtExpected = true; break; }
        }
    }

    if (foundAtExpected) {
        Serial.print(F("FOUND — re-enabling NAV messages"));
        while (GPS_SERIAL.available()) GPS_SERIAL.read();
        gpsBufferHead = 0; gpsBufferTail = 0;
        enableNavMessages();
    } else {
        Serial.println(F("not found"));
        GPS_SERIAL.end(); delay(100);

        uint32_t baudRates[] = {9600, 38400, 115200};
        bool gpsFound = false;
        for (int i = 0; i < 3; i++) {
            if (baudRates[i] == GPS_BAUD) continue;
            GPS_SERIAL.begin(baudRates[i]); delay(200);
            while (GPS_SERIAL.available()) GPS_SERIAL.read(); delay(100);
            Serial.print(F("  trying ")); Serial.print(baudRates[i]); Serial.print(F("... "));
            uint32_t t1 = millis();
            uint32_t sc = 0;
            while (millis() - t1 < 1500) {
                if (GPS_SERIAL.available() && GPS_SERIAL.read() == 0xB5) {
                    if (++sc >= 2) { gpsFound = true; break; }
                }
            }
            if (gpsFound) { Serial.print(F("FOUND at ")); Serial.println(baudRates[i]); break; }
            Serial.println(F("not found"));
            GPS_SERIAL.end(); delay(100);
        }

        if (!gpsFound) {
            Serial.println(F("WARNING: no GPS UBX sync at any baud"));
            Serial.println(F("  NEO-M9N TX → Teensy Serial5 RX (pin 21)"));
            Serial.println(F("  NEO-M9N RX ← Teensy Serial5 TX (pin 20)"));
            GPS_SERIAL.begin(GPS_BAUD); delay(200);
        }

        Serial.println(F("GPS_init: running configureUBX()..."));
        while (GPS_SERIAL.available()) GPS_SERIAL.read();
        gpsBufferHead = 0; gpsBufferTail = 0;
        configureUBX();
    }

    // Verify UBX is flowing after either init path
    Serial.print(F("GPS_init: verifying UBX output... "));
    delay(500);
    while (GPS_SERIAL.available()) GPS_SERIAL.read();
    delay(200);

    uint32_t verifyStart = millis();
    uint32_t ubxCount = 0, totalBytes = 0;
    while (millis() - verifyStart < 2000) {
        if (GPS_SERIAL.available()) {
            uint8_t b = GPS_SERIAL.read();
            totalBytes++;
            if (b == 0xB5) ubxCount++;
        }
    }
    Serial.print(ubxCount); Serial.print(F(" UBX sync in "));
    Serial.print(totalBytes); Serial.println(F(" bytes"));

    if (ubxCount == 0 && totalBytes > 0) {
        // Bytes arriving but all NMEA — flash save of CFG-PRT may have been lost
        Serial.println(F("GPS_init: no UBX detected; re-running configureUBX()..."));
        while (GPS_SERIAL.available()) GPS_SERIAL.read();
        gpsBufferHead = 0; gpsBufferTail = 0;
        configureUBX();

        delay(500);
        while (GPS_SERIAL.available()) GPS_SERIAL.read();
        delay(200);
        verifyStart = millis(); ubxCount = 0; totalBytes = 0;
        while (millis() - verifyStart < 2000) {
            if (GPS_SERIAL.available()) {
                uint8_t b = GPS_SERIAL.read();
                totalBytes++;
                if (b == 0xB5) ubxCount++;
            }
        }
        Serial.print(F("GPS_init: second verify: ")); Serial.print(ubxCount);
        Serial.print(F(" UBX sync in ")); Serial.print(totalBytes); Serial.println(F(" bytes"));
    }

    while (GPS_SERIAL.available()) GPS_SERIAL.read();
    gpsBufferHead = 0; gpsBufferTail = 0;

    lastValidPVT  = millis();
    gpsConfigured = true;

#ifdef GPS_DEBUG_DIAG
    gps_diag_bytes_rx = gps_diag_sync1_seen = gps_diag_msg_ok  = 0;
    gps_diag_ck_fail  = gps_diag_pvt_calls  = gps_diag_watchdog_fire = 0;
#endif

    Serial.println(F("GPS_init: NEO-M9N init complete."));
    Serial.println(F("Waiting for fix (30-60 s outdoors, longer indoors)..."));
}

// =============================================================================
// enableNavMessages()  —  re-enable NAV output messages without reconfiguring
// =============================================================================
// Safe to call during satellite acquisition.  Does NOT touch CFG-GNSS,
// CFG-RATE, CFG-PRT, or CFG-CFG.
// =============================================================================
void enableNavMessages() {
    struct { uint8_t cls; uint8_t id; } navOn[] = {
        {0x01, 0x07},  // NAV-PVT
        {0x01, 0x12},  // NAV-VELNED
        {0x01, 0x02},  // NAV-POSLLH
        {0x01, 0x03},  // NAV-STATUS
    };
    for (auto& m : navOn) {
        uint8_t msg[] = {
            0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
            m.cls, m.id,
            0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        };
        calculateUBXChecksum(&msg[2], 12);
        sendUBXCommand(msg, sizeof(msg), 60);
    }
}

// =============================================================================
// read_gps()  —  call from main loop every iteration (non-blocking)
// =============================================================================
void read_gps() {
    processGPSData();

    // Watchdog: re-send configuration if no valid NAV-PVT for GPS_WATCHDOG_MS
    if (millis() - lastValidPVT > GPS_WATCHDOG_MS) {
        DIAG_WATCHDOG();
        Serial.print(F("GPS watchdog fired at T+"));
        Serial.print(millis() / 1000); Serial.println(F("s — re-sending config"));
        if (gpsConfigured) gpsConfigured = false;
        while (GPS_SERIAL.available()) GPS_SERIAL.read();
        gpsBufferHead = 0; gpsBufferTail = 0;
        configureUBX();
        lastValidPVT  = millis();
        gpsConfigured = true;
    }

#ifdef GPS_DEBUG_DIAG
    gps_print_diag();
#endif
}

// =============================================================================
// serialEvent5()  —  intentionally empty
// =============================================================================
// Teensy's serialEvent5() fires only inside yield() (called by delay() or
// blocking Serial writes).  GPS bytes are drained directly in processGPSData()
// on every read_gps() call, bypassing the yield/serialEvent dependency.
// =============================================================================
void serialEvent5() {
    // Intentionally empty — see processGPSData()
}

// =============================================================================
// processGPSData()  —  drain hardware UART into UBX parser
// =============================================================================
void processGPSData() {
    while (GPS_SERIAL.available()) {
        uint8_t c = GPS_SERIAL.read();
        DIAG_BYTES_RX(1);
        if (parseUBXByte(c)) {
            handleUBXMessage();
        }
    }
}

// =============================================================================
// parseUBXByte()  —  UBX framing state machine
// Returns true when a complete, checksum-verified message is ready in ubxMsg.
// =============================================================================
bool parseUBXByte(uint8_t c) {
    static uint32_t msgCount = 0;

    switch (parseState) {

        case WAIT_SYNC1:
            if (c == UBX_SYNC1) {
                parseState = WAIT_SYNC2;
                DIAG_SYNC1();
            }
            break;

        case WAIT_SYNC2:
            parseState = (c == UBX_SYNC2) ? GET_CLASS : WAIT_SYNC1;
            if (parseState == GET_CLASS) { ubxMsg.ckA = 0; ubxMsg.ckB = 0; }
            break;

        case GET_CLASS:
            ubxMsg.msgClass = c;
            ubxMsg.ckA += c; ubxMsg.ckB += ubxMsg.ckA;
            parseState = GET_ID;
            break;

        case GET_ID:
            ubxMsg.msgID = c;
            ubxMsg.ckA += c; ubxMsg.ckB += ubxMsg.ckA;
            parseState = GET_LENGTH1;
            break;

        case GET_LENGTH1:
            ubxMsg.length = c;
            ubxMsg.ckA += c; ubxMsg.ckB += ubxMsg.ckA;
            parseState = GET_LENGTH2;
            break;

        case GET_LENGTH2:
            ubxMsg.length |= ((uint16_t)c << 8);
            ubxMsg.ckA += c; ubxMsg.ckB += ubxMsg.ckA;
            payloadIndex = 0;
            parseState   = (ubxMsg.length > 0) ? GET_PAYLOAD : GET_CHECKSUM1;
            break;

        case GET_PAYLOAD:
            if (payloadIndex < sizeof(ubxMsg.payload)) {
                ubxMsg.payload[payloadIndex++] = c;
                ubxMsg.ckA += c; ubxMsg.ckB += ubxMsg.ckA;
            }
            if (payloadIndex >= ubxMsg.length) parseState = GET_CHECKSUM1;
            break;

        case GET_CHECKSUM1:
            if (c == ubxMsg.ckA) {
                parseState = GET_CHECKSUM2;
            } else {
                DIAG_CK_FAIL();
                if (DEBUG_GPS) Serial.println(F("UBX: checksum A failed"));
                parseState = WAIT_SYNC1;
            }
            break;

        case GET_CHECKSUM2:
            parseState = WAIT_SYNC1;
            if (c == ubxMsg.ckB) {
                ubxMsg.ready = true;
                msgCount++;
                DIAG_MSG_OK();
                if (DEBUG_GPS) {
                    Serial.print(F("UBX msg #")); Serial.print(msgCount);
                    Serial.print(F(" class=0x")); Serial.print(ubxMsg.msgClass, HEX);
                    Serial.print(F(" id=0x"));    Serial.print(ubxMsg.msgID, HEX);
                    Serial.print(F(" len="));     Serial.println(ubxMsg.length);
                }
                return true;
            } else {
                DIAG_CK_FAIL();
                if (DEBUG_GPS) Serial.println(F("UBX: checksum B failed"));
            }
            break;
    }
    return false;
}

// =============================================================================
// handleUBXMessage()  —  dispatch decoded messages to parsers
// =============================================================================
void handleUBXMessage() {
    if (!ubxMsg.ready) return;
    if (ubxMsg.msgClass == UBX_CLASS_NAV) {
        switch (ubxMsg.msgID) {
            case UBX_NAV_PVT:    parseNAV_PVT();    break;
            case UBX_NAV_POSLLH: parseNAV_POSLLH(); break;
            case UBX_NAV_VELNED: parseNAV_VELNED(); break;
        }
    }
    ubxMsg.ready = false;
}

// =============================================================================
// parseNAV_PVT()  —  Position, Velocity, Time  (92 bytes on NEO-M9N)
// =============================================================================
// NAV-PVT payload offsets (UBX-19035940 §3.15.10):
//   0-3   iTOW    uint32  GPS time of week (ms)
//   4-9   year/month/day/hour/min/sec
//   11    valid   uint8   bit0=validDate, bit1=validTime
//   20    fixType uint8   0=no fix  2=2D  3=3D
//   21    flags   uint8   bit0=gnssFixOK  ← reliability check beyond fixType
//   23    numSV   uint8
//   24-27 lon     int32   1e-7 deg
//   28-31 lat     int32   1e-7 deg
//   36-39 hMSL   int32   mm above MSL
//   40-43 hAcc   uint32  mm
//   44-47 vAcc   uint32  mm
//   48-55 velN/E  int32   mm/s
//   56-59 velD    int32   mm/s (positive = descending)
//   60-63 gSpeed  int32   mm/s
// =============================================================================
void parseNAV_PVT() {
    DIAG_PVT_CALL();
    if (ubxMsg.length < 92) {
        // M9N fixed payload is 92 bytes; shorter implies wrong receiver or firmware
        Serial.print(F("NAV-PVT: unexpected length ")); Serial.println(ubxMsg.length);
        return;
    }

    uint8_t* p = ubxMsg.payload;

    memcpy(&gpsData.iTOW, &p[0], 4);  // ms — stored but NOT in telemetry frame
                                        // See Known Limitations in README / issue #1
    gpsData.year  = p[4] | ((uint16_t)p[5] << 8);
    gpsData.month = p[6];
    gpsData.day   = p[7];
    gpsData.hour  = p[8];
    gpsData.min   = p[9];
    gpsData.sec   = p[10];
    uint8_t valid     = p[11];
    gpsData.validDate = (valid & 0x01);
    gpsData.validTime = (valid & 0x02);

    gpsData.fixType   = p[20];
    gpsData.gnssFixOK = (p[21] & 0x01);  // u-blox recommended validity flag
    gpsData.numSV     = p[23];

    memcpy(&gpsData.lon,    &p[24], 4);
    memcpy(&gpsData.lat,    &p[28], 4);
    memcpy(&gpsData.height, &p[32], 4);
    memcpy(&gpsData.hMSL,   &p[36], 4);
    memcpy(&gpsData.hAcc,   &p[40], 4);
    memcpy(&gpsData.vAcc,   &p[44], 4);

    // NAV-PVT velocity is fallback only — skip if NAV-VELNED is active
    if (velN_source < 2) {
        memcpy(&gpsData.velN, &p[48], 4);
        memcpy(&gpsData.velE, &p[52], 4);
        memcpy(&gpsData.velD, &p[56], 4);
        if (velN_source == 0) velN_source = 1;
    }
    memcpy(&gpsData.gSpeed, &p[60], 4);

    if (DEBUG_GPS) {
        Serial.print(F("NAV-PVT: fix=")); Serial.print(gpsData.fixType);
        Serial.print(F(" ok="));          Serial.print(gpsData.gnssFixOK);
        Serial.print(F(" sv="));          Serial.print(gpsData.numSV);
        Serial.print(F(" velSrc="));      Serial.println(velN_source);
    }

    smoothGPSData();
    lastValidPVT = millis();
    gpsConfigured = true;
}

// =============================================================================
// parseNAV_VELNED()  —  Velocity NED  (36 bytes, preferred velocity source)
// =============================================================================
// Offsets (UBX-19035940 §3.15.17):
//   4-7   velN    int32  cm/s  North
//   8-11  velE    int32  cm/s  East
//   12-15 velD    int32  cm/s  Down (positive = descending)
//   16-19 speed   uint32 cm/s  3-D speed
// Conversion: multiply by 10 to produce mm/s matching NAV-PVT units.
// =============================================================================
void parseNAV_VELNED() {
    if (ubxMsg.length < 36) return;
    uint8_t* p = ubxMsg.payload;

    int32_t  velN_raw, velE_raw, velD_raw;
    uint32_t gSpeed_raw;
    memcpy(&velN_raw,   &p[4],  4);
    memcpy(&velE_raw,   &p[8],  4);
    memcpy(&velD_raw,   &p[12], 4);
    memcpy(&gSpeed_raw, &p[16], 4);

    gpsData.velN   = velN_raw   * 10;  // cm/s → mm/s
    gpsData.velE   = velE_raw   * 10;
    gpsData.velD   = velD_raw   * 10;
    gpsData.gSpeed = gSpeed_raw * 10;

    velN_source = 2;  // VELNED active — PVT velocity now skipped

    if (DEBUG_GPS) {
        static uint32_t lastVelDebug = 0;
        if (millis() - lastVelDebug > 2000) {
            lastVelDebug = millis();
            Serial.print(F("NAV-VELNED: vN=")); Serial.print(gpsData.velN / 1000.0f, 2);
            Serial.print(F(" vE="));            Serial.print(gpsData.velE / 1000.0f, 2);
            Serial.print(F(" vD="));            Serial.print(gpsData.velD / 1000.0f, 2);
            Serial.println(F(" m/s"));
        }
    }
}

// =============================================================================
// parseNAV_POSLLH()  —  Position only  (28 bytes)
// =============================================================================
void parseNAV_POSLLH() {
    if (ubxMsg.length < 28) return;
    uint8_t* p = ubxMsg.payload;
    memcpy(&gpsData.lon,    &p[4],  4);
    memcpy(&gpsData.lat,    &p[8],  4);
    memcpy(&gpsData.height, &p[12], 4);
    memcpy(&gpsData.hMSL,   &p[16], 4);
    memcpy(&gpsData.hAcc,   &p[20], 4);
    memcpy(&gpsData.vAcc,   &p[24], 4);
}

// =============================================================================
// smoothGPSData()  —  exponential moving average position filter
// =============================================================================
// Requires BOTH gnssFixOK AND fixType >= 2.  gnssFixOK reflects whether the
// NEO-M9N's internal accuracy filters are satisfied; fixType alone can be 3
// with gnssFixOK=0 under marginal conditions.
// GPS_ALPHA is defined in the main sketch (0.1 = maximum smoothing, 0.9 = raw).
// =============================================================================
void smoothGPSData() {
    if (gpsData.fixType < 2 || !gpsData.gnssFixOK) {
        gpsSmoothed.initialized = false;
        return;
    }
    if (!gpsSmoothed.initialized) {
        gpsSmoothed.lat         = gpsData.lat;
        gpsSmoothed.lon         = gpsData.lon;
        gpsSmoothed.hMSL        = gpsData.hMSL;
        gpsSmoothed.initialized = true;
        return;
    }

    const int64_t ALPHA_1000           = (int64_t)(GPS_ALPHA * 1000.0 + 0.5);
    const int64_t ONE_MINUS_ALPHA_1000 = 1000 - ALPHA_1000;

    gpsSmoothed.lat  = (int32_t)(((int64_t)gpsSmoothed.lat  * ONE_MINUS_ALPHA_1000
                                + (int64_t)gpsData.lat  * ALPHA_1000) / 1000);
    gpsSmoothed.lon  = (int32_t)(((int64_t)gpsSmoothed.lon  * ONE_MINUS_ALPHA_1000
                                + (int64_t)gpsData.lon  * ALPHA_1000) / 1000);
    gpsSmoothed.hMSL = (int32_t)(((int64_t)gpsSmoothed.hMSL * ONE_MINUS_ALPHA_1000
                                + (int64_t)gpsData.hMSL * ALPHA_1000) / 1000);
}

// =============================================================================
// configureUBX()  —  full NEO-M9N initialisation sequence
// =============================================================================
// CRITICAL ORDERING:
//   CFG-PRT must be saved to flash (CFG-CFG) immediately after sending.
//   CFG-GNSS triggers an internal tracking channel reconfiguration that reloads
//   port config from flash.  If CFG-PRT was RAM-only at that point the GPS
//   reverts to NMEA output.  The sequence below locks UBX-only in flash first.
//
//   1. CFG-PRT  — switch output to UBX-only (RAM)
//   2. CFG-CFG  — save to flash NOW
//   3. CFG-MSG  — disable NMEA messages (belt-and-suspenders)
//   4. CFG-RATE — 10 Hz navigation rate
//   5. CFG-GNSS — GPS + GLONASS + Galileo + BeiDou (may reload from flash)
//   6. CFG-MSG  — enable NAV-PVT, NAV-VELNED, NAV-POSLLH, NAV-STATUS
//   7. CFG-CFG  — save everything to flash
// =============================================================================
void configureUBX() {
    Serial.println(F("configureUBX: step 1 — CFG-PRT UBX-only output (RAM)"));
    uint8_t setCfgPrt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x01,               // portID = 1 (UART1)
        0x00,
        0x00, 0x00,         // txReady: disabled
        0xD0, 0x08, 0x00, 0x00,  // mode: 8N1
        0x00, 0x96, 0x00, 0x00,  // baudRate: 38400
        0x07, 0x00,         // inProtoMask:  UBX | NMEA | RTCM
        0x01, 0x00,         // outProtoMask: UBX ONLY
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00
    };
    calculateUBXChecksum(&setCfgPrt[2], 24);
    sendUBXCommand(setCfgPrt, sizeof(setCfgPrt), 300);

    Serial.println(F("configureUBX: step 2 — CFG-CFG save UBX-only to flash"));
    saveCfgToFlash();

    Serial.println(F("configureUBX: step 3 — disable NMEA messages"));
    struct { uint8_t cls; uint8_t id; } nmeaOff[] = {
        {0xF0, 0x00}, {0xF0, 0x01}, {0xF0, 0x02},
        {0xF0, 0x03}, {0xF0, 0x04}, {0xF0, 0x05},
    };
    for (auto& m : nmeaOff) {
        uint8_t msg[] = {
            0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
            m.cls, m.id,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        };
        calculateUBXChecksum(&msg[2], 12);
        sendUBXCommand(msg, sizeof(msg), 60);
    }

    Serial.println(F("configureUBX: step 4 — CFG-RATE 10 Hz"));
    uint8_t setCfgRate[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0x64, 0x00,   // measRate = 100 ms = 10 Hz
        0x01, 0x00,   // navRate  = 1
        0x00, 0x00,   // timeRef  = UTC
        0x00, 0x00
    };
    calculateUBXChecksum(&setCfgRate[2], 10);
    sendUBXCommand(setCfgRate, sizeof(setCfgRate), 200);

    Serial.println(F("configureUBX: step 5 — CFG-GNSS 4-constellation"));
    uint8_t cfgGNSS[] = {
        0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00,
        0x00, 0x00, 0xFF, 0x07,
        0x00, 0x08, 0x10, 0x00,  0x01, 0x00, 0x01, 0x01,  // GPS
        0x01, 0x01, 0x03, 0x00,  0x01, 0x00, 0x01, 0x00,  // SBAS
        0x02, 0x04, 0x08, 0x00,  0x01, 0x00, 0x01, 0x00,  // Galileo
        0x03, 0x02, 0x10, 0x00,  0x01, 0x00, 0x01, 0x00,  // BeiDou
        0x04, 0x00, 0x08, 0x00,  0x00, 0x00, 0x01, 0x00,  // IMES (disabled)
        0x05, 0x00, 0x03, 0x00,  0x01, 0x00, 0x01, 0x00,  // QZSS
        0x06, 0x08, 0x0E, 0x00,  0x01, 0x00, 0x01, 0x00,  // GLONASS
        0x00, 0x00
    };
    calculateUBXChecksum(&cfgGNSS[2], 64);
    sendUBXCommand(cfgGNSS, sizeof(cfgGNSS), 500);

    Serial.println(F("configureUBX: step 6 — enable NAV messages"));
    enableNavMessages();

    Serial.println(F("configureUBX: step 7 — CFG-CFG final save"));
    saveCfgToFlash();

    Serial.println(F("configureUBX: complete (38400 baud, 10 Hz, 4-constellation, UBX-only)"));
}

// =============================================================================
// saveCfgToFlash()  —  CFG-CFG: persist current RAM config to BBR + flash
// =============================================================================
void saveCfgToFlash() {
    uint8_t msg[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0x00, 0x00, 0x00, 0x00,    // clearMask: none
        0xFF, 0xFF, 0x00, 0x00,    // saveMask:  all sections
        0x00, 0x00, 0x00, 0x00,    // loadMask:  none
        0x17,                      // deviceMask: BBR | Flash | EEPROM | SPI Flash
        0x00, 0x00
    };
    calculateUBXChecksum(&msg[2], 17);
    sendUBXCommand(msg, sizeof(msg), 600);  // 600 ms — flash write can take ~500 ms
}

// =============================================================================
// sendUBXCommand()  —  write a UBX message and wait for GPS to process it
// =============================================================================
void sendUBXCommand(uint8_t* msg, uint16_t len, uint16_t wait_ms) {
    while (GPS_SERIAL.available()) GPS_SERIAL.read();  // flush before sending
    GPS_SERIAL.write(msg, len);
    GPS_SERIAL.flush();
    if (wait_ms > 0) delay(wait_ms);
    while (GPS_SERIAL.available()) GPS_SERIAL.read();  // discard ACK/NAK
}

// =============================================================================
// calculateUBXChecksum()  —  Fletcher-8; writes ckA/ckB into data[len..len+1]
// =============================================================================
void calculateUBXChecksum(uint8_t* data, uint16_t len) {
    uint8_t ckA = 0, ckB = 0;
    for (uint16_t i = 0; i < len; i++) {
        ckA += data[i];
        ckB += ckA;
    }
    data[len]     = ckA;
    data[len + 1] = ckB;
}

// =============================================================================
// Velocity and accuracy helper functions
//
// GPS driver stores all velocities as mm/s (int32_t) and accuracies as mm
// (uint32_t) in the gpsData struct, matching NAV-PVT native units.
// These helpers convert to m/s and m for use by the EKF and telemetry.
// =============================================================================
float getVelN_ms()  { return gpsData.velN   / 1000.0f; }  // North velocity (m/s)
float getVelE_ms()  { return gpsData.velE   / 1000.0f; }  // East  velocity (m/s)
float getVelD_ms()  { return gpsData.velD   / 1000.0f; }  // Down  velocity (m/s)
float getHAcc_m()   { return gpsData.hAcc   / 1000.0f; }  // Horizontal accuracy (m)
float getVAcc_m()   { return gpsData.vAcc   / 1000.0f; }  // Vertical   accuracy (m)
float getSpeed_ms() { return gpsData.gSpeed / 1000.0f; }  // Ground speed (m/s)
