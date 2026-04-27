# mrs-airborne-tracker

Airborne firmware for the MicroRobo Systems GPS beacon tracking system. Runs on a
Teensy 4.0 and serves two simultaneous roles: it fuses GPS, IMU, barometric, and
magnetic sensor data into a 9-state EKF and transmits a compact binary telemetry
frame at up to 112 Hz over XBee 802.15.4 for real-time gimbal tracking; and it
acts as a 3-axis rate or heading-hold gyroscope stabiliser for fixed-wing aircraft,
reading pilot inputs from an S.Bus receiver and outputting stabilised commands to
up to six hobby servos.

> **Part of a three-component system.**
> See also:
> [`mrs-gimbal-controller`](https://github.com/lcumbridge/mrs-gimbal-controller) —
> dual-axis BLDC gimbal firmware |
> [`mrs-ground-station`](https://github.com/lcumbridge/mrs-ground-station) —
> Python ground station and EKF tracker

---

## Table of Contents

1. [System Context](#system-context)
2. [Hardware Requirements](#hardware-requirements)
3. [Wiring Summary](#wiring-summary)
4. [Dependencies](#dependencies)
5. [Building and Flashing](#building-and-flashing)
6. [Configuration](#configuration)
7. [Telemetry Frame Format](#telemetry-frame-format)
8. [Architecture and Key Design Decisions](#architecture-and-key-design-decisions)
9. [Gyroscope Stabilization](#gyroscope-stabilization)
10. [Known Limitations](#known-limitations)
11. [Performance Characterization](#performance-characterization)
12. [License](#license)
13. [Citation](#citation)

---

## System Context

The firmware operates simultaneously in two roles.

**GPS beacon / telemetry role:** The Teensy 4.0 reads quaternion orientation from an
ICM-20948 IMU in DMP mode via SPI1, GPS position and velocity from a u-blox NEO-M9N
at 10 Hz, barometric altitude from a BMP581 on Wire2, and magnetic heading from a
QMC5883L on Wire0. A 9-state EKF fuses GPS position/velocity with IMU-derived
acceleration in the NED (North-East-Down) frame. Every 100 ms a 34-byte binary
telemetry frame is assembled, CRC-protected, and transmitted over XBee 802.15.4 to
the ground station, which drives a two-axis BLDC gimbal to track the target optically.

**Flight stabilization role:** Simultaneously, the firmware reads pilot stick and
switch inputs from an S.Bus receiver, if connected, and runs a 3-axis PID controller that outputs
stabilised PWM commands to up to six hobby servos. Two modes are available:
rate mode (gyro-rate tracking) and heading-hold mode (attitude stabilisation),
selected by an S.Bus switch. This allows the same hardware to act as an onboard
gyroscope stabiliser for fixed-wing aircraft while transmitting tracking telemetry.

```
[ICM-20948 IMU] ──SPI1────┐
[BMP581 baro]   ──Wire2───┤                              ┌──► [XBee TX] ~~RF~~► GS
[QMC5883L mag]  ──Wire0───┼──► [Teensy 4.0 / EKF+PID] ─┤
[NEO-M9N GPS]   ──Serial5─┘                              └──► [Servo 1–6]
[S.Bus RX]      ──Serial4─┘
```

---

## Hardware Requirements

| Component | Part / Module | Qty | Notes |
|---|---|---|---|
| Flight computer | PJRC Teensy 4.0 | 1 | 600 MHz Cortex-M7 |
| IMU | TDK ICM-20948 | 1 | DMP mode, SPI1; RE1C002UNTCL level shifters on MOSI, MISO, SCK1, and CS1 |
| Barometer | Bosch BMP581 | 1 | I2C Wire2 (pins 25/24); LGA-10 package; see [Known Limitations](#known-limitations) |
| Magnetometer | QMC5883L | 1 | I2C Wire0 (pins 18/19) |
| GPS | u-blox NEO-M9N | 1 | UBX binary, 38 400 baud, 10 Hz, SPG 4.04 firmware |
| Radio | Digi XBee 802.15.4 (Series 1 or S2C in 802.15.4 mode) | 1 | 230 400 baud |
| Ground radio (paired) | Matching XBee module | 1 | At ground station |
| Regulated supply | 3.3 V LDO or SMPS | 1 | All peripherals are 3.3 V logic |
| Power bus | 5 V for Teensy Vin | 1 | Teensy 4.0 onboard 3.3 V reg feeds peripherals |

---

## Wiring Summary

> Full schematic available in [`/docs/Teensy_Fixedwing_Gyro.pdf`](docs/Teensy_Fixedwing_Gyro.pdf).

### SPI1 — ICM-20948

The ICM-20948 connects to Teensy 4.0 SPI1 with RE1C002UNTCL MOSFET level shifters
on the MOSI, MISO, SCK1, and CS1 lines for signal integrity.

| Signal | Teensy 4.0 Pin | Notes |
|---|---|---|
| MOSI | 26 | 2.2 kΩ pull-up to 3.3 V via RE1C002UNTCL level shifter |
| MISO | 1  | 2.2 kΩ pull-up to 3.3 V via RE1C002UNTCL level shifter |
| SCK  | 27 | 2.2 kΩ pull-up to 3.3 V via RE1C002UNTCL level shifter |
| CS   | 0  | 2.2 kΩ pull-up to 3.3 V via RE1C002UNTCL level shifter |

### I2C Bus 2 (Wire2) — BMP581

| Signal | Teensy 4.0 Pin | Notes |
|---|---|---|
| SDA | 25 | 2.2 kΩ pull-up to 3.3 V |
| SCL | 24 | 2.2 kΩ pull-up to 3.3 V |
| CSB | 3.3 V | Pulled high to force I2C mode (critical — see [Known Limitations](#known-limitations)) |

### I2C Bus 0 (Wire0) — QMC5883L

| Signal | Teensy 4.0 Pin | Notes |
|---|---|---|
| SDA | 18 | 4.7 kΩ pull-up to 3.3 V on M9N-5883 GPS Module |
| SCL | 19 | 4.7 kΩ pull-up to 3.3 V on M9N-5883 GPS Module |

### Serial — NEO-M9N GPS

The NEO-M9N connects to Teensy Serial5 (TX=pin 20, RX=pin 21). The firmware configures
38 400 baud and 10 Hz UBX output during `GPS_init()` and saves the configuration to the
module's flash, so subsequent power cycles do not require full reconfiguration.

| Signal | Teensy 4.0 Pin | Notes |
|---|---|---|
| TX (GPS → Teensy) | RX5 (pin 21) | 38 400 baud after init; factory default 9600 baud |
| RX (Teensy → GPS) | TX5 (pin 20) | UBX CFG commands sent during GPS_init() |

### Serial — XBee Radio

| Signal | Teensy 4.0 Pin | Notes |
|---|---|---|
| TX (Teensy → XBee) | TX3 (pin 14) | 230 400 baud, 8N1 |
| RX (XBee → Teensy) | RX3 (pin 15) | Not used in current firmware |

### Serial — S.Bus Input (optional)

S.Bus receiver for RC input when the tracker is mounted on a vehicle requiring
attitude stabilisation. Not required for standalone GPS beacon operation.

| Signal | Teensy 4.0 Pin | Notes |
|---|---|---|
| RX (S.Bus → Teensy) | RX4 (pin 16) | 100 000 baud, 8E2, signal inverted (SERIAL_8E2_RXINV) |

### Servo Outputs (optional)

PWM servo outputs for vehicle attitude control. Not required for standalone GPS
beacon operation.

| Signal | Teensy 4.0 Pin |
|---|---|
| SERVO1 | 2 |
| SERVO2 | 3 |
| SERVO3 | 4 |
| SERVO4 | 5 |
| SERVO5 | 6 |
| SERVO6 | 7 |

---

## Dependencies

All dependencies should be installed via the Arduino Library Manager or PlatformIO,
pinned to the versions listed. Behavior on other versions is untested.

| Library | Version | Source |
|---|---|---|
| SparkFun ICM-20948 Arduino Library | **1.3.2** | Arduino Library Manager / [GitHub](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) |
| SparkFun BMP581 Arduino Library | 1.0.x (latest) | Arduino Library Manager / [GitHub](https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library) |
| DFRobot QMC5883 Library | no formal releases — install from GitHub | [GitHub](https://github.com/DFRobot/DFRobot_QMC5883) |
| Teensyduino | 1.59 | [pjrc.com](https://www.pjrc.com/teensy/td_download.html) |

> **Note:** The NEO-M9N GPS is driven by a custom UBX binary parser in `GPS_NEOM9N.ino`.
> The SparkFun u-blox GNSS library is **not** used and should not be installed.

> ⚠️ **Required manual edit — the sketch will not compile without this step.**
>
> The SparkFun ICM-20948 library ships with DMP support disabled by default because
> the DMP firmware image adds ~14 kB to the build. You must enable it manually:
>
> 1. Locate `ICM_20948_C.h` inside the installed library:
>    - **Windows:** `Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util\ICM_20948_C.h`
>    - **macOS/Linux:** `~/Arduino/libraries/SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_C.h`
> 2. Open the file in a text editor.
> 3. Find line 29 and uncomment it so it reads:
>    ```c
>    #define ICM_20948_USE_DMP
>    ```
> 4. Save the file.
>
> This edit persists across sketch compilations but must be re-applied if you reinstall or update the library.

**Toolchain:** Arduino IDE 2.x or PlatformIO with the Teensy platform. PlatformIO is
recommended; `platformio.ini` is included in the repository.

---

## Building and Flashing

### PlatformIO (recommended)

```bash
git clone https://github.com/lcumbridge/mrs-airborne-tracker.git
cd mrs-airborne-tracker
pio run --target upload
```

PlatformIO will resolve library dependencies automatically from `platformio.ini`.

### Arduino IDE

1. Install Teensyduino 1.59 from [pjrc.com](https://www.pjrc.com/teensy/td_download.html).
2. Install library dependencies listed above via **Tools → Manage Libraries**.
3. **Enable DMP support in the ICM-20948 library** — see the ⚠️ warning in the Dependencies section above. This step is required before the sketch will compile.
4. Open `mrs-airborne-tracker.ino`.
5. Select **Teensy 4.0** under **Tools → Board**.
6. Set **Tools → CPU Speed** to **600 MHz**.
7. Click **Upload**.

---

## Configuration

The following constants are the primary tuning points. EKF noise parameters live in
`EKF_Nav.ino`; GPS and telemetry timing live in the main sketch defines. Constants
marked with `†` match the `AirTelemetry_t` wire struct and must be kept consistent
with the ground-station parser if changed.

| Constant | Default | Description |
|---|---|---|
| `GPS_UPDATE_RATE_HZ` | `10` | UBX NAV-PVT rate. Match to ground-station EKF tick. |
| `Q_ACCEL` | `0.03` | EKF process noise, accelerometer (m²/s³) |
| `Q_BIAS` | `1e-3` | EKF process noise, bias random-walk (m²/s⁵) |
| `R_GPS_POS` | `4.0` | GPS position measurement noise (m²); overridden by hAcc² when available |
| `R_GPS_VEL` | `0.04` | GPS velocity measurement noise ((m/s)²) |
| `R_BARO` | `0.05` | Barometric altitude measurement noise (m²) |
| `GPS_SPIKE_THRESHOLD_M` | `15.0` | EKF innovation gate for GPS position jumps (m) |
| `Data_Radio baud` | `230400` | XBee serial baud. Must match XBee firmware config. |
| `GPS_BAUD` | `38400` | NEO-M9N operating baud rate (saved to module flash) |

### First-Time GPS Configuration

The NEO-M9N defaults to NMEA at 9600 baud from the factory. On first power-up,
`GPS_init()` detects the current baud rate, then runs `configureUBX()` which switches
to UBX-only output at 38 400 baud, enables 10 Hz NAV-PVT output, and saves the
configuration to the module's flash via CFG-CFG. Subsequent power cycles skip the full
configuration sequence because the module restores from flash. If the GPS does not
respond, connect a logic analyser or USB-Serial adapter to Serial5 RX (pin 21) and
confirm UBX 0xB5 sync bytes are arriving at 38 400 baud.

---

## Telemetry Frame Format

The airborne firmware transmits a 36-byte frame at up to 112 Hz (attitude-only) and
marks GPS fixes fresh at ~4.5 Hz. The ground station (`mrs-ground-station`) parses
this frame. If you modify the struct, the ground-station parser must be updated to match.

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 2 | sync | — | Sync bytes `0xAA 0x55` (not in CRC) |
| 2 | 2 | `uint16_t` | `frame_id` | Rolling frame counter |
| 4 | 4 | `uint32_t` | `timestamp_ms` | `millis()` at assembly time |
| 8 | 4 | `int32_t` | `lat_deg7` | WGS-84 latitude × 1e7 |
| 12 | 4 | `int32_t` | `lon_deg7` | WGS-84 longitude × 1e7 |
| 16 | 2 | `int16_t` | `alt_cm` | MSL altitude, cm |
| 18 | 2 | `int16_t` | `vel_n_cms` | NED North velocity, cm/s |
| 20 | 2 | `int16_t` | `vel_e_cms` | NED East velocity, cm/s |
| 22 | 2 | `int16_t` | `vel_d_cms` | NED Down velocity, cm/s |
| 24 | 2 | `int16_t` | `quat_x_sc` | Quaternion X × 32767 |
| 26 | 2 | `int16_t` | `quat_y_sc` | Quaternion Y × 32767 |
| 28 | 2 | `int16_t` | `quat_z_sc` | Quaternion Z × 32767 |
| 30 | 1 | `uint8_t` | `gps_fix` | bits[7:4]=fix_type  bits[3:0]=HDOP×2 |
| 31 | 1 | `uint8_t` | `gps_fresh` | 1=new GPS fix, 0=attitude-only |
| 32 | 2 | `uint16_t` | `crc16` | CRC-16/CCITT-FALSE over bytes [2..31] |

Total on wire: 2 sync bytes + 32-byte struct = **34 bytes per frame**.

Quaternion W is omitted; the ground station reconstructs it as
`qw = sqrt(max(0, 1 − qx² − qy² − qz²))`. The firmware enforces `qw ≥ 0`, so the
positive-root reconstruction is unambiguous. All multi-byte fields are little-endian.

---

## Architecture and Key Design Decisions

### ICM-20948 in DMP Mode via SPI1

The ICM-20948 connects to the Teensy 4.0 via SPI1 with RE1C002UNTCL MOSFET level
shifters on the MOSI, MISO, SCK1, and CS1 lines. The on-chip Digital Motion Processor is used
rather than raw sensor fusion on the Teensy. The DMP runs a calibrated sensor fusion
algorithm producing quaternion orientation at 112 Hz. This offloads the Teensy from
maintaining a separate AHRS and provides factory-calibrated gyroscope and accelerometer
integration. The tradeoff is reduced transparency into the fusion internals and
dependence on SparkFun's DMP driver maintaining compatibility with the silicon.

### 9-State EKF

The onboard EKF maintains a 9-element state vector: NED position (3), NED velocity (3),
and NED acceleration (3). GPS NAV-PVT provides position and velocity measurements at
10 Hz. IMU-derived linear acceleration (from the DMP quaternion and raw accelerometer
data, with gravity removed) drives the process model between GPS updates. Covariance
scaling on GPS measurements is adjusted based on the fix quality flag from NAV-PVT.

The EKF on the airborne side serves primarily to smooth GPS velocity and provide
continuous state estimates between GPS fixes. The ground station runs a second EKF
on the received telemetry for gimbal command generation.

### GPS Spike Rejection and Fix Validity

Individual GPS fixes are validated using both `fixType >= 3` and the `gnssFixOK` flag
from NAV-PVT (flags byte bit 0). The u-blox documentation recommends this dual check
because `fixType` can report 3 while internal accuracy filters are not yet satisfied.
Fixes that imply a position jump exceeding `GPS_SPIKE_THRESHOLD_M` in a single epoch
are discarded. This guards against momentary multipath or satellite reacquisition events
that would otherwise inject step errors into the velocity and acceleration estimates.

### Wire2 Bus Isolation for BMP581

The BMP581 is on Wire2 (pins 25/24) rather than Wire0 (pins 18/19). Wire0 is occupied
by the QMC5883L magnetometer. Keeping the barometer and magnetometer on separate I2C
buses avoids any potential bus contention and simplifies the driver initialisation
sequence, since each bus can be brought up independently. The ICM-20948 is on SPI1
and has no interaction with either I2C bus.

### Telemetry Frame and CRC

The 34-byte payload plus 2-byte CRC-16/CCITT-FALSE (polynomial 0x1021, initial value
0xFFFF, no input/output reflection) is the minimum frame that conveys all data the
ground station EKF needs. Frame length was constrained to fit comfortably within the
XBee 802.15.4 maximum payload (100 bytes) while leaving headroom for future fields.

---

## Gyroscope Stabilization

The firmware includes a full 3-axis PID gyroscope stabiliser for fixed-wing aircraft,
running concurrently with the GPS beacon and EKF pipeline. The stabiliser reads pilot
inputs from the S.Bus receiver, computes stabilised commands using DMP-derived attitude
and gyroscope rates, and writes PWM output to up to seven hobby servos.

### S.Bus Channel Mapping

| Channel | S.Bus Index | Function | Scaling |
|---|---|---|---|
| Ch1 | `radio_channels[0]` | Throttle | `(raw − 170) / 1800` → 0.0–1.0 |
| Ch2 | `radio_channels[1]` | Aileron setpoint | `(raw − 1000) / 1000` → −1.0–+1.0 |
| Ch3 | `radio_channels[2]` | Elevator setpoint | `(raw − 1000) / 1000` → −1.0–+1.0 |
| Ch4 | `radio_channels[3]` | Rudder setpoint | `(raw − 1000) / 1000` → −1.0–+1.0 |
| Ch8 | `radio_channels[7]` | Mode switch | < 600 = Rate, 600–1200 = Gyro Off, > 1200 = Heading Hold |
| Ch9 | `radio_channels[8]` | Ki tune (in-flight) | `(raw − 172) / 250` |
| Ch13 | `radio_channels[12]` | Kp tune (in-flight) | `(raw − 172) / 250` |
| Ch14 | `radio_channels[13]` | Kd tune (in-flight) | `(raw − 172) / 250` |

### Control Modes

Control mode is selected by the Ch8 switch (`radio_channels[7]`):

**Rate mode (Ch8 < 600):** The PID controller tracks a commanded angular rate derived
from the stick deflection. The aileron stick commands roll rate up to `MAX_ROLL_RATE`
(240 °/s), the elevator stick commands pitch rate up to `MAX_PITCH_RATE` (240 °/s),
and the rudder stick commands yaw rate up to `MAX_YAW_RATE` (120 °/s). The gyroscope
rate from the ICM-20948 DMP is the feedback signal. A 2 °/s stick deadband prevents
integrator wind-up at stick centre.

**Gyro off (600 ≤ Ch8 ≤ 1200):** The PID controller is bypassed. Stick inputs are
passed directly to the servo outputs with no stabilisation. Used for initial flight
trim and as a fallback if the gyroscope behaves unexpectedly.

**Heading-hold mode (Ch8 > 1200):** The PID controller holds the current attitude
(roll and pitch angles from the DMP) against disturbances. The stick commands an
attitude setpoint (±`maxRoll` = 90°, ±`maxPitch` = 90°) rather than a rate. Yaw
remains in rate mode under heading hold. The integrator is zeroed when the throttle
is below 250 µs or when the attitude error exceeds 45°.

### Servo Output Mapping

All servo outputs are PWM at 50 Hz, range 900–2100 µs, neutral at 1500 µs.
Gyro-stabilised scaling is ±667 µs (±1.0 PID output); passthrough scaling is ±500 µs.

| Servo | Teensy Pin | Function |
|---|---|---|
| SERVO1 | 2 | Aileron (left or right wing — match airframe) |
| SERVO2 | 3 | Aileron (paired, same axis) |
| SERVO3 | 4 | Elevator |
| SERVO4 | 5 | Elevator (paired, same axis) |
| SERVO5 | 6 | Rudder |
| SERVO6 | 7 | Rudder (paired, same axis) |

### In-Flight PID Tuning

Kp, Ki, and Kd can be adjusted in flight via radio pot channels (Ch13, Ch9, Ch14
respectively) without reflashing. The same gain set is applied to all three axes.
Gains tuned in flight should be noted and hardcoded into the PID initialisation
block before the next flight to ensure repeatability.

---

## Known Limitations

**These are documented honestly. This is a prototype system in active development.**

1. **GPS iTOW absent from telemetry frame.** The u-blox NAV-PVT message includes a
   millisecond-accurate GPS time-of-week (`iTOW`) that uniquely identifies when each
   fix was generated. This field is not included in the 34-byte telemetry frame. As a
   result, the ground station cannot distinguish between a freshly received fix and a
   retransmission or a frame that arrived late. Under normal 10 Hz operation at low
   link latency this is not a significant error source, but it becomes a systematic
   limitation at higher latencies or when XBee frames are dropped and retried. This
   is the highest-priority field to add in a frame format revision. Tracked in
   [Issue #3](https://github.com/lcumbridge/mrs-airborne-tracker/issues/3).

2. **BMP581 LGA cold-solder susceptibility.** The BMP581 is packaged in a 10-pad LGA
   (land grid array) with no visible solder joints after reflow. During hardware
   integration an intermittent I2C failure was traced to a cold solder joint on the
   BMP581. If you are assembling this hardware, use a stepped reflow profile matched
   to your solder paste specifications and inspect with X-ray or hot-air reflow
   verification if available. Symptoms of a marginal joint are sporadic `Wire2.endTransmission()`
   failures at startup.

3. **BMP581 CSB pullup required.** The BMP581's `CSB` pin must be tied to 3.3 V to
   latch the device into I2C mode at power-up. If `CSB` is left floating or pulled low,
   the device initializes in SPI mode and will not respond on I2C. This is a latent
   hardware risk on any custom PCB that omits the pullup.

4. **Single-frequency GPS accuracy.** The NEO-M9N is a multi-constellation,
   single-frequency (L1) receiver (GPS + GLONASS + Galileo + BeiDou enabled).
   Horizontal position accuracy under open-sky conditions is approximately 2–3 m CEP.
   Multipath or partial sky obstruction degrades this further. The EKF measurement
   noise parameters are tuned for open-sky operation.

5. **No IMU extrinsic calibration.** The IMU-to-body-frame alignment is assumed to be
   identity (IMU mounted nominally aligned with the airframe axes). Any physical
   misalignment introduces a constant bias in the NED acceleration estimates.

---

## Performance Characterization

**Environment:** Open-sky outdoor testing, stationary target, clear weather.

| Metric | Observed Value | Conditions |
|---|---|---|
| GPS fix acquisition (cold start) | 45–90 s | Clear sky, no almanac |
| GPS fix acquisition (warm start) | 5–15 s | Almanac current |
| Telemetry frame loss rate | < 1% | Line-of-sight, ≤ 50 m range |
| EKF position RMS vs. GPS | ~1.5 m | Stationary, open sky |
| IMU quaternion output rate | 112 Hz | DMP mode, confirmed via timestamp |
| Telemetry transmit jitter | < 2 ms | Measured at XBee TX pin |
| Teensy loop execution time | < 1 ms | Full loop including EKF predict step |

Performance at extended range, high target dynamics, or degraded GPS conditions has
not been systematically characterized.

---

## License

This project is released under the
[PolyForm Noncommercial License 1.0.0](LICENSE).

You are free to use, study, and modify this software for **noncommercial purposes**
with attribution. The required attribution notice is:

> Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)

**Commercial use — including incorporation into a commercial product, service, or
internal tooling at a for-profit entity — requires a separate commercial license from
MicroRobo Systems LLC.**

To inquire about commercial licensing, contact us via
[microrobosys.com](https://microrobosys.com).

---

## Citation

If you reference this work in a publication, technical report, or derivative project,
please cite it as:

```
Cumbridge, L. (2025). mrs-airborne-tracker: Airborne EKF and telemetry firmware
for GPS beacon tracking. MicroRobo Systems LLC.
https://github.com/lcumbridge/mrs-airborne-tracker
```

Or in BibTeX:

```bibtex
@software{cumbridge2025airborne,
  author  = {Cumbridge, Leonard},
  title   = {mrs-airborne-tracker: Airborne {EKF} and telemetry firmware for {GPS} beacon tracking},
  year    = {2025},
  url     = {https://github.com/lcumbridge/mrs-airborne-tracker},
  note    = {MicroRobo Systems LLC}
}
```
