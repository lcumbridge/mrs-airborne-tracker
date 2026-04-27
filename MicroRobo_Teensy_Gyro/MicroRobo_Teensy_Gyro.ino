// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// MicroRobo_Teensy_Gyro.ino — Main sketch: GPS beacon tracker + attitude control
// =============================================================================
//
// Top-level integration sketch for the MicroRobo Systems airborne tracker.
// Brings up all sensors, runs the main 4 ms control loop, and coordinates
// calls to the subsystem modules in the companion .ino files:
//
//   BinaryTelemetry.ino         — 34-byte binary telemetry frame transmitter
//   EKF_Nav.ino                 — 9-state GPS/IMU navigation filter
//   GPS_NEOM9N.ino              — u-blox GPS UBX parser
//   IMU_DMP_Quat6_EulerAngles.ino — ICM-20948 DMP quaternion driver
//   BMP581_I2C_Calibrated.ino  — BMP581 barometer driver
//   QMC5883_Compass_capture.ino — QMC5883L magnetometer driver
//   SBus_capture_1.ino         — Futaba S.Bus receiver parser
//
// LOOP TIMING:
//   Main loop is capped at 4000 µs (250 Hz ceiling).
//   IMU DMP runs at 112 Hz; quaternion updates occur within the loop.
//   GPS reads occur every loop iteration (non-blocking UBX parser).
//   Barometer, compass, and EKF measurement updates occur every 25th loop (~10 Hz).
//   Binary telemetry is transmitted at the IMU rate (attitude-only) and every
//   25th loop (with gps_fresh=true to trigger the ground-station EKF update).
//
// DEBUG OUTPUT:
//   Define DEBUG_USB (below) to enable USB Serial status prints at ~10 Hz.
//   Leave undefined in deployed hardware to keep the Serial port quiet.
//
// BINARY TELEMETRY:
//   USE_BINARY_PROTOCOL 1 enables the compact 34-byte binary frame via Serial3.
//   Set to 0 to fall back to the ASCII data_radio_output() path.
// =============================================================================

#include "ICM_20948.h"
#include "SparkFun_BMP581_Arduino_Library.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// Compile-time switches
// ---------------------------------------------------------------------------
// Uncomment to enable verbose USB Serial output (~10 Hz) for bench debugging.
// Leave commented in deployed hardware.
// #define DEBUG_USB

// Set to 1 to transmit compact binary telemetry frames (production).
// Set to 0 to fall back to ASCII data_radio_output() (debugging).
#define USE_BINARY_PROTOCOL 1

// ---------------------------------------------------------------------------
// Serial port aliases
// ---------------------------------------------------------------------------
#define SERIAL_PORT  Serial   // USB Serial — debug console
#define Data_Radio   Serial3  // XBee 802.15.4 radio link

// ---------------------------------------------------------------------------
// GPS data structures
// ---------------------------------------------------------------------------
struct GPSData {
    int32_t  lat;        // Latitude  (1e-7 degrees)
    int32_t  lon;        // Longitude (1e-7 degrees)
    int32_t  height;     // Height above ellipsoid (mm)
    int32_t  hMSL;       // Height above mean sea level (mm)
    int32_t  velN;       // North velocity (mm/s)
    int32_t  velE;       // East  velocity (mm/s)
    int32_t  velD;       // Down  velocity (mm/s)
    int32_t  gSpeed;     // Ground speed (mm/s)
    uint32_t hAcc;       // Horizontal accuracy estimate (mm)
    uint32_t vAcc;       // Vertical   accuracy estimate (mm)
    uint8_t  fixType;    // GPS fix type (0=no fix, 3=3D fix)
    uint8_t  numSV;      // Number of tracked satellites
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    bool     validTime;
    bool     validDate;
    uint32_t iTOW;       // GPS time-of-week (ms) — from NAV-PVT offset 0
                         // NOTE: iTOW is parsed but NOT transmitted in the
                         // telemetry frame; see Known Limitations in README.
    bool     gnssFixOK;  // NAV-PVT flags byte bit 0 — use in addition to fixType
} gpsData;

// Alpha-filter smoothed GPS (used for display; EKF is used for telemetry)
struct GPSSmoothed {
    int32_t lat;
    int32_t lon;
    int32_t hMSL;
    bool    initialized;
} gpsSmoothed = {0, 0, 0, false};

float GPS_ALPHA = 0.3f;  // EMA smoothing factor (0=no update, 1=no smoothing)

// ---------------------------------------------------------------------------
// Radio / S.Bus channel array
// ---------------------------------------------------------------------------
uint16_t channels[16];
uint16_t channels_prev[16];
uint16_t print_RX_chan[16];

// ---------------------------------------------------------------------------
// Flight mode inputs (derived from S.Bus channels in servo_commands())
// ---------------------------------------------------------------------------
unsigned int mode_command, calibrate_compass_switch, calibrate_imu_switch, cut_throttle;

const float maxRoll  = 90.0f;  // degrees
const float maxPitch = 90.0f;
const float maxYaw   = 90.0f;

const float deg2rad = PI / 180.0f;

// ---------------------------------------------------------------------------
// Sensor output globals
// ---------------------------------------------------------------------------
volatile float BMP581_temp_reading, BMP581_press_reading;

volatile float throttle_setpoint, aileron_setpoint, elevator_setpoint;
volatile float rudder_setpoint, collective_setpoint;
volatile float ref_heading;

volatile float Front_arm_position, Rear_arm_position, Right_arm_position, Left_arm_position;
volatile float Front_sin_weight, Front_cos_weight, Rear_sin_weight, Rear_cos_weight;
volatile float Right_sin_weight, Right_cos_weight, Left_sin_weight, Left_cos_weight;

volatile float PID_aileron_setpoint, PID_elevator_setpoint, PID_rudder_setpoint;
volatile float aileron_passthru, elevator_passthru, rudder_passthru;
volatile float aileron_error, elevator_error, rudder_error;
volatile float aileron_error_prev, elevator_error_prev, rudder_error_prev;

volatile float aileron_integral, aileron_integral_prev;
volatile float elevator_integral, elevator_integral_prev;
volatile float rudder_integral, rudder_integral_prev;
volatile float aileron_derivative, elevator_derivative, rudder_derivative;
volatile float aileron_PID_output, elevator_PID_output, rudder_PID_output;

volatile float motor1_PWM_command, motor2_PWM_command, motor3_PWM_command, motor4_PWM_command;
volatile int   motor1_PWM_command_out, motor2_PWM_command_out;
volatile int   motor3_PWM_command_out, motor4_PWM_command_out;
volatile int   servo1_position, servo2_position, servo3_position;
volatile int   servo4_position, servo5_position, servo6_position;

volatile int POR_loops;

// ---------------------------------------------------------------------------
// IMU quaternion and Euler angle globals
// ---------------------------------------------------------------------------
volatile float quat_w, quat_x, quat_y, quat_z;
volatile float prev_q0, prev_q1, prev_q2, prev_q3;
volatile float qw, qx, qy, qz;
volatile float q0, q1, q2, q3;
volatile float roll, pitch, yaw;
volatile float t0, t1, t2, t3, t4;

volatile float IMU_roll, IMU_pitch, IMU_yaw;
volatile float IMU_roll_prev, IMU_pitch_prev, IMU_yaw_prev;
volatile float IMU_roll_smooth, IMU_pitch_smooth, IMU_yaw_smooth;
volatile float IMU_roll_smooth_prev, IMU_pitch_smooth_prev, IMU_yaw_smooth_prev;
volatile float POR_roll, POR_pitch, POR_yaw;
volatile float roll_bias, pitch_bias, yaw_bias;

bool     new_IMU_aval;
volatile float gyro_x, gyro_y, gyro_z;
volatile float gyro_x_dps, gyro_y_dps, gyro_z_dps;
volatile float gyro_x_dps_prev, gyro_y_dps_prev, gyro_z_dps_prev;
volatile float gyro_x_bias, gyro_y_bias, gyro_z_bias;
const float Sensitivity_Scale_Factor = 16.4f;  // 2000 dps FSR: 16.4 LSB/(deg/s)

volatile float HH_or_Rate_roll, HH_or_Rate_pitch, HH_or_Rate_yaw;

// ---------------------------------------------------------------------------
// Magnetometer globals
// ---------------------------------------------------------------------------
volatile float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
volatile float compass_scale_y, compass_scale_z;
volatile float compass_offset_x, compass_offset_y, compass_offset_z;
volatile float compass_x, compass_y, compass_z;
volatile float heading;
uint8_t  compass_calibration_on;
float    compass_cal_values[6];

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
volatile float          dt;
volatile unsigned long  current_time, prev_time;
volatile unsigned long  Main_Loop_time;
volatile unsigned long  blink_counter, blink_delay;
bool blinkAlternate;
volatile unsigned long  loopStart;
volatile unsigned long  now;

const unsigned long loopDuration = 4000;  // µs — 250 Hz loop ceiling
bool     executed;
volatile unsigned long  elapsed;
volatile unsigned long  remaining;

volatile unsigned int main_loop_count = 0;
volatile unsigned int blink_timer     = 0;

// ---------------------------------------------------------------------------
// PID gain globals — tuned in flight, also adjustable via radio pot channels
// ---------------------------------------------------------------------------
volatile float Kp_test, Ki_test, Kd_test;

volatile float Kp_aileron_rate = 0.784f;
volatile float Ki_aileron_rate = 0.000f;
volatile float Kd_aileron_rate = 0.000f;

volatile float Kp_elevator_rate = 0.784f;
volatile float Ki_elevator_rate = 0.000f;
volatile float Kd_elevator_rate = 0.000f;

volatile float Kp_rudder_rate = 0.784f;
volatile float Ki_rudder_rate = 0.000f;
volatile float Kd_rudder_rate = 0.000f;

volatile float Kp_aileron_hh  = 0.784f;
volatile float Ki_aileron_hh  = 0.000f;
volatile float Kd_aileron_hh  = 0.000f;

volatile float Kp_elevator_hh = 0.784f;
volatile float Ki_elevator_hh = 0.000f;
volatile float Kd_elevator_hh = 0.000f;

volatile float Kp_rudder_hh   = 0.784f;
volatile float Ki_rudder_hh   = 0.000f;
volatile float Kd_rudder_hh   = 0.000f;

const float Kp_yaw = 0.0f;
const float Ki_yaw = 0.0f;
const float Kd_yaw = 0.0f;

// ---------------------------------------------------------------------------
// Radio failsafe channel values (µs) — applied when S.Bus signal is lost
// ---------------------------------------------------------------------------
unsigned long channel_1_fs = 1000;  // throttle — cut
unsigned long channel_2_fs = 1500;  // aileron  — centre
unsigned long channel_3_fs = 1500;  // elevator — centre
unsigned long channel_4_fs = 1500;  // rudder   — centre
unsigned long channel_5_fs = 2000;  // gear     — throttle cut active
unsigned long channel_6_fs = 1500;
unsigned long channel_7_fs = 1500;
unsigned long channel_8_fs = 1000;

// ---------------------------------------------------------------------------
// Servo output pins and objects
// ---------------------------------------------------------------------------
const int servo1Pin = 2;
const int servo2Pin = 3;
const int servo3Pin = 4;
const int servo4Pin = 5;
const int servo5Pin = 6;
const int servo6Pin = 7;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

const int DataRadio_TX = 14;
const int DataRadio_RX = 15;

// ---------------------------------------------------------------------------
// Forward declarations for functions defined in companion .ino files
// ---------------------------------------------------------------------------
extern bool capture_RX_signals(void);
extern void radio_init(void);
extern bool IMU_init(void);
extern void calibrate_IMU(void);
extern void read_IMU(void);
extern void GPS_init(void);
extern void read_gps(void);
extern void BMP581_init(void);
extern bmp5_sensor_data BMP581_read(void);
extern void Compass_init(void);
extern float Compass_capture(void);

// EKF_Nav.ino
extern void  EKF_baro_init(void);
extern void  EKF_predict(float dt);
extern void  EKF_update_GPS(double lat_deg, double lon_deg, float alt_msl_m,
                             float vN_ms, float vE_ms, float vD_ms, float hAcc_m);
extern void  EKF_update_baro(float pressure_Pa);
extern void  EKF_print_state(void);

// EKF output globals (published by EKF_Nav.ino each cycle)
extern float ekf_pN, ekf_pE, ekf_pD;
extern float ekf_vN, ekf_vE, ekf_vD;
extern float ekf_alt_m;
extern float ekf_pos_std;
extern float ekf_vel_std;
extern bool  ekf_valid;

// Raw accelerometer counts from IMU module, consumed by EKF predict
extern volatile float accel_x_body, accel_y_body, accel_z_body;

// radio_channels points at channels[] — the S.Bus parser writes channels[] in place
volatile uint16_t *radio_channels = channels;

// =============================================================================
// setup()
// =============================================================================
void setup() {
    Data_Radio.setTX(DataRadio_TX);
    Data_Radio.setRX(DataRadio_RX);
    Data_Radio.begin(230400);
    delay(1500);

    SERIAL_PORT.begin(115200);
    delay(3000);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(10);

    // Servo outputs — initialise to neutral (1500 µs)
    servo1.attach(servo1Pin, 900, 2100);
    servo2.attach(servo2Pin, 900, 2100);
    servo3.attach(servo3Pin, 900, 2100);
    servo4.attach(servo4Pin, 900, 2100);
    servo5.attach(servo5Pin, 900, 2100);
    servo6.attach(servo6Pin, 900, 2100);

    roll_bias = 0.0f;  pitch_bias = 0.0f;  yaw_bias = 0.0f;
    IMU_roll_prev  = 0.0f;  IMU_roll_smooth  = 0.0f;
    IMU_pitch_prev = 0.0f;  IMU_pitch_smooth = 0.0f;
    IMU_yaw_prev   = 0.0f;  IMU_yaw_smooth   = 0.0f;
    IMU_roll_smooth_prev  = 0.0f;
    IMU_pitch_smooth_prev = 0.0f;
    IMU_yaw_smooth_prev   = 0.0f;

    radio_init();
    delay(3000);

    // IMU_init() is called once; it returns false on unrecoverable failure.
    if (!IMU_init()) {
        Serial.println("FATAL: IMU init failed — halting");
        while (true) {
            digitalWrite(13, HIGH); delay(200);
            digitalWrite(13, LOW);  delay(200);
        }
    }
    calibrate_IMU();
    delay(5000);

    GPS_init();
    delay(200);

    BMP581_init();
    delay(200);

    Compass_init();
    delay(200);

    // Prime the BMP581 global and set the EKF barometric reference pressure
    BMP581_temp_reading = BMP581_read().temperature;
    BMP581_press_reading = BMP581_read().pressure;
    EKF_baro_init();

#if USE_BINARY_PROTOCOL
    binary_telemetry_setup();  // CRC self-test — halts on failure
#endif

    // Initialise servo outputs to neutral
    servo1_position = 1500;
    servo2_position = 1500;
    servo3_position = 1500;
    servo4_position = 1500;
    servo5_position = 1500;
    servo6_position = 1500;

    servo1.writeMicroseconds(servo1_position);
    servo2.writeMicroseconds(servo2_position);
    servo3.writeMicroseconds(servo3_position);
    servo4.writeMicroseconds(servo4_position);
    servo5.writeMicroseconds(servo5_position);
    servo6.writeMicroseconds(servo6_position);

    capture_RX_signals();
    servo_commands();
    read_gps();

    digitalWrite(13, LOW);
}

// =============================================================================
// loop()
// =============================================================================
void loop() {
    // Non-blocking S.Bus parser — must be called every iteration.
    // Returns true only when a complete, validated 25-byte frame has been decoded.
    bool sbus_fresh = capture_RX_signals();

    elapsedMicros Main_Loop_time;
    loopStart = micros();

    // LED heartbeat: double-blink every 2 s
    if      (blink_timer == 250 || blink_timer == 750 || blink_timer == 1250) digitalWrite(13, HIGH);
    else if (blink_timer == 500 || blink_timer == 1000 || blink_timer == 1750) digitalWrite(13, LOW);

    // IMU read + EKF predict + attitude PID (runs at DMP rate, ~112 Hz)
    read_IMU();
    if (new_IMU_aval == 1) {
        EKF_predict(dt);
        heading_hold_PID();
        commanded_outputs();
        new_IMU_aval = 0;

#if USE_BINARY_PROTOCOL
        // Attitude-only frame — GPS fields carry last known values; gps_fresh=false
        // tells the ground-station EKF not to apply a position/velocity update.
        send_binary_telemetry(false);
#endif
    }

    // Update radio-derived setpoints and write servo commands every loop
    if ((main_loop_count % 1) == 0) {
        servo_commands();

        // Update PID gains from radio pot channels (Ch13=P, Ch9=I, Ch14=D)
        Kp_aileron_hh = Kp_elevator_hh = Kp_rudder_hh = Kp_test;
        Ki_aileron_hh = Ki_elevator_hh = Ki_rudder_hh = Ki_test;
        Kd_aileron_hh = Kd_elevator_hh = Kd_rudder_hh = Kd_test;

        servo1.writeMicroseconds(servo1_position);
        servo2.writeMicroseconds(servo2_position);
        servo3.writeMicroseconds(servo3_position);
        servo4.writeMicroseconds(servo4_position);
        servo5.writeMicroseconds(servo5_position);
        servo6.writeMicroseconds(servo6_position);
    }

    // GPS is read every loop (non-blocking UBX parser)
    read_gps();

    // Every 25th loop (~10 Hz): sensor updates, EKF measurement step, telemetry
    if ((main_loop_count % 25) == 0) {
        bmp5_sensor_data BMP581data_new = BMP581_read();
        BMP581_temp_reading  = BMP581data_new.temperature;
        BMP581_press_reading = BMP581data_new.pressure;

        heading = Compass_capture();

        // EKF measurement updates
        EKF_update_baro(BMP581_press_reading);

        if (gpsData.fixType >= 3 && gpsData.gnssFixOK) {
            EKF_update_GPS(
                gpsData.lat  * 1e-7,     // 1e-7 ° → decimal degrees
                gpsData.lon  * 1e-7,
                gpsData.hMSL * 0.001f,   // mm → m
                getVelN_ms(), getVelE_ms(), getVelD_ms(),
                getHAcc_m()
            );
        }

#ifdef DEBUG_USB
        // USB Serial status output — active only when DEBUG_USB is defined
        print_setpoints();
        print_PID_coef();
#endif

#if USE_BINARY_PROTOCOL
        // GPS-fresh frame: ground-station EKF applies position/velocity update
        send_binary_telemetry(true);
#else
        data_radio_output();  // ASCII fallback
#endif
    }

    throttle_cut();

    // Hold loop to exactly loopDuration µs
    while (micros() - loopStart < loopDuration);

    main_loop_count++;
    blink_timer++;
    if (blink_timer >= 2000) blink_timer = 0;
    Main_Loop_time = 0;
}

// =============================================================================
// servo_commands() — decode S.Bus channels into normalised setpoints
// =============================================================================
void servo_commands() {
    throttle_setpoint  = ((radio_channels[0] - 170)  / 1800.0f);  // Ch1
    aileron_setpoint   = ((radio_channels[1] - 1000) / 1000.0f);  // Ch2
    elevator_setpoint  = ((radio_channels[2] - 1000) / 1000.0f);  // Ch3
    rudder_setpoint    = ((radio_channels[3] - 1000) / 1000.0f);  // Ch4
    cut_throttle       = ((radio_channels[4] - 1000) / 1000.0f);  // Ch5
    collective_setpoint = (radio_channels[6] - 1000) / 1000.0f;  // Ch7

    Kp_test = ((radio_channels[12] - 172) / 250.0f);  // Ch13
    Ki_test = ((radio_channels[8]  - 172) / 250.0f);  // Ch9
    Kd_test = ((radio_channels[13] - 172) / 250.0f);  // Ch14

    calibrate_compass_switch = radio_channels[9];   // Ch10
    calibrate_imu_switch     = radio_channels[10];  // Ch11
    mode_command             = radio_channels[11];  // Ch12

    // Stick deadband
    if (aileron_setpoint  > -0.005f && aileron_setpoint  < 0.005f) aileron_setpoint  = 0.0f;
    if (elevator_setpoint > -0.005f && elevator_setpoint < 0.005f) elevator_setpoint = 0.0f;
    if (rudder_setpoint   > -0.005f && rudder_setpoint   < 0.005f) rudder_setpoint   = 0.0f;

    throttle_setpoint   = constrain(throttle_setpoint, 0.0f, 1.0f);
    aileron_setpoint    = constrain(aileron_setpoint,  -1.0f, 1.0f) * maxRoll;
    elevator_setpoint   = constrain(elevator_setpoint, -1.0f, 1.0f) * maxPitch;
    rudder_setpoint     = constrain(rudder_setpoint,   -1.0f, 1.0f) * maxYaw;
    collective_setpoint = constrain(collective_setpoint, -1.0f, 1.0f);

    // Normalise to ±1.0 for servo output mapping
    aileron_passthru  = aileron_setpoint  / maxRoll;
    elevator_passthru = elevator_setpoint / maxPitch;
    rudder_passthru   = rudder_setpoint   / maxYaw;
}

// =============================================================================
// heading_hold_PID() — 3-axis attitude PID controller
//
// Switch SC (Ch8 / radio_channels[7]) position selects mode:
//   < 600 µs  : Rate command mode (gyro-rate tracking)
//   > 1200 µs : Heading hold mode (angle stabilisation)
//   600–1200  : Passthrough (handled in commanded_outputs)
// =============================================================================
void heading_hold_PID() {
    static float prev_roll_rate_error  = 0.0f;
    static float prev_pitch_rate_error = 0.0f;
    static float prev_yaw_rate_error   = 0.0f;
    static float prev_roll_for_derivative  = 0.0f;
    static float prev_pitch_for_derivative = 0.0f;

    const float MAX_ROLL_RATE  = 240.0f;   // deg/s
    const float MAX_PITCH_RATE = 240.0f;
    const float MAX_YAW_RATE   = 120.0f;
    const float STICK_DEADBAND = 0.02f;

    // ---- AILERON (ROLL) ----
    if (radio_channels[7] < 600) {
        // Rate command
        float desired_roll_rate = aileron_passthru * MAX_ROLL_RATE;
        float actual_roll_rate  = gyro_x_dps;
        float roll_rate_error   = desired_roll_rate - actual_roll_rate;

        aileron_integral = aileron_integral_prev + (roll_rate_error * dt);
        if (fabsf(aileron_passthru) < STICK_DEADBAND || fabsf(roll_rate_error) > 200.0f)
            aileron_integral = 0.0f;
        aileron_integral = constrain(aileron_integral, -100.0f, 100.0f);

        aileron_derivative = (roll_rate_error - prev_roll_rate_error) / dt;
        prev_roll_rate_error = roll_rate_error;

        float aileron_correction = (Kp_aileron_hh * roll_rate_error)
                                 + (Ki_aileron_hh * aileron_integral)
                                 + (Kd_aileron_hh * aileron_derivative);
        aileron_PID_output = constrain(aileron_correction / MAX_ROLL_RATE, -1.0f, 1.0f);
        aileron_integral_prev = aileron_integral;

    } else if (radio_channels[7] > 1200) {
        // Heading hold
        HH_or_Rate_roll = -IMU_roll_smooth;
        float aileron_base_command = aileron_passthru;

        aileron_error    = aileron_passthru - HH_or_Rate_roll;
        aileron_integral = aileron_integral_prev + (aileron_error * dt);
        if (radio_channels[0] < 250 || fabsf(aileron_error) > 45.0f) aileron_integral = 0.0f;
        aileron_integral = constrain(aileron_integral, -400.0f, 400.0f);

        aileron_derivative = -(HH_or_Rate_roll - prev_roll_for_derivative) / dt;
        prev_roll_for_derivative = HH_or_Rate_roll;

        float aileron_correction = ((Kp_aileron_hh / 50.0f) * aileron_error)
                                 + (Ki_aileron_hh * aileron_integral)
                                 + (Kd_aileron_hh * aileron_derivative);
        aileron_PID_output = constrain(aileron_base_command + aileron_correction, -1.0f, 1.0f);
        aileron_integral_prev = aileron_integral;
        aileron_error_prev    = aileron_error;
    }

    // ---- ELEVATOR (PITCH) ----
    if (radio_channels[7] < 600) {
        float desired_pitch_rate = elevator_passthru * MAX_PITCH_RATE;
        float pitch_rate_error   = desired_pitch_rate - gyro_y_dps;

        elevator_integral = elevator_integral_prev + (pitch_rate_error * dt);
        if (fabsf(elevator_passthru) < STICK_DEADBAND || fabsf(pitch_rate_error) > 200.0f)
            elevator_integral = 0.0f;
        elevator_integral = constrain(elevator_integral, -100.0f, 100.0f);

        elevator_derivative = (pitch_rate_error - prev_pitch_rate_error) / dt;
        prev_pitch_rate_error = pitch_rate_error;

        float elevator_correction = (Kp_elevator_hh * pitch_rate_error)
                                  + (Ki_elevator_hh * elevator_integral)
                                  + (Kd_elevator_hh * elevator_derivative);
        elevator_PID_output = constrain(elevator_correction / MAX_PITCH_RATE, -1.0f, 1.0f);
        elevator_integral_prev = elevator_integral;

    } else if (radio_channels[7] > 1200) {
        HH_or_Rate_pitch = IMU_pitch_smooth;
        float elevator_base_command = elevator_passthru;

        elevator_error    = elevator_passthru - HH_or_Rate_pitch;
        elevator_integral = elevator_integral_prev + (elevator_error * dt);
        if (radio_channels[0] < 250 || fabsf(elevator_error) > 45.0f) elevator_integral = 0.0f;
        elevator_integral = constrain(elevator_integral, -400.0f, 400.0f);

        elevator_derivative = -(HH_or_Rate_pitch - prev_pitch_for_derivative) / dt;
        prev_pitch_for_derivative = HH_or_Rate_pitch;

        float elevator_correction = -((Kp_elevator_hh / 50.0f) * elevator_error)
                                   -(Ki_elevator_hh * elevator_integral)
                                   -(Kd_elevator_hh * elevator_derivative);
        elevator_PID_output = constrain(elevator_base_command + elevator_correction, -1.0f, 1.0f);
        elevator_integral_prev = elevator_integral;
        elevator_error_prev    = elevator_error;
    }

    // ---- RUDDER (YAW) ---- rate mode only
    if ((radio_channels[7] < 600) || (radio_channels[7] > 1200)) {
        float yaw_rate_error = (rudder_passthru * MAX_YAW_RATE) - gyro_z_dps;

        rudder_integral = rudder_integral_prev + (yaw_rate_error * dt);
        if (fabsf(rudder_passthru) < STICK_DEADBAND || fabsf(yaw_rate_error) > 200.0f)
            rudder_integral = 0.0f;
        rudder_integral = constrain(rudder_integral, -100.0f, 100.0f);

        rudder_derivative = (yaw_rate_error - prev_yaw_rate_error) / dt;
        prev_yaw_rate_error = yaw_rate_error;

        float rudder_correction = (Kp_rudder_hh * yaw_rate_error)
                                + (Ki_rudder_hh * rudder_integral)
                                + (Kd_rudder_hh * rudder_derivative);
        rudder_PID_output = constrain(rudder_correction / MAX_YAW_RATE, -1.0f, 1.0f);
        rudder_integral_prev = rudder_integral;
    }
}

// =============================================================================
// commanded_outputs() — map PID outputs to servo µs positions
// =============================================================================
void commanded_outputs() {
    if (radio_channels[7] >= 600 && radio_channels[7] <= 1200) {
        // Passthrough mode — gyro off
        servo1_position = round(aileron_passthru  * 500) + 1500;
        servo2_position = round(aileron_passthru  * 500) + 1500;
        servo3_position = round(elevator_passthru * 500) + 1500;
        servo4_position = round(elevator_passthru * 500) + 1500;
        servo5_position = round(rudder_passthru   * 500) + 1500;
        servo6_position = round(rudder_passthru   * 500) + 1500;
    } else {
        // Gyro-stabilised mode
        servo1_position = round(aileron_PID_output  * 667) + 1500;
        servo2_position = round(aileron_PID_output  * 667) + 1500;
        servo3_position = round(elevator_PID_output * 667) + 1500;
        servo4_position = round(elevator_PID_output * 667) + 1500;
        servo5_position = round(rudder_PID_output   * 667) + 1500;
        servo6_position = round(rudder_PID_output   * 667) + 1500;
    }

    servo1_position = constrain(servo1_position, 900, 2100);
    servo2_position = constrain(servo2_position, 900, 2100);
    servo3_position = constrain(servo3_position, 900, 2100);
    servo4_position = constrain(servo4_position, 900, 2100);
    servo5_position = constrain(servo5_position, 900, 2100);
    servo6_position = constrain(servo6_position, 900, 2100);
}

// =============================================================================
// throttle_cut() — zero motor commands when Ch5 is below failsafe threshold
// =============================================================================
void throttle_cut() {
    if (radio_channels[4] < 1400) {
        motor1_PWM_command_out = 125;
        motor2_PWM_command_out = 125;
        motor3_PWM_command_out = 125;
        motor4_PWM_command_out = 125;
    }
}

// =============================================================================
// Debug print functions — compiled always, called only when DEBUG_USB is defined
// =============================================================================

void print_SBus_in() {
    for (int j = 0; j < 16; j++) {
        SERIAL_PORT.print(radio_channels[j]);
        SERIAL_PORT.print(", ");
    }
    Serial.println();
}

void printGPSData() {
    Serial.print("Fix: "); Serial.print(gpsData.fixType);
    Serial.print(" gnssFixOK: "); Serial.print(gpsData.gnssFixOK);
    Serial.print(" Sats: ");      Serial.print(gpsData.numSV);
    if (gpsData.fixType >= 3) {
        Serial.print(" Lat: "); Serial.print(gpsSmoothed.lat / 10000000.0, 7);
        Serial.print(" Lon: "); Serial.print(gpsSmoothed.lon / 10000000.0, 7);
        Serial.print(" Alt: "); Serial.print(gpsSmoothed.hMSL / 1000.0, 2);
        Serial.print("m Speed: "); Serial.print(gpsData.gSpeed / 1000.0, 2);
        Serial.print("m/s HAcc: "); Serial.print(gpsData.hAcc / 1000.0, 2); Serial.println("m");
    } else {
        Serial.println(" | Acquiring...");
    }
}

void print_Compass() {
    Serial.print("Heading: "); Serial.println(heading);
}

void print_IMU_PID_debug() {
    Serial.print("dt:"); Serial.print(dt*1000, 2); Serial.print("ms  ");
    Serial.print("R:"); Serial.print(IMU_roll_smooth, 2); Serial.print("  ");
    Serial.print("R_sp:"); Serial.print(aileron_setpoint, 2); Serial.print("  ");
    Serial.print("R_err:"); Serial.print(aileron_error, 2); Serial.print("  ");
    Serial.print("R_I:"); Serial.print(aileron_integral, 2); Serial.print("  ");
    Serial.print("R_out:"); Serial.println(aileron_PID_output, 3);
}

void print_setpoints() {
    Serial.print("accel_z_body: "); Serial.print(accel_z_body);
    Serial.print("  Heading: "); Serial.print(heading);
    Serial.print("  Temp: "); Serial.print(BMP581_temp_reading, 1);
    Serial.print("°C  Press: "); Serial.print(BMP581_press_reading, 1); Serial.println(" Pa");

    Serial.print("Fix: "); Serial.print(gpsData.fixType);
    Serial.print(" gnssFixOK: "); Serial.print(gpsData.gnssFixOK);
    Serial.print(" Sats: "); Serial.print(gpsData.numSV);

    if (gpsData.fixType >= 3 && gpsData.gnssFixOK) {
        Serial.print(" Lat: "); Serial.print(gpsData.lat / 10000000.0, 7);
        Serial.print(" Lon: "); Serial.print(gpsData.lon / 10000000.0, 7);
        Serial.print(" Alt: "); Serial.print(gpsData.hMSL / 1000.0, 2);
        Serial.print("m  velN: "); Serial.print(getVelN_ms(), 2);
        Serial.print(" velE: ");   Serial.print(getVelE_ms(), 2);
        Serial.print(" velD: ");   Serial.print(getVelD_ms(), 2);
        Serial.print(" hAcc: ");   Serial.print(getHAcc_m(), 2); Serial.println("m");
    } else {
        Serial.println(gpsData.fixType == 2 ? " | 2D fix" : " | Acquiring...");
    }

    Serial.print("EKF valid:"); Serial.print(ekf_valid ? 1 : 0);
    Serial.print(" pN:"); Serial.print(ekf_pN, 2);
    Serial.print(" pE:"); Serial.print(ekf_pE, 2);
    Serial.print(" alt:"); Serial.print(ekf_alt_m, 2);
    Serial.print("m  vN:"); Serial.print(ekf_vN, 3);
    Serial.print(" vE:"); Serial.print(ekf_vE, 3);
    Serial.print(" vD:"); Serial.print(ekf_vD, 3);
    Serial.print("  σ_pos:"); Serial.print(ekf_pos_std, 2);
    Serial.print("m  σ_vel:"); Serial.print(ekf_vel_std, 3); Serial.println("m/s");

    Serial.print("qw:"); Serial.print(qw, 4);
    Serial.print(" qx:"); Serial.print(qx, 4);
    Serial.print(" qy:"); Serial.print(qy, 4);
    Serial.print(" qz:"); Serial.println(qz, 4);

    Serial.print("Roll:"); Serial.print(IMU_roll_smooth, 4);
    Serial.print(" Pitch:"); Serial.print(IMU_pitch_smooth, 4);
    Serial.print(" Yaw:"); Serial.println(IMU_yaw_smooth, 4);
}

void print_PID_coef() {
    Serial.print("Kp:"); Serial.print(Kp_test, 4);
    Serial.print("  Ki:"); Serial.print(Ki_test, 4);
    Serial.print("  Kd:"); Serial.println(Kd_test, 4);
}

// =============================================================================
// ASCII telemetry fallback — active only when USE_BINARY_PROTOCOL == 0
// =============================================================================
void data_radio_output() {
    Data_Radio.print("Heading: ");     Data_Radio.print(heading);
    Data_Radio.print(", Temp: ");      Data_Radio.print(BMP581_temp_reading);
    Data_Radio.print("C, Press: ");    Data_Radio.println(BMP581_press_reading);

    Data_Radio.print("Fix: "); Data_Radio.print(gpsData.fixType);
    Data_Radio.print(" Sats: "); Data_Radio.print(gpsData.numSV);
    if (gpsData.fixType >= 3) {
        Data_Radio.print(" Lat: "); Data_Radio.print(gpsData.lat / 10000000.0, 7);
        Data_Radio.print(" Lon: "); Data_Radio.print(gpsData.lon / 10000000.0, 7);
        Data_Radio.print(" Alt: "); Data_Radio.print(gpsData.hMSL / 1000.0, 2);
        Data_Radio.println("m");
    } else {
        Data_Radio.println(" | Acquiring...");
    }

    Data_Radio.print("EKF_valid:"); Data_Radio.print(ekf_valid ? 1 : 0);
    Data_Radio.print(" pN:"); Data_Radio.print(ekf_pN, 2);
    Data_Radio.print(" pE:"); Data_Radio.print(ekf_pE, 2);
    Data_Radio.print(" alt:"); Data_Radio.print(ekf_alt_m, 2);
    Data_Radio.print("m vN:"); Data_Radio.print(ekf_vN, 3);
    Data_Radio.print(" vE:"); Data_Radio.print(ekf_vE, 3);
    Data_Radio.print(" vD:"); Data_Radio.println(ekf_vD, 3);

    Data_Radio.print("qw:"); Data_Radio.print(qw, 4);
    Data_Radio.print(" qx:"); Data_Radio.print(qx, 4);
    Data_Radio.print(" qy:"); Data_Radio.print(qy, 4);
    Data_Radio.print(" qz:"); Data_Radio.println(qz, 4);
    Data_Radio.println();
}
