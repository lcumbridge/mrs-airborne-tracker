// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// QMC5883_Compass_capture.ino — QMC5883L magnetometer driver
// =============================================================================
//
// Target hardware : QMC5883L magnetometer (I2C address 0x0D)
//                   Teensy 4.0, Wire (I2C0, pins 18 SDA / 19 SCL)
//
// Library         : DFRobot_QMC5883 (supports HMC5883L and QMC5883L;
//                   isQMC() returns true for the QMC5883L silicon variant)
//
// Heading output is tilt-compensated using IMU roll and pitch from the DMP,
// and corrected for magnetic declination at the deployment site.
//
// CALIBRATION:
//   Hard-iron offsets in Compass_capture() were derived from a full-rotation
//   calibration sweep processed through Magneto 1.2.  They are hardware- and
//   site-specific.  To re-calibrate, call calibrate_compass() and process the
//   printed min/max values through Magneto 1.2.
//
// MAGNETIC DECLINATION:
//   MAGNETIC_DECLINATION is set for Palm Bay, FL (−7.45° W).
//   Update this value for your deployment location using NOAA's online
//   calculator at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
// =============================================================================

#include <DFRobot_QMC5883.h>

// Magnetic declination for Palm Bay, FL (negative = West of true North)
// *** Update this for your deployment location ***
#define MAGNETIC_DECLINATION -7.4482f

// Magnetometer object on Wire (I2C0, Teensy pins 18/19)
DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

// =============================================================================
// Compass_init()
// =============================================================================
void Compass_init() {
    compass.begin();
    delay(200);

    if (!compass.isQMC()) {
        Serial.println("ERROR: QMC5883L not detected — check wiring and I2C address 0x0D");
        return;
    }
    Serial.println("QMC5883L detected — configuring...");

    compass.setRange(QMC5883_RANGE_2GA);
    Serial.print("  Range        : "); Serial.println(compass.getRange());

    compass.setMeasurementMode(QMC5883_CONTINOUS);
    Serial.print("  Mode         : "); Serial.println(compass.getMeasurementMode());

    compass.setDataRate(QMC5883_DATARATE_50HZ);
    Serial.print("  Data rate    : "); Serial.println(compass.getDataRate());

    compass.setSamples(QMC5883_SAMPLES_8);
    Serial.print("  Oversampling : "); Serial.println(compass.getSamples());

    // Set declination once here; no need to recompute a constant every call
    float declinationRad = MAGNETIC_DECLINATION * (PI / 180.0f);
    compass.setDeclinationAngle(declinationRad);
    Serial.print("  Declination  : "); Serial.print(MAGNETIC_DECLINATION, 4);
    Serial.println(" deg");

    Serial.println("QMC5883L ready.");
}

// =============================================================================
// Compass_capture()
//
// Reads the magnetometer, applies hard-iron offset correction, applies
// tilt compensation using IMU-smoothed roll and pitch, adds magnetic
// declination, and returns geographic heading in degrees (0–360).
//
// Hard-iron offsets below are derived from Magneto 1.2 output (negate the
// reported bias to correct the raw counts).  Sensor operates at 2 Gauss FS,
// 12 000 LSB/Gauss.  To re-calibrate, see calibrate_compass() below.
// =============================================================================
float Compass_capture() {
    sVector_t mag = compass.readRaw();

    compass_x = mag.XAxis;
    compass_y = mag.YAxis;
    compass_z = mag.ZAxis;

    // Hard-iron offset correction — active when not in calibration mode
    if (compass_calibration_on == 0) {
        compass_x -= 801.0f;
        compass_y -= 291.0f;
        compass_z -= 205.0f;
    }

    // Tilt compensation: project magnetometer field onto the virtual horizontal
    // plane using DMP-smoothed roll and pitch angles.
    float pitch_rad = IMU_pitch_smooth * 0.0174533f;  // degrees → radians
    float roll_rad  = IMU_roll_smooth  * 0.0174533f;

    compass_x_horizontal =  compass_y * cosf(pitch_rad)
                          + compass_x * sinf(roll_rad) * sinf(pitch_rad)
                          + compass_z * cosf(roll_rad) * sinf(pitch_rad);

    compass_y_horizontal =  compass_x * cosf(roll_rad)
                          - compass_z * sinf(roll_rad);

    // Compute heading; magnetic declination was set in Compass_init()
    actual_compass_heading  = atan2f(-compass_y_horizontal, -compass_x_horizontal)
                              * (180.0f / PI);
    actual_compass_heading += MAGNETIC_DECLINATION;

    // Normalise to [0, 360)
    if (actual_compass_heading <    0.0f) actual_compass_heading += 360.0f;
    if (actual_compass_heading >= 360.0f) actual_compass_heading -= 360.0f;

    return actual_compass_heading;
}

// =============================================================================
// calibrate_compass()
//
// Live min/max sweep calibration.  Rotate the vehicle slowly through all
// orientations while this function runs.  Results are printed to Serial on
// completion.  Record the min/max values, run through Magneto 1.2, invert
// the combined bias vector, and update the hard-iron offsets in Compass_capture().
//
// The function returns when radio channel 10 goes above 1000 µs (switch off).
// =============================================================================
void calibrate_compass() {
    compass_calibration_on = 1;
    int loops = 0;

    Serial3.print("channel 10 output ");  Serial3.println(radio_channels[9]);
    Serial3.print("calibration active "); Serial3.println(compass_calibration_on);
    delay(8000);

    while (radio_channels[9] < 1000) {
        if (loops % 25 == 0) digitalWrite(13, !digitalRead(13));
        delayMicroseconds(3700);

        heading = Compass_capture();

        if (compass_x < compass_cal_values[0]) compass_cal_values[0] = compass_x;
        if (compass_x > compass_cal_values[1]) compass_cal_values[1] = compass_x;
        if (compass_y < compass_cal_values[2]) compass_cal_values[2] = compass_y;
        if (compass_y > compass_cal_values[3]) compass_cal_values[3] = compass_y;
        if (compass_z < compass_cal_values[4]) compass_cal_values[4] = compass_z;
        if (compass_z > compass_cal_values[5]) compass_cal_values[5] = compass_z;

        loops++;
        Serial.print("\rX: ");  Serial.print(compass_x);
        Serial.print("  Y: "); Serial.print(compass_y);
        Serial.print("  Z: "); Serial.println(compass_z);

        capture_RX_signals();
        servo_commands();
    }

    compass_calibration_on = 0;

    // Compute offsets and scale factors from min/max sweep
    float xRange = compass_cal_values[1] - compass_cal_values[0];
    float yRange = compass_cal_values[3] - compass_cal_values[2];
    float zRange = compass_cal_values[5] - compass_cal_values[4];

    compass_scale_y = xRange / yRange;
    compass_scale_z = xRange / zRange;

    compass_offset_x = (xRange / 2.0f) - compass_cal_values[1];
    compass_offset_y = ((yRange / 2.0f) - compass_cal_values[3]) * compass_scale_y;
    compass_offset_z = ((zRange / 2.0f) - compass_cal_values[5]) * compass_scale_z;

    Serial.print("\rX min: ");   Serial.print(compass_cal_values[0]);
    Serial.print("  X max: ");   Serial.println(compass_cal_values[1]);
    Serial.print("Y min: ");     Serial.print(compass_cal_values[2]);
    Serial.print("  Y max: ");   Serial.println(compass_cal_values[3]);
    Serial.print("Z min: ");     Serial.print(compass_cal_values[4]);
    Serial.print("  Z max: ");   Serial.println(compass_cal_values[5]);
    Serial.print("X offset: ");  Serial.println(compass_offset_x);
    Serial.print("Y offset: ");  Serial.println(compass_offset_y);
    Serial.print("Z offset: ");  Serial.println(compass_offset_z);
    Serial.print("Y Scale: ");   Serial.println(compass_scale_y);
    Serial.print("Z Scale: ");   Serial.println(compass_scale_z);

    delay(8000);
}
