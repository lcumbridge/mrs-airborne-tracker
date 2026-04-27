// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// IMU_DMP_Quat6_EulerAngles.ino — ICM-20948 DMP quaternion and attitude driver
// =============================================================================
//
// Initialises the TDK ICM-20948 via SPI1, enables the on-chip Digital Motion
// Processor (DMP), and reads 6-axis quaternion, raw gyro, and raw accelerometer
// data from the DMP FIFO at 112 Hz.
//
// The DMP Quat6 output is a gravity-referenced quaternion derived from
// accelerometer and gyroscope fusion (no magnetometer).  Yaw is gyro-integrated
// and will drift slowly over time — use the QMC5883L heading for absolute yaw.
//
// QUATERNION AXIS REMAP:
//   The ICM-20948 is mounted such that its physical axes do not align with
//   the airframe NED convention.  The following remap is applied in read_IMU():
//     qw =  q0          (scalar component, forced positive)
//     qx = −q1          (physical Q1 negated)
//     qy =  q2          (physical Q2 unchanged)
//     qz = −q3          (physical Q3 negated)
//   The same axis swap is applied to the raw accelerometer in read_IMU() for
//   consistency with the EKF predict step in EKF_Nav.ino.
//
// DMP ENABLE REQUIREMENT:
//   The DMP firmware consumes 14 301 bytes of flash on the ICM-20948.  To
//   use this file you must edit the SparkFun ICM-20948 library header:
//     SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_C.h
//   Uncomment line 29:
//     #define ICM_20948_USE_DMP
//   Save before building.
//
// ATTRIBUTION:
//   DMP initialisation sequence derived from:
//   Paul Clark / SparkFun Electronics, "ICM-20948 DMP Quaternion 6-axis Euler"
//   example, April 25 2021 (Apache-2.0 / "Distributed as-is; no warranty").
//   Original SparkFun example code available at:
//     https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
// =============================================================================

// SPI interface configuration for ICM-20948
#define USE_SPI           // Use SPI rather than I2C
#define SPI_PORT  SPI1    // Teensy 4.0 SPI1 bus
#define CS_PIN    0       // Chip-select pin for ICM-20948

#define WIRE_PORT Wire    // Retained for the #ifdef !USE_SPI path below
#define AD0_VAL   1       // I2C address selection bit (unused in SPI mode)

// Raw accelerometer counts from the DMP FIFO.
// These are consumed by EKF_Nav.ino; do NOT scale here.
volatile float accel_x_body = 0.0f;
volatile float accel_y_body = 0.0f;
volatile float accel_z_body = 0.0f;

#ifdef USE_SPI
ICM_20948_SPI myICM;
#else
ICM_20948_I2C myICM;
#endif

icm_20948_DMP_data_t data;

// Power-on reset flag — used by calibrate_IMU() to gate bias application
bool POR = true;

// =============================================================================
// IMU_init()
//
// Initialises the ICM-20948 and loads the DMP firmware.
// Retries up to MAX_RETRIES times on transient communication failures.
//
// Returns true on success, false if all retries are exhausted.
// Caller (setup()) is responsible for halting if this returns false.
// =============================================================================
bool IMU_init(void) {
    const int MAX_RETRIES = 3;
    int  retry_count  = 0;
    bool init_success = false;

    Serial.println("\n========================================");
    Serial.println("Initializing ICM-20948 IMU...");
    Serial.println("========================================");

#ifdef USE_SPI
    SPI_PORT.begin();
#else
    WIRE_PORT.setSDA(18);
    WIRE_PORT.setSCL(19);
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif

    while (!init_success && retry_count < MAX_RETRIES) {
        if (retry_count > 0) {
            Serial.print("Retry attempt "); Serial.print(retry_count);
            Serial.println("/3");
            delay(500);
        }

#ifdef USE_SPI
        myICM.begin(CS_PIN, SPI_PORT);
#else
        myICM.begin(WIRE_PORT, AD0_VAL);
#endif

        if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.print("ERROR: IMU begin failed — ");
            Serial.println(myICM.statusString());
            retry_count++;
            continue;
        }
        Serial.println("  IMU communication established");

        myICM.swReset();
        if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.print("ERROR: Software reset failed — ");
            Serial.println(myICM.statusString());
            retry_count++;
            continue;
        }
        Serial.println("  Software reset complete");
        delay(100);

        myICM.sleep(false);
        myICM.lowPower(false);
        Serial.println("  Sensor awake");

        if (myICM.initializeDMP() != ICM_20948_Stat_Ok) {
            Serial.print("ERROR: DMP initialization failed — ");
            Serial.println(myICM.statusString());
            retry_count++;
            continue;
        }
        Serial.println("  DMP initialized");

        if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GRAVITY) != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to enable GRAVITY sensor");
            retry_count++;
            continue;
        }
        Serial.println("  Gravity sensor enabled");

        if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to enable RAW_GYRO sensor");
            retry_count++;
            continue;
        }
        Serial.println("  Raw gyroscope enabled");

        if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to enable RAW_ACCELEROMETER sensor");
            retry_count++;
            continue;
        }
        Serial.println("  Raw accelerometer enabled");

        // DMP output rates.  Value = (DMP_rate / desired_rate) − 1.
        // DMP internal rate = 112 Hz.
        if (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok) {  // 112 Hz
            Serial.println("ERROR: Failed to set Quat6 ODR"); retry_count++; continue;
        }
        if (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 1) != ICM_20948_Stat_Ok) {   // 56 Hz
            Serial.println("ERROR: Failed to set Gyro ODR");  retry_count++; continue;
        }
        if (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) != ICM_20948_Stat_Ok) {  // 112 Hz
            Serial.println("ERROR: Failed to set Accel ODR"); retry_count++; continue;
        }
        Serial.println("  DMP output rates: Quat6=112Hz  Gyro=56Hz  Accel=112Hz");

        if (myICM.enableFIFO() != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to enable FIFO"); retry_count++; continue;
        }
        if (myICM.enableDMP() != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to enable DMP");  retry_count++; continue;
        }
        if (myICM.resetDMP() != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to reset DMP");   retry_count++; continue;
        }
        if (myICM.resetFIFO() != ICM_20948_Stat_Ok) {
            Serial.println("ERROR: Failed to reset FIFO");  retry_count++; continue;
        }

        init_success = true;
        Serial.println("========================================");
        Serial.println("IMU INITIALIZATION SUCCESSFUL");
        Serial.println("========================================\n");
    }

    if (!init_success) {
        Serial.println("\n========================================");
        Serial.println("CRITICAL ERROR: IMU INITIALIZATION FAILED");
        Serial.println("Check SPI wiring, CS pin, and power supply");
        Serial.println("========================================\n");
        for (int i = 0; i < 20; i++) {
            digitalWrite(13, HIGH); delay(50);
            digitalWrite(13, LOW);  delay(50);
        }
    }

    return init_success;
}

// =============================================================================
// read_IMU()
//
// Drains the DMP FIFO and processes any quaternion, gyro, or accelerometer
// packets present.  Sets new_IMU_aval = 1 when fresh quaternion data is
// available (after the POR_loops warmup period).
//
// Must be called every loop iteration for timely FIFO draining.
// =============================================================================
void read_IMU(void) {
    static unsigned long prev_imu_time = 0;
    unsigned long current_imu_time = micros();

    myICM.readDMPdataFromFIFO(&data);

    if ((myICM.status == ICM_20948_Stat_Ok) ||
        (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {

        // ----- Quaternion (Quat6, 6-axis, no magnetometer) -----
        if ((data.header & DMP_header_bitmap_Quat6) > 0) {
            // Scale DMP fixed-point Q30 values to double
            q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
            q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
            q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
            q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            // Axis remap to match airframe NED convention (see file header)
            qw =  q0;
            qx = -q1;
            qy =  q2;
            qz = -q3;

            // Convert remapped quaternion to Euler angles
            t0 = +2.0 * (qw * qx + qy * qz);
            t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
            pitch = atan2(t0, t1) * 180.0 / PI;

            t2 = +2.0 * (qw * qy - qx * qz);
            t2 = constrain(t2, -1.0, 1.0);  // clamp to asin domain
            roll = asin(t2) * 180.0 / PI;

            t3 = +2.0 * (qw * qz + qx * qy);
            t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
            yaw = atan2(t3, t4) * 180.0 / PI;

            IMU_roll  = roll;
            IMU_pitch = pitch;
            IMU_yaw   = yaw;

            if (isnan(IMU_roll) || isnan(IMU_pitch) || isnan(IMU_yaw)) {
                Serial.println("WARNING: NaN in IMU Euler angles — skipping");
                return;
            }

            if (POR_loops >= 2000) {
                // Apply static bias from calibrate_IMU()
                IMU_roll  -= roll_bias;
                IMU_pitch -= pitch_bias;
                IMU_yaw   -= yaw_bias;

                // Complementary filter: 70% previous + 30% new
                // Roll and pitch are negated to match the servo convention
                const float alpha = 0.7f;
                IMU_roll_smooth  = alpha * IMU_roll_smooth  + (1.0f - alpha) * (-IMU_roll);
                IMU_pitch_smooth = alpha * IMU_pitch_smooth + (1.0f - alpha) * (-IMU_pitch);
                IMU_yaw_smooth   = alpha * IMU_yaw_smooth   + (1.0f - alpha) *   IMU_yaw;

                new_IMU_aval = 1;

                // Calculate loop dt here, where the IMU timestamp is fresh
                if (prev_imu_time > 0) {
                    dt = (current_imu_time - prev_imu_time) / 1000000.0f;
                    if (dt > 0.1f || dt < 0.001f) dt = 0.009f;  // clamp to expected ~112 Hz
                } else {
                    dt = 0.009f;  // first-pass initialisation
                }
                prev_imu_time = current_imu_time;

            } else {
                // Warmup: accumulate samples for calibrate_IMU()
                if (POR_loops % 100 == 0) {
                    Serial.print("Warmup — Roll:"); Serial.print(IMU_roll, 2);
                    Serial.print(" Pitch:");        Serial.println(IMU_pitch, 2);
                }
            }
        }

        // ----- Raw gyroscope -----
        if ((data.header & DMP_header_bitmap_Gyro) > 0) {
            // Axis remap mirrors the quaternion remap above
            gyro_x = -(float)data.Raw_Gyro.Data.Y;
            gyro_y =  (float)data.Raw_Gyro.Data.X;
            gyro_z =  (float)data.Raw_Gyro.Data.Z;

            // Convert counts to deg/s (2000 dps FSR, 16-bit: 16.4 LSB/dps)
            gyro_x_dps = (gyro_x / Sensitivity_Scale_Factor) - gyro_x_bias;
            gyro_y_dps = (gyro_y / Sensitivity_Scale_Factor) - gyro_y_bias;
            gyro_z_dps = (gyro_z / Sensitivity_Scale_Factor) - gyro_z_bias;

            // Low-pass filter to reduce quantisation noise at control-loop rates
            const float gyro_alpha = 0.7f;
            gyro_x_dps = gyro_alpha * gyro_x_dps_prev + (1.0f - gyro_alpha) * gyro_x_dps;
            gyro_y_dps = gyro_alpha * gyro_y_dps_prev + (1.0f - gyro_alpha) * gyro_y_dps;
            gyro_z_dps = gyro_alpha * gyro_z_dps_prev + (1.0f - gyro_alpha) * gyro_z_dps;

            gyro_x_dps_prev = gyro_x_dps;
            gyro_y_dps_prev = gyro_y_dps;
            gyro_z_dps_prev = gyro_z_dps;
        }

        // ----- Raw accelerometer (for EKF predict) -----
        // Counts are passed to EKF_Nav.ino without scaling; the EKF applies
        // the ±4g FSR factor (8192 LSB/g) internally.
        if ((data.header & DMP_header_bitmap_Accel) > 0) {
            accel_x_body = -(float)data.Raw_Accel.Data.Y;
            accel_y_body =  (float)data.Raw_Accel.Data.X;
            accel_z_body =  (float)data.Raw_Accel.Data.Z;
        }
    }
}

// =============================================================================
// calibrate_IMU()
//
// Collects up to MAX_SAMPLES readings while the vehicle is stationary and
// computes average roll/pitch/yaw and gyro bias offsets.  The derived biases
// are written to the global roll_bias, pitch_bias, yaw_bias, gyro_*_bias
// variables and applied by read_IMU() once POR_loops >= 2000.
//
// Exits early when variance drops below MAX_VARIANCE for WINDOW_SIZE consecutive
// samples.  Prints progress and results to USB Serial.
// =============================================================================
void calibrate_IMU(void) {
    Serial.println("========================================");
    Serial.println("IMU CALIBRATION — keep vehicle stationary");
    Serial.println("========================================");

    const int   MIN_SAMPLES  = 500;
    const int   MAX_SAMPLES  = 3000;
    const int   WINDOW_SIZE  = 50;
    const float MAX_VARIANCE = 0.5f;

    float roll_window[WINDOW_SIZE];
    float pitch_window[WINDOW_SIZE];
    float yaw_window[WINDOW_SIZE];
    int   window_idx = 0;

    double roll_sum  = 0.0, pitch_sum  = 0.0, yaw_sum  = 0.0;
    double gyro_x_sum = 0.0, gyro_y_sum = 0.0, gyro_z_sum = 0.0;

    int  sample_count             = 0;
    bool calibration_good         = false;
    int  consecutive_good_windows = 0;

    unsigned long last_print = 0;
    POR_loops = 0;

    while (!calibration_good && sample_count < MAX_SAMPLES) {
        read_IMU();

        if (isnan(IMU_roll) || isnan(IMU_pitch) || isnan(IMU_yaw)) {
            Serial.println("WARNING: Invalid IMU data during calibration, retrying...");
            delay(10);
            continue;
        }

        roll_sum   += IMU_roll;
        pitch_sum  += IMU_pitch;
        yaw_sum    += IMU_yaw;
        gyro_x_sum += gyro_x_dps;
        gyro_y_sum += gyro_y_dps;
        gyro_z_sum += gyro_z_dps;

        roll_window[window_idx]  = IMU_roll;
        pitch_window[window_idx] = IMU_pitch;
        yaw_window[window_idx]   = IMU_yaw;
        window_idx = (window_idx + 1) % WINDOW_SIZE;

        sample_count++;
        POR_loops = sample_count;

        // Check variance every WINDOW_SIZE samples
        if (sample_count >= MIN_SAMPLES && (sample_count % WINDOW_SIZE) == 0) {
            double roll_mean  = 0.0, pitch_mean = 0.0;
            for (int i = 0; i < WINDOW_SIZE; i++) {
                roll_mean  += roll_window[i];
                pitch_mean += pitch_window[i];
            }
            roll_mean  /= WINDOW_SIZE;
            pitch_mean /= WINDOW_SIZE;

            double roll_var = 0.0, pitch_var = 0.0;
            for (int i = 0; i < WINDOW_SIZE; i++) {
                roll_var  += (roll_window[i]  - roll_mean)  * (roll_window[i]  - roll_mean);
                pitch_var += (pitch_window[i] - pitch_mean) * (pitch_window[i] - pitch_mean);
            }
            roll_var  /= WINDOW_SIZE;
            pitch_var /= WINDOW_SIZE;

            if (roll_var < MAX_VARIANCE && pitch_var < MAX_VARIANCE) {
                consecutive_good_windows++;
                if (consecutive_good_windows >= 3) calibration_good = true;
            } else {
                consecutive_good_windows = 0;
            }
        }

        if (millis() - last_print > 500) {
            last_print = millis();
            Serial.print("  Calibrating... samples: "); Serial.print(sample_count);
            Serial.print("  R:"); Serial.print(IMU_roll, 2);
            Serial.print(" P:"); Serial.println(IMU_pitch, 2);
        }
    }

    // Compute bias from full accumulated average
    roll_bias   = (float)(roll_sum   / sample_count);
    pitch_bias  = (float)(pitch_sum  / sample_count);
    yaw_bias    = (float)(yaw_sum    / sample_count);
    gyro_x_bias = (float)(gyro_x_sum / sample_count);
    gyro_y_bias = (float)(gyro_y_sum / sample_count);
    gyro_z_bias = (float)(gyro_z_sum / sample_count);

    POR_loops = 2001;  // Signal bias-correction gate in read_IMU()

    Serial.println("========================================");
    Serial.println("IMU CALIBRATION COMPLETE");
    Serial.print("  Samples    : "); Serial.println(sample_count);
    Serial.print("  Roll bias  : "); Serial.println(roll_bias,  4);
    Serial.print("  Pitch bias : "); Serial.println(pitch_bias, 4);
    Serial.print("  Yaw bias   : "); Serial.println(yaw_bias,   4);
    Serial.print("  Gyro X bias: "); Serial.println(gyro_x_bias, 4);
    Serial.print("  Gyro Y bias: "); Serial.println(gyro_y_bias, 4);
    Serial.print("  Gyro Z bias: "); Serial.println(gyro_z_bias, 4);
    if (!calibration_good) Serial.println("  WARNING: max samples reached — variance may be high");
    Serial.println("========================================\n");
}

// =============================================================================
// calibrate_DMP()  —  low-level DMP configuration called by IMU_init()
//
// This function is called on myICM (ICM_20948_SPI) as a method, not a free
// function, because it uses private DMP memory write methods inherited from
// the SparkFun library.  The implementation is largely verbatim from the
// SparkFun ICM-20948 DMP example with FSR and ODR values updated for this
// application.
// =============================================================================
ICM_20948_Status_e calibrate_DMP(void) {
    ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;

    // The struct below is a local alias for calling the ICM class methods;
    // it is not a separate object.  See SparkFun library for method signatures.
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm4;      // ±4g accelerometer FSR
    myFSS.g = dps2000;   // ±2000 dps gyroscope FSR

    ICM_20948_Status_e result;
    result = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (result > worstResult) worstResult = result;

    result = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
    if (result > worstResult) worstResult = result;

    result = myICM.setBank(0);
    if (result > worstResult) worstResult = result;
    uint8_t zero = 0;
    result = myICM.write(AGB0_REG_FIFO_EN_1, &zero, 1);
    if (result > worstResult) worstResult = result;
    result = myICM.write(AGB0_REG_FIFO_EN_2, &zero, 1);
    if (result > worstResult) worstResult = result;

    result = myICM.intEnableRawDataReady(false);
    if (result > worstResult) worstResult = result;
    result = myICM.resetFIFO();
    if (result > worstResult) worstResult = result;

    ICM_20948_smplrt_t mySmplrt;
    mySmplrt.g = 8;  // 1100/(1+8) = 112 Hz
    mySmplrt.a = 8;
    result = myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
    if (result > worstResult) worstResult = result;

    result = myICM.setDMPstartAddress();
    if (result > worstResult) worstResult = result;
    result = myICM.loadDMPFirmware();
    if (result > worstResult) worstResult = result;
    result = myICM.setDMPstartAddress();
    if (result > worstResult) worstResult = result;

    result = myICM.setBank(0);
    if (result > worstResult) worstResult = result;
    uint8_t fix = 0x48;
    result = myICM.write(AGB0_REG_HW_FIX_DISABLE, &fix, 1);
    if (result > worstResult) worstResult = result;
    uint8_t fifoPrio = 0xE4;
    result = myICM.write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
    if (result > worstResult) worstResult = result;

    // Accel scale for DMP (±4g FSR → 0x04000000 = 2^26 = 1g)
    const unsigned char accScale[4]  = {0x04, 0x00, 0x00, 0x00};
    const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    result = myICM.writeDMPmems(ACC_SCALE,  4, &accScale[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result;

    // Compass mount matrix (AK09916 axis alignment to ICM accel/gyro axes)
    const unsigned char mountZero[4]  = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mountPlus[4]  = {0x09, 0x99, 0x99, 0x99};  // +1 in Q30
    const unsigned char mountMinus[4] = {0xF6, 0x66, 0x66, 0x67};  // −1 in Q30
    result = myICM.writeDMPmems(CPASS_MTX_00, 4, &mountPlus[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_01, 4, &mountZero[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_02, 4, &mountZero[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_10, 4, &mountZero[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_11, 4, &mountMinus[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_12, 4, &mountZero[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_20, 4, &mountZero[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_21, 4, &mountZero[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(CPASS_MTX_22, 4, &mountMinus[0]); if (result > worstResult) worstResult = result;

    // B2S (body-to-sensor) mounting matrix — identity for nominal mounting
    const unsigned char b2sPlus[4] = {0x40, 0x00, 0x00, 0x00};
    result = myICM.writeDMPmems(B2S_MTX_00, 4, &b2sPlus[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_01, 4, &mountZero[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_02, 4, &mountZero[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_10, 4, &mountZero[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_11, 4, &b2sPlus[0]);  if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_12, 4, &mountZero[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_20, 4, &mountZero[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_21, 4, &mountZero[0]); if (result > worstResult) worstResult = result;
    result = myICM.writeDMPmems(B2S_MTX_22, 4, &b2sPlus[0]);  if (result > worstResult) worstResult = result;

    // Gyro scaling factor for 112 Hz / 2000 dps
    result = myICM.setGyroSF(8, 3); if (result > worstResult) worstResult = result;

    // Gyro full-scale: 2000 dps = 2^28
    const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00};
    result = myICM.writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]);
    if (result > worstResult) worstResult = result;

    // Accel-only gain and alpha/A variance values for 112 Hz
    const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D};
    result = myICM.writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
    if (result > worstResult) worstResult = result;

    const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92};
    result = myICM.writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
    if (result > worstResult) worstResult = result;

    const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E};
    result = myICM.writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]);
    if (result > worstResult) worstResult = result;

    const unsigned char accelCalRate[4] = {0x00, 0x00};
    result = myICM.writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
    if (result > worstResult) worstResult = result;

    // Compass time buffer at 69 Hz
    const unsigned char compassRate[2] = {0x00, 0x45};
    result = myICM.writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
    if (result > worstResult) worstResult = result;

    return worstResult;
}
