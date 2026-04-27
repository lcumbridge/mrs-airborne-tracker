// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// BMP581_I2C_Calibrated.ino — BMP581 barometer driver with pressure calibration
// =============================================================================
//
// Initialises the BMP581 on I2C bus 2 (Wire2, Teensy pins 16 SDA / 17 SCL).
// Wire2 is used deliberately to isolate the BMP581 from the ICM-20948 DMP
// burst traffic on Wire (bus 0), which caused intermittent NAKs during
// integration testing.
//
// HARDWARE NOTE — LGA cold-solder risk:
//   The BMP581 is packaged in a 10-pad LGA with no visible solder joints.
//   Intermittent I2C failures on this bus are the most likely symptom of a
//   marginal solder joint.  If beginI2C() fails repeatedly, inspect with
//   X-ray or re-reflow the joint before assuming a hardware fault.
//
// HARDWARE NOTE — CSB pin:
//   The BMP581 CSB pin must be pulled to 3.3V to latch the device into I2C
//   mode at power-up.  If CSB is left floating or driven low, the device
//   initialises in SPI mode and will not respond on I2C.
//
// CALIBRATION:
//   PRESSURE_OFFSET_PA is a static offset derived from a reference measurement
//   at a specific site.  It MUST be recalibrated for each deployment.
//   See performReferenceCalibration() below for the procedure.
// =============================================================================

#include "SparkFun_BMP581_Arduino_Library.h"

BMP581 pressureSensor;

// I2C address — default 0x47 when SDO pin is low
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT;  // 0x47
// uint8_t i2cAddress = BMP581_I2C_ADDRESS_SECONDARY; // 0x46

// =============================================================================
// CALIBRATION PARAMETERS
//
// PRESSURE_OFFSET_PA is a static additive correction applied to every reading.
//
// *** THIS VALUE IS SITE- AND UNIT-SPECIFIC. ***
// *** You MUST recalibrate before deploying on a new unit or location.    ***
//
// Procedure:
//   1. Obtain a reference pressure from a nearby calibrated weather station
//      (correct to your altitude using a barometric formula, or use the
//      station's reported QFE if it is at your elevation).
//   2. Call performReferenceCalibration(reference_Pa) in setup().
//   3. Update PRESSURE_OFFSET_PA with the value printed to Serial.
//
// The value below was derived from a ground-level calibration at the
// MicroRobo Systems bench location. It will be wrong at other sites.
// =============================================================================
float PRESSURE_OFFSET_PA = -568.1f;  // *** RECALIBRATE PER DEPLOYMENT ***

// Optional: Sea-level altitude correction.
// Set USE_ALTITUDE_CORRECTION to true and provide your altitude above MSL
// if you want readings normalised to sea-level pressure (useful for comparing
// with weather reports).  Leave false for raw station pressure.
bool  USE_ALTITUDE_CORRECTION = false;
float ALTITUDE_METERS         = 9.0f;
const float SEA_LEVEL_STANDARD_PA = 101325.0f;
float REFERENCE_TEMP_C        = 20.33f;  // Used for altitude correction if not NaN

// =============================================================================
// CALIBRATION HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Apply barometric formula to convert station pressure to sea-level.
 *
 * Valid for altitudes below ~1000 m.  Uses the simplified exponential form.
 *
 * @param stationPressure_Pa  Measured pressure at your location (Pa)
 * @param altitude_m          Your altitude above sea level (m)
 * @param temperature_C       Temperature at the station (°C)
 * @return Sea-level corrected pressure in Pascals
 */
float calculateSeaLevelPressure(float stationPressure_Pa,
                                float altitude_m,
                                float temperature_C) {
    if (isnan(temperature_C)) temperature_C = 15.0f;

    const float g           = 9.80665f;
    const float M           = 0.0289644f;   // molar mass of dry air (kg/mol)
    const float R_universal = 8.31432f;     // universal gas constant (J/mol/K)
    float T_kelvin = temperature_C + 273.15f;

    float exponent = (g * M * altitude_m) / (R_universal * T_kelvin);
    return stationPressure_Pa * expf(exponent);
}

/**
 * @brief Apply offset (and optional altitude) correction to a raw pressure.
 *
 * @param rawPressure_Pa  Raw sensor reading (Pa)
 * @param temperature_C   Sensor temperature reading (Pa)
 * @return Calibrated pressure in Pascals
 */
float calibratePressure(float rawPressure_Pa, float temperature_C) {
    float calibratedPressure = rawPressure_Pa + PRESSURE_OFFSET_PA;

    if (USE_ALTITUDE_CORRECTION && ALTITUDE_METERS != 0.0f) {
        float tempToUse = isnan(REFERENCE_TEMP_C) ? temperature_C : REFERENCE_TEMP_C;
        calibratedPressure = calculateSeaLevelPressure(
            calibratedPressure, ALTITUDE_METERS, tempToUse);
    }
    return calibratedPressure;
}

// =============================================================================
// INITIALISATION AND READING
// =============================================================================

void BMP581_init() {
    Wire2.begin();
    // Wire2.setClock(400000);  // Uncomment to raise I2C speed to 400 kHz

    while (pressureSensor.beginI2C(i2cAddress, Wire2) != BMP5_OK) {
        Serial.println("BMP581: not responding — check wiring, CSB pullup, and solder joints");
        delay(1000);
    }
    Serial.println("BMP581: connected on Wire2");

    Serial.println("=== BMP581 Calibration Settings ===");
    Serial.print("  Pressure offset : "); Serial.print(PRESSURE_OFFSET_PA, 2);
    Serial.println(" Pa  *** site-specific — recalibrate per deployment ***");
    if (USE_ALTITUDE_CORRECTION) {
        Serial.print("  Altitude correction enabled: ");
        Serial.print(ALTITUDE_METERS, 1); Serial.println(" m");
    } else {
        Serial.println("  Altitude correction: disabled");
    }
    Serial.println("====================================");
}

/**
 * @brief Read sensor and return calibrated pressure and raw temperature.
 *
 * On communication error, returns {0, 0} and prints a diagnostic to Serial.
 * Callers should check data.pressure > 0 before using the result.
 */
bmp5_sensor_data BMP581_read() {
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor.getSensorData(&data);

    if (err == BMP5_OK) {
        data.pressure = calibratePressure(data.pressure, data.temperature);
        return data;
    } else {
        Serial.print("BMP581: read error (code ");
        Serial.print(err);
        Serial.println(") — check I2C bus and solder joints");
        return {0, 0};
    }
}

// =============================================================================
// CALIBRATION UTILITIES
// =============================================================================

/**
 * @brief Calibrate against a known reference pressure.
 *
 * Call from setup() (before flight) when you have a reference source.
 * The function averages 10 raw readings, computes the required offset,
 * and prints the result.  Update PRESSURE_OFFSET_PA with the printed value.
 *
 * @param referencePressure_Pa  Known-correct pressure at this site (Pa)
 */
void performReferenceCalibration(float referencePressure_Pa) {
    Serial.println("\n=== BMP581 Reference Calibration ===");
    Serial.print("Reference pressure: "); Serial.print(referencePressure_Pa, 2);
    Serial.println(" Pa");

    const int NUM_READINGS = 10;
    float sumPressure = 0.0f;
    Serial.print("Taking readings");
    for (int i = 0; i < NUM_READINGS; i++) {
        bmp5_sensor_data raw = {0, 0};
        pressureSensor.getSensorData(&raw);  // raw — do not apply existing offset
        sumPressure += raw.pressure;
        delay(100);
        Serial.print(".");
    }
    Serial.println();

    float avgRaw = sumPressure / NUM_READINGS;
    float calculatedOffset = referencePressure_Pa - avgRaw;

    Serial.print("Average raw: "); Serial.print(avgRaw, 2); Serial.println(" Pa");
    Serial.print("Required offset: "); Serial.print(calculatedOffset, 2); Serial.println(" Pa");
    Serial.println("Update your code with:");
    Serial.print("  float PRESSURE_OFFSET_PA = ");
    Serial.print(calculatedOffset, 2); Serial.println(";");
    Serial.println("=====================================\n");
}

/**
 * @brief Print raw vs calibrated pressure for verification.
 */
void printPressureDiagnostics() {
    bmp5_sensor_data raw = {0, 0};
    pressureSensor.getSensorData(&raw);

    float calibrated = calibratePressure(raw.pressure, raw.temperature);

    Serial.println("\n=== BMP581 Diagnostics ===");
    Serial.print("Raw:        "); Serial.print(raw.pressure, 2);  Serial.println(" Pa");
    Serial.print("Calibrated: "); Serial.print(calibrated, 2);    Serial.println(" Pa");
    Serial.print("Offset:     "); Serial.print(calibrated - raw.pressure, 2); Serial.println(" Pa");
    Serial.print("Temp:       "); Serial.print(raw.temperature, 2); Serial.println(" °C");
    Serial.println("==========================\n");
}
