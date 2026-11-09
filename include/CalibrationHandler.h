#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "ImuPacket.h"

class ImuManager;

class CalibrationHandler
{
public:
    bool begin();

    bool calibrateGyroscope(
        ImuManager& imu,
        uint16_t sampleCount = 500,
        uint32_t timeoutMs = 10000);

    // Keep the board stationary and level, with its positive Z axis pointing up.
    bool calibrateAccelerometer(
        ImuManager& imu,
        uint16_t sampleCount = 500,
        uint32_t timeoutMs = 10000);

    // Begin a non-blocking calibration; pass raw samples to update until complete.
    bool startMagnetometerCalibration(uint32_t durationMs = 30000);
    void updateMagnetometerCalibration(const ImuPacket* rawSample);

    void apply(ImuPacket& packet) const;
    bool isMagnetometerCalibrated() const;
    CalibrationStatus status() const;
    
private:
    bool saveMagnetometerCalibration(
        float offsetX,
        float offsetY,
        float offsetZ,
        float scaleX,
        float scaleY,
        float scaleZ);

    Preferences preferences;
    bool storageReady = false;
    bool magnetometerCalibrated = false;
    CalibrationStatus calibrationStatus = CALIBRATION_NOT_CALIBRATED;
    bool collectingMagnetometer = false;
    uint32_t magnetometerStartMs = 0;
    uint32_t magnetometerDurationMs = 0;
    uint32_t lastMagnetometerProgressMs = 0;
    uint32_t magnetometerSamples = 0;
    float magMinX = 0.0f;
    float magMinY = 0.0f;
    float magMinZ = 0.0f;
    float magMaxX = 0.0f;
    float magMaxY = 0.0f;
    float magMaxZ = 0.0f;

    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;

    float accelOffsetX = 0.0f;
    float accelOffsetY = 0.0f;
    float accelOffsetZ = 0.0f;

    float magOffsetX = 0.0f;
    float magOffsetY = 0.0f;
    float magOffsetZ = 0.0f;

    float magScaleX = 1.0f;
    float magScaleY = 1.0f;
    float magScaleZ = 1.0f;
};
