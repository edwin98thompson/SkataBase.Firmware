#pragma once

#include <stdint.h>

struct ImuPacket
{
    // Protocol v2 appends calibrationStatus as byte 76.
    uint8_t protocolVersion = 2;

    uint16_t sequence = 0;
    uint32_t timestampMs = 0;

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;

    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    float magX = 0.0f;
    float magY = 0.0f;
    float magZ = 0.0f;

    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    float temperatureC = 0.0f;

    uint8_t batteryPercent = 0;
    uint8_t calibrationStatus = 0;
};

enum CalibrationStatus : uint8_t
{
    // Sent in every packet so clients can observe calibration progress/results.
    CALIBRATION_NOT_CALIBRATED = 0,
    CALIBRATION_IN_PROGRESS = 1,
    CALIBRATION_COMPLETE = 2,
    CALIBRATION_FAILED = 3
};