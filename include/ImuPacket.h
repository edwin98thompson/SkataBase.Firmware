#pragma once

#include <stdint.h>

struct ImuPacket
{
    uint8_t protocolVersion = 1;

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
};