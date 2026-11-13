#include "ImuPacketEncoder.h"

#include <cstring>
#include <limits>
#include <Arduino.h>

static_assert(sizeof(float) == 4, "Requires 32-bit floats");
static_assert(
    std::numeric_limits<float>::is_iec559,
    "Requires IEEE-754 floats"
);

namespace
{
    size_t offset = 0;

    void writeU8(uint8_t value, uint8_t* buffer)
    {
        buffer[offset++] = value;
    }

    void writeU16(uint16_t value, uint8_t* buffer)
    {
        buffer[offset++] = static_cast<uint8_t>(value);
        buffer[offset++] = static_cast<uint8_t>(value >> 8);
    }

    void writeU32(uint32_t value, uint8_t* buffer)
    {
        buffer[offset++] = static_cast<uint8_t>(value);
        buffer[offset++] = static_cast<uint8_t>(value >> 8);
        buffer[offset++] = static_cast<uint8_t>(value >> 16);
        buffer[offset++] = static_cast<uint8_t>(value >> 24);
    }

    void writeFloat(float value, uint8_t* buffer)
    {
        uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        writeU32(bits, buffer);
    }
}

bool encodeImuPacket(
    const ImuPacket& packet,
    uint8_t* buffer,
    size_t capacity)
{
    if (buffer == nullptr || capacity < IMU_PACKET_SIZE)
    {
        return false;
    }

    offset = 0;

    writeU8(packet.protocolVersion, buffer);

    writeU16(packet.sequence, buffer);
    writeU32(packet.timestampMs, buffer);

    writeFloat(packet.roll, buffer);
    writeFloat(packet.pitch, buffer);
    writeFloat(packet.yaw, buffer);

    writeFloat(packet.accelX, buffer);
    writeFloat(packet.accelY, buffer);
    writeFloat(packet.accelZ, buffer);

    writeFloat(packet.gyroX, buffer);
    writeFloat(packet.gyroY, buffer);
    writeFloat(packet.gyroZ, buffer);

    writeFloat(packet.magX, buffer);
    writeFloat(packet.magY, buffer);
    writeFloat(packet.magZ, buffer);

    writeFloat(packet.q0, buffer);
    writeFloat(packet.q1, buffer);
    writeFloat(packet.q2, buffer);
    writeFloat(packet.q3, buffer);

    writeFloat(packet.temperatureC, buffer);

    writeU8(packet.batteryPercent, buffer);
    writeU8(packet.calibrationStatus, buffer);
    writeFloat(packet.betaValue, buffer);

    return offset == IMU_PACKET_SIZE;
}