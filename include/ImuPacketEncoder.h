#pragma once

#include <stdint.h>
#include <stddef.h>
#include "ImuPacket.h"

// Now that we have added calibrationStatus to ImuPacket, the size of the packet is 77 bytes.
constexpr size_t IMU_PACKET_SIZE = 81;

bool encodeImuPacket(
    const ImuPacket& packet,
    uint8_t* buffer,
    size_t capacity
);