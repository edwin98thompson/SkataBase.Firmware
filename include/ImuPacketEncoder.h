#pragma once

#include <stdint.h>
#include <stddef.h>
#include "ImuPacket.h"

constexpr size_t IMU_PACKET_SIZE = 76;

bool encodeImuPacket(
    const ImuPacket& packet,
    uint8_t* buffer,
    size_t capacity
);