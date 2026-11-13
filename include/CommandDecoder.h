
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "CommandPacket.h"

class CommandDecoder
{
public:
    static constexpr size_t PACKET_SIZE = 6;

    using PacketCallback = void (*)(const CommandPacket& packet, void* context);

    CommandDecoder() = default;

    void reset();

    // Feed incoming BLE bytes into the decoder.
    // Handles fragmented and consecutive packets.
    void feed(
        const uint8_t* data,
        size_t length,
        PacketCallback callback,
        void* context);

private:
    uint8_t buffer[PACKET_SIZE] = {};
    size_t bufferIndex = 0;
    bool decodePacket(
        const uint8_t* data,
        CommandPacket& packet);
};
