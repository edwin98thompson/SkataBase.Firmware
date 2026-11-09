
#include "CommandDecoder.h"

#include <Arduino.h>
#include <string.h>
#include <math.h>

void CommandDecoder::reset()
{
    bufferIndex = 0;
}

void CommandDecoder::feed(
    const uint8_t* data,
    size_t length,
    PacketCallback callback,
    void* context)
{
    if (data == nullptr)
        return;

    for (size_t i = 0; i < length; ++i)
    {
        buffer[bufferIndex++] = data[i];

        if (bufferIndex == PACKET_SIZE)
        {
            CommandPacket packet;

            if (decodePacket(buffer, packet))
            {
                if (callback != nullptr)
                    callback(packet, context);
            }

            bufferIndex = 0;
        }
    }
}

bool CommandDecoder::decodePacket(
    const uint8_t* data,
    CommandPacket& packet)
{
    const uint8_t version = data[0];
    const uint8_t commandId = data[1];

    // Validate protocol version.
    if (version != 1)
        return false;

    // Validate command ID.
    if (commandId < CMD_NONE ||
        commandId > CMD_SET_BETA_VALUE)
    {
        return false;
    }

    // Decode little-endian float.
    uint32_t floatBits =
        static_cast<uint32_t>(data[2]) |
        (static_cast<uint32_t>(data[3]) << 8) |
        (static_cast<uint32_t>(data[4]) << 16) |
        (static_cast<uint32_t>(data[5]) << 24);

    float beta;
    memcpy(&beta, &floatBits, sizeof(float));

    // Reject invalid floating-point values.
    if (!isfinite(beta))
        return false;

    packet.protocolVersion = version;
    packet.commandId = static_cast<CommandId>(commandId);
    packet.betaValue = beta;

    return true;
}
