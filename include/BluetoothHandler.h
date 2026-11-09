#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include "CommandDecoder.h"
#include "CommandPacket.h"

#include <stddef.h>
#include <stdint.h>
#include <atomic>

class BluetoothHandler
{
public:
    explicit BluetoothHandler(const String& deviceName);

    void begin();

    // Send binary data over BLE notifications.
    void sendPacket(const uint8_t* data, size_t length);

    bool isConnected() const;
    bool takeCommand(CommandPacket& packet);

private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pCharacteristic;
    String deviceName;
    bool initialized;
    bool deviceConnected;
    static constexpr uint8_t COMMAND_QUEUE_SIZE = 8;
    CommandPacket commandQueue[COMMAND_QUEUE_SIZE] = {};
    std::atomic<uint8_t> queueHead{0};
    std::atomic<uint8_t> queueTail{0};
    CommandDecoder commandDecoder;

    static void enqueueDecodedCommand(const CommandPacket& packet, void* context);
    void enqueueCommand(const CommandPacket& packet);

    class ServerCallbacks : public BLEServerCallbacks
    {
    public:
        explicit ServerCallbacks(BluetoothHandler* parent);

        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;

    private:
        BluetoothHandler* parent;
    };

    class CharacteristicCallbacks : public BLECharacteristicCallbacks
    {
    public:
        explicit CharacteristicCallbacks(BluetoothHandler* parent);

        void onWrite(BLECharacteristic* characteristic) override;

    private:
        BluetoothHandler* parent;
    };
};
