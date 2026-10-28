#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>

#include <stddef.h>
#include <stdint.h>

class BluetoothHandler
{
public:
    explicit BluetoothHandler(const String& deviceName);

    void begin();

    // Send binary data over BLE notifications.
    void sendPacket(const uint8_t* data, size_t length);

    bool isConnected() const;

private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pCharacteristic;
    String deviceName;
    bool initialized;
    bool deviceConnected;

    class ServerCallbacks : public BLEServerCallbacks
    {
    public:
        explicit ServerCallbacks(BluetoothHandler* parent);

        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;

    private:
        BluetoothHandler* parent;
    };
};