#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BluetoothHandler {
public:
    BluetoothHandler(const String& deviceName);
    void begin();
    void updateValue(const String& value);
    bool isConnected() const;

private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pCharacteristic;
    bool deviceConnected;

    class ServerCallbacks : public BLEServerCallbacks {
    public:
        ServerCallbacks(BluetoothHandler* parent);
        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;

    private:
        BluetoothHandler* parent;
    };
};

#endif
