#include "BluetoothHandler.h"
#include <Arduino.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-abcdef123456"

BluetoothHandler::BluetoothHandler(const std::string& deviceName)
: pServer(nullptr), pService(nullptr), pCharacteristic(nullptr), deviceConnected(false) {
    BLEDevice::init(deviceName);
}

void BluetoothHandler::begin() {
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );

    pCharacteristic->setValue("Hello Android");
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    pServer->getAdvertising()->start();
}

void BluetoothHandler::updateValue(const std::string& value) {
    if (deviceConnected) {
        pCharacteristic->setValue(value);
        pCharacteristic->notify();  // If notify is needed
    }
}

bool BluetoothHandler::isConnected() const {
    return deviceConnected;
}

// ----- Server Callbacks -----
BluetoothHandler::ServerCallbacks::ServerCallbacks(BluetoothHandler* parent)
: parent(parent) {}

void BluetoothHandler::ServerCallbacks::onConnect(BLEServer* pServer) {
    parent->deviceConnected = true;
    Serial.println("Client connected");
}

void BluetoothHandler::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    parent->deviceConnected = false;
    Serial.println("Client disconnected");
    pServer->startAdvertising();
}
