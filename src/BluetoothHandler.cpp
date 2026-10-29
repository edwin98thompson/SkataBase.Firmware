#include "BluetoothHandler.h"

#include <string>

namespace
{
    constexpr char SERVICE_UUID[] =
        "12345678-1234-1234-1234-1234567890ab";

    constexpr char CHARACTERISTIC_UUID[] =
        "abcd1234-ab12-cd34-ef56-abcdef123456";
    constexpr char CALIBRATE_MAGNETOMETER_COMMAND[] = "CALIBRATE_MAG";
}

BluetoothHandler::BluetoothHandler(const String& deviceName)
    : pServer(nullptr),
      pService(nullptr),
      pCharacteristic(nullptr),
      deviceConnected(false),
      calibrationRequestPending(false),
      deviceName(deviceName),
      initialized(false)
{
}

void BluetoothHandler::begin()
{
    Serial.println("Initialising BLE...");

    // Explicitly initialise BLE before creating the server.
    if (!BLEDevice::init(deviceName))
    {
        Serial.println("ERROR: BLE initialisation failed");
        return;
    }

    initialized = true;

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristic->setCallbacks(
        new CharacteristicCallbacks(this));

    pService->start();

    pServer->getAdvertising()->start();

    Serial.println("BLE advertising started");
}

void BluetoothHandler::sendPacket(
    const uint8_t* data,
    size_t length)
{
    if (!deviceConnected ||
        pCharacteristic == nullptr ||
        data == nullptr ||
        length == 0)
    {
        return;
    }

    pCharacteristic->setValue(data, length);
    pCharacteristic->notify();
}

bool BluetoothHandler::isConnected() const
{
    return deviceConnected;
}

bool BluetoothHandler::takeMagnetometerCalibrationRequest()
{
    return calibrationRequestPending.exchange(false);
}

// ----- Server callbacks -----

BluetoothHandler::ServerCallbacks::ServerCallbacks(
    BluetoothHandler* parent)
    : parent(parent)
{
}

void BluetoothHandler::ServerCallbacks::onConnect(
    BLEServer* pServer)
{
    parent->deviceConnected = true;

    Serial.println("BLE client connected");
}

void BluetoothHandler::ServerCallbacks::onDisconnect(
    BLEServer* pServer)
{
    parent->deviceConnected = false;

    Serial.println("BLE client disconnected");

    // Allow the Android app to reconnect.
    pServer->startAdvertising();
}

BluetoothHandler::CharacteristicCallbacks::CharacteristicCallbacks(
    BluetoothHandler* parent)
    : parent(parent)
{
}

/// @brief Handle write events to the BLE characteristic. If the received command is "CALIBRATE_MAG", the parent BluetoothHandler's calibrationRequestPending flag is set to true.
/// @param characteristic The BLE characteristic that received the write event.
void BluetoothHandler::CharacteristicCallbacks::onWrite(
    BLECharacteristic* characteristic)
{
    const std::string command = characteristic->getValue();
    // The Android app writes this UTF-8 command to the IMU characteristic.
    if (command == CALIBRATE_MAGNETOMETER_COMMAND)
    {
        parent->calibrationRequestPending.store(true);
    }
}