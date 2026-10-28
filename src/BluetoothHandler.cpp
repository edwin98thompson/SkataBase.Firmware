#include "BluetoothHandler.h"

namespace
{
    constexpr char SERVICE_UUID[] =
        "12345678-1234-1234-1234-1234567890ab";

    constexpr char CHARACTERISTIC_UUID[] =
        "abcd1234-ab12-cd34-ef56-abcdef123456";
}

BluetoothHandler::BluetoothHandler(const String& deviceName)
    : pServer(nullptr),
      pService(nullptr),
      pCharacteristic(nullptr),
      deviceConnected(false),
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
        BLECharacteristic::PROPERTY_NOTIFY
    );

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