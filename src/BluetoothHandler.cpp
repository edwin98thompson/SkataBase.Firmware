#include "BluetoothHandler.h"
#include <string>

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
      deviceName(deviceName),
      initialized(false),
      deviceConnected(false),
      commandDecoder()
{
}

void BluetoothHandler::begin()
{
    Serial.println("Initialising BLE...");

    // Explicitly initialise BLE
    BLEDevice::init(std::string(deviceName.c_str()));

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

    std::string payload(
        reinterpret_cast<const char*>(data),
        length
    );

    pCharacteristic->setValue(payload);
    pCharacteristic->notify();
}

bool BluetoothHandler::isConnected() const
{
    return deviceConnected;
}

bool BluetoothHandler::takeCommand(CommandPacket& packet)
{
    const uint8_t tail = queueTail.load(std::memory_order_relaxed);
    if (tail == queueHead.load(std::memory_order_acquire))
        return false;

    packet = commandQueue[tail];
    queueTail.store((tail + 1) % COMMAND_QUEUE_SIZE, std::memory_order_release);
    return true;
}

void BluetoothHandler::enqueueDecodedCommand(const CommandPacket& packet, void* context)
{
    static_cast<BluetoothHandler*>(context)->enqueueCommand(packet);
}

void BluetoothHandler::enqueueCommand(const CommandPacket& packet)
{
    const uint8_t head = queueHead.load(std::memory_order_relaxed);
    const uint8_t next = (head + 1) % COMMAND_QUEUE_SIZE;
    if (next == queueTail.load(std::memory_order_acquire))
    {
        Serial.println("Command queue full; dropping command.");
        return;
    }

    commandQueue[head] = packet;
    queueHead.store(next, std::memory_order_release);
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
    std::string value = characteristic->getValue();

    if (value.empty())
    {
        return;
    }

    parent->commandDecoder.feed(
        reinterpret_cast<const uint8_t*>(value.data()),
        value.size(),
        BluetoothHandler::enqueueDecodedCommand,
        parent
    );
}
