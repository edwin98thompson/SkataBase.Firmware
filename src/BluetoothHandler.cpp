#include "BluetoothHandler.h"
#include "ImuPacketEncoder.h"

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
    BLEDevice::init(deviceName);
    const esp_err_t mtuStatus = BLEDevice::setMTU(185);
    Serial.printf("BLE local MTU: %u (set result %d)\n",
                  static_cast<unsigned>(BLEDevice::getMTU()),
                  static_cast<int>(mtuStatus));

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
    uint8_t* data,
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

    const uint16_t connectionId = pServer->getConnId();
    const uint16_t peerMtu = pServer->getPeerMTU(connectionId);
    const uint16_t maxNotifySize = peerMtu > 3 ? peerMtu - 3 : 0;
    Serial.printf("[BLE] connected: conn=%u peerMTU=%u maxNotify=%u packet=%u %s",
                  static_cast<unsigned>(connectionId),
                  static_cast<unsigned>(peerMtu),
                  static_cast<unsigned>(maxNotifySize),
                  static_cast<unsigned>(IMU_PACKET_SIZE),
                  maxNotifySize >= IMU_PACKET_SIZE ? "fits" : "too small");
    Serial.println();
}

void BluetoothHandler::ServerCallbacks::onMtuChanged(
    BLEServer* pServer,
    esp_ble_gatts_cb_param_t* param)
{
    const uint16_t peerMtu = param->mtu.mtu;
    const uint16_t maxNotifySize = peerMtu > 3 ? peerMtu - 3 : 0;
    Serial.printf("[BLE] MTU negotiated: conn=%u peerMTU=%u maxNotify=%u packet=%u %s",
                  static_cast<unsigned>(param->mtu.conn_id),
                  static_cast<unsigned>(peerMtu),
                  static_cast<unsigned>(maxNotifySize),
                  static_cast<unsigned>(IMU_PACKET_SIZE),
                  maxNotifySize >= IMU_PACKET_SIZE ? "fits" : "too small");
    Serial.println();
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
    auto value = characteristic->getValue();

    if (value.length() == 0)
    {
        return;
    }

    parent->commandDecoder.feed(
        reinterpret_cast<const uint8_t*>(value.c_str()),
        value.length(),
        BluetoothHandler::enqueueDecodedCommand,
        parent
    );
}
