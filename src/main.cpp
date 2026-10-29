#include <Arduino.h>
#include "ImuPacket.h"
#include "ImuPacketEncoder.h"
#include "BluetoothHandler.h"
#include "ImuManager.h"
#include "CalibrationHandler.h"
#include "MotionFilter.h"

uint16_t sequence = 0;

const char* DEVICE_NAME = "ESP32_SkateIMU";
BluetoothHandler bleHandler;

ImuManager imuManager;
CalibrationHandler calibrationHandler;

constexpr float FILTER_RATE = 228.0f;
constexpr float BETA = 0.08f;

MotionFilter motionFilter;

// Timing Variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 20; // 50Hz transmission rate

/// @brief Arduino setup function to initialize the IMU, motion filter, and BLE
void setup()
{
    Serial.begin(115200);
    Serial.println("🔌 Starting...");

    if (!imuManager.begin())
    {
        Serial.println("IMU initialisation failed.");
        while (true)
        {
            delay(1000);
        }
    }

    if (!calibrationHandler.begin())
    {
        Serial.println("Failed to load calibration data from NVS.");
    }

    calibrationHandler.calibrateGyroscope(imuManager);
    calibrationHandler.calibrateAccelerometer(imuManager);

    // Implement motion filter
    motionFilter = MotionFilter(FILTER_RATE, BETA);

    // BLE setup
    bleHandler = BluetoothHandler(DEVICE_NAME);
    bleHandler.begin();
}

/// @brief Main loop for sampling IMU data and sending it over BLE
void loop()
{
    // Check for magnetometer calibration request from BLE
    if (bleHandler.takeMagnetometerCalibrationRequest())
    {
        if (calibrationHandler.startMagnetometerCalibration())
        {
            Serial.println(
                "Magnetometer calibration started. Rotate the IMU "
                "through all orientations for 30 seconds.");
        }
        else
        {
            Serial.println(
                "Magnetometer calibration request rejected.");
        }
    }

    calibrationHandler.updateMagnetometerCalibration(nullptr);

    ImuPacket packet;
    uint8_t buffer[IMU_PACKET_SIZE];

    if (imuManager.readSample(packet))
    {
        // Update magnetometer calibration if in progress and apply any calibrations
        // to the packet before sending it over BLE.
        calibrationHandler.updateMagnetometerCalibration(&packet);
        calibrationHandler.apply(packet);

        // Apply madgwick filter to the packet to compute orientation
        motionFilter.computeOrientation(
            packet,
            calibrationHandler.isMagnetometerCalibrated());

        // Send the packet over BLE at the specified interval
        if (millis() - lastSendTime >= sendInterval)
        {
            packet.sequence = sequence++;
            packet.timestampMs = millis();

            if (encodeImuPacket(packet, buffer, sizeof(buffer)))
            {
                bleHandler.sendPacket(buffer, sizeof(buffer));
                lastSendTime = millis();
            }
        }
    }
}