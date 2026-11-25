#include <Arduino.h>
#include "ImuPacket.h"
#include "ImuPacketEncoder.h"
#include "BluetoothHandler.h"
#include "ImuManager.h"
#include "CalibrationHandler.h"
#include "MotionFilter.h"
#include "CommandHandler.h"

uint16_t sequence = 0;

const char* DEVICE_NAME = "ESP32_SkateIMU";
BluetoothHandler bleHandler(DEVICE_NAME);

ImuManager imuManager;

CalibrationHandler calibrationHandler;

constexpr float FILTER_RATE = 228.0f;
// Madgwick filter parameter; higher values result in faster convergence but more noise.

// 0.08 for the XIAO ESP32-C6, 0.15 for the ESP32 DevKit v1

// needs experimentation to find the best value for your specific IMU and application.
MotionFilter motionFilter(FILTER_RATE, CommandHandler::DEFAULT_BETA);

CommandHandler commandHandler(calibrationHandler, imuManager, motionFilter);

// Timing Variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 20; //50Hz BLE congestion check

/// @brief Arduino setup function to initialize the IMU, motion filter, and BLE
void setup()
{
    Serial.begin(115200);
    Serial.println("Starting SkateData2026...");

    while (!imuManager.begin())
    {
        Serial.println("IMU initialisation failed.");

        delay(1000);
    }

    if (!calibrationHandler.begin())
    {
        Serial.println("Failed to load calibration data from NVS.");
    }

    commandHandler.begin();
    //calibrationHandler.calibrateGyroscope(imuManager);
    //calibrationHandler.calibrateAccelerometer(imuManager);

    bleHandler.begin();
}

/// @brief Main loop for sampling IMU data and sending it over BLE
void loop()
{
    // Process BLE commands on the main loop, outside the BLE callback task.
    CommandPacket command;
    while (bleHandler.takeCommand(command))
    {
        commandHandler.handle(command);
    }

    // If the magnetometer calibration is in progress, update it with a null sample,
    // this allows the method to check for timeout and update the calibration status accordingly.
    calibrationHandler.updateMagnetometerCalibration(nullptr);

    ImuPacket packet;
    packet.betaValue = motionFilter.beta;

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
            packet.betaValue = motionFilter.beta;

            if (encodeImuPacket(packet, buffer, sizeof(buffer)))
            {
                bleHandler.sendPacket(buffer, sizeof(buffer));
                lastSendTime = packet.timestampMs;
            }
        }
    }
}
