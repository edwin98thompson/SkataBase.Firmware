#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h> // Include the Madgwick Filter Library
#include "ImuPacket.h"
#include "ImuPacketEncoder.h"
#include "BluetoothHandler.h"
#include "ImuManager.h"

// I2C pins
#define SDA_PIN 22
#define SCL_PIN 23

// BLE + IMU
ICM_20948_I2C myIMU;
Madgwick filter;

uint16_t sequence = 0;

BluetoothHandler bleHandler("ESP32_SkateIMU");

ImuManager imuManager;

// Timing Variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 20; // 50Hz transmission rate
constexpr float FILTER_RATE = 228.0f;

// Calibration Structures

struct AccelCalibration
{
    float offsetX;
    float offsetY;
    float offsetZ;

    float scaleX;
    float scaleY;
    float scaleZ;
};

AccelCalibration accel =
    {
        0, 0, 0,
        1, 1, 1};

struct MagCalibration
{
    float offsetX;
    float offsetY;
    float offsetZ;
};

MagCalibration mag =
    {
        0,
        0,
        0};

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

int gyroCalSamples = 1000;

float accelBiasX = 0.0;
float accelBiasY = 0.0;
float accelBiasZ = 0.0;

int accelCalSamples = 1000;

/// @brief Calibrates the gyroscope by averaging a number of samples
void getGyroBias()
{
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    Serial.println("🕒 Calibrating gyroscope...");

    for (int i = 0; i < gyroCalSamples; i++)
    {
        if (myIMU.dataReady())
        {
            myIMU.getAGMT();
            sumX += myIMU.agmt.gyr.axes.x;
            sumY += myIMU.agmt.gyr.axes.y;
            sumZ += myIMU.agmt.gyr.axes.z;
        }
        // TODO - Add a check to ensure the sensor is stable before taking readings
        delay(5); // Small delay to allow sensor to update
    }

    gyroBiasX = sumX / gyroCalSamples;
    gyroBiasY = sumY / gyroCalSamples;
    gyroBiasZ = sumZ / gyroCalSamples;

    Serial.print("✅ Gyro Bias X: ");
    Serial.println(gyroBiasX);
    Serial.print("✅ Gyro Bias Y: ");
    Serial.println(gyroBiasY);
    Serial.print("✅ Gyro Bias Z: ");
    Serial.println(gyroBiasZ);
}

/// @brief Calibrates the accelerometer by averaging a number of samples
void getAccelBias()
{
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    Serial.println("🕒 Calibrating accelerometer...");

    for (int i = 0; i < accelCalSamples; i++)
    {
        if (myIMU.dataReady())
        {
            myIMU.getAGMT();
            sumX += myIMU.agmt.acc.axes.x;
            sumY += myIMU.agmt.acc.axes.y;
            sumZ += myIMU.agmt.acc.axes.z;
        }
        // TODO - Add a check to ensure the sensor is stable before taking readings
        delay(5); // Small delay to allow sensor to update
    }

    accelBiasX = sumX / accelCalSamples;
    accelBiasY = sumY / accelCalSamples;
    accelBiasZ = sumZ / accelCalSamples;

    Serial.print("✅ Accel Bias X: ");
    Serial.println(accelBiasX);
    Serial.print("✅ Accel Bias Y: ");
    Serial.println(accelBiasY);
    Serial.print("✅ Accel Bias Z: ");
    Serial.println(accelBiasZ);
}


/// @brief Arduino setup function to initialize Serial, I2C, IMU, and BLE
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

    // Implement calibration class and apply

    // Madgwick filter setup
    filter.begin(FILTER_RATE); // 228Hz update rate
    filter.beta = 0.08f;        // larger beta = more responsive, smaller beta = more stable

    // BLE setup
    bleHandler.begin();
}

/// @brief Main loop for sampling IMU data and sending it over BLE
void loop()
{
    static uint16_t sequence = 0;

    ImuPacket packet;
    uint8_t buffer[IMU_PACKET_SIZE];

    if (imuManager.readSample(packet))
    {
        // Apply madgwick filter to the packet to compute orientation
        // todo - filter class needed
        filter.updateIMU(
        packet.gyroX,
        packet.gyroY,
        packet.gyroZ,
        packet.accelX,
        packet.accelY,
        packet.accelZ);

        packet.roll = filter.getRoll();
        packet.pitch = filter.getPitch();
        packet.yaw = filter.getYaw();
        packet.q0 = filter.q0;
        packet.q1 = filter.q1;
        packet.q2 = filter.q2;
        packet.q3 = filter.q3;

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