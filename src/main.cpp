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

/// @brief Samples IMU data and populates the provided packet
/// @param packet Reference to the IMuPacket to populate
/// @return true if data was successfully sampled, false otherwise
bool sampleIMU(ImuPacket& packet)
{

    if (!myIMU.dataReady())
    {
        return false;
    }

    myIMU.getAGMT(); // Read raw physical registers

    // calculate acc to 3 dp
    const float ax = (myIMU.agmt.acc.axes.x - accelBiasX) / 8192.0f;
    const float ay = (myIMU.agmt.acc.axes.y - accelBiasY) / 8192.0f;
    const float az = (myIMU.agmt.acc.axes.z - accelBiasZ) / 8192.0f;

    // // Gyro: Convert raw to Degrees Per Second (Default scale is 131 LSB/dps)
    float gx = (myIMU.agmt.gyr.axes.x - gyroBiasX) / 16.4f;
    float gy = (myIMU.agmt.gyr.axes.y - gyroBiasY) / 16.4f;
    float gz = (myIMU.agmt.gyr.axes.z - gyroBiasZ) / 16.4f;

    // // Magnetometer: Raw microteslas (µT) work fine directly
    float mx = myIMU.agmt.mag.axes.x;
    float my = myIMU.agmt.mag.axes.y;
    float mz = myIMU.agmt.mag.axes.z;

    float temp = myIMU.agmt.tmp.val / 333.87f + 21.0f; // Convert raw temp to Celsius

    // print data
    // Serial.print(ax, 3); Serial.print(", ");
    // Serial.print(ay, 3); Serial.print(", ");
    // Serial.print(az, 3); Serial.print(", ");

    // Serial.print(gx, 3); Serial.print(", ");
    // Serial.print(gy, 3); Serial.print(", ");
    // Serial.print(gz, 3); Serial.print(", ");

    // Serial.print(mx, 3); Serial.print(", ");
    // Serial.print(my, 3); Serial.print(", ");
    // Serial.print(mz, 3); Serial.print(", ");
    // Serial.println();

    filter.updateIMU(
        gx,
        gy,
        gz,
        ax,
        ay,
        az);

    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();

    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.accelX = ax;
    packet.accelY = ay;
    packet.accelZ = az;
    packet.gyroX = gx;
    packet.gyroY = gy;
    packet.gyroZ = gz;
    packet.magX = mx;
    packet.magY = my;
    packet.magZ = mz;
    packet.q0 = filter.q0;
    packet.q1 = filter.q1;
    packet.q2 = filter.q2;
    packet.q3 = filter.q3;
    packet.temperatureC = temp;

    // Monitor the actual sample rate once per second
    static uint32_t samples = 0;
    static uint32_t lastPrint = millis();

    samples++;

    uint32_t now = millis();
    if (now - lastPrint >= 1000)
    {
        Serial.printf("Actual IMU rate: %lu Hz\n", samples);
        samples = 0;
        lastPrint = now;

        Serial.printf(
            "Roll: %.2f Pitch: %.2f Yaw: %.2f\n",
            roll,
            pitch,
            yaw);

        Serial.printf(
            "q0: %.2f q1: %.2f q2: %.2f q3: %.2f\n",
            filter.q0,
            filter.q1,
            filter.q2,
            filter.q3);
    }

    // Serial.printf(
    //     "Roll: %.2f Pitch: %.2f Yaw: %.2f\n",
    //     roll,
    //     pitch,
    //     yaw
    // );

    // Serial.printf(
    //     "q0: %.2f q1: %.2f q2: %.2f q3: %.2f\n",
    //     filter.q0,
    //     filter.q1,
    //     filter.q2,
    //     filter.q3
    // );

    return true;
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
        // refer to sampleIMU() for how to use the filter


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