#include "ImuManager.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

// I2C pins for the Seeed XIAO ESP32-C6
namespace
{
    // The Seeed XIAO ESP32-C6 has the following I2C pinout:
    // constexpr uint8_t SDA_PIN = 22;
    // constexpr uint8_t SCL_PIN = 23;

    // Esp32 devkit v1 has the following I2C pinout:
    constexpr uint8_t SDA_PIN = 21;
    constexpr uint8_t SCL_PIN = 22;

    constexpr uint32_t I2C_CLOCK_HZ = 400000;

    // Sensor configuration
    constexpr uint8_t ACCEL_SAMPLE_RATE = 4;
    constexpr uint8_t GYRO_SAMPLE_RATE = 4;

    // Conversion factors retained from the original main.cpp.
    constexpr float ACCEL_SCALE = 8192.0f;
    constexpr float GYRO_SCALE = 16.4f;
}

/*

bool ImuManager::begin()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n==================================");
    Serial.println("     ICM-20948 I2C DIAGNOSTICS");
    Serial.println("==================================");

    Serial.printf("SDA GPIO: %d\n", SDA_PIN);
    Serial.printf("SCL GPIO: %d\n", SCL_PIN);

    // Initialise I2C once.
    Serial.println("\n[1] Initialising I2C...");

    if (!Wire.begin(SDA_PIN, SCL_PIN))
    {
        Serial.println("ERROR: Wire.begin() failed");
        return false;
    }

    Wire.setClock(100000);
    delay(100);

    Serial.println("I2C initialised.");
    Serial.printf("I2C clock: %lu Hz\n", 100000UL);

    // Continuously diagnose the bus.
    while (true)
    {
        Serial.println("\n----------------------------------");
        Serial.printf("Diagnostic time: %lu ms\n", millis());

        // 1. Scan the I2C bus.
        Serial.println("\n[2] Scanning I2C bus...");

        int deviceCount = 0;

        for (uint8_t address = 1; address < 127; address++)
        {
            Wire.beginTransmission(address);
            uint8_t error = Wire.endTransmission();

            if (error == 0)
            {
                Serial.printf("  DEVICE FOUND: 0x%02X\n", address);
                deviceCount++;
            }
            else if (error != 2)
            {
                Serial.printf(
                    "  Address 0x%02X: I2C error %u\n",
                    address,
                    error
                );
            }
        }

        Serial.printf("Devices found: %d\n", deviceCount);

        // 2. Check both possible ICM-20948 addresses.
        for (uint8_t address : {0x68, 0x69})
        {
            Serial.printf("\n[3] Testing address 0x%02X\n", address);

            Wire.beginTransmission(address);
            uint8_t txError = Wire.endTransmission(false);

            Serial.printf("  Register selection result: %u\n", txError);

            if (txError != 0)
            {
                Serial.println("  Device did not acknowledge.");
                continue;
            }

            Wire.write((uint8_t)0x00); // WHO_AM_I register

            uint8_t received = Wire.requestFrom(address, (uint8_t)1);

            Serial.printf("  Bytes received: %u\n", received);

            if (received == 1 && Wire.available())
            {
                uint8_t whoAmI = Wire.read();

                Serial.printf("  WHO_AM_I: 0x%02X\n", whoAmI);

                if (whoAmI == 0xEA)
                {
                    Serial.println("  SUCCESS: ICM-20948 identified!");
                }
                else
                {
                    Serial.println("  WARNING: Unexpected device ID.");
                }
            }
            else
            {
                Serial.println("  ERROR: Failed to read WHO_AM_I.");
            }
        }

        // 3. Attempt SparkFun library initialisation.
        Serial.println("\n[4] Testing SparkFun library...");

        // false = AD0 LOW, address 0x68.
        // true  = AD0 HIGH, address 0x69.
        imu.begin(Wire, false);

        Serial.printf(
            "  SparkFun status: %s\n",
            imu.statusString()
        );

        if (imu.status == ICM_20948_Stat_Ok)
        {
            Serial.println("\nIMU INITIALISED SUCCESSFULLY!");

            if (!configure())
            {
                Serial.println("Sensor configuration failed.");
                return false;
            }

            Serial.println("Starting magnetometer...");
            imu.startupMagnetometer();

            Serial.println("IMU initialisation complete.");
            return true;
        }

        Serial.println("  IMU initialisation failed.");

        // 4. Bus voltage / recovery guidance.
        Serial.println("\n[5] Diagnostic summary");
        Serial.println("  Check SDA/SCL wiring.");
        Serial.println("  Check 3.3V supply.");
        Serial.println("  Check I2C pull-up resistors.");
        Serial.println("  Check device address.");

        Serial.println("\nRetrying in 3 seconds...");
        delay(3000);
    }

    return false;
}

*/


bool ImuManager::begin()
{
    Serial.println("Initialising IMU...");

    // Initialise I2C.
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    // Initialise the ICM-20948.
    // false selects the default I2C address, normally 0x69.
    while (true)
    {
        imu.begin(Wire, false);

        if (imu.status == ICM_20948_Stat_Ok)
        {
            break;
        }

        Serial.print("Failed to initialise ICM-20948: ");
        Serial.println(imu.statusString());

        Serial.println("Retrying in 500 ms...");
        delay(500);
    }

    Serial.println("ICM-20948 detected.");

    // Configure sensor ranges, sample rates, and filters.
    if (!configure())
    {
        Serial.println("Failed to configure ICM-20948.");
        return false;
    }

    // Initialise the magnetometer.
    imu.startupMagnetometer();

    Serial.println("IMU initialisation complete.");

    return true;
}

bool ImuManager::configure()
{
    // Configure the gyroscope for +/- 2000 degrees per second.
    ICM_20948_fss_t gyroFSS;
    gyroFSS.a = 0; // Unused for gyroscope.
    gyroFSS.g = ICM_20948_GYRO_CONFIG_1_FS_SEL_e::dps2000;

    if (imu.setFullScale(ICM_20948_Internal_Gyr, gyroFSS)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to configure gyroscope range.");
        return false;
    }

    // Configure the accelerometer for +/- 4 g.
    ICM_20948_fss_t accelFSS;
    accelFSS.a = ICM_20948_ACCEL_CONFIG_FS_SEL_e::gpm4;
    accelFSS.g = 0; // Unused for accelerometer.

    if (imu.setFullScale(ICM_20948_Internal_Acc, accelFSS)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to configure accelerometer range.");
        return false;
    }

    // Configure accelerometer and gyroscope sample rates.
    ICM_20948_smplrt_t sampleRate;
    sampleRate.a = ACCEL_SAMPLE_RATE;
    sampleRate.g = GYRO_SAMPLE_RATE;

    if (imu.setSampleRate(ICM_20948_Internal_Acc, sampleRate)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to configure accelerometer sample rate.");
        return false;
    }

    if (imu.setSampleRate(ICM_20948_Internal_Gyr, sampleRate)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to configure gyroscope sample rate.");
        return false;
    }

    // Configure digital low-pass filters.
    ICM_20948_dlpcfg_t dlpf;
    dlpf.g = gyr_d119bw5_n154bw3;
    dlpf.a = acc_d111bw4_n136bw;

    imu.enableDLPF(ICM_20948_Internal_Gyr, true);
    imu.enableDLPF(ICM_20948_Internal_Acc, true);

    if (imu.setDLPFcfg(ICM_20948_Internal_Gyr, dlpf)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to configure gyroscope DLPF.");
        return false;
    }

    if (imu.setDLPFcfg(ICM_20948_Internal_Acc, dlpf)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to configure accelerometer DLPF.");
        return false;
    }

    Serial.println("ICM-20948 configuration complete.");

    return true;
}

bool ImuManager::readSample(ImuPacket& packet)
{
    // Return immediately when no fresh sample is available.
    if (!imu.dataReady())
    {
        return false;
    }

    // Read the accelerometer, gyroscope, magnetometer, and temperature.
    imu.getAGMT();

    // Accelerometer: convert to g.
    const float ax = imu.agmt.acc.axes.x / ACCEL_SCALE;
    const float ay = imu.agmt.acc.axes.y / ACCEL_SCALE;
    const float az = imu.agmt.acc.axes.z / ACCEL_SCALE;

    // Gyroscope: convert to degrees per second.
    const float gx = imu.agmt.gyr.axes.x / GYRO_SCALE;
    const float gy = imu.agmt.gyr.axes.y / GYRO_SCALE;
    const float gz = imu.agmt.gyr.axes.z / GYRO_SCALE;

    // Magnetometer readings.
    const float mx = imu.agmt.mag.axes.x;
    const float my = imu.agmt.mag.axes.y;
    const float mz = imu.agmt.mag.axes.z;

    // Temperature in degrees Celsius.
    const float temperatureC =
        imu.agmt.tmp.val / 333.87f + 21.0f;

    // Populate the existing packet's sensor fields.
    // Orientation and quaternion fields are handled by the orientation
    // filter, while sequence and timestamp are handled by main.cpp.
    packet.accelX = ax;
    packet.accelY = ay;
    packet.accelZ = az;

    packet.gyroX = gx;
    packet.gyroY = gy;
    packet.gyroZ = gz;

    packet.magX = mx;
    packet.magY = my;
    packet.magZ = mz;

    packet.temperatureC = temperatureC;

    return true;
}