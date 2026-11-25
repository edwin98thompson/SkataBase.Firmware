#include "ImuManager.h"

#include <Arduino.h>
#include <SPI.h>

namespace
{
    // XIAO ESP32-C6: D8/GPIO19=SCK, D9/GPIO20=MISO, D10/GPIO18=MOSI.
    // D3/GPIO21 is connected to the IMU CS/NCS pin.
    constexpr uint8_t SPI_SCK_PIN = 19;
    constexpr uint8_t SPI_MISO_PIN = 20;
    constexpr uint8_t SPI_MOSI_PIN = 18;
    constexpr uint8_t SPI_CS_PIN = 21;
    constexpr uint32_t SPI_FREQUENCY_HZ = 1000000;

    constexpr uint8_t ACCEL_SAMPLE_RATE = 4;
    constexpr uint8_t GYRO_SAMPLE_RATE = 4;

    // The configured full-scale ranges are +/-4 g and +/-2000 dps.
    constexpr float ACCEL_SCALE = 8192.0f;
    constexpr float GYRO_SCALE = 16.4f;
    constexpr uint8_t EXPECTED_WHO_AM_I = 0xEA;
}

bool ImuManager::begin()
{
    delay(1500);
    Serial.println("[IMU] Initialising ICM-20948 over SPI...");
    Serial.printf(
        "[IMU] SPI pins SCK=%u MISO=%u MOSI=%u CS=%u at %lu Hz\n",
        static_cast<unsigned>(SPI_SCK_PIN),
        static_cast<unsigned>(SPI_MISO_PIN),
        static_cast<unsigned>(SPI_MOSI_PIN),
        static_cast<unsigned>(SPI_CS_PIN),
        static_cast<unsigned long>(SPI_FREQUENCY_HZ));

    if (!spiStarted)
    {
        pinMode(SPI_CS_PIN, OUTPUT);
        digitalWrite(SPI_CS_PIN, HIGH);
        SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
        spiStarted = true;
        delay(10);
    }

    ICM_20948_Status_e beginStatus =
        imu.begin(SPI_CS_PIN, SPI, SPI_FREQUENCY_HZ);
    uint8_t whoAmI = imu.getWhoAmI();

    Serial.printf(
        "[IMU] SPI begin: %s; WHO_AM_I=0x%02X\n",
        imu.statusString(beginStatus),
        static_cast<unsigned>(whoAmI));

    // The standalone SPI test occasionally needed one short retry during
    // startup. Keep that short recovery in the SPI path.
    if (beginStatus != ICM_20948_Stat_Ok || whoAmI != EXPECTED_WHO_AM_I)
    {
        Serial.println("[IMU] SPI init did not verify; retrying once...");
        delay(100);
        beginStatus = imu.begin(SPI_CS_PIN, SPI, SPI_FREQUENCY_HZ);
        whoAmI = imu.getWhoAmI();

        Serial.printf(
            "[IMU] SPI retry: %s; WHO_AM_I=0x%02X\n",
            imu.statusString(beginStatus),
            static_cast<unsigned>(whoAmI));
    }

    if (beginStatus != ICM_20948_Stat_Ok || whoAmI != EXPECTED_WHO_AM_I)
    {
        Serial.println("[IMU] ICM-20948 SPI initialisation failed.");
        return false;
    }

    Serial.println("[IMU] ICM-20948 detected.");

    if (!configure())
    {
        Serial.println("[IMU] ICM-20948 configuration failed.");
        return false;
    }

    const ICM_20948_Status_e magStatus = imu.startupMagnetometer();
    if (magStatus != ICM_20948_Stat_Ok)
    {
        Serial.printf(
            "[IMU] Magnetometer initialisation failed: %s\n",
            imu.statusString(magStatus));
        return false;
    }

    Serial.println("[IMU] Initialisation complete.");
    return true;
}

bool ImuManager::configure()
{
    ICM_20948_fss_t gyroFSS;
    gyroFSS.a = 0;
    gyroFSS.g = ICM_20948_GYRO_CONFIG_1_FS_SEL_e::dps2000;

    if (imu.setFullScale(ICM_20948_Internal_Gyr, gyroFSS)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to configure gyroscope range.");
        return false;
    }

    ICM_20948_fss_t accelFSS;
    accelFSS.a = ICM_20948_ACCEL_CONFIG_FS_SEL_e::gpm4;
    accelFSS.g = 0;

    if (imu.setFullScale(ICM_20948_Internal_Acc, accelFSS)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to configure accelerometer range.");
        return false;
    }

    ICM_20948_smplrt_t sampleRate;
    sampleRate.a = ACCEL_SAMPLE_RATE;
    sampleRate.g = GYRO_SAMPLE_RATE;

    if (imu.setSampleRate(ICM_20948_Internal_Acc, sampleRate)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to configure accelerometer sample rate.");
        return false;
    }

    if (imu.setSampleRate(ICM_20948_Internal_Gyr, sampleRate)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to configure gyroscope sample rate.");
        return false;
    }

    ICM_20948_dlpcfg_t dlpf;
    dlpf.g = gyr_d119bw5_n154bw3;
    dlpf.a = acc_d111bw4_n136bw;

    if (imu.enableDLPF(ICM_20948_Internal_Gyr, true)
            != ICM_20948_Stat_Ok
        || imu.enableDLPF(ICM_20948_Internal_Acc, true)
            != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to enable digital low-pass filters.");
        return false;
    }

    if (imu.setDLPFcfg(ICM_20948_Internal_Gyr, dlpf)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to configure gyroscope DLPF.");
        return false;
    }

    if (imu.setDLPFcfg(ICM_20948_Internal_Acc, dlpf)
        != ICM_20948_Stat_Ok)
    {
        Serial.println("[IMU] Failed to configure accelerometer DLPF.");
        return false;
    }

    Serial.println("[IMU] Sensor ranges, sample rates, and filters configured.");
    return true;
}

bool ImuManager::readSample(ImuPacket& packet)
{
    if (!imu.dataReady())
    {
        if (imu.status != ICM_20948_Stat_NoData)
        {
            static uint32_t lastErrorLogMs = 0;
            const uint32_t nowMs = millis();
            if (nowMs - lastErrorLogMs >= 1000)
            {
                Serial.printf(
                    "[IMU] dataReady failed: %s\n",
                    imu.statusString());
                lastErrorLogMs = nowMs;
            }
        }

        delay(1);
        return false;
    }

    imu.getAGMT();
    if (imu.status != ICM_20948_Stat_Ok)
    {
        static uint32_t lastErrorLogMs = 0;
        const uint32_t nowMs = millis();
        if (nowMs - lastErrorLogMs >= 1000)
        {
            Serial.printf("[IMU] sample read failed: %s\n", imu.statusString());
            lastErrorLogMs = nowMs;
        }
        return false;
    }

    packet.accelX = imu.agmt.acc.axes.x / ACCEL_SCALE;
    packet.accelY = imu.agmt.acc.axes.y / ACCEL_SCALE;
    packet.accelZ = imu.agmt.acc.axes.z / ACCEL_SCALE;
    packet.gyroX = imu.agmt.gyr.axes.x / GYRO_SCALE;
    packet.gyroY = imu.agmt.gyr.axes.y / GYRO_SCALE;
    packet.gyroZ = imu.agmt.gyr.axes.z / GYRO_SCALE;
    packet.magX = imu.agmt.mag.axes.x;
    packet.magY = imu.agmt.mag.axes.y;
    packet.magZ = imu.agmt.mag.axes.z;
    packet.temperatureC = imu.agmt.tmp.val / 333.87f + 21.0f;

    return true;
}
