#include "CommandHandler.h"

#include <Arduino.h>
#include <math.h>

CommandHandler::CommandHandler(
    CalibrationHandler& calibration,
    ImuManager& imu,
    MotionFilter& motionFilter)
    : calibration(calibration),
      imu(imu),
      motionFilter(motionFilter)
{
}

bool CommandHandler::begin()
{
    storageReady = preferences.begin("imu-filter", false);
    if (!storageReady)
    {
        motionFilter.setBeta(DEFAULT_BETA);
        Serial.println("Could not open filter settings in NVS; using default beta.");
        return false;
    }

    const float savedBeta = preferences.getFloat("beta", DEFAULT_BETA);
    if (!isfinite(savedBeta) || savedBeta < 0.0f)
    {
        motionFilter.setBeta(DEFAULT_BETA);
        Serial.println("Saved filter beta is invalid; using default beta.");
        return false;
    }

    motionFilter.setBeta(savedBeta);
    Serial.printf("Loaded filter beta %.6f from NVS.\n", savedBeta);
    return true;
}

bool CommandHandler::handle(const CommandPacket& packet)
{
    if (packet.protocolVersion != 1)
    {
        Serial.printf("Unsupported command protocol version: %u\n", packet.protocolVersion);
        return false;
    }

    switch (packet.commandId)
    {
    case CMD_START_MAGNETOMETER_CALIBRATION:
        if (!calibration.startMagnetometerCalibration())
        {
            Serial.println("Magnetometer calibration request rejected.");
            return false;
        }
        Serial.println("Magnetometer calibration started. Rotate the IMU through all orientations for 30 seconds.");
        return true;

    case CMD_START_ACCELEROMETER_CALIBRATION:
        return calibration.calibrateAccelerometer(imu);

    case CMD_START_GYROSCOPE_CALIBRATION:
        return calibration.calibrateGyroscope(imu);

    case CMD_SET_BETA_VALUE:
        if (!isfinite(packet.betaValue) || packet.betaValue < 0.0f)
        {
            Serial.println("Rejected invalid filter beta value.");
            return false;
        }
        motionFilter.setBeta(packet.betaValue);
        if (!storageReady)
        {
            Serial.println("Filter beta updated for this session; NVS is unavailable, so it was not saved.");
            return false;
        }
        if (preferences.putFloat("beta", packet.betaValue) != sizeof(float))
        {
            Serial.println("Filter beta updated for this session, but saving it to NVS failed.");
            return false;
        }
        Serial.printf("Filter beta set to %.6f and saved to NVS.\n", packet.betaValue);
        return true;

    case CMD_NONE:
        return true;

    case CMD_RESET_MAGNETOMETER_CALIBRATION:
    case CMD_RESET_ACCELEROMETER_CALIBRATION:
    case CMD_RESET_GYROSCOPE_CALIBRATION:
    case CMD_ENABLE_MAGNETOMETER_CALIBRATION:
    case CMD_DISABLE_MAGNETOMETER_CALIBRATION:
    case CMD_ENABLE_ACCELEROMETER_CALIBRATION:
        calibration.toggleAccelerometerCalibration(true);
        return true;
    case CMD_DISABLE_ACCELEROMETER_CALIBRATION:
        calibration.toggleAccelerometerCalibration(false);
        return true;
    case CMD_ENABLE_GYROSCOPE_CALIBRATION:
        calibration.toggleGyroScopeCalibration(true);
        return true;
    case CMD_DISABLE_GYROSCOPE_CALIBRATION:
        calibration.toggleGyroScopeCalibration(false);
        return true;
    }

    Serial.printf("Unknown command ID: %u\n", packet.commandId);
    return false;
}
