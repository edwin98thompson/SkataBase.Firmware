#include "CalibrationHandler.h"

#include "ImuManager.h"

#include <math.h>

bool CalibrationHandler::begin()
{
    // Read/write mode creates the namespace on first boot.
    if (!preferences.begin("imu-cal", false))
    {
        return false;
    }

    // Persist the default state so a fresh device has an initialized namespace.
    if (!preferences.isKey("mag-valid")
        && preferences.putBool("mag-valid", false) != 1)
    {
        preferences.end();
        return false;
    }

    // Read the stored calibration data from non-volatile storage.
    const bool stored = preferences.getBool("mag-valid", false);
    const float offsetX = preferences.getFloat("mag-ox", 0.0f);
    const float offsetY = preferences.getFloat("mag-oy", 0.0f);
    const float offsetZ = preferences.getFloat("mag-oz", 0.0f);
    const float scaleX = preferences.getFloat("mag-sx", 1.0f);
    const float scaleY = preferences.getFloat("mag-sy", 1.0f);
    const float scaleZ = preferences.getFloat("mag-sz", 1.0f);
    preferences.end();

    // Validate the stored calibration data and update the in-memory calibration state.
    if (stored
        && isfinite(offsetX) && isfinite(offsetY) && isfinite(offsetZ)
        && isfinite(scaleX) && isfinite(scaleY) && isfinite(scaleZ)
        && scaleX > 0.0f && scaleY > 0.0f && scaleZ > 0.0f)
    {
        magOffsetX = offsetX;
        magOffsetY = offsetY;
        magOffsetZ = offsetZ;
        magScaleX = scaleX;
        magScaleY = scaleY;
        magScaleZ = scaleZ;
        magnetometerCalibrated = true;
        calibrationStatus = CALIBRATION_COMPLETE;
    }
    else
    {
        magnetometerCalibrated = false;
        calibrationStatus = CALIBRATION_NOT_CALIBRATED;
    }

    storageReady = true;
    return true;
}

/// @brief Calibrate the gyroscope by taking multiple samples and computing the average offset.
/// @param imu The IMU manager instance.
/// @param sampleCount The number of samples to take.
/// @param timeoutMs The maximum time to wait for samples.
/// @return True if calibration is successful, false otherwise.
bool CalibrationHandler::calibrateGyroscope(
    ImuManager& imu,
    uint16_t sampleCount,
    uint32_t timeoutMs)
{
    if (sampleCount == 0 || timeoutMs == 0)
    {
        return false;
    }

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    uint16_t samples = 0;
    const uint32_t startMs = millis();

    while (samples < sampleCount
        && millis() - startMs < timeoutMs)
    {
        ImuPacket packet;
        if (imu.readSample(packet))
        {
            sumX += packet.gyroX;
            sumY += packet.gyroY;
            sumZ += packet.gyroZ;
            ++samples;
        }
        else
        {
            delay(1);
        }
    }

    if (samples != sampleCount)
    {
        return false;
    }

    const float inverseCount = 1.0f / samples;
    gyroOffsetX = sumX * inverseCount;
    gyroOffsetY = sumY * inverseCount;
    gyroOffsetZ = sumZ * inverseCount;
    return true;
}

/// @brief Calibrate the accelerometer by taking multiple samples and computing the average offset.
/// @param imu The IMU manager instance.
/// @param sampleCount The number of samples to take.
/// @param timeoutMs The maximum time to wait for samples.
/// @return True if calibration is successful, false otherwise.
bool CalibrationHandler::calibrateAccelerometer(
    ImuManager& imu,
    uint16_t sampleCount,
    uint32_t timeoutMs)
{
    if (sampleCount == 0 || timeoutMs == 0)
    {
        return false;
    }

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    uint16_t samples = 0;
    const uint32_t startMs = millis();

    while (samples < sampleCount
        && millis() - startMs < timeoutMs)
    {
        ImuPacket packet;
        if (imu.readSample(packet))
        {
            sumX += packet.accelX;
            sumY += packet.accelY;
            sumZ += packet.accelZ;
            ++samples;
        }
        else
        {
            delay(1);
        }
    }

    if (samples != sampleCount)
    {
        return false;
    }

    const float inverseCount = 1.0f / samples;
    const float meanX = sumX * inverseCount;
    const float meanY = sumY * inverseCount;
    const float meanZ = sumZ * inverseCount;

    accelOffsetX = meanX;
    accelOffsetY = meanY;
    accelOffsetZ = meanZ - (meanZ < 0.0f ? -1.0f : 1.0f);
    return true;
}

/// @brief Start the magnetometer calibration process.
/// @param durationMs The duration of the calibration process in milliseconds.
/// @return True if the calibration process is started successfully, false otherwise.
bool CalibrationHandler::startMagnetometerCalibration(
    uint32_t durationMs)
{
    if (!storageReady || collectingMagnetometer || durationMs == 0)
    {
        return false;
    }

    magnetometerStartMs = millis();
    magnetometerDurationMs = durationMs;
    magnetometerSamples = 0;
    collectingMagnetometer = true;
    calibrationStatus = CALIBRATION_IN_PROGRESS;
    return true;
}

/// @brief Update the magnetometer calibration with a new raw sample.
/// @param rawSample The raw magnetometer sample.
void CalibrationHandler::updateMagnetometerCalibration(
    const ImuPacket* rawSample)
{
    if (!collectingMagnetometer)
    {
        return;
    }

    if (rawSample != nullptr)
    {
        if (magnetometerSamples == 0)
        {
            // Initialize the min and max values for each axis with the first sample.
            magMinX = magMaxX = rawSample->magX;
            magMinY = magMaxY = rawSample->magY;
            magMinZ = magMaxZ = rawSample->magZ;
        }
        else
        {
            // Update the min and max values for each axis based on the new sample.
            magMinX = fminf(magMinX, rawSample->magX);
            magMinY = fminf(magMinY, rawSample->magY);
            magMinZ = fminf(magMinZ, rawSample->magZ);
            magMaxX = fmaxf(magMaxX, rawSample->magX);
            magMaxY = fmaxf(magMaxY, rawSample->magY);
            magMaxZ = fmaxf(magMaxZ, rawSample->magZ);
        }
        ++magnetometerSamples;
    }

    // Keep collecting samples until the specified duration has elapsed.
    if (millis() - magnetometerStartMs < magnetometerDurationMs)
    {
        return;
    }

    collectingMagnetometer = false;
    if (magnetometerSamples < 100)
    {
        calibrationStatus = CALIBRATION_FAILED;
        return;
    }

    const float halfRangeX = (magMaxX - magMinX) * 0.5f;
    const float halfRangeY = (magMaxY - magMinY) * 0.5f;
    const float halfRangeZ = (magMaxZ - magMinZ) * 0.5f;
    const float largestHalfRange =
        fmaxf(halfRangeX, fmaxf(halfRangeY, halfRangeZ));

    // Ensure that the half ranges are reasonable.
    if (!isfinite(largestHalfRange)
        || largestHalfRange <= 0.0f
        || halfRangeX < largestHalfRange * 0.2f
        || halfRangeY < largestHalfRange * 0.2f
        || halfRangeZ < largestHalfRange * 0.2f)
    {
        calibrationStatus = CALIBRATION_FAILED;
        return;
    }

    // Compute the offsets and scale factors for each axis.
    const float averageHalfRange =
        (halfRangeX + halfRangeY + halfRangeZ) / 3.0f;
    const float offsetX = (magMaxX + magMinX) * 0.5f;
    const float offsetY = (magMaxY + magMinY) * 0.5f;
    const float offsetZ = (magMaxZ + magMinZ) * 0.5f;
    const float scaleX = averageHalfRange / halfRangeX;
    const float scaleY = averageHalfRange / halfRangeY;
    const float scaleZ = averageHalfRange / halfRangeZ;

    // Save the calibration data to non-volatile storage.
    if (!saveMagnetometerCalibration(
            offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ))
    {
        calibrationStatus = CALIBRATION_FAILED;
        return;
    }

    // Update the in-memory calibration data.
    magOffsetX = offsetX;
    magOffsetY = offsetY;
    magOffsetZ = offsetZ;
    magScaleX = scaleX;
    magScaleY = scaleY;
    magScaleZ = scaleZ;
    magnetometerCalibrated = true;
    calibrationStatus = CALIBRATION_COMPLETE;
}

/// @brief Save the magnetometer calibration data to non-volatile storage.
/// @param offsetX The offset for the X-axis.
/// @param offsetY The offset for the Y-axis.
/// @param offsetZ The offset for the Z-axis.
/// @param scaleX The scale factor for the X-axis.
/// @param scaleY The scale factor for the Y-axis.
/// @param scaleZ The scale factor for the Z-axis.
/// @return True if the data is saved successfully, false otherwise.
bool CalibrationHandler::saveMagnetometerCalibration(
    float offsetX,
    float offsetY,
    float offsetZ,
    float scaleX,
    float scaleY,
    float scaleZ)
{
    if (!preferences.begin("imu-cal", false))
    {
        return false;
    }

    // Save the calibration data to non-volatile storage.
    bool saved = preferences.putBool("mag-valid", false) == 1
        && preferences.putFloat("mag-ox", offsetX) == sizeof(float)
        && preferences.putFloat("mag-oy", offsetY) == sizeof(float)
        && preferences.putFloat("mag-oz", offsetZ) == sizeof(float)
        && preferences.putFloat("mag-sx", scaleX) == sizeof(float)
        && preferences.putFloat("mag-sy", scaleY) == sizeof(float)
        && preferences.putFloat("mag-sz", scaleZ) == sizeof(float);

    if (saved)
    {
        saved = preferences.putBool("mag-valid", true) == 1;
    }

    if (!saved)
    {
        preferences.putBool("mag-valid", false);
    }
    preferences.end();
    return saved;
}

/// @brief Apply the calibration data to the IMU packet.
/// @param packet The IMU packet to calibrate.
void CalibrationHandler::apply(ImuPacket& packet) const
{
    packet.gyroX -= gyroOffsetX;
    packet.gyroY -= gyroOffsetY;
    packet.gyroZ -= gyroOffsetZ;

    packet.accelX -= accelOffsetX;
    packet.accelY -= accelOffsetY;
    packet.accelZ -= accelOffsetZ;

    if (magnetometerCalibrated)
    {
        packet.magX = (packet.magX - magOffsetX) * magScaleX;
        packet.magY = (packet.magY - magOffsetY) * magScaleY;
        packet.magZ = (packet.magZ - magOffsetZ) * magScaleZ;
    }
    packet.calibrationStatus = static_cast<uint8_t>(calibrationStatus);
}

/// @brief Check if the magnetometer is calibrated.
/// @return True if the magnetometer is calibrated, false otherwise.
bool CalibrationHandler::isMagnetometerCalibrated() const
{
    return magnetometerCalibrated;
}

/// @brief Get the calibration status.
/// @return The calibration status.
CalibrationStatus CalibrationHandler::status() const
{
    return calibrationStatus;
}
