#include "CalibrationHandler.h"

#include "ImuManager.h"

#include <math.h>

bool CalibrationHandler::begin()
{
    // Read/write mode creates the namespace on first boot.
    if (!preferences.begin("imu-cal", false))
    {
        Serial.println("Calibration: failed to open NVS namespace 'imu-cal'.");
        return false;
    }

    // Persist the default state so a fresh device has an initialized namespace.
    if (!preferences.isKey("mag-valid")
        && preferences.putBool("mag-valid", false) != 1)
    {
        preferences.end();
        Serial.println("Calibration: failed to initialize NVS calibration state.");
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
        Serial.printf(
            "Calibration: loaded magnetometer calibration; "
            "offset=(%.2f, %.2f, %.2f), scale=(%.3f, %.3f, %.3f).\n",
            magOffsetX, magOffsetY, magOffsetZ,
            magScaleX, magScaleY, magScaleZ);
    }
    else
    {
        magnetometerCalibrated = false;
        calibrationStatus = CALIBRATION_NOT_CALIBRATED;
        Serial.println(stored
            ? "Calibration: stored magnetometer data is invalid; recalibration required."
            : "Calibration: no stored magnetometer calibration.");
    }

    storageReady = true;
    Serial.println("Calibration: NVS ready.");
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
        Serial.printf(
            "Calibration: gyroscope calibration rejected "
            "(sampleCount=%u, timeoutMs=%lu).\n",
            static_cast<unsigned>(sampleCount),
            static_cast<unsigned long>(timeoutMs));
        return false;
    }

    Serial.printf(
        "Calibration: gyroscope calibration started "
        "(target=%u samples, timeout=%lu ms). Keep the IMU still.\n",
        static_cast<unsigned>(sampleCount),
        static_cast<unsigned long>(timeoutMs));

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
        Serial.printf(
            "Calibration: gyroscope calibration FAILED "
            "(received %u/%u samples in %lu ms).\n",
            static_cast<unsigned>(samples),
            static_cast<unsigned>(sampleCount),
            static_cast<unsigned long>(millis() - startMs));
        return false;
    }

    const float inverseCount = 1.0f / samples;
    gyroOffsetX = sumX * inverseCount;
    gyroOffsetY = sumY * inverseCount;
    gyroOffsetZ = sumZ * inverseCount;
    Serial.printf(
        "Calibration: gyroscope calibration COMPLETE; "
        "offset=(%.5f, %.5f, %.5f), samples=%u, elapsed=%lu ms.\n",
        gyroOffsetX, gyroOffsetY, gyroOffsetZ,
        static_cast<unsigned>(samples),
        static_cast<unsigned long>(millis() - startMs));
    return true;
}

/// @brief Toggle gyroscope calibration.
/// @param enable 
void CalibrationHandler::toggleGyroScopeCalibration(bool enable)
{
    gyroCalibrationEnabled = enable;
    Serial.printf(
        "Calibration: gyroscope calibration %s.\n",
        enable ? "enabled" : "disabled");
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
        Serial.printf(
            "Calibration: accelerometer calibration rejected "
            "(sampleCount=%u, timeoutMs=%lu).\n",
            static_cast<unsigned>(sampleCount),
            static_cast<unsigned long>(timeoutMs));
        return false;
    }

    Serial.printf(
        "Calibration: accelerometer calibration started "
        "(target=%u samples, timeout=%lu ms). Keep the IMU level and still.\n",
        static_cast<unsigned>(sampleCount),
        static_cast<unsigned long>(timeoutMs));

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
        Serial.printf(
            "Calibration: accelerometer calibration FAILED "
            "(received %u/%u samples in %lu ms).\n",
            static_cast<unsigned>(samples),
            static_cast<unsigned>(sampleCount),
            static_cast<unsigned long>(millis() - startMs));
        return false;
    }

    const float inverseCount = 1.0f / samples;
    const float meanX = sumX * inverseCount;
    const float meanY = sumY * inverseCount;
    const float meanZ = sumZ * inverseCount;

    accelOffsetX = meanX;
    accelOffsetY = meanY;
    accelOffsetZ = meanZ - (meanZ < 0.0f ? -1.0f : 1.0f);
    Serial.printf(
        "Calibration: accelerometer calibration COMPLETE; "
        "offset=(%.5f, %.5f, %.5f), samples=%u, elapsed=%lu ms.\n",
        accelOffsetX, accelOffsetY, accelOffsetZ,
        static_cast<unsigned>(samples),
        static_cast<unsigned long>(millis() - startMs));
    return true;
}

/// @brief Toggle accelerometer calibration.
/// @param enable 
void CalibrationHandler::toggleAccelerometerCalibration(bool enable)
{
    accelCalibrationEnabled = enable;
    Serial.printf(
        "Calibration: accelerometer calibration %s.\n",
        enable ? "enabled" : "disabled");
}

/// @brief Start the magnetometer calibration process.
/// @param durationMs The duration of the calibration process in milliseconds.
/// @return True if the calibration process is started successfully, false otherwise.
bool CalibrationHandler::startMagnetometerCalibration(
    uint32_t durationMs)
{
    if (!storageReady)
    {
        Serial.println(
            "Calibration: magnetometer calibration rejected; NVS is not ready.");
        return false;
    }

    if (collectingMagnetometer)
    {
        Serial.println(
            "Calibration: magnetometer calibration rejected; one is already running.");
        return false;
    }

    if (durationMs == 0)
    {
        Serial.println(
            "Calibration: magnetometer calibration rejected; duration must be nonzero.");
        return false;
    }

    magnetometerStartMs = millis();
    magnetometerDurationMs = durationMs;
    magnetometerSamples = 0;
    lastMagnetometerProgressMs = 0;
    collectingMagnetometer = true;
    calibrationStatus = CALIBRATION_IN_PROGRESS;
    Serial.printf(
        "Calibration: magnetometer calibration started for %lu ms. "
        "Rotate the IMU through all orientations.\n",
        static_cast<unsigned long>(durationMs));
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
    const uint32_t elapsedMs = millis() - magnetometerStartMs;
    if (elapsedMs < magnetometerDurationMs)
    {
        if (elapsedMs - lastMagnetometerProgressMs >= 5000)
        {
            lastMagnetometerProgressMs = elapsedMs;
            Serial.printf(
                "Calibration: magnetometer progress %lu/%lu ms, "
                "%lu samples; min=(%.2f, %.2f, %.2f), "
                "max=(%.2f, %.2f, %.2f).\n",
                static_cast<unsigned long>(elapsedMs),
                static_cast<unsigned long>(magnetometerDurationMs),
                static_cast<unsigned long>(magnetometerSamples),
                magMinX, magMinY, magMinZ,
                magMaxX, magMaxY, magMaxZ);
        }
        return;
    }

    collectingMagnetometer = false;
    Serial.printf(
        "Calibration: magnetometer collection finished after %lu ms "
        "with %lu samples.\n",
        static_cast<unsigned long>(elapsedMs),
        static_cast<unsigned long>(magnetometerSamples));

    if (magnetometerSamples < 100)
    {
        calibrationStatus = CALIBRATION_FAILED;
        Serial.printf(
            "Calibration: magnetometer calibration FAILED; "
            "at least 100 samples are required (got %lu).\n",
            static_cast<unsigned long>(magnetometerSamples));
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
        Serial.printf(
            "Calibration: magnetometer calibration FAILED; "
            "insufficient axis coverage. Half-ranges=(%.2f, %.2f, %.2f).\n",
            halfRangeX, halfRangeY, halfRangeZ);
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
        Serial.println(
            "Calibration: magnetometer calibration FAILED; "
            "could not save results to NVS.");
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
    Serial.printf(
        "Calibration: magnetometer calibration COMPLETE; "
        "offset=(%.2f, %.2f, %.2f), scale=(%.3f, %.3f, %.3f).\n",
        magOffsetX, magOffsetY, magOffsetZ,
        magScaleX, magScaleY, magScaleZ);
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
        Serial.println(
            "Calibration: failed to open NVS while saving magnetometer results.");
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
        Serial.println(
            "Calibration: one or more magnetometer NVS writes failed.");
    }
    preferences.end();
    return saved;
}

/// @brief Apply the calibration data to the IMU packet.
/// @param packet The IMU packet to calibrate.
void CalibrationHandler::apply(ImuPacket& packet) const
{
    if (gyroCalibrationEnabled)
    {
        packet.gyroX -= gyroOffsetX;
        packet.gyroY -= gyroOffsetY;
        packet.gyroZ -= gyroOffsetZ;
    }

    if (accelCalibrationEnabled)
    {
        packet.accelX -= accelOffsetX;
        packet.accelY -= accelOffsetY;
        packet.accelZ -= accelOffsetZ;
    }

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
