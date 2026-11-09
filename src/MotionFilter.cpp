#include "MotionFilter.h"

MotionFilter::MotionFilter(float filterRate, float beta)
    : filter(),
      filterRate(filterRate),
      beta(beta)
{
    filter.begin(filterRate);
    filter.beta = beta;
}

void MotionFilter::setBeta(float value)
{
    beta = value;
    filter.beta = value;
}

void MotionFilter::computeOrientation(
    ImuPacket& packet,
    bool magnetometerCalibrated)
{
    // Update the filter with the latest sensor readings, if magnetometer calibration is available,
    // use the magnetometer data; otherwise, use only the accelerometer and gyroscope data.
    if (magnetometerCalibrated)
    {
        filter.update(
            packet.gyroX, packet.gyroY, packet.gyroZ,
            packet.accelX, packet.accelY, packet.accelZ,
            packet.magX, packet.magY, packet.magZ);
    }
    else
    {
        filter.updateIMU(
            packet.gyroX, packet.gyroY, packet.gyroZ,
            packet.accelX, packet.accelY, packet.accelZ);
    }

    // Retrieve the computed orientation angles in degrees.
    packet.roll = filter.getRoll();
    packet.pitch = filter.getPitch();
    packet.yaw = filter.getYaw();
    packet.q0 = filter.q0;
    packet.q1 = filter.q1;
    packet.q2 = filter.q2;
    packet.q3 = filter.q3;
}
