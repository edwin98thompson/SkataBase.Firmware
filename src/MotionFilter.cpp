#include "MotionFilter.h"

MotionFilter::MotionFilter(float filterRate, float beta)
    : filter(),
      filterRate(filterRate),
      beta(beta)
{
    filter.begin(filterRate);
    filter.beta = beta;
}

void MotionFilter::computeOrientation(ImuPacket& packet)
{
    // Update the filter with the latest sensor readings.
    filter.update(
        packet.gyroX, packet.gyroY, packet.gyroZ,
        packet.accelX, packet.accelY, packet.accelZ,
        packet.magX, packet.magY, packet.magZ);

    // Retrieve the computed orientation angles in degrees.
    packet.roll = filter.getRoll();
    packet.pitch = filter.getPitch();
    packet.yaw = filter.getYaw();
    packet.q0 = filter.q0;
    packet.q1 = filter.q1;
    packet.q2 = filter.q2;
    packet.q3 = filter.q3;
}