#ifndef MOTIONFILTER_H
#define MOTIONFILTER_H

#include <MadgwickAHRS.h>
#include "ImuPacket.h"
#include "CommandPacket.h"

class MotionFilter
{
public:
    explicit MotionFilter(float filterRate, float beta);

    void computeOrientation(
        ImuPacket& packet,
        bool magnetometerCalibrated);

    void setBeta(float value);

    float beta;

private:
    Madgwick filter;
    float filterRate;
};

#endif // MOTIONFILTER_H
