#include <MadgwickAHRS.h>
#include "ImuPacket.h"

class MotionFilter
{
public: 
    explicit MotionFilter(float filterRate, float beta);

    void computeOrientation(
        ImuPacket& packet,
        bool magnetometerCalibrated);


private:
    Madgwick filter;
    float filterRate;
    float beta;
};