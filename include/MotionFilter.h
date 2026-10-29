#include <MadgwickAHRS.h>
#include "ImuPacket.h"

class MotionFilter
{
public: 
    explicit MotionFilter(float filterRate, float beta);

    void computeOrientation(ImuPacket& packet);


private:
    Madgwick filter;
    float filterRate;
    float beta;
};