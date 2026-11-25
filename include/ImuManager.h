#pragma once

#include <Arduino.h>
#include <ICM_20948.h>
#include "ImuPacket.h"

class ImuManager
{
public:
    bool begin();
    bool readSample(ImuPacket& packet);

private:
    bool configure();

    ICM_20948_SPI imu;
    bool spiStarted = false;
};
