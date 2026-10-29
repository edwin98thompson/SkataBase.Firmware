#pragma once

#include <Arduino.h>
#include <ICM_20948.h>
#include "ImuPacket.h"

class ImuManager
{
public:
    bool begin();

    // Reads a new sensor sample and populates the relevant
    // sensor fields in the existing ImuPacket.
    bool readSample(ImuPacket& packet);

private:
    bool configure();

    ICM_20948_I2C imu;
};