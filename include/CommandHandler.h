#pragma once

#include "CalibrationHandler.h"
#include "CommandPacket.h"
#include "ImuManager.h"
#include "MotionFilter.h"
#include <Preferences.h>

// Applies decoded commands in the main loop, keeping application behavior out
// of the BLE callback and byte decoder.
class CommandHandler
{
public:
    static constexpr float DEFAULT_BETA = 0.2f;

    CommandHandler(
        CalibrationHandler& calibration,
        ImuManager& imu,
        MotionFilter& motionFilter);

    // Opens NVS and loads the saved beta, falling back to defaultBeta().
    bool begin();

    // Returns true when the command was handled successfully.
    bool handle(const CommandPacket& packet);

private:
    CalibrationHandler& calibration;
    ImuManager& imu;
    MotionFilter& motionFilter;
    Preferences preferences;
    bool storageReady = false;
};
