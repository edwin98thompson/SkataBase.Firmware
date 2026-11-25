#pragma once

#include <Arduino.h>
#include "ImuPacket.h"

// Samples the battery divider and writes a LiPo state-of-charge estimate into
// the existing ImuPacket::batteryPercent field.
class BatteryManager
{
public:
    BatteryManager(
        uint8_t adcPin,
        float dividerTopOhms,
        float dividerBottomOhms,
        uint8_t seriesCellCount = 1,
        uint32_t updateIntervalMs = 1000,
        uint8_t samplesPerUpdate = 16);

    bool begin();
    void update(ImuPacket& packet);

    float batteryVoltage() const { return _batteryVoltage; }
    uint8_t batteryPercent() const { return _batteryPercent; }

private:
    uint8_t estimatePercent(float perCellVoltage) const;

    uint8_t _adcPin;
    float _dividerTopOhms;
    float _dividerBottomOhms;
    uint8_t _seriesCellCount;
    uint32_t _updateIntervalMs;
    uint8_t _samplesPerUpdate;

    uint32_t _lastSampleMs = 0;
    float _batteryVoltage = 0.0f;
    uint8_t _batteryPercent = 0;
    bool _initialized = false;
    bool _hasSample = false;
};