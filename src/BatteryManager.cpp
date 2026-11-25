#include "BatteryManager.h"

BatteryManager::BatteryManager(
    uint8_t adcPin,
    float dividerTopOhms,
    float dividerBottomOhms,
    uint8_t seriesCellCount,
    uint32_t updateIntervalMs,
    uint8_t samplesPerUpdate)
    : _adcPin(adcPin),
      _dividerTopOhms(dividerTopOhms),
      _dividerBottomOhms(dividerBottomOhms),
      _seriesCellCount(seriesCellCount),
      _updateIntervalMs(updateIntervalMs),
      _samplesPerUpdate(samplesPerUpdate)
{
}

bool BatteryManager::begin()
{
    if (_dividerTopOhms <= 0.0f ||
        _dividerBottomOhms <= 0.0f ||
        _seriesCellCount == 0 ||
        _updateIntervalMs == 0 ||
        _samplesPerUpdate == 0)
    {
        return false;
    }

    analogReadResolution(12);
    analogSetPinAttenuation(_adcPin, ADC_11db);
    pinMode(_adcPin, INPUT);

    _initialized = true;
    return true;
}

void BatteryManager::update(ImuPacket& packet)
{
    if (_initialized)
    {
        const uint32_t now = millis();
        if (!_hasSample ||
            static_cast<uint32_t>(now - _lastSampleMs) >= _updateIntervalMs)
        {
            uint64_t totalMilliVolts = 0;
            for (uint8_t i = 0; i < _samplesPerUpdate; ++i)
            {
                totalMilliVolts += analogReadMilliVolts(_adcPin);
                delayMicroseconds(200);
            }

            const float adcVoltage =
                static_cast<float>(totalMilliVolts) /
                static_cast<float>(_samplesPerUpdate) / 1000.0f;

            const float dividerScale =
                (_dividerTopOhms + _dividerBottomOhms) /
                _dividerBottomOhms;
            const float measuredBatteryVoltage = adcVoltage * dividerScale;

            // Smooth ADC noise while still tracking a changing battery level.
            if (_hasSample)
            {
                constexpr float FILTER_ALPHA = 0.25f;
                _batteryVoltage +=
                    FILTER_ALPHA * (measuredBatteryVoltage - _batteryVoltage);
            }
            else
            {
                _batteryVoltage = measuredBatteryVoltage;
                _hasSample = true;
            }

            _batteryPercent =
                estimatePercent(_batteryVoltage / _seriesCellCount);
            _lastSampleMs = now;
        }
    }

    // Preserve the latest value in every outgoing packet.
    packet.batteryPercent = _batteryPercent;
}

uint8_t BatteryManager::estimatePercent(float perCellVoltage) const
{
    // Approximate rested single-cell LiPo voltage curve. Under-load voltage
    // sag and cell variation mean this is an estimate, not a fuel gauge.
    struct Point
    {
        float voltage;
        float percent;
    };
    static const Point curve[] = {
        {3.30f, 0.0f},
        {3.50f, 5.0f},
        {3.60f, 10.0f},
        {3.70f, 20.0f},
        {3.75f, 30.0f},
        {3.80f, 40.0f},
        {3.85f, 50.0f},
        {3.90f, 60.0f},
        {3.95f, 70.0f},
        {4.00f, 80.0f},
        {4.05f, 85.0f},
        {4.10f, 90.0f},
        {4.15f, 95.0f},
        {4.20f, 100.0f}};

    constexpr size_t CURVE_SIZE = sizeof(curve) / sizeof(curve[0]);
    if (perCellVoltage <= curve[0].voltage)
    {
        return 0;
    }

    for (size_t i = 1; i < CURVE_SIZE; ++i)
    {
        if (perCellVoltage <= curve[i].voltage)
        {
            const float fraction =
                (perCellVoltage - curve[i - 1].voltage) /
                (curve[i].voltage - curve[i - 1].voltage);
            const float percent =
                curve[i - 1].percent +
                fraction * (curve[i].percent - curve[i - 1].percent);
            return static_cast<uint8_t>(percent + 0.5f);
        }
    }

    return 100;
}