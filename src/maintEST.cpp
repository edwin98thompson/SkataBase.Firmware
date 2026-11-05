#include <Wire.h>
#include <Arduino.h>

constexpr int SDA_PIN = 22;
constexpr int SCL_PIN = 23;

void setup()
{
    Serial.begin(115200);
    delay(15000);

    Serial.println("\nI2C transaction test - no initial probe");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    delay(100);

    for (uint8_t address : {0x68, 0x69})
    {
        Serial.printf("\nTesting 0x%02X\n", address);

        // Test 1: Write register pointer WITHOUT probing first
        Wire.beginTransmission(address);
        Wire.write((uint8_t)0x00);

        uint8_t result = Wire.endTransmission(true);

        Serial.printf("Register write result: %u\n", result);

        // Test 2: Independent read
        uint8_t count = Wire.requestFrom(address, (uint8_t)1);

        Serial.printf("Bytes received: %u\n", count);

        if (count == 1)
        {
            Serial.printf("WHO_AM_I: 0x%02X\n", Wire.read());
        }
    }
}

void loop() {}