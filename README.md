# SkateData2026

SkateData2026 is the sensor firmware for a skateboard session analytics system. A XIAO ESP32-C6 reads motion from an ICM-20948 IMU mounted on the skateboard setup, calculates orientation, and streams timestamped sensor data over Bluetooth Low Energy (BLE) to an Android phone. The phone is intended to receive and use this stream for analysing skateboard sessions, such as reviewing movement and ride data. This repository implements the device side of that connection; Android app functionality is not included here.

The firmware is built with PlatformIO and the Arduino framework.

## Hardware

- Seeed Studio XIAO ESP32-C6
- ICM-20948 IMU connected over SPI

The SPI pins configured in `src/ImuManager.cpp` are:

| Signal | XIAO pin | ESP32-C6 GPIO |
| --- | --- | --- |
| SCK | D8 | 19 |
| MISO | D9 | 20 |
| MOSI | D10 | 18 |
| CS/NCS | D3 | 21 |

The firmware expects the sensor's `WHO_AM_I` value to be `0xEA`. Check the board wiring and the startup messages on the serial monitor if the sensor does not initialize.

## Build and flash

1. Install [PlatformIO](https://platformio.org/) (the VS Code extension or PlatformIO Core).
2. Open this project directory.
3. Build the configured environment:

   ```sh
   pio run -e seeed_xiao_esp32_c6
   ```

4. Connect the XIAO over USB and upload:

   ```sh
   pio run -e seeed_xiao_esp32_c6 -t upload
   ```

5. Open the serial monitor at 115200 baud:

   ```sh
   pio device monitor -b 115200
   ```

PlatformIO downloads the ESP32 platform and the Madgwick library declared in `platformio.ini`. The SparkFun ICM-20948 library is included locally under `lib/` with DMP support enabled in that copy. DMP support is not enabled in the current build flags.

## Android phone connection

The intended workflow is to power the board-mounted sensor, connect to it from the Android application over BLE, and receive live IMU notifications for a skateboard session. On startup, the device advertises as **ESP32_SkateIMU**. The Android app can discover the service below, subscribe to notifications to receive sensor packets, and write commands to the same characteristic for calibration and filter settings.

The firmware exposes one service and one characteristic:

| Item | UUID | Properties |
| --- | --- | --- |
| Service | `12345678-1234-1234-1234-1234567890ab` | — |
| Characteristic | `abcd1234-ab12-cd34-ef56-abcdef123456` | Read, notify, write |

The firmware schedules notifications at up to 50 Hz when a client is connected; the IMU sampling loop is driven by sensor data-ready events. A negotiated BLE MTU must support the 81-byte notification payload (MTU 84 or greater). The packet includes a sequence number and device uptime timestamp so the receiving app can order samples and relate them to a session timeline.

## Data and command protocols

### Outgoing IMU packet

Each notification is an 81-byte, little-endian binary packet. Multi-byte integers and IEEE-754 32-bit floats are encoded little-endian; fields are serialized explicitly, so the C++ structure's in-memory padding is not part of the protocol.

| Byte offset | Size | Field | Notes |
| ---: | ---: | --- | --- |
| 0 | 1 | Protocol version | Currently 3 |
| 1 | 2 | Sequence | Unsigned 16-bit counter |
| 3 | 4 | Timestamp | Milliseconds since boot |
| 7 | 12 | Roll, pitch, yaw | Degrees |
| 19 | 12 | Acceleration X, Y, Z | g |
| 31 | 12 | Angular rate X, Y, Z | degrees/second |
| 43 | 12 | Magnetometer X, Y, Z | Sensor units, calibrated when calibration is stored |
| 55 | 16 | Quaternion q0, q1, q2, q3 | Madgwick orientation output |
| 71 | 4 | Temperature | °C |
| 75 | 1 | Battery percentage | Current firmware does not initialize or update `BatteryManager`, so this remains 0 |
| 76 | 1 | Calibration status | 0 not calibrated, 1 in progress, 2 complete, 3 failed |
| 77 | 4 | Filter beta | Madgwick filter parameter |

Angles, acceleration, gyro, magnetometer, quaternion, temperature, and beta fields are 32-bit floats. Magnetometer calibration status also determines whether the orientation filter uses magnetometer data or falls back to accel/gyro only.

### Incoming command packet

Commands are 6 bytes: protocol version (`uint8`, currently 1), command ID (`uint8`), then beta value (`float32`, little-endian). The decoder handles fragmented and consecutive writes. For commands that do not use beta, send `0.0` in the float field.

| ID | Command |
| ---: | --- |
| 0 | No operation |
| 1 | Start magnetometer calibration (30 seconds) |
| 2 | Calibrate accelerometer (keep the board level and still) |
| 3 | Calibrate gyroscope (keep the board still) |
| 4–6 | Reset magnetometer, accelerometer, or gyroscope calibration (reserved; currently no reset action) |
| 7–8 | Enable/disable magnetometer calibration (reserved; currently no action) |
| 9–10 | Enable/disable applying accelerometer calibration |
| 11–12 | Enable/disable applying gyroscope calibration |
| 13 | Set Madgwick beta to the float value; saved to NVS when storage is available |

Accelerometer and gyroscope offsets are held in RAM and are recalculated by their calibration commands. Magnetometer offsets and scales are stored in ESP32 NVS and loaded on startup. For magnetometer calibration, rotate the sensor through a range of orientations throughout the 30-second collection period.

## Project layout

- `src/main.cpp` — startup, main sampling loop, command dispatch, and BLE packet transmission
- `src/ImuManager.cpp` — SPI setup, sensor configuration, and sample conversion
- `src/BluetoothHandler.cpp` — BLE service, characteristic, notifications, and command queue
- `src/CalibrationHandler.cpp` — sensor calibration and NVS storage
- `src/MotionFilter.cpp` — Madgwick orientation calculation
- `src/ImuPacketEncoder.cpp` — explicit binary packet serialization
- `src/BatteryManager.cpp` — optional battery-divider estimator (not currently wired into the main loop)
- `include/` — module interfaces and packet definitions
- `lib/SparkFun_ICM20948/` — local SparkFun sensor library

## Serial diagnostics

Startup and runtime status is printed over USB serial at 115200 baud. Logs report SPI initialization and sensor identity, BLE advertising and MTU negotiation, calibration progress, and NVS status. An 81-byte notification fits in the negotiated ATT payload when the peer MTU is at least 84; the firmware logs whether the negotiated size fits.
