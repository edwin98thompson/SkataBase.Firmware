#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h> // Include the Madgwick Filter Library

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-abcdef123456"

// BLE + IMU
ICM_20948_I2C  myIMU;
Madgwick filter;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Timing Variables
unsigned long lastSRT = 0; // For tracking exact sample delta time
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 20; // 50Hz transmission rate
constexpr float FILTER_RATE = 228.0f;

// Calibration Structures

struct AccelCalibration
{
    float offsetX;
    float offsetY;
    float offsetZ;

    float scaleX;
    float scaleY;
    float scaleZ;
};

AccelCalibration accel =
{
    0,0,0,
    1,1,1
};

struct MagCalibration
{
    float offsetX;
    float offsetY;
    float offsetZ;
};

MagCalibration mag =
{
    0,
    0,
    0
};

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

int gyroCalSamples = 1000;

float accelBiasX = 0.0;
float accelBiasY = 0.0;
float accelBiasZ = 0.0;

int accelCalSamples = 1000;

// BLE callbacks
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override {
        deviceConnected = true;
        Serial.println("✅ BLE Client connected");
    }

    void onDisconnect(BLEServer *pServer) override {
        deviceConnected = false;
        Serial.println("❌ BLE Client disconnected");
        pServer->startAdvertising();
    }
};

// Utility: clamp and scale float to int16_t
float clamp(float val, float minVal, float maxVal) {
    return (val < minVal) ? minVal : (val > maxVal ? maxVal : val);
}

int16_t scaleFloatToInt16(float val, float minVal, float maxVal) {
    val = clamp(val, minVal, maxVal);
    return (int16_t)((((val - minVal) / (maxVal - minVal)) * 65535.0f) - 32768.0f);
}

void getGyroBias()
{
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    Serial.println("🕒 Calibrating gyroscope...");

    for (int i = 0; i < gyroCalSamples; i++) {
        if (myIMU.dataReady()) {
            myIMU.getAGMT();
            sumX += myIMU.agmt.gyr.axes.x;
            sumY += myIMU.agmt.gyr.axes.y;
            sumZ += myIMU.agmt.gyr.axes.z;
        }
        delay(1); // Small delay to allow sensor to update
    }

    gyroBiasX = sumX / gyroCalSamples;
    gyroBiasY = sumY / gyroCalSamples;
    gyroBiasZ = sumZ / gyroCalSamples;

    Serial.print("✅ Gyro Bias X: "); Serial.println(gyroBiasX);
    Serial.print("✅ Gyro Bias Y: "); Serial.println(gyroBiasY);
    Serial.print("✅ Gyro Bias Z: "); Serial.println(gyroBiasZ);
}

void getAccelBias()
{
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    Serial.println("🕒 Calibrating accelerometer...");

    for (int i = 0; i < accelCalSamples; i++) {
        if (myIMU.dataReady()) {
            myIMU.getAGMT();
            sumX += myIMU.agmt.acc.axes.x;
            sumY += myIMU.agmt.acc.axes.y;
            sumZ += myIMU.agmt.acc.axes.z;
        }
        delay(1); // Small delay to allow sensor to update
    }

    accelBiasX = sumX / accelCalSamples;
    accelBiasY = sumY / accelCalSamples;
    accelBiasZ = sumZ / accelCalSamples;

    Serial.print("✅ Accel Bias X: "); Serial.println(accelBiasX);
    Serial.print("✅ Accel Bias Y: "); Serial.println(accelBiasY);
    Serial.print("✅ Accel Bias Z: "); Serial.println(accelBiasZ);
}


// IMU sampling and on-board fusion processing
bool sampleIMU(float* outData) {

    if (!myIMU.dataReady())
    {
        return false;
    }

    myIMU.getAGMT(); // Read raw physical registers

    // calculate acc to 3 dp
    const float ax = (myIMU.agmt.acc.axes.x - accelBiasX) / 8192.0f;
    const float ay = (myIMU.agmt.acc.axes.y - accelBiasY) / 8192.0f;
    const float az = (myIMU.agmt.acc.axes.z - accelBiasZ) / 8192.0f;

    // // Gyro: Convert raw to Degrees Per Second (Default scale is 131 LSB/dps)
    float gx = (myIMU.agmt.gyr.axes.x - gyroBiasX) / 32.8f;
    float gy = (myIMU.agmt.gyr.axes.y - gyroBiasY) / 32.8f;
    float gz = (myIMU.agmt.gyr.axes.z - gyroBiasZ) / 32.8f;

    // // Magnetometer: Raw microteslas (µT) work fine directly
    float mx = myIMU.agmt.mag.axes.x;
    float my = myIMU.agmt.mag.axes.y;
    float mz = myIMU.agmt.mag.axes.z;

    float temp = myIMU.agmt.tmp.val / 333.87f + 21.0f; // Convert raw temp to Celsius

    // print data
    // Serial.print(ax, 3); Serial.print(", ");
    // Serial.print(ay, 3); Serial.print(", ");
    // Serial.print(az, 3); Serial.print(", ");

    // Serial.print(gx, 3); Serial.print(", ");
    // Serial.print(gy, 3); Serial.print(", ");
    // Serial.print(gz, 3); Serial.print(", ");

    // Serial.print(mx, 3); Serial.print(", ");
    // Serial.print(my, 3); Serial.print(", ");
    // Serial.print(mz, 3); Serial.print(", ");
    // Serial.println();

    filter.updateIMU(
        gx,
        gy,
        gz,
        ax,
        ay,
        az
    );

    float roll  = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw   = filter.getYaw();

    outData[0] = ax;
    outData[1] = ay;
    outData[2] = az;
    outData[3] = gx;
    outData[4] = gy;
    outData[5] = gz;
    outData[6] = mx;
    outData[7] = my;
    outData[8] = mz;
    outData[9] = temp;

    // Monitor the actual sample rate once per second
    // static uint32_t samples = 0;
    // static uint32_t lastPrint = millis();

    // samples++;

    // uint32_t now = millis();
    // if (now - lastPrint >= 1000)
    // {
    //     Serial.printf("Actual IMU rate: %lu Hz\n", samples);
    //     samples = 0;
    //     lastPrint = now;
    // }

    // Serial.printf(
    //     "Roll: %.2f Pitch: %.2f Yaw: %.2f\n",
    //     roll,
    //     pitch,
    //     yaw
    // );

    return true;
}

// Send BLE value
void sendBLEPacket(float* data, size_t count) {
    if (deviceConnected && pCharacteristic != nullptr) {
        pCharacteristic->setValue((uint8_t*)data, count * sizeof(int16_t));
        pCharacteristic->notify();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("🔌 Starting...");

    Wire.begin(21, 22);  // SDA, SCL
    Wire.setClock(400000); // ICM-20948 DMP works best with fast I2C

    Serial.println("🕒 Initializing IMU...");

    // Initialize the ICM hardware
    bool initialized = false;
    while (!initialized) {
        // 0x69 is the default address for SparkFun. Use 0x68 if AD0 pin is low.
        myIMU.begin(Wire, 0); 
        if (myIMU.status != ICM_20948_Stat_Ok) {
            Serial.println("❌ Failed to start ICM-20948 hardware. Retrying...");
            delay(500);
        } else {
            initialized = true;
        }
    }

    ICM_20948_fss_t gyroFSS;
    gyroFSS.a = 0; // unused
    gyroFSS.g = ICM_20948_GYRO_CONFIG_1_FS_SEL_e::dps1000;

    ICM_20948_fss_t accelFSS;
    accelFSS.a = ICM_20948_ACCEL_CONFIG_FS_SEL_e::gpm4;
    accelFSS.g = 0; // unused

    // Configure gyro ranges
    if (myIMU.setFullScale(ICM_20948_Internal_Gyr, gyroFSS) 
            != ICM_20948_Stat_Ok)
    {
        Serial.println("❌ Failed to set gyro full scale");
    }

    // Configure accel ranges
    if (myIMU.setFullScale(ICM_20948_Internal_Acc, accelFSS) 
            != ICM_20948_Stat_Ok)
    {
        Serial.println("❌ Failed to set accel full scale");
    }

    ICM_20948_smplrt_t sampleRate;

    sampleRate.a = 4;   // 225 Hz accel
    sampleRate.g = 4;   // 220 Hz gyro

    if (myIMU.setSampleRate(ICM_20948_Internal_Acc, sampleRate) != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to set accelerometer sample rate");
    }

    if (myIMU.setSampleRate(ICM_20948_Internal_Gyr, sampleRate) != ICM_20948_Stat_Ok)
    {
        Serial.println("Failed to set gyroscope sample rate");
    }

    // ICM_20948_dlpcfg_t dlpf;

    // dlpf.g = gyr_d119bw5_n154bw3;   // Example
    // dlpf.a = /* accelerometer enum */;

    // myIMU.enableDLPF(ICM_20948_Internal_Gyr, true);
    // myIMU.enableDLPF(ICM_20948_Internal_Acc, true);

    // myIMU.setDLPFcfg(ICM_20948_Internal_Gyr, dlpf);
    // myIMU.setDLPFcfg(ICM_20948_Internal_Acc, dlpf);

    myIMU.startupMagnetometer();

    Serial.println("✅ ICM-20948 configured");

    getGyroBias();
    Serial.println("✅ Gyroscope calibration complete");
    getAccelBias();
    Serial.println("✅ Accelerometer calibration complete");

    // Madgwick filter setup
    filter.begin(FILTER_RATE);      // 220Hz update rate
    lastSRT = micros();

    // BLE setup
    BLEDevice::init("ESP32_SkateIMU");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();

    Serial.println("📡 BLE Advertising started");
}

void loop() {
    static float imuData[10]; // Create an array of 10 items, initialized to zero

    // Keep pulling data to prevent the sensor's FIFO buffer from overflowing
    if (sampleIMU(imuData)) {
        // Only transmit over BLE at your designated transmission rate (100ms)
        if (millis() - lastSendTime >= sendInterval) {
            sendBLEPacket(imuData, 10);
            lastSendTime = millis();
        }
    }
}

