#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
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

// IMU sampling using DMP Quaternions
// bool sampleIMU(int16_t* outData) {
//     icm_20948_DMP_data_t dmpData;
    
//     // Read the latest data block from the sensor's memory FIFO
//     myIMU.readDMPdataFromFIFO(&dmpData);

//     // Check if data was successfully fetched from the chip
//     if ((myIMU.status == ICM_20948_Stat_Ok) || (myIMU.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
//         Serial.println("data was successfully fetched from the chip");
        
//         // Check if the data packet contains our 9-axis orientation vector
//         if (dmpData.header & DMP_header_bitmap_Quat9) {
//             Serial.println("data packedet contains our 9-axis orientation vector");

            
//             // Extract Q1, Q2, and Q3 from the SparkFun DMP data structure
//             float q1 = ((float)dmpData.Quat9.Data.Q1) / 1073741824.0f;
//             float q2 = ((float)dmpData.Quat9.Data.Q2) / 1073741824.0f;
//             float q3 = ((float)dmpData.Quat9.Data.Q3) / 1073741824.0f;

//             // Derived Math: Compute Q0 using the remaining three values
//             float q0_sqr = 1.0f - ((q1 * q1) + (q2 * q2) + (q3 * q3));
//             float q0 = (q0_sqr > 0.0f) ? sqrtf(q0_sqr) : 0.0f;

//             // Scale floats into your original int16_t BLE packet structure
//             outData[0] = scaleFloatToInt16(q0, -1.0f, 1.0f); // W component
//             outData[1] = scaleFloatToInt16(q1, -1.0f, 1.0f); // X component
//             outData[2] = scaleFloatToInt16(q2, -1.0f, 1.0f); // Y component
//             outData[3] = scaleFloatToInt16(q3, -1.0f, 1.0f); // Z component
            
//             return true; // New valid data successfully mapped
//         }
//     }
//     return false; // No new data available in this loop cycle
// }


// manually create quats
// // IMU sampling and on-board fusion processing
// bool sampleIMU(int16_t* outData) {
//     if (myIMU.dataReady()) {
//         myIMU.getAGMT();

//         unsigned long now = micros();
//         float samplePeriod = (now - lastSRT) / 1000000.0f;
//         lastSRT = now;
        
//         if (samplePeriod <= 0.0f || samplePeriod > 0.5f) samplePeriod = 0.01f;

//         // Convert raw sensors to float units
//         float ax = myIMU.agmt.acc.axes.x;
//         float ay = myIMU.agmt.acc.axes.y;
//         float az = myIMU.agmt.acc.axes.z;
//         float gx = myIMU.agmt.gyr.axes.x / 131.0f;
//         float gy = myIMU.agmt.gyr.axes.y / 131.0f;
//         float gz = myIMU.agmt.gyr.axes.z / 131.0f;
//         float mx = myIMU.agmt.mag.axes.x;
//         float my = myIMU.agmt.mag.axes.y;
//         float mz = myIMU.agmt.mag.axes.z;

//         // Update the unmodified library filter object
//         filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

//         // Fetch the public orientation radians from the library
//         float yaw   = filter.getYawRadians();
//         float pitch = filter.getPitchRadians();
//         float roll  = filter.getRollRadians();

//         // Standard Math: Convert Euler angles back into a precise 3D Quaternion
//         float cy = cosf(yaw * 0.5f);
//         float sy = sinf(yaw * 0.5f);
//         float cp = cosf(pitch * 0.5f);
//         float sp = sinf(pitch * 0.5f);
//         float cr = cosf(roll * 0.5f);
//         float sr = sinf(roll * 0.5f);

//         float q0 = cr * cp * cy + sr * sp * sy; // W
//         float q1 = sr * cp * cy - cr * sp * sy; // X
//         float q2 = cr * sp * cy + sr * cp * sy; // Y
//         float q3 = cr * cp * sy - sr * sp * cy; // Z

//         // Pack scaled data safely into the array slots
//         outData[0] = scaleFloatToInt16(q0, -1.0f, 1.0f);
//         outData[1] = scaleFloatToInt16(q1, -1.0f, 1.0f);
//         outData[2] = scaleFloatToInt16(q2, -1.0f, 1.0f);
//         outData[3] = scaleFloatToInt16(q3, -1.0f, 1.0f);

//         for(int i = 4; i < 10; i++) outData[i] = 0;
//         return true;
//     }
//     return false;
// }

// IMU sampling and on-board fusion processing
bool sampleIMU(int16_t* outData) {
    if (myIMU.dataReady()) {
        myIMU.getAGMT(); // Read raw physical registers

        // Calculate exact time elapsed since the last loop execution (delta t)
        unsigned long now = micros();
        float samplePeriod = (now - lastSRT) / 1000000.0f;
        lastSRT = now;
        
        // Safety fallback for the first loop
        if (samplePeriod <= 0.0f || samplePeriod > 0.5f) samplePeriod = 0.01f;

        // Convert raw values to standard units for the math filter
        // Accel: Convert to Gs (Default scale is 16384 LSB/g)
        float ax = myIMU.agmt.acc.axes.x / 100.0f;
        float ay = myIMU.agmt.acc.axes.y / 100.0f;
        float az = myIMU.agmt.acc.axes.z / 100.0f;

        // Gyro: Convert raw to Degrees Per Second (Default scale is 131 LSB/dps)
        float gx = myIMU.agmt.gyr.axes.x / 131.0f;
        float gy = myIMU.agmt.gyr.axes.y / 131.0f;
        float gz = myIMU.agmt.gyr.axes.z / 131.0f;

        // Magnetometer: Raw microteslas (µT) work fine directly
        float mx = myIMU.agmt.mag.axes.x;
        float my = myIMU.agmt.mag.axes.y;
        float mz = myIMU.agmt.mag.axes.z;

        float temp = myIMU.agmt.tmp.val / 333.87f + 21.0f; // Convert raw temp to Celsius

        // Feed data into the Madgwick algorithm
        // Note: If you experience compass interference, swap this line for:
        // filter.updateIMU(gx, gy, gz, ax, ay, az); 
        // filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

        // // Extract calculated quaternions (Values are clean floats from -1.0 to 1.0)
        // float q0 = filter.q0;
        // float q1 = filter.q1;
        // float q2 = filter.q2;
        // float q3 = filter.q3;

        // Pack into your original int16_t BLE array indices
        // outData[0] = scaleFloatToInt16(q0, -1.0f, 1.0f);
        // outData[1] = scaleFloatToInt16(q1, -1.0f, 1.0f);
        // outData[2] = scaleFloatToInt16(q2, -1.0f, 1.0f);
        // outData[3] = scaleFloatToInt16(q3, -1.0f, 1.0f);
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

        // Zero out remaining slots to preserve your 10-element packet footprint
        // for(int i = 4; i < 10; i++) {
        //     outData[i] = 0;
        // }

        return true;
    }
    return false;
}

// Send BLE value
void sendBLEPacket(int16_t* data, size_t count) {
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

    Serial.println("🕒 Initializing ICM-20948 DMP...");
    
    // Enable the DMP internal engine
    bool dmpReady = false;
    if (myIMU.initializeDMP() == ICM_20948_Stat_Ok) {
        // Enable 9-axis Game Rotation Vector (Quaternions derived from gyro/accel/mag)
        if (myIMU.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok) {
            // Set DMP output rate to 50Hz (matching your 20000us original target)
            myIMU.setDMPODRrate(DMP_ODR_Reg_Quat9, 20000); 
            
            // Enable the internal FIFO buffer
            myIMU.enableFIFO(true);
            myIMU.enableDMP(true);
            myIMU.resetFIFO();
            myIMU.resetDMP();
            dmpReady = true;
        }
    }

    if (!dmpReady) {
        Serial.println("❌ Failed to configure ICM-20948 DMP!");
        while(1);
    }

    Serial.println("✅ ICM-20948 configured");

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
    static int16_t imuData[10] = {0}; // Create an array of 10 items, initialized to zero

    // Keep pulling data to prevent the sensor's FIFO buffer from overflowing
    if (sampleIMU(imuData)) {
        // Only transmit over BLE at your designated transmission rate (100ms)
        if (millis() - lastSendTime >= sendInterval) {
            sendBLEPacket(imuData, 10);
            lastSendTime = millis();
        }
    }

    delay(1); // Small yield to prevent locking up the ESP32 background tasks
}

