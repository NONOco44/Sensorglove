#include <SoftWire.h>
#include <AsyncDelay.h>
#include <math.h>

// Define the pins for each sensor
int sdaPins[] = {14, 25, 7, 24, 28, 32, 31, 8};
int sclPins[] = {30, 39, 6, 10, 27, 29, 11, 9};

// BMI160 I2C address
const uint8_t I2C_ADDRESS = 0x69;  
const uint8_t BMI160_GYRO_DATA_REG = 0x0C;  
const uint8_t BMI160_ACC_DATA_REG = 0x12;   
const uint8_t BMI160_PMU_STATUS = 0x03;  
const uint8_t BMI160_CMD_REG = 0x7E;  
const uint8_t BMI160_ACC_MODE_CMD = 0x11;

// Create SoftWire instances for each sensor
SoftWire sw[8] = {
  SoftWire(sdaPins[0], sclPins[0]),
  SoftWire(sdaPins[1], sclPins[1]),
  SoftWire(sdaPins[2], sclPins[2]),
  SoftWire(sdaPins[3], sclPins[3]),
  SoftWire(sdaPins[4], sclPins[4]),
  SoftWire(sdaPins[5], sclPins[5]),
  SoftWire(sdaPins[6], sclPins[6]),
  SoftWire(sdaPins[7], sclPins[7])
};

char swTxBuffer[8][16];
char swRxBuffer[8][16];

AsyncDelay readInterval;

const float gyroSensitivity = 250.0 / 32768.0;  
const float accelScale = 16384.0;  

// Sensor data structures
struct SensorData {
  float angleX, angleY, angleZ;
  float accelRoll, accelPitch;
  float rollOffset, pitchOffset;
  unsigned long lastTime;
};

SensorData sensors[8];

const float alpha = 0.90;  // **Lowered from 0.98 to 0.90 for faster response!**

void enableAccelerometer(SoftWire &sw) {
  Serial.println("🔧 Setting Accelerometer to Normal Mode...");
  sw.beginTransmission(I2C_ADDRESS);
  sw.write(BMI160_CMD_REG);
  sw.write(BMI160_ACC_MODE_CMD);
  int error = sw.endTransmission();
  if (error != 0) {
    Serial.println("❌ Error: Could not activate Accelerometer.");
    return;
  }
  delay(50);
}

bool isAccelerometerActive(SoftWire &sw) {
  sw.beginTransmission(I2C_ADDRESS);
  sw.write(BMI160_PMU_STATUS);
  if (sw.endTransmission(false) != 0) return false;

  sw.requestFrom(I2C_ADDRESS, (uint8_t)1);
  if (!sw.available()) return false;

  uint8_t pmu_status = sw.read();
  return ((pmu_status & 0x30) == 0x10);
}

bool readAccelerometer(SoftWire &sw, SensorData &sensor) {
  sw.beginTransmission(I2C_ADDRESS);
  sw.write(BMI160_ACC_DATA_REG);
  int error = sw.endTransmission();
  if (error != 0) return false;

  int numBytes = sw.requestFrom(I2C_ADDRESS, (uint8_t)6);
  if (numBytes != 6) return false;

  int16_t accX = (int16_t)(sw.read() | (sw.read() << 8));
  int16_t accY = (int16_t)(sw.read() | (sw.read() << 8));
  int16_t accZ = (int16_t)(sw.read() | (sw.read() << 8));

  float ax = accX / accelScale;
  float ay = accY / accelScale;
  float az = accZ / accelScale;

  if (ax == 0 && ay == 0 && az == 0) return false;

  sensor.accelRoll = atan2(ay, az) * 180.0 / PI;
  sensor.accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  return true;
}

void readGyroData(SoftWire &sw, SensorData &sensor) {
  sw.beginTransmission(I2C_ADDRESS);
  sw.write(BMI160_GYRO_DATA_REG);
  int error = sw.endTransmission();
  if (error != 0) return;

  int numBytes = sw.requestFrom(I2C_ADDRESS, (uint8_t)6);
  if (numBytes != 6) return;

  int16_t gyroX = (int16_t)(sw.read() | (sw.read() << 8));
  int16_t gyroY = (int16_t)(sw.read() | (sw.read() << 8));
  int16_t gyroZ = (int16_t)(sw.read() | (sw.read() << 8));

  float gyroX_dps = gyroX * gyroSensitivity;
  float gyroY_dps = gyroY * gyroSensitivity;
  float gyroZ_dps = gyroZ * gyroSensitivity;

  unsigned long currentTime = millis();
  float dt = (currentTime - sensor.lastTime) / 1000.0;
  sensor.lastTime = currentTime;

  float correctedRoll = sensor.accelRoll - sensor.rollOffset;
  float correctedPitch = sensor.accelPitch - sensor.pitchOffset;

  sensor.angleX = (alpha * (sensor.angleX + gyroX_dps * dt)) + ((1 - alpha) * correctedRoll);
  sensor.angleY = (alpha * (sensor.angleY + gyroY_dps * dt)) + ((1 - alpha) * correctedPitch);
  sensor.angleZ += gyroZ_dps * dt;

  //Serial.print(sensor.angleX + sensor.rollOffset);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 8; i++) {
    sw[i].setTxBuffer(swTxBuffer[i], sizeof(swTxBuffer[i]));
    sw[i].setRxBuffer(swRxBuffer[i], sizeof(swRxBuffer[i]));
    sw[i].setDelay_us(5);
    sw[i].setTimeout(1000);
    sw[i].begin();
  }

  Serial.println("🔄 Checking Accelerometer Mode...");
  for (int i = 0; i < 8; i++) {
    enableAccelerometer(sw[i]);
    
    int attempts = 0;
    while (!isAccelerometerActive(sw[i]) && attempts < 10) {
      Serial.println("⚠️ Waiting for Accelerometer...");
      delay(100);
      attempts++;
    }

    if (attempts >= 10) {
      Serial.println("❌ ERROR: Accelerometer did not start.");
    } else {
      Serial.println("✅ Accelerometer Ready.");
    }
  }

  Serial.println("🔄 Calibrating Initial Position...");
  for (int i = 0; i < 8; i++) {
    int attempts = 0;
    while (!readAccelerometer(sw[i], sensors[i]) && attempts < 10) {
      Serial.println("⚠️ Retrying accelerometer reading...");
      delay(100);
      attempts++;
    }

    if (attempts >= 10) {
      Serial.println("❌ ERROR: Could not get initial accelerometer reading.");
    } else {
      sensors[i].rollOffset = sensors[i].accelRoll;
      sensors[i].pitchOffset = sensors[i].accelPitch;
      Serial.print("✅ Roll Offset Stored: ");
      Serial.println(sensors[i].rollOffset);
      Serial.print("✅ Pitch Offset Stored: ");
      Serial.println(sensors[i].pitchOffset);
    }
  }

  readInterval.start(20, AsyncDelay::MILLIS);
  for (int i = 0; i < 8; i++) {
    sensors[i].lastTime = millis();
  }
}

void loop() {
  if (readInterval.isExpired()) {
    for (int i = 0; i < 8; i++) {
      if (readAccelerometer(sw[i], sensors[i])) {
        readGyroData(sw[i], sensors[i]);
        Serial.print(String(sensors[i].angleX) + ",");
        //Serial.print(String(sensors[i].angleX + sensors[i].rollOffset) + ",");
      }
    }
    Serial.println();
    readInterval.restart();
  }
}