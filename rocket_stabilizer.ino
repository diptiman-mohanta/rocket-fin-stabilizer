// Rocket Fin Stabilization System - Arduino Mega 2560
// Uses MPU6050 for attitude sensing and servo control for fin actuation

#include <Wire.h>
#include <Servo.h>

// MPU6050 I2C addresses and registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

// Hardware pin definitions for Arduino Mega
const uint8_t FIN_SERVO_PINS[4] = {8, 9, 10, 11};

// Servo control objects
Servo finServos[4];

// System variables
struct MotionData {
  float pitch;
  float roll;
  float yaw;
} currentAttitude;

struct CalibrationData {
  int16_t accelBias[3];
  int16_t gyroBias[3];
} sensorOffsets;

unsigned long lastUpdateTime = 0;
const uint16_t UPDATE_INTERVAL = 50; // 20Hz update rate
const uint8_t SERVO_CENTER = 90;
const float ANGLE_DEADBAND = 0.5;

void setup() {
  initializeSystem();
  setupIMU();
  attachFinServos();
  performSystemCheck();
}

void loop() {
  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    updateSensorData();
    computeStabilization();
    lastUpdateTime = millis();
  }
}

void initializeSystem() {
  Serial.begin(115200);
  Serial.println("Rocket Stabilization System Starting...");
}

void setupIMU() {
  Wire.begin();
  Wire.setClock(400000); // Fast I2C mode
  
  // Initialize MPU6050
  writeIMURegister(PWR_MGMT_1, 0x00);
  delay(100);
  
  // Configure accelerometer and gyroscope
  writeIMURegister(0x1C, 0x00); // ±2g accelerometer range
  writeIMURegister(0x1B, 0x00); // ±250°/s gyroscope range
  
  calibrateSensors();
}

void attachFinServos() {
  for (int i = 0; i < 4; i++) {
    finServos[i].attach(FIN_SERVO_PINS[i]);
    finServos[i].write(SERVO_CENTER);
    delay(100);
  }
}

void performSystemCheck() {
  Serial.println("System Check Complete - Ready for Flight");
}

void updateSensorData() {
  int16_t rawData[6];
  readIMUData(rawData);
  
  // Apply calibration offsets
  for (int i = 0; i < 3; i++) {
    rawData[i] -= sensorOffsets.accelBias[i];
    rawData[i+3] -= sensorOffsets.gyroBias[i];
  }
  
  // Convert raw accelerometer data to angles
  float accelX = rawData[0] / 16384.0;  // Convert to g-force
  float accelY = rawData[1] / 16384.0;
  float accelZ = rawData[2] / 16384.0;
  
  // Calculate pitch and roll from accelerometer
  currentAttitude.pitch = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 180.0 / PI;
  currentAttitude.roll = atan2(-accelX, accelZ) * 180.0 / PI;
  
  // Debug output every second
  if (millis() % 1000 < UPDATE_INTERVAL) {
    Serial.print("Pitch: "); Serial.print(currentAttitude.pitch, 2);
    Serial.print(" | Roll: "); Serial.println(currentAttitude.roll, 2);
  }
}

void computeStabilization() {
  uint8_t servoPositions[4];
  
  // Calculate servo positions based on attitude
  if (abs(currentAttitude.pitch) < ANGLE_DEADBAND && abs(currentAttitude.roll) < ANGLE_DEADBAND) {
    // Stable - center all servos
    for (int i = 0; i < 4; i++) {
      servoPositions[i] = SERVO_CENTER;
    }
  } else {
    // Apply correction
    servoPositions[0] = constrain(SERVO_CENTER + currentAttitude.roll, 0, 180);  // Right fin
    servoPositions[1] = constrain(SERVO_CENTER - currentAttitude.roll, 0, 180);  // Left fin
    servoPositions[2] = constrain(SERVO_CENTER + currentAttitude.pitch, 0, 180); // Top fin
    servoPositions[3] = constrain(SERVO_CENTER - currentAttitude.pitch, 0, 180); // Bottom fin
  }
  
  // Command servos
  for (int i = 0; i < 4; i++) {
    finServos[i].write(servoPositions[i]);
  }
}

void calibrateSensors() {
  Serial.println("Calibrating sensors - keep rocket level and still");
  delay(2000); // Give time to position rocket
  
  long accelSum[3] = {0, 0, 0};
  long gyroSum[3] = {0, 0, 0};
  const int samples = 500;
  
  Serial.println("Calibration in progress...");
  
  for (int i = 0; i < samples; i++) {
    int16_t rawData[6];
    readIMUData(rawData);
    
    for (int j = 0; j < 3; j++) {
      accelSum[j] += rawData[j];
      gyroSum[j] += rawData[j+3];
    }
    
    if (i % 100 == 0) {
      Serial.print(".");
    }
    delay(3);
  }
  
  Serial.println();
  
  for (int i = 0; i < 3; i++) {
    sensorOffsets.accelBias[i] = accelSum[i] / samples;
    sensorOffsets.gyroBias[i] = gyroSum[i] / samples;
  }
  
  // Adjust Z-axis bias for 1g offset
  sensorOffsets.accelBias[2] -= 16384;
  
  Serial.println("Calibration complete");
}

void readIMUData(int16_t* data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  // Read accelerometer data (X, Y, Z)
  data[0] = (Wire.read() << 8) | Wire.read(); // ACCEL_X
  data[1] = (Wire.read() << 8) | Wire.read(); // ACCEL_Y  
  data[2] = (Wire.read() << 8) | Wire.read(); // ACCEL_Z
  
  // Skip temperature readings
  Wire.read(); Wire.read();
  
  // Read gyroscope data (X, Y, Z)
  data[3] = (Wire.read() << 8) | Wire.read(); // GYRO_X
  data[4] = (Wire.read() << 8) | Wire.read(); // GYRO_Y
  data[5] = (Wire.read() << 8) | Wire.read(); // GYRO_Z
}

void writeIMURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
