#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float angleZ = 0;
unsigned long lastTime = 0;

// Gyroscope calibration offsets
float gyroZoffset = 0;

void calibrateGyro() {
  int numSamples = 500;
  float sumZ = 0;

  Serial.println("Calibrating gyro... DO NOT MOVE SENSOR");
  delay(1000);

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    delay(10);
  }

  gyroZoffset = sumZ / numSamples;

  Serial.println("Calibration complete!");
}

void setup() {
  Wire.begin();
  Serial.begin(115200); // Faster baud rate
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  calibrateGyro(); // Perform calibration
  lastTime = millis();
}

float gyro_value(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  // Apply calibration offsets
  float gyroZ = g.gyro.z - gyroZoffset;
  // Apply a deadband to reduce drift when the sensor is stationary
  if (fabs(gyroZ) < 0.02) {  // threshold; adjust based on your observations
  gyroZ = 0;
  }
  angleZ += gyroZ * dt * (180.0 / PI);
  return angleZ;
}

void loop() {

  Serial.print("Z-Axis Angle: ");
  Serial.println(gyro_value(), 1); // 1 decimal place
  delay(10); // Reduced delay for faster sampling
}
