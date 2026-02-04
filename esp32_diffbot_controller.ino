/*
 * ESP32 Differential Drive Robot Controller
 * 
 * This sketch handles:
 * - Reading MPU6050 IMU data and sending to ROS2
 * - Receiving velocity commands from ROS2
 * - Controlling motor speeds based on velocity commands
 * 
 * Communication Protocol: JSON over Serial (115200 baud)
 * 
 * Receives from ROS2: {"linear": 0.5, "angular": 0.2}
 * Sends to ROS2: {"ax": 0.1, "ay": 0.0, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.0}
 */

#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// MPU6050 sensor
MPU6050 mpu;

// Motor pins (adjust based on your motor driver)
#define LEFT_MOTOR_PWM 25
#define LEFT_MOTOR_DIR1 26
#define LEFT_MOTOR_DIR2 27
#define RIGHT_MOTOR_PWM 32
#define RIGHT_MOTOR_DIR1 33
#define RIGHT_MOTOR_DIR2 34

// Robot parameters
const float WHEEL_SEPARATION = 0.45;  // meters (match URDF)
const float WHEEL_RADIUS = 0.1;       // meters (match URDF)
const int PWM_FREQUENCY = 1000;       // Hz
const int PWM_RESOLUTION = 8;         // 8-bit (0-255)

// Timing
unsigned long lastIMUPublish = 0;
const unsigned long IMU_PUBLISH_INTERVAL = 50;  // 20 Hz

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for MPU6050
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("{\"error\": \"MPU6050 connection failed\"}");
  }
  
  // Configure motor pins
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
  
  // Configure PWM
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);  // Left motor
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);  // Right motor
  ledcAttachPin(LEFT_MOTOR_PWM, 0);
  ledcAttachPin(RIGHT_MOTOR_PWM, 1);
  
  Serial.println("{\"status\": \"ESP32 ready\"}");
}

void loop() {
  // Read and publish IMU data
  if (millis() - lastIMUPublish >= IMU_PUBLISH_INTERVAL) {
    publishIMUData();
    lastIMUPublish = millis();
  }
  
  // Check for incoming velocity commands
  if (Serial.available() > 0) {
    String jsonString = Serial.readStringUntil('\n');
    processVelocityCommand(jsonString);
  }
}

void publishIMUData() {
  // Read raw IMU data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  // Convert to SI units
  // MPU6050 default scale: ±2g for accel, ±250°/s for gyro
  float accel_x = ax / 16384.0 * 9.81;  // m/s^2
  float accel_y = ay / 16384.0 * 9.81;
  float accel_z = az / 16384.0 * 9.81;
  float gyro_x = gx / 131.0 * 0.0174533;  // rad/s
  float gyro_y = gy / 131.0 * 0.0174533;
  float gyro_z = gz / 131.0 * 0.0174533;
  
  // Create JSON message
  StaticJsonDocument<200> doc;
  doc["ax"] = accel_x;
  doc["ay"] = accel_y;
  doc["az"] = accel_z;
  doc["gx"] = gyro_x;
  doc["gy"] = gyro_y;
  doc["gz"] = gyro_z;
  
  // Send to serial
  serializeJson(doc, Serial);
  Serial.println();
}

void processVelocityCommand(String jsonString) {
  // Parse JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    return;  // Invalid JSON, ignore
  }
  
  // Extract velocities
  float linear_vel = doc["linear"];   // m/s
  float angular_vel = doc["angular"]; // rad/s
  
  // Convert to wheel velocities (differential drive kinematics)
  float left_wheel_vel = (linear_vel - (angular_vel * WHEEL_SEPARATION / 2.0)) / WHEEL_RADIUS;
  float right_wheel_vel = (linear_vel + (angular_vel * WHEEL_SEPARATION / 2.0)) / WHEEL_RADIUS;
  
  // Set motor speeds
  setMotorSpeed(0, left_wheel_vel);   // Left motor (channel 0)
  setMotorSpeed(1, right_wheel_vel);  // Right motor (channel 1)
}

void setMotorSpeed(int channel, float wheel_velocity) {
  // Convert wheel velocity (rad/s) to PWM (0-255)
  // Adjust MAX_WHEEL_VELOCITY based on your motors
  const float MAX_WHEEL_VELOCITY = 10.0;  // rad/s
  
  int pwm = abs(wheel_velocity) / MAX_WHEEL_VELOCITY * 255;
  pwm = constrain(pwm, 0, 255);
  
  // Set direction and PWM
  if (channel == 0) {  // Left motor
    if (wheel_velocity >= 0) {
      digitalWrite(LEFT_MOTOR_DIR1, HIGH);
      digitalWrite(LEFT_MOTOR_DIR2, LOW);
    } else {
      digitalWrite(LEFT_MOTOR_DIR1, LOW);
      digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    }
    ledcWrite(0, pwm);
  } else {  // Right motor
    if (wheel_velocity >= 0) {
      digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
      digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    } else {
      digitalWrite(RIGHT_MOTOR_DIR1, LOW);
      digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    }
    ledcWrite(1, pwm);
  }
}
