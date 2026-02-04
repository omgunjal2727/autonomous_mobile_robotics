/*
 * ESP32 Motor Control Bridge
 * 
 * Receives velocity commands from ROS2 via serial (JSON format)
 * and controls differential drive motors.
 * 
 * Hardware:
 * - ESP32 Dev Board
 * - Motor Driver (L298N, TB6612, or similar)
 * - Two DC motors for differential drive
 * 
 * Serial Protocol:
 * - Baud: 115200
 * - Format: JSON
 * - Example: {"linear": 0.5, "angular": 0.2}
 *   - linear: forward/backward velocity in m/s (-1.0 to 1.0)
 *   - angular: rotational velocity in rad/s (-2.0 to 2.0)
 */

#include <ArduinoJson.h>

// ===== CONFIGURATION =====
// Motor driver pins (adjust for your hardware)
const int MOTOR_LEFT_PWM = 25;    // Left motor PWM
const int MOTOR_LEFT_DIR1 = 26;   // Left motor direction pin 1
const int MOTOR_LEFT_DIR2 = 27;   // Left motor direction pin 2

const int MOTOR_RIGHT_PWM = 32;   // Right motor PWM
const int MOTOR_RIGHT_DIR1 = 33;  // Right motor direction pin 1
const int MOTOR_RIGHT_DIR2 = 14;  // Right motor direction pin 2

// Robot physical parameters
const float WHEEL_SEPARATION = 0.45;  // Distance between wheels (meters)
const float WHEEL_RADIUS = 0.1;       // Wheel radius (meters)
const float MAX_LINEAR_SPEED = 1.0;   // Max linear speed (m/s)
const float MAX_ANGULAR_SPEED = 2.0;  // Max angular speed (rad/s)

// PWM configuration
const int PWM_FREQUENCY = 1000;  // 1 kHz
const int PWM_RESOLUTION = 8;    // 8-bit (0-255)
const int PWM_CHANNEL_LEFT = 0;
const int PWM_CHANNEL_RIGHT = 1;

// Safety timeout (milliseconds)
const unsigned long CMD_TIMEOUT = 500;  // Stop if no command for 500ms
unsigned long lastCmdTime = 0;

// ===== SETUP =====
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure motor pins
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHANNEL_RIGHT);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("ESP32 Motor Bridge Ready");
  Serial.println("Waiting for velocity commands...");
}

// ===== MAIN LOOP =====
void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    String jsonString = Serial.readStringUntil('\n');
    
    // Parse JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (error) {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Extract velocities
    float linear = doc["linear"] | 0.0;   // Default to 0 if not present
    float angular = doc["angular"] | 0.0;
    
    // Constrain to safe limits
    linear = constrain(linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    angular = constrain(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    
    // Update last command time
    lastCmdTime = millis();
    
    // Control motors
    setVelocity(linear, angular);
    
    // Debug output
    Serial.print("Cmd: L=");
    Serial.print(linear, 3);
    Serial.print(" A=");
    Serial.println(angular, 3);
  }
  
  // Safety check - stop if no command received for timeout period
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    stopMotors();
  }
  
  delay(10);  // Small delay to prevent overwhelming the CPU
}

// ===== MOTOR CONTROL FUNCTIONS =====

/**
 * Convert linear and angular velocities to wheel speeds
 * and control the motors accordingly
 */
void setVelocity(float linear, float angular) {
  // Differential drive kinematics
  // v_left = linear - (angular * wheel_separation / 2)
  // v_right = linear + (angular * wheel_separation / 2)
  
  float leftSpeed = linear - (angular * WHEEL_SEPARATION / 2.0);
  float rightSpeed = linear + (angular * WHEEL_SEPARATION / 2.0);
  
  // Convert m/s to PWM (0-255)
  // Assuming max speed corresponds to max PWM
  int leftPWM = abs(leftSpeed / MAX_LINEAR_SPEED * 255);
  int rightPWM = abs(rightSpeed / MAX_LINEAR_SPEED * 255);
  
  // Constrain PWM values
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);
  
  // Set motor directions and speeds
  setMotor(MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, PWM_CHANNEL_LEFT, leftSpeed, leftPWM);
  setMotor(MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, PWM_CHANNEL_RIGHT, rightSpeed, rightPWM);
}

/**
 * Control a single motor
 */
void setMotor(int dir1Pin, int dir2Pin, int pwmChannel, float speed, int pwm) {
  if (speed > 0.01) {
    // Forward
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    ledcWrite(pwmChannel, pwm);
  } else if (speed < -0.01) {
    // Backward
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    ledcWrite(pwmChannel, pwm);
  } else {
    // Stop
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

/**
 * Stop all motors
 */
void stopMotors() {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
}
