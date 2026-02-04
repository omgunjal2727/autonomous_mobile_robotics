# ESP32 Firmware for Motor Control

This Arduino sketch runs on the ESP32 and controls the robot's motors based on velocity commands received from ROS2.

## Hardware Requirements

- ESP32 Development Board
- Motor Driver (L298N, TB6612FNG, or similar)
- Two DC motors (for differential drive)
- Power supply for motors

## Installation

### 1. Install Arduino IDE

Download from: https://www.arduino.cc/en/software

### 2. Install ESP32 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. Add this URL to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools → Board → Boards Manager**
5. Search for "esp32" and install "esp32 by Espressif Systems"

### 3. Install ArduinoJson Library

1. Go to **Sketch → Include Library → Manage Libraries**
2. Search for "ArduinoJson"
3. Install version 6.x (not 7.x)

## Configuration

### Pin Configuration

Edit the pin definitions in `motor_control.ino` to match your motor driver:

```cpp
const int MOTOR_LEFT_PWM = 25;
const int MOTOR_LEFT_DIR1 = 26;
const int MOTOR_LEFT_DIR2 = 27;

const int MOTOR_RIGHT_PWM = 32;
const int MOTOR_RIGHT_DIR1 = 33;
const int MOTOR_RIGHT_DIR2 = 14;
```

### Robot Parameters

Adjust these to match your robot:

```cpp
const float WHEEL_SEPARATION = 0.45;  // Distance between wheels (meters)
const float WHEEL_RADIUS = 0.1;       // Wheel radius (meters)
const float MAX_LINEAR_SPEED = 1.0;   // Max speed (m/s)
```

## Upload to ESP32

1. Connect ESP32 to your computer via USB
2. Open `motor_control.ino` in Arduino IDE
3. Select **Tools → Board → ESP32 Dev Module**
4. Select **Tools → Port → [Your ESP32 Port]**
5. Click **Upload** button

## Testing

### Serial Monitor Test

1. Open **Tools → Serial Monitor**
2. Set baud rate to **115200**
3. Send test commands:
   ```json
   {"linear": 0.2, "angular": 0.0}
   {"linear": 0.0, "angular": 0.5}
   {"linear": 0.0, "angular": 0.0}
   ```

### Motor Wiring

**L298N Example:**
```
ESP32 Pin 25 → L298N ENA (Left Motor PWM)
ESP32 Pin 26 → L298N IN1 (Left Motor Direction 1)
ESP32 Pin 27 → L298N IN2 (Left Motor Direction 2)
ESP32 Pin 32 → L298N ENB (Right Motor PWM)
ESP32 Pin 33 → L298N IN3 (Right Motor Direction 1)
ESP32 Pin 14 → L298N IN4 (Right Motor Direction 2)

L298N OUT1, OUT2 → Left Motor
L298N OUT3, OUT4 → Right Motor
```

**Important:** Connect motor driver power separately, not from ESP32!

## Troubleshooting

### Motors Not Moving

- Check power supply to motor driver
- Verify pin connections
- Test motors directly with motor driver
- Check serial monitor for error messages

### Wrong Direction

Swap the direction pins in code:
```cpp
// If left motor goes backward when it should go forward:
const int MOTOR_LEFT_DIR1 = 27;  // Swap these
const int MOTOR_LEFT_DIR2 = 26;  // two values
```

### Jerky Movement

- Increase PWM frequency: `const int PWM_FREQUENCY = 5000;`
- Add motor driver capacitors
- Check power supply stability

## Safety Features

- **Timeout**: Motors stop if no command received for 500ms
- **Speed Limiting**: Velocities constrained to safe maximums
- **Deadband**: Small speeds (<0.01) treated as stop

## License

Apache-2.0
