# CubeMars Motor Control with MCP2515 and ESP32

This repository contains Arduino sketches for controlling CubeMars motors using the MCP2515 CAN bus controller interfaced with an ESP32 microcontroller. Two versions of the code are provided: one for controlling a single motor (`cubemars_one_motor.ino`) and another for controlling multiple motors (`cubemars_mult_motor.ino`).

## Hardware Setup

### ESP32 and MCP2515 Pin Connections
The MCP2515 CAN bus module is connected to the ESP32 with the following pin configuration:
- **INT**: GPIO4
- **SCK**: GPIO18
- **SI**: GPIO23
- **SO**: GPIO19
- **CS**: GPIO5
- **GND**: GND
- **VCC**: 5V

Ensure proper connections between the ESP32 and the MCP2515 module, and connect the CAN bus (CAN_H and CAN_L) to the CubeMars motor's CAN interface.

### Required Library
The code uses the MCP2515 library, which can be downloaded from:
[https://github.com/autowp/arduino-mcp2515](https://github.com/autowp/arduino-mcp2515)

Install the library in your Arduino IDE by adding the `.zip` file or cloning the repository into your Arduino libraries folder.

## Code Overview

### `cubemars_one_motor.ino`
This sketch is designed to control a single CubeMars motor via the CAN bus.
- **Functionality**:
  - Initializes the MCP2515 CAN controller at 1000 kbps with an 8 MHz clock.
  - Enters motor mode to enable the motor.
  - Sends sinusoidal position commands to the motor, constrained between -12.5 and 12.5 radians.
  - Receives and unpacks motor feedback (position, velocity, and torque).
  - Supports commands to enter/exit motor mode and zero the motor.
- **Key Parameters**:
  - Position: -12.5 to 12.5 radians (16-bit)
  - Velocity: -30 to 30 rad/s (12-bit)
  - Torque: -18 to 18 N-m (12-bit)
  - Kp (position gain): 0 to 500 N-m/rad (12-bit)
  - Kd (velocity gain): 0 to 5 N-m*s/rad (12-bit)

### `cubemars_mult_motor.ino`
This sketch extends the single-motor control to handle multiple CubeMars motors (default: 2 motors).
- **Functionality**:
  - Manages multiple motors with unique CAN IDs (e.g., 0x00 and 0x01).
  - Initializes the MCP2515 CAN controller similarly to the single-motor version.
  - Sends sinusoidal position commands to each motor, scaled differently based on motor index.
  - Receives and unpacks feedback for each motor based on their CAN ID.
  - Supports motor mode entry/exit and zeroing for each motor.
- **Key Parameters**: Same as the single-motor version, applied per motor.

### CAN Communication
Both sketches use the MCP2515 to communicate with CubeMars motors over the CAN bus. The CAN packet structure for commands and replies is as follows:
- **Command Packet** (8 bytes):
  - Position (16-bit): Bytes 0-1
  - Velocity (12-bit): Bytes 2-3
  - Kp (12-bit): Bytes 3-4
  - Kd (12-bit): Bytes 5-6
  - Torque (12-bit): Bytes 6-7
- **Reply Packet** (5 bytes):
  - Position (16-bit): Bytes 0-1
  - Velocity (12-bit): Bytes 2-3
  - Current (12-bit): Bytes 3-4

### Utility Functions
- `float_to_uint` and `uint_to_float`: Convert between float values and unsigned integers for CAN communication, respecting bit resolution and range constraints.
- `pack_cmd`: Packs motor control parameters into a CAN command packet.
- `unpack_reply`: Unpacks motor feedback from a CAN reply packet.
- `EnterMotorMode`, `ExitMotorMode`, `Zero`: Send specific CAN commands to control motor state.

## Usage
1. **Install the Library**:
   - Download and install the MCP2515 library from [https://github.com/autowp/arduino-mcp2515](https://github.com/autowp/arduino-mcp2515).
2. **Connect Hardware**:
   - Wire the MCP2515 to the ESP32 as described in the pin connections section.
   - Connect the CAN bus to the CubeMars motor(s).
3. **Upload Code**:
   - Open `cubemars_one_motor.ino` or `cubemars_mult_motor.ino` in the Arduino IDE.
   - Select your ESP32 board and upload the sketch.
4. **Monitor Output**:
   - Open the Serial Monitor (115200 baud) to verify CAN bus initialization and motor status.
5. **Customize**:
   - Adjust motor parameters (e.g., `kp_in`, `kd_in`, `p_in`) or CAN IDs in the code as needed.
   - For `cubemars_mult_motor.ino`, modify `NUM_MOTORS` and the `motors` array to match the number of motors and their CAN IDs.

## Notes
- Ensure the CAN bus is properly terminated with 120-ohm resistors at both ends.
- The sinusoidal position commands are for demonstration. Modify the `loop` function to implement your desired control logic.
- The code assumes the CubeMars motor is configured to respond to the specified CAN commands. Refer to the motor's documentation for details.
- For multiple motors, ensure each motor has a unique CAN ID to avoid communication conflicts.

## License
This project is provided as-is for educational and development purposes. Refer to the MCP2515 library's license for its terms of use.