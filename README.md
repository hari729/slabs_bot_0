# slabs_bot v0

A small differential-drive robot built to learn end-to-end ROS 2 control on real hardware. The stack runs inside a Dockerized Jazzy workspace and talks to an ESP32 over a framed serial protocol. `ros2_control`'s `diff_drive_controller` generates wheel velocities; a custom hardware interface converts them to RPM and streams them to the ESP32, which closes a PI loop around motor encoders.

## Quick start

```bash
# 1. Build the container, remove the override compose file if not use the dev stage.
# Edit the compose file if not use NVIDI gpu.
docker compose build

# 2. Start it
docker compose up -d

# 3. Enter the workspace
docker compose exec ros2 bash

# 4. Build the ROS 2 workspace
colcon build --symlink-install

# 5. Launch the robot (controllers + RViz)
source install/setup.bash
ros2 launch my_robot_bringup my_robot.launch.py
```

In a second terminal:

```bash
# Keyboard teleop
ros2 launch my_robot_bringup teleop_keyboard.launch.xml

# OR joystick teleop
ros2 launch my_robot_bringup teleop_joystick.launch.xml joy_dev:=/dev/input/js0
```

## Calibration workflow

1. **Measure ticks per revolution.** Use the `esp32_pid` project, spin the motor shaft manually
   and read encoder counts. The firmware uses `TICKS_PER_REVOLUTION = 1399.2`.
2. **Tune the PI loop.** `esp32_pid` runs a step-response test and prints good starting `Kp` and `Ki` values.
3. **Copy gains into firmware.** Paste `FF_DUTY_PER_RPM`, `PI_KP`, `PI_KI`, and `MIN_START_DUTY` into `esp32_firmware/src/main.cpp`.
4. **Upload firmware.** From `esp32_firmware/`, run `pio run -t upload`.
5. **Run the robot.** Launch the bringup and teleop packages.

## Hardware

### Pin map

| Function    | ESP32 GPIO | Notes                        |
| ----------- | ---------- | ---------------------------- |
| Left PWM    | 19         | LEDC channel 0               |
| Left DIR    | 18         |                              |
| Right PWM   | 26         | LEDC channel 1               |
| Right DIR   | 25         |                              |
| Left enc A  | 34         | Quadrature encoder           |
| Left enc B  | 35         |                              |
| Right enc A | 32         | Quadrature encoder           |
| Right enc B | 33         |                              |
| E-stop      | 4          | Active-low, internal pull-up |

### Materials

- ESP32-Dev
- 4x DC motor with quadrature encoder (Same input for both motors on a single side, encoder from any one motor.)
- 2x motor driver (H-bridge, PWM + DIR input)
- Differential-drive chassis
- USB cable for flashing and serial

## Architecture

```mermaid
flowchart LR
    A[Joystick / Keyboard] -->|/joy| B[teleop_twist_joy]
    B -->|TwistStamped| C[/diff_drive_controller/cmd_vel]
    C --> D[diff_drive_controller]
    D -->|wheel rad/s| E[esp32_hardware interface]
    E -->|VelocityCmd| F[ESP32]
    F -->|EncoderReport| E
    E -->|joint states| D
    F --> G[Motors + Encoders]
```

1. **Teleop** (`teleop_twist_keyboard` or `teleop_twist_joy`) publishes `TwistStamped`.
2. **diff_drive_controller** computes left/right wheel velocities in rad/s and exposes joint states.
3. **esp32_hardware** is a `ros2_control` `SystemInterface` plugin. It reads commands, sends a binary `VelocityCmd` frame, and parses `EncoderReport` frames.
4. **ESP32 firmware** runs the PI controller at 100 Hz, reports encoders at 50 Hz, and monitors a command watchdog.

## Engineering notes

### Why a custom binary protocol instead of text?

The original upstream project sent comma-separated strings. This repo uses a framed binary protocol:

```
[0xAA][0x55][type u8][len u8][payload...][crc16 u16]
```

- Fixed-size structs map directly to C/C++ `struct` with `#pragma pack` and `static_assert` checks.
- CRC-16/CCITT-FALSE catches byte drops and bit errors on the USB-serial line.
- A state-machine parser resyncs on the next valid sync word after a bad frame, so one corrupt byte does not desync the stream.

### Watchdog

If the ESP32 receives no valid `VelocityCmd` for 500 ms, it zeros the target RPM, resets the integrator, and stops the motors. This protects against the PC crashing or the serial cable being unplugged.

### PI controller

The firmware runs two independent PI loops at 100 Hz. A feedforward term (`FF_DUTY_PER_RPM`) provides the bulk of the command, while the PI term corrects steady-state error. An integral limit prevents windup, and a minimum start duty overcomes static friction.

Measured RPM is computed from encoder delta ticks, the calibrated TPR, and the 10 ms control period.

## Project structure

```text
.
├── Dockerfile                      # Jazzy base + dev container stages
├── docker-compose.yml              # Runtime container with device access
├── docker-compose.override.yml     # Dev stage with editors/debuggers
├── esp32_firmware/                 # Production FreeRTOS firmware
│   ├── include/protocol.h          # Shared binary protocol header
│   └── src/main.cpp                # PI loop, serial framing, watchdog
├── esp32_pid/                      # PID tuning helper firmware
├── src/
│   ├── esp32_hardware/             # ros2_control SystemInterface plugin
│   │   ├── include/esp32_hardware/
│   │   │   ├── esp32_hardware_interface.hpp
│   │   │   ├── esp32_hardware_driver.hpp
│   │   │   └── protocol.h
│   │   └── src/
│   │       ├── esp32_hardware_interface.cpp
│   │       └── esp32_hardware_driver.cpp
│   ├── my_robot_bringup/           # Launch files and controller config
│   │   ├── config/my_robot_controllers.yaml
│   │   ├── launch/my_robot.launch.py
│   │   ├── launch/teleop_keyboard.launch.xml
│   │   └── launch/teleop_joystick.launch.xml
│   └── my_robot_description/       # URDF / xacro
│       └── urdf/
└── README.md                       # This file
```

## License

Apache License
