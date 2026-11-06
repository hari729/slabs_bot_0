# MeArm Arduino Firmware

PlatformIO firmware for the MeArm mounted on the diff-drive base.

## Pin map

| Function       | Arduino pin           |
| -------------- | --------------------- |
| Base servo     | 6                     |
| Shoulder servo | 9                     |
| Elbow servo    | 10                    |
| Claw servo     | 11                    |
| Setup mode     | 2 (pull LOW to enter) |

## Servo conventions

- **Base**: continuous-rotation servo. Command `joint_0` is speed: `-1.0` full reverse, `0.0` stop, `+1.0` full forward.
- **Shoulder / elbow**: position servos. Command `joint_1` and `joint_2` are physical servo angles in radians, range `0 .. pi` (`0 deg .. 180 deg`). `pi/2` (90 deg) is the attach/reference position.
- **Claw**: position servo. Command `joint_3` is physical servo angle in radians, range `1.39 .. 1.6581 rad` (`80 deg` closed, `95 deg` open).

## Setup mode

Pull pin 2 to ground and reset the Arduino. All servos move to the physical 90° / stop position so the arm links and claw can be attached. Reset again (with pin 2 floating) to enter normal mode.

## Serial protocol

Same framed binary protocol as the ESP32 base firmware:

```
[0xAA][0x55][type u8][len u8][payload...][crc16 u16]
```

Packet type `0x10` (`ServoCmdPacket`) carries four little-endian floats:

```c
struct ServoCmd {
  float joint_0;  // base speed
  float joint_1;  // shoulder physical angle (rad)
  float joint_2;  // elbow physical angle (rad)
  float joint_3;  // claw physical angle (rad)
};
```

A 500 ms command watchdog stops the base if no valid frame is received.

## Build and upload

```bash
cd mearm_arduino_firmware
pio run -t upload
pio device monitor
```
