# slabs bot v0

Learning project for integrating ros2 with physical robot.

## Hardware Setup

Measure Ticks Per Revolution (TPR) for you encoder by rotating the motor shaft after properly wiring the motor encoder.
You could use one of the esp32 firware code and look at serial monitor for this.
Once we get TPR, use the esp32_pid project to get valid constants for the PI loop for your hardware.
Then add those constants into the esp32_firmware file and upload it.
