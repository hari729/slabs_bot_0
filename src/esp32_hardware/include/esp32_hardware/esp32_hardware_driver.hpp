#ifndef ESP32_HARDWARE_DRIVER_HPP
#define ESP32_HARDWARE_DRIVER_HPP

/**
 * @file esp32_hardware_driver.hpp
 * @brief Controller-side driver for the ESP32 wheel bridge.
 *
 * Handles the USB serial protocol and all unit conversions between
 * ROS/joint-space (rad, rad/s) and the ESP32 firmware protocol (ticks, RPM).
 *
 * The ESP32 firmware sends EncoderReport frames and accepts VelocityCmd frames.
 * See protocol.h for the wire format and struct definitions.
 */

#include <cstdint>
#include <memory>
#include <string>

#include "esp32_hardware/protocol.h"

namespace esp32_hardware
{

enum class WheelSide { Left, Right };

class Esp32HardwareDriver
{
public:
  Esp32HardwareDriver(const std::string & port, int baud_rate, double ticks_per_revolution);
  ~Esp32HardwareDriver();

  /**
   * @brief Open the serial port and configure termios.
   * @return true on success
   */
  bool init();

  /** @brief Prepare the driver for active control. Resets tick history. */
  void activate();

  /** @brief Send a zero-velocity command before stopping. */
  void deactivate();

  /** @brief Close the serial port. */
  void close();

  /** @brief True if the serial port is open. */
  bool isConnected() const;

  /**
   * @brief Poll available serial bytes and update position/velocity estimates.
   * @param period_sec Control-loop period in seconds.
   */
  void update(double period_sec);

  /** @brief Send per-wheel velocity targets in rad/s. */
  void setTargetVelocityRadianPerSec(double left, double right);

  /** @brief Latest wheel position in radians. */
  double getPositionRadian(WheelSide side) const;

  /** @brief Latest wheel velocity in rad/s. */
  double getVelocityRadianPerSec(WheelSide side) const;

private:
  bool openPort();
  void readAvailableBytes();
  bool parseLatestEncoderReport(long & left_ticks, long & right_ticks);
  bool sendVelocityCommand(double left_rpm, double right_rpm);

  double ticksToRadians(long ticks) const;
  double ticksToRadPerSec(long tick_delta, double period_sec) const;
  double radPerSecToRpm(double rad_per_sec) const;

  std::string port_;
  int baud_rate_;
  double ticks_per_revolution_;

  int fd_{-1};
  esp32_protocol::FrameParser frame_parser_;
  bool have_latest_report_{false};
  esp32_protocol::EncoderReport latest_report_{};

  double left_position_{0.0};
  double right_position_{0.0};
  double left_velocity_{0.0};
  double right_velocity_{0.0};

  long prev_left_ticks_{0};
  long prev_right_ticks_{0};
  bool have_prev_ticks_{false};
};

}  // namespace esp32_hardware

#endif  // ESP32_HARDWARE_DRIVER_HPP
