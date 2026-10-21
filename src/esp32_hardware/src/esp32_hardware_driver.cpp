/**
 * @file esp32_hardware_driver.cpp
 * @brief Implementation of the ESP32 wheel bridge driver.
 */

#include "esp32_hardware/esp32_hardware_driver.hpp"

#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace esp32_hardware
{

Esp32HardwareDriver::Esp32HardwareDriver(
  const std::string & port, int baud_rate, double ticks_per_revolution)
: port_(port),
  baud_rate_(baud_rate),
  ticks_per_revolution_(ticks_per_revolution)
{
}

Esp32HardwareDriver::~Esp32HardwareDriver()
{
  close();
}

bool Esp32HardwareDriver::init()
{
  return openPort();
}

void Esp32HardwareDriver::activate()
{
  have_prev_ticks_ = false;
  left_velocity_ = 0.0;
  right_velocity_ = 0.0;
  sendVelocityCommand(0.0, 0.0);
}

void Esp32HardwareDriver::deactivate()
{
  sendVelocityCommand(0.0, 0.0);
}

void Esp32HardwareDriver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  frame_parser_.reset();
  have_latest_report_ = false;
}

bool Esp32HardwareDriver::isConnected() const
{
  return fd_ >= 0;
}

void Esp32HardwareDriver::update(double period_sec)
{
  readAvailableBytes();

  long left_ticks = 0;
  long right_ticks = 0;

  if (parseLatestEncoderReport(left_ticks, right_ticks)) {
    left_position_ = ticksToRadians(left_ticks);
    right_position_ = ticksToRadians(right_ticks);

    if (have_prev_ticks_ && period_sec > 0.0) {
      left_velocity_ = ticksToRadPerSec(left_ticks - prev_left_ticks_, period_sec);
      right_velocity_ = ticksToRadPerSec(right_ticks - prev_right_ticks_, period_sec);
    }

    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;
    have_prev_ticks_ = true;
  }
}

void Esp32HardwareDriver::setTargetVelocityRadianPerSec(double left, double right)
{
  sendVelocityCommand(radPerSecToRpm(left), radPerSecToRpm(right));
}

double Esp32HardwareDriver::getPositionRadian(WheelSide side) const
{
  return side == WheelSide::Left ? left_position_ : right_position_;
}

double Esp32HardwareDriver::getVelocityRadianPerSec(WheelSide side) const
{
  return side == WheelSide::Left ? left_velocity_ : right_velocity_;
}

bool Esp32HardwareDriver::openPort()
{
  close();

  // O_NONBLOCK lets us poll without blocking the control loop.
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  // Map numeric baud rate to termios constant
  speed_t speed = B115200;
  if (baud_rate_ == 921600) {
    speed = B921600;
  } else if (baud_rate_ == 57600) {
    speed = B57600;
  } else if (baud_rate_ == 115200) {
    speed = B115200;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, raw mode
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  return true;
}

void Esp32HardwareDriver::readAvailableBytes()
{
  if (fd_ < 0) {
    return;
  }

  uint8_t buffer[256];
  while (true) {
    ssize_t n = ::read(fd_, buffer, sizeof(buffer));
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        const auto result = frame_parser_.feed(buffer[i]);
        if (result == esp32_protocol::FrameParser::Result::FrameReady) {
          if (frame_parser_.type() == esp32_protocol::PacketType::EncoderReport &&
              frame_parser_.length() == sizeof(esp32_protocol::EncoderReport))
          {
            std::memcpy(&latest_report_, frame_parser_.payload(), sizeof(latest_report_));
            have_latest_report_ = true;
          }
          frame_parser_.reset();
        } else if (result == esp32_protocol::FrameParser::Result::InvalidFrame) {
          frame_parser_.reset();
        }
      }
    } else {
      break;
    }
  }
}

bool Esp32HardwareDriver::parseLatestEncoderReport(long & left_ticks, long & right_ticks)
{
  if (!have_latest_report_) {
    return false;
  }

  left_ticks = static_cast<long>(latest_report_.left_ticks);
  right_ticks = static_cast<long>(latest_report_.right_ticks);
  have_latest_report_ = false;
  return true;
}

bool Esp32HardwareDriver::sendVelocityCommand(double left_rpm, double right_rpm)
{
  if (fd_ < 0) {
    return false;
  }

  esp32_protocol::VelocityCmd cmd;
  cmd.left_rpm = static_cast<float>(left_rpm);
  cmd.right_rpm = static_cast<float>(right_rpm);

  uint8_t frame[sizeof(cmd) + 6];
  const uint16_t frame_len = esp32_protocol::frame_pack(
      esp32_protocol::PacketType::VelocityCmd, &cmd, sizeof(cmd), frame);

  ssize_t written = ::write(fd_, frame, frame_len);
  return written == static_cast<ssize_t>(frame_len);
}

double Esp32HardwareDriver::ticksToRadians(long ticks) const
{
  return (static_cast<double>(ticks) / ticks_per_revolution_) * 2.0 * M_PI;
}

double Esp32HardwareDriver::ticksToRadPerSec(long tick_delta, double period_sec) const
{
  return ticksToRadians(tick_delta) / period_sec;
}

double Esp32HardwareDriver::radPerSecToRpm(double rad_per_sec) const
{
  return rad_per_sec * 60.0 / (2.0 * M_PI);
}

}  // namespace esp32_hardware
