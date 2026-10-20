/**
 * @file esp32_hardware_driver.cpp
 * @brief Implementation of the ESP32 wheel bridge driver.
 */

#include "esp32_hardware/esp32_hardware_driver.hpp"

#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
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
  read_buffer_.clear();
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

  if (parseLatestEncoderLine(left_ticks, right_ticks)) {
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

  char buffer[256];
  while (true) {
    ssize_t n = ::read(fd_, buffer, sizeof(buffer));
    if (n > 0) {
      read_buffer_.append(buffer, static_cast<size_t>(n));
    } else {
      break;
    }
  }
}

bool Esp32HardwareDriver::parseLatestEncoderLine(long & left_ticks, long & right_ticks)
{
  bool found = false;

  size_t newline_pos = read_buffer_.find('\n');
  while (newline_pos != std::string::npos) {
    std::string line = read_buffer_.substr(0, newline_pos);
    read_buffer_.erase(0, newline_pos + 1);

    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Parse encoder report: "E <left> <right> <timestamp>"
    if (line.size() > 2 && line[0] == 'E' && line[1] == ' ') {
      std::istringstream iss(line.substr(2));
      long left = 0;
      long right = 0;
      long timestamp = 0;
      if (iss >> left >> right >> timestamp) {
        left_ticks = left;
        right_ticks = right;
        found = true;
      }
    }

    newline_pos = read_buffer_.find('\n');
  }

  return found;
}

bool Esp32HardwareDriver::sendVelocityCommand(double left_rpm, double right_rpm)
{
  if (fd_ < 0) {
    return false;
  }

  std::ostringstream oss;
  oss << "V " << left_rpm << " " << right_rpm << "\n";
  const std::string msg = oss.str();

  ssize_t written = ::write(fd_, msg.c_str(), msg.size());
  return written == static_cast<ssize_t>(msg.size());
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
