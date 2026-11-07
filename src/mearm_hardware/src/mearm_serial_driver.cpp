#include "mearm_hardware/mearm_serial_driver.hpp"

#include <cmath>
#include <cstring>

#include "mearm_hardware/protocol.h"

namespace mearm_hardware
{

namespace
{

speed_t baudRateToSpeed(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default: return B115200;
  }
}

}  // namespace

MeArmSerialDriver::MeArmSerialDriver(const std::string & port, int baud_rate)
: port_(port), baud_rate_(baud_rate)
{
}

MeArmSerialDriver::~MeArmSerialDriver()
{
  close();
}

bool MeArmSerialDriver::init()
{
  if (fd_ >= 0) {
    return true;
  }

  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    return false;
  }

  // Clear non-blocking flag after open so writes block normally.
  const int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NDELAY);

  if (!configurePort()) {
    close();
    return false;
  }

  return true;
}

void MeArmSerialDriver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool MeArmSerialDriver::isOpen() const
{
  return fd_ >= 0;
}

bool MeArmSerialDriver::configurePort()
{
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    return false;
  }

  cfsetospeed(&tty, baudRateToSpeed(baud_rate_));
  cfsetispeed(&tty, baudRateToSpeed(baud_rate_));

  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;                // 8-bit characters
  tty.c_cflag &= ~PARENB;            // no parity
  tty.c_cflag &= ~CSTOPB;            // 1 stop bit

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw input
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // no software flow control
  tty.c_oflag &= ~OPOST;                           // raw output

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

bool MeArmSerialDriver::sendServoCommand(
  float joint_0, float joint_1, float joint_2, float joint_3)
{
  if (fd_ < 0) {
    return false;
  }

  if (std::isnan(joint_0) || std::isnan(joint_1) || std::isnan(joint_2) || std::isnan(joint_3)) {
    return false;
  }

  mearm_protocol::ServoCmd cmd;
  cmd.joint_0 = joint_0;
  cmd.joint_1 = joint_1;
  cmd.joint_2 = joint_2;
  cmd.joint_3 = joint_3;

  uint8_t frame[sizeof(cmd) + 6];
  const uint16_t frame_len = mearm_protocol::frame_pack(
    mearm_protocol::PacketType::ServoCmdPacket, &cmd, sizeof(cmd), frame);

  const ssize_t written = write(fd_, frame, frame_len);
  return written == static_cast<ssize_t>(frame_len);
}

}  // namespace mearm_hardware
