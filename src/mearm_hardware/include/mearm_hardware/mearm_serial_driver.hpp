#ifndef MEARM_SERIAL_DRIVER_HPP
#define MEARM_SERIAL_DRIVER_HPP

#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace mearm_hardware
{

class MeArmSerialDriver
{
public:
  explicit MeArmSerialDriver(const std::string & port, int baud_rate = 115200);
  ~MeArmSerialDriver();

  bool init();
  void close();
  bool isOpen() const;

  bool sendServoCommand(float joint_0, float joint_1, float joint_2, float joint_3);

private:
  bool configurePort();

  std::string port_;
  int baud_rate_;
  int fd_{-1};
};

}  // namespace mearm_hardware

#endif  // MEARM_SERIAL_DRIVER_HPP
