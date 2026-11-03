/**
 * @file esp32_hardware_interface.cpp
 * @brief Jazzy SystemInterface implementation for the ESP32 wheel bridge.
 */

#include "esp32_hardware/esp32_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace esp32_hardware
{

namespace
{

std::string positionInterfaceName(const std::string & joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_POSITION;
}

std::string velocityInterfaceName(const std::string & joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
}

std::string imuSensorInterfaceName(const std::string & sensor_name, const std::string & interface_name)
{
  return sensor_name + "/" + interface_name;
}

}  // namespace

bool Esp32HardwareInterface::isLeftWheel(const std::string & joint_name)
{
  return joint_name.find("left") != std::string::npos;
}

bool Esp32HardwareInterface::isRightWheel(const std::string & joint_name)
{
  return joint_name.find("right") != std::string::npos;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Classify all joints by side. Each side may have multiple wheels driven by
  // the same motor command.
  for (const auto & joint : info_.joints) {
    if (isLeftWheel(joint.name)) {
      left_wheel_joint_names_.push_back(joint.name);
    } else if (isRightWheel(joint.name)) {
      right_wheel_joint_names_.push_back(joint.name);
    }
  }

  if (left_wheel_joint_names_.empty() || right_wheel_joint_names_.empty()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from <ros2_control> URDF block
  const std::string serial_port = info_.hardware_parameters.count("serial_port") ?
    info_.hardware_parameters.at("serial_port") : "/dev/esp32";

  int baud_rate = 115200;
  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  double ticks_per_revolution = 2048.0;
  if (info_.hardware_parameters.count("ticks_per_revolution")) {
    ticks_per_revolution = std::stod(info_.hardware_parameters.at("ticks_per_revolution"));
  }

  if (!info_.sensors.empty()) {
    imu_sensor_name_ = info_.sensors.front().name;
  }

  driver_ = std::make_shared<Esp32HardwareDriver>(serial_port, baud_rate, ticks_per_revolution);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!driver_->init()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->activate();

  const auto zeroState = [this](const std::vector<std::string> & joint_names) {
      for (const auto & joint_name : joint_names) {
        set_state(positionInterfaceName(joint_name), 0.0);
        set_state(velocityInterfaceName(joint_name), 0.0);
      }
    };

  const auto zeroCommand = [this](const std::vector<std::string> & joint_names) {
      for (const auto & joint_name : joint_names) {
        set_command(velocityInterfaceName(joint_name), 0.0);
      }
    };

  zeroState(left_wheel_joint_names_);
  zeroState(right_wheel_joint_names_);
  zeroCommand(left_wheel_joint_names_);
  zeroCommand(right_wheel_joint_names_);

  // Initialize IMU sensor state interfaces.
  set_state(imuSensorInterfaceName(imu_sensor_name_, "orientation.x"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "orientation.y"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "orientation.z"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "orientation.w"), 1.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "angular_velocity.x"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "angular_velocity.y"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "angular_velocity.z"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "linear_acceleration.x"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "linear_acceleration.y"), 0.0);
  set_state(imuSensorInterfaceName(imu_sensor_name_, "linear_acceleration.z"), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->deactivate();
  driver_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Esp32HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  driver_->update(period.seconds());

  const auto setWheelState = [this](
    const std::vector<std::string> & joint_names, double position, double velocity) {
      for (const auto & joint_name : joint_names) {
        set_state(positionInterfaceName(joint_name), position);
        set_state(velocityInterfaceName(joint_name), velocity);
      }
    };

  setWheelState(
    left_wheel_joint_names_,
    driver_->getPositionRadian(WheelSide::Left),
    driver_->getVelocityRadianPerSec(WheelSide::Left));

  setWheelState(
    right_wheel_joint_names_,
    driver_->getPositionRadian(WheelSide::Right),
    driver_->getVelocityRadianPerSec(WheelSide::Right));

  // Update IMU sensor state interfaces.
  set_state(imuSensorInterfaceName(imu_sensor_name_, "angular_velocity.x"), driver_->getAngularVelocity(0));
  set_state(imuSensorInterfaceName(imu_sensor_name_, "angular_velocity.y"), driver_->getAngularVelocity(1));
  set_state(imuSensorInterfaceName(imu_sensor_name_, "angular_velocity.z"), driver_->getAngularVelocity(2));
  set_state(imuSensorInterfaceName(imu_sensor_name_, "linear_acceleration.x"), driver_->getLinearAcceleration(0));
  set_state(imuSensorInterfaceName(imu_sensor_name_, "linear_acceleration.y"), driver_->getLinearAcceleration(1));
  set_state(imuSensorInterfaceName(imu_sensor_name_, "linear_acceleration.z"), driver_->getLinearAcceleration(2));

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Esp32HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // diff_drive_controller writes the same velocity to every joint on a side.
  // Use the first joint's command for each side.
  double left_cmd = get_command(velocityInterfaceName(left_wheel_joint_names_.front()));
  double right_cmd = get_command(velocityInterfaceName(right_wheel_joint_names_.front()));

  // Guard against NaN commands before any controller has been activated or
  // after it has been deactivated. NaN would be sent to the ESP32 and turn
  // into an undefined motor command.
  if (std::isnan(left_cmd) || std::isnan(right_cmd)) {
    left_cmd = 0.0;
    right_cmd = 0.0;
  }

  driver_->setTargetVelocityRadianPerSec(left_cmd, right_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace esp32_hardware

PLUGINLIB_EXPORT_CLASS(
  esp32_hardware::Esp32HardwareInterface,
  hardware_interface::SystemInterface)
