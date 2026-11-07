#include "mearm_hardware/mearm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace mearm_hardware
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

double paramToDouble(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, double default_value)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return default_value;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}

}  // namespace

hardware_interface::CallbackReturn MeArmHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Expect exactly the four mearm joints in the ros2_control block, in order:
  // base, shoulder, elbow, claw.
  if (info_.joints.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Expected 4 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < 4; ++i) {
    const auto & joint = info_.joints[i];
    joint_names_.push_back(joint.name);

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have exactly one command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & limits = info_.limits.at(joint.name);
    min_positions_[i] = limits.min_position;
    max_positions_[i] = limits.max_position;

    const auto & cmd = joint.command_interfaces[0];
    scales_[i] = paramToDouble(cmd.parameters, "scale", 1.0);
    offsets_[i] = paramToDouble(cmd.parameters, "offset", 0.0);

    RCLCPP_INFO(
      get_logger(),
      "Joint '%s': limits [%.4f, %.4f], servo = %.4f * q + %.4f",
      joint.name.c_str(), min_positions_[i], max_positions_[i], scales_[i], offsets_[i]);
  }

  const std::string serial_port = info_.hardware_parameters.count("serial_port") ?
    info_.hardware_parameters.at("serial_port") : "/dev/ttyACM0";

  int baud_rate = 115200;
  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  driver_ = std::make_shared<MeArmSerialDriver>(serial_port, baud_rate);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MeArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!driver_->init()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MeArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Zero/initialize commands and states to avoid NaNs.
  // Base stopped, shoulder/elbow at URDF 0 (→ servo 90°), claw at URDF max (→ servo 95° open).
  cmd_positions_[0] = 0.0;
  cmd_positions_[1] = 0.0;
  cmd_positions_[2] = 0.0;
  cmd_positions_[3] = max_positions_[3];

  for (size_t i = 0; i < 4; ++i) {
    state_positions_[i] = cmd_positions_[i];
    state_velocities_[i] = 0.0;
    set_state(positionInterfaceName(joint_names_[i]), state_positions_[i]);
    set_state(velocityInterfaceName(joint_names_[i]), state_velocities_[i]);
    set_command(positionInterfaceName(joint_names_[i]), cmd_positions_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MeArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the base, hold last commanded arm position.
  driver_->sendServoCommand(
    0.0f,
    static_cast<float>(jointToServo(1, cmd_positions_[1])),
    static_cast<float>(jointToServo(2, cmd_positions_[2])),
    static_cast<float>(jointToServo(3, cmd_positions_[3])));
  driver_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MeArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // No encoders on the arm: report the last commanded positions back as state.
  for (size_t i = 0; i < 4; ++i) {
    if (i == 0) {
      // Base is continuous-rotation: integrate commanded speed into position.
      state_positions_[0] += cmd_positions_[0] * period.seconds();
      state_velocities_[0] = cmd_positions_[0];
    } else {
      state_positions_[i] = cmd_positions_[i];
      state_velocities_[i] = 0.0;
    }
    set_state(positionInterfaceName(joint_names_[i]), state_positions_[i]);
    set_state(velocityInterfaceName(joint_names_[i]), state_velocities_[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MeArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < 4; ++i) {
    double cmd = get_command(positionInterfaceName(joint_names_[i]));
    if (std::isnan(cmd)) {
      // Hold last command if the controller has not set one yet.
      cmd = cmd_positions_[i];
    }
    cmd_positions_[i] = clampToLimits(i, cmd);
  }

  const bool ok = driver_->sendServoCommand(
    static_cast<float>(cmd_positions_[0]),
    static_cast<float>(jointToServo(1, cmd_positions_[1])),
    static_cast<float>(jointToServo(2, cmd_positions_[2])),
    static_cast<float>(jointToServo(3, cmd_positions_[3])));

  return ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

double MeArmHardwareInterface::clampToLimits(size_t index, double value) const
{
  if (std::isnan(min_positions_[index]) || std::isnan(max_positions_[index])) {
    return value;
  }
  return std::max(min_positions_[index], std::min(value, max_positions_[index]));
}

double MeArmHardwareInterface::jointToServo(size_t index, double value) const
{
  return scales_[index] * value + offsets_[index];
}

}  // namespace mearm_hardware

PLUGINLIB_EXPORT_CLASS(
  mearm_hardware::MeArmHardwareInterface,
  hardware_interface::SystemInterface)
