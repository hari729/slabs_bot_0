#ifndef ESP32_HARDWARE_INTERFACE_HPP
#define ESP32_HARDWARE_INTERFACE_HPP

/**
 * @file esp32_hardware_interface.hpp
 * @brief ros2_control SystemInterface for the ESP32 wheel bridge.
 *
 * This class is Jazzy-only glue. It reads URDF parameters, wires the
 * state/command interfaces to the Esp32HardwareDriver, and implements the
 * ros2_control lifecycle.
 */

#include <memory>
#include <string>
#include <vector>

#include "esp32_hardware/esp32_hardware_driver.hpp"
#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace esp32_hardware
{

class Esp32HardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static bool isLeftWheel(const std::string & joint_name);
  static bool isRightWheel(const std::string & joint_name);

  std::shared_ptr<Esp32HardwareDriver> driver_;

  std::vector<std::string> left_wheel_joint_names_;
  std::vector<std::string> right_wheel_joint_names_;
};

}  // namespace esp32_hardware

#endif  // ESP32_HARDWARE_INTERFACE_HPP
