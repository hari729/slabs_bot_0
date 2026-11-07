#ifndef MEARM_HARDWARE_INTERFACE_HPP
#define MEARM_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "mearm_hardware/mearm_serial_driver.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mearm_hardware
{

class MeArmHardwareInterface : public hardware_interface::SystemInterface
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
  std::vector<std::string> joint_names_;

  std::shared_ptr<MeArmSerialDriver> driver_;

  double cmd_positions_[4]{0.0, 0.0, 0.0, 0.0};
  double state_positions_[4]{0.0, 0.0, 0.0, 0.0};
  double state_velocities_[4]{0.0, 0.0, 0.0, 0.0};

  double min_positions_[4]{0.0, 0.0, 0.0, 0.0};
  double max_positions_[4]{0.0, 0.0, 0.0, 0.0};
  double scales_[4]{1.0, 1.0, 1.0, 1.0};
  double offsets_[4]{0.0, 0.0, 0.0, 0.0};

  double clampToLimits(size_t index, double value) const;
  double jointToServo(size_t index, double value) const;
};

}  // namespace mearm_hardware

#endif  // MEARM_HARDWARE_INTERFACE_HPP
