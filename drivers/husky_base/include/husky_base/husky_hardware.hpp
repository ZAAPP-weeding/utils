#ifndef HUSKY_BASE__HUSKY_HARDWARE_HPP_
#define HUSKY_BASE__HUSKY_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "husky_base/husky_diagnostics.h"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "husky_base/horizon_legacy_wrapper.h"

using namespace std::chrono_literals;

namespace husky_base
{
using CallbackReturn = hardware_interface::CallbackReturn;

class HuskyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HuskyHardware)

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  void updateDiagnostics();

private:
  void resetTravelOffset();
  double linearToAngular(const double &travel) const;
  double angularToLinear(const double &angle) const;
  void writeCommandsToHardware();
  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
  void updateJointsFromHardware();
  void initializeDiagnostics();

  // ROS Parameters
  rclcpp::Node::SharedPtr node_;
  std::string serial_port_;
  double polling_timeout_;
  double wheel_diameter_, max_accel_, max_speed_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

  // Diagnostics
  rclcpp::Publisher<husky_msgs::msg::HuskyStatus>::SharedPtr diagnostic_publisher_;
  husky_msgs::msg::HuskyStatus husky_status_msg_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus>::SharedPtr system_status_task_;
  HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem>::SharedPtr power_status_task_;
  HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::SharedPtr safety_status_task_;
  HuskySoftwareDiagnosticTask::SharedPtr software_status_task_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE__HUSKY_HARDWARE_HPP_
