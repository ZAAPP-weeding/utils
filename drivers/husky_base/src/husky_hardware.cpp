/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "husky_base/husky_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  const uint8_t LEFT = 0, RIGHT = 2;
}

namespace husky_base
{
  static const std::string HW_NAME = "HuskyHardware";

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void HuskyHardware::resetTravelOffset()
  {
    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc =
        horizon_legacy::Channel<clearpath::DataEncoders>::requestData(polling_timeout_);
    if (enc)
    {
      for (auto i = 0u; i < hw_states_position_offset_.size(); i++)
      {
        hw_states_position_offset_[i] = linearToAngular(enc->getTravel(i % 2));
      }
    }
    else
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME), "Could not get encoder data to calibrate travel offset");
    }
  }

  /**
  * Husky reports travel in metres, need radians for ros_control RobotHW
  */
  double HuskyHardware::linearToAngular(const double &travel) const
  {
    return (travel / wheel_diameter_ * 2.0f);
  }

  /**
  * RobotHW provides velocity command in rad/s, Husky needs m/s,
  */
  double HuskyHardware::angularToLinear(const double &angle) const
  {
    return (angle * wheel_diameter_ / 2.0f);
  }

  void HuskyHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(hw_commands_[LEFT]);
    double diff_speed_right = angularToLinear(hw_commands_[RIGHT]);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    horizon_legacy::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
  }

  void HuskyHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }


  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void HuskyHardware::updateJointsFromHardware()
  {

    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc =
      horizon_legacy::Channel<clearpath::DataEncoders>::requestData(polling_timeout_);
    if (enc)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger(HW_NAME),
        "Received linear distance information (L: %f, R: %f)",
        enc->getTravel(LEFT), enc->getTravel(RIGHT));
      for (auto i = 0u; i < hw_states_position_.size(); i++)
      {
        double delta = linearToAngular(enc->getTravel(i >= 2)) - hw_states_position_[i] - hw_states_position_offset_[i];

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0f)
        {
          hw_states_position_[i] += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          hw_states_position_offset_[i] += delta;
          RCLCPP_WARN(
            rclcpp::get_logger(HW_NAME),"Dropping overflow measurement from encoder");
        }
      }
    }

    horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed =
      horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(polling_timeout_);
    if (speed)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger(HW_NAME),
        "Received linear speed information (L: %f, R: %f)",
        speed->getLeftSpeed(), speed->getRightSpeed());

      for (auto i = 0u; i < hw_states_velocity_.size(); i++)
      {

        if (i < 2)
        {
          hw_states_velocity_[i] = linearToAngular(speed->getLeftSpeed());
        }
        else
        { // assume RIGHT
          hw_states_velocity_[i] = linearToAngular(speed->getRightSpeed());
        }
      }
    }
  }


CallbackReturn HuskyHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("husky_base");

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Name: %s", info_.name.c_str());

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Number of Joints %zu", info_.joints.size());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_position_offset_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
  max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
  max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);
  polling_timeout_ = std::stod(info_.hardware_parameters["polling_timeout"]);

  serial_port_ = info_.hardware_parameters["serial_port"];

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Port: %s", serial_port_.c_str());
  horizon_legacy::connect(serial_port_);
  horizon_legacy::configureLimits(max_speed_, max_accel_);
  resetTravelOffset();

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // HuskyHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  initializeDiagnostics();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HuskyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn HuskyHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++)
  {
    if (std::isnan(hw_states_position_[i]))
    {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn HuskyHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HuskyHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");

  updateDiagnostics();

  updateJointsFromHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");

  writeCommandsToHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

void HuskyHardware::initializeDiagnostics() {
  horizon_legacy::Channel<clearpath::DataPlatformInfo>::Ptr info =
      horizon_legacy::Channel<clearpath::DataPlatformInfo>::requestData(polling_timeout_);
    std::ostringstream hardware_id_stream;
    hardware_id_stream << "Husky " << info->getModel() << "-" << info->getSerial();
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
    diagnostic_updater_->setHardwareID(hardware_id_stream.str());

    system_status_task_= std::make_shared<HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus>>(husky_status_msg_);
    power_status_task_ = std::make_shared<HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem>>(husky_status_msg_);
    safety_status_task_ = std::make_shared<HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>>(husky_status_msg_);
    // TODO(shrijitsingh99): Get target control frequnecy parameter instead of setting to 10
    software_status_task_ = std::make_shared<HuskySoftwareDiagnosticTask>(husky_status_msg_, 10);
    
    diagnostic_updater_->add(*system_status_task_);
    diagnostic_updater_->add(*power_status_task_);
    diagnostic_updater_->add(*safety_status_task_);
    diagnostic_updater_->add(*software_status_task_);
    diagnostic_publisher_ = node_-> create_publisher<husky_msgs::msg::HuskyStatus>("status", 10);
}

void HuskyHardware::updateDiagnostics() {
  diagnostic_updater_->force_update();
  husky_status_msg_.header.stamp = node_->now();
  diagnostic_publisher_->publish(husky_status_msg_);
}

}  // namespace husky_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  husky_base::HuskyHardware, hardware_interface::SystemInterface)