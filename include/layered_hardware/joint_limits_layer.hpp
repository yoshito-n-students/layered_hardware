#ifndef LAYERED_HARDWARE_JOINT_LIMITS_LAYER_HPP
#define LAYERED_HARDWARE_JOINT_LIMITS_LAYER_HPP

#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
// #include <joint_limits/joint_limits.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/string_registry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware {

// JointLimitsLayer currently supports simple clamping between lower and upper limits.
// it will be updated to use ros2_control/joint_limits for more complex limits once available.

class JointLimitsLayer : public LayerInterface {
public:
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized = LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // store joint limits from given hardware info
    for (const auto &joint_info : hardware_info.joints) {
      for (const auto &command_info : joint_info.command_interfaces) {
        // get joint limits
        const std::string full_iface_name =
            hi::CommandInterface(joint_info.name, command_info.name).get_name();
        const double min = to_double(command_info.min), max = to_double(command_info.max);
        if (std::isnan(min) && std::isnan(max)) {
          continue;
        }
        // skip contradictory limits
        if ((!std::isnan(min)) && (!std::isnan(max)) && (min > max)) {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("layered_hardware"),
                             "JointLimitsLayer::on_init(): "
                                 << "Ignored contradictory limit settings where min: " << min
                                 << " > max: " << max << " for \"" << full_iface_name
                                 << "\" interface");
          continue;
        }
        // store validated limits
        command_limits_.emplace(full_iface_name, std::make_pair(min, max));
        RCLCPP_WARN_STREAM(rclcpp::get_logger("layered_hardware"),
                           "JointLimitsLayer::on_init(): " << "Loaded limit settings [" << min
                                                           << ", " << max << "] for \""
                                                           << full_iface_name << "\" interface");
      }
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export nothing because no states are owned by this layer
    return {};
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export nothing because no commands are owned by this layer
    return {};
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // any state interfaces required from other layers because this layer modifies commands only
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // request other layers for command interfaces of joints of interest
    ci::InterfaceConfiguration config;
    config.type = ci::interface_configuration_type::INDIVIDUAL;
    for (const auto &[name, limits] : command_limits_) {
      config.names.push_back(name);
    }
    return config;
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*parent_loaned_states*/,
                    std::vector<hi::LoanedCommandInterface> &&parent_loaned_commands) override {
    // loan command handles for joints of interest from parent
    std::vector<hi::LoanedCommandInterface> returned_commands;
    for (auto &parent_loaned_command : parent_loaned_commands) {
      if (command_limits_.count(parent_loaned_command.get_name()) > 0) {
        loaned_commands_.emplace_back(std::move(parent_loaned_command));
      } else {
        returned_commands.emplace_back(std::move(parent_loaned_command));
      }
    }
    parent_loaned_commands = std::move(returned_commands);
  }

  virtual hi::return_type
  prepare_command_mode_switch(const StringRegistry & /*active_interfaces*/) override {
    return hi::return_type::OK;
  }

  virtual hi::return_type
  perform_command_mode_switch(const StringRegistry & /*active_interfaces*/) override {
    return hi::return_type::OK;
  }

  virtual hi::return_type read(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) override {
    return hi::return_type::OK;
  }

  virtual hi::return_type write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) override {
    // apply limits to command interfaces
    for (auto &loaned_command : loaned_commands_) {
      // do nothing if the command value is NaN
      if (std::isnan(loaned_command.get_value())) {
        continue;
      }
      // apply limits to numeric command
      const auto &[min, max] = command_limits_[loaned_command.get_name()];
      if (!std::isnan(min)) {
        loaned_command.set_value(std::max(min, loaned_command.get_value()));
      }
      if (!std::isnan(max)) {
        loaned_command.set_value(std::min(max, loaned_command.get_value()));
      }
    }

    return hi::return_type::OK;
  }

protected:
  static double to_double(const std::string &str) {
    if (!str.empty()) {
      try {
        std::istringstream iss(str);
        double val;
        iss >> val;
        return val;
      } catch (const std::ios::failure &error) {
        return std::numeric_limits<double>::quiet_NaN();
      }
    } else {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

protected:
  std::vector<hi::LoanedCommandInterface> loaned_commands_;
  std::map<std::string, std::pair<double, double>> command_limits_;
};

} // namespace layered_hardware

#endif