#ifndef LAYERED_HARDWARE_COMMAND_CLAMPER_LAYER_HPP
#define LAYERED_HARDWARE_COMMAND_CLAMPER_LAYER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/logging_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware {

class CommandClamperLayer : public LayerInterface {
public:
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized = LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // store command limits on joint, sensor and gpio info
    for (const auto components :
         {&hardware_info.joints, &hardware_info.sensors, &hardware_info.gpios}) {
      for (const auto &component_info : *components) {
        for (const auto &command_info : component_info.command_interfaces) {
          // skip the interface without double-type limits
          if (!command_info.enable_limits) {
            continue;
          }
          if (command_info.data_type != "double") {
            continue;
          }
          // get full name of interface by joining component and interface names in official manner
          const std::string full_iface_name =
              hi::CommandInterface(component_info.name, command_info.name).get_name();
          // get lower & upper limits on the interface
          const double lower_limit = to_double(command_info.min),
                       upper_limit = to_double(command_info.max);
          // skip empty or contradictory limits
          if (std::isnan(lower_limit) && std::isnan(upper_limit)) {
            continue;
          }
          if ((!std::isnan(lower_limit)) && (!std::isnan(upper_limit)) &&
              (lower_limit > upper_limit)) {
            lh_warn(
                "CommandClamperLayer::on_init(): "
                "Ignored contradictory limit settings where min: %g > max: %g for \"%s\" interface",
                lower_limit, upper_limit, full_iface_name);
            continue;
          }
          // store validated limits
          command_names_.emplace_back(full_iface_name);
          lower_limits_.emplace_back(lower_limit);
          upper_limits_.emplace_back(upper_limit);
          lh_info(
              "CommandClamperLayer::on_init(): Loaded limit settings [%g, %g] for \"%s\" interface",
              lower_limit, upper_limit, full_iface_name);
        }
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
    // request other layers for command interfaces of interest
    return {ci::interface_configuration_type::INDIVIDUAL, command_names_};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*parent_loaned_states*/,
                    std::vector<hi::LoanedCommandInterface> &&parent_loaned_commands) override {
    // loan command handles for interfaces of interest from parent
    std::vector<std::string> updated_command_names;
    std::vector<double> updated_lower_limits, updated_upper_limits;
    std::vector<hi::LoanedCommandInterface> returned_commands;
    for (auto &parent_loaned_command : parent_loaned_commands) {
      // check if limits for the command handle are given
      const std::size_t i_found = std::find(command_names_.begin(), command_names_.end(),
                                            parent_loaned_command.get_name()) -
                                  command_names_.begin();
      if (i_found < command_names_.size()) {
        // if limits exists, loan the command handle
        loaned_commands_.emplace_back(std::move(parent_loaned_command));
        updated_command_names.emplace_back(std::move(command_names_[i_found]));
        updated_lower_limits.emplace_back(std::move(lower_limits_[i_found]));
        updated_upper_limits.emplace_back(std::move(upper_limits_[i_found]));
      } else {
        // if not, move the command handle to temporary strage to return it
        returned_commands.emplace_back(std::move(parent_loaned_command));
      }
    }
    // update list of names and limits with those of loaned handles
    command_names_ = std::move(updated_command_names);
    lower_limits_ = std::move(updated_lower_limits);
    upper_limits_ = std::move(updated_upper_limits);
    // return unloaned handles to parent
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
    auto loaned_command = loaned_commands_.begin();
    auto lower_limit = lower_limits_.begin(), upper_limit = upper_limits_.begin();
    for (; loaned_command != loaned_commands_.end();
         ++loaned_command, ++lower_limit, ++upper_limit) {
      // do nothing if the command value is NaN
      if (std::isnan(loaned_command->get_value())) {
        continue;
      }
      // apply limits to numeric command
      if (!std::isnan(*lower_limit)) {
        loaned_command->set_value(std::max(*lower_limit, loaned_command->get_value()));
      }
      if (!std::isnan(*upper_limit)) {
        loaned_command->set_value(std::min(*upper_limit, loaned_command->get_value()));
      }
    }

    return hi::return_type::OK;
  }

protected:
  static double to_double(const std::string &str) {
    if (!str.empty()) {
      try {
        double val;
        std::istringstream(str) >> val;
        return val;
      } catch (const std::ios::failure &error) {
        lh_warn("CommandClamperLayer::to_double(): Treating non-convertible \"%s\" as nan: %s", //
                str, error);
        return std::numeric_limits<double>::quiet_NaN();
      }
    } else {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

protected:
  std::vector<std::string> command_names_;
  std::vector<double> lower_limits_, upper_limits_;
  std::vector<hi::LoanedCommandInterface> loaned_commands_;
};

} // namespace layered_hardware

#endif