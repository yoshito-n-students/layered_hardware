#ifndef LAYERED_HARDWARE_MONITOR_LAYER_HPP
#define LAYERED_HARDWARE_MONITOR_LAYER_HPP

#include <iomanip>
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
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/string_registry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware {

class MonitorLayer : public LayerInterface {
public:
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized = LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // find parameter group for this layer
    const auto params_it = hardware_info.hardware_parameters.find(layer_name);
    if (params_it == hardware_info.hardware_parameters.end()) {
      // if not found, set default values for parameter
      precision_ = 10;
    } else {
      // if found, parse parameters for this layer as yaml
      try {
        const YAML::Node params = YAML::Load(params_it->second);
        precision_ = params["precision"].as<int>(10);
      } catch (const YAML::Exception &error) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                            "MonitorLayer::on_init(): " << error.what() << " (on parsing \""
                                                        << layer_name << "\" parameter)");
        return CallbackReturn::ERROR;
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
    // request all available state interfaces owned by other layers for monitoring purpose
    return {ci::interface_configuration_type::ALL, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // request all available command interfaces owned by other layers for monitoring purpose
    return {ci::interface_configuration_type::ALL, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> &&parent_loaned_states,
                    std::vector<hi::LoanedCommandInterface> &&parent_loaned_commands) override {
    // loan all states & command from parent
    loaned_states_ = std::move(parent_loaned_states);
    loaned_commands_ = std::move(parent_loaned_commands);

    // initialize storage for previous values, differently from the current value.
    // this simplifies change detection on the first run.
    for (const auto &state : loaned_states_) {
      previous_states_.emplace(state.get_name(), //
                               state.get_value() > 0. ? -std::numeric_limits<double>::infinity()
                                                      : std::numeric_limits<double>::infinity());
    }
    for (const auto &command : loaned_commands_) {
      previous_commands_.emplace(command.get_name(), //
                                 command.get_value() > 0.
                                     ? -std::numeric_limits<double>::infinity()
                                     : std::numeric_limits<double>::infinity());
    }
  }

  virtual hi::return_type
  prepare_command_mode_switch(const StringRegistry & /*active_interfaces*/) override {
    return hi::return_type::OK;
  }

  virtual hi::return_type
  perform_command_mode_switch(const StringRegistry & /*active_interfaces*/) override {
    return hi::return_type::OK;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // check if any commands have been changed from previous call
    std::vector<const hi::LoanedCommandInterface *> changed_commands;
    for (const auto &command : loaned_commands_) {
      if (!bitwise_equal(previous_commands_[command.get_name()], command.get_value())) {
        changed_commands.emplace_back(&command);
      }
    }

    // check if any states have been changed from previous call
    std::vector<const hi::LoanedStateInterface *> changed_states;
    for (const auto &state : loaned_states_) {
      if (!bitwise_equal(previous_states_[state.get_name()], state.get_value())) {
        changed_states.push_back(&state);
      }
    }

    // do nothing if no change on commands & states
    if (changed_commands.empty() && changed_states.empty()) {
      return hi::return_type::OK;
    }

    // print changed values of interfaces
    std::ostringstream msg;
    msg << "MonitorLayer::read()\n";
    msg << "  Time: " << time.nanoseconds() << "ns\n";
    msg << "  Period: " << period.nanoseconds() << "ns\n";
    if (!changed_commands.empty()) {
      msg << "  Command interfaces:\n";
      for (const auto command : changed_commands) {
        msg << "    " << std::quoted(command->get_name()) << ": " //
            << std::setprecision(precision_) << command->get_value() << "\n";
      }
    }
    if (!changed_states.empty()) {
      msg << "  State interfaces:\n";
      for (const auto state : changed_states) {
        msg << "    " << std::quoted(state->get_name()) << ": " //
            << std::setprecision(precision_) << state->get_value() << "\n";
      }
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("layered_hardware"), msg.str());

    // record changed commands & states
    for (const auto command : changed_commands) {
      previous_commands_[command->get_name()] = command->get_value();
    }
    for (const auto state : changed_states) {
      previous_states_[state->get_name()] = state->get_value();
    }

    return hi::return_type::OK;
  }

  virtual hi::return_type write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) override {
    return hi::return_type::OK;
  }

protected:
  static bool bitwise_equal(const double a, const double b) {
    return std::memcmp(&a, &b, sizeof(double)) == 0;
  }

protected:
  int precision_;

  std::vector<hi::LoanedStateInterface> loaned_states_;
  std::map<std::string, double> previous_states_;
  std::vector<hi::LoanedCommandInterface> loaned_commands_;
  std::map<std::string, double> previous_commands_;
};

} // namespace layered_hardware

#endif