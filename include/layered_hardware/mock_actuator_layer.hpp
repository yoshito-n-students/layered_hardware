#ifndef LAYERED_HARDWARE_MOCK_ACTUATOR_LAYER_HPP
#define LAYERED_HARDWARE_MOCK_ACTUATOR_LAYER_HPP

#include <map>
#include <memory>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/mock_actuator.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware {

class MockActuatorLayer : public LayerInterface {
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
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("layered_hardware"),
                          "MockActuatorLayer::on_init(): " << "\"" << layer_name
                                                           << "\" parameter is missing");
      return CallbackReturn::ERROR;
    }

    // parse parameters for this layer as yaml
    std::map<std::string, YAML::Node> actuator_list;
    try {
      const YAML::Node params = YAML::Load(params_it->second);
      for (const auto &name_param_pair : params["actuators"]) {
        actuator_list.emplace(name_param_pair.first.as<std::string>(), name_param_pair.second);
      }
    } catch (const YAML::Exception &error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "MockActuatorLayer::on_init(): " << error.what() << " (on parsing \""
                                                           << layer_name << "\" parameter)");
      return CallbackReturn::ERROR;
    }

    // init actuators with param "actuators/<actuator_name>"
    for (const auto &[ator_name, ator_params] : actuator_list) {
      try {
        actuators_.emplace_back(new MockActuator(ator_name, ator_params));
      } catch (const std::runtime_error &error) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                            "MockActuatorLayer::on_init(): " << "Failed to create \"" << ator_name
                                                             << "\" actuator");
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("layered_hardware"),
                         "MockActuatorLayer::on_init(): " << "Created \"" << ator_name
                                                          << "\" actuator");
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export reference to actuator states owned by this layer
    std::vector<hi::StateInterface> ifaces;
    for (const auto &ator : actuators_) {
      ifaces = merge(std::move(ifaces), ator->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export reference to actuator commands owned by this layer
    std::vector<hi::CommandInterface> ifaces;
    for (const auto &ator : actuators_) {
      ifaces = merge(std::move(ifaces), ator->export_command_interfaces());
    }
    return ifaces;
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // any state interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // any command interfaces required from other layers because this layer is "source"
    return {ci::interface_configuration_type::NONE, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> && /*state_interfaces*/,
                    std::vector<hi::LoanedCommandInterface> && /*command_interfaces*/) override {
    // any interfaces has to be imported from other layers because this layer is "source"
  }

  virtual hi::return_type
  prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                              const std::vector<std::string> & /*stop_interfaces*/) override {
    // no preparation for mode switch is required
    return hi::return_type::OK;
  }

  virtual hi::return_type
  perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override {
    // notify controller switching to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = merge(result, ator->perform_command_mode_switch(start_interfaces, stop_interfaces));
    }
    return result;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // read from all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = merge(result, ator->read(time, period));
    }
    return result;
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    // write to all actuators
    hi::return_type result = hi::return_type::OK;
    for (const auto &ator : actuators_) {
      result = merge(result, ator->write(time, period));
    }
    return result;
  }

protected:
  std::vector<std::unique_ptr<MockActuator>> actuators_;
};

} // namespace layered_hardware

#endif