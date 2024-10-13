#ifndef LAYERED_HARDWARE_TRANSMISSION_LAYER_HPP
#define LAYERED_HARDWARE_TRANSMISSION_LAYER_HPP

#include <exception>
#include <map>
#include <memory>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      //for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <layered_hardware/actuator_to_joint_transmission.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/joint_to_actuator_transmission.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/logging_utils.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/exceptions.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <transmission_interface/exception.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>

namespace layered_hardware {

class TransmissionLayer : public LayerInterface {
public:
  TransmissionLayer()
      : converter_factory_loader_("transmission_interface",
                                  "transmission_interface::TransmissionLoader") {}

  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized = LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // pick converter class names in this hardware
    for (const auto &trans_info : hardware_info.transmissions) {
      converter_factories_.emplace(trans_info.type, nullptr);
    }

    // load converter factories
    for (auto &[trans_type, converter_factory] : converter_factories_) {
      try {
        converter_factory.reset(converter_factory_loader_.createUnmanagedInstance(trans_type));
      } catch (const pluginlib::PluginlibException &error) {
        LH_ERROR("TransmissionLayer::on_init(): Failed to load factory for \"%s\": %s",
                 trans_type.c_str(), error.what());
        return CallbackReturn::ERROR;
      }
    }

    // create joint-to-actuator transmissions
    for (const auto &trans_info : hardware_info.transmissions) {
      const std::string trans_disp_name =
          "\"" + trans_info.name + "\" transmission (" + trans_info.type + ", joint-to-actuator)";
      // create converter from joint to actuator space
      ti::TransmissionSharedPtr converter = converter_factories_[trans_info.type]->load(trans_info);
      if (!converter) {
        LH_ERROR("TransmissionLayer::on_init(): Failed to create converter for %s",
                 trans_disp_name.c_str());
        return CallbackReturn::ERROR;
      }
      // create transmission
      std::unique_ptr<JointToActuatorTransmission> trans;
      try {
        trans.reset(new JointToActuatorTransmission(trans_info, std::move(converter)));
      } catch (const std::runtime_error &error) {
        LH_ERROR("TransmissionLayer::on_init(): Failed to create %s: %s", //
                 trans_disp_name.c_str(), error.what());
        return CallbackReturn::ERROR;
      }
      // make transmission owned by this layer
      joint_to_actuator_transmissions_[trans_info.name] = std::move(trans);
      LH_INFO("TransmissionLayer::on_init(): Loaded %s", trans_disp_name.c_str());
    }

    // create actuator-to-joint transmissions
    for (const auto &trans_info : hardware_info.transmissions) {
      const std::string trans_disp_name =
          "\"" + trans_info.name + "\" transmission (" + trans_info.type + ", actuator-to-joint)";
      // create converter from actuator to joint space
      ti::TransmissionSharedPtr converter = converter_factories_[trans_info.type]->load(trans_info);
      if (!converter) {
        LH_ERROR("TransmissionLayer::on_init(): Failed to create converter for %s",
                 trans_disp_name.c_str());
        return CallbackReturn::ERROR;
      }
      // create transmission
      std::unique_ptr<ActuatorToJointTransmission> trans;
      try {
        trans.reset(new ActuatorToJointTransmission(trans_info, std::move(converter)));
      } catch (const std::runtime_error &error) {
        LH_ERROR("TransmissionLayer::on_init(): Failed to create %s: %s", //
                 trans_disp_name.c_str(), error.what());
        return CallbackReturn::ERROR;
      }
      // make transmission owned by this layer
      actuator_to_joint_transmissions_[trans_info.name] = std::move(trans);
      LH_INFO("TransmissionLayer::on_init(): Loaded %s", trans_disp_name.c_str());
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    // export handles which hold reference to joint states
    std::vector<hi::StateInterface> ifaces;
    for (const auto &[trans_name, trans] : actuator_to_joint_transmissions_) {
      ifaces = merge(std::move(ifaces), trans->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    // export handles which hold reference to joint commands
    std::vector<hi::CommandInterface> ifaces;
    for (const auto &[trans_name, trans] : joint_to_actuator_transmissions_) {
      ifaces = merge(std::move(ifaces), trans->export_command_interfaces());
    }
    return ifaces;
  }

  virtual ci::InterfaceConfiguration state_interface_configuration() const override {
    // request all available interface of actuator states
    return {ci::interface_configuration_type::ALL, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // request all available interface of actuator commands
    return {ci::interface_configuration_type::ALL, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> &&loaned_states,
                    std::vector<hi::LoanedCommandInterface> &&loaned_commands) override {
    // assign references to actuator states to state-converting transmissions
    for (const auto &[trans_name, trans] : actuator_to_joint_transmissions_) {
      trans->assign_interfaces(loaned_states);
    }
    // assign references to actuator commands to command-converting transmissions
    for (const auto &[trans_name, trans] : joint_to_actuator_transmissions_) {
      trans->assign_interfaces(loaned_commands);
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

  virtual hi::return_type read(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) override {
    // convert actuator states to joint states
    for (const auto &[trans_name, trans] : actuator_to_joint_transmissions_) {
      trans->actuator_to_joint();
    }
    return hi::return_type::OK;
  }

  virtual hi::return_type write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) override {
    // convert joint commands to actuator commands
    for (const auto &[trans_name, trans] : joint_to_actuator_transmissions_) {
      trans->joint_to_actuator();
    }
    return hi::return_type::OK;
  }

protected:
  // plugin loader for converter factories
  pluginlib::ClassLoader<ti::TransmissionLoader> converter_factory_loader_;

  // maps converter class name to converter factory.
  // a factory creates instances of converter from joint to actuator space or vice varsa
  std::map<std::string, std::unique_ptr<ti::TransmissionLoader>> converter_factories_;

  // maps transmission name to transmission instances.
  // a transmission instance owns a converter,
  // joint space variables, and actuator space variables
  std::map<std::string, std::unique_ptr<JointToActuatorTransmission>>
      joint_to_actuator_transmissions_;
  std::map<std::string, std::unique_ptr<ActuatorToJointTransmission>>
      actuator_to_joint_transmissions_;
};

} // namespace layered_hardware

#endif