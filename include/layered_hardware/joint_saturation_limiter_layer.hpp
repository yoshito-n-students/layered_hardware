#ifndef LAYERED_HARDWARE_JOINT_SATURATION_LIMITTER_LAYER_HPP
#define LAYERED_HARDWARE_JOINT_SATURATION_LIMITTER_LAYER_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/individual_joint_saturation_limiter.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/logging_utils.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware {

class JointSaturationLimiterLayer : public LayerInterface {
public:
  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const CallbackReturn is_base_initialized = LayerInterface::on_init(layer_name, hardware_info);
    if (is_base_initialized != CallbackReturn::SUCCESS) {
      return is_base_initialized;
    }

    // parse parameters for this layer as yaml (optional)
    std::string robot_description_topic = "/robot_description";
    try {
      const auto params_it = hardware_info.hardware_parameters.find(layer_name);
      if (params_it != hardware_info.hardware_parameters.end()) {
        const YAML::Node params = YAML::Load(params_it->second);
        robot_description_topic = params["robot_description_topic"].as<std::string>();
      }
    } catch (const YAML::Exception &error) {
      lh_error("JointSaturationLimiterLayer::on_init(): %s (on parsing \"%s\" parameter)", //
               error, layer_name);
      return CallbackReturn::ERROR;
    }

    // create ROS node for joint command limiters to access ROS parameters & robot description topic
    const auto node = std::make_shared<rclcpp::Node>(layer_name);

    // create command limiters for individual joints
    for (const auto &joint_info : hardware_info.joints) {
      try {
        command_limiters_.emplace_back(
            new IndividualJointSaturationLimiter(joint_info.name, node, robot_description_topic));
      } catch (const std::runtime_error &error) {
        lh_error("JointSaturationLimiterLayer::on_init(): "
                 "%s (on creating command limiter for \"%s\" joint)",
                 error, joint_info.name);
        return CallbackReturn::ERROR;
      }
      lh_info(
          "JointSaturationLimiterLayer::on_init(): Created joint command limiter for \"%s\" joint",
          joint_info.name);
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
    // request all available interface of joint states
    return {ci::interface_configuration_type::ALL, {}};
  }

  virtual ci::InterfaceConfiguration command_interface_configuration() const override {
    // request all available interface of joint commands
    return {ci::interface_configuration_type::ALL, {}};
  }

  virtual void
  assign_interfaces(std::vector<hi::LoanedStateInterface> &&loaned_states,
                    std::vector<hi::LoanedCommandInterface> &&loaned_commands) override {
    // assign references to joint states & commands to limiters for individual joints
    for (const auto &limiter : command_limiters_) {
      limiter->configure(loaned_states, loaned_commands);
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
    return hi::return_type::OK;
  }

  virtual hi::return_type write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration &period) override {
    // enforce limits for each joint command
    for (const auto &limiter : command_limiters_) {
      limiter->enforce(period);
    }
    return hi::return_type::OK;
  }

protected:
  std::vector<std::unique_ptr<IndividualJointSaturationLimiter>> command_limiters_;
};

} // namespace layered_hardware

#endif