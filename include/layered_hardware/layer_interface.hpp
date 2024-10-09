#ifndef LAYERED_HARDWARE_LAYER_INTERFACE_HPP
#define LAYERED_HARDWARE_LAYER_INTERFACE_HPP

#include <string>
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp> // for hi::return_type
#include <layered_hardware/common_namespaces.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace layered_hardware {

class LayerInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface {
public:
  LayerInterface() = default;
  LayerInterface(const LayerInterface &other) = delete;
  LayerInterface(LayerInterface &&other) = default;
  virtual ~LayerInterface() = default;

  virtual CallbackReturn on_init(const std::string &layer_name,
                                 const hi::HardwareInfo & /*hardware_info*/) {
    name_ = layer_name;
    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() = 0;
  virtual std::vector<hi::CommandInterface> export_command_interfaces() = 0;
  virtual ci::InterfaceConfiguration state_interface_configuration() const = 0;
  virtual ci::InterfaceConfiguration command_interface_configuration() const = 0;
  virtual void assign_interfaces(std::vector<hi::LoanedStateInterface> &&state_interfaces,
                                 std::vector<hi::LoanedCommandInterface> &&command_interfaces) = 0;

  virtual hi::return_type
  prepare_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                              const std::vector<std::string> & /*stop_interfaces*/) {
    return hi::return_type::OK;
  }
  virtual hi::return_type
  perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                              const std::vector<std::string> & /*stop_interfaces*/) {
    return hi::return_type::OK;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;
  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

  virtual std::string get_name() const { return name_; }

protected:
  std::string name_;
};

} // namespace layered_hardware

#endif