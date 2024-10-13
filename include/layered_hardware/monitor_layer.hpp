#ifndef LAYERED_HARDWARE_MONITOR_LAYER_HPP
#define LAYERED_HARDWARE_MONITOR_LAYER_HPP

#include <functional>
#include <iomanip>
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
    // loan all states & commands from parent
    loaned_states_ = std::move(parent_loaned_states);
    loaned_commands_ = std::move(parent_loaned_commands);
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
    // get state & command interfaces with value changes, or all on first run
    const auto changed_states =
        (previous_states_.empty()
             ? pick_pointers(loaned_states_, All{})
             : pick_pointers(loaned_states_, BitwiseDifferent{previous_states_}));
    const auto changed_commands =
        (previous_commands_.empty()
             ? pick_pointers(loaned_commands_, All{})
             : pick_pointers(loaned_commands_, BitwiseDifferent{previous_commands_}));

    // do nothing if no changed states & commands
    if (changed_states.empty() && changed_commands.empty()) {
      return hi::return_type::OK;
    }

    // print changed values
    std::ostringstream msg;
    msg << "MonitorLayer::read()\n";
    msg << "  Time: " << time.nanoseconds() << "ns\n";
    msg << "  Period: " << period.nanoseconds() << "ns\n";
    if (!changed_states.empty()) {
      msg << "  State interfaces:\n";
      format(msg, /* n_indent = */ 4, precision_, changed_states);
    }
    if (!changed_commands.empty()) {
      msg << "  Command interfaces:\n";
      format(msg, /* n_indent = */ 4, precision_, changed_commands);
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("layered_hardware"), msg.str());

    // update storage for previous states & commands
    previous_states_ = extract_values(loaned_states_);
    previous_commands_ = extract_values(loaned_commands_);

    return hi::return_type::OK;
  }

  virtual hi::return_type write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) override {
    return hi::return_type::OK;
  }

protected:
  // utility: return pointers to interfaces where do_pick(iface.get_value()) returns true

  struct All {
    bool operator()(const std::size_t /*i*/, const double /*value*/) const { return true; }
  };

  struct BitwiseDifferent {
    const std::vector<double> &other_values;

    bool operator()(const std::size_t i, const double value) const {
      return std::memcmp(&value, &other_values[i], sizeof(double)) != 0;
    }
  };

  template <class Interface>
  static std::vector<const Interface *>
  pick_pointers(const std::vector<Interface> &ifaces,
                const std::function<bool(const std::size_t, const double)> &do_pick) {
    std::vector<const Interface *> result;
    for (std::size_t i = 0; i < ifaces.size(); ++i) {
      if (do_pick(i, ifaces[i].get_value())) {
        result.push_back(&ifaces[i]);
      }
    }
    return result;
  }

  // utility: compare current and previous values, and output those that meet condition to stream
  template <class Interface>
  static void format(std::ostream &os, const std::size_t n_indent, const int precision,
                     const std::vector<Interface *> &ifaces) {
    const std::string indent(n_indent, ' ');
    for (const auto &iface : ifaces) {
      os << indent << std::quoted(iface->get_name()) << ": " //
         << std::setprecision(precision) << iface->get_value() << "\n";
    }
  }

  // utility: Create a new vector from values of each interface
  template <class Interface>
  static std::vector<double> extract_values(const std::vector<Interface> &ifaces) {
    std::vector<double> values;
    values.reserve(ifaces.size());
    for (const auto &iface : ifaces) {
      values.push_back(iface.get_value());
    }
    return values;
  }

protected:
  int precision_;

  std::vector<hi::LoanedStateInterface> loaned_states_;
  std::vector<double> previous_states_;
  std::vector<hi::LoanedCommandInterface> loaned_commands_;
  std::vector<double> previous_commands_;
};

} // namespace layered_hardware

#endif