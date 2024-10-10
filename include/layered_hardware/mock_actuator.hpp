#ifndef LAYERED_HARDWARE_MOCK_ACTUATOR_HPP
#define LAYERED_HARDWARE_MOCK_ACTUATOR_HPP

#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/string_registry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware {

class MockActuator {
public:
  MockActuator(const std::string &name, const YAML::Node &params) : name_(name) {
    // parse parameters for this actuator
    std::vector<std::string> mapped_mode_names;
    try {
      for (const auto &iface_mode_pair : params["command_mode_map"]) {
        bound_interfaces_.push_back(iface_mode_pair.first.as<std::string>());
        mapped_mode_names.push_back(iface_mode_pair.second.as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      throw std::runtime_error("Failed to parse parameter for \"" + name +
                               "\" actuator: " + error.what());
    }

    // build command mode map according to parameters
    try {
      for (const auto &mode_name : mapped_mode_names) {
        mapped_modes_.push_back(to_mode(mode_name));
      }
    } catch (const std::runtime_error &error) {
      throw std::runtime_error("Failed to build command mode map for \"" + name +
                               "\" actuator: " + error.what());
    }
  }

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export reference to state variables of this actuator
    std::vector<hi::StateInterface> ifaces;
    ifaces.emplace_back(name_, hi::HW_IF_POSITION, &position_);
    ifaces.emplace_back(name_, hi::HW_IF_VELOCITY, &velocity_);
    ifaces.emplace_back(name_, hi::HW_IF_EFFORT, &effort_);
    return ifaces;
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export reference to command variables of this actuator
    std::vector<hi::CommandInterface> ifaces;
    ifaces.emplace_back(name_, hi::HW_IF_POSITION, &position_command_);
    ifaces.emplace_back(name_, hi::HW_IF_VELOCITY, &velocity_command_);
    ifaces.emplace_back(name_, hi::HW_IF_EFFORT, &effort_command_);
    return ifaces;
  }

  hi::return_type prepare_command_mode_switch(const StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() <= 1) {
      return hi::return_type::OK;
    } else { // active_bound_ifaces.size() >= 2
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "MockActuator::prepare_command_mode_switch(): "
                              << "Reject mode switching of \"" << name_ << "\" actuator because "
                              << active_bound_ifaces.size()
                              << " bound interfaces are about to be active");
      return hi::return_type::ERROR;
    }
  }

  hi::return_type perform_command_mode_switch(const StringRegistry &active_interfaces) {
    // check how many interfaces associated with actuator command mode are active
    const std::vector<std::size_t> active_bound_ifaces = active_interfaces.find(bound_interfaces_);
    if (active_bound_ifaces.size() >= 2) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "MockActuator::perform_command_mode_switch(): "
                              << "Could not switch mode of \"" << name_ << "\" actuator because "
                              << bound_interfaces_.size() << " bound interfaces are active");
      return hi::return_type::ERROR;
    }

    // switch to actuator command mode associated with active bound interface
    const command_mode next_mode =
        (!active_bound_ifaces.empty() ? mapped_modes_[active_bound_ifaces.front()]
                                      : command_mode::READ_ONLY /*fallback*/);
    if (present_mode_ != next_mode) {
      present_mode_ = next_mode;
      RCLCPP_INFO_STREAM(rclcpp::get_logger("layered_hardware"),
                         "MockActuator::perform_command_mode_switch(): "
                             << "Switched \"" << name_ << "\" actuator to \""
                             << to_string(present_mode_) << "\" mode");
    }
    return hi::return_type::OK;
  }

  hi::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // noting to do as states will be updated in write()
    return hi::return_type::OK;
  }

  hi::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
    // update actuator states based on present command mode
    switch (present_mode_) {
    case command_mode::READ_ONLY:
      // nothing to do
      return hi::return_type::OK;
    case command_mode::POSITION:
      if (!std::isnan(position_command_)) {
        velocity_ = (position_command_ - position_) / period.seconds();
        position_ = position_command_;
        effort_ = 0.;
      }
      return hi::return_type::OK;
    case command_mode::VELOCITY:
      if (!std::isnan(velocity_command_)) {
        position_ += velocity_command_ * period.seconds();
        velocity_ = velocity_command_;
        effort_ = 0.;
      }
      return hi::return_type::OK;
    case command_mode::EFFORT:
      if (!std::isnan(effort_command_)) {
        position_ = 0.;
        velocity_ = 0.;
        effort_ = effort_command_;
      }
      return hi::return_type::OK;
    default:
      // should never happen
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "MockActuator::write(): " << "Unknown command mode id ("
                                                    << static_cast<int>(present_mode_) << ")");
      return hi::return_type::ERROR;
    }
  }

protected:
  enum class command_mode : std::uint8_t { READ_ONLY, POSITION, VELOCITY, EFFORT };

  static command_mode to_mode(const std::string &str) {
    if (str == "read_only") {
      return command_mode::READ_ONLY;
    } else if (str == "position") {
      return command_mode::POSITION;
    } else if (str == "velocity") {
      return command_mode::VELOCITY;
    } else if (str == "effort") {
      return command_mode::EFFORT;
    } else {
      throw std::runtime_error("Unknown command mode name \"" + str + "\"");
    }
  }

  static std::string to_string(const command_mode &mode) {
    switch (mode) {
    case command_mode::READ_ONLY:
      return "read_only";
    case command_mode::POSITION:
      return "position";
    case command_mode::VELOCITY:
      return "velocity";
    case command_mode::EFFORT:
      return "effort";
    default:
      std::ostringstream oss;
      oss << "Unkonwn command mode id (" << static_cast<int>(mode) << ")";
      throw std::runtime_error(oss.str());
    }
  }

protected:
  // name of actuator
  const std::string name_;
  // actuator states
  double position_ = 0., velocity_ = 0., effort_ = 0.;
  // actuator commands
  double position_command_ = std::numeric_limits<double>::quiet_NaN(),
         velocity_command_ = std::numeric_limits<double>::quiet_NaN(),
         effort_command_ = std::numeric_limits<double>::quiet_NaN();
  // present command mode
  command_mode present_mode_ = command_mode::READ_ONLY;
  // map from command interface to command mode
  // (i.e. mapped_modes_[i] is associated with bound_interfaces_[i])
  std::vector<std::string> bound_interfaces_;
  std::vector<command_mode> mapped_modes_;
};

} // namespace layered_hardware

#endif