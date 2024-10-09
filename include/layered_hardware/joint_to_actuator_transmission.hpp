#ifndef LAYERED_HARDWARE_JOINT_TO_ACTUATOR_TRANSMISSION_HPP
#define LAYERED_HARDWARE_JOINT_TO_ACTUATOR_TRANSMISSION_HPP

#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::CommandInterface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <rclcpp/logging.hpp>
#include <transmission_interface/exception.hpp>
#include <transmission_interface/handle.hpp>
#include <transmission_interface/transmission.hpp>

namespace layered_hardware {

class JointToActuatorTransmission {
public:
  JointToActuatorTransmission(const hi::TransmissionInfo &trans_info,
                              const ti::TransmissionSharedPtr &&converter)
      : trans_info_(trans_info), converter_(std::move(converter)) {
    // converter must be unique because it will be exclusively configured in assign_interfaces()
    if (!converter_) {
      throw std::runtime_error("Null converter for \"" + trans_info.name + "\" transmission");
    }
    if (converter_.use_count() != 1) {
      throw std::runtime_error("Non-unique converter for \"" + trans_info.name + "\" transmission");
    }

    // allocate joint command variables
    for (const auto &joint_info : trans_info.joints) {
      for (const auto &command_iface : joint_info.command_interfaces) {
        joint_commands_[joint_info.name][command_iface] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    // allocate actuator command variables
    // (we cannot complete allocations here
    // because actuator_info.command_interfaces is usually empty)
    for (const auto &actuator_info : trans_info.actuators) {
      actuator_commands_[actuator_info.name] = {};
    }
  }

  std::vector<hi::CommandInterface> export_command_interfaces() {
    // export handles which hold reference to joint commands
    std::vector<hi::CommandInterface> ifaces;
    for (auto &[joint_name, iface_command_map] : joint_commands_) {
      for (auto &[iface_name, command] : iface_command_map) {
        ifaces.emplace_back(joint_name, iface_name, &command);
      }
    }
    return ifaces;
  }

  void assign_interfaces(std::vector<hi::LoanedCommandInterface> &parent_loaned_commands) {
    // loan command handles for actuators of interest from parent
    {
      std::vector<hi::LoanedCommandInterface> returned_commands;
      for (auto &command : parent_loaned_commands) {
        if (actuator_commands_.count(command.get_prefix_name()) > 0) {
          loaned_commands_.push_back(std::move(command));
        } else {
          returned_commands.push_back(std::move(command));
        }
      }
      parent_loaned_commands = std::move(returned_commands);
    }

    // complete allocation of actuator command variables
    for (const auto &command : loaned_commands_) {
      actuator_commands_[command.get_prefix_name()][command.get_interface_name()] =
          std::numeric_limits<double>::quiet_NaN();
    }

    // configure converter
    // 1. create joint command handles, which hold references to joint commands
    std::vector<ti::JointHandle> joint_handles;
    for (auto &[joint_name, iface_command_map] : joint_commands_) {
      for (auto &[iface_name, command] : iface_command_map) {
        joint_handles.emplace_back(joint_name, iface_name, &command);
      }
    }
    // 2. create actuator command handles, which hold references to actuator commands
    std::vector<ti::ActuatorHandle> actuator_handles;
    for (auto &[ator_name, iface_command_map] : actuator_commands_) {
      for (auto &[iface_name, command] : iface_command_map) {
        actuator_handles.emplace_back(ator_name, iface_name, &command);
      }
    }
    // 3. configure converter with handles
    try {
      converter_->configure(joint_handles, actuator_handles);
    } catch (const ti::Exception &error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "JointToActuatorTransmission::assign_interfaces(): "
                              << "Failed to configure \"" << trans_info_.name
                              << "\": " << error.what());
      return;
    }
  }

  void joint_to_actuator() {
    // convert joint commands to actuator commands
    converter_->joint_to_actuator();

    // write actuator command value to external variable
    for (auto &command : loaned_commands_) {
      command.set_value(
          actuator_commands_[command.get_prefix_name()][command.get_interface_name()]);
    }
  }

protected:
  const hi::TransmissionInfo trans_info_;

  // value converter from joint to actuator space
  const ti::TransmissionSharedPtr converter_;
  // map of variables in joint and actuator space to be given to the converter,
  // which maps joint or actuator name (ex. "xxx_joint", "xxx_actuator"),
  // and interface name (ex. "position", "velocity") to variable
  std::map<std::string, std::map<std::string, double>> joint_commands_, actuator_commands_;

  // references to external variables owned by other layers.
  // we should export values to them after conversion.
  std::vector<hi::LoanedCommandInterface> loaned_commands_;
};

} // namespace layered_hardware

#endif