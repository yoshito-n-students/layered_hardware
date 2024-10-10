#ifndef LAYERED_HARDWARE_ACTUATOR_TO_JOINT_TRANSMISSION_HPP
#define LAYERED_HARDWARE_ACTUATOR_TO_JOINT_TRANSMISSION_HPP

#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::StateInterface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <rclcpp/logging.hpp>
#include <transmission_interface/exception.hpp>
#include <transmission_interface/handle.hpp>
#include <transmission_interface/transmission.hpp>

namespace layered_hardware {

class ActuatorToJointTransmission {
public:
  ActuatorToJointTransmission(const hi::TransmissionInfo &trans_info,
                              ti::TransmissionSharedPtr &&converter)
      : trans_info_(trans_info), converter_(std::move(converter)) {
    // converter must be unique because it will be exclusively configured in assign_interfaces()
    if (!converter_) {
      throw std::runtime_error("Null converter for \"" + trans_info.name + "\" transmission");
    }
    if (converter_.use_count() != 1) {
      throw std::runtime_error("Non-unique converter for \"" + trans_info.name + "\" transmission");
    }

    // allocate joint state variables
    for (const auto &joint_info : trans_info.joints) {
      for (const auto &state_iface : joint_info.state_interfaces) {
        joint_states_[joint_info.name][state_iface] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    // allocate actuator state variables
    // (we cannot complete allocations here
    // because actuator_info.state_interfaces is usually empty)
    for (const auto &actuator_info : trans_info.actuators) {
      actuator_states_[actuator_info.name] = {};
    }
  }

  std::vector<hi::StateInterface> export_state_interfaces() {
    // export handles which hold reference to joint states
    std::vector<hi::StateInterface> ifaces;
    for (auto &[joint_name, iface_state_map] : joint_states_) {
      for (auto &[iface_name, state] : iface_state_map) {
        ifaces.emplace_back(joint_name, iface_name, &state);
      }
    }
    return ifaces;
  }

  void assign_interfaces(std::vector<hi::LoanedStateInterface> &parent_loaned_states) {
    // loan state handles for actuators of interest from parent
    {
      std::vector<hi::LoanedStateInterface> returned_states;
      for (auto &state : parent_loaned_states) {
        if (actuator_states_.count(state.get_prefix_name()) > 0) {
          loaned_states_.push_back(std::move(state));
        } else {
          returned_states.push_back(std::move(state));
        }
      }
      parent_loaned_states = std::move(returned_states);
    }

    // complete allocation of actuator state variables
    for (const auto &state : loaned_states_) {
      actuator_states_[state.get_prefix_name()][state.get_interface_name()] =
          std::numeric_limits<double>::quiet_NaN();
    }

    // configure converter
    // 1. create joint state handles, which hold references to joint states
    std::vector<ti::JointHandle> joint_handles;
    for (auto &[joint_name, iface_state_map] : joint_states_) {
      for (auto &[iface_name, state] : iface_state_map) {
        joint_handles.emplace_back(joint_name, iface_name, &state);
      }
    }
    // 2. create actuator state handles, which hold references to actuator states
    std::vector<ti::ActuatorHandle> actuator_handles;
    for (auto &[ator_name, iface_state_map] : actuator_states_) {
      for (auto &[iface_name, state] : iface_state_map) {
        actuator_handles.emplace_back(ator_name, iface_name, &state);
      }
    }
    // 3. configure converter with handles
    try {
      converter_->configure(joint_handles, actuator_handles);
    } catch (const ti::Exception &error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "ActuatorToJointTransmission::assign_interfaces(): "
                              << "Failed to configure converter for \"" << trans_info_.name
                              << "\" transmission: " << error.what());
      return;
    }
  }

  void actuator_to_joint() {
    // get actuator state from external variables
    for (const auto &state : loaned_states_) {
      actuator_states_[state.get_prefix_name()][state.get_interface_name()] = state.get_value();
    }

    // convert actuator states to joint states
    converter_->actuator_to_joint();
  }

protected:
  const hi::TransmissionInfo trans_info_;

  // value converter from actuator to joint space
  const ti::TransmissionSharedPtr converter_;
  // map of variables in joint and actuator space to be given to the converter,
  // which maps joint or actuator name (ex. "xxx_joint", "xxx_actuator"),
  // and interface name (ex. "position", "velocity") to variable
  std::map<std::string, std::map<std::string, double>> joint_states_, actuator_states_;

  // references to external variables owned by other layers.
  // we should import these values before conversion.
  std::vector<hi::LoanedStateInterface> loaned_states_;
};

} // namespace layered_hardware

#endif