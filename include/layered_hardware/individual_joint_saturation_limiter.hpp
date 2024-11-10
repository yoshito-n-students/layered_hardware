#ifndef LAYERED_HARDWARE_INDIVIDUAL_JOINT_SATURATION_LIMITTER_HPP
#define LAYERED_HARDWARE_INDIVIDUAL_JOINT_SATURATION_LIMITTER_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_saturation_limiter.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/logging_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

namespace layered_hardware {

class IndividualJointSaturationLimiter {
public:
  IndividualJointSaturationLimiter(const std::string &joint_name,
                                   const std::shared_ptr<rclcpp::Node> &node,
                                   const std::string &robot_description_topic)
      : joint_name_(joint_name) {
    // load limits for command_limiter from node params and robot_description topic
    if (!command_limiter_.init(std::vector<std::string>(1, joint_name_), node,
                               robot_description_topic)) {
      throw std::runtime_error("Failed to init command limiter for \"" + joint_name_ + "\" joint");
    }
  }

  hi::return_type configure(std::vector<hi::LoanedStateInterface> &parent_loaned_states,
                            std::vector<hi::LoanedCommandInterface> &parent_loaned_commands) {
    // loan state interfaces for the joint of interest from parent
    loaned_states_.move_from(parent_loaned_states, joint_name_);

    // loan command interfaces for the joint of interest from parent
    loaned_commands_.move_from(parent_loaned_commands, joint_name_);

    // configure command_limiter with current states of the joint
    if (!command_limiter_.configure(loaned_states_.to_data())) {
      lh_error("IndividualJointSaturationLimiter::configure(): "
               "Failed to configure command limiter for \"%s\" joint",
               joint_name_);
      return hi::return_type::ERROR;
    }

    return hi::return_type::OK;
  }

  void enforce(const rclcpp::Duration &period) {
    // get states & commands for the joint
    jl::JointLimitsStateDataType state_data = loaned_states_.to_data(),
                                 command_data = loaned_commands_.to_data();

    // enforce limits to command values
    // (command_limiter_.enforce() returns true if limits are enforced, but we simply ignore)
    command_limiter_.enforce(state_data, command_data, period);

    // apply modified commands to the command interfaces for the joint
    loaned_commands_.apply_data(command_data);
  }

protected:
  // strage for interfaces of a single joint
  template <typename Interface> class JointInterfaces {
  public:
    // move source interfaces matching joint_name to internal storage
    void move_from(std::vector<Interface> &src_ifaces, const std::string &joint_name) {
      std::vector<Interface> returned_ifaces;
      for (auto &src_iface : src_ifaces) {
        const bool joint_name_matched = (src_iface.get_prefix_name() == joint_name);
        const std::string src_iface_name = src_iface.get_interface_name();
        if (joint_name_matched && src_iface_name == hi::HW_IF_POSITION) {
          pos_iface_.emplace(std::move(src_iface));
        } else if (joint_name_matched && src_iface_name == hi::HW_IF_VELOCITY) {
          vel_iface_.emplace(std::move(src_iface));
        } else if (joint_name_matched && src_iface_name == hi::HW_IF_ACCELERATION) {
          acc_iface_.emplace(std::move(src_iface));
        } else if (joint_name_matched && src_iface_name == hi::HW_IF_EFFORT) {
          eff_iface_.emplace(std::move(src_iface));
        } else {
          returned_ifaces.emplace_back(std::move(src_iface));
        }
      }
      src_ifaces = std::move(returned_ifaces);
    }

    // export values on internal interfaces as Data
    jl::JointLimitsStateDataType to_data() const {
      static const auto to_vector = [](const std::optional<Interface> &iface) {
        // returns the value from interface in a vector,
        // or an empty vector if interface is unavailable or value is NaN.
        const double value =
            (iface ? iface->get_value() : std::numeric_limits<double>::quiet_NaN());
        return (!std::isnan(value)) ? std::vector<double>(1, value) : std::vector<double>();
      };

      jl::JointLimitsStateDataType data;
      data.positions = to_vector(pos_iface_);
      data.velocities = to_vector(vel_iface_);
      data.accelerations = to_vector(acc_iface_);
      data.effort = to_vector(eff_iface_);
      return data;
    }

    // update values on internal interfaces with given data
    void apply_data(const jl::JointLimitsStateDataType &data) {
      static const auto apply_vector = [](std::optional<Interface> &iface,
                                          const std::vector<double> &vec) {
        if (iface && !vec.empty()) {
          iface->set_value(vec.front());
        }
      };

      apply_vector(pos_iface_, data.positions);
      apply_vector(vel_iface_, data.velocities);
      apply_vector(acc_iface_, data.accelerations);
      apply_vector(eff_iface_, data.effort);
    }

  private:
    std::optional<Interface> pos_iface_, vel_iface_, acc_iface_, eff_iface_;
  };

protected:
  const std::string joint_name_;
  jl::JointSaturationLimiter<jl::JointLimits> command_limiter_;
  JointInterfaces<hi::LoanedCommandInterface> loaned_commands_;
  JointInterfaces<hi::LoanedStateInterface> loaned_states_;
};

} // namespace layered_hardware

#endif