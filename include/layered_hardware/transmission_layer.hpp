#ifndef LAYERED_HARDWARE_TRANSMISSION_LAYER_HPP
#define LAYERED_HARDWARE_TRANSMISSION_LAYER_HPP

#include <algorithm>
#include <list>
#include <string>
#include <vector>

#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_parser.h>

#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

namespace layered_hardware {

class TransmissionLayer : public LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    // extract transmission informations from URDF
    std::vector< ti::TransmissionInfo > infos;
    if (!ti::TransmissionParser::parse(urdf_str, infos)) {
      ROS_ERROR("TransmissionLayer::init(): Failed to parse transmissions from URDF");
      return false;
    }

    // get actuator names already registered in the hardware
    const hi::ActuatorStateInterface *const ator_iface(hw->get< hi::ActuatorStateInterface >());
    if (!ator_iface) {
      ROS_ERROR("TransmissionLayer::init(): No actuator registered");
      return false;
    }
    const std::vector< std::string > hw_ator_names(ator_iface->getNames());

    // load all transmissions for actuators in the hardware
    iface_loader_.reset(new ti::TransmissionInterfaceLoader(hw, &transmissions_));
    BOOST_FOREACH (const ti::TransmissionInfo &info, infos) {
      // confirm the transmission is for some of actuators in the hardware
      bool has_non_hw_ator(false);
      BOOST_FOREACH (const ti::ActuatorInfo &ator_info, info.actuators_) {
        if (std::find(hw_ator_names.begin(), hw_ator_names.end(), ator_info.name_) ==
            hw_ator_names.end()) {
          has_non_hw_ator = true;
          break;
        }
      }
      if (has_non_hw_ator) {
        continue;
      }
      // load the transmission.
      // this will register handles of joints associated to the transmission to the hardware.
      if (!iface_loader_->load(info)) {
        ROS_ERROR_STREAM("TransmissionLayer::init(): Failed to load the transmission "
                         << info.name_);
        return false;
      }
      ROS_INFO_STREAM("TransmissionLayer::init(): Initialized the transmission '" << info.name_
                                                                                  << "'");
    }

    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // convert actuator states to joint states because the actuator states may be
    // changed/initialized by ActuatorLayer::doSwich() executed just before this.
    propagate< ti::ActuatorToJointStateInterface >();

    // instead, we do not have to convert joint commands to actuator commands
    // because the new controllers never started at this timing.
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read joint state from actuator state
    propagate< ti::ActuatorToJointStateInterface >();
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write joint commands to actuator commands
    propagate< ti::JointToActuatorPositionInterface >();
    propagate< ti::JointToActuatorVelocityInterface >();
    propagate< ti::JointToActuatorEffortInterface >();
  }

private:
  template < typename Interface > void propagate() {
    Interface *const iface(transmissions_.get< Interface >());
    if (iface) {
      iface->propagate();
    }
  }

private:
  ti::RobotTransmissions transmissions_;
  boost::scoped_ptr< ti::TransmissionInterfaceLoader > iface_loader_;
};
} // namespace layered_hardware

#endif