#ifndef LAYERED_HARDWARE_DUMMY_ACTUATOR_LAYERS_HPP
#define LAYERED_HARDWARE_DUMMY_ACTUATOR_LAYERS_HPP

#include <list>
#include <map>
#include <string>
#include <vector>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_base.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

namespace layered_hardware {

template < typename CommandInterface, typename CommandHandle, typename CommandWriter >
class DummyActuatorLayer : public LayerBase {
public:
  virtual bool init(hi::RobotHW &hw, ros::NodeHandle &param_nh, const std::string &urdf_str) {
    // register actuator interfaces to the hardware so that other layers can find the interfaces
    if (!hw.get< hi::ActuatorStateInterface >()) {
      hw.registerInterface(&getStateInterface());
    }
    hi::ActuatorStateInterface &state_iface(*hw.get< hi::ActuatorStateInterface >());
    if (!hw.get< CommandInterface >()) {
      hw.registerInterface(&getCommandInterface());
    }
    CommandInterface &cmd_iface(*hw.get< CommandInterface >());

    // get actuator names from param
    std::vector< std::string > ator_names;
    if (!param_nh.getParam("actuators", ator_names)) {
      ROS_ERROR_STREAM("DummyActuatorLayer::init(): Failed to get param '"
                       << param_nh.resolveName("actuators") << "'");
      return false;
    }

    // register all actuators defined in URDF
    BOOST_FOREACH (const std::string &ator_name, ator_names) {
      ActuatorData &data(data_map_[ator_name]);
      data.pos = 0.;
      data.vel = 0.;
      data.eff = 0.;
      data.cmd = 0.;

      const hi::ActuatorStateHandle state_handle(ator_name, &data.pos, &data.vel, &data.eff);
      state_iface.registerHandle(state_handle);

      const CommandHandle cmd_handle(state_handle, &data.cmd);
      cmd_iface.registerHandle(cmd_handle);

      ROS_INFO_STREAM("DummyActuatorLayer::init(): Initialized the actuator '" << ator_name << "'");
    }

    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // nothing to do
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // noting to do
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write to all actuators
    BOOST_FOREACH (typename ActuatorDataMap::value_type &data, data_map_) {
      CommandWriter::write(&data.second.pos, &data.second.vel, &data.second.eff, data.second.cmd,
                           period);
    }
  }

private:
  static hi::ActuatorStateInterface &getStateInterface() {
    static hi::ActuatorStateInterface state_iface;
    return state_iface;
  }

  static CommandInterface &getCommandInterface() {
    static CommandInterface cmd_iface;
    return cmd_iface;
  }

private:
  struct ActuatorData {
    double pos, vel, eff, cmd;
  };
  typedef std::map< std::string, ActuatorData > ActuatorDataMap;

  ActuatorDataMap data_map_;
};

// command writers

struct ActuatorPositionCommandWriter {
  static void write(double *pos, double *vel, double *eff, const double pos_cmd,
                    const ros::Duration &period) {
    if (!boost::math::isnan(pos_cmd)) {
      *vel = (pos_cmd - *pos) / period.toSec();
      *pos = pos_cmd;
      *eff = 0.;
    }
  }
};

struct ActuatorVelocityCommandWriter {
  static void write(double *pos, double *vel, double *eff, const double vel_cmd,
                    const ros::Duration &period) {
    if (!boost::math::isnan(vel_cmd)) {
      *pos += vel_cmd * period.toSec();
      *vel = vel_cmd;
      *eff = 0.;
    }
  }
};

struct ActuatorEffortCommandWriter {
  static void write(double *pos, double *vel, double *eff, const double eff_cmd,
                    const ros::Duration &period) {
    if (!boost::math::isnan(eff_cmd)) {
      *pos = 0.;
      *vel = 0.;
      *eff = eff_cmd;
    }
  }
};

// layer definitions

typedef DummyActuatorLayer< hi::PositionActuatorInterface, hi::ActuatorHandle,
                            ActuatorPositionCommandWriter >
    DummyPositionActuatorLayer;

typedef DummyActuatorLayer< hi::VelocityActuatorInterface, hi::ActuatorHandle,
                            ActuatorVelocityCommandWriter >
    DummyVelocityActuatorLayer;

typedef DummyActuatorLayer< hi::EffortActuatorInterface, hi::ActuatorHandle,
                            ActuatorEffortCommandWriter >
    DummyEffortActuatorLayer;
} // namespace layered_hardware

#endif