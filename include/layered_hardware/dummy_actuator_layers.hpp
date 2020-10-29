#ifndef LAYERED_HARDWARE_DUMMY_ACTUATOR_LAYERS_HPP
#define LAYERED_HARDWARE_DUMMY_ACTUATOR_LAYERS_HPP

#include <cmath>
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

namespace layered_hardware {

struct DummyActuatorData {
  DummyActuatorData() : pos(0.), vel(0.), eff(0.), pos_cmd(0.), vel_cmd(0.), eff_cmd(0.) {}
  double pos, vel, eff, pos_cmd, vel_cmd, eff_cmd;
};

template < typename CommandWriter > class DummyActuatorLayer : public LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    // register actuator interfaces to the hardware so that other layers can find the interfaces
    hi::ActuatorStateInterface &state_iface(*makeRegistered< hi::ActuatorStateInterface >(hw));
    hi::PositionActuatorInterface &pos_cmd_iface(
        *makeRegistered< hi::PositionActuatorInterface >(hw));
    hi::VelocityActuatorInterface &vel_cmd_iface(
        *makeRegistered< hi::VelocityActuatorInterface >(hw));
    hi::EffortActuatorInterface &eff_cmd_iface(*makeRegistered< hi::EffortActuatorInterface >(hw));

    // get actuator names from param
    std::vector< std::string > ator_names;
    if (!param_nh.getParam("actuators", ator_names)) {
      ROS_ERROR_STREAM("DummyActuatorLayer::init(): Failed to get param '"
                       << param_nh.resolveName("actuators") << "'");
      return false;
    }

    // register all actuators defined in URDF
    for (const std::string &ator_name : ator_names) {
      DummyActuatorData &data(data_map_[ator_name]);

      const hi::ActuatorStateHandle state_handle(ator_name, &data.pos, &data.vel, &data.eff);
      state_iface.registerHandle(state_handle);
      pos_cmd_iface.registerHandle(hi::ActuatorHandle(state_handle, &data.pos_cmd));
      vel_cmd_iface.registerHandle(hi::ActuatorHandle(state_handle, &data.vel_cmd));
      eff_cmd_iface.registerHandle(hi::ActuatorHandle(state_handle, &data.eff_cmd));

      ROS_INFO_STREAM("DummyActuatorLayer::init(): Initialized the actuator '" << ator_name << "'");
    }

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    // always ready to switch
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
    for (typename ActuatorDataMap::value_type &data : data_map_) {
      CommandWriter::write(&data.second, period);
    }
  }

protected:
  // makes an interface registered & returns the registered interface.
  // if newly register an interface, it will be allocated in the static memory space
  // so that other plugins can modify it.
  template < typename Interface > static Interface *makeRegistered(hi::RobotHW *const hw) {
    if (!hw->get< Interface >()) {
      static Interface iface;
      hw->registerInterface(&iface);
    }
    return hw->get< Interface >();
  }

protected:
  typedef std::map< std::string, DummyActuatorData > ActuatorDataMap;
  ActuatorDataMap data_map_;
};

// command writers

struct DummyActuatorPositionCommandWriter {
  static void write(DummyActuatorData *const data, const ros::Duration &period) {
    if (!std::isnan(data->pos_cmd)) {
      data->vel = (data->pos_cmd - data->pos) / period.toSec();
      data->pos = data->pos_cmd;
      data->eff = 0.;
    }
  }
};

struct DummyActuatorVelocityCommandWriter {
  static void write(DummyActuatorData *const data, const ros::Duration &period) {
    if (!std::isnan(data->vel_cmd)) {
      data->pos += data->vel_cmd * period.toSec();
      data->vel = data->vel_cmd;
      data->eff = 0.;
    }
  }
};

struct DummyActuatorEffortCommandWriter {
  static void write(DummyActuatorData *const data, const ros::Duration &period) {
    if (!std::isnan(data->eff_cmd)) {
      data->pos = 0.;
      data->vel = 0.;
      data->eff = data->eff_cmd;
    }
  }
};

// layer definitions
typedef DummyActuatorLayer< DummyActuatorPositionCommandWriter > DummyPositionActuatorLayer;
typedef DummyActuatorLayer< DummyActuatorVelocityCommandWriter > DummyVelocityActuatorLayer;
typedef DummyActuatorLayer< DummyActuatorEffortCommandWriter > DummyEffortActuatorLayer;
} // namespace layered_hardware

#endif