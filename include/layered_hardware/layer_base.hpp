#ifndef LAYERED_HARDWARE_LAYER_BASE_HPP
#define LAYERED_HARDWARE_LAYER_BASE_HPP

#include <list>
#include <string>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/shared_ptr.hpp>

namespace layered_hardware {

class LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) = 0;

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) = 0;

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

protected:
  // as of Melodic, [val NodeHandle::param(key, default_val)] is not a const method
  // although [void NodeHandle::param(key, val_ref, default_val)] is.
  // this offers the former signature to const NodeHandle.
  template < typename T >
  static T param(const ros::NodeHandle &nh, const std::string &key, const T &default_val) {
    T val;
    nh.param(key, val, default_val);
    return val;
  }
};

// although this package is compiled as c++11,
// we still use boost::shared_ptr instead of std::shared_ptr
// because these pointer types should match returned types of pluginlib.
typedef boost::shared_ptr< LayerBase > LayerPtr;
typedef boost::shared_ptr< const LayerBase > LayerConstPtr;
} // namespace layered_hardware

#endif