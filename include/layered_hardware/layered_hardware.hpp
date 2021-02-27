#ifndef LAYERED_HARDWARE_LAYERED_HARDWARE_HPP
#define LAYERED_HARDWARE_LAYERED_HARDWARE_HPP

#include <list>
#include <string>
#include <vector>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

#include <boost/foreach.hpp>

namespace layered_hardware {

class LayeredHardware : public hi::RobotHW {
public:
  LayeredHardware() : layer_loader_("layered_hardware", "layered_hardware::LayerBase") {}

  virtual ~LayeredHardware() {
    // before destructing layer loader,
    // deallocate layers which were created by plugins or loader cannot unload plugins
    layers_.clear();
  }

  bool init(const ros::NodeHandle &param_nh) {
    // get URDF description from param
    const std::string urdf_str(getURDFStr(param_nh));
    if (urdf_str.empty()) {
      ROS_WARN("LayeredHardware::init(): Failed to get URDF description. "
               "Every layers will be initialized with empty string.");
      // continue because layers may not require robot description
    }

    // get layer names from param
    std::vector< std::string > layer_names;
    if (!param_nh.getParam("layers", layer_names)) {
      ROS_ERROR_STREAM("LayeredHardware::init(): Failed to get param '"
                       << param_nh.resolveName("layers") << "'");
      return false;
    }
    if (layer_names.empty()) {
      ROS_ERROR_STREAM("LayeredHardware::init(): Param '" << param_nh.resolveName("layers")
                                                          << "' must be a string array");
      return false;
    }

    // load & init layer instances from bottom (actuator-side) to upper (controller-side)
    layers_.resize(layer_names.size());
    for (int i = layers_.size() - 1; i >= 0; --i) {
      const std::string &layer_name(layer_names[i]);
      LayerPtr &layer(layers_[i]);
      const ros::NodeHandle layer_param_nh(param_nh, layer_name);

      // get layer's typename from param
      std::string lookup_name;
      if (!layer_param_nh.getParam("type", lookup_name)) {
        ROS_ERROR_STREAM("LayeredHardware::init(): Failed to get param '"
                         << layer_param_nh.resolveName("type") << "'");
        return false;
      }

      // create a layer instance by typename
      layer = loadLayer(lookup_name, layer_param_nh, urdf_str);
      if (!layer) {
        ROS_ERROR_STREAM("LayeredHardware::init(): Failed to load the layer '" << layer_name
                                                                               << "'");
        return false;
      }

      ROS_INFO_STREAM("LayeredHardware::init(): Initialized the layer '" << layer_name << "'");
    }

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) override {
    // ask each layers if stopping/starting given controllers is possible
    BOOST_REVERSE_FOREACH(const LayerPtr &layer, layers_) {
      if (!layer->prepareSwitch(start_list, stop_list)) {
        return false;
      }
    }
    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) override {
    // do something required on just before switching controllers
    BOOST_REVERSE_FOREACH(const LayerPtr &layer, layers_) {
      layer->doSwitch(start_list, stop_list);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) override {
    // read states from actuators
    BOOST_REVERSE_FOREACH(const LayerPtr &layer, layers_) { layer->read(time, period); }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) override {
    // write commands to actuators
    for (const LayerPtr &layer : layers_) {
      layer->write(time, period);
    }
  }

  std::size_t size() const { return layers_.size(); }

  LayerPtr layer(const std::size_t i) { return layers_[i]; }

  LayerConstPtr layer(const std::size_t i) const { return layers_[i]; }

protected:
  static std::string getURDFStr(const ros::NodeHandle &param_nh) {
    std::string urdf_str;
    if (!param_nh.getParam("robot_description", urdf_str) &&
        !ros::param::get("robot_description", urdf_str)) {
      ROS_WARN_STREAM(
          "LayeredHardware::getURDFStr(): Failed to get URDF description from params neither '"
          << param_nh.resolveName("robot_description") << "' nor '"
          << ros::names::resolve("robot_description"));
      return std::string();
    }
    return urdf_str;
  }

  LayerPtr loadLayer(const std::string &lookup_name, const ros::NodeHandle &param_nh,
                     const std::string &urdf_str) {
    LayerPtr layer;

    // create a layer by typename
    try {
      layer = layer_loader_.createInstance(lookup_name);
    } catch (const pluginlib::PluginlibException &ex) {
      ROS_ERROR_STREAM("LayeredHardware::loadLayer(): Failed to create a layer by the lookup name '"
                       << lookup_name << "': " << ex.what());
      return LayerPtr();
    }

    // init the layer
    if (!layer->init(this, param_nh, urdf_str)) {
      ROS_ERROR_STREAM("LayeredHardware::loadLayer(): Failed to initialize a layer");
      return LayerPtr();
    }

    return layer;
  }

protected:
  pluginlib::ClassLoader< LayerBase > layer_loader_;
  std::vector< LayerPtr > layers_;
};
} // namespace layered_hardware

#endif