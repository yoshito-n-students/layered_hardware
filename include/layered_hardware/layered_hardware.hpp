#ifndef LAYERED_HARDWARE_LAYERED_HARDWARE_HPP
#define LAYERED_HARDWARE_LAYERED_HARDWARE_HPP

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <layered_hardware/common_namespaces.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/merge_utils.hpp>
#include <layered_hardware/string_registry.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/exceptions.hpp>
#include <rclcpp/logging.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware {

class LayeredHardware : public hi::SystemInterface {
public:
  LayeredHardware() : layer_loader_("layered_hardware", "layered_hardware::LayerInterface") {}

  virtual ~LayeredHardware() {
    // before destructing layer loader,
    // deallocate layers which were created by plugins or loader cannot unload plugins
    layers_.clear();
  }

  virtual CallbackReturn on_init(const hi::HardwareInfo &hardware_info) override {
    // initialize the base class first
    const auto base_result = hi::SystemInterface::on_init(hardware_info);
    if (base_result != CallbackReturn::SUCCESS) {
      return base_result;
    }

    // check if "layers" parameter is given
    const auto layers_param_it = hardware_info.hardware_parameters.find("layers");
    if (layers_param_it == hardware_info.hardware_parameters.end()) {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("layered_hardware"),
                          "LayeredHardware::on_init(): " << "\"layers\" parameter is missing");
      return CallbackReturn::ERROR;
    }

    // parse the "layers" parameter as yaml
    std::vector<std::string> layer_names, layer_types;
    try {
      const YAML::Node layers_param = YAML::Load(layers_param_it->second);
      for (const YAML::Node &layer_param : layers_param) {
        layer_names.push_back(layer_param["name"].as<std::string>());
        layer_types.push_back(layer_param["type"].as<std::string>());
      }
    } catch (const YAML::Exception &error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                          "LayeredHardware::on_init(): " << error.what()
                                                         << " (on parsing \"layers\" parameter)");
      return CallbackReturn::ERROR;
    }

    // load & initialize layers according to "layers" parameter
    for (std::size_t i = 0; i < layer_names.size(); ++i) {
      const std::string layer_disp_name =
          "\"" + layer_names[i] + "\" layer (" + layer_types[i] + ")";
      RCLCPP_INFO_STREAM(rclcpp::get_logger("layered_hardware"),
                         "LayeredHardware::on_init(): " << "Loading " << layer_disp_name);
      // load layer
      std::unique_ptr<LayerInterface> layer;
      try {
        layer.reset(layer_loader_.createUnmanagedInstance(layer_types[i]));
      } catch (const pluginlib::PluginlibException &error) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                            "LayeredHardware::on_init(): " << error.what() << " (on creating "
                                                           << layer_disp_name << ")");
        return CallbackReturn::ERROR;
      }
      // initialize layer
      const CallbackReturn is_layer_initialized = layer->on_init(layer_names[i], hardware_info);
      if (is_layer_initialized != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("layered_hardware"),
                            "LayeredHardware::on_init(): " << "Failed to initialize "
                                                           << layer_disp_name);
        return is_layer_initialized;
      }
      // store successfully-loaded layer
      layers_.push_back(std::move(layer));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("layered_hardware"),
                         "LayeredHardware::on_init(): " << "Loaded " << layer_disp_name);
    }

    // check if one layer at least has been loaded??

    // populate command & state interfaces from each layer
    // (these interfaces are referenced by layers,
    //  so they must be member variables to match the layers' lifespan)
    state_interfaces_ = export_state_interfaces();
    command_interfaces_ = export_command_interfaces();

    // assign state & command interfaces to each layer
    for (auto &layer : layers_) {
      layer->assign_interfaces(loan_interfaces<hi::LoanedStateInterface>(
                                   state_interfaces_, layer->state_interface_configuration()),
                               loan_interfaces<hi::LoanedCommandInterface>(
                                   command_interfaces_, layer->command_interface_configuration()));
    }

    return CallbackReturn::SUCCESS;
  }

  virtual std::vector<hi::StateInterface> export_state_interfaces() override {
    std::vector<hi::StateInterface> ifaces;
    for (auto &layer : layers_) {
      ifaces = merge(std::move(ifaces), layer->export_state_interfaces());
    }
    return ifaces;
  }

  virtual std::vector<hi::CommandInterface> export_command_interfaces() override {
    std::vector<hi::CommandInterface> ifaces;
    for (auto &layer : layers_) {
      ifaces = merge(std::move(ifaces), layer->export_command_interfaces());
    }
    return ifaces;
  }

  virtual hi::return_type
  prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override {
    // update the list of active interfaces
    // (However, since the actual activation/deactivation does not occur at the time,
    // the update will be rolled back later)
    active_interfaces_.update(start_interfaces, stop_interfaces);

    // ask each layer whether updating the interfaces is acceptable
    hi::return_type result = hi::return_type::OK;
    for (auto &layer : layers_) {
      result = merge(result, layer->prepare_command_mode_switch(active_interfaces_));
    }

    // rollback the update of list
    active_interfaces_.update(stop_interfaces, start_interfaces);

    return result;
  }

  virtual hi::return_type
  perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                              const std::vector<std::string> &stop_interfaces) override {
    // update the list of active interfaces
    active_interfaces_.update(start_interfaces, stop_interfaces);

    // notigy each layers the update of the active interfaces to trigger layer's mode switch
    hi::return_type result = hi::return_type::OK;
    for (auto &layer : layers_) {
      result = merge(result, layer->perform_command_mode_switch(active_interfaces_));
    }

    return result;
  }

  virtual hi::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    //  iterate layers in reverse order
    hi::return_type result = hi::return_type::OK;
    for (auto layer = layers_.rbegin(); layer != layers_.rend(); ++layer) {
      result = merge(result, (*layer)->read(time, period));
    }
    return result;
  }

  virtual hi::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    hi::return_type result = hi::return_type::OK;
    for (const auto &layer : layers_) {
      result = merge(result, layer->write(time, period));
    }
    return result;
  }

private:
  // pick some of interfaces based on given config
  template <class LoanedIface, class Iface>
  static std::vector<LoanedIface> loan_interfaces(std::vector<Iface> &ifaces,
                                                  const ci::InterfaceConfiguration &config) {
    std::vector<LoanedIface> loaned_ifaces;
    switch (config.type) {
    case ci::interface_configuration_type::ALL:
      for (auto &iface : ifaces) {
        loaned_ifaces.emplace_back(iface);
      }
      break;
    case ci::interface_configuration_type::INDIVIDUAL:
      for (auto &iface : ifaces) {
        if (std::find(config.names.begin(), config.names.end(), iface.get_name()) !=
            config.names.end()) {
          loaned_ifaces.emplace_back(iface);
        }
      }
      break;
    case ci::interface_configuration_type::NONE:
      break;
    default:
      // TODO: warn unknown config type
      break;
    }
    return loaned_ifaces;
  }

protected:
  pluginlib::ClassLoader<LayerInterface> layer_loader_;
  std::vector<std::unique_ptr<LayerInterface>> layers_;
  std::vector<hi::StateInterface> state_interfaces_;
  std::vector<hi::CommandInterface> command_interfaces_;
  StringRegistry active_interfaces_;
};

} // namespace layered_hardware

#endif