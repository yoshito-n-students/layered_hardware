#include <layered_hardware/joint_command_clamper_layer.hpp>
#include <layered_hardware/joint_saturation_limiter_layer.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/mock_actuator_layer.hpp>
#include <layered_hardware/monitor_layer.hpp>
#include <layered_hardware/transmission_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware::JointCommandClamperLayer,
                       layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::JointSaturationLimiterLayer,
                       layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::MockActuatorLayer, layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::MonitorLayer, layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::TransmissionLayer, layered_hardware::LayerInterface);