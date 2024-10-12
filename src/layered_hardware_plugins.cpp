#include <layered_hardware/joint_limits_layer.hpp>
#include <layered_hardware/layer_interface.hpp>
#include <layered_hardware/layered_hardware.hpp>
#include <layered_hardware/mock_actuator_layer.hpp>
#include <layered_hardware/monitor_layer.hpp>
#include <layered_hardware/transmission_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(layered_hardware::JointLimitsLayer, layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::MockActuatorLayer, layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::MonitorLayer, layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::TransmissionLayer, layered_hardware::LayerInterface);
PLUGINLIB_EXPORT_CLASS(layered_hardware::LayeredHardware, hardware_interface::SystemInterface);