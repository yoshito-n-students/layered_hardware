#include <layered_hardware/dummy_actuator_layers.hpp>
#include <layered_hardware/joint_limits_layer.hpp>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware/transmission_layer.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(layered_hardware::DummyEffortActuatorLayer, layered_hardware::LayerBase);
PLUGINLIB_EXPORT_CLASS(layered_hardware::DummyPositionActuatorLayer, layered_hardware::LayerBase);
PLUGINLIB_EXPORT_CLASS(layered_hardware::DummyVelocityActuatorLayer, layered_hardware::LayerBase);
PLUGINLIB_EXPORT_CLASS(layered_hardware::JointLimitsLayer, layered_hardware::LayerBase);
PLUGINLIB_EXPORT_CLASS(layered_hardware::TransmissionLayer, layered_hardware::LayerBase);