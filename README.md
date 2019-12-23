# layered_hardware
A ros_control implementation that adopts layered scheme

## The layered scheme
* each ros_control's components (ex. joint_limits, transmissions) are implemented as a layer plugin (base_class: [layered_hardware::LayerBase](include/layered_hardware/layer_base.hpp))
* one can reuse same implementation of non-actuator-specific layers

![](https://raw.githubusercontent.com/yoshito-n-students/layered_hardware/images/images/layered_scheme.png)

## Node: layered_hardware_node
### Parameters
**~control_frequency** (double, default: 10.0)
* frequency of control step (reading from the layers, updating the controllers, and writing to the layers) in Hz

**~use_expected_period** (bool, default: false)
* if true, the node uses the expected control cycle time instead of the actual when reading from/writing from the layers
* useful as workaround for clock jump

**~layers** (string array, required)
* names of layers from upper (controller-side) to bottom (actuator-side)

**~<layer_name>/type** (string, required)
* lookup name for each layer plugin like 'layered_hardware/TransmissionLayer'

### Example
see [launch/example.launch](launch/example.launch)

## Plugins: layered_hardware_default_plugins
### layered_hardware/JointLimitsLayer
* implements general joint_limits_interface scheme
* supports only hard lmits. will support soft limits soon.

### layered_hardware/TransmissionLayer
* implements general transmission_interface scheme

### layered_hardware/Dummy{Position, Velocity, Effort}ActuatorLayer
* implements dummy {position, velocity, effort}-controlled actuators
* useful to debug your command generation, state visualization nodes, or transmissions without physical actuators and dynamics simulators
#### Parameters
**~<layer_name>/actuators** (string array, required)
* names of actuators to be managed by this layer

## Related packages
**[layered_hardware_dynamixel](https://github.com/yoshito-n-students/layered_hardware_dynamixel)**
* layer implementation for ROBOTIS Dynamixel actuators

**layered_hardware_epos**
* layer implementation for MAXOS EPOS actuator dirvers (coming soon)