# layered_hardware
A ros2_control implementation that adopts layered scheme

## The layered scheme
* every ros2_control's component (ex. joint_limits, transmissions) is implemented as a layer plugin (base_class: [layered_hardware::LayerInterface](include/layered_hardware/layer_interface.hpp))
* one can reuse plugins of non-actuator-specific layers for different actuators

![](https://raw.githubusercontent.com/yoshito-n-students/layered_hardware/images/images/layered_scheme_ros2.png)

## Plugins: layered_hardware_hardware_plugins
### layered_hardware/LayeredHardware
#### Parameters
```xml
<ros2_control name="LayeredHardware" type="system">
    <hardware>
        <plugin>layered_hardware/LayeredHardware</plugin>
        <param name="layers">
            - name: example_layer # string, required
              type: layered_hardware/ExampleLayer # string, required
            - name: ...
        </param>
        <param name="example_layer">
            ... # parameters for the layer
        </param>
    </hardware>
    ...
</ros2_control>
```

## Plugins: layered_hardware_layer_plugins
### layered_hardware/JointLimitsLayer
* implements general joint_limits procedures

### layered_hardware/TransmissionLayer
* implements general transmission_interface procedures

### layered_hardware/MockActuatorLayer
* implements mock {position, velocity, effort}-controlled actuators
* useful to debug your command generation, state visualization nodes, or transmissions without physical actuators and dynamics simulators
#### Parameters
```xml
<param name="example_mock_actuator_layer">
    actuators:
        example_actuator_1:
            command_mode_map:
                # map from actuator's command mode (position, velocity, or effort)
                # to interface bound to the mode
                example_joint_1/position: position
                ...
        example_actuator_2:
            command_mode_map:
                ...
</param>
```

### layered_hardware/MonitorLayer
* monitors changes on commands and states owned by other layers for debug or logging purpose

## Examples
see [examples](examples)

## Related packages
**[layered_hardware_dynamixel](https://github.com/yoshito-n-students/layered_hardware_dynamixel/tree/jazzy)**
* layer implementation for ROBOTIS Dynamixel actuators

**[layered_hardware_epos](https://github.com/yoshito-n-students/layered_hardware_epos/tree/jazzy)**
* layer implementation for maxon EPOS actuator controllers

**[layered_hardware_unitree](https://github.com/yoshito-n-students/layered_hardware_unitree/tree/jazzy)**
* layer implementation for Unitree actuators

**[layered_hardware_gz](https://github.com/yoshito-n-students/layered_hardware_ign/tree/jazzy)**
* layer implementation for joints of a virtual robot in the Ignition Gazebo simulator