# layered_hardware
A ros2_control implementation that adopts layered scheme

## The layered scheme
* every ros2_control's component (ex. joint_limits, transmissions) is implemented as a layer plugin (base_class: [layered_hardware::LayerInterface](include/layered_hardware/layer_interface.hpp))
* one can reuse plugins of non-actuator-specific layers for different actuators

![](https://raw.githubusercontent.com/yoshito-n-students/layered_hardware/images/images/layered_scheme_ros2.png)

## Plugins: layered_hardware_hardware_plugins
### layered_hardware/LayeredHardware
#### Hardware parameters
___layers___ (yaml, required)
* sequence of layers to be loaded by this ros2_control hardware

___layers[*].name___ (string, required)
* arbitary name of the layer

___layers[*].type___ (string, required)
* valid type name of the layer
* the type must be exported to the layered_hardware package
* the base class of the type must be [layered_hardware::LayerInterface](include/layered_hardware/layer_interface.hpp)

#### Example of ros2_control-tag in your robot description
```xml
<ros2_control name="LayeredHardware" type="system">
    <hardware>
        <plugin>layered_hardware/LayeredHardware</plugin>
        <param name="layers">
            - name: example_layer
              type: layered_hardware/ExampleLayer
            - name: ...
        </param>
        ...
    </hardware>
    ...
</ros2_control>
```

## Plugins: layered_hardware_layer_plugins
### layered_hardware/JointLimitsLayer
* applies limits to all joint command interfaces within the `write()` function

### layered_hardware/TransmissionLayer
* converts joint commands to actuator commands using reduction ratio of transmission within `write()` function
* converts actuator states to joint states within `read()` function

### layered_hardware/MockActuatorLayer
* implements mock {position, velocity, effort}-controlled actuators
* switches mock actuators' command modes within `perform_command_mode_swtich()` function when controllers using associated interfaces activate
* changes actuator states based on commands within `write()` function
* useful to debug your command generation, state visualization nodes, or transmissions without physical actuators and dynamics simulators

#### Hardware parameters
___<layer_name>___ (yaml, required)
* map of parameter names and values for this layer

___<layer_name>.actuators___ (map, required)
* map of parameters for each mock actuator

___<layer_name>.actuators.<actuator_name>.command_mode_map___ (map, required)
* map to actuator command mode names (`position`, `velocity`, `effort`) from associated interface names (typically joint interfaces)


#### Example of parameter description
```xml
<param name="example_mock_actuator_layer">
    actuators:
        example_actuator_1:
            command_mode_map:
                example_joint_1/position: position
                ...
        example_actuator_2:
            ...
</param>
```

### layered_hardware/MonitorLayer
* prints changes on commands and states within `read()` function for debug or logging purpose

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