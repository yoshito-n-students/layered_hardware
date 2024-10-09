#ifndef LAYERED_HARDWARE_COMMON_NAMESPACES_HPP
#define LAYERED_HARDWARE_COMMON_NAMESPACES_HPP

namespace controller_interface {}

namespace hardware_interface {}

namespace joint_limits {}

namespace transmission_interface {}

namespace layered_hardware {
namespace ci = controller_interface;
namespace hi = hardware_interface;
namespace jl = joint_limits;
namespace ti = transmission_interface;
} // namespace layered_hardware

#endif