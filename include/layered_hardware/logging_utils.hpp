#ifndef LAYERED_HARDWARE_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_LOGGING_UTILS_HPP

#include <rclcpp/logging.hpp>

#define LH_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("layered_hardware"), __VA_ARGS__)
#define LH_INFO(...) RCLCPP_INFO(rclcpp::get_logger("layered_hardware"), __VA_ARGS__)
#define LH_WARN(...) RCLCPP_WARN(rclcpp::get_logger("layered_hardware"), __VA_ARGS__)
#define LH_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("layered_hardware"), __VA_ARGS__)
#define LH_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("layered_hardware"), __VA_ARGS__)

#endif