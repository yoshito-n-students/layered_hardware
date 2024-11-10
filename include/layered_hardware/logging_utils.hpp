#ifndef LAYERED_HARDWARE_LOGGING_UTILS_HPP
#define LAYERED_HARDWARE_LOGGING_UTILS_HPP

#include <exception>
#include <string>
#include <type_traits>
#include <utility> // for std::forward()

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace layered_hardware {

// returns reference to the common logger without construction overhead
static inline rclcpp::Logger &get_lh_logger() {
  static rclcpp::Logger logger = rclcpp::get_logger("layered_hardware");
  return logger;
}

// helper function to allow cpp-string and exception arguments for logging functions below.
// calls .c_str() for cpp-string arguments and .what() for exception arguments;
// forwards other types (typically numbers and c-string) using std::forward().
template <typename Arg> static inline auto to_format_arg(Arg &&arg) {
  if constexpr (std::is_same_v<std::string, std::remove_cv_t<std::remove_reference_t<Arg>>>) {
    return arg.c_str();
  } else if constexpr (std::is_base_of_v<std::exception, std::remove_reference_t<Arg>>) {
    return arg.what();
  } else {
    return std::forward<Arg>(arg);
  }
}

// logging functions which supports cpp-string arguments

template <typename... Args> static inline void lh_debug(const char *format, Args &&...args) {
  RCLCPP_DEBUG(get_lh_logger(), format, to_format_arg(args)...);
}

template <typename... Args> static inline void lh_info(const char *format, Args &&...args) {
  RCLCPP_INFO(get_lh_logger(), format, to_format_arg(args)...);
}

template <typename... Args> static inline void lh_warn(const char *format, Args &&...args) {
  RCLCPP_WARN(get_lh_logger(), format, to_format_arg(args)...);
}

template <typename... Args> static inline void lh_error(const char *format, Args &&...args) {
  RCLCPP_ERROR(get_lh_logger(), format, to_format_arg(args)...);
}

template <typename... Args> static inline void lh_fatal(const char *format, Args &&...args) {
  RCLCPP_FATAL(get_lh_logger(), format, to_format_arg(args)...);
}

} // namespace layered_hardware

#endif