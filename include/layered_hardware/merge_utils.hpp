#ifndef LAYERED_HARDWARE_MERGE_UTILS_HPP
#define LAYERED_HARDWARE_MERGE_UTILS_HPP

#include <algorithm>
#include <iterator>
#include <string>
#include <utility> // for std::move()
#include <vector>

#include <controller_interface/controller_interface_base.hpp> // for ci::InterfaceConfiguration
#include <hardware_interface/handle.hpp>                      // for hi::{State,Command}Interface
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <layered_hardware/common_namespaces.hpp>

#include <yaml-cpp/yaml.h>

namespace layered_hardware {

// merge two vectors of hi::StateInterface by move
static inline std::vector<hi::StateInterface> merge(std::vector<hi::StateInterface> &&a,
                                                    std::vector<hi::StateInterface> &&b) {
  std::vector<hi::StateInterface> result;
  result.reserve(a.size() + b.size());
  std::move(a.begin(), a.end(), std::back_inserter(result));
  std::move(b.begin(), b.end(), std::back_inserter(result));
  return result;
}

// merge two vectors of hi::CommandInterface by move
static inline std::vector<hi::CommandInterface> merge(std::vector<hi::CommandInterface> &&a,
                                                      std::vector<hi::CommandInterface> &&b) {
  std::vector<hi::CommandInterface> result;
  result.reserve(a.size() + b.size());
  std::move(a.begin(), a.end(), std::back_inserter(result));
  std::move(b.begin(), b.end(), std::back_inserter(result));
  return result;
}

// merge two ci::InterfaceConfiguration by move
static inline ci::InterfaceConfiguration merge(ci::InterfaceConfiguration &&a,
                                               ci::InterfaceConfiguration &&b) {
  using conf_t = ci::interface_configuration_type;
  // ALL + {ALL or INDIVIDUAL or NONE} -> ALL
  if (a.type == conf_t::ALL || b.type == conf_t::ALL) {
    return {conf_t::ALL, {}};
  }
  // INDIVIDUAL + {INDIVIDUAL or NONE} -> INDIVIDUAL
  if (a.type == conf_t::INDIVIDUAL && b.type == conf_t::INDIVIDUAL) {
    std::vector<std::string> names;
    names.reserve(a.names.size() + b.names.size());
    std::move(a.names.begin(), a.names.end(), std::back_inserter(names));
    std::move(b.names.begin(), b.names.end(), std::back_inserter(names));
    return {conf_t::INDIVIDUAL, std::move(names)};
  } else if (a.type == conf_t::INDIVIDUAL) {
    return {conf_t::INDIVIDUAL, std::move(a.names)};
  } else if (b.type == conf_t::INDIVIDUAL) {
    return {conf_t::INDIVIDUAL, std::move(b.names)};
  }
  // NONE + NONE -> NONE
  return {conf_t::NONE, {}};
}

// merge two hi::return_type by taking worse value
static inline hi::return_type merge(const hi::return_type a, const hi::return_type b) {
  using ret_t = hi::return_type;
  // ERROR + {ERROR or OK} -> ERROR
  if (a == ret_t::ERROR || b == ret_t::ERROR) {
    return ret_t::ERROR;
  }
  // OK + OK -> OK
  return ret_t::OK;
}

} // namespace layered_hardware

#endif