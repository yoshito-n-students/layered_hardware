#ifndef LAYERED_HARDWARE_STRING_REGISTRY_HPP
#define LAYERED_HARDWARE_STRING_REGISTRY_HPP

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace layered_hardware {

class StringRegistry {
public:
  // modify the database by adding & erasing the string items
  void update(const std::vector<std::string> &add_list,
              const std::vector<std::string> &remove_list) {
    // if the same interface is both added and removed, reject the changes to the database
    const auto duplicate_it = std::find_first_of(add_list.begin(), add_list.end(),
                                                 remove_list.begin(), remove_list.end());
    if (duplicate_it != add_list.end()) {
      throw std::runtime_error("\"" + *duplicate_it +
                               "\" interface duplicates in add_list and remove_list");
    }

    // modify the database
    for (const auto &remove_val : remove_list) {
      interfaces_.erase(remove_val);
    }
    interfaces_.insert(add_list.begin(), add_list.end());
  }

  // find string items in the database
  std::vector<size_t> find(const std::vector<std::string> &search_list) const {
    std::vector<std::size_t> result;
    for (std::size_t i = 0; i < search_list.size(); ++i) {
      if (interfaces_.count(search_list[i])) {
        result.push_back(i);
      }
    }
    return result;
  }

private:
  std::set<std::string> interfaces_;
};

} // namespace layered_hardware

#endif