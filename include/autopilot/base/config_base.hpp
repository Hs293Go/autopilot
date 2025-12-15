#ifndef AUTOPILOT_BASE_HPP_
#define AUTOPILOT_BASE_HPP_

#include "autopilot/core/definitions.hpp"
namespace autopilot {

struct ConfigBase {
  virtual ~ConfigBase() = default;

  // Returns the unique key (e.g., "GeometricPosition") associated with this
  // config
  [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace autopilot

#endif  // AUTOPILOT_BASE_HPP_
