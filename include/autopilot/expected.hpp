
#ifndef AUTOPILOT_EXPECTED_HPP_
#define AUTOPILOT_EXPECTED_HPP_

#if __has_include(<expected>)
#include <expected>
#endif

#if defined(__cpp_lib_expected) && __cpp_lib_expected >= 202202L
namespace autopilot {
using std::expected;
using std::unexpected;
}  // namespace autopilot
#else
#include "third_party/tl/expected.hpp"
namespace autopilot {
using tl::expected;
using tl::unexpected;
}  // namespace autopilot
#endif

#endif  // AUTOPILOT_EXPECTED_HPP_
