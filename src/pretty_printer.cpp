#include "autopilot/extensions/pretty_printer.hpp"

#include "fmt/format.h"
#include "fmt/ranges.h"

using namespace std::literals;

template <typename T>
struct fmt::formatter<autopilot::Bounds<T>> : NoSpecifierFormatter {
  template <typename FormatContext>
  // FIX 1: Added 'const' to the function signature
  auto format(const autopilot::Bounds<T>& bounds, FormatContext& ctx) const {
    auto out = ctx.out();

    // Outputs are quoted because parenthesized pairs are not valid YAML
    std::visit(
        autopilot::Overload{[&](auto) { out = fmt::format_to(out, "\"(-inf"); },
                            [&](const autopilot::Inclusive<T>& inc) {
                              out = fmt::format_to(out, "\"[{},", inc.value);
                            },
                            [&](const autopilot::Exclusive<T>& exc) {
                              out = fmt::format_to(out, "\"({}, ", exc.value);
                            }},
        bounds.lower);

    std::visit(
        autopilot::Overload{[&](auto) { out = fmt::format_to(out, "inf)\""); },
                            [&](const autopilot::Inclusive<T>& inc) {
                              out = fmt::format_to(out, "{}]\"", inc.value);
                            },
                            [&](const autopilot::Exclusive<T>& exc) {
                              out = fmt::format_to(out, "{})\"", exc.value);
                            }},
        bounds.upper);

    return out;
  }
};

namespace autopilot {

template <typename T, typename P>
void PrettyPrinter::FormatKeyValue(std::ostream_iterator<char>& it,
                                   std::string_view key, const T& value,
                                   const P& props) const {
  auto active_width = options_.indent_width * indent_level_;
  if (!options_.show_details) {
    it = fmt::format_to(it, "{: >{}}{}: {}\n", "", active_width, key, value);
  }
  // 1. Print key
  it = fmt::format_to(it, "{: >{}}{}:\n", "", active_width, key);
  // 2. Indent, creating a subobject
  active_width += options_.indent_width;
  // 3a. Print value
  it = fmt::format_to(it, "{: >{}}value: {}\n", "", active_width, value);

  // 3b. Print common metadata: description, prefer_user_provided
  it = fmt::format_to(it, "{: >{}}description: {}\n", "", active_width,
                      props.desc);
  it = fmt::format_to(it, "{: >{}}prefer_user_provided: {}\n", "", active_width,
                      props.prefer_user_provided);

  // 3c. Print specialized metadata based on property type
  Overload{[](auto&& /*unused*/) {},
           [&it, active_width](const StrProperties& p) {
             it = fmt::format_to(it, "{: >{}}non_empty: {}\n", "", active_width,
                                 p.non_empty);
           },
           [&it, active_width]<typename S>(const NumericProperties<S>& p) {
             it = fmt::format_to(it, "{: >{}}bounds: {}\n", "", active_width,
                                 p.bounds);
           }}(props);
}

PrettyPrinter::PrettyPrinter(std::ostream& os, const PrintOptions& options)
    : os_(os), options_(options) {}

VisitResult PrettyPrinter::visit(std::string_view key, double value,
                                 const F64Properties& props) {
  std::ostream_iterator<char> it(os_);
  FormatKeyValue(it, key, value, props);
  return {};
}

VisitResult PrettyPrinter::visit(std::string_view key, std::int64_t value,
                                 const I64Properties& props) {
  std::ostream_iterator<char> it(os_);
  FormatKeyValue(it, key, value, props);
  return {};
}

VisitResult PrettyPrinter::visit(std::string_view key, bool value,
                                 const Properties& props) {
  std::ostream_iterator<char> it(os_);
  FormatKeyValue(it, key, value, props);
  return {};
}

VisitResult PrettyPrinter::visit(std::string_view key, const std::string& value,
                                 const StrProperties& props) {
  std::ostream_iterator<char> it(os_);
  FormatKeyValue(it, key, value, props);
  return {};
}

VisitResult PrettyPrinter::visit(std::string_view key,
                                 std::span<const double> value,
                                 const F64Properties& props) {
  std::ostream_iterator<char> it(os_);
  FormatKeyValue(it, key, value, props);
  return {};
}

VisitResult PrettyPrinter::visit(std::string_view key,
                                 std::span<const std::int64_t> value,
                                 const I64Properties& props) {
  std::ostream_iterator<char> it(os_);
  FormatKeyValue(it, key, value, props);
  return {};
}

VisitResult PrettyPrinter::visit(
    std::string_view key, const std::shared_ptr<const ConfigBase>& config,
    const Properties& props) {
  // Use const_cast to modify indentation state while keeping the Visitor
  // interface const
  std::ostream_iterator<char> it(os_);

  // Nested objects don't print metadata on the opening brace line
  it = fmt::format_to(it, "{: >{}}{}:\n", "",
                      options_.indent_width * indent_level_, key);

  indent_level_++;

  std::ignore = config->accept(*this);
  indent_level_--;

  // Add a newline after the closing brace of the object
  return {};
}

}  // namespace autopilot
