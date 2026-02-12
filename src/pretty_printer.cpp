#include "autopilot/core/pretty_printer.hpp"

#include <fstream>
#include <sstream>

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
    std::visit(autopilot::Overload{
                   [&](auto) { out = fmt::format_to(out, "\"(-inf, "); },
                   [&](const autopilot::Inclusive<T>& inc) {
                     out = fmt::format_to(out, "\"[{}, ", inc.value);
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

template <typename OutputIter>
class PrettyPrinter;

template <class OutIt>
void PrintTo(OutIt out, const ConfigBase& cfg, const PrintOptions& opts) {
  PrettyPrinter<OutIt> printer(out, opts);
  std::ignore = cfg.accept(printer);
}

fmt::appender FormatConfig(fmt::appender out, const ConfigBase& cfg,
                           const PrintOptions& opts) {
  PrintTo(out, cfg, opts);
  return out;
}

char* FormatConfig(char* out, const ConfigBase& cfg, const PrintOptions& opts) {
  PrintTo(out, cfg, opts);
  return out;
}

std::string ToString(const ConfigBase& cfg, const PrintOptions& opts) {
  std::ostringstream oss;
  PrintTo(std::ostream_iterator<char>(oss), cfg, opts);
  return oss.str();
}

std::expected<std::size_t, std::errc> DumpToFile(
    const std::filesystem::path& path, const ConfigBase& cfg,
    const PrintOptions& opts) {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return std::unexpected(std::errc::io_error);
  }
  PrintTo(std::ostream_iterator<char>(ofs), cfg, opts);
  if (ofs.bad()) {
    return std::unexpected(std::errc::io_error);
  }
  return static_cast<std::size_t>(ofs.tellp());
}

template <typename OutputIter>
class PrettyPrinter final : public ConstConfigVisitor {
 public:
  PrettyPrinter(OutputIter it, const PrintOptions& options)
      : it_(it), options_(options) {}

  VisitResult visit(std::string_view key, double value,
                    const F64Properties& props) override;

  VisitResult visit(std::string_view key, std::int64_t value,
                    const I64Properties& props) override;

  VisitResult visit(std::string_view key, bool value,
                    const Properties& props) override;

  VisitResult visit(std::string_view key, const std::string& value,
                    const StrProperties& props) override;

  VisitResult visit(std::string_view key, std::span<const double> value,
                    const F64Properties& props) override;

  VisitResult visit(std::string_view key, std::span<const std::int64_t> value,
                    const I64Properties& props) override;

  VisitResult visit(std::string_view key, const ConfigBase& config,
                    const Properties& props) override;

  // Supports nullable dynamic configs (e.g. inside Polymorphic)
  VisitResult visit(std::string_view key,
                    const std::shared_ptr<const ConfigBase>& config,
                    const Properties& props) override;

 private:
  OutputIter it_;
  mutable int indent_level_ = 0;
  mutable bool object_open_ = true;

  PrintOptions options_;

  OutputIter handleNewlineWithComma(const OutputIter& it) const;

  template <typename T, typename P>
  void FormatKeyValue(OutputIter& it, std::string_view key, const T& value,
                      const P& props) const;
};

template <typename OutputIter>
template <typename T, typename P>
void PrettyPrinter<OutputIter>::FormatKeyValue(OutputIter& it,
                                               std::string_view key,
                                               const T& value,
                                               const P& props) const {
  auto active_width = options_.indent_width * indent_level_;
  if (!options_.show_details) {
    it = fmt::format_to(it, "{: >{}}{}: {}\n", "", active_width, key, value);
    return;
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
           [&it, active_width](const auto& p)
             requires(std::is_same_v<P, F64Properties> ||
                      std::is_same_v<P, I64Properties>)
           {
             it = fmt::format_to(it, "{: >{}}bounds: {}\n", "", active_width,
                                 p.bounds);
           }}(props);
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(std::string_view key, double value,
                                             const F64Properties& props) {
  FormatKeyValue(it_, key, value, props);
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(std::string_view key,
                                             std::int64_t value,
                                             const I64Properties& props) {
  FormatKeyValue(it_, key, value, props);
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(std::string_view key, bool value,
                                             const Properties& props) {
  FormatKeyValue(it_, key, value, props);
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(std::string_view key,
                                             const std::string& value,
                                             const StrProperties& props) {
  FormatKeyValue(it_, key, value, props);
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(std::string_view key,
                                             std::span<const double> value,
                                             const F64Properties& props) {
  FormatKeyValue(it_, key, value, props);
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(
    std::string_view key, std::span<const std::int64_t> value,
    const I64Properties& props) {
  FormatKeyValue(it_, key, value, props);
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(std::string_view key,
                                             const ConfigBase& config,
                                             const Properties& /*props*/) {
  // Nested objects don't print metadata on the opening brace line
  it_ = fmt::format_to(it_, "{: >{}}{}:\n", "",
                       options_.indent_width * indent_level_, key);

  indent_level_++;
  std::ignore = config.accept(*this);
  indent_level_--;

  // Add a newline after the closing brace of the object
  return {};
}

template <typename OutputIter>
VisitResult PrettyPrinter<OutputIter>::visit(
    std::string_view key, const std::shared_ptr<const ConfigBase>& config,
    const Properties& /*props*/) {
  if (config) {
    return visit(key, *config, Properties{});
  }

  it_ = fmt::format_to(it_, "{: >{}}{}: null", "",
                       options_.indent_width * indent_level_, key);
  return {};
}

}  // namespace autopilot
