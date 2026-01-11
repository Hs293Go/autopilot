#include <expected>
#include <filesystem>

#include "autopilot/base/config_base.hpp"

namespace autopilot {

struct PrintOptions {
  bool show_details = false;
  int indent_width = 2;
};

fmt::appender FormatConfig(fmt::appender out, const ConfigBase& cfg,
                           const PrintOptions& opts = {});

char* FormatConfig(char* out, const ConfigBase& cfg,
                   const PrintOptions& opts = {});

std::string ToString(const ConfigBase& cfg, const PrintOptions& opts = {});

std::expected<std::size_t, std::error_code> DumpToFile(
    const std::filesystem::path& path, const ConfigBase& cfg,
    const PrintOptions& opts = {});
}  // namespace autopilot

namespace fmt {
constexpr std::expected<char const*, std::errc> FromChars(const char* begin,
                                                          const char* end,
                                                          int& value) {
  value = 0;
  for (const auto* it = begin; it != end; ++it) {
    if (*it < '0' || *it > '9') {
      return std::unexpected(std::errc::invalid_argument);
    }
    value = value * 10 + (*it - '0');
  }
  return end;
}

template <std::derived_from<autopilot::ConfigBase> Derived>
struct formatter<Derived> {
  constexpr auto parse(format_parse_context& ctx) {
    const auto* it = ctx.begin();
    const auto* end = ctx.end();

    const auto* needle = std::ranges::find_if_not(
        it, end, [](auto c) { return std::isdigit(c); });
    if (needle != end && needle != it) {
      if (const auto res = FromChars(it, needle, opts_.indent_width)) {
        it = *res;
      } else {
        throw fmt::format_error("Invalid indent width in format specifier");
      }
    }

    // Parse detailed flag (e.g., 'd')
    if (it != end && *it == 'd') {
      opts_.show_details = true;
      ++it;
    }

    // Return iterator past the last parsed character
    return it;
  }

  template <typename FormatContext>
  auto format(const autopilot::ConfigBase& config, FormatContext& ctx) const {
    return autopilot::FormatConfig(ctx.out(), config, opts_);
  }

  autopilot::PrintOptions opts_;
};

}  // namespace fmt

static_assert(fmt::is_formattable<autopilot::ConfigBase>::value,  //
              "autopilot::ConfigBase should be formattable with fmt");
