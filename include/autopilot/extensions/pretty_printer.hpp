#include <ostream>

#include "autopilot/base/config_base.hpp"

namespace autopilot {

struct PrintOptions {
  bool show_details = false;
  int indent_width = 2;
};

class PrettyPrinter final : public ConstConfigVisitor {
 public:
  explicit PrettyPrinter(std::ostream& os, const PrintOptions& options = {});

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
  std::ostream& os_;
  mutable int indent_level_ = 0;
  mutable bool object_open_ = true;

  PrintOptions options_;

  std::ostream_iterator<char> handleNewlineWithComma(
      const std::ostream_iterator<char>& it) const;

  template <typename T, typename P>
  void FormatKeyValue(std::ostream_iterator<char>& it, std::string_view key,
                      const T& value, const P& props) const;
};
}  // namespace autopilot
