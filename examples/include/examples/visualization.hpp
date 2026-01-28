#ifndef EXAMPLES_VISUALIZATION_HPP_
#define EXAMPLES_VISUALIZATION_HPP_

#include <deque>

#include "autopilot/core/definitions.hpp"
#include "rerun.hpp"

namespace rr = rerun;
namespace rrc = rerun::components;

namespace autopilot::examples {

struct LineSpec {
  rrc::Color color;
  float linewidth;
};

class Plotter {
 public:
  explicit Plotter(rr::RecordingStream& rec_stream, std::string entity,
                   const std::optional<LineSpec>& line_spec = std::nullopt);

  bool log(double value);

 private:
  rr::RecordingStream* rec_;
  std::string entity_;
};

enum class SuffixStyle { kIndex, kComponents };

struct MultiaxisPlotterConfig {
  std::size_t axis_count;
  std::string prefix;
  SuffixStyle suffix_style;
  std::vector<LineSpec> line_specs;
};

class MultiaxisPlotter {
 public:
  explicit MultiaxisPlotter(rr::RecordingStream& rec_stream,
                            const MultiaxisPlotterConfig& config);

  bool log(const Eigen::Ref<const Eigen::VectorXd>& value);

 private:
  rr::RecordingStream* rec_;
  std::vector<std::string> entities_;
  MultiaxisPlotterConfig config_;
};

class QuadrotorVisualizer {
 public:
  explicit QuadrotorVisualizer(rr::RecordingStream& rec_stream,
                               std::string_view prefix = "",
                               std::size_t max_path_length = 1000);

  bool log(const QuadrotorState& state);
  bool log(const QuadrotorState& state, const QuadrotorCommand& cmd);

 private:
  rr::RecordingStream* rec_;
  std::string quad_odom_;
  std::string quad_path_;
  std::string quad_setpoint_;
  std::deque<rrc::Translation3D> path_;
  std::size_t max_path_length_;
};
}  // namespace autopilot::examples

#endif /* end of include guard: EXAMPLES_VISUALIZATION_HPP_ */
