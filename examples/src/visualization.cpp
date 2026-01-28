#include "examples/visualization.hpp"

#include <fmt/core.h>

namespace autopilot::examples {
static const rrc::Color kRed(255, 0, 0);
static const rrc::Color kGreen(0, 255, 0);
static const rrc::Color kBlue(0, 0, 255);

Plotter::Plotter(rr::RecordingStream& rec_stream, std::string entity,
                 const std::optional<LineSpec>& line_spec)
    : rec_(&rec_stream), entity_(std::move(entity)) {
  if (line_spec.has_value()) {
    auto line = rr::SeriesLines()
                    .with_colors(line_spec->color)
                    .with_widths(line_spec->linewidth);
    rec_->log(entity_, line);
  }
}

bool Plotter::log(double value) {
  if (!rec_) {
    return false;
  }
  rec_->log(entity_, rr::Scalars(value));
  return true;
}

MultiaxisPlotter::MultiaxisPlotter(rr::RecordingStream& rec_stream,
                                   const MultiaxisPlotterConfig& config)
    : rec_(&rec_stream), config_(config) {
  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    if (config_.suffix_style == SuffixStyle::kIndex) {
      entities_.emplace_back(fmt::format("{}/{}", config_.prefix, i + 1));
    } else {
      char axis_char;
      if (i < 3) {
        axis_char = "xyz"[i];
      } else {
        axis_char = static_cast<char>(static_cast<int>('a') + (i - 3) % 26);
      }

      entities_.emplace_back(fmt::format("{}/{}", config_.prefix, axis_char));
    }

    if (i < config_.line_specs.size()) {
      const auto& spec = config_.line_specs[i];
      auto line =
          rr::SeriesLines().with_colors(spec.color).with_widths(spec.linewidth);
      rec_->log(entities_.back(), line);
    }
  }
}

bool MultiaxisPlotter::log(const Eigen::Ref<const Eigen::VectorXd>& value) {
  if (!rec_ || std::cmp_greater(value.size(), config_.axis_count)) {
    return false;
  }

  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    const rr::Scalars scalar(value[static_cast<Eigen::Index>(i)]);
    rec_->log(entities_[i], scalar);
  }
  return true;
}

QuadrotorVisualizer::QuadrotorVisualizer(rr::RecordingStream& rec_stream,
                                         std::string_view prefix,
                                         std::size_t max_path_length)
    : rec_(&rec_stream),
      quad_odom_(fmt::format("{}/drone", prefix)),
      quad_path_(fmt::format("{}/trajectory", prefix)),
      quad_setpoint_(fmt::format("{}/setpoint", prefix)),
      max_path_length_(max_path_length) {
  const auto frame =
      rr::Arrows3D()
          .with_origins({rrc::Vector3D(0.0F, 0.0F, 0.0F)})
          .with_vectors(
              {{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}})
          .with_colors({kRed, kGreen, kBlue});
  rec_->log(quad_odom_, frame);
}

bool QuadrotorVisualizer::log(const QuadrotorState& state) {
  if (!rec_) {
    return false;
  }

  const Eigen::Vector3f p = state.odometry.pose().translation().cast<float>();
  const Eigen::Quaternionf q = state.odometry.pose().rotation().cast<float>();

  rrc::Translation3D positions(p.x(), p.y(), p.z());
  path_.emplace_back(positions);
  if (path_.size() > max_path_length_) {
    path_.pop_front();
  }
  auto tform = rr::Transform3D().with_translation(positions).with_quaternion(
      rr::Quaternion::from_xyzw(q.x(), q.y(), q.z(), q.w()));
  rec_->log(quad_odom_, tform);

  auto path = rr::LineStrips3D()
                  .with_colors({0xFFFF00FF})
                  .with_radii({0.04f})
                  .with_strips(rrc::LineStrip3D(path_));
  rec_->log(quad_path_, path);

  return true;
}

bool QuadrotorVisualizer::log(const QuadrotorState& state,
                              const QuadrotorCommand& cmd) {
  if (!log(state)) {
    return false;
  }
  if (cmd.hasComponent(QuadrotorStateComponent::kPosition)) {
    const Eigen::Vector3f setpoint = cmd.position().cast<float>();
    auto point = rr::Points3D({{setpoint.x(), setpoint.y(), setpoint.z()}})
                     .with_colors({0xFF0000FF})  // Red
                     .with_radii({0.05f});
    rec_->log(quad_setpoint_, point);
  }
  return true;
}

}  // namespace autopilot::examples
