
#ifndef AUTOPILOT_CONTROLLERS_PID_HPP_
#define AUTOPILOT_CONTROLLERS_PID_HPP_

#include <system_error>

#include "autopilot/base/config_base.hpp"

namespace autopilot {

struct PidConfigData {
  double kp = 1.0;
  double ki = 0.1;
  double kd = 0.0;
  double sample_time_s = 0.1;  // Sample time in seconds
  double output_min = -5.0;
  double output_max = 5.0;
};

struct PidConfig final : public ReflectiveConfigBase<PidConfig>,
                         public PidConfigData {
  PidConfig() = default;

  PidConfig(const PidConfigData& data) : PidConfigData(data) {}

  std::string_view name() const override { return "PidConfig"; }

  std::error_code setTunings(double cand_kp, double cand_ki, double cand_kd) {
    if (cand_kp < 0 || cand_ki < 0 || cand_kd < 0) {
      return make_error_code(std::errc::invalid_argument);
    }

    kp = cand_kp;
    ki = cand_ki * sample_time_s;
    kd = cand_kd / sample_time_s;
    return {};
  }

  std::error_code setSampleTime(double cand_sample_time_secs) {
    if (cand_sample_time_secs <= 0) {
      return make_error_code(std::errc::invalid_argument);
    }
    double ratio = cand_sample_time_secs / sample_time_s;
    ki *= ratio;
    kd /= ratio;
    sample_time_s = cand_sample_time_secs;
    return {};
  }

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("kp", &PidConfig::kp,
               F64Properties{.desc = "Proportional Gain",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe("ki", &PidConfig::ki,
               F64Properties{.desc = "Integral Gain",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe("kd", &PidConfig::kd,
               F64Properties{.desc = "Derivative Gain",
                             .bounds = Bounds<double>::NonNegative()}),
      Describe("sample_time_s", &PidConfig::sample_time_s,
               F64Properties{.desc = "Sample Time (seconds)",
                             .bounds = Bounds<double>::Positive()}),
      Describe("output_min", &PidConfig::output_min,
               F64Properties{.desc = "Minimum Output Limit"}),
      Describe("output_max", &PidConfig::output_max,
               F64Properties{.desc = "Maximum Output Limit"}));
};

enum class Mode { kManual = 0, kAutomatic = 1 };

class PID {
 public:
  PID() = default;

  // Constructor
  // config: The initial tuning and limit configuration.
  // Defaults to Automatic mode. Time is uninitialized until first compute().
  explicit PID(const PidConfig& config);

  // Sets PID to either Manual (0) or Auto (non-0).
  // To ensure a "bumpless transfer" when switching to Automatic,
  // you must provide the current input and output values of the system.
  void setMode(Mode mode, double current_input = 0.0,
               double current_output = 0.0);

  // Performs the PID calculation.
  // On the very first call, this synchronizes the time and input history
  // without generating a new output (returns the initial/previous state).
  double compute(double input, double setpoint, double current_time_s);

  // Applies a new configuration.
  std::error_code setConfig(const PidConfig& config);

  // Resets the internal history (integral term, last input, time).
  // The next compute() call will be treated as a "first hit".
  void reset();

  const PidConfig& config() const { return config_; }

  Mode mode() const { return in_auto_ ? Mode::kAutomatic : Mode::kManual; }

 private:
  void initialize(double current_input, double current_output);

  PidConfig config_;

  double last_time_ = 0.0;
  double output_sum_ = 0.0;
  double last_input_ = 0.0;
  double last_output_ = 0.0;

  bool in_auto_ = true;
  bool is_initialized_ = false;  // Flags if we have established a time baseline
};
}  // namespace autopilot
#endif
