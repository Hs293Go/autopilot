
#include "autopilot/controllers/pid.hpp"

#include <algorithm>
#include <algorithm>  // Required for std::clamp
#include <cmath>
#include <tuple>

#include "autopilot/controllers/pid.hpp"

namespace autopilot {

// Swallow configuration error and leave default config if invalid
PID::PID(const PidConfig& config) { std::ignore = setConfig(config); }

std::error_code PID::setConfig(const PidConfig& config) {
  if (config.sample_time_s <= 0 || config.output_min >= config.output_max) {
    return make_error_code(std::errc::invalid_argument);
  }
  if (config.kp < 0 || config.ki < 0 || config.kd < 0) {
    return make_error_code(std::errc::invalid_argument);
  }

  config_ = config;

  // Clamp internal state if the new limits are tighter
  if (in_auto_) {
    last_output_ =
        std::clamp(last_output_, config_.output_min, config_.output_max);
    output_sum_ =
        std::clamp(output_sum_, config_.output_min, config_.output_max);
  }
  return {};
}

double PID::compute(double input, double setpoint, double current_time_s) {
  if (!in_auto_) {
    return last_output_;
  }

  // 3. Auto-Init on first hit
  if (!is_initialized_) {
    last_input_ = input;
    last_time_ = current_time_s;

    // Ensure internal state starts valid
    output_sum_ =
        std::clamp(output_sum_, config_.output_min, config_.output_max);
    last_output_ =
        std::clamp(last_output_, config_.output_min, config_.output_max);

    is_initialized_ = true;

    // Return the initial state without calculating P, I, or D
    // because we don't have a valid dt yet.
    return last_output_;
  }

  double dt = current_time_s - last_time_;

  if (dt >= config_.sample_time_s) {
    double error = setpoint - input;
    double d_input = (input - last_input_);

    // Integral calculation
    output_sum_ += (config_.ki * error * dt);
    output_sum_ =
        std::clamp(output_sum_, config_.output_min, config_.output_max);

    // Proportional calculation
    double output_val = config_.kp * error;

    // Derivative calculation (on measurement)
    output_val += output_sum_ - (config_.kd * d_input / dt);

    // Final Output Clamp
    output_val = std::clamp(output_val, config_.output_min, config_.output_max);

    last_output_ = output_val;
    last_input_ = input;
    last_time_ = current_time_s;

    return output_val;
  }

  return last_output_;
}

void PID::setMode(Mode mode, double current_input, double current_output) {
  bool new_auto = (mode == Mode::kAutomatic);

  if (new_auto && !in_auto_) {
    initialize(current_input, current_output);
  }

  in_auto_ = new_auto;
}

void PID::initialize(double current_input, double current_output) {
  last_input_ = current_input;
  last_output_ = current_output;

  // When forcibly initializing, we assume the provided output
  // is the integral sum starting point (bumpless transfer)
  output_sum_ =
      std::clamp(current_output, config_.output_min, config_.output_max);

  // We reset initialization flag so the next compute() captures the new time
  // baseline
  is_initialized_ = false;
}

void PID::reset() {
  // Resetting internal memory but keeping configuration
  is_initialized_ = false;
  output_sum_ = 0.0;
  last_output_ = 0.0;
}

}  // namespace autopilot
