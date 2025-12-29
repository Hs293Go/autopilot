#ifndef AUTOPILOT_CORE_BUTTERWORTH_HPP_
#define AUTOPILOT_CORE_BUTTERWORTH_HPP_

#include <array>
#include <cmath>
#include <system_error>

#include "Eigen/Dense"

namespace autopilot {

/** @brief Butterworth filter context structure.
 *
 * This structure holds the state of the Butterworth filter, including
 * coefficients, input and output buffers, and the current index for
 * processing.
 */
template <typename T, std::size_t Dim>
class ButterworthFilter {
 public:
  using Sample = Eigen::Vector<T, Dim>;
  static constexpr int kRingSize = 2;  // For a 2nd order Butterworth filter

  ButterworthFilter() = default;

  /** @brief Initialize the Butterworth filter context.
   *
   * This function initializes the Butterworth filter context with the specified
   * cutoff frequency and sampling frequency.
   *
   * @param cutoff_frequency Cutoff frequency in Hz.
   * @param sampling_frequency Sampling frequency in Hz.
   * @return true if initialization is successful
   * @return false if either cutoff or sampling frequency is non-positive or
   * non-finite, or if the sampling frequency is less than twice the cutoff
   * frequency.
   */
  std::error_code initialize(T cutoff_frequency, T sampling_frequency) {
    using std::isfinite;
    if (!isfinite(cutoff_frequency) || !isfinite(sampling_frequency)) {
      return make_error_code(std::errc::invalid_argument);
    }

    if (cutoff_frequency <= 0 || sampling_frequency <= 0 ||
        sampling_frequency < 2 * cutoff_frequency) {
      return make_error_code(std::errc::argument_out_of_domain);
    }

    cutoff_frequency_ = cutoff_frequency;
    sampling_frequency_ = sampling_frequency;
    reset_numden();

    reset();
    return {};
  }

  /** @brief Set the cutoff frequency of the Butterworth filter.
   *
   * This function sets the cutoff frequency for the Butterworth filter and
   * recalculates the filter coefficients.
   *
   * @param cutoff_frequency New cutoff frequency in Hz.
   * @return true if the cutoff frequency is set successfully
   * @return false if the cutoff frequency is non-positive, non-finite, or twice
   * the cutoff frequency is greater than the sampling frequency.
   */
  std::error_code set_cutoff_frequency(T cutoff_frequency) {
    using std::isfinite;
    if (!isfinite(cutoff_frequency)) {
      return make_error_code(std::errc::invalid_argument);
    }

    if (cutoff_frequency <= 0 || sampling_frequency_ < 2 * cutoff_frequency) {
      return make_error_code(std::errc::argument_out_of_domain);
    }

    cutoff_frequency_ = cutoff_frequency;
    // Recalculate den and num coefficients here
    reset_numden();

    return {};
  }

  /** @brief Set the sampling frequency of the Butterworth filter.
   *
   * This function sets the sampling frequency for the Butterworth filter and
   * recalculates the filter coefficients.
   *
   * @param sampling_frequency New sampling frequency in Hz.
   * @return true if the sampling frequency is set successfully
   * @return false if the sampling frequency is non-positive, non-finite, or
   * less than twice the cutoff frequency.
   */
  std::error_code set_sampling_frequency(T sampling_frequency) {
    using std::isfinite;
    if (!isfinite(sampling_frequency)) {
      return make_error_code(std::errc::invalid_argument);
    }

    if (sampling_frequency <= 0 || sampling_frequency < 2 * cutoff_frequency_) {
      return make_error_code(std::errc::argument_out_of_domain);
    }

    sampling_frequency_ = sampling_frequency;
    // Recalculate den and num coefficients here
    reset_numden();

    return {};
  }

  /** @brief Compute the Butterworth filter output for a given sample.
   *
   * This function computes the output of the Butterworth filter for a given
   * input sample and updates the internal state of the filter.
   *
   * @param sample Pointer to the input sample array of size Dim.
   * @param output Pointer to the output sample array of size Dim.
   * @return true if the computation is successful
   * @return false if the context is not initialized or if the input sample is
   * not valid.
   */
  template <typename Derived>
    requires(bool(Eigen::MatrixBase<Derived>::IsVectorAtCompileTime) &&
             Derived::SizeAtCompileTime == Dim)
  Sample compute(const Eigen::MatrixBase<Derived>& sample) {
    using std::fma;
    using std::isfinite;
    const auto& y_nm2 = prev_output(1);
    const auto& y_nm1 = output_[curr_idx_];

    Sample x_nm2 = prev_input(1);
    curr_idx_++;
    curr_idx_ %= kRingSize;

    const auto& x_nm1 = prev_input(1);

    const Sample result = sample * num_[0] + x_nm1 * num_[1] + x_nm2 * num_[0] +
                          -y_nm1 * den_[0] + -y_nm2 * den_[1];

    input_[curr_idx_] = sample;
    output_[curr_idx_] = result;
    return result;
  }

  /** @brief Reset the Butterworth filter context.
   *
   * This function resets the Butterworth filter context to its initial state,
   * clearing all internal buffers and coefficients.
   *
   * @param ctx Pointer to the Butterworth filter context.
   * @return true if the reset is successful
   * @return false if the context pointer is NULL.
   */
  bool reset() {
    // Reset the current index
    curr_idx_ = 0;

    // Reset input and output buffers
    for (auto& inp : input_) {
      inp.setZero();
    }
    for (auto& outp : output_) {
      outp.setZero();
    }

    return true;
  }

 private:
  void reset_numden() {
    using std::tan;
    using std::numbers::pi_v;
    using std::numbers::sqrt2_v;
    const T k = tan(pi_v<T> * cutoff_frequency_ / sampling_frequency_);
    const T k2 = k * k;
    const T poly = k2 + sqrt2_v<T> * k + T(1);

    const T b0 = k2 / poly;
    const T a1 = T(2) * (k2 - T(1)) / poly;
    const T a2 = (k2 - sqrt2_v<T> * k + T(1)) / poly;

    num_[0] = b0;
    num_[1] = (T(2) * b0);
    den_[0] = a1;
    den_[1] = a2;
  }

  const Sample& prev_input(size_t idx) const {
    return input_[(curr_idx_ + kRingSize - idx) % kRingSize];
  }

  const Sample& prev_output(size_t idx) const {
    return output_[(curr_idx_ + kRingSize - idx) % kRingSize];
  }

  T cutoff_frequency_ = T(40);     // Cutoff frequency in Hz
  T sampling_frequency_ = T(100);  // Sampling frequency in Hz
  std::array<T, kRingSize> den_;   // Denominator coefficients
  std::array<T, kRingSize> num_;   // Numerator coefficients
  // Input buffer for current and previous values
  std::array<Sample, kRingSize> input_{};
  // Output buffer for current and previous values
  std::array<Sample, kRingSize> output_{};

  // Index for current input that evolves circularly so that input/output are
  // like ring buffers
  size_t curr_idx_ = 0;
};

}  // namespace autopilot

#endif  // AUTOPILOT_CORE_BUTTERWORTH_HPP_
