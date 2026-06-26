/**
 * Teleop UXV Joy node utilities.
 *
 * May 10, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <teleop_uxv_joy/teleop_uxv_joy.hpp>

namespace teleop_uxv_joy
{

/**
 * @brief Computes the smoothing factor of a first-order low-pass given a cutoff and dt.
 *
 * @param cutoff Cutoff frequency [Hz].
 * @param dt Sampling period [s].
 * @return Smoothing factor alpha in [0, 1].
 */
static inline float one_euro_alpha(float cutoff, float dt)
{
  // alpha = dt / (dt + tau), with tau = 1 / (2 * pi * cutoff)
  constexpr float two_pi = 6.283185307179586f;
  const float tau = 1.0f / (two_pi * cutoff);
  return dt / (dt + tau);
}

/**
 * @brief First-order exponential low-pass (EMA) step.
 *
 * @param current Latest sample.
 * @param previous Previous filtered value.
 * @param alpha Smoothing factor in [0, 1].
 * @return Updated filtered value.
 */
static inline float low_pass(float current, float previous, float alpha)
{
  return (alpha * current) + ((1.0f - alpha) * previous);
}

float TeleopUXVJoy::filter_axis(float value, float dt, OneEuroState & state)
{
  // Seed the state on the first sample: no derivative nor smoothing yet
  if (!state.initialized) {
    state.x_prev = value;
    state.x_hat = value;
    state.dx_hat = 0.0f;
    state.last_out = value;
    state.initialized = true;
    return value;
  }

  // No time elapsed (duplicate/equal timestamps): hold the last emitted output
  if (dt <= 0.0f) {
    return state.last_out;
  }

  // Low-pass the signal derivative to obtain a robust speed estimate
  const float dx = (value - state.x_prev) / dt;
  state.dx_hat = low_pass(
    dx, state.dx_hat, one_euro_alpha(static_cast<float>(axes_filter_dcutoff_), dt));

  // Adapt the cutoff to the (smoothed) speed: faster motion -> less smoothing
  const float cutoff = static_cast<float>(axes_filter_min_cutoff_) +
                       static_cast<float>(axes_filter_beta_) * std::fabs(state.dx_hat);

  // Low-pass the signal with the adaptive cutoff
  state.x_hat = low_pass(value, state.x_hat, one_euro_alpha(cutoff, dt));
  state.x_prev = value;

  // Settle to exact zero: kill the geometric decay tail near neutral
  if (std::fabs(state.x_hat) < static_cast<float>(axes_filter_settle_threshold_)) {
    state.x_hat = 0.0f;
    state.dx_hat = 0.0f;
    state.last_out = 0.0f;
    return 0.0f;
  }

  // Send-on-delta deadband: hold the last output until the signal moves enough,
  // flattening residual noise without grid-boundary chatter (0 -> exact passthrough)
  if (std::fabs(state.x_hat - state.last_out) >=
      static_cast<float>(axes_filter_output_deadband_)) {
    state.last_out = state.x_hat;
  }
  return state.last_out;
}

std::string TeleopUXVJoy::get_gear_str(const int16_t & gear)
{
  switch (gear) {
    case UXVGear::GEAR_R:
      return "R";
    case UXVGear::GEAR_N:
      return "N";
    case UXVGear::GEAR_D:
      {
        if (gear_automatic_) return "D";
        return std::to_string(gear);
      }
    case UXVGear::GEAR_INVALID:
      return "INVALID";
    default:
      return std::to_string(gear);
  }
}

} // namespace teleop_uxv_joy
