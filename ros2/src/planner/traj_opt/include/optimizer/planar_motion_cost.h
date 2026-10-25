#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace ego_planner
{
// Short-horizon body heading prediction from the measured yaw and yaw rate.
// Penalize sideways and backward motion softly, allowing obstacle avoidance
// to override the preference. World-frame Y motion is not constrained.
inline double headingAlignmentCost(const Eigen::Vector3d &velocity,
                                    double yaw, double yaw_rate, double time, double decay,
                                    Eigen::Vector3d &grad_velocity, double &grad_time)
{
  const double angle = yaw + yaw_rate * time;
  const Eigen::Vector3d forward(std::cos(angle), std::sin(angle), 0.0);
  const Eigen::Vector3d sideways(-forward.y(), forward.x(), 0.0);
  const double vx = velocity.dot(forward), vy = velocity.dot(sideways);
  const double reverse = std::min(0.0, vx);
  const double weight = std::exp(-time / decay);
  const double cost = weight * (vy * vy + reverse * reverse);
  grad_velocity = 2.0 * weight * (vy * sideways + reverse * forward);
  grad_time = -cost / decay + 2.0 * weight * yaw_rate * vy * (reverse - vx);
  return cost;
}

// A regularized (trajectory yaw rate - measured yaw rate) penalty. It tends
// smoothly to zero at rest: translation cannot represent an in-place turn.
// Exponential decay prefers the current turn near the start without locking
// the robot into that direction for the entire trajectory.
inline double yawRateContinuityCost(const Eigen::Vector3d &velocity,
                                    const Eigen::Vector3d &acceleration,
                                    double measured_rate, double time, double decay,
                                    Eigen::Vector3d &grad_velocity,
                                    Eigen::Vector3d &grad_acceleration, double &grad_time)
{
  constexpr double speed_regularization = 0.05; // m/s
  const double speed_squared = velocity.head<2>().squaredNorm();
  const double denominator = speed_squared + speed_regularization * speed_regularization;
  const double residual = (velocity.x() * acceleration.y() - velocity.y() * acceleration.x()
                           - measured_rate * speed_squared) / denominator;
  const double weight = std::exp(-time / decay);
  const double cost = weight * residual * residual;
  const double scale = 2.0 * weight * residual / denominator;
  grad_velocity = scale * Eigen::Vector3d(
      acceleration.y() - 2.0 * (measured_rate + residual) * velocity.x(),
      -acceleration.x() - 2.0 * (measured_rate + residual) * velocity.y(), 0.0);
  grad_acceleration = scale * Eigen::Vector3d(-velocity.y(), velocity.x(), 0.0);
  grad_time = -cost / decay;
  return cost;
}
} // namespace ego_planner
