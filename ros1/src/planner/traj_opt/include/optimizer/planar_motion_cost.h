#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace ego_planner
{
// MINCO fixes its initial velocity and acceleration, so project the planar
// head state onto the measured chassis heading before constructing the seed.
// At rest, a small forward speed defines the initial tangent exactly.
inline void forwardPlanarHead(const Eigen::Vector3d &measured_velocity,
                              const Eigen::Vector3d &measured_acceleration,
                              double yaw, double yaw_rate, double minimum_start_acceleration,
                              Eigen::Vector3d &velocity, Eigen::Vector3d &acceleration)
{
  const Eigen::Vector3d forward(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d sideways(-forward.y(), forward.x(), 0.0);
  const double forward_speed = std::max(0.02, measured_velocity.dot(forward));
  velocity = forward_speed * forward;
  velocity.z() = measured_velocity.z();
  const double forward_acceleration = measured_velocity.dot(forward) > 1e-6
      ? measured_acceleration.dot(forward)
      : std::max(minimum_start_acceleration, measured_acceleration.dot(forward));
  acceleration = forward_acceleration * forward + forward_speed * yaw_rate * sideways;
  acceleration.z() = measured_acceleration.z();
}

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

// Soft turning-rate preference over the whole trajectory. The speed factor
// fades the cost at rest, where the tangent cannot define the vehicle yaw.
// A pseudo-Huber loss keeps large violations from dominating obstacle costs.
inline double softYawRateCost(const Eigen::Vector3d &velocity,
                              const Eigen::Vector3d &acceleration,
                              double preferred_max_rate,
                              Eigen::Vector3d &grad_velocity,
                              Eigen::Vector3d &grad_acceleration)
{
  constexpr double speed_regularization_squared = 0.05 * 0.05;
  constexpr double huber_scale = 0.5;
  const double speed_squared = velocity.head<2>().squaredNorm();
  const double denominator = speed_squared + speed_regularization_squared;
  const double cross = velocity.x() * acceleration.y() - velocity.y() * acceleration.x();
  const double rate = cross / denominator;
  const double excess = std::abs(rate) - preferred_max_rate;
  grad_velocity.setZero();
  grad_acceleration.setZero();
  if (excess <= 0.0)
    return 0.0;

  const double normalized = excess / huber_scale;
  const double root = std::sqrt(1.0 + normalized * normalized);
  const double loss = huber_scale * huber_scale * (root - 1.0);
  const double loss_gradient = std::copysign(excess / root, rate);
  const double speed_factor = speed_squared / denominator;
  const Eigen::Vector3d cross_gradient(acceleration.y(), -acceleration.x(), 0.0);
  const Eigen::Vector3d speed_gradient(2.0 * velocity.x(), 2.0 * velocity.y(), 0.0);
  grad_velocity = speed_factor * loss_gradient *
      (cross_gradient - rate * speed_gradient) / denominator +
      loss * speed_regularization_squared * speed_gradient / (denominator * denominator);
  grad_acceleration = speed_factor * loss_gradient *
      Eigen::Vector3d(-velocity.y(), velocity.x(), 0.0) / denominator;
  return speed_factor * loss;
}

// The time derivative of the regularized path yaw rate. Penalizing rapid
// changes over the whole horizon matches the MPC angular acceleration limit.
inline double softYawAccelerationCost(const Eigen::Vector3d &velocity,
                                      const Eigen::Vector3d &acceleration,
                                      const Eigen::Vector3d &jerk,
                                      double preferred_max_acceleration,
                                      Eigen::Vector3d &grad_velocity,
                                      Eigen::Vector3d &grad_acceleration,
                                      Eigen::Vector3d &grad_jerk)
{
  constexpr double speed_regularization_squared = 0.1 * 0.1;
  constexpr double huber_scale = 2.0;
  const double n = velocity.head<2>().squaredNorm();
  const double denominator = n + speed_regularization_squared;
  const double cross = velocity.x() * acceleration.y() - velocity.y() * acceleration.x();
  const double jerk_cross = velocity.x() * jerk.y() - velocity.y() * jerk.x();
  const double speed_acceleration = velocity.x() * acceleration.x() + velocity.y() * acceleration.y();
  const double yaw_acceleration = jerk_cross / denominator -
      2.0 * cross * speed_acceleration / (denominator * denominator);
  const double excess = std::abs(yaw_acceleration) - preferred_max_acceleration;
  grad_velocity.setZero();
  grad_acceleration.setZero();
  grad_jerk.setZero();
  if (excess <= 0.0)
    return 0.0;

  const double normalized = excess / huber_scale;
  const double root = std::sqrt(1.0 + normalized * normalized);
  const double loss = huber_scale * huber_scale * (root - 1.0);
  const double loss_gradient = std::copysign(excess / root, yaw_acceleration);
  const double speed_factor = (n / denominator) * (n / denominator);
  const Eigen::Vector3d dD(2.0 * velocity.x(), 2.0 * velocity.y(), 0.0);
  const Eigen::Vector3d dC_dv(acceleration.y(), -acceleration.x(), 0.0);
  const Eigen::Vector3d dC_da(-velocity.y(), velocity.x(), 0.0);
  const Eigen::Vector3d dQ_dv(jerk.y(), -jerk.x(), 0.0);
  const Eigen::Vector3d dQ_dj(-velocity.y(), velocity.x(), 0.0);
  const double d2 = denominator * denominator, d3 = d2 * denominator;
  const Eigen::Vector3d dYaw_dv = dQ_dv / denominator - jerk_cross * dD / d2 -
      2.0 * (speed_acceleration * dC_dv + cross *
      Eigen::Vector3d(acceleration.x(), acceleration.y(), 0.0)) / d2 +
      4.0 * cross * speed_acceleration * dD / d3;
  const Eigen::Vector3d dYaw_da = -2.0 * (speed_acceleration * dC_da + cross *
      Eigen::Vector3d(velocity.x(), velocity.y(), 0.0)) / d2;
  grad_velocity = speed_factor * loss_gradient * dYaw_dv +
      loss * (2.0 * n * speed_regularization_squared / d3) * dD;
  grad_acceleration = speed_factor * loss_gradient * dYaw_da;
  grad_jerk = speed_factor * loss_gradient * dQ_dj / denominator;
  return speed_factor * loss;
}
} // namespace ego_planner
