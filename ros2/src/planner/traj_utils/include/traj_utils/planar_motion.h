#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace traj_utils
{
// Odometry linear velocity is in world coordinates; the IMU angular velocity
// is in body coordinates. Differentiate the projected body X axis to obtain
// yaw rate, including roll/pitch, without differencing noisy yaw samples.
inline double planarYawRate(const Eigen::Quaterniond &orientation,
                            const Eigen::Vector3d &body_angular_velocity)
{
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-6 ||
      !body_angular_velocity.allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
  const Eigen::Vector3d heading = rotation.col(0);
  const Eigen::Vector3d derivative = rotation *
      body_angular_velocity.cross(Eigen::Vector3d::UnitX());
  const double planar_norm = heading.head<2>().squaredNorm();
  if (planar_norm < 1e-6)
    return std::numeric_limits<double>::quiet_NaN();
  return (heading.x() * derivative.y() - heading.y() * derivative.x()) / planar_norm;
}

inline Eigen::Vector3d turningAcceleration(const Eigen::Vector3d &world_velocity,
                                          double yaw_rate)
{
  if (!std::isfinite(yaw_rate))
    return Eigen::Vector3d::Zero();
  return Eigen::Vector3d(-yaw_rate * world_velocity.y(), yaw_rate * world_velocity.x(), 0.0);
}
} // namespace traj_utils
