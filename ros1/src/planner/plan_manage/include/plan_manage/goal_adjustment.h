#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace ego_planner
{
// Distance is measured to voxel faces, not just voxel centers. Only XY is
// tested: the virtual slab's ceiling/floor must not consume this clearance.
template <typename IsFree>
bool hasGoalClearanceXY(const Eigen::Vector3d &point, double resolution,
                        double clearance, IsFree is_free)
{
  if (!point.allFinite() || !std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(clearance) || clearance < 0.0 || !is_free(point))
    return false;
  const Eigen::Vector2i lower = ((point.head<2>().array() - clearance) / resolution).floor().cast<int>();
  const Eigen::Vector2i upper = ((point.head<2>().array() + clearance) / resolution).floor().cast<int>();
  for (int x = lower.x(); x <= upper.x(); ++x)
    for (int y = lower.y(); y <= upper.y(); ++y)
    {
      const Eigen::Vector2d center((x + 0.5) * resolution, (y + 0.5) * resolution);
      const Eigen::Vector2d delta = ((center - point.head<2>()).cwiseAbs().array() - resolution / 2.0).max(0.0);
      if (delta.squaredNorm() <= clearance * clearance + 1e-12 &&
          !is_free(Eigen::Vector3d(center.x(), center.y(), point.z())))
        return false;
    }
  return true;
}

// Search all grid centers in the available local XY bounds; no radius cap.
// Reachability and trajectory generation are deliberately separate filters.
template <typename IsSafe>
std::vector<Eigen::Vector3d> goalCandidatesXY(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const Eigen::Vector3d &lower, const Eigen::Vector3d &upper,
    double resolution, IsSafe is_safe)
{
  std::vector<Eigen::Vector3d> candidates;
  if (!start.allFinite() || !goal.allFinite() || !lower.allFinite() || !upper.allFinite() ||
      !std::isfinite(resolution) || resolution <= 0.0 || (lower.array() >= upper.array()).any())
    return candidates;
  const Eigen::Vector2i lo = (lower.head<2>() / resolution).array().floor().cast<int>();
  const Eigen::Vector2i hi = (upper.head<2>() / resolution).array().floor().cast<int>();
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
    {
      const Eigen::Vector3d p((x + 0.5) * resolution, (y + 0.5) * resolution, goal.z());
      if ((p.array() > lower.array()).all() && (p.array() < upper.array()).all() && is_safe(p))
        candidates.push_back(p);
    }
  std::sort(candidates.begin(), candidates.end(), [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
  {
    const double da = (a - goal).squaredNorm(), db = (b - goal).squaredNorm();
    return da == db ? (a - start).squaredNorm() < (b - start).squaredNorm() : da < db;
  });
  return candidates;
}
} // namespace ego_planner
