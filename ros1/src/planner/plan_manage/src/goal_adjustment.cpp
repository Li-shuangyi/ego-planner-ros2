#include <plan_manage/goal_adjustment.h>
#include <plan_manage/planner_manager.h>

namespace ego_planner
{
bool EGOPlannerManager::planAdjustedGoal(AStar &search, const Eigen::Vector3d &start,
                                       const Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration, Eigen::Vector3d &goal,
                                       double clearance)
{
  const auto &map = grid_map_;
  const double resolution = map->getResolution();
  Eigen::Vector3d lower, upper;
  const auto is_free = [&](const Eigen::Vector3d &p)
  {
    return map->isInLocalMap(p) && !map->getInflateOccupancy(p);
  };
  if (!start.allFinite() || !velocity.allFinite() || !acceleration.allFinite() || !goal.allFinite() ||
      !is_free(start) || !map->getLocalMapBounds(lower, upper))
    return false;
  const ros::WallTime begin = ros::WallTime::now();
  // Search the robot's connected component once. A free goal can still be
  // behind a wall; repeatedly testing that component would exhaust the budget.
  const int width = int(std::ceil((upper.x() - lower.x()) / resolution));
  const int height = int(std::ceil((upper.y() - lower.y()) / resolution));
  if (width <= 0 || height <= 0 || int64_t(width) * height > 2000000)
    return false;
  const auto index = [&](const Eigen::Vector3d &p) {
    const int x = int(std::floor((p.x() - lower.x()) / resolution));
    const int y = int(std::floor((p.y() - lower.y()) / resolution));
    return x >= 0 && y >= 0 && x < width && y < height ? y * width + x : -1;
  };
  const auto position = [&](int x, int y) {
    return Eigen::Vector3d(lower.x() + (x + 0.5) * resolution,
                           lower.y() + (y + 0.5) * resolution, goal.z());
  };
  const int first = index(start);
  if (first < 0 || !is_free(position(first % width, first / width)))
    return false;
  std::vector<uint8_t> reached(width * height, 0);
  std::vector<int> queue{first};
  reached[first] = 1;
  for (size_t i = 0; i < queue.size(); ++i)
  {
    if ((i % 256) == 0 && (ros::WallTime::now() - begin).toSec() > 0.2)
      return false;
    const int x = queue[i] % width, y = queue[i] / width;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
      {
        const int nx = x + dx, ny = y + dy;
        if ((!dx && !dy) || nx < 0 || ny < 0 || nx >= width || ny >= height)
          continue;
        const int next = ny * width + nx;
        if (reached[next] || !is_free(position(nx, ny)) ||
            (dx && dy && (!is_free(position(nx, y)) || !is_free(position(x, ny)))))
          continue;
        reached[next] = 1;
        queue.push_back(next);
      }
  }
  const double initial_distance = (start - goal).head<2>().norm();
  const auto candidates = goalCandidatesXY(start, goal, lower, upper, resolution,
      [&](const Eigen::Vector3d &p) {
        const int cell = index(p);
        return cell >= 0 && reached[cell] &&
               (p - goal).head<2>().norm() + resolution <= initial_distance &&
               hasGoalClearanceXY(p, resolution, clearance, is_free);
      });

  const auto saved_global = traj_.global_traj;
  const auto saved_local = traj_.local_traj;
  const int saved_failures = continous_failures_count_;
  const auto restore = [&]()
  {
    traj_.global_traj = saved_global;
    traj_.local_traj = saved_local;
    continous_failures_count_ = saved_failures;
  };
  int attempted = 0;
  for (const auto &candidate : candidates)
  {
    // Bound callback work, not correction distance. Keep the old task on timeout.
    if ((ros::WallTime::now() - begin).toSec() > 0.2)
    {
      ROS_WARN("[EGO_GOAL_TIMEOUT] Candidate search budget reached after %d attempts", attempted);
      break;
    }
    if ((candidate - start).norm() < 1e-3)
      continue;
    ++attempted;
    if (search.AstarSearch(resolution, start, candidate) != ASTAR_RET::SUCCESS)
      continue;
    // Test an actual stopping trajectory to this candidate, not just a prefix
    // within the usual planning horizon. No optimization formula is changed.
    bool valid = reboundReplan(start, velocity, acceleration, candidate,
                               Eigen::Vector3d::Zero(), true, false, true);
    const auto &local = traj_.local_traj;
    valid = valid && local.traj_id != saved_local.traj_id &&
            local.traj.getPieceNum() > 0 && std::isfinite(local.duration) && local.duration > 0.0;
    if (valid)
    {
      const double speed = local.traj.getMaxVelRate();
      valid = std::isfinite(speed);
      if (valid)
      {
        const double dt = std::min(0.02, resolution / (2.0 * std::max(speed, 0.01)));
        for (double t = 0.0; valid && t < local.duration; t += dt)
          valid = is_free(local.traj.getPos(t));
        valid = valid && is_free(local.traj.getPos(local.duration)) &&
                (local.traj.getPos(local.duration) - candidate).norm() < 1e-3;
      }
    }
    if (valid)
    {
      // Generating the global reference resets local metadata; keep the exact
      // local trajectory that was just verified for the eventual publication.
      const auto validated_local = traj_.local_traj;
      if (planGlobalTrajWaypoints(start, velocity, acceleration, {candidate},
                                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
      {
        traj_.local_traj = validated_local;
        ROS_WARN_STREAM("[EGO_GOAL_ADJUST] requested=" << goal.transpose()
                        << " accepted=" << candidate.transpose()
                        << " distance=" << (goal - candidate).norm()
                        << " clearance=" << clearance << " trajectory_verified=true");
        goal = candidate;
        return true;
      }
    }
    restore();
  }
  restore();
  return false;
}
} // namespace ego_planner
