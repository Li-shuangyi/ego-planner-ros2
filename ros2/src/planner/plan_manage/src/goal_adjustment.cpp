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
  const auto candidates = goalCandidatesXY(start, goal, lower, upper, resolution,
      [&](const Eigen::Vector3d &p) { return hasGoalClearanceXY(p, resolution, clearance, is_free); });

  const auto saved_global = traj_.global_traj;
  const auto saved_local = traj_.local_traj;
  const int saved_failures = continous_failures_count_;
  const auto restore = [&]()
  {
    traj_.global_traj = saved_global;
    traj_.local_traj = saved_local;
    continous_failures_count_ = saved_failures;
  };
  const rclcpp::Time begin = rclcpp::Clock(RCL_STEADY_TIME).now();
  int attempted = 0;
  for (const auto &candidate : candidates)
  {
    // Bound callback work, not correction distance. Keep the old task on timeout.
    if ((rclcpp::Clock(RCL_STEADY_TIME).now() - begin).seconds() > 0.2)
    {
      RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "[EGO_GOAL_TIMEOUT] Candidate search budget reached after %d attempts", attempted);
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
        RCLCPP_WARN_STREAM(rclcpp::get_logger("ego_planner"), "[EGO_GOAL_ADJUST] requested=" << goal.transpose()
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
