
#include <plan_manage/ego_replan_fsm.h>
#include <stdexcept>

namespace ego_planner
{
  using std::to_string;

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &nh)
  {
    node_ = nh;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;
    task_id_ = retired_id_ = 0;
    execute_ = preview_sent_ = false;
    requested_goal_.setZero();

    /*  fsm param  */
    nh->get_parameter_or("fsm.flight_type", target_type_, -1);
    nh->get_parameter_or("fsm.thresh_replan_time", replan_thresh_, -1.0);
    if (!std::isfinite(replan_thresh_) || replan_thresh_ <= 0.0)
      throw std::invalid_argument("fsm/thresh_replan_time must be finite and positive");
    double error_threshold, error_confirmation, error_interval;
    nh->get_parameter_or("fsm.tracking_error_threshold", error_threshold, 0.30);
    nh->get_parameter_or("fsm.tracking_error_confirmation", error_confirmation, 0.20);
    nh->get_parameter_or("fsm.tracking_error_replan_interval", error_interval, 1.0);
    tracking_error_trigger_ = TrackingErrorTrigger(error_threshold, error_confirmation, error_interval);
    nh->get_parameter_or("fsm.planning_horizon", planning_horizen_, -1.0);
    nh->get_parameter_or("fsm.emergency_time", emergency_time_, 1.0);
    nh->get_parameter_or("optimization.obstacle_clearance", goal_clearance_, 0.1);
    if (!std::isfinite(goal_clearance_) || goal_clearance_ < 0.0)
      throw std::invalid_argument("optimization/obstacle_clearance must be finite and nonnegative");
    nh->get_parameter_or("fsm.realworld_experiment", flag_realworld_experiment_, false);
    nh->get_parameter_or("fsm.fail_safe", enable_fail_safe_, true);
    nh->get_parameter_or("fsm.ground_height_measurement", enable_ground_height_measurement_, false);
    nh->get_parameter_or("navigation.arrival_tolerance", arrival_tolerance_, 0.75);
    if (!std::isfinite(arrival_tolerance_) || arrival_tolerance_ <= 0.0)
      throw std::invalid_argument("navigation.arrival_tolerance must be finite and positive");

    nh->get_parameter_or("fsm.waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh->get_parameter_or("fsm.waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh->get_parameter_or("fsm.waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh->get_parameter_or("fsm.waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }


    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    goal_astar_.reset(new AStar);
    goal_astar_->initGridMap(planner_manager_->grid_map_,
                            Eigen::Vector3i(100, 100, planner_manager_->grid_map_->isHeightLimited() ? 3 : 100), true);

    have_trigger_ = !flag_realworld_experiment_;
    no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;

    /* callback */
    exec_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.01), std::bind(&EGOReplanFSM::execFSMCallback, this));
    safety_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>("~/odom_world", rclcpp::SensorDataQoS().keep_last(1), std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));
    mandatory_stop_sub_ = nh->create_subscription<std_msgs::msg::Empty>("~/mandatory_stop", 1, std::bind(&EGOReplanFSM::mandatoryStopCallback, this, std::placeholders::_1));

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh->create_publisher<traj_utils::msg::MINCOTraj>("~/planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh->create_subscription<traj_utils::msg::MINCOTraj>("~/planning/broadcast_traj_recv", 100, std::bind(&EGOReplanFSM::RecvBroadcastMINCOTrajCallback, this, std::placeholders::_1));

    poly_traj_pub_ = nh->create_publisher<traj_utils::msg::PolyTraj>("~/planning/trajectory", 10);
    data_disp_pub_ = nh->create_publisher<traj_utils::msg::DataDisp>("~/planning/data_display", 100);
    heartbeat_pub_ = nh->create_publisher<std_msgs::msg::Empty>("~/planning/heartbeat", 10);
    ground_height_pub_ = nh->create_publisher<std_msgs::msg::Float64>("/ground_height_measurement", 10);
    cancel_sub_ = nh->create_subscription<std_msgs::msg::UInt64>("/navigation/cancel", 10,
      [this](std_msgs::msg::UInt64::ConstSharedPtr m) {
        retired_id_ = std::max(retired_id_, m->data);
        if (task_id_ <= retired_id_)
        {
          task_id_ = 0;
          execute_ = preview_sent_ = false;
          have_target_ = have_trigger_ = have_new_target_ = false;
          if (have_odom_)
            callEmergencyStop(odom_pos_);
          changeFSMExecState(WAIT_TARGET, "CANCEL");
        }
      });

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh->create_subscription<quadrotor_msgs::msg::GoalSet>("/goal", 1, std::bind(&EGOReplanFSM::waypointCallback, this, std::placeholders::_1));
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", 1, std::bind(&EGOReplanFSM::triggerCallback, this, std::placeholders::_1));

      RCLCPP_INFO(rclcpp::get_logger("ego_planner"), "Wait for 2 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 2000)
      {
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(0.001 * 1e9)));
      }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage
    std_msgs::msg::Empty heartbeat_msg;
    heartbeat_pub_->publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else if (have_target_)
        {
          RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else if (have_target_)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else if (have_target_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      if (!ensureCollisionFreeGoal() || exec_state_ != EXEC_TRAJ)
        break;
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      const double now = node_->now().seconds();
      const double elapsed = now - info->start_time;
      const double t_cur = std::max(0.0, std::min(info->duration, elapsed));
      const PtsChk_t *chk_ptr = &info->pts_chk;
      bool close_to_current_traj_end = !chk_ptr->empty() && !chk_ptr->back().empty() &&
                                       chk_ptr->back().back().first - t_cur < emergency_time_;
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - odom_pos_).norm() < no_replan_thresh_) // case 2: assign the next waypoint
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if (!execute_)
      {
        have_target_ = false;
        have_trigger_ = false;
        preview_sent_ = true;
        changeFSMExecState(WAIT_TARGET, "PREVIEW");
      }
      else if ((final_goal_ - odom_pos_).norm() < arrival_tolerance_ && touch_the_goal) // actual arrival
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          // prepare for next round
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (!std::isfinite(elapsed) || !std::isfinite(info->duration) ||
               elapsed < 0.0 || elapsed >= info->duration)
      {
        replan_from_odometry_ = true;
        changeFSMExecState(REPLAN_TRAJ, "TRAJ_EXPIRED");
      }
      else if (tracking_error_trigger_.update(
                   (info->traj.getPos(t_cur) - odom_pos_).head<2>().norm(), now))
      {
        replan_from_odometry_ = true;
        RCLCPP_WARN_STREAM(rclcpp::get_logger("ego_planner"), "[EGO_TRACKING_ERROR] xy_error="
                        << (info->traj.getPos(t_cur) - odom_pos_).head<2>().norm());
        changeFSMExecState(REPLAN_TRAJ, "TRACKING_ERROR");
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end))
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "AAAA");

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    exec_timer_->reset();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::setStartStateFromOdometry()
  {
    const rclcpp::Time now = node_->now();
    const double interval = (last_plan_attempt_.nanoseconds() == 0) ? 0.0 : (now - last_plan_attempt_).seconds();
    last_plan_attempt_ = now;
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    // Preserve the measured turn. Tangential acceleration is unknown; only
    // the kinematic turning component is reconstructed in world coordinates.
    start_acc_ = odom_acc_;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ego_planner"), "[EGO_REPLAN_STATE] source=EKF interval=" << interval
                    << " odom_age=" << (now - odom_stamp_).seconds()
                    << " p=" << start_pt_.transpose() << " v=" << start_vel_.transpose()
                    << " yaw_rate=" << odom_yaw_rate_ << " a=" << start_acc_.transpose());
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    // some warnings
    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    const double t_cur = node_->now().seconds() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return; // A collision replan must not override the timeout stop.
    }
    if (exec_state_ == EMERGENCY_STOP)
      return;

    /* ---------- check trajectory ---------- */
    double t_temp = t_cur; // t_temp will be changed in the next function!
    int i_start = info->traj.locatePieceIdx(t_temp);

    if (i_start >= (int)pts_chk.size())
    {
      return;
    }
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;
            if (dist < allowed_dist)
            {
              RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          replan_from_odometry_ = true;
          if (planFromLocalTraj()) // Make a chance
          {
            RCLCPP_INFO(rclcpp::get_logger("ego_planner"), "Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (!have_target_)
              return; // Goal handling already published a stop trajectory.
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::msg::PolyTraj poly_msg;
    traj_utils::msg::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    publishTrajectory(poly_msg);
    broadcast_ploytraj_pub_->publish(MINCO_msg);

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (!have_target_)
      return false;
    planner_manager_->setCurrentPlanarMotion(odom_yaw_, odom_yaw_rate_);
    if (planner_manager_->grid_map_->isInLocalMap(final_goal_) &&
        planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
      return ensureCollisionFreeGoal(); // Publishes the verified replacement trajectory.

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {

      traj_utils::msg::PolyTraj poly_msg;
      traj_utils::msg::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      publishTrajectory(poly_msg);
      broadcast_ploytraj_pub_->publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    if (!have_odom_ || !odom_pos_.allFinite() || !odom_vel_.allFinite())
      return false;
    setStartStateFromOdometry();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        replan_from_odometry_ = false;
        tracking_error_trigger_.reset(node_->now().seconds());
        return true;
      }
      if (!have_target_)
        return false;
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    const double t_cur = node_->now().seconds() - info->start_time;
    const bool reusable_trajectory = info->traj_id > 0 && std::isfinite(t_cur) &&
        std::isfinite(info->duration) && t_cur >= 0.0 && t_cur < info->duration &&
        planner_manager_->traj_.global_traj.last_glb_t_of_lc_tgt >= 0.0;
    // Never extrapolate an expired polynomial, including on failed retries.
    replan_from_odometry_ = replan_from_odometry_ || info->traj_id <= 0 ||
        !std::isfinite(t_cur) || !std::isfinite(info->duration) ||
        t_cur < 0.0 || t_cur >= info->duration;
    if (replan_from_odometry_)
    {
      if (!have_odom_ || !odom_pos_.allFinite() || !odom_vel_.allFinite())
        return false;
      setStartStateFromOdometry();
    }
    else
    {
      start_pt_ = info->traj.getPos(t_cur);
      start_vel_ = info->traj.getVel(t_cur);
      start_acc_ = info->traj.getAcc(t_cur);
    }

    // Keep the previous route as an initial guess, including during recovery.
    // computeInitState rebuilds it with the selected (possibly measured) head.
    // Collision checks and optimization still validate the entire new result.
    bool success = callReboundReplan(!reusable_trajectory, false);

    if (!success)
    {
      if (!have_target_)
        return false;
      if (reusable_trajectory)
        success = callReboundReplan(true, false);
      if (!success)
      {
        if (!have_target_)
          return false;
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
          if (!have_target_)
            return false;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    if (replan_from_odometry_)
      tracking_error_trigger_.reset(node_->now().seconds());
    replan_from_odometry_ = false;
    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(Eigen::Vector3d next_wp)
  {
    // Validate before adapting Z so non-finite input cannot be hidden.
    if (!have_odom_ || !odom_pos_.allFinite() || !odom_vel_.allFinite() || !next_wp.allFinite())
      return false;
    if (planner_manager_->grid_map_->isHeightLimited())
      next_wp.z() = planner_manager_->grid_map_->getPlanningHeight();
    if (!planner_manager_->grid_map_->isInPlanningBounds(next_wp))
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("ego_planner"), "Rejecting goal outside planning bounds: " << next_wp.transpose());
      return false;
    }
    const Eigen::Vector3d requested_goal = next_wp;
    planner_manager_->setCurrentPlanarMotion(odom_yaw_, odom_yaw_rate_);
    const bool known = planner_manager_->grid_map_->isInLocalMap(next_wp);
    const bool adjusted = known && planner_manager_->grid_map_->getInflateOccupancy(next_wp);
    if (!known)
      RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "[EGO_GOAL_DEFER] Goal outside current local map; occupancy check deferred");
    if (adjusted && !planner_manager_->planAdjustedGoal(*goal_astar_, odom_pos_, odom_vel_, odom_acc_, next_wp, goal_clearance_))
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("ego_planner"), "[EGO_GOAL_REJECT] No replacement with a verified trajectory found for: "
                      << requested_goal.transpose() << "; keeping previous task");
      return false;
    }
    // A goal at the robot does not need a global polynomial with zero duration.
    if ((next_wp - odom_pos_).norm() < 1e-3)
    {
      final_goal_ = next_wp;
      have_target_ = have_trigger_ = have_new_target_ = false;
      callEmergencyStop(odom_pos_);
      changeFSMExecState(WAIT_TARGET, "GOAL_AT_ROBOT");
      visualization_->displayGoalPoint(requested_goal, Eigen::Vector4d(1, 0.5, 0, 1), 0.15, 1);
      visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(0, 1, 0, 1), 0.3, 0);
      return true;
    }
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = adjusted || planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, odom_acc_,
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      final_goal_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = !adjusted;

      /*** FSM ***/
      if (adjusted && (have_trigger_ || target_type_ == TARGET_TYPE::MANUAL_TARGET))
      {
        local_target_pt_ = final_goal_;
        local_target_vel_.setZero();
        touch_goal_ = true;
        have_trigger_ = true;
        flag_escape_emergency_ = true;
        setStartStateFromOdometry();
        traj_utils::msg::PolyTraj poly_msg;
        traj_utils::msg::MINCOTraj MINCO_msg;
        polyTraj2ROSMsg(poly_msg, MINCO_msg);
        publishTrajectory(poly_msg);
        broadcast_ploytraj_pub_->publish(MINCO_msg);
        changeFSMExecState(EXEC_TRAJ, "GOAL_ADJUSTED");
        replan_from_odometry_ = false;
        tracking_error_trigger_.reset(node_->now().seconds());
      }
      else if (exec_state_ != WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      visualization_->displayGoalPoint(requested_goal, Eigen::Vector4d(1, 0.5, 0, 1), 0.15, 1);
      visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(0, 1, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "Unable to generate global trajectory!");
    }

    return success;
  }

  bool EGOReplanFSM::ensureCollisionFreeGoal()
  {
    const auto &map = planner_manager_->grid_map_;
    if (!have_target_)
      return false;
    if (!map->isInLocalMap(final_goal_) || !map->getInflateOccupancy(final_goal_))
      return true;
    if (planNextWaypoint(final_goal_))
      return have_target_;

    // This is an already-active goal that became blocked, not a rejected
    // incoming waypoint. Do not keep executing toward it or retry forever.
    RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "[EGO_GOAL_STOP] Active goal became blocked and could not be adjusted");
    have_target_ = have_trigger_ = have_new_target_ = false;
    callEmergencyStop(odom_pos_);
    changeFSMExecState(WAIT_TARGET, "INVALID_GOAL");
    return false;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::msg::GoalSet::ConstSharedPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id)
      return;
    if (msg->header.frame_id != "map")
      return;
    const double age = (node_->now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (age < -0.1 || age > 0.5)
      return;
    if (msg->task_id <= std::max(task_id_, retired_id_))
      return;

    RCLCPP_INFO(rclcpp::get_logger("ego_planner"), "Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    task_id_ = msg->task_id;
    execute_ = msg->execute;
    preview_sent_ = false;
    requested_goal_ = Eigen::Vector3d(msg->goal[0], msg->goal[1], msg->goal[2]);
    Eigen::Vector3d end_wp = requested_goal_;
    if (planNextWaypoint(end_wp))
    {
      have_trigger_ = have_target_;
    }
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(0.001 * 1e9)));
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::msg::Empty &msg)
  {
    mandatory_stop_ = true;
    RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_stamp_ = msg->header.stamp;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_yaw_rate_ = std::numeric_limits<double>::quiet_NaN();
    odom_yaw_ = std::numeric_limits<double>::quiet_NaN();
    odom_acc_.setZero();

    // Only the planner's copy is flattened; mapping and public EKF odometry
    // keep their original 3D pose and velocity.
    if (planner_manager_->grid_map_->isHeightLimited() &&
        odom_pos_.allFinite() && odom_vel_.allFinite())
    {
      odom_pos_.z() = planner_manager_->grid_map_->getPlanningHeight();
      odom_vel_.z() = 0.0;
      const auto &q = msg->pose.pose.orientation;
      const auto &w = msg->twist.twist.angular;
      odom_yaw_rate_ = traj_utils::planarYawRate(
          Eigen::Quaterniond(q.w, q.x, q.y, q.z), Eigen::Vector3d(w.x, w.y, w.z));
      if (std::isfinite(odom_yaw_rate_))
      {
        const Eigen::Vector3d heading = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized() * Eigen::Vector3d::UnitX();
        odom_yaw_ = std::atan2(heading.y(), heading.x());
      }
      odom_acc_ = traj_utils::turningAcceleration(odom_vel_, odom_yaw_rate_);
    }

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::msg::MINCOTraj::ConstSharedPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        rclcpp::Time(msg->start_time).seconds() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    rclcpp::Time t_now = node_->now();
    if (abs((t_now - msg->start_time).seconds()) > 0.25)
    {

      if (abs((t_now - msg->start_time).seconds()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).seconds());
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).seconds());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    if ( rclcpp::Time(msg->start_time).seconds() <= planner_manager_->traj_.swarm_traj[recv_id].start_time ) // This must be called after buffer fill-up
    {
      RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "Old traj received, ignored.");
      return;
    }

    /* Parse and store data */

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd cps_chk = MJO.getInitConstraintPoints(5); // K = 5, such accuracy is sufficient
    bool far_away = true;
    for (int i = 0; i < cps_chk.cols(); ++i)
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) // close to me that can not be ignored
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !have_recv_pre_agent_) // Accept a far traj if no previous agent received
    {
      poly_traj::Trajectory trajectory = MJO.getTraj();
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = rclcpp::Time(msg->start_time).seconds();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      /* Check Collision */
      if (exec_state_ != EMERGENCY_STOP && have_target_ && planner_manager_->checkCollision(recv_id))
      {
        replan_from_odometry_ = true;
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
      }

      /* Check if receive agents have lower drone id */
      if (!have_recv_pre_agent_)
      {
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
        {
          for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
            {
              break;
            }

            have_recv_pre_agent_ = true;
          }
        }
      }
    }
    else
    {
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = -1; // Means this trajectory is invalid
    }
  }

  void EGOReplanFSM::publishTrajectory(traj_utils::msg::PolyTraj &msg)
  {
    msg.header.frame_id = "map";
    msg.header.stamp = node_->now();
    msg.task_id = task_id_;
    msg.execute = execute_;
    msg.requested_goal[0] = requested_goal_(0);
    msg.requested_goal[1] = requested_goal_(1);
    msg.requested_goal[2] = requested_goal_(2);
    msg.effective_goal[0] = final_goal_(0);
    msg.effective_goal[1] = final_goal_(1);
    msg.effective_goal[2] = final_goal_(2);
    msg.map_stamp_ns = planner_manager_->grid_map_->stamp();
    poly_traj_pub_->publish(msg);
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::msg::PolyTraj &poly_msg, traj_utils::msg::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = rclcpp::Time(static_cast<int64_t>((data->start_time) * 1e9), RCL_ROS_TIME);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = rclcpp::Time(static_cast<int64_t>((data->start_time) * 1e9), RCL_ROS_TIME);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    MINCO_msg.des_clearance = planner_manager_->getSwarmClearance();
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    rclcpp::Time t_now = node_->now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.seconds() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::msg::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_->publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
} // namespace ego_planner
