#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <plan_manage/tracking_error_trigger.h>
#include <traj_utils/planar_motion.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/msg/poly_traj.hpp>
#include <traj_utils/msg/minco_traj.hpp>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(rclcpp::Node::SharedPtr &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    double arrival_tolerance_=0.75;
    uint64_t task_id_=0, retired_id_=0;
    bool execute_=false, preview_sent_=false;
    Eigen::Vector3d requested_goal_=Eigen::Vector3d::Zero();
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr cancel_sub_;
    void publishTrajectory(traj_utils::msg::PolyTraj &msg);
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    AStar::Ptr goal_astar_;
    traj_utils::msg::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    TrackingErrorTrigger tracking_error_trigger_;
    bool replan_from_odometry_{false};
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    double goal_clearance_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_{Eigen::Vector3d::Zero()};
    double odom_yaw_{std::numeric_limits<double>::quiet_NaN()};
    double odom_yaw_rate_{std::numeric_limits<double>::quiet_NaN()};
    std::vector<Eigen::Vector3d> wps_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Time last_plan_attempt_{0, 0, RCL_ROS_TIME}, odom_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::SubscriptionBase::SharedPtr waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_pub_; rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_; rclcpp::Publisher<traj_utils::msg::MINCOTraj>::SharedPtr broadcast_ploytraj_pub_; rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_; rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_height_pub_;

    /* state machine functions */
    void execFSMCallback();
    void setStartStateFromOdometry();
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback();
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const quadrotor_msgs::msg::GoalSet::ConstSharedPtr &msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp);
    bool ensureCollisionFreeGoal();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::msg::Empty &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::msg::MINCOTraj::ConstSharedPtr &msg);
    void polyTraj2ROSMsg(traj_utils::msg::PolyTraj &poly_msg, traj_utils::msg::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
  };

} // namespace ego_planner

#endif
