#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/tracking_error_trigger.h>
#include <traj_utils/planar_motion.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
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
    traj_utils::DataDisp data_disp_;

    /* parameters */
    uint32_t request_id_ = 0;
    int32_t published_traj_id_ = 0;
    ros::Time request_stamp_;
    bool rotation_active_ = false;
    bool rotation_to_goal_ = false;
    bool rotation_waiting_for_stop_ = false;
    ros::Time rotation_end_;
    ros::Time rotation_settled_since_;
    double rotation_target_yaw_ = 0.0;
    double rotation_speed_ = 0.25;
    ros::Publisher safety_stop_pub_;
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
    bool flag_escape_emergency_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // accepted endpoint
    Eigen::Vector3d requested_goal_{Eigen::Vector3d::Zero()};
    ros::WallTime last_progress_attempt_;
    ros::Publisher accepted_goal_pub_;
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_{Eigen::Vector3d::Zero()};
    double odom_yaw_{std::numeric_limits<double>::quiet_NaN()};
    double odom_yaw_rate_{std::numeric_limits<double>::quiet_NaN()};
    std::vector<Eigen::Vector3d> wps_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Time last_plan_attempt_, odom_stamp_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void setStartStateFromOdometry();
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg);
    void publishRotationTrajectory(const std_msgs::Header &header, double target_yaw);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp, bool force_progress = false);
    bool ensureCollisionFreeGoal();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

  };

} // namespace ego_planner

#endif
