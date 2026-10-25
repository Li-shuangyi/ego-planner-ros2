#include <mpc.h>
#include <stdexcept>
#include <chrono>
#include <limits>

void checkDelayHistory();
void checkDelayedPlant();

void require(bool value, const char *message)
{
  if (!value) throw std::runtime_error(message);
}

void checkYawBranch()
{
  MPC controller;
  controller.T = 100;
  controller.dt = 0.02;
  controller.control_a = false;
  controller.max_speed = 0.5;
  controller.max_omega = 1.0;
  controller.max_cv = 0.02;
  controller.max_comega = 0.04;
  controller.Q = {0, 0, 2.5, 2};
  controller.R = {2, 2};
  controller.Rd = {0.5, 0.3};
  controller.xref = Eigen::MatrixXd::Zero(4, 500);
  controller.dref = controller.output = Eigen::MatrixXd::Zero(2, 500);
  const double rad = M_PI / 180.0;
  auto unwrap = [&](double heading, double reference, double rate)
  {
    controller.now_state.theta = heading * rad;
    controller.now_state.yaw_rate = rate;
    controller.xref.row(3).setConstant(reference * rad);
    controller.smooth_yaw();
    return (controller.xref(3, 0) - controller.now_state.theta) / rad;
  };
  require(std::abs(unwrap(179, -179, 0) - 2) < 1e-9,
          "absolute +/-pi crossing must remain a two-degree error");
  require(std::abs(unwrap(0, 179.5, 0.6) - 179.5) < 1e-9, "initialize left turn");
  for (int i = 0; i < 10; ++i)
    require(std::abs(unwrap(0, i % 2 ? 179.5 : -179.5, i % 2 ? 0.01 : -0.01) -
                         (i % 2 ? 179.5 : 180.5)) < 1e-9,
            "small reference/gyro sign changes must retain the chosen branch");

  traj_utils::PolyTraj::Ptr replacement(new traj_utils::PolyTraj);
  replacement->header.frame_id="world";replacement->header.stamp=ros::Time::now();
  replacement->start_time=replacement->header.stamp;replacement->request_id=1;
  controller.translation_speed_=controller.max_speed;
  replacement->order = 5;
  replacement->traj_id = 42;
  replacement->duration = {1.0};
  replacement->coef_x.assign(6, 0.0);
  replacement->coef_y.assign(6, 0.0);
  replacement->coef_z.assign(6, 0.0);
  controller.polyTrajCallback(replacement);
  require(controller.yaw_turn_direction_ == 1 && unwrap(0, -179.5, -0.1) > 180,
          "trajectory replacement must retain branch memory");
  require(std::abs(unwrap(0, -150, 0.6) + 150) < 1e-9 && controller.yaw_turn_direction_ == 0,
          "outside the ambiguity region the shortest turn must be available");
  require(std::abs(unwrap(0, 179.5, -0.6) + 180.5) < 1e-9 &&
          std::abs(unwrap(0, -179.5, 0) + 179.5) < 1e-9,
          "measured right turn must select the mirrored branch");
  unwrap(0, 0, 0);
  require(std::abs(unwrap(179, 0, 0.6) - 181) < 1e-9 &&
          std::abs(unwrap(-179, 0, 0.6) - 179) < 1e-9,
          "branch must survive the odometry yaw crossing +/-pi too");
  controller.now_state.theta = 0;
  for (int i = 0; i < controller.T; ++i)
  {
    double angle = (179.0 + 0.04 * i) * rad;
    controller.normlize_theta(angle);
    controller.xref(3, i) = angle;
  }
  controller.smooth_yaw();
  for (int i = 0; i < controller.T; ++i)
    require(std::abs(controller.xref(3, i) - (179.0 + 0.04 * i) * rad) < 1e-9,
            "the complete prediction horizon must use the same continuous branch");
  controller.allow_reverse = true;
  controller.xref.row(2).setConstant(0.2);
  require(std::abs(unwrap(0, 179, 0.6) + 1) < 1e-9 &&
          controller.xref(2, 0) < 0 && controller.yaw_turn_direction_ == 0,
          "legacy reverse mode must retain its original behavior");
  controller.allow_reverse = false;
  controller.xref.row(2).setZero();
  auto command = [&](double reference, double rate)
  {
    unwrap(0, reference, rate);
    controller.output.setZero();
    controller.predictMotion();
    controller.solveMPCV();
    require(controller.output.row(1).head(controller.T).allFinite() &&
            controller.output.row(1).head(controller.T).cwiseAbs().maxCoeff() <= 1.00001,
            "branch selection must preserve finite commands and angular speed cap");
    return controller.output(1, 0);
  };
  require(command(179.5, 0.6) > 0.1 && command(-179.5, 0.6) > 0.1,
          "actual MPC must keep turning left across the reference branch cut");
  require(command(-150, 0.6) < -0.1,
          "actual MPC must still turn right when the reference clearly requires it");
  require(command(179.5, -0.6) < -0.1 && command(-179.5, -0.6) < -0.1,
          "actual MPC right-turn behavior must mirror left-turn behavior");
  controller.emergencyStopCallback(std_msgs::Empty());
  require(controller.yaw_turn_direction_ == 0, "emergency stop must clear branch memory");
  std::cout << "PASS: yaw branch crossings, jitter, replans, odometry wrap, horizon, release, reverse mode and actual MPC commands\n";
}


void checkTaskAndRotation()
{
  MPC c;c.T=100;c.dt=.02;c.control_a=false;c.translation_speed_=.4;c.max_omega=.2;
  c.max_cv=.02;c.max_comega=.01;c.Q={0,0,2.5,20};c.R={2,2};c.Rd={.5,.3};
  c.xref=Eigen::MatrixXd::Zero(4,500);c.dref=c.output=Eigen::MatrixXd::Zero(2,500);
  traj_utils::PolyTraj::Ptr m(new traj_utils::PolyTraj);
  m->header.frame_id="world";m->start_time=ros::Time::now();m->request_id=71;
  m->order=5;m->duration={10};m->coef_x.assign(6,0);m->coef_y.assign(6,0);m->coef_z.assign(6,0);
  m->rotation_only=true;m->coef_yaw={0,0,0,0,.1,0};c.polyTrajCallback(m);
  require(c.receive_traj_ && c.request_id_==71 && c.max_speed==0,"accept spin through same MPC");
  const auto ref=c.getPolyTrajState(5);
  require(std::abs(ref(2))<1e-12 && std::abs(ref(3)-.5)<1e-9,"spin polynomial yields zero speed and target heading");
  c.xref.row(3).setConstant(.5);c.predictMotion();c.solveMPCV();
  require(c.output.row(0).head(c.T).cwiseAbs().maxCoeff()<1e-5,"spin cannot translate");
  require(c.output(1,0)>0 && c.output.row(1).head(c.T).cwiseAbs().maxCoeff()<=.20001,"spin follows heading within existing angular cap");
  c.emergencyStopCallback(std_msgs::Empty());c.polyTrajCallback(m);
  require(c.emergency_stop_flag,"cancelled task cannot rearm MPC");
  m->request_id=72;m->rotation_only=false;c.polyTrajCallback(m);
  require(!c.emergency_stop_flag && c.max_speed==.4 && c.request_id_==72,"next valid translation restores original limit");
  m->request_id=73;m->coef_x[0]=std::numeric_limits<float>::quiet_NaN();c.polyTrajCallback(m);
  require(c.request_id_==72,"invalid replacement does not alter accepted task");
  std::cout<<"PASS: task rejection, cancelled replay, same-MPC spin, zero translation and platform limits\n";
}

int main(int argc, char **argv)
{
  ros::Time::init(); // No node/master is needed for the numerical regression.
  try
  {
    MPC controller;
    checkDelayHistory();
    checkDelayedPlant();
    checkYawBranch();
    checkTaskAndRotation();
    // Call the real velocity optimizer directly; do not start publishers/timers.
    controller.control_a = false;
    controller.T = 100;
    controller.dt = 0.02;
    controller.max_speed = 0.5;
    controller.max_omega = 1.0;
    controller.max_cv = 0.02;
    controller.max_comega = 0.02;
    controller.xref = Eigen::MatrixXd::Zero(4, 500);
    controller.dref = controller.output = Eigen::MatrixXd::Zero(2, 500);
    // Isolate the speed cost so a position error cannot hide a zero reference.
    controller.Q[0] = controller.Q[1] = 0;
    auto solve = [&](double reference_speed, int delay)
    {
      controller.delay_num = delay;
      controller.output_buff.assign(delay, Eigen::Vector2d::Zero());
      controller.output.setZero();
      controller.xref.row(2).setConstant(reference_speed);
      controller.predictMotion();
      controller.solveMPCV();
      const auto speeds = controller.output.row(0).segment(delay, controller.T - delay);
      require(speeds.allFinite() && speeds.minCoeff() >= -1e-5 && speeds.maxCoeff() <= 0.50001,
              "velocity solution must respect forward-only and speed limits");
      return controller.output(0, delay);
    };
    const double moving = solve(0.4, 0);
    require(moving > 0.38 && moving < 0.41, "nonzero trajectory speed must drive motion without position error");
    require(solve(1.0, 0) > 0.49, "catch-up speed capped at 0.5 m/s");
    require(std::abs(solve(0.0, 0)) < 1e-5, "zero reference must stop");
    require(std::abs(solve(-0.4, 0)) < 1e-5, "forward-only constraint preserved");
    require(solve(0.4, 2) > 0.38, "delay mode must use trajectory speed too");
    controller.xref.setZero();
    controller.Q[3] = 0;
    auto turn = [&](double measured_rate, int delay, double queued_rate, double yaw_reference)
    {
      controller.delay_num = delay;
      controller.now_state.yaw_rate = measured_rate;
      controller.output_buff.assign(delay, Eigen::Vector2d(0, queued_rate));
      controller.output.setZero();
      controller.xref.row(3).setConstant(yaw_reference);
      controller.predictMotion();
      controller.solveMPCV();
      const auto rates = controller.output.row(1).segment(delay, controller.T - delay);
      require(rates.allFinite() && rates.cwiseAbs().maxCoeff() <= 1.00001, "angular speed cap preserved");
      for (int k = delay + 1; k < controller.T; ++k)
        require(std::abs(controller.output(1, k) - controller.output(1, k - 1)) <= controller.max_comega + 1e-5,
                "existing inter-step angular acceleration limit preserved");
      return controller.output(1, delay);
    };
    const double left = turn(0.6, 0, 0, 0), right = turn(-0.6, 0, 0, 0);
    require(left > 0.01 && right < -0.01 && std::abs(left + right) < 1e-5,
            "first control soft cost must preserve current turn preference symmetrically");
    require(turn(0.6, 2, -0.4, 0) < -0.01, "delay mode must join the last queued command");
    controller.Q[3] = 20;
    require(turn(0.6, 0, 0, -M_PI) < 0.6 - controller.max_comega - 1e-3,
            "new first-step continuity must be soft, not a new hard bound");
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header.frame_id="world";odom->child_frame_id="base_link";odom->header.stamp=ros::Time::now();
    odom->pose.pose.orientation.w = 1;
    odom->twist.twist.angular.z = -0.4;
    controller.rcvOdomCallBack(odom);
    require(std::abs(controller.now_state.yaw_rate + 0.4) < 1e-9, "MPC must consume measured angular velocity");
    controller.control_a = true;
    controller.delay_num = 0;
    controller.now_state.yaw_rate = 0.6;
    controller.Q[3] = 0;
    controller.xref.setZero();
    controller.output.setZero();
    controller.predictMotion();
    controller.solveMPCA();
    require(controller.output(1, 0) > 0.01, "acceleration mode shares the soft angular continuity cost");
    std::cout << "PASS: soft first-command turn continuity, reversible preference, delay and unchanged hard limits\n";
    std::cout << "PASS: actual MPC velocity optimizer, v_ref=0.4 -> v=" << moving
              << ", speed cap 0.5, zero/negative references and delay indexing\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}

void configureDelayTest(MPC& controller, int delay)
{
  controller.control_a = false;
  controller.delay_num = delay;
  controller.T = delay + 100;
  controller.dt = 0.02;
  controller.max_speed = 0.4;
  controller.max_omega = 0.2;
  controller.max_cv = 0.02;
  controller.max_comega = 0.04;
  controller.Q = {5, 5, 2.5, 2};
  controller.R = {2, 2};
  controller.Rd = {0.5, 0.3};
  controller.xref = Eigen::MatrixXd::Zero(4, 500);
  controller.dref = controller.output = controller.last_output = Eigen::MatrixXd::Zero(2, 500);
  controller.output_buff.assign(delay, Eigen::Vector2d::Zero());
}

void checkDelayHistory()
{
  MPC controller;
  configureDelayTest(controller, 50);
  geometry_msgs::Twist applied;
  // Deliberately irregular receipt times, including a change inside a 20 ms bin.
  applied.linear.x = 0.2;
  applied.angular.z = 0.1;
  controller.recordAppliedCommand(applied, ros::Time(10.0));
  applied.linear.x = 0.4;
  applied.angular.z = -0.2;
  controller.recordAppliedCommand(applied, ros::Time(10.315));
  applied = geometry_msgs::Twist(); // actual gate stop, not optimizer's last output
  controller.recordAppliedCommand(applied, ros::Time(10.7));
  controller.sampleDelayCommands(ros::Time(11.0));
  require(std::abs(controller.output.row(0).head(50).sum() * 0.02 - 0.217) < 1e-9,
          "history must integrate exactly one second of actual command time");
  require(std::abs(controller.output(0, 15) - 0.25) < 1e-9,
          "20 ms model bins must account for 100 Hz commands and receipt jitter");
  controller.predictMotion();
  require(std::abs(controller.xbar[50].theta - (0.315 * 0.1 - 0.385 * 0.2)) < 1e-9,
          "first rollout must already contain current applied history");
  controller.sampleDelayCommands(ros::Time(12.0));
  require(controller.output.leftCols(50).norm() < 1e-9,
          "stop/restart cannot replay the old optimizer queue");
  controller.sampleDelayCommands(ros::Time(9.0));
  require(controller.applied_commands_.empty() && controller.output.leftCols(50).norm() == 0,
          "clock rewind must clear future commands");
  for (int i = 0; i <= 200; ++i)
  {
    applied.linear.x = i < 150 ? 0.2 : 0;
    controller.recordAppliedCommand(applied, ros::Time(20.0 + i * 0.01));
  }
  controller.sampleDelayCommands(ros::Time(22.0));
  require(std::abs(controller.output.row(0).head(50).sum() * 0.02 - 0.1) < 1e-8,
          "50 model steps must retain 100 command samples, not only half a second");
  require(controller.applied_commands_.size() <= 102, "history must remain bounded by time");
  applied.linear.x = 0.2;
  controller.recordAppliedCommand(applied, ros::Time(0.1));
  controller.sampleDelayCommands(ros::Time(0.2));
  require(std::abs(controller.output.row(0).head(50).sum() * 0.02 - 0.02) < 1e-9,
          "simulated time below the delay must not underflow ros::Time");
  std::cout << "PASS: timestamp history, jitter, first rollout, 100/50 Hz resampling, stop/restart, rewind\n";
}

void checkDelayedPlant()
{
  double rms[2];
  for (int mode = 0; mode < 2; ++mode)
  {
    MPC controller;
    configureDelayTest(controller, mode ? 50 : 0);
    controller.Q[0] = controller.Q[1] = 0; // isolate heading dynamics
    std::deque<geometry_msgs::Twist> plant_queue(100);
    double squared = 0;
    const auto begin = std::chrono::steady_clock::now();
    for (int tick = 0; tick < 1600; ++tick)
    {
      const double t = tick * 0.01;
      for (int k = 0; k < controller.T; ++k)
        controller.xref(3, k) = 0.25 * std::sin(0.7 * (t + (k + 1) * controller.dt));
      controller.sampleDelayCommands(ros::Time(100 + t));
      for (int iteration = 0; iteration < 2; ++iteration)
      {
        controller.predictMotion();
        controller.solveMPCV();
      }
      geometry_msgs::Twist command;
      command.angular.z = controller.output(1, controller.delay_num);
      require(std::isfinite(command.angular.z) && std::abs(command.angular.z) <= 0.20001,
              "delayed plant command must remain finite and bounded");
      controller.recordAppliedCommand(command, ros::Time(100 + t));
      const auto actual = plant_queue.front();
      plant_queue.pop_front();
      plant_queue.push_back(command);
      if (tick >= 200)
        squared += std::pow(controller.now_state.theta - 0.25 * std::sin(0.7 * t), 2);
      controller.now_state.theta += actual.angular.z * 0.01;
      controller.now_state.yaw_rate = actual.angular.z;
    }
    rms[mode] = std::sqrt(squared / 1400);
    std::cout << "SIM: compensation=" << mode << "s plant_delay=1s yaw_RMSE=" << rms[mode]
              << " rad mean_control_compute_ms="
              << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count() / 1600 << '\n';
  }
  require(rms[1] < rms[0] * 0.8, "one-second prediction should reduce known delayed-plant heading error");
}
