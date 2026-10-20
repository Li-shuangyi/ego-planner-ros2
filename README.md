# ego-planner-ros2

Go2 上实际编译、运行的 EGO Planner（ROS 2 / ament / rclcpp）。

## 包含的包

- `planner/plan_manage`（`ego_planner`，节点 `ego_planner_node`）
- `planner/plan_env`
- `planner/path_searching`
- `planner/traj_opt`
- `planner/traj_utils`
- `Utils/quadrotor_msgs`

## 不含

HKUST 原树里的 ROS 1 / catkin 残留，例如 `uav_simulator`、`drone_detect`、`swarm_bridge`、`traj_server.cpp`，以及 Utils 下其它 catkin 包。

## 编译

把本仓库放到 colcon workspace 的 `src` 后：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ego_planner
```
