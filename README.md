# EGO-MPC 2D

2D trajectory planning and MPC tracking for ground robots, with native **ROS1 Noetic** and **ROS2 Jazzy** implementations. [中文说明](README_zh.md)

The repository contains the planner, controller, task messages, standalone launch files, guarded Twist output, pinned solver sources and runnable regression checks. Localization, map production and robot drivers are external inputs.

## Layout

| Path | Purpose |
|---|---|
| ros1/src | Native catkin packages |
| ros2/src | Native ament packages |
| safety | Shared, ROS-independent output checks |
| tests | Synthetic pipeline verification |
| third_party | Pinned OSQP, OsqpEigen and QDLDL sources with licenses |
| scripts | Repeatable build and isolated test commands |

Both versions are maintained together on main. Use the matching workspace; their task messages are different and must not be mixed.

## Build

Install the selected ROS distribution, Eigen3, CMake **3.22–3.x**, and the distribution's build tools. ROS1 requires catkin; ROS2 requires colcon. On Ubuntu 20.04, the system CMake needs upgrading because the vendored OSQP requires at least 3.18. CI pins CMake 3.28.4 for Noetic.

Resolve ROS dependencies with rosdep for the workspace you selected:

~~~bash
git clone https://github.com/Li-shuangyi/ego-mpc-2d.git
cd ego-mpc-2d
# After configuring rosdep for your ROS installation:
rosdep install --from-paths ros1/src --ignore-src --rosdistro noetic -r -y
bash scripts/build.sh ros1
# Or, in a Jazzy environment:
rosdep install --from-paths ros2/src --ignore-src --rosdistro jazzy -r -y
bash scripts/build.sh ros2
~~~

The scripts select the matching ROS underlay and system interpreter. Solver compilation is offline; no global solver install or nested Git checkout is needed.

## Run

Start your map and localization providers first. Read the [input and safety contract](docs/interfaces.md), particularly map inflation, timestamps and velocity coordinates.

~~~bash
# ROS1, separate terminal
source /opt/ros/noetic/setup.bash
source ros1/devel/setup.bash
roslaunch ego_mpc_bringup navigation.launch

# ROS2, separate terminal
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 launch ego_mpc_bringup navigation.launch.xml
~~~

Default mode plans but publishes zero final velocity. After checking the platform and its own command watchdog, explicitly pass **enable_motion:=true** to the launch command to allow guarded output. Inputs and output can be remapped with map_topic, odom_topic, goal_topic and cmd_topic.

RViz's 2D Goal Pose tool can publish to /goal. Use frame world for ROS1 and map for ROS2. The target's XY position is used; final yaw is not an arrival requirement.

~~~bash
# ROS1 example (fresh timestamp)
rostopic pub -1 /goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: world}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}'
# ROS2 example (fresh timestamp)
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped '{header: {stamp: now, frame_id: map}, pose: {position: {x: 4.0, y: 0.0}, orientation: {w: 1.0}}}'
~~~

The controller outputs forward velocity and yaw rate. Differential-drive or equivalent velocity-controlled platforms fit this interface. Ackermann steering needs a separately validated vehicle model and adapter.

## Verify

~~~bash
bash scripts/test.sh ros1
bash scripts/test.sh ros2
~~~

Tests use a dedicated loopback ROS1 master or localhost ROS2 domain 193, synthetic map/odometry, and /demo/cmd_vel. They do not start sensors, robot SDKs or chassis drivers. See [verification scope](docs/verification.md). GitHub Actions builds and tests both workspaces.

## License and origin

GPL-3.0-only for this combined distribution, with retained third-party notices. This is an independent ground-robot adaptation based on EGO-Planner and existing MPC work. See [THIRD_PARTY.md](THIRD_PARTY.md) for sources and solver revisions.
