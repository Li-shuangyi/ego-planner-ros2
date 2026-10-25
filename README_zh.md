# EGO-MPC 2D

面向地面移动机器人的二维轨迹规划与 MPC 跟踪控制，提供原生 **ROS1 Noetic** 与 **ROS2 Jazzy** 工程。[English](README.md)

包含规划器、控制器、自定义消息、独立启动入口、速度门控、固定版本的求解器源码与可运行回归测试。地图、定位和底盘驱动由外部提供；无需相机、模型服务或特定机器人 SDK。

## 目录与构建

ROS1 位于 ros1/src，ROS2 位于 ros2/src；两套版本共同维护在 main，不要混用它们的自定义消息。third_party 保存 OSQP、OsqpEigen、QDLDL 源码和许可证。

准备对应 ROS 环境、Eigen3、CMake 3.22–3.x，以及 catkin / colcon。Noetic 的 Ubuntu 20.04 系统 CMake 需要升级；CI 使用 3.28.4。使用 rosdep 安装所选工作区的依赖后：

~~~bash
git clone https://github.com/Li-shuangyi/ego-mpc-2d.git
cd ego-mpc-2d
bash scripts/build.sh ros1
# 或：
bash scripts/build.sh ros2
~~~

求解器随源码离线构建，不需要全局安装，也不需要额外拉取子模块。

## 启动

先启动地图与定位提供者，确认 [接口与安全约定](docs/interfaces.md)。

~~~bash
# ROS1
source /opt/ros/noetic/setup.bash
source ros1/devel/setup.bash
roslaunch ego_mpc_bringup navigation.launch

# ROS2
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 launch ego_mpc_bringup navigation.launch.xml
~~~

默认可以规划，最终速度输出为零。确认实机条件、速度上限和底盘自身的指令超时保护后，在启动参数中显式增加 enable_motion:=true。

通过 map_topic、odom_topic、goal_topic、cmd_topic 参数指定输入输出话题。RViz 的 2D Goal Pose 工具应发布到 /goal；ROS1 使用 world，ROS2 使用 map。只使用目标的 XY，最终朝向不作为到达要求。命令行发送目标的完整例子见英文 README。

输出为前向速度和偏航角速度，适用于差速或等效速度接口平台。阿克曼车辆需要另外验证车辆模型与转向适配。

## 测试与维护

~~~bash
bash scripts/test.sh ros1
bash scripts/test.sh ros2
~~~

测试使用独立回环 ROS master / ROS2 本地 domain、合成地图和里程计，控制输出仅连接 /demo/cmd_vel，不启动真实传感器、SDK 或底盘。覆盖求解器、跟踪、消息边界、地图、门控以及合成整链路；准确范围见 [验证说明](docs/verification.md)。GitHub Actions 对两个版本执行构建和测试。

**需要注意的继承语义**：地图必须预先完成足够的机器人轮廓与障碍膨胀；未知栅格 -1 和代价 0..99 在现有规划器中可通行，100 为硬障碍，软代价不参与加权优化。里程计线速度采用本项目继承的世界坐标表达，而非标准 Odometry 常见的机体坐标表达。具体帧、分辨率和时效要求均在接口文档列出。

代码按 GPL-3.0-only 发布，保留第三方版权与许可证；来源和固定依赖版本见 [THIRD_PARTY.md](THIRD_PARTY.md)。
