#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
case "${1:-}" in
 ros1)
  source /opt/ros/noetic/setup.bash
  catkin_make -C ros1 -j2 -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
  ;;
 ros2)
  source /opt/ros/jazzy/setup.bash
  CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base ros2/log build --base-paths ros2/src --build-base ros2/build --install-base ros2/install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
  ;;
 *) echo 'Usage: bash scripts/build.sh ros1|ros2' >&2;exit 2;;
esac
