#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
mkdir -p .release
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
children=()

stop_process() {
  local child=$1 descendants
  descendants=$(pgrep -P "$child") || descendants=''
  kill -INT "$child" 2>/dev/null || true
  for attempt in {1..100}; do
    if ! kill -0 "$child" 2>/dev/null; then
      wait "$child" 2>/dev/null || true
      return
    fi
    sleep .1
  done
  # ROS1 launch gives its nodes separate process groups. These PIDs were
  # direct children of this test launcher, never discovered by a name match.
  for descendant in $descendants; do kill -TERM "$descendant" 2>/dev/null || true; done
  kill -TERM "$child" 2>/dev/null || true
  sleep .5
  kill -KILL "$child" 2>/dev/null || true
  wait "$child" 2>/dev/null || true
}

cleanup() {
  for ((index=${#children[@]}-1;index>=0;index--)); do
    stop_process "${children[index]}"
  done
}
trap cleanup EXIT

wait_navigation() {
  local version=$1 launch_pid=$2 nodes
  for attempt in {1..60}; do
    if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
    if [[ "$version" == ros1 ]]; then
      nodes=$(timeout 2 rosnode list 2>/dev/null) || nodes=''
    else
      nodes=$(timeout 3 ros2 node list --no-daemon 2>/dev/null) || nodes=''
    fi
    if [[ "$nodes" == *'/ego_mpc_navigation'* && "$nodes" == *'/ego_planner'* && "$nodes" == *'/mpc'* ]]; then return; fi
    sleep .5
  done
  echo "Navigation launch did not become ready; inspect .release/ runtime logs" >&2
  return 1
}

case "${1:-}" in
  ros1)
    source /opt/ros/noetic/setup.bash
    source ros1/devel/setup.bash
    export ROS_MASTER_URI=http://127.0.0.1:11459 ROS_IP=127.0.0.1 ROS_LOG_DIR="$ROOT/.release/ros1-logs"
    unset ROS_HOSTNAME
    if timeout 2 rosparam list >/dev/null 2>&1; then
      echo 'Refusing to reuse an existing test master' >&2
      exit 1
    fi
    roscore -p 11459 >.release/ros1-master.log 2>&1 & children+=("$!")
    ready=false
    for attempt in {1..40}; do
      if timeout 1 rosparam list >/dev/null 2>&1; then ready=true; break; fi
      sleep .25
    done
    if ! "$ready"; then echo 'Test master failed to start' >&2; exit 1; fi
    ctest --test-dir ros1/build --output-on-failure
    timeout 20 ros1/devel/lib/plan_env/check_shared_grid
    for enabled in false true; do
      roslaunch ego_mpc_bringup navigation.launch enable_motion:="$enabled" publish_identity_tf:=true \
        map_topic:=/demo/map odom_topic:=/demo/odom goal_topic:=/demo/goal cmd_topic:=/demo/cmd_vel \
        >".release/ros1-runtime-$enabled.log" 2>&1 & launch_pid=$!; children+=("$launch_pid")
      wait_navigation ros1 "$launch_pid"
      args=(); if [[ "$enabled" == false ]]; then args+=(--preview); fi
      timeout 25 ros1/devel/lib/ego_mpc_bringup/runtime_pipeline "${args[@]}"
      stop_process "$launch_pid"; unset "children[${#children[@]}-1]"
    done
    ;;
  ros2)
    source /opt/ros/jazzy/setup.bash
    source ros2/install/setup.bash
    export ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="$ROOT/.release/ros2-logs"
    unset ROS_LOCALHOST_ONLY
    colcon --log-base ros2/log test --base-paths ros2/src --build-base ros2/build --install-base ros2/install \
      --executor sequential --return-code-on-test-failure --event-handlers console_direct+
    colcon test-result --test-result-base ros2/build --verbose
    for enabled in false true; do
      ros2 launch ego_mpc_bringup navigation.launch.xml enable_motion:="$enabled" \
        map_topic:=/demo/map odom_topic:=/demo/odom goal_topic:=/demo/goal cmd_topic:=/demo/cmd_vel \
        >".release/ros2-runtime-$enabled.log" 2>&1 & launch_pid=$!; children+=("$launch_pid")
      wait_navigation ros2 "$launch_pid"
      args=(); if [[ "$enabled" == false ]]; then args+=(--preview); fi
      timeout 25 ros2/install/ego_mpc_bringup/lib/ego_mpc_bringup/runtime_pipeline "${args[@]}"
      stop_process "$launch_pid"; unset "children[${#children[@]}-1]"
    done
    ;;
  *) echo 'Usage: bash scripts/test.sh ros1|ros2' >&2; exit 2;;
esac
