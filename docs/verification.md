# Verification

Run bash scripts/build.sh ros1 or ros2, then bash scripts/test.sh for that version. Build and test artifacts are ignored. Tests fail with a nonzero exit code.

| Check | Coverage |
|---|---|
| clean native builds | Package manifests, message generation, includes, linking and native launch targets |
| MPC solver | Actual OSQP/OsqpEigen solve and constraints |
| MPC tracking | Real velocity optimizer, yaw wrapping/branch stability, speed limits and inherited regression cases |
| ROS1 trajectory contract | EGO and MPC polynomial position/velocity/acceleration agree at piece boundaries |
| shared grid | Complete replacement, obstacle clearing, unknown/soft policy, bounds, invalid metadata and source timestamps |
| output gate | Preview mode, task identity, stale replay, cancellation, invalid values, limits, estop and monotonic expiry |
| synthetic pipeline | Actual EGO and MPC nodes, generated trajectory, default zero output, simulated unicycle progress, map-loss stop, and no restart on map recovery |

The runtime test publishes only synthetic /demo/map, /demo/odom and /demo/goal, and reads /demo/cmd_vel. It uses ROS1 master port 11459 on loopback or ROS2 localhost domain 193. The ROS1 script refuses an existing master. Do not attach hardware drivers to these test namespaces/domains. No test loads private maps or invokes a robot SDK.

The simulation validates software integration. It does not validate a physical footprint, localization accuracy, drivetrain calibration, braking distance, real-time scheduling or a hardware emergency stop. Validate those on the target platform before allowing motion.

GitHub Actions is configured to repeat the same source build/test commands in ROS Noetic/Focal and Jazzy/Noble containers. Local evidence and the exact tested commit are recorded in the release report. A workflow being present does not mean its remote run has completed.

## Local verification record — 2026-10-25 (Asia/Shanghai)

Verified on an x86_64 Ubuntu 24.04 host with installed Noetic and Jazzy underlays. Both workspaces completed clean native builds, followed by incremental builds for the readiness fix.

- ROS1: four CTest checks passed; the separate live-master grid regression passed; preview and actual EGO/MPC synthetic tracking/map-loss/recovery scenarios passed.
- ROS2: four tests reported zero errors, failures and skips; preview and actual EGO/MPC synthetic tracking/map-loss/recovery scenarios passed.
- Both preview scenarios observed zero motion; control scenarios observed generated trajectories and simulated progress, stopped after map loss, and stayed stopped after map recovery. These scenarios do not assert physical arrival accuracy.
- Source review caught and fixed mixed ROS API/schema copies, stale body-frame fixtures, planner readiness before initial odometry, replacement-task isolation, and the ROS2 replan/map-age budget mismatch.
- Publication scan checked the release file list and package/launch XML, excluded generated ROS Python messages and build outputs, and compared the 556 solver files byte-for-byte with their pinned local import. Public upstream documentation assets and Windows package version numbers in the upstream lockfile were identified separately.

This record covers local verification. Container/remote CI status is available in GitHub Actions after publication. No hardware motion was performed.
