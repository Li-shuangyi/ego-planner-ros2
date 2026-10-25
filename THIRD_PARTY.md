# Sources and third-party notices

This repository is an independent ground-robot integration. Original source notices are retained. The combined distribution is GPL-3.0-only; separately licensed dependencies keep their own terms.

| Component | Upstream | License |
|---|---|---|
| EGO planner and trajectory utilities | https://github.com/ZJU-FAST-Lab/EGO-Planner-v2 and https://github.com/ZJU-FAST-Lab/ego-planner-swarm | GPL-3.0 |
| Existing ground planner/MPC integration | https://github.com/FAST-FIRE/ColAG | GPL-3.0 |
| OSQP | https://github.com/osqp/osqp at 1572ae068e9ce9ca723cf8223548ade1ff7acc29 | Apache-2.0; bundled AMD notices retained |
| OsqpEigen | https://github.com/gbionics/osqp-eigen at 7587e6994dc194cf22511d909bf4cc5d5e0e4eb2 | BSD-3-Clause |
| QDLDL | https://github.com/osqp/qdldl at 138fdac58b9cd1c4137ff1b99152c8108a6cff5b | Apache-2.0 |

The planner, interfaces, planar map consumer and controller include downstream ROS1/ROS2 migration, ground-trajectory, task validation and safety changes. This release separates them from robot-specific launchers, sensor drivers, shared-map services and private deployment assets; adds native standalone bringup and final guarded Twist output; and fixes build/export dependencies and source/message consistency. Solver implementations are copied unchanged, including their license files. See third_party/SOURCES.md for the original import record.
