# MPC solver sources

Imported on 2026-09-26 (Asia/Shanghai), following the upstream repositories listed in [ColAG's README](https://github.com/FAST-FIRE/ColAG#2-how-to-use).
The upstream pair compiled successfully, so ColAG's `.bak` was not needed.

| Directory | Upstream | Pinned revision | License |
|---|---|---|---|
| `osqp` | https://github.com/osqp/osqp | `1572ae068e9ce9ca723cf8223548ade1ff7acc29` | Apache-2.0; bundled AMD notices retained |
| `osqp-eigen` | https://github.com/gbionics/osqp-eigen | `7587e6994dc194cf22511d909bf4cc5d5e0e4eb2` (0.11.2) | BSD-3-Clause |
| `qdldl` | https://github.com/osqp/qdldl | `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` (v0.1.8) | Apache-2.0 |

All sources and license files are retained; only nested Git metadata is omitted. No upstream implementation changes were made.

`src/control/mpc/CMakeLists.txt` adds OSQP and OsqpEigen as subdirectories and supplies the local QDLDL source path to OSQP's FetchContent declaration. FetchContent is disconnected; builds do not download dependencies. The pinned OSQP uses the final v1 API, which is explicitly selected for OsqpEigen when linking these build targets.

The solver libraries are built statically and linked into `mpc_controller_node`. There is no system install step or runtime OSQP shared-library dependency. Generated headers and libraries stay in the workspace's build/devel directories. Standard ROS Noetic, Eigen and compiler dependencies remain necessary.
