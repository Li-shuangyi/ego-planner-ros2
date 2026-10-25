# Contributing

Keep ROS1 and ROS2 changes together when they affect a shared behavior. Describe any intentional interface differences in docs/interfaces.md. Preserve upstream attribution and solver pins.

Before opening a pull request, run the affected workspace's build and tests. For safety or task changes, also run both workspaces and the shared gate/runtime checks. Keep validation, watchdogs, cancellation and error paths in the review; do not bypass a failing check to make CI green.

Do not commit generated build/install/log trees, environment files, keys, recordings, private maps, robot IP addresses or machine-specific deployment paths. Include a small reproducer or regression for a behavior fix. Hardware results should state the platform, conditions and limitations separately from synthetic test results.
