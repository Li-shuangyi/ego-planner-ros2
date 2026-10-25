# Input, task and output contract

## Public topics

| Topic | Type | Contract |
|---|---|---|
| /map | nav_msgs/OccupancyGrid | Complete, already inflated snapshots; fresh increasing source timestamps, normally >= 5 Hz |
| /odom | nav_msgs/Odometry | Finite planar pose, unit quaternion, fresh increasing timestamps, normally >= 10 Hz |
| /goal | geometry_msgs/PoseStamped | Fresh timestamp, world frame on ROS1 / map frame on ROS2, finite XY |
| /cmd_vel | geometry_msgs/Twist | Guarded forward x and angular z, zero other components; default motion disabled |
| /cancel | std_msgs/Empty | Retire the active task; resuming inputs cannot restart it |
| /emergency_stop | std_msgs/Empty | Latch a stop; restart navigation before accepting a new task |

Use the matching ROS1 type spelling or ROS2 /msg/ spelling in CLI commands. No launch file starts a chassis driver. A platform driver must enforce its own command timeout, since a stopped or killed publisher cannot keep publishing zero.

## Coordinates and maps

ROS1 odometry and goals use world; the map uses map, with a verified identity world-to-map transform. If your data already share that numerical frame, you can explicitly set publish_identity_tf:=true. The default leaves transform ownership to your localization stack. Non-identity transforms are rejected. ROS1 maps require 0.1 m resolution, identity origin orientation, zero origin Z and bounds inside [-50,50] m.

ROS2 uses map for odometry, goals and grids, and base_link as the child/body frame. It accepts planar rotated origins and resolutions 0.01–1.0 m. The bringup adapter derives the required thin PointCloud2 from each same-stamped grid; the planner validates both representations before applying the snapshot. This is a representation conversion, not a separate map fusion algorithm.

Both versions expect base_link odometry with planar Z near zero (planning slab approximately +/-0.05 m). **Linear velocity in Odometry.twist is expressed in the world/map frame**, preserving the imported planner/controller convention; transform body-frame velocity before feeding this input. All quantities use metres, seconds and radians.

Maps are limited to two million cells. Full snapshots replace old data and origins. Original policy is preserved: **100 is blocked; -1 and 0..99 are traversable**. Soft costs are not added to the weighted optimizer. Map edges and expired/invalid maps are blocked. Inflate obstacles upstream for the complete footprint and desired margin; the published map integration adds no additional footprint inflation.

Static map publishers with old timestamps do not satisfy this live navigation contract. Refresh the map from a live, healthy provider; do not refresh timestamps on stale sensor data merely to satisfy the watchdog.

## Tasks and motion

The adapter stamps and identifies accepted goals, forwards native GoalSet to the planner, observes native PolyTraj, and checks identified MPC commands before output. ROS1 uses uint32 request_id and PlannerCommand; ROS2 uses uint64 task_id, execute, trajectory ID and map source timestamp in MotionCommand. Preview tasks cannot enable the final output. A launch started without enable_motion cannot be turned into control by publishing another goal.

The default final limits are 0.4 m/s forward speed and 0.8 rad/s yaw rate. Commands with non-finite values, unsupported components or excessive magnitude are rejected. Motion stops when applied-map receipts exceed 0.5 s, odometry/planner heartbeat exceed 0.3 s, commands exceed 0.2 s, or the trajectory expires. Source timestamps are checked in addition to monotonic receipt deadlines. Initial planning gets up to 2 s to deliver its first valid command, with zero output while waiting. Cancellation, failure and estop retire the task; fresh inputs alone cannot revive it.

The planner heartbeat is emitted only after initialization with fresh validated odometry and an applied map; goals cannot use an uninitialized planner state. A replacement task clears the previous planning state before generating its trajectory. The ROS2 bringup replans every 0.2 s so trajectory map stamps stay within the 0.8 s command freshness budget. Core planner/controller safety and their native timeout checks remain in place. The wrapper provides the final independent output boundary. ROS2 defaults to a 0.75 m planner arrival tolerance; inherited ROS1 planning/tracking thresholds differ. Final yaw alignment and centimetre-level arrival are not promised.
