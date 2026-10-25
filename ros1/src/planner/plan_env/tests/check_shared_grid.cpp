#include <plan_env/grid_map.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

nav_msgs::OccupancyGrid::Ptr snapshot()
{
  nav_msgs::OccupancyGrid::Ptr msg(new nav_msgs::OccupancyGrid);
  msg->header.frame_id = "map";
  msg->header.stamp = ros::Time::now();
  msg->info.width = msg->info.height = 40;
  msg->info.resolution = 0.1;
  msg->info.origin.position.x = msg->info.origin.position.y = -2;
  msg->info.origin.orientation.w = 1;
  msg->data.assign(1600, -1);
  return msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_shared_grid");
  ros::NodeHandle nh("~");
  GridMap map;
  map.initMap(nh);
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "world";
  tf.transform.rotation.w = 1;
  map.tf_.setTransform(tf, "test", true);
  const Eigen::Vector3d cell(0.05, 0.05, 0);
  require(map.getInflateOccupancy(cell) == 1, "missing map must block");
  auto msg = snapshot();
  msg->data[20*40+20] = 100;
  msg->data[20*40+21] = 99;
  map.receive(msg);
  require(map.getInflateOccupancy(cell) == 1, "lethal cell");
  require(map.getInflateOccupancy(Eigen::Vector3d(0.15, 0.05, 0)) == 0, "soft band must not be dilated twice");
  require(map.getInflateOccupancy(Eigen::Vector3d(-0.05, 0.05, 0)) == 0, "preserve unknown-space policy");
  require(map.getInflateOccupancy(Eigen::Vector3d(0.05, 0.05, 0.1)) == 1, "height bounds");
  require(!map.isInPlanningBounds(Eigen::Vector3d(50, 0, 0)), "world bounds");
  map.receive(snapshot());
  require(map.getInflateOccupancy(cell) == 0, "snapshot clears old occupied cell");
  msg = snapshot();
  msg->info.origin.position.x = 1;
  msg->data[20*40] = 100;
  map.receive(msg);
  require(map.getInflateOccupancy(cell) == 1, "outside moved map is blocked");
  require(map.getInflateOccupancy(Eigen::Vector3d(1.05, 0.05, 0)) == 1, "respect new origin");
  Eigen::Vector3d lower, upper;
  require(map.getLocalMapBounds(lower, upper) && lower.x() == 1 && upper.x() > 4.99, "updated bounds");
  for (int invalid = 0; invalid < 9; ++invalid)
  {
    msg = snapshot();
    switch (invalid)
    {
      case 0: msg->data.pop_back(); break;
      case 1: msg->header.frame_id = "world"; break;
      case 2: msg->info.resolution = 0.2; break;
      case 3: msg->info.origin.position.x = std::numeric_limits<double>::quiet_NaN(); break;
      case 4: msg->info.origin.orientation.w = 0; break;
      case 5: msg->data[0] = 101; break;
      case 6: msg->header.stamp -= ros::Duration(6); break;
      case 7: msg->header.stamp += ros::Duration(1); break;
      case 8: msg->info.origin.position.x = 49; break;
    }
    map.receive(msg);
    require(map.getInflateOccupancy(cell) == 1 && !map.fresh(), "malformed map must invalidate cached map");
  }
  map.tf_.clear();
  tf.transform.translation.x = 1;
  map.tf_.setTransform(tf, "test", true);
  map.receive(snapshot());
  require(!map.fresh(), "non-identity transform must not be guessed");
  map.tf_.clear();
  tf.transform.translation.x = 0;
  map.tf_.setTransform(tf, "test", true);
  msg = snapshot();
  msg->header.stamp -= ros::Duration(4.9);
  map.receive(msg);
  require(map.fresh(), "4.9-second-old map must be accepted");
  msg = snapshot();
  map.receive(msg);
  auto older = snapshot();
  older->header.stamp = msg->header.stamp - ros::Duration(0.1);
  older->data.assign(1600, 100);
  map.receive(older);
  require(map.getInflateOccupancy(cell) == 0, "old snapshots cannot overwrite new ones");
  ros::WallDuration(4.9).sleep();
  require(map.fresh() && map.getInflateOccupancy(cell) == 0, "cached map must remain usable within five seconds");
  ros::WallDuration(0.2).sleep();
  require(!map.fresh() && map.getInflateOccupancy(cell) == 1, "stale map must block");
  std::cout << "PASS shared map: replacement, origin, costs, bounds, validation, transform, ordering, timeout\n";
}
