#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_listener.h>

// The shared receiver already owns fusion, peer footprints and hard inflation.
// This map replaces snapshots directly; it never fuses local clouds or depth.
class GridMap
{
public:
  using Ptr = std::shared_ptr<GridMap>;
  void initMap(ros::NodeHandle &nh);
  int getInflateOccupancy(const Eigen::Vector3d &pos) const;
  double getResolution() const { return 0.1; }
  bool isHeightLimited() const { return true; }
  double getPlanningHeight() const { return height_; }
  bool isInPlanningBounds(const Eigen::Vector3d &pos) const;
  bool isInLocalMap(const Eigen::Vector3d &pos) const;
  bool getLocalMapBounds(Eigen::Vector3d &lower, Eigen::Vector3d &upper) const;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool fresh() const;

private:
  void receive(const nav_msgs::OccupancyGridConstPtr &msg);
  nav_msgs::OccupancyGridConstPtr map_;
  ros::SteadyTime received_;
  double height_ = 0.0;
  double half_height_ = 0.05;
  tf2_ros::Buffer tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  ros::Subscriber subscriber_;
  ros::Publisher applied_;
};
