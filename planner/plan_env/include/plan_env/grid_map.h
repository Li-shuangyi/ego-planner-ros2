#pragma once
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <chrono>
using std::shared_ptr;
using std::string;
using std::vector;
using std::cout;
using std::endl;
class GridMap {
public:
  using Ptr = std::shared_ptr<GridMap>;
  void initMap(rclcpp::Node::SharedPtr &node);
  bool applySnapshot(const nav_msgs::msg::OccupancyGrid &map,
                     const sensor_msgs::msg::PointCloud2 &cloud,
                     int64_t now_ns, std::string &reason);
  int cellValue(const Eigen::Vector3d &p) const;
  // Match Go1: in-map unknown (-1) and soft 1..99 are traversable.
  // Invalid/stale maps and outside cells remain blocked. No weighted soft cost.
  int getInflateOccupancy(Eigen::Vector3d p) const {
    int x,y;
    return !index(p,x,y) || map_.data[y*map_.info.width+x]==100;
  }
  int getOccupancy(Eigen::Vector3d p) const { return getInflateOccupancy(p); }
  double getResolution() const { return map_.info.resolution > 0 ? map_.info.resolution : 0.1; }
  bool isInMap(const Eigen::Vector3d &p) const;
  bool isInLocalMap(const Eigen::Vector3d &p) const { return isInMap(p); }
  bool isInPlanningBounds(const Eigen::Vector3d &p) const { return isInMap(p); }
  bool isHeightLimited() const { return true; }
  double getPlanningHeight() const { return 0.0; }
  bool getLocalMapBounds(Eigen::Vector3d &lo,Eigen::Vector3d &hi) const;
  bool getOdomDepthTimeout() const;
  int64_t stamp() const { return stamp_ns_; }
private:
  nav_msgs::msg::OccupancyGrid map_;
  bool valid_=false;
  std::chrono::steady_clock::time_point received_;
  int64_t stamp_ns_=0;
  double c_=1,s_=0;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_grid_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  void applyPending();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr receipt_;
  bool index(const Eigen::Vector3d &p,int &x,int &y) const;
};
