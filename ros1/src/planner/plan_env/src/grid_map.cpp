#include <plan_env/grid_map.h>
#include <stdexcept>

void GridMap::initMap(ros::NodeHandle &nh)
{
  nh.param("planning/height", height_, 0.0);
  nh.param("planning/half_height", half_height_, 0.05);
  if (!std::isfinite(height_) || !std::isfinite(half_height_) || half_height_ <= 0 ||
      height_ - half_height_ <= -1 || height_ + half_height_ >= 4)
    throw std::invalid_argument("invalid planar planning height");
  listener_.reset(new tf2_ros::TransformListener(tf_));
  applied_ = nh.advertise<std_msgs::Header>("grid_map/applied", 1, true);
  subscriber_ = nh.subscribe("grid_map/shared", 1, &GridMap::receive, this);
}

void GridMap::receive(const nav_msgs::OccupancyGridConstPtr &msg)
{
  const auto &info = msg->info;
  const auto &p = info.origin.position;
  const auto &q = info.origin.orientation;
  const double age = (ros::Time::now() - msg->header.stamp).toSec();
  const uint64_t cells = uint64_t(info.width) * info.height;
  bool valid = msg->header.frame_id == "map" && !msg->header.stamp.isZero() &&
      age >= -0.1 && age <= 5.0 && info.width > 0 && info.height > 0 &&
      cells <= 2000000 && cells == msg->data.size() &&
      std::isfinite(info.resolution) && std::abs(info.resolution - 0.1) < 1e-7 &&
      std::isfinite(p.x) && std::isfinite(p.y) && p.z == 0 &&
      q.x == 0 && q.y == 0 && q.z == 0 && q.w == 1 &&
      p.x >= -50.00001 && p.y >= -50.00001 &&
      p.x + info.width * double(info.resolution) <= 50.00001 &&
      p.y + info.height * double(info.resolution) <= 50.00001 &&
      std::all_of(msg->data.begin(), msg->data.end(), [](int8_t c) { return c >= -1 && c <= 100; });
  try
  {
    const auto transform = tf_.lookupTransform("world", "map", msg->header.stamp);
    const auto &t = transform.transform.translation;
    const auto &r = transform.transform.rotation;
    // The original shared-prior contract is map == world. Never guess a transform.
    valid = valid && std::isfinite(t.x+t.y+t.z+r.x+r.y+r.z+r.w) &&
        std::abs(t.x)+std::abs(t.y)+std::abs(t.z)+std::abs(r.x)+std::abs(r.y)+std::abs(r.z) < 1e-6 &&
        std::abs(std::abs(r.w)-1) < 1e-6;
  }
  catch (const tf2::TransformException &) { valid = false; }
  if (!valid)
  {
    map_.reset();
    applied_.publish(std_msgs::Header());
    ROS_WARN_THROTTLE(2, "Rejecting shared map: invalid geometry, timestamp or map/world transform");
    return;
  }
  if (map_ && msg->header.stamp <= map_->header.stamp)
    return;
  map_ = msg; // Includes free/unknown cells and changed origin: clears old obstacles.
  received_ = ros::SteadyTime::now();
  applied_.publish(msg->header);
}

bool GridMap::fresh() const
{
  return map_ && (ros::SteadyTime::now() - received_).toSec() <= 5.0 &&
      (ros::Time::now() - map_->header.stamp).toSec() >= -0.1 &&
      (ros::Time::now() - map_->header.stamp).toSec() <= 5.0;
}

bool GridMap::isInPlanningBounds(const Eigen::Vector3d &pos) const
{
  return pos.allFinite() && std::abs(pos.x()) < 50 && std::abs(pos.y()) < 50 &&
      std::abs(pos.z() - height_) < half_height_;
}

bool GridMap::isInLocalMap(const Eigen::Vector3d &pos) const
{
  if (!fresh() || !isInPlanningBounds(pos)) return false;
  const auto &info = map_->info;
  const int x = std::floor((pos.x() - info.origin.position.x) / info.resolution);
  const int y = std::floor((pos.y() - info.origin.position.y) / info.resolution);
  return x >= 0 && y >= 0 && x < int(info.width) && y < int(info.height);
}

int GridMap::getInflateOccupancy(const Eigen::Vector3d &pos) const
{
  if (!isInLocalMap(pos)) return 1;
  const auto &info = map_->info;
  const int x = std::floor((pos.x() - info.origin.position.x) / info.resolution);
  const int y = std::floor((pos.y() - info.origin.position.y) / info.resolution);
  // Match the original move_base layer: 100 lethal; -1 and soft costs passable.
  // EGO's existing trajectory-clearance cost remains, without another dilation.
  return map_->data[y * info.width + x] == 100;
}

bool GridMap::getLocalMapBounds(Eigen::Vector3d &lower, Eigen::Vector3d &upper) const
{
  if (!fresh()) return false;
  const auto &info = map_->info;
  lower = Eigen::Vector3d(info.origin.position.x, info.origin.position.y, height_ - half_height_);
  upper = Eigen::Vector3d(lower.x() + info.width * double(info.resolution),
                          lower.y() + info.height * double(info.resolution), height_ + half_height_);
  return true;
}
