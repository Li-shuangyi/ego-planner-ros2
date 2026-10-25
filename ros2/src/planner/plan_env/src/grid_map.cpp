#include <plan_env/grid_map.h>
#include <algorithm>
#include <limits>
#include <cstring>
void GridMap::initMap(rclcpp::Node::SharedPtr &node) {
  node_=node;
  receipt_=node->create_publisher<std_msgs::msg::String>("/navigation/map_applied",1);
  sub_=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map2d/planning",rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){
      pending_grid_=msg; applyPending();
    });
  cloud_sub_=node->create_subscription<sensor_msgs::msg::PointCloud2>("~/grid_map/cloud",1,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){pending_cloud_=msg;applyPending();});
}
void GridMap::applyPending() {
  // Grid supplies bounds/cost validation; only its matching thin cloud supplies obstacles.
  // A grid alone cannot refresh the execution gate's map health.
  if(!pending_grid_ || !pending_cloud_ || pending_grid_->header.stamp!=pending_cloud_->header.stamp)return;
  std::string why;
  const bool ok=applySnapshot(*pending_grid_,*pending_cloud_,node_->now().nanoseconds(),why);
  pending_grid_.reset();pending_cloud_.reset();
  std_msgs::msg::String receipt;
  receipt.data="{\"accepted\":"+std::string(ok?"true":"false")+",\"stamp_ns\":"+std::to_string(stamp_ns_)+",\"reason\":\""+why+"\"}";
  receipt_->publish(receipt);
  if(!ok)RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,"Rejected planning cloud: %s",why.c_str());
}
bool GridMap::applySnapshot(const nav_msgs::msg::OccupancyGrid &m,const sensor_msgs::msg::PointCloud2 &cloud,int64_t now,std::string &why) {
  const auto &i=m.info; const auto &p=i.origin.position; const auto &q=i.origin.orientation;
  const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();
  auto reject=[&](const char *s){why=s;valid_=false;return false;};
  if(m.header.frame_id!="map") return reject("frame_mismatch");
  if(stamp<=0 || stamp<=stamp_ns_ || stamp>now+50000000LL || now-stamp>5000000000LL) return reject("stale_or_replayed_stamp");
  if(!std::isfinite(i.resolution)||i.resolution<0.01||i.resolution>1.0||!i.width||!i.height||uint64_t(i.width)*i.height>2000000||m.data.size()!=uint64_t(i.width)*i.height) return reject("invalid_geometry");
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::abs(p.z)>1e-6) return reject("invalid_origin");
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(norm)||std::abs(norm-1)>1e-5||std::abs(q.x)>1e-6||std::abs(q.y)>1e-6) return reject("nonplanar_origin");
  if(std::any_of(m.data.begin(),m.data.end(),[](int v){return v < -1 || v>100;})) return reject("invalid_cost");
  if(cloud.header.frame_id!=m.header.frame_id || cloud.header.stamp!=m.header.stamp)return reject("cloud_grid_header_mismatch");
  if(cloud.height!=1 || cloud.width>m.data.size() || cloud.is_bigendian || cloud.point_step!=12 ||
     cloud.row_step!=uint64_t(cloud.width)*12 || cloud.data.size()!=cloud.row_step || cloud.fields.size()!=3)
    return reject("invalid_cloud_layout");
  for(int k=0;k<3;++k) {
    const auto &f=cloud.fields[k];
    if(f.name!=std::string(1,"xyz"[k]) || f.offset!=uint32_t(k*4) || f.datatype!=sensor_msgs::msg::PointField::FLOAT32 || f.count!=1)
      return reject("invalid_cloud_fields");
  }
  auto candidate=m;
  size_t expected=0;
  for(auto &v:candidate.data)if(v==100){v=0;++expected;}
  if(cloud.width!=expected)return reject("cloud_grid_obstacle_mismatch");
  const double c=1-2*q.z*q.z,s=2*q.w*q.z;
  for(uint32_t n=0;n<cloud.width;++n) {
    float xyz[3];std::memcpy(xyz,cloud.data.data()+n*12,12);
    if(!std::isfinite(xyz[0])||!std::isfinite(xyz[1])||!std::isfinite(xyz[2])||std::abs(xyz[2]-.05)>1e-5)
      return reject("invalid_cloud_point");
    const double dx=xyz[0]-p.x,dy=xyz[1]-p.y;
    const double gx=(c*dx+s*dy)/i.resolution,gy=(-s*dx+c*dy)/i.resolution;
    if(gx<0||gy<0||gx>=i.width||gy>=i.height)return reject("cloud_outside_grid");
    const int x=std::floor(gx),y=std::floor(gy);
    if(std::abs(gx-x-.5)>1e-3||std::abs(gy-y-.5)>1e-3)return reject("cloud_not_cell_center");
    const auto index=y*i.width+x;
    if(m.data[index]!=100 || candidate.data[index]==100)return reject("cloud_grid_obstacle_mismatch");
    candidate.data[index]=100;
  }
  // Commit data, dimensions and origin together after ALL validation.
  map_=std::move(candidate); stamp_ns_=stamp; received_=std::chrono::steady_clock::now(); c_=c; s_=s; valid_=true; why="applied"; return true;
}
bool GridMap::index(const Eigen::Vector3d &p,int &x,int &y) const {
  if(getOdomDepthTimeout()||!p.allFinite()||std::abs(p.z())>0.05) return false;
  const double dx=p.x()-map_.info.origin.position.x,dy=p.y()-map_.info.origin.position.y;
  const double gx=(c_*dx+s_*dy)/getResolution(),gy=(-s_*dx+c_*dy)/getResolution();
  if(gx<0||gy<0||gx>=map_.info.width||gy>=map_.info.height) return false;
  x=static_cast<int>(std::floor(gx));y=static_cast<int>(std::floor(gy));return true;
}
int GridMap::cellValue(const Eigen::Vector3d &p) const {int x,y;return index(p,x,y)?map_.data[y*map_.info.width+x]:-1;}
bool GridMap::isInMap(const Eigen::Vector3d &p) const {int x,y;return index(p,x,y);}
bool GridMap::getLocalMapBounds(Eigen::Vector3d &lo,Eigen::Vector3d &hi) const {
  if(getOdomDepthTimeout())return false;
  lo=Eigen::Vector3d(INFINITY,INFINITY,-0.05);hi=Eigen::Vector3d(-INFINITY,-INFINITY,0.05);
  for(double x:{0.0,map_.info.width*getResolution()}) for(double y:{0.0,map_.info.height*getResolution()}) {
    Eigen::Vector3d p(map_.info.origin.position.x+c_*x-s_*y,map_.info.origin.position.y+s_*x+c_*y,0);
    lo.x()=std::min(lo.x(),p.x());lo.y()=std::min(lo.y(),p.y());hi.x()=std::max(hi.x(),p.x());hi.y()=std::max(hi.y(),p.y());
  } return true;
}
bool GridMap::getOdomDepthTimeout() const {return !valid_ || std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>5.0 || (node_ && (node_->now().nanoseconds()-stamp_ns_>5000000000LL || node_->now().nanoseconds()<stamp_ns_));}
