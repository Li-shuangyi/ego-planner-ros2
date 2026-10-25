#include <plan_env/grid_map.h>
#include <cassert>
#include <iostream>
#include <thread>
#include <cstring>
#include <limits>
sensor_msgs::msg::PointCloud2 thin(const nav_msgs::msg::OccupancyGrid &g) {
  sensor_msgs::msg::PointCloud2 m;m.header=g.header;m.height=1;m.point_step=12;
  for(int k=0;k<3;++k){sensor_msgs::msg::PointField f;f.name=std::string(1,"xyz"[k]);f.offset=k*4;f.count=1;f.datatype=7;m.fields.push_back(f);}
  auto &q=g.info.origin.orientation;const double c=1-2*q.z*q.z,s=2*q.w*q.z;
  for(size_t n=0;n<g.data.size();++n)if(g.data[n]==100 && g.info.width){
    double x=(n%g.info.width+.5)*g.info.resolution,y=(n/g.info.width+.5)*g.info.resolution;
    float p[3]={float(g.info.origin.position.x+c*x-s*y),float(g.info.origin.position.y+s*x+c*y),.05f};
    auto old=m.data.size();m.data.resize(old+12);std::memcpy(m.data.data()+old,p,12);++m.width;
  }
  m.row_step=m.data.size();return m;
}
int main() {
  GridMap map;nav_msgs::msg::OccupancyGrid g;std::string why;
  assert(map.getInflateOccupancy({0,0,0})==1);
  int64_t now=10000000000LL;
  g.header.frame_id="map";g.info.width=4;g.info.height=3;g.info.resolution=.1;g.info.origin.orientation.w=1;g.data.assign(12,0);
  auto apply=[&](){now+=1000000;g.header.stamp=rclcpp::Time(now);return map.applySnapshot(g,thin(g),now,why);};
  assert(apply());Eigen::Vector3d p(.15,.15,0);assert(map.cellValue(p)==0);
  g.data[5]=100;assert(apply());assert(map.getInflateOccupancy(p)==1);
  g.data[5]=0;assert(apply());assert(map.getInflateOccupancy(p)==0);
  g.data[5]=-1;assert(apply());assert(map.getInflateOccupancy(p)==0);
  g.data[5]=55;assert(apply());assert(map.getInflateOccupancy(p)==0 && map.cellValue(p)==55);
  g.info.width=2;g.info.height=2;g.data={100,0,0,0};g.info.origin.position.x=5;assert(apply());assert(!map.isInMap(p));assert(map.getInflateOccupancy(p)==1);assert(map.cellValue({5.05,.05,0})==100);
  g.info.origin.orientation.z=std::sin(M_PI/4);g.info.origin.orientation.w=std::cos(M_PI/4);assert(apply());assert(map.cellValue({4.95,.05,0})==100);
  auto good=g;g.data[0]=101;assert(!apply()&&why=="invalid_cost");assert(map.getInflateOccupancy({4.95,.05,0})==1);
  g=good;assert(apply());auto previous=map.stamp();assert(!map.applySnapshot(g,thin(g),now,why)&&map.stamp()==previous);
  g.header.frame_id="world";assert(!apply()&&why=="frame_mismatch");g=good;
  g.info.origin.orientation.x=.1;assert(!apply());g=good;
  g.info.resolution=0;assert(!apply());g=good;
  g.data.pop_back();assert(!apply());g=good;
  g.header.stamp=rclcpp::Time(now-6000000000LL);assert(!map.applySnapshot(g,thin(g),now,why));
  g.header.stamp=rclcpp::Time(now+1000000000LL);assert(!map.applySnapshot(g,thin(g),now,why));
  g=good;g.data.assign(4,-1);assert(apply());assert(map.getInflateOccupancy({4.95,.05,0})==0);
  g=good;g.info.origin.orientation.z=0;g.info.origin.orientation.w=1;g.data={100,0,0,0};
  now+=1000000;g.header.stamp=rclcpp::Time(now);auto cloud=thin(g);auto valid_cloud=cloud;
  cloud.header.stamp=rclcpp::Time(now-1);assert(!map.applySnapshot(g,cloud,now,why));
  cloud=valid_cloud;cloud.header.frame_id="world";assert(!map.applySnapshot(g,cloud,now,why));
  cloud=valid_cloud;cloud.data.pop_back();assert(!map.applySnapshot(g,cloud,now,why));
  cloud=valid_cloud;float nan=std::numeric_limits<float>::quiet_NaN();std::memcpy(cloud.data.data(),&nan,4);assert(!map.applySnapshot(g,cloud,now,why));
  cloud=valid_cloud;cloud.width=0;cloud.row_step=0;cloud.data.clear();assert(!map.applySnapshot(g,cloud,now,why));
  assert(map.applySnapshot(g,valid_cloud,now,why));
  g.data.assign(4,0);assert(apply());assert(map.getInflateOccupancy({5.05,.05,0})==0);
  // Source age and receipt age must both tolerate gaps up to five seconds.
  GridMap delayed, expired;
  assert(delayed.applySnapshot(g,thin(g),now+5000000000LL,why));
  assert(!expired.applySnapshot(g,thin(g),now+5000000001LL,why));
  assert(why=="stale_or_replayed_stamp");
  std::this_thread::sleep_for(std::chrono::milliseconds(4900));
  assert(!map.getOdomDepthTimeout() && map.getInflateOccupancy({5.05,.05,0})==0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  assert(map.getOdomDepthTimeout() && map.getInflateOccupancy({5.05,.05,0})==1);
  std::cout<<"PASS five-second source-age boundary and cached-map expiry\n";
  std::cout<<"PASS snapshot replace, clear, unknown, soft policy, dimensions, moving origin, rotated origin, invalid costs/frame/origin/size/stamps\n";
}
