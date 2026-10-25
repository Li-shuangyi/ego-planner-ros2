#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <traj_utils/msg/motion_command.hpp>
#include <ego_mpc_2d/command_gate.h>
#include <chrono>
#include <cstring>
#include <regex>

class Navigation : public rclcpp::Node {
  ego_mpc_2d::CommandGate gate_;
  uint64_t sequence_;
  int64_t map_stamp_=0,odom_stamp_=0;
  int trajectory_id_=0;
  double height_=0;
  rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goals_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr cancels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clouds_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  bool timely(int64_t stamp,double limit) const {double age=(now().nanoseconds()-stamp)*1e-9;return stamp>0 && age>=-.05 && age<=limit;}
  template<class T,class F>void sub(const char* name,F fn,rclcpp::QoS qos=rclcpp::QoS(10)){
    subscriptions_.push_back(create_subscription<T>(name,qos,fn));
  }
  void cancel(){if(gate_.active){std_msgs::msg::UInt64 m;m.data=gate_.active;cancels_->publish(m);}gate_.cancel();output_->publish(geometry_msgs::msg::Twist());}
  void map(const nav_msgs::msg::OccupancyGrid& m){
    const uint64_t cells=uint64_t(m.info.width)*m.info.height;
    if(!cells || cells>2000000 || cells!=m.data.size() || m.header.frame_id!="map" ||
       !timely(rclcpp::Time(m.header.stamp).nanoseconds(),.5) || !std::isfinite(m.info.resolution) || m.info.resolution<=0){cancel();return;}
    sensor_msgs::msg::PointCloud2 cloud;cloud.header=m.header;cloud.height=1;cloud.point_step=12;
    cloud.is_dense=true;cloud.is_bigendian=false;
    for(int k=0;k<3;++k){sensor_msgs::msg::PointField f;f.name=std::string(1,"xyz"[k]);f.offset=k*4;f.datatype=f.FLOAT32;f.count=1;cloud.fields.push_back(f);}
    const auto& q=m.info.origin.orientation;double c=1-2*q.z*q.z,s=2*q.w*q.z;
    for(uint32_t y=0;y<m.info.height;++y)for(uint32_t x=0;x<m.info.width;++x)if(m.data[y*m.info.width+x]==100){
      double gx=(x+.5)*m.info.resolution,gy=(y+.5)*m.info.resolution;
      float xyz[3]={float(m.info.origin.position.x+c*gx-s*gy),float(m.info.origin.position.y+s*gx+c*gy),.05f};
      auto size=cloud.data.size();cloud.data.resize(size+12);std::memcpy(cloud.data.data()+size,xyz,12);++cloud.width;
    }
    cloud.row_step=cloud.width*12;maps_->publish(m);clouds_->publish(cloud);
  }
public:
  Navigation():Node("ego_mpc_navigation"),sequence_(std::chrono::system_clock::now().time_since_epoch().count()){
    gate_.enabled=declare_parameter("enable_motion",false);
    gate_.max_speed=declare_parameter("max_speed",.4);gate_.max_yaw_rate=declare_parameter("max_yaw_rate",.8);
    if(!std::isfinite(gate_.max_speed)||gate_.max_speed<=0||!std::isfinite(gate_.max_yaw_rate)||gate_.max_yaw_rate<=0)throw std::invalid_argument("Invalid output limits");
    goals_=create_publisher<quadrotor_msgs::msg::GoalSet>("/navigation/goal",10);
    cancels_=create_publisher<std_msgs::msg::UInt64>("/navigation/cancel",10);
    output_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    maps_=create_publisher<nav_msgs::msg::OccupancyGrid>("/map2d/planning",rclcpp::QoS(1).transient_local());
    clouds_=create_publisher<sensor_msgs::msg::PointCloud2>("/map2d/planning_cloud",1);
    sub<nav_msgs::msg::OccupancyGrid>("map",[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map(*m);},rclcpp::QoS(1));
    sub<nav_msgs::msg::Odometry>("odom",[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();const auto& p=m->pose.pose.position;const auto& q=m->pose.pose.orientation;
      if(m->header.frame_id!="map"||m->child_frame_id!="base_link"||!timely(stamp,.2)||stamp<=odom_stamp_||
         !std::isfinite(p.x+p.y+p.z+q.x+q.y+q.z+q.w+m->twist.twist.linear.x+m->twist.twist.linear.y+m->twist.twist.linear.z+m->twist.twist.angular.x+m->twist.twist.angular.y+m->twist.twist.angular.z)||std::abs(p.z)>.05||std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3){cancel();return;}
      odom_stamp_=stamp;height_=p.z;gate_.odom_at=steady();
    },rclcpp::SensorDataQoS());
    sub<std_msgs::msg::Empty>("/navigation/heartbeat",[this](std_msgs::msg::Empty::ConstSharedPtr){gate_.heartbeat_at=steady();});
    sub<std_msgs::msg::String>("/navigation/map_applied",[this](std_msgs::msg::String::ConstSharedPtr m){
      static const std::regex receipt(R"receipt(\{"accepted":true,"stamp_ns":([0-9]+),"reason":"applied"\})receipt");std::smatch match;
      try{if(!std::regex_match(m->data,match,receipt)){cancel();return;}auto stamp=std::stoll(match[1]);
        if(stamp<=map_stamp_||!timely(stamp,.5)){cancel();return;}map_stamp_=stamp;gate_.map_at=steady();
      }catch(const std::exception&){cancel();}
    });
    sub<geometry_msgs::msg::PoseStamped>("goal",[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
      auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();const auto& p=m->pose.position;
      if(m->header.frame_id!="map"||!timely(stamp,.5)||!std::isfinite(p.x)||!std::isfinite(p.y)){RCLCPP_WARN(get_logger(),"Rejected goal metadata");return;}
      cancel();quadrotor_msgs::msg::GoalSet goal;goal.header.stamp=now();goal.header.frame_id="map";
      goal.task_id=++sequence_;goal.execute=gate_.enabled;goal.goal={float(p.x),float(p.y),float(height_)};
      if(!gate_.begin(goal.task_id,now().nanoseconds(),steady())){RCLCPP_WARN(get_logger(),"Goal requires fresh map, odometry and planner heartbeat");return;}
      goals_->publish(goal);
    });
    sub<traj_utils::msg::PolyTraj>("/ego/trajectory",[this](traj_utils::msg::PolyTraj::ConstSharedPtr m){
      if(m->task_id!=gate_.active||m->header.frame_id!="map"||!timely(rclcpp::Time(m->header.stamp).nanoseconds(),.5)||m->duration.empty()||m->duration.size()>500)return;
      double total=0;for(double d:m->duration){if(!std::isfinite(d)||d<=0||d>180){cancel();return;}total+=d;}
      if(gate_.trajectory(m->task_id,total-(now()-rclcpp::Time(m->start_time)).seconds(),steady()))trajectory_id_=m->traj_id;
    });
    sub<traj_utils::msg::MotionCommand>("/mpc/command",[this](traj_utils::msg::MotionCommand::ConstSharedPtr m){
      if(m->header.frame_id!="base_link"||!timely(rclcpp::Time(m->header.stamp).nanoseconds(),.2)||m->traj_id!=trajectory_id_||m->map_stamp_ns<=0||m->map_stamp_ns>map_stamp_||!timely(m->map_stamp_ns,.8))return;
      const auto& t=m->twist;gate_.receive(m->task_id,rclcpp::Time(m->header.stamp).nanoseconds(),{t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z},steady());
    });
    sub<std_msgs::msg::Empty>("cancel",[this](std_msgs::msg::Empty::ConstSharedPtr){cancel();});
    sub<std_msgs::msg::Empty>("/emergency_stop",[this](std_msgs::msg::Empty::ConstSharedPtr){gate_.estop=true;cancel();});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{
      if(map_stamp_&&!timely(map_stamp_,.8))cancel();auto old=gate_.active;auto c=gate_.output(steady());
      if(old && !gate_.active){std_msgs::msg::UInt64 m;m.data=old;cancels_->publish(m);}
      geometry_msgs::msg::Twist t;t.linear.x=c[0];t.angular.z=c[5];output_->publish(t);
    });
  }
  ~Navigation(){if(rclcpp::ok())output_->publish(geometry_msgs::msg::Twist());}
};
int main(int argc,char** argv){rclcpp::init(argc,argv);{auto node=std::make_shared<Navigation>();rclcpp::spin(node);}rclcpp::shutdown();}
