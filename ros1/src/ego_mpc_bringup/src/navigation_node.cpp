#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/PlannerCommand.h>
#include <ego_mpc_2d/command_gate.h>
#include <chrono>

class Navigation {
 ros::NodeHandle node_,private_{"~"};
 ego_mpc_2d::CommandGate gate_;
 uint32_t sequence_;
 int64_t map_stamp_=0,odom_stamp_=0;
 double height_=0;
 ros::Publisher goals_,output_;
 std::vector<ros::Subscriber> subscriptions_;
 ros::WallTimer timer_;
 static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 static bool timely(const ros::Time& stamp,double limit){double age=(ros::Time::now()-stamp).toSec();return !stamp.isZero()&&age>=-.05&&age<=limit;}
 void cancel(){if(gate_.active){quadrotor_msgs::GoalSet m;m.header.stamp=ros::Time::now();m.header.frame_id="world";m.request_id=gate_.active;m.cancel=true;goals_.publish(m);}gate_.cancel();output_.publish(geometry_msgs::Twist());}
public:
 Navigation():sequence_(ros::Time::now().sec){
  private_.param("enable_motion",gate_.enabled,false);
  private_.param("max_speed",gate_.max_speed,.4);private_.param("max_yaw_rate",gate_.max_yaw_rate,.8);
  if(!std::isfinite(gate_.max_speed)||gate_.max_speed<=0||!std::isfinite(gate_.max_yaw_rate)||gate_.max_yaw_rate<=0)throw std::invalid_argument("Invalid output limits");
  goals_=node_.advertise<quadrotor_msgs::GoalSet>("/navigation/goal",10);
  output_=node_.advertise<geometry_msgs::Twist>("cmd_vel",1);
  subscriptions_.push_back(node_.subscribe<nav_msgs::Odometry>("odom",1,[this](const nav_msgs::Odometry::ConstPtr& m){
   const auto& p=m->pose.pose.position;const auto& q=m->pose.pose.orientation;auto stamp=m->header.stamp.toNSec();
   if(m->header.frame_id!="world"||m->child_frame_id!="base_link"||!timely(m->header.stamp,.2)||stamp<=uint64_t(odom_stamp_)||
      !std::isfinite(p.x+p.y+p.z+q.x+q.y+q.z+q.w+m->twist.twist.linear.x+m->twist.twist.linear.y+m->twist.twist.linear.z+m->twist.twist.angular.x+m->twist.twist.angular.y+m->twist.twist.angular.z)||std::abs(p.z)>.05||std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3){cancel();return;}
   odom_stamp_=stamp;height_=p.z;gate_.odom_at=steady();
  }));
  subscriptions_.push_back(node_.subscribe<std_msgs::Empty>("/navigation/heartbeat",1,[this](const std_msgs::Empty::ConstPtr&){gate_.heartbeat_at=steady();}));
  subscriptions_.push_back(node_.subscribe<std_msgs::Header>("/navigation/map_applied",1,[this](const std_msgs::Header::ConstPtr& m){
   auto stamp=m->stamp.toNSec();if(m->frame_id!="map"||!timely(m->stamp,.5)||stamp<=uint64_t(map_stamp_)){cancel();return;}
   map_stamp_=stamp;gate_.map_at=steady();
  }));
  subscriptions_.push_back(node_.subscribe<geometry_msgs::PoseStamped>("goal",1,[this](const geometry_msgs::PoseStamped::ConstPtr& m){
   const auto& p=m->pose.position;
   if(m->header.frame_id!="world"||!timely(m->header.stamp,.5)||!std::isfinite(p.x)||!std::isfinite(p.y)){ROS_WARN("Rejected goal metadata");return;}
   cancel();quadrotor_msgs::GoalSet goal;goal.header.stamp=ros::Time::now();goal.header.frame_id="world";
   if(sequence_==UINT32_MAX){ROS_ERROR("Request ID exhausted; restart navigation");return;}
   goal.request_id=++sequence_;goal.goal={float(p.x),float(p.y),float(height_)};
   if(!gate_.begin(goal.request_id,goal.header.stamp.toNSec(),steady())){ROS_WARN("Goal requires fresh map, odometry and planner heartbeat");return;}
   goals_.publish(goal);
  }));
  subscriptions_.push_back(node_.subscribe<traj_utils::PolyTraj>("/ego/trajectory",10,[this](const traj_utils::PolyTraj::ConstPtr& m){
   if(m->request_id!=gate_.active||m->header.frame_id!="world"||!timely(m->header.stamp,.5)||m->duration.empty()||m->duration.size()>500)return;
   double total=0;for(double d:m->duration){if(!std::isfinite(d)||d<=0||d>180){cancel();return;}total+=d;}
   gate_.trajectory(m->request_id,total-(ros::Time::now()-m->start_time).toSec(),steady());
  }));
  subscriptions_.push_back(node_.subscribe<traj_utils::PlannerCommand>("/mpc/command",1,[this](const traj_utils::PlannerCommand::ConstPtr& m){
   if(m->header.frame_id!="base_link"||!timely(m->header.stamp,.2))return;const auto& t=m->twist;
   gate_.receive(m->request_id,m->header.stamp.toNSec(),{t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z},steady());
  }));
  subscriptions_.push_back(node_.subscribe<std_msgs::UInt32>("/navigation/stop",1,[this](const std_msgs::UInt32::ConstPtr& m){if(m->data==gate_.active)cancel();}));
  subscriptions_.push_back(node_.subscribe<std_msgs::Empty>("cancel",1,[this](const std_msgs::Empty::ConstPtr&){cancel();}));
  subscriptions_.push_back(node_.subscribe<std_msgs::Empty>("/emergency_stop",1,[this](const std_msgs::Empty::ConstPtr&){gate_.estop=true;cancel();}));
  timer_=node_.createWallTimer(ros::WallDuration(.02),[this](const ros::WallTimerEvent&){
   if(map_stamp_&&!timely(ros::Time().fromNSec(map_stamp_),.8))cancel();auto old=gate_.active;auto c=gate_.output(steady());
   if(old&&!gate_.active){quadrotor_msgs::GoalSet m;m.header.stamp=ros::Time::now();m.header.frame_id="world";m.request_id=old;m.cancel=true;goals_.publish(m);}
   geometry_msgs::Twist t;t.linear.x=c[0];t.angular.z=c[5];output_.publish(t);
  });
 }
};
int main(int argc,char** argv){ros::init(argc,argv,"ego_mpc_navigation");Navigation node;ros::spin();}
