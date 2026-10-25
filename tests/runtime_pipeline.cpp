#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <functional>
#include <stdexcept>
#ifdef EGO_MPC_ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <std_msgs/msg/empty.hpp>
#else
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <traj_utils/PolyTraj.h>
#include <std_msgs/Empty.h>
#endif

struct State {
 double x=0,y=0,yaw=0,v=0,w=0,elapsed=0,last_nonzero=-1;
 unsigned commands=0,trajectories=0;bool preview=false;
 double first_ready=-1,ready_at=-INFINITY,goal_at=-1;
 void ready(){if(first_ready<0)first_ready=elapsed;ready_at=elapsed;}
 template<class T>void command(const T& c){
  if(!std::isfinite(c.linear.x)||!std::isfinite(c.angular.z)||std::abs(c.linear.x)>.4001||std::abs(c.angular.z)>.8001||
     std::abs(c.linear.y)+std::abs(c.linear.z)+std::abs(c.angular.x)+std::abs(c.angular.y)>1e-8)throw std::runtime_error("invalid command");
  v=c.linear.x;w=c.angular.z;++commands;
  if(std::abs(v)+std::abs(w)>1e-5){last_nonzero=elapsed;if(preview)throw std::runtime_error("preview emitted motion");if(goal_at>=0 && elapsed>goal_at+5.8)throw std::runtime_error("stale map or recovered inputs revived motion");}
 }
 template<class M>void fillMap(M& m){
  m.header.frame_id="map";m.info.width=m.info.height=100;m.info.resolution=.1;
  m.info.origin.position.x=m.info.origin.position.y=-5;m.info.origin.orientation.w=1;m.data.assign(10000,0);
  // One inflated obstacle away from the straight test route; snapshot tests cover replacement/bounds.
  for(unsigned y=68;y<74;++y)for(unsigned x=62;x<68;++x)m.data[y*100+x]=100;
 }
 template<class M>void fillOdom(M& m,const char* frame){
  m.header.frame_id=frame;m.child_frame_id="base_link";m.pose.pose.position.x=x;m.pose.pose.position.y=y;
  m.pose.pose.orientation.z=std::sin(yaw/2);m.pose.pose.orientation.w=std::cos(yaw/2);
  // The imported planner/controller contract uses world-frame linear velocity.
  m.twist.twist.linear.x=v*std::cos(yaw);m.twist.twist.linear.y=v*std::sin(yaw);m.twist.twist.angular.z=w;
 }
};
int main(int argc,char** argv){
 State state;for(int k=1;k<argc;++k)if(std::string(argv[k])=="--preview")state.preview=true;
 try {
 std::function<void(bool,bool)> publish;std::function<void()> spin;
#ifdef EGO_MPC_ROS2
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("ego_mpc_runtime_check");
 auto maps=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/demo/map",1);
 auto odom=node->create_publisher<nav_msgs::msg::Odometry>("/demo/odom",10);
 auto goals=node->create_publisher<geometry_msgs::msg::PoseStamped>("/demo/goal",1);
 auto commands=node->create_subscription<geometry_msgs::msg::Twist>("/demo/cmd_vel",10,[&](geometry_msgs::msg::Twist::ConstSharedPtr m){state.command(*m);});
 auto trajectory=node->create_subscription<traj_utils::msg::PolyTraj>("/ego/trajectory",10,[&](traj_utils::msg::PolyTraj::ConstSharedPtr m){if(m->task_id&&m->order==5&&!m->duration.empty())++state.trajectories;});
 auto ready=node->create_subscription<std_msgs::msg::Empty>("/navigation/heartbeat",10,[&](std_msgs::msg::Empty::ConstSharedPtr){state.ready();});
 publish=[&](bool map_enabled,bool goal){
  if(map_enabled){nav_msgs::msg::OccupancyGrid m;state.fillMap(m);m.header.stamp=node->now();maps->publish(m);}
  nav_msgs::msg::Odometry o;state.fillOdom(o,"map");o.header.stamp=node->now();odom->publish(o);
  if(goal){geometry_msgs::msg::PoseStamped g;g.header.frame_id="map";g.header.stamp=node->now();g.pose.position.x=4;g.pose.orientation.w=1;goals->publish(g);}
 };spin=[&]{rclcpp::spin_some(node);};
#else
 ros::init(argc,argv,"ego_mpc_runtime_check");ros::NodeHandle node;
 auto maps=node.advertise<nav_msgs::OccupancyGrid>("/demo/map",1);
 auto odom=node.advertise<nav_msgs::Odometry>("/demo/odom",10);
 auto goals=node.advertise<geometry_msgs::PoseStamped>("/demo/goal",1);
 auto commands=node.subscribe<geometry_msgs::Twist>("/demo/cmd_vel",10,[&](const geometry_msgs::Twist::ConstPtr& m){state.command(*m);});
 auto trajectory=node.subscribe<traj_utils::PolyTraj>("/ego/trajectory",10,[&](const traj_utils::PolyTraj::ConstPtr& m){if(m->request_id&&m->order==5&&!m->duration.empty())++state.trajectories;});
 auto ready=node.subscribe<std_msgs::Empty>("/navigation/heartbeat",10,[&](const std_msgs::Empty::ConstPtr&){state.ready();});
 publish=[&](bool map_enabled,bool goal){
  if(map_enabled){nav_msgs::OccupancyGrid m;state.fillMap(m);m.header.stamp=ros::Time::now();maps.publish(m);}
  nav_msgs::Odometry o;state.fillOdom(o,"world");o.header.stamp=ros::Time::now();odom.publish(o);
  if(goal){geometry_msgs::PoseStamped g;g.header.frame_id="world";g.header.stamp=ros::Time::now();g.pose.position.x=4;g.pose.orientation.w=1;goals.publish(g);}
 };spin=[]{ros::spinOnce();};
#endif
 const auto started=std::chrono::steady_clock::now();auto previous=started;bool sent=false;
 while(state.elapsed<20 && (!sent || state.elapsed<state.goal_at+8.5)){auto current=std::chrono::steady_clock::now();double dt=std::chrono::duration<double>(current-previous).count();previous=current;
  state.elapsed=std::chrono::duration<double>(current-started).count();
  state.x+=state.v*std::cos(state.yaw)*dt;state.y+=state.v*std::sin(state.yaw)*dt;state.yaw+=state.w*dt;
  bool goal=!sent&&state.first_ready>=0&&state.elapsed-state.first_ready>.2&&state.elapsed-state.ready_at<.2;
  if(goal){sent=true;state.goal_at=state.elapsed;}
  publish(!sent||state.elapsed<state.goal_at+5.||state.elapsed>state.goal_at+7.,goal);spin();std::this_thread::sleep_for(std::chrono::milliseconds(20));
 }
 if(!sent||!state.trajectories||state.commands<50)throw std::runtime_error("pipeline did not produce real trajectory and guarded commands");
 if(!state.preview&&(state.x<.2||state.last_nonzero<state.goal_at+4.5))throw std::runtime_error("controller did not track before map-loss test");
 std::cout<<"PASS runtime "<<(state.preview?"preview":"tracking/map-loss/recovery")<<" trajectories="<<state.trajectories<<" commands="<<state.commands<<" x="<<state.x<<" last_nonzero="<<state.last_nonzero<<'\n';
 return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL runtime: "<<e.what()<<" x="<<state.x<<" trajectories="<<state.trajectories<<" last_nonzero="<<state.last_nonzero<<'\n';return 1;}
}
