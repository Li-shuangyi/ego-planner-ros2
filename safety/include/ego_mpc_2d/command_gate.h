#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace ego_mpc_2d {
// Receipts use steady-clock seconds; message timestamps are checked by each ROS adapter.
struct CommandGate {
  bool enabled=false, estop=false;
  uint64_t active=0, retired=0;
  int64_t started_ns=0;
  double max_speed=.4, max_yaw_rate=.8;
  double map_at=-INFINITY, odom_at=-INFINITY, heartbeat_at=-INFINITY;
  double command_at=-INFINITY, expires=-INFINITY, waiting_until=-INFINITY;
  std::array<double,6> command{};
  bool fresh(double now) const {
    return now-map_at<=.5 && now-odom_at<=.3 && now-heartbeat_at<=.3;
  }
  void cancel() { retired=std::max(retired,active); active=0; command={}; }
  bool begin(uint64_t id,int64_t stamp,double now) {
    if(estop || !fresh(now) || !id || id<=retired) return false;
    cancel(); active=id; started_ns=stamp; command_at=expires=-INFINITY;
    waiting_until=now+2.; return true;
  }
  bool trajectory(uint64_t id,double remaining,double now) {
    if(id!=active || id<=retired || !std::isfinite(remaining) || remaining<=0 || remaining>300) return false;
    expires=now+remaining; return true;
  }
  bool receive(uint64_t id,int64_t stamp,const std::array<double,6>& value,double now) {
    if(!active || id!=active || id<=retired || stamp<started_ns || estop || !fresh(now) || now>expires) return false;
    for(double v:value) if(!std::isfinite(v)) {cancel();return false;}
    if(std::abs(value[0])>max_speed+1e-5 || std::abs(value[5])>max_yaw_rate+1e-5 ||
       std::abs(value[1])+std::abs(value[2])+std::abs(value[3])+std::abs(value[4])>1e-8) {cancel();return false;}
    command=value; command_at=now; return true;
  }
  std::array<double,6> output(double now) {
    if(!active) return {};
    if(estop || !fresh(now) || (now>waiting_until && (now>expires || now-command_at>.2))) {cancel();return {};}
    if(!enabled || now>expires || now-command_at>.2) return {};
    return command;
  }
};
}
