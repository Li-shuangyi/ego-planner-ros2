#include <ego_mpc_2d/command_gate.h>
#include <stdexcept>
#include <iostream>
using ego_mpc_2d::CommandGate;
void require(bool value){if(!value)throw std::runtime_error("command gate regression");}
int main(){
 CommandGate g;g.map_at=g.odom_at=g.heartbeat_at=10.;require(g.begin(1,100,10));
 require(g.trajectory(1,4,10));require(g.receive(1,100,{.2,0,0,0,0,.3},10));
 require(g.output(10)[0]==0);g.enabled=true;require(g.output(10)[0]==.2);
 require(!g.receive(2,100,{.3,0,0,0,0,0},10));
 require(!g.receive(1,99,{.3,0,0,0,0,0},10));
 g.map_at=g.odom_at=g.heartbeat_at=13.;require(g.output(13)[0]==0 && !g.active);
 require(!g.begin(1,200,13));require(g.begin(2,200,13));require(g.trajectory(2,3,13));
 require(!g.receive(2,200,{NAN,0,0,0,0,0},13) && !g.active);
 require(g.begin(3,300,13));require(g.trajectory(3,3,13));
 require(!g.receive(3,300,{.2,.1,0,0,0,0},13) && !g.active);
 require(g.begin(4,400,13));require(g.trajectory(4,3,13));
 require(!g.receive(4,400,{1,0,0,0,0,0},13) && !g.active);
 require(g.begin(5,500,13));require(g.trajectory(5,3,13));
 require(g.receive(5,500,{.2,0,0,0,0,0},13));g.estop=true;
 require(g.output(13)[0]==0 && !g.active && !g.begin(6,600,13));
 CommandGate frozen;frozen.enabled=true;frozen.map_at=frozen.odom_at=frozen.heartbeat_at=1;
 require(frozen.begin(1,100,1));require(frozen.trajectory(1,20,1));
 require(frozen.receive(1,100,{.2,0,0,0,0,0},1));require(frozen.output(1.6)[0]==0 && !frozen.active);
 std::cout<<"PASS preview, task identity, replay, cancellation, invalid values, limits, estop and steady-clock expiry\n";
}
