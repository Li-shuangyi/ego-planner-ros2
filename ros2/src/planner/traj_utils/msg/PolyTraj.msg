int16 drone_id
int32 traj_id
builtin_interfaces/Time start_time

uint8 order
float32[] coef_x
float32[] coef_y
float32[] coef_z
float32[] duration

std_msgs/Header header
uint64 task_id
bool execute
float64[3] requested_goal
float64[3] effective_goal
int64 map_stamp_ns
