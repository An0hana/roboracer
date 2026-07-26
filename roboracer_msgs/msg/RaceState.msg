uint8 INIT=0
uint8 READY=1
uint8 GLOBAL_TRACK=2
uint8 TRAILING=3
uint8 OVERTAKE=4
uint8 FAULT=5
uint8 STOP=6
uint8 RETURN=7

int8 SIDE_RIGHT=-1
int8 SIDE_NONE=0
int8 SIDE_LEFT=1

std_msgs/Header header
uint8 state

# Behavior command consumed by the planner/controller.
int8 preferred_side
float64 speed_scale
bool use_local_trajectory
bool fallback_ftg

string reason
