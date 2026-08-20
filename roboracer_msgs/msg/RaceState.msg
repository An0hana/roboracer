uint8 INIT=0
uint8 READY=1
uint8 GLOBAL_TRACK=2
uint8 TRAILING=3
uint8 OVERTAKE=4
uint8 FAULT=5
uint8 STOP=6
uint8 RETURN=7
uint8 RECOVERY=8

uint8 SAFETY_INIT=0
uint8 SAFETY_READY=1
uint8 SAFETY_FAULT=2
uint8 SAFETY_STOP=3

uint8 BEHAVIOR_RACING=0
uint8 BEHAVIOR_TRAILING=1
uint8 BEHAVIOR_OVERTAKE=2
uint8 BEHAVIOR_RECOVERY=3

uint8 RECOVERY_PHASE_NONE=0
uint8 RECOVERY_PHASE_REVERSE=1
uint8 RECOVERY_PHASE_SETTLE=2

int8 SIDE_RIGHT=-1
int8 SIDE_NONE=0
int8 SIDE_LEFT=1

std_msgs/Header header

# Hierarchical state consumed by the direct-control MPPI.
uint8 safety_state
uint8 behavior_state
uint8 recovery_phase
float64 track_confidence
float64 raceline_weight_scale
float64 safety_weight_scale
float64 lateral_reference_offset
bool stop_requested

# Legacy flat state and planner fields retained for source compatibility.
# The direct-control chain never emits RETURN or requests a local trajectory.
uint8 state

int8 preferred_side
float64 speed_scale
bool use_local_trajectory
bool fallback_ftg

string reason
