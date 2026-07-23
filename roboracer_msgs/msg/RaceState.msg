uint8 INIT=0
uint8 READY=1
uint8 GLOBAL_TRACK=2
uint8 TRAILING=3
uint8 OVERTAKE=4
uint8 FAULT=5
uint8 STOP=6

std_msgs/Header header
uint8 state
string reason
