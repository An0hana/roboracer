uint8 UNKNOWN=0
uint8 OPPONENT=1
uint8 TRACK_BOUNDARY=2
uint8 STATIC_OBSTACLE=3

int32 id
uint8 classification

float64 x
float64 y
float64 yaw
float64 vx
float64 vy

# Frenet coordinates relative to the active reference line.
float64 s
float64 d

float64 length
float64 width
float64 confidence

bool dynamic
bool visible
