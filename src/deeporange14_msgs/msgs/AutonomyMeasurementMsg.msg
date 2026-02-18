# Message containing information to send over CAN from Raptor to autonomy stack

std_msgs/Header header

float32 vx_meas
float32 curv_meas

uint8 dbw_state
uint8 dbw_seq

# constants for clamping and adusting commands
float32 VX_FACTOR = 0.001
float32 VX_MIN = -131.072
float32 VX_MAX = 131.071

float32 CURV_FACTOR = 0.001
float32 CURV_MIN = -131.072
float32 CURV_MAX = 131.071

# constants for clamping state and sequence
uint8 STATE_MIN = 0
uint8 STATE_MAX = 255

uint8 SEQ_MIN = 0
uint8 SEQ_MAX = 255