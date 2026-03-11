# Message containing internal information that can be used to help debug the state supervisor

std_msgs/Header header

# internal state information (should match sent/received states)
uint8 au_state
uint8 dbw_state

# internal command and measurments (shoudl match sent/received values)
float32 vx_cmd
float32 vx_meas

float32 curv_cmd
float32 curv_meas

# commanded and calculated measurement of yaw rate
float32 wx_cmd
float32 wx_meas

# internal booleans used to control some state transitions
bool stack_fault
bool stop_ros
bool mission_running
bool mission_complete
bool mission_aborted
