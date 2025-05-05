# Measured velocities from gps; Vx -> novatel/odom; Wz -> novatel/corrimu 
std_msgs/Header header

float64 measured_vx
float64 measured_wz
uint8 rtk_status
float64 timesec_vx
float64 timesec_wz
float64 timesec_rtk