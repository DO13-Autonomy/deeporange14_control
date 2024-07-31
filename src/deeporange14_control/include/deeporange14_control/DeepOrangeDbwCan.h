/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#ifndef _DEEPORANGE_DBW_CAN_H_
#define _DEEPORANGE_DBW_CAN_H_

#include <ros/console.h>
#include <string.h>
#include <ros/ros.h>
// ROS messages
#include <can_msgs/Frame.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <deeporange14_msgs/RaptorStateMsg.h>
#include <deeporange14_msgs/MobilityMsg.h>
#include <deeporange14_msgs/AuStatusMsg.h>
#include <novatel_oem7_msgs/INSPVAX.h>
// Can dbc parser mackage
#include <can_dbc_parser/DbcMessage.h>
#include <can_dbc_parser/DbcSignal.h>
#include <can_dbc_parser/Dbc.h>
#include <can_dbc_parser/DbcBuilder.h>
// enumerations
#include <deeporange14_control/dispatch_can_msgs.h>

namespace deeporange14
{
class DeepOrangeDbwCan
{
    public:
    DeepOrangeDbwCan(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
    ~DeepOrangeDbwCan();

    private:
    void recvCAN(const can_msgs::Frame::ConstPtr& msg);
    void publishCAN(const ros::TimerEvent& event);
    void publishCommandstoCAN(const deeporange14_msgs::MobilityMsg& msg);
    void publishAuStatustoCAN(const deeporange14_msgs::AuStatusMsg& msg);
    void getMeasuredVx(const nav_msgs::Odometry& msg);
    void getMeasuredWz(const sensor_msgs::Imu& msg);
    void getRtkStatus(const novatel_oem7_msgs::INSPVAX& msg);
    void publishAuStatus(const ros::TimerEvent& event);
    // int getLogStatus(const deeporange14_msgs::*****& msg); // TODO -> Include message

    // Ros timer object
    ros::Timer timer_;

    // Publishers
    ros::Publisher pub_can_;
    ros::Publisher pub_raptorState_;
    ros::Publisher pub_auStatus_;

    // Subscribers
    ros::Subscriber sub_can_;
    ros::Subscriber sub_auMobility_;
    ros::Subscriber sub_auStatus_;
    ros::Subscriber sub_gpsImu_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_rtk_;
    ros::Subscriber sub_autonomyLog_;

    // Published msgs
    deeporange14_msgs::RaptorStateMsg raptorMsg_;
    deeporange14_msgs::AuStatusMsg auStatusMsg_;

    // Subscribed msgs
    deeporange14_msgs::MobilityMsg mobilityMsg_;
    nav_msgs::Odometry odometryMsg_;
    sensor_msgs::Imu gpsImuMsg_;

    // Variables for measured velocities
    std::vector<float> vectorWz_;
    float averageWz_ = 0;
    std::vector<float> vectorVx_;
    float averageVx_ = 0;

    // Frame ID
    std::string frameId_;
    can_msgs::Frame frame_;

    // dbc files
    NewEagle::Dbc rosDbc_;
    NewEagle::Dbc raptorDbc_;
    std::string dbcFileRos_;
    std::string dbcFileRaptor_;

    // Heartbeat var & ptr
    int ros_hb_ = 0;
    int *ros_hb_ptr_ = &ros_hb_;

    // Brake pressure var
    int brk_ip_Rpres_;  // right
    int brk_ip_Lpres_;  // left

    // Variables to store rtk, log, and gps velocities and their pointers
    int rtk_status_ = 0;
    double meas_Vx_ = 0.0;
    double meas_Wz_ = 0.0;
    int *rtk_status_ptr_ = &rtk_status_;
    double *measVx_ptr_ = &meas_Vx_;
    double *measWz_ptr_ = &meas_Wz_;

    // Variables to store timestamp and their pointers
    int time_Vx_ = 0;
    int time_Wz_ = 0;
    int time_Rtk_ = 0;
    int *time_Vx_ptr_ = &time_Vx_;
    int *time_Wz_ptr_ = &time_Wz_;
    int *time_Rtk_ptr_ = &time_Rtk_;

    // Param to retrive
    int log_st;
    std::string topic_ns = "/deeporange1314";
};
}  // namespace deeporange14

#endif  // _DEEPORANGE_DBW_CAN_H_
