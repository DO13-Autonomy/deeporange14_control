/* A high level state machine to interact with Raptor to control brakes torque command */

#ifndef _DEEPORANGE_STATE_SUPERVISOR_H_
#define _DEEPORANGE_STATE_SUPERVISOR_H_

#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include <string>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <deeporange14_control/DeeporangeStateEnums.h>
#include <deeporange14_msgs/MobilityMsg.h>
#include <deeporange14_msgs/MissionStatus.h>
#include <deeporange14_msgs/RaptorStateMsg.h>
#include <deeporange14_msgs/TorqueCmdStamped.h>


namespace deeporange14
{
    class DeepOrangeStateSupervisor{
        public:
        DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
        ~DeepOrangeStateSupervisor();

        private:
        void checkStackStatus(const geometry_msgs::Twist::ConstPtr& cmdVelMsg);
        
        void getMissionStatus(const std_msgs::String::ConstPtr& missionStatus);
    
        
        void getTorqueValues(const deeporange14_msgs::TorqueCmdStamped::ConstPtr& controllerTrqValues);

        void getStopRos(const std_msgs::Bool::ConstPtr& stopRosMsg);

        void getRaptorMsg(const deeporange14_msgs::RaptorStateMsg::ConstPtr& raptorMsg);

        void supervisorControlUpdate(const ros::TimerEvent& event);

        void updateROSState();

        //member variables 
        bool raptor_hb_detected;
        bool stack_fault;
        // bool dbw_ros_en;
        // bool dbw_ros_controlled;
        bool dbw_ros_mode;
        std::string mission_status;
        float brkL_pr;
        float brkR_pr;
        float tqL_cmd_controller;
        float tqR_cmd_controller;
        bool stop_ros;
        bool raptorbrakeAck;
        int delay;
        int desired_delay;
        int delay_threshold;
        int prevSt;

        allStates state;
        double raptor_hb_timestamp;
        double cmdvel_timestamp;
        double stop_ros_timestamp;
        double mission_update_timestamp;
        uint speed_state;
        uint au_state;
        double counter;
        float cmdvel_timeout;
        float raptorhb_timeout;
        int update_freq;
        float brake_disengaged_threshold;
        
        // Publishers
        ros::Timer timer;
        ros::Publisher pub_mobility; 
        ros::Publisher pub_states;

        // Subscribers
        ros::Subscriber sub_cmdVel;
        ros::Subscriber sub_missionStatus;
        ros::Subscriber sub_brakeStatus;
        ros::Subscriber sub_rosController;
        ros::Subscriber sub_rosStop;
        ros::Subscriber sub_raptorState;
        ros::Subscriber sub_stopRos;
        std::string topic_ns = "/deeporange1314";
        
        // Init the msg variables
        std_msgs::UInt8 auStateMsg;
        deeporange14_msgs::MobilityMsg mobilityMsg;
        deeporange14_msgs::TorqueCmdStamped trqvalues;
        deeporange14_msgs::RaptorStateMsg raptorMsg;

    };

}

#endif
