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
        void checkStackStatus(const geometry_msgs::Twist::ConstPtr& cmdVelMsg); // Stack fault is  true if there are no new cmd_vel msgs from stack side
        void getMissionStatus(const std_msgs::String::ConstPtr& missionStatus); //get stack status and set to class variables      
        void getTorqueValues(const deeporange14_msgs::TorqueCmdStamped::ConstPtr& controllerTrqValues);// Get torque values from velocity controller node
        void getStopRos(const std_msgs::Bool::ConstPtr& stopRosMsg);// get bool of stopRos from stack side
        void getRaptorMsg(const deeporange14_msgs::RaptorStateMsg::ConstPtr& raptorMsg); // get raptor systems, dbwmode, speed st, and raptor hb timeout check
        void supervisorControlUpdate(const ros::TimerEvent& event); // contantly publish appropriate states based on logic and transition at 50Hz
        void updateROSStateMsg();

        //member variables 
        bool raptor_hb_detected; // true if raptor timestamp and msg are old
        bool stack_fault; // true if cmd_vel timestamp and msg are old
        bool dbw_ros_mode; // if it is 3 or 4
        std::string mission_status; // expected values: GlobalPlanReady, MissionCancelled, MissionCompleted 
        float tqL_cmd_controller; // left torque from velcontroller node
        float tqR_cmd_controller; // right torque from velcontroller node
        bool stop_ros; // stack flag to end mission
        uint prevSt; // keep track of every previous state coming from transition, to know that we returning from fault 
        uint delay; // having a 20 secs delay counter after returning to IDLE from fault
        uint delay_threshold; // its update_frequency * (20secs)
        allStates state; // containing enum of all expected states
        double raptor_hb_timestamp; // collect timestamp of raptor msg
        double cmdvel_timestamp; // initializes to ros time now for every new cmd_vel msg 
        uint speed_state; // raptor ready to move state
        uint au_state; // to publish std_msgs to stack
        double counter;
        float cmdvel_timeout; // allowable timeout
        float raptorhb_timeout; //allowable threshold 
        int update_freq; // time update freq
        
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
