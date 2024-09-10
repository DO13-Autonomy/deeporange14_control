/* 
    A class definition to automatically initiate Rosbag record (for certain
    topics) and CAN data logging (for all CAN buses together). Data logging 
    starts (if not already happening) when the signal log_cmd in /raptor_state
    (received via CAN) becomes 1 and stops when it becomes 0.
    In addition, a (retro-active) CAN log file is created when the Raptor
    system state becomes larger than or equal to 100 (which indicates a fault state)
*/

#include <deeporange14_control/DataLogger.h>

namespace deeporange14
{
    DataLogger::DataLogger(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
    {
        // Obtain ros::Subscriber object
        sub_raptor_ = node.subscribe(std::string(topic_ns + "/raptor_state"), 10, 
                                     &DataLogger::recordRosbagAndCANlog, this, 
                                     ros::TransportHints().tcpNoDelay(true));
        
        // Initialize recording state to false
        isRecording = false;
        isSystemFault = false;

        // start candump
        startcandumpPeriod = ros::Duration(5.0); // 5 seconds
        lastStartcandump = ros::Time::now();
        system("~/Scripts/startcandump");
    }
    
    DataLogger::~DataLogger(){}

    /* callback for the topic /raptor_state */
    void DataLogger::recordRosbagAndCANlog(const deeporange14_msgs::RaptorStateMsg::ConstPtr& msg)
    {
        // restart candump if it failed for some reason
        ros::Time now = ros::Time::now();
        if ((now - lastStartcandump) > startcandumpPeriod)
        {
            lastStartcandump = now;
            system("~/Scripts/startcandump");
        }

        // toggle logging if the value of log_cmd changes
        if (msg->log_cmd && !isRecording)
        {
            // start both ROS and CAN logging -- runs in the background and terminates when logging is stopped
            system("~/Scripts/startlogging &");
            isRecording = true;
            ROS_INFO("Started CAN and ROS logging. Current System State = %d", msg->system_state);
        }
        else if (!msg->log_cmd && isRecording)
        {
            // stop both ROS and CAN logging
            system("~/Scripts/stoplogging");
            isRecording = false;
            ROS_INFO("Rosbag record and CAN logging terminated");
        }
        
        // create an error log if the system state indicates a fault
        if (msg->system_state > 100 && !isSystemFault)
        {
            // transition to fault state has occurred
            system(("~/Scripts/savelogonerror "+std::to_string(msg->system_state)).c_str());
            isSystemFault = true;
            ROS_INFO("Creating error log.  Current System State = %d", msg->system_state);
        }
        else if (msg->system_state <=100)
        {
            // recovered from system fault --> reset state
            isSystemFault = false;
        }
    }
}
