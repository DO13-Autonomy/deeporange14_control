#include <deeporange14_control/DeepOrangeStateSupervisor.h>

namespace deeporange14
{

    DeepOrangeStateSupervisor::DeepOrangeStateSupervisor(ros::NodeHandle &nh, ros::NodeHandle &priv_nh)
    {

        // Instantiate sub/pubs

        sub_missionStatus = nh.subscribe(std::string(topic_ns + "/mission_status"), 10, &DeepOrangeStateSupervisor::getMissionStatus, this, ros::TransportHints().tcpNoDelay(true));
        sub_stopRos = nh.subscribe(std::string(topic_ns + "/stop_ros"), 10, &DeepOrangeStateSupervisor::getStopRos, this, ros::TransportHints().tcpNoDelay(true));
        sub_rosController = nh.subscribe(std::string(topic_ns + "/cmd_trq"), 10, &DeepOrangeStateSupervisor::getTorqueValues, this, ros::TransportHints().tcpNoDelay(true));
        sub_raptorState = nh.subscribe(std::string(topic_ns + "/raptor_state"), 10, &DeepOrangeStateSupervisor::getRaptorMsg, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdVel = nh.subscribe(std::string(topic_ns + "/cmd_vel"), 10, &DeepOrangeStateSupervisor::checkStackStatus, this, ros::TransportHints().tcpNoDelay(true));
        pub_mobility = nh.advertise<deeporange14_msgs::MobilityMsg>(std::string(topic_ns + "/cmd_mobility"), 10, this);
        pub_states = nh.advertise<std_msgs::UInt8>(std::string(topic_ns + "/au_states"), 10, this);
        /* Initiate ROS State with a Startup state to be safe. This state will be published till the ...
        timer object intentionally changes it.Default Node is On and it is running continuously in linux service*/

        state = AU_0_DEFAULT;
        raptor_hb_detected = false;
        stack_fault = true;
        dbw_ros_mode = false;
        // dbw_ros_controlled = false;
        mission_status = "";
        tqL_cmd_controller = 0.0;
        tqR_cmd_controller = 0.0;
        stop_ros = false;
        brake_disengaged_threshold = 2.0;
        delay = 0;

        mobilityMsg.tqL_cmd = 0.0;
        mobilityMsg.tqR_cmd = 0.0;
        mobilityMsg.brkL_cmd = 1.0;
        mobilityMsg.brkR_cmd = 1.0;
        mobilityMsg.au_state = state;

        priv_nh.getParam("cmdvel_timeout", cmdvel_timeout);
        priv_nh.getParam("raptorhb_timeout", raptorhb_timeout);
        priv_nh.getParam("update_freq", update_freq);

        // Set up Timer - with calback to publish ROS state all the time that the node is running
        timer = nh.createTimer(ros::Duration(1.0 / update_freq), &DeepOrangeStateSupervisor::supervisorControlUpdate, this);
    }
    DeepOrangeStateSupervisor::~DeepOrangeStateSupervisor() {}

    void DeepOrangeStateSupervisor::checkStackStatus(const geometry_msgs::Twist::ConstPtr &cmdVelMsg)
    {
        cmdvel_timestamp = ros::Time::now().toSec();
        // ROS_WARN("cmd_vel timestamp : %f",cmdvel_timestamp);
    }
    void DeepOrangeStateSupervisor::getMissionStatus(const std_msgs::String::ConstPtr &missionStatus)
    {
        mission_status = missionStatus->data;
    }
    void DeepOrangeStateSupervisor::getTorqueValues(const deeporange14_msgs::TorqueCmdStamped::ConstPtr &controllerTrqValues)
    {
        tqL_cmd_controller  = controllerTrqValues->tqL_cmd;
        tqR_cmd_controller = controllerTrqValues->tqR_cmd;
    }
    void DeepOrangeStateSupervisor::getStopRos(const std_msgs::Bool::ConstPtr &stopRosMsg)
    {
        stop_ros = stopRosMsg->data;
    }
    void DeepOrangeStateSupervisor::getRaptorMsg(const deeporange14_msgs::RaptorStateMsg::ConstPtr &raptorMsg)
    {
        raptor_hb_timestamp = raptorMsg->header.stamp.sec + raptorMsg->header.stamp.nsec * (1e-9);
        dbw_ros_mode = raptorMsg->dbw_mode == DBW_3_ROS_EN || raptorMsg->dbw_mode == DBW_4_ROS_CONTROLLED ;
    
        brkL_pr = raptorMsg->brk_Lpres; 
        brkR_pr = raptorMsg->brk_Rpres; 
        speed_state = raptorMsg->speed_state;
     
    }

    void DeepOrangeStateSupervisor::supervisorControlUpdate(const ros::TimerEvent &event)
    {
        /* Always continue to publish ROS state  */
        stack_fault = (std::abs(cmdvel_timestamp - ros::Time::now().toSec()) > cmdvel_timeout);
        raptor_hb_detected = (std::abs(raptor_hb_timestamp - ros::Time::now().toSec()) < raptorhb_timeout);

        DeepOrangeStateSupervisor::updateROSStateMsg();
        mobilityMsg.au_state = state;
        auStateMsg.data = state;
        pub_states.publish(auStateMsg);
        pub_mobility.publish(mobilityMsg);
    }

    void DeepOrangeStateSupervisor::updateROSStateMsg()
    {

        switch (state)
        {

        case AU_0_DEFAULT:
        {
            prevSt = 0;
            state = AU_1_STARTUP;
            break;
        }
        case AU_1_STARTUP:
        {
            mobilityMsg.tqL_cmd = 0.0;
            mobilityMsg.tqR_cmd = 0.0;
            mobilityMsg.brkL_cmd = 1.0;
            mobilityMsg.brkR_cmd = 1.0;
            mission_status="";
            // ROS_WARN("In startup");

            if(raptor_hb_detected){
                ROS_WARN("WARN:[AU_1_STARTUP]: RaptorHandshake is established, transitioning to AU_2_IDLE ");
                prevSt = 1;
                state = AU_2_IDLE;
                break;
            }
            else
            {
                // do nothing , stay in same state 
                ROS_WARN("[AU_1_STARTUP]:Raptor Handshake failed or not established yet");
                break;
            }


        }

        case AU_2_IDLE:
        {

            mobilityMsg.tqL_cmd = 0.0;
            mobilityMsg.tqR_cmd = 0.0;
            mobilityMsg.brkL_cmd = 1.0;
            mobilityMsg.brkR_cmd = 1.0;
            mission_status="";

            // ROS_WARN("In Idle");

            if (!raptor_hb_detected)
            {
                state = AU_1_STARTUP;
                ROS_ERROR("ERROR: [AU_2_IDLE]: RaptorHandshake failed ");
                break;
            }
            else if (prevSt > 1){
                //  it has returned from fault of below states, add delay
                if (delay < 1000){
                    delay++;
                    break;
                }else{
                    prevSt = 1;
                    break;
                }

            }

            else if(dbw_ros_mode)
            {
                ROS_WARN("WARN: [AU_2_IDLE]: Transitioning to AU_3_ROS_EN ");
                prevSt = 2;
                state = AU_3_ROS_MODE_EN;
                break;
            }

            else 
            {
                // do nothing ,stay in same state
                ROS_WARN("WARN: [AU_2_IDLE]: RaptorHandshake failed or DBW ros mode disabled"); 
                break;
            }
        }
        case AU_3_ROS_MODE_EN:
        {

            mobilityMsg.tqL_cmd = 0.0;
            mobilityMsg.tqR_cmd = 0.0; // Also check from stack if brake_enable command from stack should be true
            mobilityMsg.brkL_cmd = 1.0;
            mobilityMsg.brkR_cmd = 1.0;

            if (!raptor_hb_detected)
            {
                state = AU_1_STARTUP;
                ROS_ERROR("ERROR:[AU_3_ROS_MODE_EN]: RaptorHandshake failed ");
                break;
            }
            else if (!dbw_ros_mode)
            {
                state = AU_2_IDLE;
                ROS_ERROR("ERROR: [AU_3_ROS_MODE_EN]: Out of ROS dbwMode ");
                break;
            }
            else if (stack_fault)
            {
                state = AU_2_IDLE;
                ROS_ERROR("ERROR: [AU_3_ROS_MODE_EN]:Stack Crashed or failed ");
                break;
            }
            else if (stop_ros)
            {
                //  go backuint8 left_brkPressure
                state = AU_2_IDLE;
                ROS_ERROR("ERROR: [AU_3_ROS_MODE_EN]:stop button is pressed ");
                break;
            }


            else if (mission_status == "globalPlanReady")
            {
                prevSt = 3;
                state = AU_4_DISENGAGING_BRAKES; 
                ROS_WARN("[AU_3_ROS_MODE_EN]: Global Plan Ready , disengaging brakes ");
                break;
            }

            else
            {   
                ROS_WARN("[AU_3_ROS_MODE_EN]: Waiting for globalPlan Ready");
                // Do nothing
                break;
            }
        }
        case AU_4_DISENGAGING_BRAKES:
        {

            mobilityMsg.tqL_cmd = tqL_cmd_controller;
            mobilityMsg.tqR_cmd = tqR_cmd_controller;
            mobilityMsg.brkL_cmd = 0.0;
            mobilityMsg.brkL_cmd = 0.0;
            // Also check from stack if brake_enable command is false from stack,
            //  because now global plan is ready and brakes should be disengaged

            if (!raptor_hb_detected)
            {
                state = AU_1_STARTUP;
                ROS_ERROR("ERROR: [AU_4_DISENGAGING_BRAKES]:RaptorHandshake failed ");
                break;
            }
            else if (!dbw_ros_mode)
            {
                state = AU_2_IDLE;
                ROS_ERROR("ERROR:[AU_4_DISENGAGING_BRAKES]: Out of dbwMode ");
                break;
            }
            else if (stack_fault)
            {
                state = AU_2_IDLE;
                break;
                ROS_ERROR("ERROR: [AU_4_DISENGAGING_BRAKES]:Stack Crashed or failed ");
                break;
            }
            else if (stop_ros)
            {
                //  go back to idle
                state = AU_2_IDLE;
                ROS_ERROR("ERROR: [AU_4_DISENGAGING_BRAKES]:stop button is pressed ");
                break;
            }

            else if (speed_state == SPEED_STATE_Ready2Move)
            {
                prevSt = 4;
                state = AU_5_ROS_CONTROLLED;
                ROS_WARN("[AU_4_DISENGAGING_BRAKES]: Brakes disengaged, about to move ");
                break;
            }
            else
            {
                //  Do nothing
                ROS_WARN("[AU_4_DISENGAGING_BRAKES]: Waiting for brakes to be disengaged ");
                break;
            }
        }


        case AU_5_ROS_CONTROLLED:
        {

            mobilityMsg.tqL_cmd = tqL_cmd_controller;
            mobilityMsg.tqR_cmd = tqR_cmd_controller;
            mobilityMsg.brkL_cmd = 0.0;
            mobilityMsg.brkR_cmd = 0.0;
            
            if (!raptor_hb_detected)
            {
                state = AU_1_STARTUP;
                ROS_ERROR("ERROR: [AU_5_ROS_CONTROLLED]: RaptorHandshake failed ");
                break;
            }
            else if (!dbw_ros_mode)
            {
                state = AU_2_IDLE;
                ROS_ERROR("ERROR: [AU_5_ROS_CONTROLLED]:Out of dbwMode ");
                break;
            }
            else if (stack_fault)
            {
                state = AU_2_IDLE;
                ROS_ERROR("ERROR: [AU_5_ROS_CONTROLLED]: Stack Crashed or failed ");
                break;
            }
            else if (stop_ros)
            {
                //  go back to idle
                state = AU_2_IDLE;
                ROS_WARN("Warning: [AU_5_ROS_CONTROLLED]:stop button is pressed ");
                break;
            }

            else if (mission_status == "MissionCompleted" || mission_status == "MissionCancelled")
            {
                prevSt = 5;
                state = AU_3_ROS_MODE_EN ;
                ROS_WARN("[AU_5_ROS_CONTROLLED]: Mission Completed or Mission Cancelled going back to AU_3_ROS_MODE_EN");
                break;
            }
            
            else{
                // do nothing, remain in same state
                ROS_WARN("[AU_5_ROS_CONTROLLED]: Mission Executing");
                break;

            }

        }


        default:{
            prevSt = 0;
            ROS_ERROR(" Unknown State, shut down");
            break;
        }
    }
}
}
