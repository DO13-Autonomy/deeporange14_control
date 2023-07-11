#include <deeporange14_control/StateMachine.h>

namespace deeporange14 {

    StateMachine::StateMachine(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) {
    
        // Instantiate sub/pubs       

        sub_missionStatus = nh.subscribe(std::string(topic_ns + "/mission_status"), 10, &StateMachine::getMissionStatus, this, ros::TransportHints().tcpNoDelay(true));
        sub_brakeStatus = nh.subscribe(std::string(topic_ns + "/brake_enable_stack"), 10, &StateMachine::getBrakeEnable, this, ros::TransportHints().tcpNoDelay(true));
        sub_rosController = nh.subscribe(std::string(topic_ns + "/cmd_trq"), 10, &StateMachine::getTorqueValues, this, ros::TransportHints().tcpNoDelay(true));
        sub_raptorState = nh.subscribe(std::string(topic_ns + "/raptor_state"), 10, &StateMachine::getRaptorMsg, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdvel = nh.subscribe(std::string(topic_ns + "/cmd_vel"), 10, &StateMachine::checkStackStatus, this, ros::TransportHints().tcpNoDelay(true));
        pub_mobility = nh.advertise<deeporange14_msgs::MobilityMsg>(std::string(topic_ns +"/cmd_mobility"), 10, this);

        /* Initiate ROS State with a Startup state to be safe. This state will be published till the ...
        timer object intentionally changes it.Default Node is On and it is running continuously in linux service*/

        state = AU1_STARTUP; 
        raptorhs_fail = true;
        stack_fault = true;
        dbwmode_disable = false;
        mission_status = "";
        brake_enable_stack = true;
        l_torque = 0.0;
        r_torque = 0.0;
        stop_ros = false;
        raptorbrakeAck = false;
        system_state = 2;


        mobilityMsg.left_torque = 0.0;
        mobilityMsg.right_torque = 0.0;
        mobilityMsg.brake_enable = true;
        mobilityMsg.au_state = state;

        start_timer = ros::Time::now().toSec();

        // raptor_hb_timestamp = ros::Time::now().toSec();
        // cmdvel_timestamp = ros::Time::now().toSec();

        // Set up Timer - with calback to publish ROS state all the time that the node is running
        timer = nh.createTimer(ros::Duration(1.0 / 50.0), &StateMachine::publishROSState, this);
        
    }
    StateMachine::~StateMachine(){}

    void StateMachine::checkStackStatus(const geometry_msgs::Twist::ConstPtr& cmdvelMsg){
        cmdvel_timestamp = ros::Time::now().toSec(); 
        // ROS_WARN("cmd_vel timestamp : %f",cmdvel_timestamp); 
    }
    void StateMachine::getMissionStatus(const deeporange14_msgs::MissionStatus::ConstPtr& missionStatus){
        mission_status = missionStatus->status;
        stop_ros = missionStatus->stop_ros;
    }   
    void StateMachine::getBrakeEnable(const std_msgs::Bool::ConstPtr& brakeEnable){
        brake_enable_stack = brakeEnable->data;
    }       
    void StateMachine::getTorqueValues(const deeporange14_msgs::TorqueValuesMsg::ConstPtr& trqvalues){
        l_torque = trqvalues->left_torque;
        r_torque = trqvalues->right_torque;
    }

    void StateMachine::getRaptorMsg(const deeporange14_msgs::RaptorStateMsg::ConstPtr& raptorMsg){
        raptor_hb_timestamp = raptorMsg->header.stamp.sec + raptorMsg->header.stamp.nsec*(1e-9);  
        system_state = raptorMsg->system_state;
        dbwmode_disable = raptorMsg->dbw_mode != 3;
        raptorbrakeAck = raptorMsg->brake_enable_status;        
    }
 
    void StateMachine::publishROSState(const ros::TimerEvent& event)
    {
        /* Always continue to publish ROS state  */
        stack_fault = (std::abs(cmdvel_timestamp - ros::Time::now().toSec()) > 0.1);
        raptorhs_fail = (std::abs(raptor_hb_timestamp - ros::Time::now().toSec()) > 0.025);
        // ROS_WARN("raptorHs fail timestmp %f",raptor_hb_timestamp);
        // ROS_WARN("ros time now %f",ros::Time::now().toSec());
        StateMachine::updateROSStateMsg();
        pub_mobility.publish(mobilityMsg);
    }

    void StateMachine::updateROSStateMsg(){

        switch(state){

            case AU1_STARTUP:{

                mobilityMsg.left_torque = 0.0;
                mobilityMsg.right_torque = 0.0;
                mobilityMsg.brake_enable = true;
                mobilityMsg.au_state = state;
                pub_mobility.publish(mobilityMsg);
                ROS_WARN("In startup");

                if(!raptorhs_fail){ 

                    state = AU2_IDLE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    break;

                }else{
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU1_STARTUP;
                    pub_mobility.publish(mobilityMsg);
                   
                    ROS_WARN("WARN:[AU1_STARTUP]: RaptorHandshake not established");
                    break;
                }
                
            }
            
            case AU2_IDLE:{

                mobilityMsg.left_torque = 0.0;
                mobilityMsg.right_torque = 0.0;
                mobilityMsg.brake_enable = true;
                mobilityMsg.au_state = state;
                pub_mobility.publish(mobilityMsg);
                ROS_WARN("In Idle");


                if (!raptorhs_fail && system_state == 8 && !dbwmode_disable){
                    state = AU3_WAIT_EXECUTION;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    break;
                    
                }
                else if(raptorhs_fail){
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU1_STARTUP;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR: [AU2_IDLE]: RaptorHandshake failed ");
                    break;
                }

                else if (system_state != 8 or dbwmode_disable){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU2_IDLE;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR: [AU2_IDLE]: SystemState not 8 or dbw mode not 3 ");
                    break;
                }


            }
            case AU3_WAIT_EXECUTION:{

                mobilityMsg.left_torque = 0.0;
                mobilityMsg.right_torque = 0.0;
                mobilityMsg.brake_enable = true; // Also check from stack if brake_enable command from stack should be true
                mobilityMsg.au_state = state;
                pub_mobility.publish(mobilityMsg);

                if(!raptorhs_fail && !dbwmode_disable && !stack_fault && brake_enable_stack && mission_status =="executedNav" && !stop_ros){
                    state = AU4_EXEC_IMINENT;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    break;
                }

                else if (raptorhs_fail){
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR:[AU3_WAIT_EXECUTION]: RaptorHandshake failed ");
                    break;
                }
                else if(dbwmode_disable){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU3_WAIT_EXECUTION]: Out of dbwMode ");
                    break;                    
                }
                else if (stack_fault){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU3_WAIT_EXECUTION]:Stack Crashed or failed ");
                    break;
                    
                }

                else if(!brake_enable_stack){
                    state=AU3_WAIT_EXECUTION;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;                   
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU3_WAIT_EXECUTION]:Stack should not disable brakes by now, brk enable should be false, meaning dont enable brakes yet");
                    break;
                }
                else if(mission_status != "executedNav"){
                    state = AU3_WAIT_EXECUTION;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU3_WAIT_EXECUTION]:execute button is not pressed yet or mission status incorrect");
                    break;
                }
                else if(stop_ros){
                    //  go backuint8 left_brkPressure

                    state = AU2_IDLE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU3_WAIT_EXECUTION]:stop button is pressed ");
                    break;
                }

            }
            case AU4_EXEC_IMINENT:{

                mobilityMsg.left_torque = l_torque;
                mobilityMsg.right_torque = r_torque;
                mobilityMsg.brake_enable = false; // Also check from stack if brake_enable command is false from stack,
                //  because now global plan is ready and brakes should be disengaged
                mobilityMsg.au_state = state;
                pub_mobility.publish(mobilityMsg);     

                if(!raptorhs_fail && !dbwmode_disable && !stack_fault && !brake_enable_stack && mission_status =="globalPlanReady"&& !stop_ros){
                        state = AU4_EXEC_IMINENT;
                        mobilityMsg.brake_enable = brake_enable_stack;
                        mobilityMsg.au_state = state;
                        pub_mobility.publish(mobilityMsg);
                        break;
                }

                else if (raptorhs_fail){
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR: [AU4_EXEC_IMINENT]:RaptorHandshake failed ");
                    break;
                }
                else if(dbwmode_disable){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR:[AU4_EXEC_IMINENT]: Out of dbwMode ");
                    break;                    
                }
                else if (stack_fault){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU4_EXEC_IMINENT]:Stack Crashed or failed ");
                    break;
                    
                }
                else if(brake_enable_stack ){
                    state = AU4_EXEC_IMINENT;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU4_EXEC_IMINENT]:Brakes from stack side are still enabled, they need to be disabled by now");
                    break;
                }

                else if(mission_status != "globalPlanReady"){
                    ROS_INFO("My variable value: %s", mission_status.c_str());
                    state = AU4_EXEC_IMINENT;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU4_EXEC_IMINENT]:Execute button pressed but global plan not ready yet");
                    break;
                }
                else if(stop_ros){
                    //  go back to idle
                    state = AU2_IDLE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU4_EXEC_IMINENT]:stop button is pressed ");
                    break;
                }
                      
            }

            case AU5_DISENGAGED_BRAKE:{
                mobilityMsg.left_torque = l_torque;
                mobilityMsg.right_torque = r_torque;
                mobilityMsg.brake_enable = false; // Also check from stack if brake command is true from stack
                mobilityMsg.au_state = state;
                pub_mobility.publish(mobilityMsg);     

                if(!raptorhs_fail && !dbwmode_disable && !stack_fault && !brake_enable_stack && raptorbrakeAck && mission_status =="BrakesDisengaging"&& !stop_ros ){
                        state = AU4_EXEC_IMINENT;
                        mobilityMsg.brake_enable = brake_enable_stack;
                        mobilityMsg.au_state = state;
                        pub_mobility.publish(mobilityMsg);
                        break;
                }

                else if (raptorhs_fail){
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR: [AU5_DISENGAGED_BRAKE]:RaptorHandshake failed ");
                    break;
                }
                else if(dbwmode_disable){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU5_DISENGAGED_BRAKE]:Out of dbwMode ");
                    break;                    
                }
                else if (stack_fault){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU5_DISENGAGED_BRAKE]:Stack Crashed or failed ");
                    break;
                    
                }
                else if(brake_enable_stack ){
                    state = AU5_DISENGAGED_BRAKE;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU5_DISENGAGED_BRAKE]:Brakes enable command from stack is True, it should be false, they need to be disengaged by now");
                    break;
                }
                else if(raptorbrakeAck){
                    state = AU5_DISENGAGED_BRAKE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU5_DISENGAGED_BRAKE]:Raptor brakes disengaged acknowledgement not received !");
                    break;
                }

                else if(mission_status != "BrakesDisengaging"){
                    state = AU5_DISENGAGED_BRAKE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU5_DISENGAGED_BRAKE]:Execute button pressed but global plan not ready yet");
                    break;
                }

                else if(stop_ros){
                    //  go back to idle
                    state = AU2_IDLE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU5_DISENGAGED_BRAKE]:stop button is pressed ");
                    break;
                }

            }                
            
            case AU6_COMMAND_TORQUES:{ 

                while(!raptorhs_fail && !dbwmode_disable && !stack_fault && !brake_enable_stack && mission_status =="CommandingTorques"&& !stop_ros){          
                    
                    state = AU6_COMMAND_TORQUES;
                    mobilityMsg.left_torque = l_torque;
                    mobilityMsg.right_torque = r_torque;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    // Call velcityController->commandTorques method with arguments 
                    if (mission_status == "MissionCompleted" or mission_status == "MissionCancelled"){
                        state = AU7_SAFE_STOP;
                        mobilityMsg.au_state = state;
                        pub_mobility.publish(mobilityMsg);
                        break;
                    }
                    
                }

                if (raptorhs_fail){
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR: [AU6_COMMAND_TORQUES]: RaptorHandshake failed ");
                    break;
                }
                else if(dbwmode_disable){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU6_COMMAND_TORQUES]:Out of dbwMode ");
                    break;                    
                }
                else if (stack_fault){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU6_COMMAND_TORQUES]: Stack Crashed or failed ");
                    break;
                    
                }
                else if(brake_enable_stack ){
                    state = AU5_DISENGAGED_BRAKE;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU6_COMMAND_TORQUES]: Brakes enable command from stack is True, it should be false, they need to be disengaged by now");
                    break;
                }

                else if(mission_status != "CommandingTorques"){
                    state = AU6_COMMAND_TORQUES;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU6_COMMAND_TORQUES]:Mission status not changed to commanding torques from stack side");
                    break;
                }

                else if(stop_ros){
                    //  go back to idle
                    state = AU2_IDLE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU6_COMMAND_TORQUES]:stop button is pressed ");
                    break;
                }
            }
             
            case AU7_SAFE_STOP:{

                if(!raptorhs_fail && !dbwmode_disable && !stack_fault && brake_enable_stack && mission_status =="MissionCompleted" or mission_status == "MissionCancelled" && !stop_ros){
                    state = AU2_IDLE;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);                    
                }

                else if (raptorhs_fail){
                    state = AU1_STARTUP;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR ("ERROR: [AU7_SAFE_STOP]:RaptorHandshake failed ");
                    break;
                }
                else if(dbwmode_disable){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU7_SAFE_STOP]: Out of dbwMode ");
                    break;                    
                }
                else if (stack_fault){
                    state = AU2_IDLE;
                    mobilityMsg.au_state = AU98_FAULT;
                    pub_mobility.publish(mobilityMsg);
                    ROS_ERROR("ERROR: [AU7_SAFE_STOP]: Stack Crashed or failed ");
                    break;
                    
                }
                else if(!brake_enable_stack ){
                    state = AU5_DISENGAGED_BRAKE;
                    mobilityMsg.brake_enable = brake_enable_stack;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU7_SAFE_STOP]:Brakes enable command from stack is False, it should be true, they need to be disengaged by now");
                    break;
                }

                else if(mission_status != "MissionCompleted" or mission_status != "MissionCancelled" ){
                    state = AU7_SAFE_STOP;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU7_SAFE_STOP]: Mission status not changed to completed or cancelled from stack side");
                    break;
                }

                else if(stop_ros){
                    //  go back to idle
                    state = AU2_IDLE;
                    mobilityMsg.au_state = state;
                    pub_mobility.publish(mobilityMsg);
                    ROS_WARN("Warning: [AU7_SAFE_STOP]:stop button is pressed ");
                    break;
                }
            }
            case AU98_FAULT:
                ROS_ERROR("[AU98_Fault_state] : remain in fault");
                break;

            default:
                std::cout << "Unknown state" << std::endl;
                break;
             
            }   

        } 
    }

    
