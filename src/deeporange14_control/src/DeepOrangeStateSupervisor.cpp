/*
Implement a state machine based on information from both the Raptor and Phoenix autonomy stack
*/

#include <string>
#include <deeporange14_control/DeepOrangeStateSupervisor.h>

namespace deeporange14 {
DeepOrangeStateSupervisor::DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh) {
  /* subscribers and publishers */

  // from Phoenix
  sub_missionStatus_ = node.subscribe(std::string(topic_ns_ + "/mission_status"), 10,
                                   &DeepOrangeStateSupervisor::getMissionStatus, this,
                                   ros::TransportHints().tcpNoDelay(true));  // TODO - this is for sequence controller, needed?
  sub_mppi_mission_ = node.subscribe(std::string(topic_ns_ + "/mission/status"), 10,
                            &DeepOrangeStateSupervisor::getPhxStatus, this, ros::TransportHints().tcpNoDelay(true));

  sub_stopRos_ = node.subscribe(std::string(topic_ns_ + "/stop_ros"), 10, &DeepOrangeStateSupervisor::getStopRos, this,
                             ros::TransportHints().tcpNoDelay(true));
  sub_cmdVel_ = node.subscribe(std::string(topic_ns_ + "/cmd_vel"), 10, &DeepOrangeStateSupervisor::checkStackStatus, this,
                            ros::TransportHints().tcpNoDelay(true));

  sub_au_meas_ = node.subscribe(std::string(topic_ns_ + "/au_meas"), 10,
                              &DeepOrangeStateSupervisor::getMeasurements, this, ros::TransportHints().tcpNoDelay(true));

  pub_au_cmd_ = node.advertise<deeporange14_msgs::AutonomyCommandMsg>(std::string(topic_ns_ + "/au_cmd"), 10, this);
  
  /* Initiate ROS State in the Default state and false booleans to ensure transition only when it actually receives a 
     True. This state will be published by the timer object at 50Hz update_freq. */

  au_state_ = AU_0_NO_HEARTBEAT;
  raptor_hb_detected = false;
  stack_fault = true;
  dbw_ros_mode = false;
  stop_ros_timestamp = 0.0;

  mission_status = "";
  stop_ros = false;
  
  vx_meas_ = 0.0;
  vx_cmd_ = 0.0;
  curv_meas_ = 0.0;
  curv_cmd_ = 0.0;
  dbw_state_ = DBW_0_AUTO_OFF;
  au_state_ = AU_0_NO_HEARTBEAT;
  prev_au_state_ = AU_0_NO_HEARTBEAT;

  priv_nh.getParam("cmdvel_timeout", cmdvel_timeout);
  priv_nh.getParam("raptorhb_timeout", raptorhb_timeout);
  priv_nh.getParam("update_freq", update_freq);

  fault_delay_s_ = 20;  // after fault, wait 20 sec before state transition
  fault_delay_ct_ = fault_delay_s_ * update_freq;  // threshold for counter
  fault_delay_ = 0;  // delay counter

  // set up timer to publish autonomy commands
  timer_ = node.createTimer(ros::Duration(1.0 / update_freq), &DeepOrangeStateSupervisor::supervisorControlUpdate, this);
}
DeepOrangeStateSupervisor::~DeepOrangeStateSupervisor() {}

void DeepOrangeStateSupervisor::getMeasurements(const deeporange14_msgs::AutonomyMeasurementMsg::ConstPtr& msg) {
  // unpack the AutonomyMeasurement message to get information from the Raptor
  float vx_tmp = msg->vx_meas;
  float curv_tmp = msg->curv_meas;

  // speed and curvature must be (1) multiplied by the factor and (2) clamped by the limits in the message
  vx_meas_ = std::min(std::max(vx_tmp * msg->VX_FACTOR, msg->VX_MIN), msg->VX_MAX);
  curv_meas_ = std::min(std::max(curv_tmp * msg->CURV_FACTOR, msg->CURV_MIN), msg->CURV_MAX);

  dbw_state_ = msg->dbw_state;  // state can be directly translated from the measurement message

  // TODO - publish these measurements to appropriate topic, calculate yaw_rate and publish?
}

void DeepOrangeStateSupervisor::checkStackStatus(const geometry_msgs::Twist::ConstPtr &cmdVelMsg) {
  cmdvel_timestamp = ros::Time::now().toSec();
}

void DeepOrangeStateSupervisor::getMissionStatus(const std_msgs::String::ConstPtr &missionStatus) {
  mission_update_timestamp = ros::Time::now().toSec();
  mission_status = missionStatus->data;
}

void DeepOrangeStateSupervisor::getStopRos(const std_msgs::Bool::ConstPtr &stopRosMsg) {
  stop_ros_timestamp = ros::Time::now().toSec();
}

void DeepOrangeStateSupervisor::supervisorControlUpdate(const ros::TimerEvent &event) {
  /* Always continue to publish ROS state  */
  stack_fault = ros::Time::now().toSec() - cmdvel_timestamp > cmdvel_timeout;
  raptor_hb_detected = ros::Time::now().toSec() - raptor_hb_timestamp < raptorhb_timeout;
  stop_ros = (ros::Time::now().toSec() - stop_ros_timestamp) < 5 ? 1 : 0;
  mission_status = (ros::Time::now().toSec() - mission_update_timestamp < 5) ? mission_status : "";

  DeepOrangeStateSupervisor::updateROSState();

  //mobilityMsg.au_state = state;
  //auStateMsg.data = state;
  //pub_states.publish(auStateMsg);  // Additional standard msg for stack side
  //pub_mobility.publish(mobilityMsg);  // custom deeporange14 msg for DBW Can node
}

void DeepOrangeStateSupervisor::getPhxStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &statusMsg)
{
  if (!statusMsg->status_list.empty())
  {
    mppi_status = statusMsg->status_list[0].status;
    ROS_DEBUG("MPPI Status: %d", mppi_status);
  }
}

void DeepOrangeStateSupervisor::updateROSState() {
  switch (state) {
    case AU_0_NO_HEARTBEAT: {
      ROS_DEBUG("In default state, AU_0_NO_HEARTBEAT");

      prev_au_state_ = AU_0_NO_HEARTBEAT;
      au_state_ = AU_1_WAITING_HEARTBEAT;
      break;
    }
    case AU_1_WAITING_HEARTBEAT: {
      ROS_DEBUG("In startup state, AU_1_WAITING_HEARTBEAT");

      if (raptor_hb_detected) {
        ROS_INFO("[AU_1_WAITING_HEARTBEAT]: Raptor handshake is established, transitioning to AU_2_WAITING_HANDOFF");
        prev_au_state_ = AU_1_WAITING_HEARTBEAT;
        au_state_ = AU_2_WAITING_HANDOFF;
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_1_WAITING_HEARTBEAT]: Raptor handshake not established yet");
        break;
      }
    }
    case AU_2_WAITING_HANDOFF: {
      ROS_DEBUG("In idle state, AU_2_WAITING_HANDOFF");

      if (!raptor_hb_detected) {
        au_state_ = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_2_WAITING_HANDOFF]: Raptor handshake failed");
        break;
      }
      else if (prev_au_state_ > AU_1_WAITING_HEARTBEAT) {
        // it has returned from fault of below states, add delay
        // TODO - resetting this counter (if it is kept)
        if (fault_delay_ < fault_delay_ct_) {
          fault_delay_++;
          break;
        }
        else {
          prev_au_state_ = AU_1_WAITING_HEARTBEAT;
          break;
        }
      }
      else if (dbw_ros_mode) {
        ROS_INFO("[AU_2_WAITING_HANDOFF]: Transitioning to AU_3_ROS_EN");
        prev_au_state_ = AU_2_WAITING_HANDOFF;
        au_state_ = AU_3_READY_FOR_MISSION;
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_2_WAITING_HANDOFF]: Ready to enter DBW-ROS mode");
        break;
      }
    }
    case AU_3_READY_FOR_MISSION: {
      ROS_DEBUG("In DBW-ROS state, AU_3_READY_FOR_MISSION");

      if (!raptor_hb_detected) {
        au_state_ = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Raptor handshake failed");
        break;
      }
      else if (!dbw_ros_mode) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Exiting DBW-ROS mode");
        break;
      }
      else if (stack_fault) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_ERROR("[AU_3_READY_FOR_MISSION]: Stack crashed or failed");
        break;
      }
      else if (stop_ros) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Stop button is pressed");
        break;
      }
      else if (mppi_status == 1 || mppi_status == 4) {
        prev_au_state_ = AU_3_READY_FOR_MISSION;
        au_state_ = AU_4_MISSION_IN_PROGRESS;
        ROS_INFO("[AU_3_READY_FOR_MISSION]: Local plan ready, disengaging brakes");
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_3_READY_FOR_MISSION]: Waiting for local plan ready signal");
        break;
      }
    }
    case AU_4_MISSION_IN_PROGRESS: {
      ROS_DEBUG("In disengage brake state, AU_4_MISSION_IN_PROGRESS");

      // Also check from stack if brake_enable command is false from stack,
      // because plan is ready and brakes should be disengaged before moving
      if (!raptor_hb_detected) {
        au_state_ = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Raptor handshake failed");
        break;
      }
      else if (!dbw_ros_mode) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Exiting DBW-ROS mode");
        break;
      }
      else if (stack_fault) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_ERROR("[AU_4_MISSION_IN_PROGRESS]: Stack crashed or failed");
        break;
      }
      else if (mppi_status == 3) {
        au_state_ = AU_3_READY_FOR_MISSION;
        ROS_DEBUG("[AU_4_MISSION_IN_PROGRESS]: Phoenix stack mission status changed to %d", mppi_status);
        break;
      }
      else if (stop_ros) {
        //  go back to idle
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Stop button is pressed");
        break;
      }
      else if (speed_state == SPEED_STATE_Ready2Move) {
        prev_au_state_ = AU_4_MISSION_IN_PROGRESS;
        //state = AU_5_ROS_CONTROLLED;
        ROS_INFO("[AU_4_MISSION_IN_PROGRESS]: Brakes disengaged, transitioning to ROS-controlled mode, ready to move");
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_4_MISSION_IN_PROGRESS]: Waiting for ROS-controlled mode");
        break;
      }
    }
    /*case AU_5_ROS_CONTROLLED: {
      ROS_DEBUG("In ROS-controlled state, AU_5_ROS_CONTROLLED");

      mobilityMsg.tqL_cmd = tqL_cmd_controller;
      mobilityMsg.tqR_cmd = tqR_cmd_controller;
      mobilityMsg.brkL_cmd = 0.0;
      mobilityMsg.brkR_cmd = 0.0;

      if (!raptor_hb_detected) {
        state = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_5_ROS_CONTROLLED]: Raptor handshake failed");
        break;
      }
      else if (!dbw_ros_mode) {
        state = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_5_ROS_CONTROLLED]: Exiting DBW-ROS mode");
        break;
      }
      else if (stack_fault) {
        state = AU_2_WAITING_HANDOFF;
        ROS_ERROR("[AU_5_ROS_CONTROLLED]: Stack crashed or failed ");
        break;
      }
      else if (mppi_status == 3) {
        state = AU_3_READY_FOR_MISSION;
        ROS_DEBUG("WARN: [AU_5_ROS_CONTROLLED]: Phoenix Stack mission status changed to %d", mppi_status);
        break;
      }
      else if (stop_ros) {
        //  go back to idle
        state = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_5_ROS_CONTROLLED]: Stop button is pressed");
        break;
      }
      else if (mission_status == "MissionCompleted" || mission_status == "MissionCancelled") {
        prev_au_state_ = AU_5_ROS_CONTROLLED;
        state = AU_3_READY_FOR_MISSION;
        ROS_INFO("[AU_5_ROS_CONTROLLED]: Mission completed or cancelled; returning to AU_3_READY_FOR_MISSION");
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_5_ROS_CONTROLLED]: Mission Executing");
        break;
      }
    }*/
    default: {
      prev_au_state_ = AU_0_NO_HEARTBEAT;
      ROS_ERROR("Unknown State, shut down");
      break;
    }
  }
}
}  // namespace deeporange14
