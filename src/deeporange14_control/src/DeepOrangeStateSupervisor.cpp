/*This class acts as a StateMachine and transitions into correct state based on Raptor, Autonomy Stack information*/

#include <deeporange14_control/DeepOrangeStateSupervisor.h>

namespace deeporange14 {
DeepOrangeStateSupervisor::DeepOrangeStateSupervisor(rclcpp::Node::SharedPtr node) : node_(node) {
  // Instantiate sub/pubs
  // TODO - manage QOS for subscriptions to match the "TCPNoDelay" transport hint
  sub_missionStatus = node->create_subscription<std_msgs::msg::String>(std::string(topic_ns + "/mission_status"), 10,
                                   std::bind(&DeepOrangeStateSupervisor::getMissionStatus, this, std::placeholders::_1));  // ros::TransportHints().tcpNoDelay(true));
  sub_stopRos = node->create_subscription<std_msgs::msg::Bool>(std::string(topic_ns + "/stop_ros"), 10,
                                   std::bind(&DeepOrangeStateSupervisor::getStopRos, this, std::placeholders::_1));  // ros::TransportHints().tcpNoDelay(true));
  sub_rosController = node->create_subscription<deeporange14_msgs::msg::TorqueCmdStamped>(std::string(topic_ns + "/cmd_trq"), 10,
                                   std::bind(&DeepOrangeStateSupervisor::getTorqueValues, this, std::placeholders::_1));  // ros::TransportHints().tcpNoDelay(true));
  sub_raptorState = node->create_subscription<deeporange14_msgs::msg::RaptorState>(std::string(topic_ns + "/raptor_state"), 10,
                                   std::bind(&DeepOrangeStateSupervisor::getRaptorMsg, this, std::placeholders::_1));  // ros::TransportHints().tcpNoDelay(true));
  sub_cmdVel = node->create_subscription<geometry_msgs::msg::Twist>(std::string(topic_ns + "/cmd_vel"), 10,
                                   std::bind(&DeepOrangeStateSupervisor::checkStackStatus, this, std::placeholders::_1));  // ros::TransportHints().tcpNoDelay(true));
  sub_mppi_mission = node->create_subscription<actionlib_msgs::msg::GoalStatusArray>(std::string(topic_ns + "/mission/status"), 10,
                                   std::bind(&DeepOrangeStateSupervisor::getPhxStatus, this, std::placeholders::_1));  // ros::TransportHints().tcpNoDelay(true));
  
  pub_mobility = node->create_publisher<deeporange14_msgs::msg::Mobility>(std::string(topic_ns + "/cmd_mobility"), 10);
  pub_states = node->create_publisher<std_msgs::msg::UInt16>(std::string(topic_ns + "/au_states"), 10);
  /* Initiate ROS State in the Default state and false booleans to ensure transition only when it actually receives a 
     True. This state will be published till the timer object at 50Hz update_freq.
  */

  state = AU_0_DEFAULT;
  raptor_hb_detected = false;
  stack_fault = true;
  dbw_ros_mode = false;
  stop_ros_timestamp = 0.0;

  mission_status = "";
  tqL_cmd_controller = 0.0;
  tqR_cmd_controller = 0.0;
  stop_ros = false;
  brake_disengaged_threshold = 2.0;
  delay = 0;

  //  Initiate the mobility torque and brake commands to avoid garbage value initialization
  mobilityMsg.tql_cmd = 0.0;
  mobilityMsg.tqr_cmd = 0.0;
  mobilityMsg.brkl_cmd = 1.0;
  mobilityMsg.brkr_cmd = 1.0;
  mobilityMsg.au_state = state;

  node->get_parameter("cmdvel_timeout", cmdvel_timeout);
  node->get_parameter("raptorhb_timeout", raptorhb_timeout);
  node->get_parameter("update_freq", update_freq);
  desired_delay = 20;  // Adding 20 secs delay after fault to wait for transition
  delay_threshold = desired_delay * update_freq;

  // Set up Timer - with calback to publish ROS state all the time that the node is running
  timer = node->create_wall_timer(std::chrono::duration<double, std::nano>(1.0 / update_freq), std::bind(&DeepOrangeStateSupervisor::supervisorControlUpdate, this));
}

DeepOrangeStateSupervisor::~DeepOrangeStateSupervisor() {}

void DeepOrangeStateSupervisor::checkStackStatus(const geometry_msgs::msg::Twist &cmdVelMsg) {
  cmdvel_timestamp = node_->get_clock()->now().seconds();
}

void DeepOrangeStateSupervisor::getMissionStatus(const std_msgs::msg::String &missionStatus) {
  mission_update_timestamp = node_->get_clock()->now().seconds();
  mission_status = missionStatus.data;
}

void DeepOrangeStateSupervisor::getTorqueValues(
    const deeporange14_msgs::msg::TorqueCmdStamped &controllerTrqValues) {
  tqL_cmd_controller  = controllerTrqValues.tql_cmd;
  tqR_cmd_controller = controllerTrqValues.tqr_cmd;
}

void DeepOrangeStateSupervisor::getStopRos(const std_msgs::msg::Bool &stopRosMsg) {
  stop_ros_timestamp = node_->get_clock()->now().seconds();
}

void DeepOrangeStateSupervisor::getRaptorMsg(const deeporange14_msgs::msg::RaptorState &raptorMsg) {
  raptor_hb_timestamp = raptorMsg.header.stamp.sec + raptorMsg.header.stamp.nanosec * (1e-9);
  // transition to DBW mode if ROS mode 3 or 4
  dbw_ros_mode = raptorMsg.dbw_mode == DBW_3_ROS_EN || raptorMsg.dbw_mode == DBW_4_ROS_CONTROLLED;
  brkL_pr = raptorMsg.brk_lpres;
  brkR_pr = raptorMsg.brk_rpres;
  speed_state = raptorMsg.speed_state;
}

void DeepOrangeStateSupervisor::supervisorControlUpdate() {
  /* Always continue to publish ROS state  */
  stack_fault = node_->get_clock()->now().seconds() - cmdvel_timestamp > cmdvel_timeout;
  raptor_hb_detected = node_->get_clock()->now().seconds() - raptor_hb_timestamp < raptorhb_timeout;
  stop_ros = (node_->get_clock()->now().seconds() - stop_ros_timestamp) < 5 ? 1 : 0;
  mission_status = (node_->get_clock()->now().seconds() - mission_update_timestamp < 5) ? mission_status : "";

  DeepOrangeStateSupervisor::updateROSState();

  mobilityMsg.au_state = state;
  auStateMsg.data = state;
  this->pub_states->publish(auStateMsg);  // Additional standard msg for stack side
  this->pub_mobility->publish(mobilityMsg);  // custom deeporange14 msg for DBW Can node
}

void DeepOrangeStateSupervisor::getPhxStatus(const actionlib_msgs::msg::GoalStatusArray &statusMsg)
{
  if (!statusMsg.status_list.empty())
  {
    mppi_status = statusMsg.status_list[0].status;
    // RCLCPP_INFO(node_->get_logger(), "MPPI Status: %d", mppi_status);
  }
}

void DeepOrangeStateSupervisor::updateROSState() {
  switch (state) {
    case AU_0_DEFAULT: {
      prevSt = 0;
      state = AU_1_STARTUP;
      RCLCPP_INFO(node_->get_logger(), "In Default 0 st");
      break;
    }
    case AU_1_STARTUP: {
      mobilityMsg.tql_cmd = 0.0;
      mobilityMsg.tqr_cmd = 0.0;
      mobilityMsg.brkl_cmd = 1.0;
      mobilityMsg.brkr_cmd = 1.0;
      // mission_status = "";
      // RCLCPP_INFO(node_->get_logger(), "In startup State");

      if (raptor_hb_detected) {
        RCLCPP_WARN(node_->get_logger(), "WARN:[AU_1_STARTUP]: RaptorHandshake is established, transitioning to AU_2_IDLE ");
        prevSt = 1;
        state = AU_2_IDLE;
        break;
      }
      else {
        // do nothing , stay in same state
        RCLCPP_WARN(node_->get_logger(), "[AU_1_STARTUP]: Raptor Handshake failed or not established yet");
        break;
      }
    }
    case AU_2_IDLE: {
      mobilityMsg.tql_cmd = 0.0;
      mobilityMsg.tqr_cmd = 0.0;
      mobilityMsg.brkl_cmd = 1.0;
      mobilityMsg.brkr_cmd = 1.0;
      // mission_status = "";
      // RCLCPP_INFO(node_->get_logger(), "In Idle State");
      // RCLCPP_WARN(node_->get_logger(), "In Idle");

      if (!raptor_hb_detected) {
        state = AU_1_STARTUP;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_2_IDLE]: RaptorHandshake failed ");
        break;
      }
      else if (prevSt > 1) {
        //  it has returned from fault of below states, add delay
        if (delay < delay_threshold) {
          delay++;
          break;
        }
        else {
          prevSt = 1;
          break;
        }
      }
      else if (dbw_ros_mode) {
        RCLCPP_WARN(node_->get_logger(), "WARN: [AU_2_IDLE]: Transitioning to AU_3_ROS_EN ");
        prevSt = 2;
        state = AU_3_ROS_MODE_EN;
        break;
      }
      else {
        // do nothing ,stay in same state
        // RCLCPP_WARN(node_->get_logger(), "WARN: [AU_2_IDLE]: RaptorHandshake failed or DBW ros mode disabled");
        break;
      }
    }
    case AU_3_ROS_MODE_EN: {
      mobilityMsg.tql_cmd = 0.0;
      mobilityMsg.tqr_cmd = 0.0;  // Also check from stack if brake_enable command from stack should be true
      mobilityMsg.brkl_cmd = 1.0;
      mobilityMsg.brkr_cmd = 1.0;

      if (!raptor_hb_detected) {
        state = AU_1_STARTUP;
        RCLCPP_ERROR(node_->get_logger(), "ERROR:[AU_3_ROS_MODE_EN]: RaptorHandshake failed ");
        break;
      }
      else if (!dbw_ros_mode) {
        state = AU_2_IDLE;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_3_ROS_MODE_EN]: Out of ROS dbwMode ");
        break;
      }
      else if (stack_fault) {
        state = AU_2_IDLE;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_3_ROS_MODE_EN]: Stack Crashed or failed ");
        break;
      }
      else if (stop_ros) {
        // go backuint8 left_brkPressure
        // stop_ros = false;
        state = AU_2_IDLE;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_3_ROS_MODE_EN]: Stop button is pressed ");
        break;
      }
      else if (mission_status == "globalPlanReady") {
        prevSt = 3;
        state = AU_4_DISENGAGING_BRAKES;
        // mission_status = "";
        RCLCPP_WARN(node_->get_logger(), "[AU_3_ROS_MODE_EN]: Global Plan Ready, transitioning to disengaging brakes ");
        break;
      }
      else if (mppi_status == 1 || mppi_status == 4) {
        prevSt = 3;
        state = AU_4_DISENGAGING_BRAKES;
        // mission_status = "";
        RCLCPP_WARN(node_->get_logger(), "[AU_3_ROS_MODE_EN]: Local Plan Ready, transitioning to disengaging brakes ");
        break;
      }
      else {
        RCLCPP_WARN(node_->get_logger(), "[AU_3_ROS_MODE_EN]: Waiting for globalPlanReady or LocalPlan Ready");
        // do nothing
        break;
      }
    }
    case AU_4_DISENGAGING_BRAKES: {
      mobilityMsg.tql_cmd = tqL_cmd_controller;
      mobilityMsg.tqr_cmd = tqR_cmd_controller;
      mobilityMsg.brkl_cmd = 0.0;
      mobilityMsg.brkr_cmd = 0.0;
      // Also check from stack if brake_enable command is false from stack,
      //  because now global plan is ready and brakes should be disengaged

      if (!raptor_hb_detected) {
        state = AU_1_STARTUP;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_4_DISENGAGING_BRAKES]: RaptorHandshake failed ");
        break;
      }
      else if (!dbw_ros_mode) {
        state = AU_2_IDLE;
        RCLCPP_ERROR(node_->get_logger(), "ERROR:[AU_4_DISENGAGING_BRAKES]: Out of dbwMode ");
        break;
      }
      else if (stack_fault) {
        state = AU_2_IDLE;
        break;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_4_DISENGAGING_BRAKES]: Stack Crashed or failed ");
        break;
      }
      else if (mppi_status == 3) {
        state = AU_3_ROS_MODE_EN;
        break;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_4_DISENGAGING_BRAKES]: Phoenix Stack mission changed to %d", mppi_status);
        break;
      }
      else if (stop_ros) {
        //  go back to idle
        state = AU_2_IDLE;
        // stop_ros = false;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_4_DISENGAGING_BRAKES]: Stop button is pressed ");
        break;
      }
      else if (speed_state == SPEED_STATE_Ready2Move) {
        prevSt = 4;
        state = AU_5_ROS_CONTROLLED;
        RCLCPP_WARN(node_->get_logger(), "[AU_4_DISENGAGING_BRAKES]: Brakes disengaged, transitioning to Ros controlled, about to move ");
        break;
      }
      else {
        // do nothing
        // RCLCPP_WARN(node_->get_logger(), "[AU_4_DISENGAGING_BRAKES]: Waiting for brakes to be disengaged ");
        break;
      }
    }
    case AU_5_ROS_CONTROLLED: {
      mobilityMsg.tql_cmd = tqL_cmd_controller;
      mobilityMsg.tqr_cmd = tqR_cmd_controller;
      mobilityMsg.brkl_cmd = 0.0;
      mobilityMsg.brkr_cmd = 0.0;

      if (!raptor_hb_detected) {
        state = AU_1_STARTUP;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_5_ROS_CONTROLLED]: RaptorHandshake failed ");
        break;
      }
      else if (!dbw_ros_mode) {
        state = AU_2_IDLE;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_5_ROS_CONTROLLED]:Out of dbwMode ");
        break;
      }
      else if (stack_fault) {
        state = AU_2_IDLE;
        RCLCPP_ERROR(node_->get_logger(), "ERROR: [AU_5_ROS_CONTROLLED]: Stack Crashed or failed ");
        break;
      }
      else if (mppi_status == 3) {
        state = AU_3_ROS_MODE_EN;
        break;
        RCLCPP_WARN(node_->get_logger(), "WARN: [AU_5_ROS_CONTROLLED]: Phoenix Stack mission changed to %d", mppi_status);
        break;
      }
      else if (stop_ros) {
        //  go back to idle
        state = AU_2_IDLE;
        // stop_ros = false;
        RCLCPP_ERROR(node_->get_logger(), "Warning: [AU_5_ROS_CONTROLLED]: Stop button is pressed ");
        break;
      }
      else if (mission_status == "MissionCompleted" || mission_status == "MissionCancelled") {
        prevSt = 5;
        state = AU_3_ROS_MODE_EN;
        RCLCPP_INFO(node_->get_logger(), "[AU_5_ROS_CONTROLLED]: Mission Completed or Mission Cancelled going back to AU_3_ROS_MODE_EN");
        break;
      }
      else {
        // do nothing, remain in same state
        // RCLCPP_WARN(node_->get_logger(), "[AU_5_ROS_CONTROLLED]: Mission Executing");
        break;
      }
    }
    default: {
      prevSt = 0;
      RCLCPP_ERROR(node_->get_logger(), " Unknown State, shut down");
      break;
    }
  }
}
}  // namespace deeporange14
