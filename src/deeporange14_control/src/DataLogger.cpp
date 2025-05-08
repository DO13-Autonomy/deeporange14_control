/*
    A class definition to automatically initiate Rosbag record (for certain
    topics) and CAN data logging (for all CAN buses together). Data logging
    starts (if not already happening) if the Raptor ECU is in a system state
    from [SS2_WAKE, SS8_NOMINALOP]. The data logging (rosbag and CAN log) is
    stopped if the system state becomes SS_31 or an error state (>= 200) is
    reached.
*/
#include <deeporange14_control/DataLogger.h>

namespace deeporange14
{
DataLogger::DataLogger(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // Obtain ros::Subscriber object
  // TODO - set QOS to match TCPNoDelay
  sub_raptor_ = node->create_subscription<deeporange14_msgs::msg::RaptorState>(
    topic_ns + std::string("/raptor_state"),
    10,
    std::bind(&DataLogger::recordRosbagAndCANlog, this, std::placeholders::_1));

  // Initialize recording state to false
  isRecording = false;

  kill_timer = 400;  // at 50 Hz, 400 should be 8 seconds
  logging_counter = 200;

  node->declare_parameter("/log_status", rclcpp::PARAMETER_INTEGER);
}

DataLogger::~DataLogger() {}

void DataLogger::recordRosbagAndCANlog(const deeporange14_msgs::msg::RaptorState & msg)
{
  if (!isRecording && msg.system_state == 6) {
    node_->set_parameter(rclcpp::Parameter("/log_status", 1));
    RCLCPP_INFO(node_->get_logger(), "Entered Data logger");

    // Obtaining timestamp (EST) to name ROS bag and CAN dump
    // this uses the UTC clock with microsecond resultion from the Boost::posix_time package
    boost::posix_time::ptime my_posix_time(boost::posix_time::microsec_clock::universal_time());
    typedef boost::date_time::local_adjustor<boost::posix_time::ptime, -5, boost::posix_time::us_dst> us_eastern;
    my_posix_time = us_eastern::utc_to_local(my_posix_time);  // Conversion from UTC to EST (Clemson, South Carolina)
    std::string iso_time_str = boost::posix_time::to_iso_string(my_posix_time);

    // construct the file name by extracting date/time from the ISO time-string
    std::string file_name_header = std::string("do14_") + iso_time_str.substr(0, 4) +
      std::string("-") + iso_time_str.substr(4, 2) + std::string("-") + iso_time_str.substr(6, 2) +
      std::string("_") + iso_time_str.substr(9, 2) + std::string("-") +
      iso_time_str.substr(11, 2) + std::string("-") + iso_time_str.substr(13, 2);
    can_log_name = file_name_header + std::string(".CAN.log");
    ros_bag_name = file_name_header + std::string(".ROS.bag");
    std::string logs_dir = std::string("~/do14_logs/");
    can_log_name = logs_dir + can_log_name;
    ros_bag_name = logs_dir + ros_bag_name;
    std::cout << "ISO Time: " << iso_time_str << std::endl;
    std::cout << "CAN log name: " << can_log_name << std::endl;
    std::cout << "ROS bag name: " << ros_bag_name << std::endl;

    // Record CAN log for all CAN buses combined
    system(("candump any -ta >" + can_log_name + " &").c_str());

    // Record ROS bags for relevant topics only
    // TODO: this line is much too long, can it be made shorter (and easier to read/understand)?
    system(("rosbag record -e '(.*)cmd_mobility(.*)' -e '(.*)mission_status(.*)' -e '(.*)cmd_vel_reprojected(.*)' -e '(.*)cmd_trq(.*)' -e '(.*)pid_components(.*)' -e '(.*)remapping_state(.*)' -e '(.*)brake_command(.*)' -e '(.*)gps(.*)' -e '(.*)pose(.*)' -e '(.*)cmd_vel(.*)' -e '(.*)/novatel/oem7(.*)' -e (.*)local_planner_and_controller(.*)' -e '(.*)tf(.*)' -x '(.*)approach_object(.*)' -x '(.*)approach_object_behavior(.*)' -x '(.*)global_costmap(.*)' -e '(.*)global_planner(.*)' -x '(.*)goto_object_behavior(.*)' -x '(.*)novatel/oem7(.*)'  -x '(.*)local_costmap(.*)' -x '(.*)omnigraph(.*)' -x '(.*)parameter_updates(.*)' -x '(.*)point_cloud_pipeline(.*)' -x '(.*)point_cloud_cache(.*)' -x '(.*)local_planner(.*)'  -x '(.*)grid(.*)' -x '(.*)center_lidar(.*)' -x '(.*)status(.*)' -x '(.*)server_status(.*)'  -x '(.*)parameter_descriptions(.*)'  -e '(.*)odom(.*)' -x '(.*)navigation_manager(.*)' -O " + ros_bag_name + " __name:=rosbag_recording &").c_str());

    // Update recording state
    isRecording = true;

    RCLCPP_INFO(node_->get_logger(), "Started data recording");
    // RCLCPP_INFO(node_->get_logger(), "Current System State = %d", msg->system_state);
  }
  if (isRecording && msg.system_state == 32 || msg.system_state >= 98) {
    // Wait for timer before killing data logging to capture data in case of system error
    if (kill_timer > 0) {
      kill_timer--;
      return;
    } else {
      // Kill rosbag record and CAN logging
      system("rosnode kill /rosbag_recording");
      system("killall -SIGKILL candump");

      // Update recording state
      isRecording = false;
      kill_timer = 400;  // reinitialized for next recording
      RCLCPP_INFO(node_->get_logger(), "Rosbag record and CAN dump killed");
      node_->set_parameter(rclcpp::Parameter("/log_status", 0));
    }
  } else {
    // DO NOTHING
  }
}

void DataLogger::monitorFileSize(const std::string & can_file, const std::string & ros_bag)
{
  if (logging_counter == 100) {
    // Check for file size after every 2 second
    can_log_size = system(("stat -c %s " + can_file + " &").c_str());
    ros_bag_size = system(("stat -c %s " + ros_bag + ".active" + " &").c_str());
    std::cout << "can log size: " << can_log_size << std::endl;
    std::cout << "ros bag size: " << ros_bag_size << std::endl;
  }

  if (logging_counter == 0) {
    // Check for file size again after every 4 second
    curr_ros_bag_size = system(("stat -c %s " + ros_bag + ".active" + " &").c_str());
    curr_can_log_size = system(("stat -c %s " + can_file + " &").c_str());

    if (curr_ros_bag_size - ros_bag_size == 0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ROS bag size not increasing. No new ROS data is being recorded!");
      node_->set_parameter(rclcpp::Parameter("/log_status", 0));
    }

    if (curr_can_log_size - can_log_size == 0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "CAN log file size not increasing. No new CAN data is being recorded!");
      node_->set_parameter(rclcpp::Parameter("/log_status", 0));
    }

    logging_counter = 200;  // reinitialized for next recording
  }
  logging_counter--;
}
}  // namespace deeporange14
