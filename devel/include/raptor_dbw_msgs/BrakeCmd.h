// Generated by gencpp from file raptor_dbw_msgs/BrakeCmd.msg
// DO NOT EDIT!


#ifndef RAPTOR_DBW_MSGS_MESSAGE_BRAKECMD_H
#define RAPTOR_DBW_MSGS_MESSAGE_BRAKECMD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <raptor_dbw_msgs/ActuatorControlMode.h>

namespace raptor_dbw_msgs
{
template <class ContainerAllocator>
struct BrakeCmd_
{
  typedef BrakeCmd_<ContainerAllocator> Type;

  BrakeCmd_()
    : pedal_cmd(0.0)
    , enable(false)
    , rolling_counter(0)
    , torque_cmd(0.0)
    , decel_limit(0.0)
    , control_type()
    , decel_negative_jerk_limit(0.0)  {
    }
  BrakeCmd_(const ContainerAllocator& _alloc)
    : pedal_cmd(0.0)
    , enable(false)
    , rolling_counter(0)
    , torque_cmd(0.0)
    , decel_limit(0.0)
    , control_type(_alloc)
    , decel_negative_jerk_limit(0.0)  {
  (void)_alloc;
    }



   typedef float _pedal_cmd_type;
  _pedal_cmd_type pedal_cmd;

   typedef uint8_t _enable_type;
  _enable_type enable;

   typedef uint8_t _rolling_counter_type;
  _rolling_counter_type rolling_counter;

   typedef float _torque_cmd_type;
  _torque_cmd_type torque_cmd;

   typedef float _decel_limit_type;
  _decel_limit_type decel_limit;

   typedef  ::raptor_dbw_msgs::ActuatorControlMode_<ContainerAllocator>  _control_type_type;
  _control_type_type control_type;

   typedef float _decel_negative_jerk_limit_type;
  _decel_negative_jerk_limit_type decel_negative_jerk_limit;





  typedef boost::shared_ptr< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> const> ConstPtr;

}; // struct BrakeCmd_

typedef ::raptor_dbw_msgs::BrakeCmd_<std::allocator<void> > BrakeCmd;

typedef boost::shared_ptr< ::raptor_dbw_msgs::BrakeCmd > BrakeCmdPtr;
typedef boost::shared_ptr< ::raptor_dbw_msgs::BrakeCmd const> BrakeCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator2> & rhs)
{
  return lhs.pedal_cmd == rhs.pedal_cmd &&
    lhs.enable == rhs.enable &&
    lhs.rolling_counter == rhs.rolling_counter &&
    lhs.torque_cmd == rhs.torque_cmd &&
    lhs.decel_limit == rhs.decel_limit &&
    lhs.control_type == rhs.control_type &&
    lhs.decel_negative_jerk_limit == rhs.decel_negative_jerk_limit;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace raptor_dbw_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4d6dead8aa3923674dc63b7884f45810";
  }

  static const char* value(const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4d6dead8aa392367ULL;
  static const uint64_t static_value2 = 0x4dc63b7884f45810ULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "raptor_dbw_msgs/BrakeCmd";
  }

  static const char* value(const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Brake pedal (%)\n"
"float32 pedal_cmd\n"
"\n"
"# Enable\n"
"bool enable\n"
"\n"
"# Watchdog counter (optional)\n"
"uint8 rolling_counter\n"
"\n"
"float32 torque_cmd # %-torque \n"
"float32 decel_limit # m/s^2\n"
"\n"
"ActuatorControlMode control_type\n"
"\n"
"float32 decel_negative_jerk_limit # m/s^3\n"
"================================================================================\n"
"MSG: raptor_dbw_msgs/ActuatorControlMode\n"
"uint8 value\n"
"\n"
"uint8 open_loop = 0\n"
"uint8 closed_loop_actuator = 1\n"
"uint8 closed_loop_vehicle = 2\n"
"uint8 none = 255\n"
;
  }

  static const char* value(const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pedal_cmd);
      stream.next(m.enable);
      stream.next(m.rolling_counter);
      stream.next(m.torque_cmd);
      stream.next(m.decel_limit);
      stream.next(m.control_type);
      stream.next(m.decel_negative_jerk_limit);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BrakeCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::raptor_dbw_msgs::BrakeCmd_<ContainerAllocator>& v)
  {
    s << indent << "pedal_cmd: ";
    Printer<float>::stream(s, indent + "  ", v.pedal_cmd);
    s << indent << "enable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enable);
    s << indent << "rolling_counter: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rolling_counter);
    s << indent << "torque_cmd: ";
    Printer<float>::stream(s, indent + "  ", v.torque_cmd);
    s << indent << "decel_limit: ";
    Printer<float>::stream(s, indent + "  ", v.decel_limit);
    s << indent << "control_type: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::ActuatorControlMode_<ContainerAllocator> >::stream(s, indent + "  ", v.control_type);
    s << indent << "decel_negative_jerk_limit: ";
    Printer<float>::stream(s, indent + "  ", v.decel_negative_jerk_limit);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAPTOR_DBW_MSGS_MESSAGE_BRAKECMD_H
