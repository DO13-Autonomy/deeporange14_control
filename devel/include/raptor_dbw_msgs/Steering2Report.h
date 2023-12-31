// Generated by gencpp from file raptor_dbw_msgs/Steering2Report.msg
// DO NOT EDIT!


#ifndef RAPTOR_DBW_MSGS_MESSAGE_STEERING2REPORT_H
#define RAPTOR_DBW_MSGS_MESSAGE_STEERING2REPORT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace raptor_dbw_msgs
{
template <class ContainerAllocator>
struct Steering2Report_
{
  typedef Steering2Report_<ContainerAllocator> Type;

  Steering2Report_()
    : header()
    , vehicle_curvature_actual(0.0)
    , max_torque_driver(0.0)
    , max_torque_motor(0.0)  {
    }
  Steering2Report_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vehicle_curvature_actual(0.0)
    , max_torque_driver(0.0)
    , max_torque_motor(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _vehicle_curvature_actual_type;
  _vehicle_curvature_actual_type vehicle_curvature_actual;

   typedef float _max_torque_driver_type;
  _max_torque_driver_type max_torque_driver;

   typedef float _max_torque_motor_type;
  _max_torque_motor_type max_torque_motor;





  typedef boost::shared_ptr< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> const> ConstPtr;

}; // struct Steering2Report_

typedef ::raptor_dbw_msgs::Steering2Report_<std::allocator<void> > Steering2Report;

typedef boost::shared_ptr< ::raptor_dbw_msgs::Steering2Report > Steering2ReportPtr;
typedef boost::shared_ptr< ::raptor_dbw_msgs::Steering2Report const> Steering2ReportConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vehicle_curvature_actual == rhs.vehicle_curvature_actual &&
    lhs.max_torque_driver == rhs.max_torque_driver &&
    lhs.max_torque_motor == rhs.max_torque_motor;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace raptor_dbw_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4692c5b5a349ffecffb2e0a8766fe010";
  }

  static const char* value(const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4692c5b5a349ffecULL;
  static const uint64_t static_value2 = 0xffb2e0a8766fe010ULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "raptor_dbw_msgs/Steering2Report";
  }

  static const char* value(const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float32 vehicle_curvature_actual # units are 1/m\n"
"\n"
"float32 max_torque_driver # %-Torque\n"
"\n"
"float32 max_torque_motor # %-Torque\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vehicle_curvature_actual);
      stream.next(m.max_torque_driver);
      stream.next(m.max_torque_motor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Steering2Report_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::raptor_dbw_msgs::Steering2Report_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vehicle_curvature_actual: ";
    Printer<float>::stream(s, indent + "  ", v.vehicle_curvature_actual);
    s << indent << "max_torque_driver: ";
    Printer<float>::stream(s, indent + "  ", v.max_torque_driver);
    s << indent << "max_torque_motor: ";
    Printer<float>::stream(s, indent + "  ", v.max_torque_motor);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAPTOR_DBW_MSGS_MESSAGE_STEERING2REPORT_H
