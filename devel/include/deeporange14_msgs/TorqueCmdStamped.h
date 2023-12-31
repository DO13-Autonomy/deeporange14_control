// Generated by gencpp from file deeporange14_msgs/TorqueCmdStamped.msg
// DO NOT EDIT!


#ifndef DEEPORANGE14_MSGS_MESSAGE_TORQUECMDSTAMPED_H
#define DEEPORANGE14_MSGS_MESSAGE_TORQUECMDSTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace deeporange14_msgs
{
template <class ContainerAllocator>
struct TorqueCmdStamped_
{
  typedef TorqueCmdStamped_<ContainerAllocator> Type;

  TorqueCmdStamped_()
    : header()
    , tqL_cmd(0.0)
    , tqR_cmd(0.0)  {
    }
  TorqueCmdStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , tqL_cmd(0.0)
    , tqR_cmd(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _tqL_cmd_type;
  _tqL_cmd_type tqL_cmd;

   typedef double _tqR_cmd_type;
  _tqR_cmd_type tqR_cmd;





  typedef boost::shared_ptr< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> const> ConstPtr;

}; // struct TorqueCmdStamped_

typedef ::deeporange14_msgs::TorqueCmdStamped_<std::allocator<void> > TorqueCmdStamped;

typedef boost::shared_ptr< ::deeporange14_msgs::TorqueCmdStamped > TorqueCmdStampedPtr;
typedef boost::shared_ptr< ::deeporange14_msgs::TorqueCmdStamped const> TorqueCmdStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator1> & lhs, const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.tqL_cmd == rhs.tqL_cmd &&
    lhs.tqR_cmd == rhs.tqR_cmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator1> & lhs, const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace deeporange14_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d2b993d9f9c7329f407ee1c3b8d87a65";
  }

  static const char* value(const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd2b993d9f9c7329fULL;
  static const uint64_t static_value2 = 0x407ee1c3b8d87a65ULL;
};

template<class ContainerAllocator>
struct DataType< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "deeporange14_msgs/TorqueCmdStamped";
  }

  static const char* value(const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Message contains timestamped torque commands\n"
"Header header\n"
"\n"
"float64 tqL_cmd\n"
"float64 tqR_cmd\n"
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

  static const char* value(const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.tqL_cmd);
      stream.next(m.tqR_cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TorqueCmdStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::deeporange14_msgs::TorqueCmdStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "tqL_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.tqL_cmd);
    s << indent << "tqR_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.tqR_cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DEEPORANGE14_MSGS_MESSAGE_TORQUECMDSTAMPED_H
