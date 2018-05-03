/* Auto-generated by genmsg_cpp for file /home/simon/cs403_ros_workspace/assignments/compsci403_assignment4/msg/ObstacleMsg.msg */
#ifndef COMPSCI403_ASSIGNMENT4_MESSAGE_OBSTACLEMSG_H
#define COMPSCI403_ASSIGNMENT4_MESSAGE_OBSTACLEMSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"

namespace compsci403_assignment4
{
template <class ContainerAllocator>
struct ObstacleMsg_ {
  typedef ObstacleMsg_<ContainerAllocator> Type;

  ObstacleMsg_()
  : header()
  , obstacle_points()
  {
  }

  ObstacleMsg_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , obstacle_points(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  _obstacle_points_type;
  std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  obstacle_points;


  typedef boost::shared_ptr< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator>  const> ConstPtr;
}; // struct ObstacleMsg
typedef  ::compsci403_assignment4::ObstacleMsg_<std::allocator<void> > ObstacleMsg;

typedef boost::shared_ptr< ::compsci403_assignment4::ObstacleMsg> ObstacleMsgPtr;
typedef boost::shared_ptr< ::compsci403_assignment4::ObstacleMsg const> ObstacleMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace compsci403_assignment4

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a29f7b4409b527b2f8154194df18d3b1";
  }

  static const char* value(const  ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa29f7b4409b527b2ULL;
  static const uint64_t static_value2 = 0xf8154194df18d3b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "compsci403_assignment4/ObstacleMsg";
  }

  static const char* value(const  ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
geometry_msgs/Point32[] obstacle_points\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.obstacle_points);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct ObstacleMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::compsci403_assignment4::ObstacleMsg_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "obstacle_points[]" << std::endl;
    for (size_t i = 0; i < v.obstacle_points.size(); ++i)
    {
      s << indent << "  obstacle_points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacle_points[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // COMPSCI403_ASSIGNMENT4_MESSAGE_OBSTACLEMSG_H
