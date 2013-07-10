/* Auto-generated by genmsg_cpp for file /home/annal/ros_workspace/ar_marker_tools/marker_detect/msg/MarkerXYs.msg */
#ifndef MARKER_DETECT_MESSAGE_MARKERXYS_H
#define MARKER_DETECT_MESSAGE_MARKERXYS_H
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

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

namespace marker_detect
{
template <class ContainerAllocator>
struct MarkerXYs_ {
  typedef MarkerXYs_<ContainerAllocator> Type;

  MarkerXYs_()
  : ids()
  , xys()
  , oris()
  {
  }

  MarkerXYs_(const ContainerAllocator& _alloc)
  : ids(_alloc)
  , xys(_alloc)
  , oris(_alloc)
  {
  }

  typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _ids_type;
  std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  ids;

  typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _xys_type;
  std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  xys;

  typedef std::vector< ::geometry_msgs::Quaternion_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Quaternion_<ContainerAllocator> >::other >  _oris_type;
  std::vector< ::geometry_msgs::Quaternion_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Quaternion_<ContainerAllocator> >::other >  oris;


  typedef boost::shared_ptr< ::marker_detect::MarkerXYs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::marker_detect::MarkerXYs_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MarkerXYs
typedef  ::marker_detect::MarkerXYs_<std::allocator<void> > MarkerXYs;

typedef boost::shared_ptr< ::marker_detect::MarkerXYs> MarkerXYsPtr;
typedef boost::shared_ptr< ::marker_detect::MarkerXYs const> MarkerXYsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::marker_detect::MarkerXYs_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::marker_detect::MarkerXYs_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace marker_detect

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::marker_detect::MarkerXYs_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::marker_detect::MarkerXYs_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::marker_detect::MarkerXYs_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c07f000d1086329eaac495a4e5e32e3c";
  }

  static const char* value(const  ::marker_detect::MarkerXYs_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc07f000d1086329eULL;
  static const uint64_t static_value2 = 0xaac495a4e5e32e3cULL;
};

template<class ContainerAllocator>
struct DataType< ::marker_detect::MarkerXYs_<ContainerAllocator> > {
  static const char* value() 
  {
    return "marker_detect/MarkerXYs";
  }

  static const char* value(const  ::marker_detect::MarkerXYs_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::marker_detect::MarkerXYs_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint32[] ids\n\
geometry_msgs/Point[] xys\n\
geometry_msgs/Quaternion[] oris\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::marker_detect::MarkerXYs_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::marker_detect::MarkerXYs_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ids);
    stream.next(m.xys);
    stream.next(m.oris);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MarkerXYs_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::marker_detect::MarkerXYs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::marker_detect::MarkerXYs_<ContainerAllocator> & v) 
  {
    s << indent << "ids[]" << std::endl;
    for (size_t i = 0; i < v.ids.size(); ++i)
    {
      s << indent << "  ids[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.ids[i]);
    }
    s << indent << "xys[]" << std::endl;
    for (size_t i = 0; i < v.xys.size(); ++i)
    {
      s << indent << "  xys[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.xys[i]);
    }
    s << indent << "oris[]" << std::endl;
    for (size_t i = 0; i < v.oris.size(); ++i)
    {
      s << indent << "  oris[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "    ", v.oris[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // MARKER_DETECT_MESSAGE_MARKERXYS_H

