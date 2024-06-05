// Generated by gencpp from file depthai_ros_msgs/SpatialDetection.msg
// DO NOT EDIT!


#ifndef DEPTHAI_ROS_MSGS_MESSAGE_SPATIALDETECTION_H
#define DEPTHAI_ROS_MSGS_MESSAGE_SPATIALDETECTION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <vision_msgs/ObjectHypothesis.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Point.h>

namespace depthai_ros_msgs
{
template <class ContainerAllocator>
struct SpatialDetection_
{
  typedef SpatialDetection_<ContainerAllocator> Type;

  SpatialDetection_()
    : results()
    , bbox()
    , position()
    , is_tracking(false)
    , tracking_id()  {
    }
  SpatialDetection_(const ContainerAllocator& _alloc)
    : results(_alloc)
    , bbox(_alloc)
    , position(_alloc)
    , is_tracking(false)
    , tracking_id(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::vision_msgs::ObjectHypothesis_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::vision_msgs::ObjectHypothesis_<ContainerAllocator> >> _results_type;
  _results_type results;

   typedef  ::vision_msgs::BoundingBox2D_<ContainerAllocator>  _bbox_type;
  _bbox_type bbox;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef uint8_t _is_tracking_type;
  _is_tracking_type is_tracking;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _tracking_id_type;
  _tracking_id_type tracking_id;





  typedef boost::shared_ptr< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> const> ConstPtr;

}; // struct SpatialDetection_

typedef ::depthai_ros_msgs::SpatialDetection_<std::allocator<void> > SpatialDetection;

typedef boost::shared_ptr< ::depthai_ros_msgs::SpatialDetection > SpatialDetectionPtr;
typedef boost::shared_ptr< ::depthai_ros_msgs::SpatialDetection const> SpatialDetectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator1> & lhs, const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator2> & rhs)
{
  return lhs.results == rhs.results &&
    lhs.bbox == rhs.bbox &&
    lhs.position == rhs.position &&
    lhs.is_tracking == rhs.is_tracking &&
    lhs.tracking_id == rhs.tracking_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator1> & lhs, const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace depthai_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "503c7980b555f0fd79e92d14cb9ac446";
  }

  static const char* value(const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x503c7980b555f0fdULL;
  static const uint64_t static_value2 = 0x79e92d14cb9ac446ULL;
};

template<class ContainerAllocator>
struct DataType< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "depthai_ros_msgs/SpatialDetection";
  }

  static const char* value(const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"# Class probabilities\n"
"vision_msgs/ObjectHypothesis[] results\n"
"\n"
"# 2D bounding box surrounding the object.\n"
"vision_msgs/BoundingBox2D bbox\n"
"\n"
"# Center of the detected object in meters \n"
"geometry_msgs/Point position\n"
"\n"
"# If true, this message contains object tracking information.\n"
"bool is_tracking\n"
"\n"
"# ID used for consistency across multiple detection messages. This value will\n"
"# likely differ from the id field set in each individual ObjectHypothesis.\n"
"# If you set this field, be sure to also set is_tracking to True.\n"
"string tracking_id\n"
"================================================================================\n"
"MSG: vision_msgs/ObjectHypothesis\n"
"# An object hypothesis that contains no position information.\n"
"\n"
"# The unique numeric ID of object detected. To get additional information about\n"
"#   this ID, such as its human-readable name, listeners should perform a lookup\n"
"#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\n"
"int64 id\n"
"\n"
"# The probability or confidence value of the detected object. By convention,\n"
"#   this value should lie in the range [0-1].\n"
"float64 score\n"
"================================================================================\n"
"MSG: vision_msgs/BoundingBox2D\n"
"# A 2D bounding box that can be rotated about its center.\n"
"# All dimensions are in pixels, but represented using floating-point\n"
"#   values to allow sub-pixel precision. If an exact pixel crop is required\n"
"#   for a rotated bounding box, it can be calculated using Bresenham's line\n"
"#   algorithm.\n"
"\n"
"# The 2D position (in pixels) and orientation of the bounding box center.\n"
"geometry_msgs/Pose2D center\n"
"\n"
"# The size (in pixels) of the bounding box surrounding the object relative\n"
"#   to the pose of its center.\n"
"float64 size_x\n"
"float64 size_y\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.results);
      stream.next(m.bbox);
      stream.next(m.position);
      stream.next(m.is_tracking);
      stream.next(m.tracking_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpatialDetection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::depthai_ros_msgs::SpatialDetection_<ContainerAllocator>& v)
  {
    s << indent << "results[]" << std::endl;
    for (size_t i = 0; i < v.results.size(); ++i)
    {
      s << indent << "  results[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::vision_msgs::ObjectHypothesis_<ContainerAllocator> >::stream(s, indent + "    ", v.results[i]);
    }
    s << indent << "bbox: ";
    s << std::endl;
    Printer< ::vision_msgs::BoundingBox2D_<ContainerAllocator> >::stream(s, indent + "  ", v.bbox);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "is_tracking: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_tracking);
    s << indent << "tracking_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.tracking_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DEPTHAI_ROS_MSGS_MESSAGE_SPATIALDETECTION_H
