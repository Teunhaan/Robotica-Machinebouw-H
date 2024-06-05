// Generated by gencpp from file xarm_msgs/MoveVeloRequest.msg
// DO NOT EDIT!


#ifndef XARM_MSGS_MESSAGE_MOVEVELOREQUEST_H
#define XARM_MSGS_MESSAGE_MOVEVELOREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xarm_msgs
{
template <class ContainerAllocator>
struct MoveVeloRequest_
{
  typedef MoveVeloRequest_<ContainerAllocator> Type;

  MoveVeloRequest_()
    : velocities()
    , jnt_sync(0)
    , coord(0)  {
    }
  MoveVeloRequest_(const ContainerAllocator& _alloc)
    : velocities(_alloc)
    , jnt_sync(0)
    , coord(0)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _velocities_type;
  _velocities_type velocities;

   typedef int16_t _jnt_sync_type;
  _jnt_sync_type jnt_sync;

   typedef int16_t _coord_type;
  _coord_type coord;





  typedef boost::shared_ptr< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MoveVeloRequest_

typedef ::xarm_msgs::MoveVeloRequest_<std::allocator<void> > MoveVeloRequest;

typedef boost::shared_ptr< ::xarm_msgs::MoveVeloRequest > MoveVeloRequestPtr;
typedef boost::shared_ptr< ::xarm_msgs::MoveVeloRequest const> MoveVeloRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator1> & lhs, const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator2> & rhs)
{
  return lhs.velocities == rhs.velocities &&
    lhs.jnt_sync == rhs.jnt_sync &&
    lhs.coord == rhs.coord;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator1> & lhs, const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xarm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "de45efefc7a22e2ad261a65d9d8c2df1";
  }

  static const char* value(const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xde45efefc7a22e2aULL;
  static const uint64_t static_value2 = 0xd261a65d9d8c2df1ULL;
};

template<class ContainerAllocator>
struct DataType< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xarm_msgs/MoveVeloRequest";
  }

  static const char* value(const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# request: command specification for velocity executions.\n"
"# Units:\n"
"#	joint space/angles: radian/s\n"
"#	Cartesian space: mm/s, radian/s.\n"
"\n"
"# velocities: the velocity list of the joints/tcp\n"
"#   For velo_move_joint service: [joint1_velocity, ..., joint7_velocity]\n"
"#   For velo_move_line service: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]\n"
"float32[] velocities\n"
"\n"
"# jnt_sync: this is special for velo_move_joint service, meaning whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no.\n"
"int16 jnt_sync\n"
"\n"
"# coord: this is special for velo_move_line service, meaning whether motion is in tool coordinate(1) or not(0)\n"
"int16 coord\n"
"\n"
;
  }

  static const char* value(const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.velocities);
      stream.next(m.jnt_sync);
      stream.next(m.coord);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveVeloRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xarm_msgs::MoveVeloRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xarm_msgs::MoveVeloRequest_<ContainerAllocator>& v)
  {
    s << indent << "velocities[]" << std::endl;
    for (size_t i = 0; i < v.velocities.size(); ++i)
    {
      s << indent << "  velocities[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.velocities[i]);
    }
    s << indent << "jnt_sync: ";
    Printer<int16_t>::stream(s, indent + "  ", v.jnt_sync);
    s << indent << "coord: ";
    Printer<int16_t>::stream(s, indent + "  ", v.coord);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XARM_MSGS_MESSAGE_MOVEVELOREQUEST_H
