// Generated by gencpp from file xarm_msgs/GetFloat32ListResponse.msg
// DO NOT EDIT!


#ifndef XARM_MSGS_MESSAGE_GETFLOAT32LISTRESPONSE_H
#define XARM_MSGS_MESSAGE_GETFLOAT32LISTRESPONSE_H


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
struct GetFloat32ListResponse_
{
  typedef GetFloat32ListResponse_<ContainerAllocator> Type;

  GetFloat32ListResponse_()
    : ret(0)
    , message()
    , datas()  {
    }
  GetFloat32ListResponse_(const ContainerAllocator& _alloc)
    : ret(0)
    , message(_alloc)
    , datas(_alloc)  {
  (void)_alloc;
    }



   typedef int16_t _ret_type;
  _ret_type ret;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _datas_type;
  _datas_type datas;





  typedef boost::shared_ptr< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetFloat32ListResponse_

typedef ::xarm_msgs::GetFloat32ListResponse_<std::allocator<void> > GetFloat32ListResponse;

typedef boost::shared_ptr< ::xarm_msgs::GetFloat32ListResponse > GetFloat32ListResponsePtr;
typedef boost::shared_ptr< ::xarm_msgs::GetFloat32ListResponse const> GetFloat32ListResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator1> & lhs, const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator2> & rhs)
{
  return lhs.ret == rhs.ret &&
    lhs.message == rhs.message &&
    lhs.datas == rhs.datas;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator1> & lhs, const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xarm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9470c18b0944f559a19d029d524ee6b1";
  }

  static const char* value(const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9470c18b0944f559ULL;
  static const uint64_t static_value2 = 0xa19d029d524ee6b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xarm_msgs/GetFloat32ListResponse";
  }

  static const char* value(const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"int16 ret\n"
"string message\n"
"\n"
"float32[] datas\n"
;
  }

  static const char* value(const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ret);
      stream.next(m.message);
      stream.next(m.datas);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetFloat32ListResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xarm_msgs::GetFloat32ListResponse_<ContainerAllocator>& v)
  {
    s << indent << "ret: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ret);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "datas[]" << std::endl;
    for (size_t i = 0; i < v.datas.size(); ++i)
    {
      s << indent << "  datas[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.datas[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // XARM_MSGS_MESSAGE_GETFLOAT32LISTRESPONSE_H
