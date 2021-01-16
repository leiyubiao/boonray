// Generated by gencpp from file lidar_localization/optimizeMapResponse.msg
// DO NOT EDIT!


#ifndef LIDAR_LOCALIZATION_MESSAGE_OPTIMIZEMAPRESPONSE_H
#define LIDAR_LOCALIZATION_MESSAGE_OPTIMIZEMAPRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_localization
{
template <class ContainerAllocator>
struct optimizeMapResponse_
{
  typedef optimizeMapResponse_<ContainerAllocator> Type;

  optimizeMapResponse_()
    : succeed(false)  {
    }
  optimizeMapResponse_(const ContainerAllocator& _alloc)
    : succeed(false)  {
  (void)_alloc;
    }



   typedef uint8_t _succeed_type;
  _succeed_type succeed;





  typedef boost::shared_ptr< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> const> ConstPtr;

}; // struct optimizeMapResponse_

typedef ::lidar_localization::optimizeMapResponse_<std::allocator<void> > optimizeMapResponse;

typedef boost::shared_ptr< ::lidar_localization::optimizeMapResponse > optimizeMapResponsePtr;
typedef boost::shared_ptr< ::lidar_localization::optimizeMapResponse const> optimizeMapResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_localization::optimizeMapResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lidar_localization

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d9c3b918a0afafe09791ef8d7853918";
  }

  static const char* value(const ::lidar_localization::optimizeMapResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d9c3b918a0afafeULL;
  static const uint64_t static_value2 = 0x09791ef8d7853918ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_localization/optimizeMapResponse";
  }

  static const char* value(const ::lidar_localization::optimizeMapResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool succeed\n\
";
  }

  static const char* value(const ::lidar_localization::optimizeMapResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.succeed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct optimizeMapResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_localization::optimizeMapResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_localization::optimizeMapResponse_<ContainerAllocator>& v)
  {
    s << indent << "succeed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.succeed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_LOCALIZATION_MESSAGE_OPTIMIZEMAPRESPONSE_H
