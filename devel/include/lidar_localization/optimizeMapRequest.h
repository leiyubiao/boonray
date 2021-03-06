// Generated by gencpp from file lidar_localization/optimizeMapRequest.msg
// DO NOT EDIT!


#ifndef LIDAR_LOCALIZATION_MESSAGE_OPTIMIZEMAPREQUEST_H
#define LIDAR_LOCALIZATION_MESSAGE_OPTIMIZEMAPREQUEST_H


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
struct optimizeMapRequest_
{
  typedef optimizeMapRequest_<ContainerAllocator> Type;

  optimizeMapRequest_()
    {
    }
  optimizeMapRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct optimizeMapRequest_

typedef ::lidar_localization::optimizeMapRequest_<std::allocator<void> > optimizeMapRequest;

typedef boost::shared_ptr< ::lidar_localization::optimizeMapRequest > optimizeMapRequestPtr;
typedef boost::shared_ptr< ::lidar_localization::optimizeMapRequest const> optimizeMapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_localization::optimizeMapRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::lidar_localization::optimizeMapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_localization/optimizeMapRequest";
  }

  static const char* value(const ::lidar_localization::optimizeMapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::lidar_localization::optimizeMapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct optimizeMapRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_localization::optimizeMapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::lidar_localization::optimizeMapRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_LOCALIZATION_MESSAGE_OPTIMIZEMAPREQUEST_H
