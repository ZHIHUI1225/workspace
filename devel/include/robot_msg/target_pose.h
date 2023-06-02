// Generated by gencpp from file robot_msg/target_pose.msg
// DO NOT EDIT!


#ifndef ROBOT_MSG_MESSAGE_TARGET_POSE_H
#define ROBOT_MSG_MESSAGE_TARGET_POSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace robot_msg
{
template <class ContainerAllocator>
struct target_pose_
{
  typedef target_pose_<ContainerAllocator> Type;

  target_pose_()
    : ID(0)
    , position()  {
    }
  target_pose_(const ContainerAllocator& _alloc)
    : ID(0)
    , position(_alloc)  {
  (void)_alloc;
    }



   typedef int8_t _ID_type;
  _ID_type ID;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;





  typedef boost::shared_ptr< ::robot_msg::target_pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msg::target_pose_<ContainerAllocator> const> ConstPtr;

}; // struct target_pose_

typedef ::robot_msg::target_pose_<std::allocator<void> > target_pose;

typedef boost::shared_ptr< ::robot_msg::target_pose > target_posePtr;
typedef boost::shared_ptr< ::robot_msg::target_pose const> target_poseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msg::target_pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msg::target_pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_msg::target_pose_<ContainerAllocator1> & lhs, const ::robot_msg::target_pose_<ContainerAllocator2> & rhs)
{
  return lhs.ID == rhs.ID &&
    lhs.position == rhs.position;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_msg::target_pose_<ContainerAllocator1> & lhs, const ::robot_msg::target_pose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robot_msg::target_pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msg::target_pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msg::target_pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msg::target_pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msg::target_pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msg::target_pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msg::target_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "90b8db49b6b04a35daa272b200b90cf3";
  }

  static const char* value(const ::robot_msg::target_pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x90b8db49b6b04a35ULL;
  static const uint64_t static_value2 = 0xdaa272b200b90cf3ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msg::target_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msg/target_pose";
  }

  static const char* value(const ::robot_msg::target_pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msg::target_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 ID\n"
"geometry_msgs/Point position\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::robot_msg::target_pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msg::target_pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ID);
      stream.next(m.position);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct target_pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msg::target_pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msg::target_pose_<ContainerAllocator>& v)
  {
    s << indent << "ID: ";
    Printer<int8_t>::stream(s, indent + "  ", v.ID);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSG_MESSAGE_TARGET_POSE_H