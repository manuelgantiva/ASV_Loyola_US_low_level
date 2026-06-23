// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asv_interfaces:msg/StateNeighbor.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__STRUCT_HPP_
#define ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__asv_interfaces__msg__StateNeighbor __attribute__((deprecated))
#else
# define DEPRECATED__asv_interfaces__msg__StateNeighbor __declspec(deprecated)
#endif

namespace asv_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateNeighbor_
{
  using Type = StateNeighbor_<ContainerAllocator>;

  explicit StateNeighbor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
      this->msg_from = 0ll;
    }
  }

  explicit StateNeighbor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : id(_alloc),
    point(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
      this->msg_from = 0ll;
    }
  }

  // field types and members
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_type id;
  using _point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _point_type point;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _msg_from_type =
    int64_t;
  _msg_from_type msg_from;

  // setters for named parameter idiom
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->point = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__msg_from(
    const int64_t & _arg)
  {
    this->msg_from = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asv_interfaces::msg::StateNeighbor_<ContainerAllocator> *;
  using ConstRawPtr =
    const asv_interfaces::msg::StateNeighbor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::StateNeighbor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::StateNeighbor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asv_interfaces__msg__StateNeighbor
    std::shared_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asv_interfaces__msg__StateNeighbor
    std::shared_ptr<asv_interfaces::msg::StateNeighbor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateNeighbor_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->point != other.point) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->msg_from != other.msg_from) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateNeighbor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateNeighbor_

// alias to use template instance with default allocator
using StateNeighbor =
  asv_interfaces::msg::StateNeighbor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__STRUCT_HPP_
