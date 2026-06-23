// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asv_interfaces:msg/StateObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__STRUCT_HPP_
#define ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'disturbances'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__asv_interfaces__msg__StateObserver __attribute__((deprecated))
#else
# define DEPRECATED__asv_interfaces__msg__StateObserver __declspec(deprecated)
#endif

namespace asv_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateObserver_
{
  using Type = StateObserver_<ContainerAllocator>;

  explicit StateObserver_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    point(_init),
    velocity(_init),
    disturbances(_init)
  {
    (void)_init;
  }

  explicit StateObserver_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    point(_alloc, _init),
    velocity(_alloc, _init),
    disturbances(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _point_type point;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _disturbances_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _disturbances_type disturbances;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
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
  Type & set__disturbances(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->disturbances = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asv_interfaces::msg::StateObserver_<ContainerAllocator> *;
  using ConstRawPtr =
    const asv_interfaces::msg::StateObserver_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::StateObserver_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::StateObserver_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asv_interfaces__msg__StateObserver
    std::shared_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asv_interfaces__msg__StateObserver
    std::shared_ptr<asv_interfaces::msg::StateObserver_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateObserver_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->point != other.point) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->disturbances != other.disturbances) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateObserver_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateObserver_

// alias to use template instance with default allocator
using StateObserver =
  asv_interfaces::msg::StateObserver_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__STRUCT_HPP_
