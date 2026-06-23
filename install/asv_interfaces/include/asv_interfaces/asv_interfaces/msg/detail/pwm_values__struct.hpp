// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__STRUCT_HPP_
#define ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__asv_interfaces__msg__PwmValues __attribute__((deprecated))
#else
# define DEPRECATED__asv_interfaces__msg__PwmValues __declspec(deprecated)
#endif

namespace asv_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PwmValues_
{
  using Type = PwmValues_<ContainerAllocator>;

  explicit PwmValues_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t_left = 0;
      this->t_righ = 0;
    }
  }

  explicit PwmValues_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t_left = 0;
      this->t_righ = 0;
    }
  }

  // field types and members
  using _t_left_type =
    uint16_t;
  _t_left_type t_left;
  using _t_righ_type =
    uint16_t;
  _t_righ_type t_righ;

  // setters for named parameter idiom
  Type & set__t_left(
    const uint16_t & _arg)
  {
    this->t_left = _arg;
    return *this;
  }
  Type & set__t_righ(
    const uint16_t & _arg)
  {
    this->t_righ = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asv_interfaces::msg::PwmValues_<ContainerAllocator> *;
  using ConstRawPtr =
    const asv_interfaces::msg::PwmValues_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::PwmValues_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::PwmValues_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asv_interfaces__msg__PwmValues
    std::shared_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asv_interfaces__msg__PwmValues
    std::shared_ptr<asv_interfaces::msg::PwmValues_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PwmValues_ & other) const
  {
    if (this->t_left != other.t_left) {
      return false;
    }
    if (this->t_righ != other.t_righ) {
      return false;
    }
    return true;
  }
  bool operator!=(const PwmValues_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PwmValues_

// alias to use template instance with default allocator
using PwmValues =
  asv_interfaces::msg::PwmValues_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__STRUCT_HPP_
