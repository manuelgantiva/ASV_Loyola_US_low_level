// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asv_interfaces:msg/ReferenceLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__STRUCT_HPP_
#define ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'references'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'u_tar'
#include "std_msgs/msg/detail/float64__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__asv_interfaces__msg__ReferenceLlc __attribute__((deprecated))
#else
# define DEPRECATED__asv_interfaces__msg__ReferenceLlc __declspec(deprecated)
#endif

namespace asv_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReferenceLlc_
{
  using Type = ReferenceLlc_<ContainerAllocator>;

  explicit ReferenceLlc_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : u_tar(_init)
  {
    (void)_init;
  }

  explicit ReferenceLlc_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : u_tar(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _references_type =
    std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>>;
  _references_type references;
  using _u_tar_type =
    std_msgs::msg::Float64_<ContainerAllocator>;
  _u_tar_type u_tar;

  // setters for named parameter idiom
  Type & set__references(
    const std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>> & _arg)
  {
    this->references = _arg;
    return *this;
  }
  Type & set__u_tar(
    const std_msgs::msg::Float64_<ContainerAllocator> & _arg)
  {
    this->u_tar = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asv_interfaces::msg::ReferenceLlc_<ContainerAllocator> *;
  using ConstRawPtr =
    const asv_interfaces::msg::ReferenceLlc_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::ReferenceLlc_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::msg::ReferenceLlc_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asv_interfaces__msg__ReferenceLlc
    std::shared_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asv_interfaces__msg__ReferenceLlc
    std::shared_ptr<asv_interfaces::msg::ReferenceLlc_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReferenceLlc_ & other) const
  {
    if (this->references != other.references) {
      return false;
    }
    if (this->u_tar != other.u_tar) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReferenceLlc_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReferenceLlc_

// alias to use template instance with default allocator
using ReferenceLlc =
  asv_interfaces::msg::ReferenceLlc_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__STRUCT_HPP_
