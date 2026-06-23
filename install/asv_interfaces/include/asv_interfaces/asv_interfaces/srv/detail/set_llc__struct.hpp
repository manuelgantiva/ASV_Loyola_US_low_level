// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asv_interfaces:srv/SetLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__SRV__DETAIL__SET_LLC__STRUCT_HPP_
#define ASV_INTERFACES__SRV__DETAIL__SET_LLC__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__asv_interfaces__srv__SetLlc_Request __attribute__((deprecated))
#else
# define DEPRECATED__asv_interfaces__srv__SetLlc_Request __declspec(deprecated)
#endif

namespace asv_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLlc_Request_
{
  using Type = SetLlc_Request_<ContainerAllocator>;

  explicit SetLlc_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->llc_mode = 0;
    }
  }

  explicit SetLlc_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->llc_mode = 0;
    }
  }

  // field types and members
  using _llc_mode_type =
    uint8_t;
  _llc_mode_type llc_mode;

  // setters for named parameter idiom
  Type & set__llc_mode(
    const uint8_t & _arg)
  {
    this->llc_mode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LLC_APM =
    1u;
  static constexpr uint8_t LLC_IFAC =
    2u;
  static constexpr uint8_t LLC_MPC =
    3u;

  // pointer types
  using RawPtr =
    asv_interfaces::srv::SetLlc_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const asv_interfaces::srv::SetLlc_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::srv::SetLlc_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::srv::SetLlc_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asv_interfaces__srv__SetLlc_Request
    std::shared_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asv_interfaces__srv__SetLlc_Request
    std::shared_ptr<asv_interfaces::srv::SetLlc_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLlc_Request_ & other) const
  {
    if (this->llc_mode != other.llc_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLlc_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLlc_Request_

// alias to use template instance with default allocator
using SetLlc_Request =
  asv_interfaces::srv::SetLlc_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetLlc_Request_<ContainerAllocator>::LLC_APM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetLlc_Request_<ContainerAllocator>::LLC_IFAC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetLlc_Request_<ContainerAllocator>::LLC_MPC;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace asv_interfaces


#ifndef _WIN32
# define DEPRECATED__asv_interfaces__srv__SetLlc_Response __attribute__((deprecated))
#else
# define DEPRECATED__asv_interfaces__srv__SetLlc_Response __declspec(deprecated)
#endif

namespace asv_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLlc_Response_
{
  using Type = SetLlc_Response_<ContainerAllocator>;

  explicit SetLlc_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetLlc_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asv_interfaces::srv::SetLlc_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const asv_interfaces::srv::SetLlc_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::srv::SetLlc_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asv_interfaces::srv::SetLlc_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asv_interfaces__srv__SetLlc_Response
    std::shared_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asv_interfaces__srv__SetLlc_Response
    std::shared_ptr<asv_interfaces::srv::SetLlc_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLlc_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLlc_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLlc_Response_

// alias to use template instance with default allocator
using SetLlc_Response =
  asv_interfaces::srv::SetLlc_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace asv_interfaces

namespace asv_interfaces
{

namespace srv
{

struct SetLlc
{
  using Request = asv_interfaces::srv::SetLlc_Request;
  using Response = asv_interfaces::srv::SetLlc_Response;
};

}  // namespace srv

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__SRV__DETAIL__SET_LLC__STRUCT_HPP_
