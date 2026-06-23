// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:msg/StateObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__BUILDER_HPP_
#define ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/msg/detail/state_observer__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace msg
{

namespace builder
{

class Init_StateObserver_disturbances
{
public:
  explicit Init_StateObserver_disturbances(::asv_interfaces::msg::StateObserver & msg)
  : msg_(msg)
  {}
  ::asv_interfaces::msg::StateObserver disturbances(::asv_interfaces::msg::StateObserver::_disturbances_type arg)
  {
    msg_.disturbances = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::msg::StateObserver msg_;
};

class Init_StateObserver_velocity
{
public:
  explicit Init_StateObserver_velocity(::asv_interfaces::msg::StateObserver & msg)
  : msg_(msg)
  {}
  Init_StateObserver_disturbances velocity(::asv_interfaces::msg::StateObserver::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_StateObserver_disturbances(msg_);
  }

private:
  ::asv_interfaces::msg::StateObserver msg_;
};

class Init_StateObserver_point
{
public:
  explicit Init_StateObserver_point(::asv_interfaces::msg::StateObserver & msg)
  : msg_(msg)
  {}
  Init_StateObserver_velocity point(::asv_interfaces::msg::StateObserver::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_StateObserver_velocity(msg_);
  }

private:
  ::asv_interfaces::msg::StateObserver msg_;
};

class Init_StateObserver_header
{
public:
  Init_StateObserver_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateObserver_point header(::asv_interfaces::msg::StateObserver::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StateObserver_point(msg_);
  }

private:
  ::asv_interfaces::msg::StateObserver msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::msg::StateObserver>()
{
  return asv_interfaces::msg::builder::Init_StateObserver_header();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__BUILDER_HPP_
