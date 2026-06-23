// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__BUILDER_HPP_
#define ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/msg/detail/xbee_observer__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace msg
{

namespace builder
{

class Init_XbeeObserver_states
{
public:
  explicit Init_XbeeObserver_states(::asv_interfaces::msg::XbeeObserver & msg)
  : msg_(msg)
  {}
  ::asv_interfaces::msg::XbeeObserver states(::asv_interfaces::msg::XbeeObserver::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::msg::XbeeObserver msg_;
};

class Init_XbeeObserver_counter
{
public:
  Init_XbeeObserver_counter()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_XbeeObserver_states counter(::asv_interfaces::msg::XbeeObserver::_counter_type arg)
  {
    msg_.counter = std::move(arg);
    return Init_XbeeObserver_states(msg_);
  }

private:
  ::asv_interfaces::msg::XbeeObserver msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::msg::XbeeObserver>()
{
  return asv_interfaces::msg::builder::Init_XbeeObserver_counter();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__BUILDER_HPP_
