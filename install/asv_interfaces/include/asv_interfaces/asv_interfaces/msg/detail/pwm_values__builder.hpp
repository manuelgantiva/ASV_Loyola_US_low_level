// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__BUILDER_HPP_
#define ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/msg/detail/pwm_values__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace msg
{

namespace builder
{

class Init_PwmValues_t_righ
{
public:
  explicit Init_PwmValues_t_righ(::asv_interfaces::msg::PwmValues & msg)
  : msg_(msg)
  {}
  ::asv_interfaces::msg::PwmValues t_righ(::asv_interfaces::msg::PwmValues::_t_righ_type arg)
  {
    msg_.t_righ = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::msg::PwmValues msg_;
};

class Init_PwmValues_t_left
{
public:
  Init_PwmValues_t_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PwmValues_t_righ t_left(::asv_interfaces::msg::PwmValues::_t_left_type arg)
  {
    msg_.t_left = std::move(arg);
    return Init_PwmValues_t_righ(msg_);
  }

private:
  ::asv_interfaces::msg::PwmValues msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::msg::PwmValues>()
{
  return asv_interfaces::msg::builder::Init_PwmValues_t_left();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__BUILDER_HPP_
