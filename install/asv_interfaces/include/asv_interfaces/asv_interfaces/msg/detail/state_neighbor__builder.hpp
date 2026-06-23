// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:msg/StateNeighbor.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__BUILDER_HPP_
#define ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/msg/detail/state_neighbor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace msg
{

namespace builder
{

class Init_StateNeighbor_msg_from
{
public:
  explicit Init_StateNeighbor_msg_from(::asv_interfaces::msg::StateNeighbor & msg)
  : msg_(msg)
  {}
  ::asv_interfaces::msg::StateNeighbor msg_from(::asv_interfaces::msg::StateNeighbor::_msg_from_type arg)
  {
    msg_.msg_from = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::msg::StateNeighbor msg_;
};

class Init_StateNeighbor_velocity
{
public:
  explicit Init_StateNeighbor_velocity(::asv_interfaces::msg::StateNeighbor & msg)
  : msg_(msg)
  {}
  Init_StateNeighbor_msg_from velocity(::asv_interfaces::msg::StateNeighbor::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_StateNeighbor_msg_from(msg_);
  }

private:
  ::asv_interfaces::msg::StateNeighbor msg_;
};

class Init_StateNeighbor_point
{
public:
  explicit Init_StateNeighbor_point(::asv_interfaces::msg::StateNeighbor & msg)
  : msg_(msg)
  {}
  Init_StateNeighbor_velocity point(::asv_interfaces::msg::StateNeighbor::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_StateNeighbor_velocity(msg_);
  }

private:
  ::asv_interfaces::msg::StateNeighbor msg_;
};

class Init_StateNeighbor_id
{
public:
  Init_StateNeighbor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateNeighbor_point id(::asv_interfaces::msg::StateNeighbor::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_StateNeighbor_point(msg_);
  }

private:
  ::asv_interfaces::msg::StateNeighbor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::msg::StateNeighbor>()
{
  return asv_interfaces::msg::builder::Init_StateNeighbor_id();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__BUILDER_HPP_
