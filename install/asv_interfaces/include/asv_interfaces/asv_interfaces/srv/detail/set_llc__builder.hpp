// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:srv/SetLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__SRV__DETAIL__SET_LLC__BUILDER_HPP_
#define ASV_INTERFACES__SRV__DETAIL__SET_LLC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/srv/detail/set_llc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLlc_Request_llc_mode
{
public:
  Init_SetLlc_Request_llc_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::asv_interfaces::srv::SetLlc_Request llc_mode(::asv_interfaces::srv::SetLlc_Request::_llc_mode_type arg)
  {
    msg_.llc_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::srv::SetLlc_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::srv::SetLlc_Request>()
{
  return asv_interfaces::srv::builder::Init_SetLlc_Request_llc_mode();
}

}  // namespace asv_interfaces


namespace asv_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLlc_Response_success
{
public:
  Init_SetLlc_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::asv_interfaces::srv::SetLlc_Response success(::asv_interfaces::srv::SetLlc_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::srv::SetLlc_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::srv::SetLlc_Response>()
{
  return asv_interfaces::srv::builder::Init_SetLlc_Response_success();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__SRV__DETAIL__SET_LLC__BUILDER_HPP_
