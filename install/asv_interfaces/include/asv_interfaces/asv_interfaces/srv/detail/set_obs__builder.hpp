// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:srv/SetObs.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__SRV__DETAIL__SET_OBS__BUILDER_HPP_
#define ASV_INTERFACES__SRV__DETAIL__SET_OBS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/srv/detail/set_obs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetObs_Request_eso_mode
{
public:
  Init_SetObs_Request_eso_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::asv_interfaces::srv::SetObs_Request eso_mode(::asv_interfaces::srv::SetObs_Request::_eso_mode_type arg)
  {
    msg_.eso_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::srv::SetObs_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::srv::SetObs_Request>()
{
  return asv_interfaces::srv::builder::Init_SetObs_Request_eso_mode();
}

}  // namespace asv_interfaces


namespace asv_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetObs_Response_success
{
public:
  Init_SetObs_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::asv_interfaces::srv::SetObs_Response success(::asv_interfaces::srv::SetObs_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::srv::SetObs_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::srv::SetObs_Response>()
{
  return asv_interfaces::srv::builder::Init_SetObs_Response_success();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__SRV__DETAIL__SET_OBS__BUILDER_HPP_
