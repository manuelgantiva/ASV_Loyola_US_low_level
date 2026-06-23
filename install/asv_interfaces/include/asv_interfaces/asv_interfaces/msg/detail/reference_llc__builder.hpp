// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asv_interfaces:msg/ReferenceLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__BUILDER_HPP_
#define ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asv_interfaces/msg/detail/reference_llc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asv_interfaces
{

namespace msg
{

namespace builder
{

class Init_ReferenceLlc_u_tar
{
public:
  explicit Init_ReferenceLlc_u_tar(::asv_interfaces::msg::ReferenceLlc & msg)
  : msg_(msg)
  {}
  ::asv_interfaces::msg::ReferenceLlc u_tar(::asv_interfaces::msg::ReferenceLlc::_u_tar_type arg)
  {
    msg_.u_tar = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asv_interfaces::msg::ReferenceLlc msg_;
};

class Init_ReferenceLlc_references
{
public:
  Init_ReferenceLlc_references()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReferenceLlc_u_tar references(::asv_interfaces::msg::ReferenceLlc::_references_type arg)
  {
    msg_.references = std::move(arg);
    return Init_ReferenceLlc_u_tar(msg_);
  }

private:
  ::asv_interfaces::msg::ReferenceLlc msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asv_interfaces::msg::ReferenceLlc>()
{
  return asv_interfaces::msg::builder::Init_ReferenceLlc_references();
}

}  // namespace asv_interfaces

#endif  // ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__BUILDER_HPP_
