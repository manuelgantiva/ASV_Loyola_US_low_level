// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from asv_interfaces:msg/ReferenceLlc.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "asv_interfaces/msg/detail/reference_llc__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace asv_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ReferenceLlc_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asv_interfaces::msg::ReferenceLlc(_init);
}

void ReferenceLlc_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asv_interfaces::msg::ReferenceLlc *>(message_memory);
  typed_message->~ReferenceLlc();
}

size_t size_function__ReferenceLlc__references(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Vector3> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReferenceLlc__references(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Vector3> *>(untyped_member);
  return &member[index];
}

void * get_function__ReferenceLlc__references(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Vector3> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReferenceLlc__references(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Vector3 *>(
    get_const_function__ReferenceLlc__references(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Vector3 *>(untyped_value);
  value = item;
}

void assign_function__ReferenceLlc__references(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Vector3 *>(
    get_function__ReferenceLlc__references(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Vector3 *>(untyped_value);
  item = value;
}

void resize_function__ReferenceLlc__references(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Vector3> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReferenceLlc_message_member_array[2] = {
  {
    "references",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::msg::ReferenceLlc, references),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReferenceLlc__references,  // size() function pointer
    get_const_function__ReferenceLlc__references,  // get_const(index) function pointer
    get_function__ReferenceLlc__references,  // get(index) function pointer
    fetch_function__ReferenceLlc__references,  // fetch(index, &value) function pointer
    assign_function__ReferenceLlc__references,  // assign(index, value) function pointer
    resize_function__ReferenceLlc__references  // resize(index) function pointer
  },
  {
    "u_tar",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Float64>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::msg::ReferenceLlc, u_tar),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReferenceLlc_message_members = {
  "asv_interfaces::msg",  // message namespace
  "ReferenceLlc",  // message name
  2,  // number of fields
  sizeof(asv_interfaces::msg::ReferenceLlc),
  ReferenceLlc_message_member_array,  // message members
  ReferenceLlc_init_function,  // function to initialize message memory (memory has to be allocated)
  ReferenceLlc_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReferenceLlc_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReferenceLlc_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace asv_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<asv_interfaces::msg::ReferenceLlc>()
{
  return &::asv_interfaces::msg::rosidl_typesupport_introspection_cpp::ReferenceLlc_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asv_interfaces, msg, ReferenceLlc)() {
  return &::asv_interfaces::msg::rosidl_typesupport_introspection_cpp::ReferenceLlc_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
