// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "asv_interfaces/msg/detail/pwm_values__struct.hpp"
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

void PwmValues_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asv_interfaces::msg::PwmValues(_init);
}

void PwmValues_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asv_interfaces::msg::PwmValues *>(message_memory);
  typed_message->~PwmValues();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PwmValues_message_member_array[2] = {
  {
    "t_left",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::msg::PwmValues, t_left),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_righ",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::msg::PwmValues, t_righ),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PwmValues_message_members = {
  "asv_interfaces::msg",  // message namespace
  "PwmValues",  // message name
  2,  // number of fields
  sizeof(asv_interfaces::msg::PwmValues),
  PwmValues_message_member_array,  // message members
  PwmValues_init_function,  // function to initialize message memory (memory has to be allocated)
  PwmValues_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PwmValues_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PwmValues_message_members,
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
get_message_type_support_handle<asv_interfaces::msg::PwmValues>()
{
  return &::asv_interfaces::msg::rosidl_typesupport_introspection_cpp::PwmValues_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asv_interfaces, msg, PwmValues)() {
  return &::asv_interfaces::msg::rosidl_typesupport_introspection_cpp::PwmValues_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
