// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "asv_interfaces/msg/detail/xbee_observer__struct.hpp"
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

void XbeeObserver_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asv_interfaces::msg::XbeeObserver(_init);
}

void XbeeObserver_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asv_interfaces::msg::XbeeObserver *>(message_memory);
  typed_message->~XbeeObserver();
}

size_t size_function__XbeeObserver__states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<asv_interfaces::msg::StateNeighbor> *>(untyped_member);
  return member->size();
}

const void * get_const_function__XbeeObserver__states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<asv_interfaces::msg::StateNeighbor> *>(untyped_member);
  return &member[index];
}

void * get_function__XbeeObserver__states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<asv_interfaces::msg::StateNeighbor> *>(untyped_member);
  return &member[index];
}

void fetch_function__XbeeObserver__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const asv_interfaces::msg::StateNeighbor *>(
    get_const_function__XbeeObserver__states(untyped_member, index));
  auto & value = *reinterpret_cast<asv_interfaces::msg::StateNeighbor *>(untyped_value);
  value = item;
}

void assign_function__XbeeObserver__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<asv_interfaces::msg::StateNeighbor *>(
    get_function__XbeeObserver__states(untyped_member, index));
  const auto & value = *reinterpret_cast<const asv_interfaces::msg::StateNeighbor *>(untyped_value);
  item = value;
}

void resize_function__XbeeObserver__states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<asv_interfaces::msg::StateNeighbor> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember XbeeObserver_message_member_array[2] = {
  {
    "counter",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::msg::XbeeObserver, counter),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<asv_interfaces::msg::StateNeighbor>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::msg::XbeeObserver, states),  // bytes offset in struct
    nullptr,  // default value
    size_function__XbeeObserver__states,  // size() function pointer
    get_const_function__XbeeObserver__states,  // get_const(index) function pointer
    get_function__XbeeObserver__states,  // get(index) function pointer
    fetch_function__XbeeObserver__states,  // fetch(index, &value) function pointer
    assign_function__XbeeObserver__states,  // assign(index, value) function pointer
    resize_function__XbeeObserver__states  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers XbeeObserver_message_members = {
  "asv_interfaces::msg",  // message namespace
  "XbeeObserver",  // message name
  2,  // number of fields
  sizeof(asv_interfaces::msg::XbeeObserver),
  XbeeObserver_message_member_array,  // message members
  XbeeObserver_init_function,  // function to initialize message memory (memory has to be allocated)
  XbeeObserver_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t XbeeObserver_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &XbeeObserver_message_members,
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
get_message_type_support_handle<asv_interfaces::msg::XbeeObserver>()
{
  return &::asv_interfaces::msg::rosidl_typesupport_introspection_cpp::XbeeObserver_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asv_interfaces, msg, XbeeObserver)() {
  return &::asv_interfaces::msg::rosidl_typesupport_introspection_cpp::XbeeObserver_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
