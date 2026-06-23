// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asv_interfaces/msg/detail/xbee_observer__rosidl_typesupport_introspection_c.h"
#include "asv_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asv_interfaces/msg/detail/xbee_observer__functions.h"
#include "asv_interfaces/msg/detail/xbee_observer__struct.h"


// Include directives for member types
// Member `states`
#include "asv_interfaces/msg/state_neighbor.h"
// Member `states`
#include "asv_interfaces/msg/detail/state_neighbor__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asv_interfaces__msg__XbeeObserver__init(message_memory);
}

void asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_fini_function(void * message_memory)
{
  asv_interfaces__msg__XbeeObserver__fini(message_memory);
}

size_t asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__size_function__XbeeObserver__states(
  const void * untyped_member)
{
  const asv_interfaces__msg__StateNeighbor__Sequence * member =
    (const asv_interfaces__msg__StateNeighbor__Sequence *)(untyped_member);
  return member->size;
}

const void * asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__get_const_function__XbeeObserver__states(
  const void * untyped_member, size_t index)
{
  const asv_interfaces__msg__StateNeighbor__Sequence * member =
    (const asv_interfaces__msg__StateNeighbor__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__get_function__XbeeObserver__states(
  void * untyped_member, size_t index)
{
  asv_interfaces__msg__StateNeighbor__Sequence * member =
    (asv_interfaces__msg__StateNeighbor__Sequence *)(untyped_member);
  return &member->data[index];
}

void asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__fetch_function__XbeeObserver__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const asv_interfaces__msg__StateNeighbor * item =
    ((const asv_interfaces__msg__StateNeighbor *)
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__get_const_function__XbeeObserver__states(untyped_member, index));
  asv_interfaces__msg__StateNeighbor * value =
    (asv_interfaces__msg__StateNeighbor *)(untyped_value);
  *value = *item;
}

void asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__assign_function__XbeeObserver__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  asv_interfaces__msg__StateNeighbor * item =
    ((asv_interfaces__msg__StateNeighbor *)
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__get_function__XbeeObserver__states(untyped_member, index));
  const asv_interfaces__msg__StateNeighbor * value =
    (const asv_interfaces__msg__StateNeighbor *)(untyped_value);
  *item = *value;
}

bool asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__resize_function__XbeeObserver__states(
  void * untyped_member, size_t size)
{
  asv_interfaces__msg__StateNeighbor__Sequence * member =
    (asv_interfaces__msg__StateNeighbor__Sequence *)(untyped_member);
  asv_interfaces__msg__StateNeighbor__Sequence__fini(member);
  return asv_interfaces__msg__StateNeighbor__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_member_array[2] = {
  {
    "counter",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__XbeeObserver, counter),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__XbeeObserver, states),  // bytes offset in struct
    NULL,  // default value
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__size_function__XbeeObserver__states,  // size() function pointer
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__get_const_function__XbeeObserver__states,  // get_const(index) function pointer
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__get_function__XbeeObserver__states,  // get(index) function pointer
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__fetch_function__XbeeObserver__states,  // fetch(index, &value) function pointer
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__assign_function__XbeeObserver__states,  // assign(index, value) function pointer
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__resize_function__XbeeObserver__states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_members = {
  "asv_interfaces__msg",  // message namespace
  "XbeeObserver",  // message name
  2,  // number of fields
  sizeof(asv_interfaces__msg__XbeeObserver),
  asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_member_array,  // message members
  asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_init_function,  // function to initialize message memory (memory has to be allocated)
  asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_type_support_handle = {
  0,
  &asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asv_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asv_interfaces, msg, XbeeObserver)() {
  asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asv_interfaces, msg, StateNeighbor)();
  if (!asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_type_support_handle.typesupport_identifier) {
    asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asv_interfaces__msg__XbeeObserver__rosidl_typesupport_introspection_c__XbeeObserver_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
