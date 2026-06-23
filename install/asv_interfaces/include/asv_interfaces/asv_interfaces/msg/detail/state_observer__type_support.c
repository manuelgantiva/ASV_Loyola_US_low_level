// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asv_interfaces:msg/StateObserver.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asv_interfaces/msg/detail/state_observer__rosidl_typesupport_introspection_c.h"
#include "asv_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asv_interfaces/msg/detail/state_observer__functions.h"
#include "asv_interfaces/msg/detail/state_observer__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `point`
#include "geometry_msgs/msg/point.h"
// Member `point`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `velocity`
// Member `disturbances`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity`
// Member `disturbances`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asv_interfaces__msg__StateObserver__init(message_memory);
}

void asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_fini_function(void * message_memory)
{
  asv_interfaces__msg__StateObserver__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__StateObserver, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__StateObserver, point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__StateObserver, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "disturbances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__StateObserver, disturbances),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_members = {
  "asv_interfaces__msg",  // message namespace
  "StateObserver",  // message name
  4,  // number of fields
  sizeof(asv_interfaces__msg__StateObserver),
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_member_array,  // message members
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_init_function,  // function to initialize message memory (memory has to be allocated)
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_type_support_handle = {
  0,
  &asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asv_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asv_interfaces, msg, StateObserver)() {
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_type_support_handle.typesupport_identifier) {
    asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asv_interfaces__msg__StateObserver__rosidl_typesupport_introspection_c__StateObserver_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
