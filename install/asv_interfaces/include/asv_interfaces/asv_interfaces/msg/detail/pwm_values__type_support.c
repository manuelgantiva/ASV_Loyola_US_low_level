// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asv_interfaces/msg/detail/pwm_values__rosidl_typesupport_introspection_c.h"
#include "asv_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asv_interfaces/msg/detail/pwm_values__functions.h"
#include "asv_interfaces/msg/detail/pwm_values__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asv_interfaces__msg__PwmValues__init(message_memory);
}

void asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_fini_function(void * message_memory)
{
  asv_interfaces__msg__PwmValues__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_member_array[2] = {
  {
    "t_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__PwmValues, t_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "t_righ",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces__msg__PwmValues, t_righ),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_members = {
  "asv_interfaces__msg",  // message namespace
  "PwmValues",  // message name
  2,  // number of fields
  sizeof(asv_interfaces__msg__PwmValues),
  asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_member_array,  // message members
  asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_init_function,  // function to initialize message memory (memory has to be allocated)
  asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_type_support_handle = {
  0,
  &asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asv_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asv_interfaces, msg, PwmValues)() {
  if (!asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_type_support_handle.typesupport_identifier) {
    asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asv_interfaces__msg__PwmValues__rosidl_typesupport_introspection_c__PwmValues_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
