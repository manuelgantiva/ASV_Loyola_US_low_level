// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:msg/StateObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__STRUCT_H_
#define ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'disturbances'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/StateObserver in the package asv_interfaces.
/**
  * this is a message used to communicate the results of the state observer
  * If you want to embed it in another message.
 */
typedef struct asv_interfaces__msg__StateObserver
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point point;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 disturbances;
} asv_interfaces__msg__StateObserver;

// Struct for a sequence of asv_interfaces__msg__StateObserver.
typedef struct asv_interfaces__msg__StateObserver__Sequence
{
  asv_interfaces__msg__StateObserver * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__msg__StateObserver__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__STRUCT_H_
