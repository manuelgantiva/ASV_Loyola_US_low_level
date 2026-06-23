// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:msg/StateNeighbor.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__STRUCT_H_
#define ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'id'
#include "rosidl_runtime_c/string.h"
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/StateNeighbor in the package asv_interfaces.
/**
  * this is a message used to communicate the results of the state observer Neighbor
  * If you want to embed it in another message.
 */
typedef struct asv_interfaces__msg__StateNeighbor
{
  rosidl_runtime_c__String id;
  geometry_msgs__msg__Point point;
  geometry_msgs__msg__Vector3 velocity;
  int64_t msg_from;
} asv_interfaces__msg__StateNeighbor;

// Struct for a sequence of asv_interfaces__msg__StateNeighbor.
typedef struct asv_interfaces__msg__StateNeighbor__Sequence
{
  asv_interfaces__msg__StateNeighbor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__msg__StateNeighbor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__STRUCT_H_
