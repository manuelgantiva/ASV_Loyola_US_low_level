// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:msg/ReferenceLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__STRUCT_H_
#define ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'references'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'u_tar'
#include "std_msgs/msg/detail/float64__struct.h"

/// Struct defined in msg/ReferenceLlc in the package asv_interfaces.
/**
  * This is a message with References velocities values
  * If you want to embed it in another message.
 */
typedef struct asv_interfaces__msg__ReferenceLlc
{
  geometry_msgs__msg__Vector3__Sequence references;
  std_msgs__msg__Float64 u_tar;
} asv_interfaces__msg__ReferenceLlc;

// Struct for a sequence of asv_interfaces__msg__ReferenceLlc.
typedef struct asv_interfaces__msg__ReferenceLlc__Sequence
{
  asv_interfaces__msg__ReferenceLlc * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__msg__ReferenceLlc__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__STRUCT_H_
