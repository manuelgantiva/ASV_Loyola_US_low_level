// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__STRUCT_H_
#define ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'states'
#include "asv_interfaces/msg/detail/state_neighbor__struct.h"

/// Struct defined in msg/XbeeObserver in the package asv_interfaces.
/**
  * this is a message used to communicate the results of the state observer by Xbee
  * If you want to embed it in another message.
 */
typedef struct asv_interfaces__msg__XbeeObserver
{
  uint8_t counter;
  asv_interfaces__msg__StateNeighbor__Sequence states;
} asv_interfaces__msg__XbeeObserver;

// Struct for a sequence of asv_interfaces__msg__XbeeObserver.
typedef struct asv_interfaces__msg__XbeeObserver__Sequence
{
  asv_interfaces__msg__XbeeObserver * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__msg__XbeeObserver__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__STRUCT_H_
