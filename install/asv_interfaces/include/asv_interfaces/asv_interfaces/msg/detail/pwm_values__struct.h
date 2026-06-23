// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__STRUCT_H_
#define ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PwmValues in the package asv_interfaces.
/**
  * This is a message with pwm values
  * If you want to embed it in another message.
 */
typedef struct asv_interfaces__msg__PwmValues
{
  uint16_t t_left;
  uint16_t t_righ;
} asv_interfaces__msg__PwmValues;

// Struct for a sequence of asv_interfaces__msg__PwmValues.
typedef struct asv_interfaces__msg__PwmValues__Sequence
{
  asv_interfaces__msg__PwmValues * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__msg__PwmValues__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__STRUCT_H_
