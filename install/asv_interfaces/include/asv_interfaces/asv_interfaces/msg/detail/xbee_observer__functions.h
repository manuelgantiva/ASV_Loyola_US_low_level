// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__FUNCTIONS_H_
#define ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "asv_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "asv_interfaces/msg/detail/xbee_observer__struct.h"

/// Initialize msg/XbeeObserver message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * asv_interfaces__msg__XbeeObserver
 * )) before or use
 * asv_interfaces__msg__XbeeObserver__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
bool
asv_interfaces__msg__XbeeObserver__init(asv_interfaces__msg__XbeeObserver * msg);

/// Finalize msg/XbeeObserver message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
void
asv_interfaces__msg__XbeeObserver__fini(asv_interfaces__msg__XbeeObserver * msg);

/// Create msg/XbeeObserver message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * asv_interfaces__msg__XbeeObserver__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
asv_interfaces__msg__XbeeObserver *
asv_interfaces__msg__XbeeObserver__create();

/// Destroy msg/XbeeObserver message.
/**
 * It calls
 * asv_interfaces__msg__XbeeObserver__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
void
asv_interfaces__msg__XbeeObserver__destroy(asv_interfaces__msg__XbeeObserver * msg);

/// Check for msg/XbeeObserver message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
bool
asv_interfaces__msg__XbeeObserver__are_equal(const asv_interfaces__msg__XbeeObserver * lhs, const asv_interfaces__msg__XbeeObserver * rhs);

/// Copy a msg/XbeeObserver message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
bool
asv_interfaces__msg__XbeeObserver__copy(
  const asv_interfaces__msg__XbeeObserver * input,
  asv_interfaces__msg__XbeeObserver * output);

/// Initialize array of msg/XbeeObserver messages.
/**
 * It allocates the memory for the number of elements and calls
 * asv_interfaces__msg__XbeeObserver__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
bool
asv_interfaces__msg__XbeeObserver__Sequence__init(asv_interfaces__msg__XbeeObserver__Sequence * array, size_t size);

/// Finalize array of msg/XbeeObserver messages.
/**
 * It calls
 * asv_interfaces__msg__XbeeObserver__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
void
asv_interfaces__msg__XbeeObserver__Sequence__fini(asv_interfaces__msg__XbeeObserver__Sequence * array);

/// Create array of msg/XbeeObserver messages.
/**
 * It allocates the memory for the array and calls
 * asv_interfaces__msg__XbeeObserver__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
asv_interfaces__msg__XbeeObserver__Sequence *
asv_interfaces__msg__XbeeObserver__Sequence__create(size_t size);

/// Destroy array of msg/XbeeObserver messages.
/**
 * It calls
 * asv_interfaces__msg__XbeeObserver__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
void
asv_interfaces__msg__XbeeObserver__Sequence__destroy(asv_interfaces__msg__XbeeObserver__Sequence * array);

/// Check for msg/XbeeObserver message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
bool
asv_interfaces__msg__XbeeObserver__Sequence__are_equal(const asv_interfaces__msg__XbeeObserver__Sequence * lhs, const asv_interfaces__msg__XbeeObserver__Sequence * rhs);

/// Copy an array of msg/XbeeObserver messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_asv_interfaces
bool
asv_interfaces__msg__XbeeObserver__Sequence__copy(
  const asv_interfaces__msg__XbeeObserver__Sequence * input,
  asv_interfaces__msg__XbeeObserver__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__FUNCTIONS_H_
