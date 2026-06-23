// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:srv/SetLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__SRV__DETAIL__SET_LLC__STRUCT_H_
#define ASV_INTERFACES__SRV__DETAIL__SET_LLC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LLC_APM'.
/**
  * basic LLC_MODE
 */
enum
{
  asv_interfaces__srv__SetLlc_Request__LLC_APM = 1
};

/// Constant 'LLC_IFAC'.
enum
{
  asv_interfaces__srv__SetLlc_Request__LLC_IFAC = 2
};

/// Constant 'LLC_MPC'.
enum
{
  asv_interfaces__srv__SetLlc_Request__LLC_MPC = 3
};

/// Struct defined in srv/SetLlc in the package asv_interfaces.
typedef struct asv_interfaces__srv__SetLlc_Request
{
  /// filled by LLC_MODE enum value
  uint8_t llc_mode;
} asv_interfaces__srv__SetLlc_Request;

// Struct for a sequence of asv_interfaces__srv__SetLlc_Request.
typedef struct asv_interfaces__srv__SetLlc_Request__Sequence
{
  asv_interfaces__srv__SetLlc_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__srv__SetLlc_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetLlc in the package asv_interfaces.
typedef struct asv_interfaces__srv__SetLlc_Response
{
  /// Mode correctly and SET_ESO are sent
  bool success;
} asv_interfaces__srv__SetLlc_Response;

// Struct for a sequence of asv_interfaces__srv__SetLlc_Response.
typedef struct asv_interfaces__srv__SetLlc_Response__Sequence
{
  asv_interfaces__srv__SetLlc_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__srv__SetLlc_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__SRV__DETAIL__SET_LLC__STRUCT_H_
