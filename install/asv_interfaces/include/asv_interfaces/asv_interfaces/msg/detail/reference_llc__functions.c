// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asv_interfaces:msg/ReferenceLlc.idl
// generated code does not contain a copyright notice
#include "asv_interfaces/msg/detail/reference_llc__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `references`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `u_tar`
#include "std_msgs/msg/detail/float64__functions.h"

bool
asv_interfaces__msg__ReferenceLlc__init(asv_interfaces__msg__ReferenceLlc * msg)
{
  if (!msg) {
    return false;
  }
  // references
  if (!geometry_msgs__msg__Vector3__Sequence__init(&msg->references, 0)) {
    asv_interfaces__msg__ReferenceLlc__fini(msg);
    return false;
  }
  // u_tar
  if (!std_msgs__msg__Float64__init(&msg->u_tar)) {
    asv_interfaces__msg__ReferenceLlc__fini(msg);
    return false;
  }
  return true;
}

void
asv_interfaces__msg__ReferenceLlc__fini(asv_interfaces__msg__ReferenceLlc * msg)
{
  if (!msg) {
    return;
  }
  // references
  geometry_msgs__msg__Vector3__Sequence__fini(&msg->references);
  // u_tar
  std_msgs__msg__Float64__fini(&msg->u_tar);
}

bool
asv_interfaces__msg__ReferenceLlc__are_equal(const asv_interfaces__msg__ReferenceLlc * lhs, const asv_interfaces__msg__ReferenceLlc * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // references
  if (!geometry_msgs__msg__Vector3__Sequence__are_equal(
      &(lhs->references), &(rhs->references)))
  {
    return false;
  }
  // u_tar
  if (!std_msgs__msg__Float64__are_equal(
      &(lhs->u_tar), &(rhs->u_tar)))
  {
    return false;
  }
  return true;
}

bool
asv_interfaces__msg__ReferenceLlc__copy(
  const asv_interfaces__msg__ReferenceLlc * input,
  asv_interfaces__msg__ReferenceLlc * output)
{
  if (!input || !output) {
    return false;
  }
  // references
  if (!geometry_msgs__msg__Vector3__Sequence__copy(
      &(input->references), &(output->references)))
  {
    return false;
  }
  // u_tar
  if (!std_msgs__msg__Float64__copy(
      &(input->u_tar), &(output->u_tar)))
  {
    return false;
  }
  return true;
}

asv_interfaces__msg__ReferenceLlc *
asv_interfaces__msg__ReferenceLlc__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__ReferenceLlc * msg = (asv_interfaces__msg__ReferenceLlc *)allocator.allocate(sizeof(asv_interfaces__msg__ReferenceLlc), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asv_interfaces__msg__ReferenceLlc));
  bool success = asv_interfaces__msg__ReferenceLlc__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asv_interfaces__msg__ReferenceLlc__destroy(asv_interfaces__msg__ReferenceLlc * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asv_interfaces__msg__ReferenceLlc__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asv_interfaces__msg__ReferenceLlc__Sequence__init(asv_interfaces__msg__ReferenceLlc__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__ReferenceLlc * data = NULL;

  if (size) {
    data = (asv_interfaces__msg__ReferenceLlc *)allocator.zero_allocate(size, sizeof(asv_interfaces__msg__ReferenceLlc), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asv_interfaces__msg__ReferenceLlc__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asv_interfaces__msg__ReferenceLlc__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
asv_interfaces__msg__ReferenceLlc__Sequence__fini(asv_interfaces__msg__ReferenceLlc__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      asv_interfaces__msg__ReferenceLlc__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

asv_interfaces__msg__ReferenceLlc__Sequence *
asv_interfaces__msg__ReferenceLlc__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__ReferenceLlc__Sequence * array = (asv_interfaces__msg__ReferenceLlc__Sequence *)allocator.allocate(sizeof(asv_interfaces__msg__ReferenceLlc__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asv_interfaces__msg__ReferenceLlc__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asv_interfaces__msg__ReferenceLlc__Sequence__destroy(asv_interfaces__msg__ReferenceLlc__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asv_interfaces__msg__ReferenceLlc__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asv_interfaces__msg__ReferenceLlc__Sequence__are_equal(const asv_interfaces__msg__ReferenceLlc__Sequence * lhs, const asv_interfaces__msg__ReferenceLlc__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asv_interfaces__msg__ReferenceLlc__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asv_interfaces__msg__ReferenceLlc__Sequence__copy(
  const asv_interfaces__msg__ReferenceLlc__Sequence * input,
  asv_interfaces__msg__ReferenceLlc__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asv_interfaces__msg__ReferenceLlc);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asv_interfaces__msg__ReferenceLlc * data =
      (asv_interfaces__msg__ReferenceLlc *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asv_interfaces__msg__ReferenceLlc__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asv_interfaces__msg__ReferenceLlc__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asv_interfaces__msg__ReferenceLlc__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
