// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice
#include "asv_interfaces/msg/detail/pwm_values__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
asv_interfaces__msg__PwmValues__init(asv_interfaces__msg__PwmValues * msg)
{
  if (!msg) {
    return false;
  }
  // t_left
  // t_righ
  return true;
}

void
asv_interfaces__msg__PwmValues__fini(asv_interfaces__msg__PwmValues * msg)
{
  if (!msg) {
    return;
  }
  // t_left
  // t_righ
}

bool
asv_interfaces__msg__PwmValues__are_equal(const asv_interfaces__msg__PwmValues * lhs, const asv_interfaces__msg__PwmValues * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // t_left
  if (lhs->t_left != rhs->t_left) {
    return false;
  }
  // t_righ
  if (lhs->t_righ != rhs->t_righ) {
    return false;
  }
  return true;
}

bool
asv_interfaces__msg__PwmValues__copy(
  const asv_interfaces__msg__PwmValues * input,
  asv_interfaces__msg__PwmValues * output)
{
  if (!input || !output) {
    return false;
  }
  // t_left
  output->t_left = input->t_left;
  // t_righ
  output->t_righ = input->t_righ;
  return true;
}

asv_interfaces__msg__PwmValues *
asv_interfaces__msg__PwmValues__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__PwmValues * msg = (asv_interfaces__msg__PwmValues *)allocator.allocate(sizeof(asv_interfaces__msg__PwmValues), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asv_interfaces__msg__PwmValues));
  bool success = asv_interfaces__msg__PwmValues__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asv_interfaces__msg__PwmValues__destroy(asv_interfaces__msg__PwmValues * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asv_interfaces__msg__PwmValues__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asv_interfaces__msg__PwmValues__Sequence__init(asv_interfaces__msg__PwmValues__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__PwmValues * data = NULL;

  if (size) {
    data = (asv_interfaces__msg__PwmValues *)allocator.zero_allocate(size, sizeof(asv_interfaces__msg__PwmValues), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asv_interfaces__msg__PwmValues__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asv_interfaces__msg__PwmValues__fini(&data[i - 1]);
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
asv_interfaces__msg__PwmValues__Sequence__fini(asv_interfaces__msg__PwmValues__Sequence * array)
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
      asv_interfaces__msg__PwmValues__fini(&array->data[i]);
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

asv_interfaces__msg__PwmValues__Sequence *
asv_interfaces__msg__PwmValues__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__PwmValues__Sequence * array = (asv_interfaces__msg__PwmValues__Sequence *)allocator.allocate(sizeof(asv_interfaces__msg__PwmValues__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asv_interfaces__msg__PwmValues__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asv_interfaces__msg__PwmValues__Sequence__destroy(asv_interfaces__msg__PwmValues__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asv_interfaces__msg__PwmValues__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asv_interfaces__msg__PwmValues__Sequence__are_equal(const asv_interfaces__msg__PwmValues__Sequence * lhs, const asv_interfaces__msg__PwmValues__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asv_interfaces__msg__PwmValues__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asv_interfaces__msg__PwmValues__Sequence__copy(
  const asv_interfaces__msg__PwmValues__Sequence * input,
  asv_interfaces__msg__PwmValues__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asv_interfaces__msg__PwmValues);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asv_interfaces__msg__PwmValues * data =
      (asv_interfaces__msg__PwmValues *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asv_interfaces__msg__PwmValues__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asv_interfaces__msg__PwmValues__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asv_interfaces__msg__PwmValues__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
