// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice
#include "asv_interfaces/msg/detail/xbee_observer__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `states`
#include "asv_interfaces/msg/detail/state_neighbor__functions.h"

bool
asv_interfaces__msg__XbeeObserver__init(asv_interfaces__msg__XbeeObserver * msg)
{
  if (!msg) {
    return false;
  }
  // counter
  // states
  if (!asv_interfaces__msg__StateNeighbor__Sequence__init(&msg->states, 0)) {
    asv_interfaces__msg__XbeeObserver__fini(msg);
    return false;
  }
  return true;
}

void
asv_interfaces__msg__XbeeObserver__fini(asv_interfaces__msg__XbeeObserver * msg)
{
  if (!msg) {
    return;
  }
  // counter
  // states
  asv_interfaces__msg__StateNeighbor__Sequence__fini(&msg->states);
}

bool
asv_interfaces__msg__XbeeObserver__are_equal(const asv_interfaces__msg__XbeeObserver * lhs, const asv_interfaces__msg__XbeeObserver * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // counter
  if (lhs->counter != rhs->counter) {
    return false;
  }
  // states
  if (!asv_interfaces__msg__StateNeighbor__Sequence__are_equal(
      &(lhs->states), &(rhs->states)))
  {
    return false;
  }
  return true;
}

bool
asv_interfaces__msg__XbeeObserver__copy(
  const asv_interfaces__msg__XbeeObserver * input,
  asv_interfaces__msg__XbeeObserver * output)
{
  if (!input || !output) {
    return false;
  }
  // counter
  output->counter = input->counter;
  // states
  if (!asv_interfaces__msg__StateNeighbor__Sequence__copy(
      &(input->states), &(output->states)))
  {
    return false;
  }
  return true;
}

asv_interfaces__msg__XbeeObserver *
asv_interfaces__msg__XbeeObserver__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__XbeeObserver * msg = (asv_interfaces__msg__XbeeObserver *)allocator.allocate(sizeof(asv_interfaces__msg__XbeeObserver), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asv_interfaces__msg__XbeeObserver));
  bool success = asv_interfaces__msg__XbeeObserver__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asv_interfaces__msg__XbeeObserver__destroy(asv_interfaces__msg__XbeeObserver * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asv_interfaces__msg__XbeeObserver__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asv_interfaces__msg__XbeeObserver__Sequence__init(asv_interfaces__msg__XbeeObserver__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__XbeeObserver * data = NULL;

  if (size) {
    data = (asv_interfaces__msg__XbeeObserver *)allocator.zero_allocate(size, sizeof(asv_interfaces__msg__XbeeObserver), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asv_interfaces__msg__XbeeObserver__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asv_interfaces__msg__XbeeObserver__fini(&data[i - 1]);
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
asv_interfaces__msg__XbeeObserver__Sequence__fini(asv_interfaces__msg__XbeeObserver__Sequence * array)
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
      asv_interfaces__msg__XbeeObserver__fini(&array->data[i]);
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

asv_interfaces__msg__XbeeObserver__Sequence *
asv_interfaces__msg__XbeeObserver__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__XbeeObserver__Sequence * array = (asv_interfaces__msg__XbeeObserver__Sequence *)allocator.allocate(sizeof(asv_interfaces__msg__XbeeObserver__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asv_interfaces__msg__XbeeObserver__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asv_interfaces__msg__XbeeObserver__Sequence__destroy(asv_interfaces__msg__XbeeObserver__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asv_interfaces__msg__XbeeObserver__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asv_interfaces__msg__XbeeObserver__Sequence__are_equal(const asv_interfaces__msg__XbeeObserver__Sequence * lhs, const asv_interfaces__msg__XbeeObserver__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asv_interfaces__msg__XbeeObserver__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asv_interfaces__msg__XbeeObserver__Sequence__copy(
  const asv_interfaces__msg__XbeeObserver__Sequence * input,
  asv_interfaces__msg__XbeeObserver__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asv_interfaces__msg__XbeeObserver);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asv_interfaces__msg__XbeeObserver * data =
      (asv_interfaces__msg__XbeeObserver *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asv_interfaces__msg__XbeeObserver__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asv_interfaces__msg__XbeeObserver__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asv_interfaces__msg__XbeeObserver__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
