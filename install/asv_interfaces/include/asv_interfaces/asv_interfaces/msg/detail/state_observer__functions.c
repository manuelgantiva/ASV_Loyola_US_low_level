// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asv_interfaces:msg/StateObserver.idl
// generated code does not contain a copyright notice
#include "asv_interfaces/msg/detail/state_observer__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `point`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `disturbances`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
asv_interfaces__msg__StateObserver__init(asv_interfaces__msg__StateObserver * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    asv_interfaces__msg__StateObserver__fini(msg);
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__init(&msg->point)) {
    asv_interfaces__msg__StateObserver__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    asv_interfaces__msg__StateObserver__fini(msg);
    return false;
  }
  // disturbances
  if (!geometry_msgs__msg__Vector3__init(&msg->disturbances)) {
    asv_interfaces__msg__StateObserver__fini(msg);
    return false;
  }
  return true;
}

void
asv_interfaces__msg__StateObserver__fini(asv_interfaces__msg__StateObserver * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // point
  geometry_msgs__msg__Point__fini(&msg->point);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // disturbances
  geometry_msgs__msg__Vector3__fini(&msg->disturbances);
}

bool
asv_interfaces__msg__StateObserver__are_equal(const asv_interfaces__msg__StateObserver * lhs, const asv_interfaces__msg__StateObserver * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->point), &(rhs->point)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // disturbances
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->disturbances), &(rhs->disturbances)))
  {
    return false;
  }
  return true;
}

bool
asv_interfaces__msg__StateObserver__copy(
  const asv_interfaces__msg__StateObserver * input,
  asv_interfaces__msg__StateObserver * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__copy(
      &(input->point), &(output->point)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // disturbances
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->disturbances), &(output->disturbances)))
  {
    return false;
  }
  return true;
}

asv_interfaces__msg__StateObserver *
asv_interfaces__msg__StateObserver__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__StateObserver * msg = (asv_interfaces__msg__StateObserver *)allocator.allocate(sizeof(asv_interfaces__msg__StateObserver), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asv_interfaces__msg__StateObserver));
  bool success = asv_interfaces__msg__StateObserver__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asv_interfaces__msg__StateObserver__destroy(asv_interfaces__msg__StateObserver * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asv_interfaces__msg__StateObserver__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asv_interfaces__msg__StateObserver__Sequence__init(asv_interfaces__msg__StateObserver__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__StateObserver * data = NULL;

  if (size) {
    data = (asv_interfaces__msg__StateObserver *)allocator.zero_allocate(size, sizeof(asv_interfaces__msg__StateObserver), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asv_interfaces__msg__StateObserver__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asv_interfaces__msg__StateObserver__fini(&data[i - 1]);
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
asv_interfaces__msg__StateObserver__Sequence__fini(asv_interfaces__msg__StateObserver__Sequence * array)
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
      asv_interfaces__msg__StateObserver__fini(&array->data[i]);
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

asv_interfaces__msg__StateObserver__Sequence *
asv_interfaces__msg__StateObserver__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__msg__StateObserver__Sequence * array = (asv_interfaces__msg__StateObserver__Sequence *)allocator.allocate(sizeof(asv_interfaces__msg__StateObserver__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asv_interfaces__msg__StateObserver__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asv_interfaces__msg__StateObserver__Sequence__destroy(asv_interfaces__msg__StateObserver__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asv_interfaces__msg__StateObserver__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asv_interfaces__msg__StateObserver__Sequence__are_equal(const asv_interfaces__msg__StateObserver__Sequence * lhs, const asv_interfaces__msg__StateObserver__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asv_interfaces__msg__StateObserver__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asv_interfaces__msg__StateObserver__Sequence__copy(
  const asv_interfaces__msg__StateObserver__Sequence * input,
  asv_interfaces__msg__StateObserver__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asv_interfaces__msg__StateObserver);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asv_interfaces__msg__StateObserver * data =
      (asv_interfaces__msg__StateObserver *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asv_interfaces__msg__StateObserver__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asv_interfaces__msg__StateObserver__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asv_interfaces__msg__StateObserver__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
