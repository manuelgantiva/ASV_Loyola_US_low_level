// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asv_interfaces:srv/SetObs.idl
// generated code does not contain a copyright notice
#include "asv_interfaces/srv/detail/set_obs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
asv_interfaces__srv__SetObs_Request__init(asv_interfaces__srv__SetObs_Request * msg)
{
  if (!msg) {
    return false;
  }
  // eso_mode
  return true;
}

void
asv_interfaces__srv__SetObs_Request__fini(asv_interfaces__srv__SetObs_Request * msg)
{
  if (!msg) {
    return;
  }
  // eso_mode
}

bool
asv_interfaces__srv__SetObs_Request__are_equal(const asv_interfaces__srv__SetObs_Request * lhs, const asv_interfaces__srv__SetObs_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // eso_mode
  if (lhs->eso_mode != rhs->eso_mode) {
    return false;
  }
  return true;
}

bool
asv_interfaces__srv__SetObs_Request__copy(
  const asv_interfaces__srv__SetObs_Request * input,
  asv_interfaces__srv__SetObs_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // eso_mode
  output->eso_mode = input->eso_mode;
  return true;
}

asv_interfaces__srv__SetObs_Request *
asv_interfaces__srv__SetObs_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__srv__SetObs_Request * msg = (asv_interfaces__srv__SetObs_Request *)allocator.allocate(sizeof(asv_interfaces__srv__SetObs_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asv_interfaces__srv__SetObs_Request));
  bool success = asv_interfaces__srv__SetObs_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asv_interfaces__srv__SetObs_Request__destroy(asv_interfaces__srv__SetObs_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asv_interfaces__srv__SetObs_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asv_interfaces__srv__SetObs_Request__Sequence__init(asv_interfaces__srv__SetObs_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__srv__SetObs_Request * data = NULL;

  if (size) {
    data = (asv_interfaces__srv__SetObs_Request *)allocator.zero_allocate(size, sizeof(asv_interfaces__srv__SetObs_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asv_interfaces__srv__SetObs_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asv_interfaces__srv__SetObs_Request__fini(&data[i - 1]);
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
asv_interfaces__srv__SetObs_Request__Sequence__fini(asv_interfaces__srv__SetObs_Request__Sequence * array)
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
      asv_interfaces__srv__SetObs_Request__fini(&array->data[i]);
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

asv_interfaces__srv__SetObs_Request__Sequence *
asv_interfaces__srv__SetObs_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__srv__SetObs_Request__Sequence * array = (asv_interfaces__srv__SetObs_Request__Sequence *)allocator.allocate(sizeof(asv_interfaces__srv__SetObs_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asv_interfaces__srv__SetObs_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asv_interfaces__srv__SetObs_Request__Sequence__destroy(asv_interfaces__srv__SetObs_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asv_interfaces__srv__SetObs_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asv_interfaces__srv__SetObs_Request__Sequence__are_equal(const asv_interfaces__srv__SetObs_Request__Sequence * lhs, const asv_interfaces__srv__SetObs_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asv_interfaces__srv__SetObs_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asv_interfaces__srv__SetObs_Request__Sequence__copy(
  const asv_interfaces__srv__SetObs_Request__Sequence * input,
  asv_interfaces__srv__SetObs_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asv_interfaces__srv__SetObs_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asv_interfaces__srv__SetObs_Request * data =
      (asv_interfaces__srv__SetObs_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asv_interfaces__srv__SetObs_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asv_interfaces__srv__SetObs_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asv_interfaces__srv__SetObs_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
asv_interfaces__srv__SetObs_Response__init(asv_interfaces__srv__SetObs_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
asv_interfaces__srv__SetObs_Response__fini(asv_interfaces__srv__SetObs_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
asv_interfaces__srv__SetObs_Response__are_equal(const asv_interfaces__srv__SetObs_Response * lhs, const asv_interfaces__srv__SetObs_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
asv_interfaces__srv__SetObs_Response__copy(
  const asv_interfaces__srv__SetObs_Response * input,
  asv_interfaces__srv__SetObs_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

asv_interfaces__srv__SetObs_Response *
asv_interfaces__srv__SetObs_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__srv__SetObs_Response * msg = (asv_interfaces__srv__SetObs_Response *)allocator.allocate(sizeof(asv_interfaces__srv__SetObs_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asv_interfaces__srv__SetObs_Response));
  bool success = asv_interfaces__srv__SetObs_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asv_interfaces__srv__SetObs_Response__destroy(asv_interfaces__srv__SetObs_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asv_interfaces__srv__SetObs_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asv_interfaces__srv__SetObs_Response__Sequence__init(asv_interfaces__srv__SetObs_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__srv__SetObs_Response * data = NULL;

  if (size) {
    data = (asv_interfaces__srv__SetObs_Response *)allocator.zero_allocate(size, sizeof(asv_interfaces__srv__SetObs_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asv_interfaces__srv__SetObs_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asv_interfaces__srv__SetObs_Response__fini(&data[i - 1]);
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
asv_interfaces__srv__SetObs_Response__Sequence__fini(asv_interfaces__srv__SetObs_Response__Sequence * array)
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
      asv_interfaces__srv__SetObs_Response__fini(&array->data[i]);
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

asv_interfaces__srv__SetObs_Response__Sequence *
asv_interfaces__srv__SetObs_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asv_interfaces__srv__SetObs_Response__Sequence * array = (asv_interfaces__srv__SetObs_Response__Sequence *)allocator.allocate(sizeof(asv_interfaces__srv__SetObs_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asv_interfaces__srv__SetObs_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asv_interfaces__srv__SetObs_Response__Sequence__destroy(asv_interfaces__srv__SetObs_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asv_interfaces__srv__SetObs_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asv_interfaces__srv__SetObs_Response__Sequence__are_equal(const asv_interfaces__srv__SetObs_Response__Sequence * lhs, const asv_interfaces__srv__SetObs_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asv_interfaces__srv__SetObs_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asv_interfaces__srv__SetObs_Response__Sequence__copy(
  const asv_interfaces__srv__SetObs_Response__Sequence * input,
  asv_interfaces__srv__SetObs_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asv_interfaces__srv__SetObs_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asv_interfaces__srv__SetObs_Response * data =
      (asv_interfaces__srv__SetObs_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asv_interfaces__srv__SetObs_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asv_interfaces__srv__SetObs_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asv_interfaces__srv__SetObs_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
