// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from visibility_graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice
#include "visibility_graph_msg/msg/detail/graph__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `nodes`
#include "visibility_graph_msg/msg/detail/node__functions.h"

bool
visibility_graph_msg__msg__Graph__init(visibility_graph_msg__msg__Graph * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    visibility_graph_msg__msg__Graph__fini(msg);
    return false;
  }
  // robot_id
  // nodes
  if (!visibility_graph_msg__msg__Node__Sequence__init(&msg->nodes, 0)) {
    visibility_graph_msg__msg__Graph__fini(msg);
    return false;
  }
  // size
  return true;
}

void
visibility_graph_msg__msg__Graph__fini(visibility_graph_msg__msg__Graph * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // robot_id
  // nodes
  visibility_graph_msg__msg__Node__Sequence__fini(&msg->nodes);
  // size
}

bool
visibility_graph_msg__msg__Graph__are_equal(const visibility_graph_msg__msg__Graph * lhs, const visibility_graph_msg__msg__Graph * rhs)
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
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // nodes
  if (!visibility_graph_msg__msg__Node__Sequence__are_equal(
      &(lhs->nodes), &(rhs->nodes)))
  {
    return false;
  }
  // size
  if (lhs->size != rhs->size) {
    return false;
  }
  return true;
}

bool
visibility_graph_msg__msg__Graph__copy(
  const visibility_graph_msg__msg__Graph * input,
  visibility_graph_msg__msg__Graph * output)
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
  // robot_id
  output->robot_id = input->robot_id;
  // nodes
  if (!visibility_graph_msg__msg__Node__Sequence__copy(
      &(input->nodes), &(output->nodes)))
  {
    return false;
  }
  // size
  output->size = input->size;
  return true;
}

visibility_graph_msg__msg__Graph *
visibility_graph_msg__msg__Graph__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  visibility_graph_msg__msg__Graph * msg = (visibility_graph_msg__msg__Graph *)allocator.allocate(sizeof(visibility_graph_msg__msg__Graph), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(visibility_graph_msg__msg__Graph));
  bool success = visibility_graph_msg__msg__Graph__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
visibility_graph_msg__msg__Graph__destroy(visibility_graph_msg__msg__Graph * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    visibility_graph_msg__msg__Graph__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
visibility_graph_msg__msg__Graph__Sequence__init(visibility_graph_msg__msg__Graph__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  visibility_graph_msg__msg__Graph * data = NULL;

  if (size) {
    data = (visibility_graph_msg__msg__Graph *)allocator.zero_allocate(size, sizeof(visibility_graph_msg__msg__Graph), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = visibility_graph_msg__msg__Graph__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        visibility_graph_msg__msg__Graph__fini(&data[i - 1]);
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
visibility_graph_msg__msg__Graph__Sequence__fini(visibility_graph_msg__msg__Graph__Sequence * array)
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
      visibility_graph_msg__msg__Graph__fini(&array->data[i]);
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

visibility_graph_msg__msg__Graph__Sequence *
visibility_graph_msg__msg__Graph__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  visibility_graph_msg__msg__Graph__Sequence * array = (visibility_graph_msg__msg__Graph__Sequence *)allocator.allocate(sizeof(visibility_graph_msg__msg__Graph__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = visibility_graph_msg__msg__Graph__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
visibility_graph_msg__msg__Graph__Sequence__destroy(visibility_graph_msg__msg__Graph__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    visibility_graph_msg__msg__Graph__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
visibility_graph_msg__msg__Graph__Sequence__are_equal(const visibility_graph_msg__msg__Graph__Sequence * lhs, const visibility_graph_msg__msg__Graph__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!visibility_graph_msg__msg__Graph__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
visibility_graph_msg__msg__Graph__Sequence__copy(
  const visibility_graph_msg__msg__Graph__Sequence * input,
  visibility_graph_msg__msg__Graph__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(visibility_graph_msg__msg__Graph);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    visibility_graph_msg__msg__Graph * data =
      (visibility_graph_msg__msg__Graph *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!visibility_graph_msg__msg__Graph__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          visibility_graph_msg__msg__Graph__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!visibility_graph_msg__msg__Graph__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
