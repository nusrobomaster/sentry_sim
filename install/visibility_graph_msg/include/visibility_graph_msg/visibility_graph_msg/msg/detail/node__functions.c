// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from visibility_graph_msg:msg/Node.idl
// generated code does not contain a copyright notice
#include "visibility_graph_msg/msg/detail/node__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
// Member `surface_dirs`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `connect_nodes`
// Member `poly_connects`
// Member `contour_connects`
// Member `trajectory_connects`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
visibility_graph_msg__msg__Node__init(visibility_graph_msg__msg__Node * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  // id
  // freetype
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  // surface_dirs
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->surface_dirs, 0)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  // is_covered
  // is_frontier
  // is_navpoint
  // is_boundary
  // connect_nodes
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->connect_nodes, 0)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  // poly_connects
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->poly_connects, 0)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  // contour_connects
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->contour_connects, 0)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  // trajectory_connects
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->trajectory_connects, 0)) {
    visibility_graph_msg__msg__Node__fini(msg);
    return false;
  }
  return true;
}

void
visibility_graph_msg__msg__Node__fini(visibility_graph_msg__msg__Node * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // freetype
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // surface_dirs
  geometry_msgs__msg__Point__Sequence__fini(&msg->surface_dirs);
  // is_covered
  // is_frontier
  // is_navpoint
  // is_boundary
  // connect_nodes
  rosidl_runtime_c__uint32__Sequence__fini(&msg->connect_nodes);
  // poly_connects
  rosidl_runtime_c__uint32__Sequence__fini(&msg->poly_connects);
  // contour_connects
  rosidl_runtime_c__uint32__Sequence__fini(&msg->contour_connects);
  // trajectory_connects
  rosidl_runtime_c__uint32__Sequence__fini(&msg->trajectory_connects);
}

bool
visibility_graph_msg__msg__Node__are_equal(const visibility_graph_msg__msg__Node * lhs, const visibility_graph_msg__msg__Node * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // freetype
  if (lhs->freetype != rhs->freetype) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // surface_dirs
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->surface_dirs), &(rhs->surface_dirs)))
  {
    return false;
  }
  // is_covered
  if (lhs->is_covered != rhs->is_covered) {
    return false;
  }
  // is_frontier
  if (lhs->is_frontier != rhs->is_frontier) {
    return false;
  }
  // is_navpoint
  if (lhs->is_navpoint != rhs->is_navpoint) {
    return false;
  }
  // is_boundary
  if (lhs->is_boundary != rhs->is_boundary) {
    return false;
  }
  // connect_nodes
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->connect_nodes), &(rhs->connect_nodes)))
  {
    return false;
  }
  // poly_connects
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->poly_connects), &(rhs->poly_connects)))
  {
    return false;
  }
  // contour_connects
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->contour_connects), &(rhs->contour_connects)))
  {
    return false;
  }
  // trajectory_connects
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->trajectory_connects), &(rhs->trajectory_connects)))
  {
    return false;
  }
  return true;
}

bool
visibility_graph_msg__msg__Node__copy(
  const visibility_graph_msg__msg__Node * input,
  visibility_graph_msg__msg__Node * output)
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
  // id
  output->id = input->id;
  // freetype
  output->freetype = input->freetype;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // surface_dirs
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->surface_dirs), &(output->surface_dirs)))
  {
    return false;
  }
  // is_covered
  output->is_covered = input->is_covered;
  // is_frontier
  output->is_frontier = input->is_frontier;
  // is_navpoint
  output->is_navpoint = input->is_navpoint;
  // is_boundary
  output->is_boundary = input->is_boundary;
  // connect_nodes
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->connect_nodes), &(output->connect_nodes)))
  {
    return false;
  }
  // poly_connects
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->poly_connects), &(output->poly_connects)))
  {
    return false;
  }
  // contour_connects
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->contour_connects), &(output->contour_connects)))
  {
    return false;
  }
  // trajectory_connects
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->trajectory_connects), &(output->trajectory_connects)))
  {
    return false;
  }
  return true;
}

visibility_graph_msg__msg__Node *
visibility_graph_msg__msg__Node__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  visibility_graph_msg__msg__Node * msg = (visibility_graph_msg__msg__Node *)allocator.allocate(sizeof(visibility_graph_msg__msg__Node), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(visibility_graph_msg__msg__Node));
  bool success = visibility_graph_msg__msg__Node__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
visibility_graph_msg__msg__Node__destroy(visibility_graph_msg__msg__Node * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    visibility_graph_msg__msg__Node__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
visibility_graph_msg__msg__Node__Sequence__init(visibility_graph_msg__msg__Node__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  visibility_graph_msg__msg__Node * data = NULL;

  if (size) {
    data = (visibility_graph_msg__msg__Node *)allocator.zero_allocate(size, sizeof(visibility_graph_msg__msg__Node), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = visibility_graph_msg__msg__Node__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        visibility_graph_msg__msg__Node__fini(&data[i - 1]);
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
visibility_graph_msg__msg__Node__Sequence__fini(visibility_graph_msg__msg__Node__Sequence * array)
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
      visibility_graph_msg__msg__Node__fini(&array->data[i]);
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

visibility_graph_msg__msg__Node__Sequence *
visibility_graph_msg__msg__Node__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  visibility_graph_msg__msg__Node__Sequence * array = (visibility_graph_msg__msg__Node__Sequence *)allocator.allocate(sizeof(visibility_graph_msg__msg__Node__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = visibility_graph_msg__msg__Node__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
visibility_graph_msg__msg__Node__Sequence__destroy(visibility_graph_msg__msg__Node__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    visibility_graph_msg__msg__Node__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
visibility_graph_msg__msg__Node__Sequence__are_equal(const visibility_graph_msg__msg__Node__Sequence * lhs, const visibility_graph_msg__msg__Node__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!visibility_graph_msg__msg__Node__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
visibility_graph_msg__msg__Node__Sequence__copy(
  const visibility_graph_msg__msg__Node__Sequence * input,
  visibility_graph_msg__msg__Node__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(visibility_graph_msg__msg__Node);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    visibility_graph_msg__msg__Node * data =
      (visibility_graph_msg__msg__Node *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!visibility_graph_msg__msg__Node__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          visibility_graph_msg__msg__Node__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!visibility_graph_msg__msg__Node__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
