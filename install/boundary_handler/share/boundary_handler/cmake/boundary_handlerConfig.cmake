# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_boundary_handler_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED boundary_handler_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(boundary_handler_FOUND FALSE)
  elseif(NOT boundary_handler_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(boundary_handler_FOUND FALSE)
  endif()
  return()
endif()
set(_boundary_handler_CONFIG_INCLUDED TRUE)

# output package information
if(NOT boundary_handler_FIND_QUIETLY)
  message(STATUS "Found boundary_handler: 0.0.1 (${boundary_handler_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'boundary_handler' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${boundary_handler_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(boundary_handler_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${boundary_handler_DIR}/${_extra}")
endforeach()
