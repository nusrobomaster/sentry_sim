# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_loam_interface_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED loam_interface_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(loam_interface_FOUND FALSE)
  elseif(NOT loam_interface_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(loam_interface_FOUND FALSE)
  endif()
  return()
endif()
set(_loam_interface_CONFIG_INCLUDED TRUE)

# output package information
if(NOT loam_interface_FIND_QUIETLY)
  message(STATUS "Found loam_interface: 0.0.2 (${loam_interface_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'loam_interface' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${loam_interface_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(loam_interface_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${loam_interface_DIR}/${_extra}")
endforeach()
