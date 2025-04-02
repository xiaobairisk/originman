# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tros_perception_render_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tros_perception_render_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tros_perception_render_FOUND FALSE)
  elseif(NOT tros_perception_render_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tros_perception_render_FOUND FALSE)
  endif()
  return()
endif()
set(_tros_perception_render_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tros_perception_render_FIND_QUIETLY)
  message(STATUS "Found tros_perception_render: 0.0.0 (${tros_perception_render_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tros_perception_render' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tros_perception_render_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tros_perception_render_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tros_perception_render_DIR}/${_extra}")
endforeach()
