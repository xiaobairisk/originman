# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tros_ai_fusion_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tros_ai_fusion_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tros_ai_fusion_FOUND FALSE)
  elseif(NOT tros_ai_fusion_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tros_ai_fusion_FOUND FALSE)
  endif()
  return()
endif()
set(_tros_ai_fusion_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tros_ai_fusion_FIND_QUIETLY)
  message(STATUS "Found tros_ai_fusion: 0.0.0 (${tros_ai_fusion_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tros_ai_fusion' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tros_ai_fusion_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tros_ai_fusion_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tros_ai_fusion_DIR}/${_extra}")
endforeach()
