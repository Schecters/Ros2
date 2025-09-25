# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gocargo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gocargo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gocargo_FOUND FALSE)
  elseif(NOT gocargo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gocargo_FOUND FALSE)
  endif()
  return()
endif()
set(_gocargo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gocargo_FIND_QUIETLY)
  message(STATUS "Found gocargo: 0.0.0 (${gocargo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gocargo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gocargo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gocargo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gocargo_DIR}/${_extra}")
endforeach()
