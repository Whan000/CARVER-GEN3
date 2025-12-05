# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_AMT212E-V_Interface_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED AMT212E-V_Interface_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(AMT212E-V_Interface_FOUND FALSE)
  elseif(NOT AMT212E-V_Interface_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(AMT212E-V_Interface_FOUND FALSE)
  endif()
  return()
endif()
set(_AMT212E-V_Interface_CONFIG_INCLUDED TRUE)

# output package information
if(NOT AMT212E-V_Interface_FIND_QUIETLY)
  message(STATUS "Found AMT212E-V_Interface: 0.0.0 (${AMT212E-V_Interface_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'AMT212E-V_Interface' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${AMT212E-V_Interface_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(AMT212E-V_Interface_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${AMT212E-V_Interface_DIR}/${_extra}")
endforeach()
