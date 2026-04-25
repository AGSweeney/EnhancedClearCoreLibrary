# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles\\ClearCNC_Controller_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\ClearCNC_Controller_autogen.dir\\ParseCache.txt"
  "ClearCNC_Controller_autogen"
  )
endif()
