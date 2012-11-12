if(CMAKE_COMMON_IS_ALREADY_INCLUDED)
    return()
endif()
set(CMAKE_COMMON_IS_ALREADY_INCLUDED True)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR})

include(base)
include(make_test)
