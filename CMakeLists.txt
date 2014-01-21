cmake_minimum_required(VERSION 2.6)
project(SPEEX C)

include_directories(include)

if(WIN32)
include_directories(win32)
endif()

add_subdirectory(include)
add_subdirectory(libspeex)
