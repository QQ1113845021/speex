cmake_minimum_required(VERSION 2.6)
project(LIBSPEEX C)

include_directories(include)

if(WIN32)
include_directories(win32)
endif()

add_subdirectory(include)
add_subdirectory(libspeex)

if(INSTALL_COMPONENT_IN_POSTBUILD)
	add_install_target(INSTALL_libspeex COMP_libspeex "libspeex libspeexdsp")
endif()
