############################################################################
# CMakeLists.txt
# Copyright (C) 2014  Belledonne Communications, Grenoble France
#
############################################################################
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
#
############################################################################

cmake_minimum_required(VERSION 2.8)
project(SPEEX C)


set(SPEEX_MAJOR_VERSION 1)
set(SPEEX_MINOR_VERSION 1)
set(SPEEX_MICRO_VERSION 16)
set(SPEEX_EXTRA_VERSION " ")
set(SPEEX_VERSION "1.2rc1")


option(ENABLE_VALGRIND "Enable valgrind extra checks" OFF)
option(ENABLE_SSE "Enable SSE support" OFF)
option(ENABLE_FIXED_POINT "Compile as fixed-point" OFF)
option(ENABLE_FLOAT_API "Enable the floating-point API" ON)
option(ENABLE_VBR "Enable VBR and VAD from the codec" ON)
option(ENABLE_ARM4_ASM "Make use of ARM4 assembly optimizations" OFF)
option(ENABLE_ARM5E_ASM "Make use of ARM5E assembly optimizations" OFF)
option(ENABLE_ARMV7_NEON_ASM "Make use of ARMV7a NEON assembly optimizations" OFF)
option(ENABLE_BLACKFIN_ASM "Make use of Blackfin assembly optimizations" OFF)
option(ENABLE_FIXED_POINT_DEBUG "Debug fixed-point implementation" OFF)
option(ENABLE_TI_C55X "Enable support for TI C55X DSP" OFF)
option(ENABLE_VORBIS_PSY "Enable the Vorbis psy model" OFF)
set(WITH_FFT "" CACHE STRING "Use an alternate FFT implementation. The available choices are kiss (default fixed point) and smallft (default floating point)")
option(ENABLE_STATIC "Build static library (default is shared library)." OFF)


include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckLibraryExists)
include(CheckCSourceCompiles)
include(CheckTypeSize)


check_include_file("alloca.h" HAVE_ALLOCA_H)
check_include_file("dlfcn.h" HAVE_DLFCN_H)
check_include_file("getopt.h" HAVE_GETOPT_H)
check_include_file("inttypes.h" HAVE_INTTYPES_H)
check_include_file("memory.h" HAVE_MEMORY_H)
check_include_file("stdint.h" HAVE_STDINT_H)
check_include_file("stdlib.h" HAVE_STDLIB_H)
check_include_file("strings.h" HAVE_STRINGS_H)
check_include_file("string.h" HAVE_STRING_H)
check_include_file("sys/audioio.h" HAVE_SYS_AUDIOIO_H)
check_include_file("sys/soundcard.h" HAVE_SYS_SOUNDCARD_H)
check_include_file("sys/stat.h" HAVE_SYS_STAT_H)
check_include_file("sys/types.h" HAVE_SYS_TYPES_H)
check_include_file("unistd.h" HAVE_UNISTD_H)

check_function_exists("getopt_long" HAVE_GETOPT_LONG)

check_library_exists("m" "sin" "" HAVE_LIBM)
check_library_exists("winmm" "main" "" HAVE_LIBWINMM)

check_c_source_compiles("int main(int argc, char *argv[]) {
int foo;
foo = 10;
int array[foo];
}"
	VAR_ARRAYS)
set(ALLOCA_SOURCE "#include <stdlib.h>
int main(int argc, char *argv[]) {
int foo=10;
int *array = alloca(foo);
}")
if(${HAVE_ALLOCA_H})
	set(ALLOCA_SOURCE "#include <alloca.h>
${ALLOCA_SOURCE}")
endif(${HAVE_ALLOCA_H})
check_c_source_compiles("${ALLOCA_SOURCE}" HAS_ALLOCA)
if("${HAS_ALLOCA}" AND NOT "${VAR_ARRAYS}")
	set(USE_ALLOCA 1)
endif("${HAS_ALLOCA}" AND NOT "${VAR_ARRAYS}")
check_c_source_compiles("#include <xmmintrin.h>
__m128 testfunc(float *a, float *b) {
return _mm_add_ps(_mm_loadu_ps(a), _mm_loadu_ps(b));
}
int main(int argc, char *argv[]) {}"
	HAS_SSE)
set(CMAKE_C_FLAGS_SAVED ${CMAKE_C_FLAGS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")
check_c_source_compiles("#pragma GCC visibility push(hidden)
__attribute__((visibility(\"default\")))
int var=10;
int main(int argc, char *argv[]) {}"
	HAS_VISIBILITY)
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS_SAVED})
if(${HAS_VISIBILITY})
	set(EXPORT "__attribute__((visibility(\"default\")))")
else(${HAS_VISIBILITY})
	set(EXPORT " ")
endif(${HAS_VISIBILITY})


set(FFT smallft)
if("${HAS_SSE}" AND "${ENABLE_SSE}")
	set(HAS_SSE 1)
else("${HAS_SSE}" AND "${ENABLE_SSE}")
	set(HAS_SSE 0)
endif("${HAS_SSE}" AND "${ENABLE_SSE}")
if(${ENABLE_FIXED_POINT})
	set(FIXED_POINT 1)
	set(HAS_SSE 0)
	set(FFT kiss)
else(${ENABLE_FIXED_POINT})
	set(FLOATING_POINT 1)
endif(${ENABLE_FIXED_POINT})
set(ARM4_ASM ${ENABLE_ARM4_ASM})
set(ARM5E_ASM ${ENABLE_ARM5E_ASM})
set(ARMV7NEON_ASM ${ENABLE_ARMV7_NEON_ASM})
if(${ARMV7NEON_ASM})
	add_definitions(-marm -DSHORTCUTS)
endif(${ARMV7NEON_ASM})
set(BFIN_ASM ${ENABLE_BLACKFIN_ASM})
set(TI_C55X ${ENABLE_TI_C55X})
if(${TI_C55X})
	set(HAS_CHAR16 1)
endif(${TI_C55X})
if(NOT ENABLE_FLOAT_API)
	set(DISABLE_FLOAT_API 1)
endif(NOT ENABLE_FLOAT_API)
if(NOT ENABLE_VBR)
	set(DISABLE_VBR 1)
endif(NOT ENABLE_VBR)
set(FIXED_DEBUG ${ENABLE_FIXED_POINT_DEBUG})
set(VORBIS_PSYCHO ${ENABLE_VORBIS_PSY})
if(${HAS_SSE})
	set(_USE_SSE 1)
endif(${HAS_SSE})
if(${WITH_FFT})
	set(FFT ${WITH_FFT})
endif(${WITH_FFT})

if("${FFT}" STREQUAL "kiss")
	set(USE_KISS_FFT 1)
endif("${FFT}" STREQUAL "kiss")
if("${FFT}" STREQUAL "smallft")
	set(USE_SMALLFT 1)
endif("${FFT}" STREQUAL "smallft")


set(TYPES_CHECKED short int long)
set(CMAKE_EXTRA_INCLUDE_FILES stdint.h)
foreach(TYPE ${TYPES_CHECKED})
	string(TOUPPER ${TYPE} TYPE_SIZE_VAR)
	check_type_size(${TYPE} ${TYPE_SIZE_VAR})
endforeach()
set(CMAKE_EXTRA_INCLUDE_FILES)


if(${HAS_CHAR16})
	if(${SHORT} EQUAL 1)
		set(SIZE16 "short")
	endif(${SHORT} EQUAL 1)
	if(${INT} EQUAL 1)
		set(SIZE16 "int")
	endif(${INT} EQUAL 1)
	if(${INT} EQUAL 2)
		set(SIZE32 "int")
	endif(${INT} EQUAL 2)
	if(${LONG} EQUAL 2)
		set(SIZE32 "long")
	endif(${LONG} EQUAL 2)
	if(${SHORT} EQUAL 2)
		set(SIZE32 "short")
	endif(${SHORT} EQUAL 2)
else(${HAS_CHAR16})
	if(${SHORT} EQUAL 2)
		set(SIZE16 "short")
	endif(${SHORT} EQUAL 2)
	if(${INT} EQUAL 2)
		set(SIZE16 "int")
	endif(${INT} EQUAL 2)
	if(${INT} EQUAL 4)
		set(SIZE32 "int")
	endif(${INT} EQUAL 4)
	if(${LONG} EQUAL 4)
		set(SIZE32 "long")
	endif(${LONG} EQUAL 4)
	if(${SHORT} EQUAL 4)
		set(SIZE32 "short")
	endif(${SHORT} EQUAL 4)
endif(${HAS_CHAR16})

foreach(KEYWORD "inline" "__inline__" "__inline")
	if(NOT DEFINED C_INLINE)
		try_compile(C_HAS_${KEYWORD} "${CMAKE_CURRENT_BINARY_DIR}"
			"${CMAKE_CURRENT_SOURCE_DIR}/test_inline.c"
			COMPILE_DEFINITIONS "-Dinline=${KEYWORD}")
		if(C_HAS_${KEYWORD})
			set(C_INLINE TRUE)
			if(NOT "${KEYWORD}" STREQUAL "inline")
				set(inline ${KEYWORD})
			endif(NOT "${KEYWORD}" STREQUAL "inline")
		endif(C_HAS_${KEYWORD})
	endif(NOT DEFINED C_INLINE)
endforeach(KEYWORD)


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/speex/speex_config_types.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/speex/speex_config_types.h)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/speex/speex_config_types.h
        DESTINATION include/speex
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix ${prefix}/bin)
set(libdir ${prefix}/lib)
set(includedir ${prefix}/include)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/speex.pc.in ${CMAKE_CURRENT_BINARY_DIR}/speex.pc)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/speexdsp.pc.in ${CMAKE_CURRENT_BINARY_DIR}/speexdsp.pc)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/speex.pc ${CMAKE_CURRENT_BINARY_DIR}/speexdsp.pc DESTINATION lib/pkgconfig)


add_definitions(-DHAVE_CONFIG_H)

include_directories(
	include
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_BINARY_DIR}/speex
)

add_subdirectory(include)
add_subdirectory(libspeex)
