if(WIN32)
set(LIBS ws2_32)
endif(WIN32)

set(LIBSPEEX_SOURCE_FILES
	bits.c
	cb_search.c
	exc_10_16_table.c
	exc_10_32_table.c
	exc_20_32_table.c
	exc_5_256_table.c
	exc_5_64_table.c
	exc_8_128_table.c
	filters.c
	gain_table.c
	gain_table_lbr.c
	hexc_10_32_table.c
	hexc_table.c
	high_lsp_tables.c
	kiss_fft.c
	kiss_fftr.c
	lpc.c
	lsp.c
	lsp_tables_nb.c
	ltp.c
	modes.c
	modes_wb.c
	nb_celp.c
	quant_lsp.c
	sb_celp.c
	smallft.c
	speex.c
	speex_callbacks.c
	speex_header.c
	stereo.c
	vbr.c
	vq.c
	window.c
	_kiss_fft_guts.h
	kiss_fft.h
	kiss_fftr.h
)

add_definitions(
	-DHAVE_CONFIG_H
)

if(WIN32)
set(LIBSPEEX_SOURCE_FILES
	${LIBSPEEX_SOURCE_FILES}
	../win32/config.h
	libspeex.def
)
endif(WIN32)

add_library(speex SHARED ${LIBSPEEX_SOURCE_FILES})
set_target_properties(speex PROPERTIES VERSION 1.2 SOVERSION 6)

target_link_libraries(speex ${LIBS})

install(TARGETS speex
	DESTINATION ${LIB_INSTALL_DIR}
	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
if(WIN32)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Debug/speex.pdb
	DESTINATION ${LIB_INSTALL_DIR}
	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
endif()
endif(WIN32)



set(LIBSPEEXDSP_SOURCE_FILES
	buffer.c
	fftwrap.c
	filterbank.c
	jitter.c
	kiss_fft.c
	kiss_fftr.c
	mdf.c
	preprocess.c
	resample.c
	resample_neon.c
	scal.c
	smallft.c
	speexdsp.c
	_kiss_fft_guts.h
	kiss_fft.h
	kiss_fftr.h
)

if(WIN32)
set(LIBSPEEXDSP_SOURCE_FILES
	${LIBSPEEXDSP_SOURCE_FILES}
	../win32/config.h
	libspeexdsp.def
)
endif(WIN32)

add_library(speexdsp SHARED ${LIBSPEEXDSP_SOURCE_FILES})
set_target_properties(speexdsp PROPERTIES VERSION 1.2 SOVERSION 6)

target_link_libraries(speexdsp ${LIBS})

install(TARGETS speexdsp
	DESTINATION ${LIB_INSTALL_DIR}
	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
if(WIN32)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Debug/speexdsp.pdb
	DESTINATION ${LIB_INSTALL_DIR}
	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
endif()
endif(WIN32)
