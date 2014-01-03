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
	fftwrap.c
	filterbank.c
	filters.c
	gain_table.c
	gain_table_lbr.c
	hexc_10_32_table.c
	hexc_table.c
	high_lsp_tables.c
	jitter.c
	kiss_fft.c
	kiss_fftr.c
	lpc.c
	lsp.c
	lsp_tables_nb.c
	ltp.c
	mdf.c
	modes.c
	modes_wb.c
	nb_celp.c
	preprocess.c
	quant_lsp.c
	sb_celp.c
	smallft.c
	speex.c
	speexdsp.c
	speex_callbacks.c
	speex_header.c
	stereo.c
	vbr.c
	vq.c
	window.c
	../include/speex/speex.h
	../include/speex/speex_bits.h
	../include/speex/speex_callbacks.h
	../include/speex/speex_echo.h
	../include/speex/speex_header.h
	../include/speex/speex_jitter.h
	../include/speex/speex_preprocess.h
	../include/speex/speex_stereo.h
	../include/speex/speex_types.h
	cb_search.h
	fftwrap.h
	filters.h
	kiss_fft.h
	kiss_fftr.h
	lpc.h
	lsp.h
	ltp.h
	math_approx.h
	modes.h
	nb_celp.h
	pseudofloat.h
	quant_lsp.h
	sb_celp.h
	stack_alloc.h
	vbr.h
	vq.h
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

add_library(libspeex SHARED ${LIBSPEEX_SOURCE_FILES})
set_target_properties(libspeex PROPERTIES VERSION 1.2 SOVERSION 6)

target_link_libraries(libspeex ${LIBS})

install(TARGETS libspeex
		COMPONENT COMP_libspeex
        DESTINATION ${LIB_INSTALL_DIR}
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)



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
	#resample_neon.c
	scal.c
	smallft.c
	speexdsp.c
	../include/speex/speex.h
	../include/speex/speex_bits.h
	../include/speex/speex_buffer.h
	../include/speex/speex_echo.h
	../include/speex/speex_jitter.h
	../include/speex/speex_preprocess.h
	../include/speex/speex_resampler.h
	../include/speex/speex_types.h
	arch.h
	fftwrap.h
	filterbank.h
	fixed_debug.h
	fixed_generic.h
	kiss_fft.h
	kiss_fftr.h
	math_approx.h
	os_support.h
	pseudofloat.h
	smallft.h
	_kiss_fft_guts.h
)

if(WIN32)
set(LIBSPEEXDSP_SOURCE_FILES
	${LIBSPEEXDSP_SOURCE_FILES}
	../win32/config.h
	libspeexdsp.def
)
endif(WIN32)

add_library(libspeexdsp SHARED ${LIBSPEEXDSP_SOURCE_FILES})
set_target_properties(libspeexdsp PROPERTIES VERSION 1.2 SOVERSION 6)

target_link_libraries(libspeexdsp ${LIBS})

install(TARGETS libspeexdsp
		COMPONENT COMP_libspeex
        DESTINATION ${LIB_INSTALL_DIR}
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
