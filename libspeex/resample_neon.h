/* Copyright (C) 2011 Belledonne Communications SARL
 */
/**
   @file resample_neon.h
   @brief Resampler functions (Neon version)
*/
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   
   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   
   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   
   - Neither the name of the Xiph.org Foundation nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <arm_neon.h>

extern int libspeex_cpu_features;

#define OVERRIDE_INNER_PRODUCT_SINGLE
static inline spx_int32_t inner_product_single(const spx_int16_t *a, const spx_int16_t *b, unsigned int len){
	spx_int32_t ret;

	__asm  (
			/* save len */
			"mov r4, %3  \n\t"
			/* clear q0 */
			"vmov.i16 q0, #0 \n\t"
			/* load 8 values from a in q1*/
			"1: vld1.16 {d2,d3},[%1]! \n\t"
			/* load 8 values from b in q2 */
			"vld1.16 {d4,d5},[%2]! \n\t"
			/* multiply-add 4 first values into q0 */
			"vmlal.s16 q0, d2, d4 \n\t"
			/* multiply-add 4 last into q0 */
			"vmlal.s16 q0, d3, d5 \n\t"
			/* decrement len by 8 */
			"subs r4, r4, #8 \n\t"
			/* loop if needed */
            "bne 1b \n\t"
			/* add individual 32 bits results */
			"vpadd.s32 d0, d0, d1 \n\t"
			/* store result in ret  */
			"vmov.32 %0, d0[0] \n\t"
			: "=r"(ret), "+r"(a), "+r"(b)/* out */
			: "r"(len)/*in*/
			: "q0", "q1", "q2", "r4" /*modified*/
			);

	return ret;
}

#define OVERRIDE_INTERPOLATE_PRODUCT_SINGLE
static inline spx_int32_t interpolate_product_single(const spx_int16_t *a, const spx_int16_t *b, unsigned int len, const spx_uint32_t oversample, spx_int16_t *frac){
if (!(libspeex_cpu_features & SPEEX_LIB_CPU_FEATURE_NEON)) {
		/*no neon*/
		/*from speex resampler.c*/
		int accum[4] = {0,0,0,0};
		int j;
		for(j=0;j<len;j++) {
			const short int curr_in=a[j];
			accum[0] += MULT16_16(curr_in,*(b  + j*oversample) );
			accum[1] += MULT16_16(curr_in,*((b + 1) + j*oversample));
			accum[2] += MULT16_16(curr_in,*((b + 2) + j*oversample));
			accum[3] += MULT16_16(curr_in,*((b + 3) + j*oversample));
		}
		return MULT16_32_Q15(frac[0],SHR32(accum[0], 1)) + MULT16_32_Q15(frac[1],SHR32(accum[1], 1)) + MULT16_32_Q15(frac[2],SHR32(accum[2], 1)) + MULT16_32_Q15(frac[3],SHR32(accum[3], 1));

	}

	int i,j;
	int32x4_t sum = vdupq_n_s32 (0);
	int16x4_t f=vld1_s16 ((const int16_t*)frac);
	int32x4_t f2=vmovl_s16(f);
	
	f2=vshlq_n_s32(f2,16);

	for(i=0,j=0;i<len;i+=2,j+=(2*oversample)) {
		sum=vqdmlal_s16(sum,vld1_dup_s16 ((const int16_t*)(a+i)), vld1_s16 ((const int16_t*)(b+j)));
		sum=vqdmlal_s16(sum,vld1_dup_s16 ((const int16_t*)(a+i+1)), vld1_s16 ((const int16_t*)(b+j+oversample)));
	}
	sum=vshrq_n_s32(sum,1);
	sum=vqdmulhq_s32(f2,sum);
	sum=vshrq_n_s32(sum,1);
	
	int32x2_t tmp=vadd_s32(vget_low_s32(sum),vget_high_s32(sum));
	tmp=vpadd_s32(tmp,tmp);
	
	return vget_lane_s32 (tmp,0);
}

