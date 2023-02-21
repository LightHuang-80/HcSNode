/*
 * scurve.c
 *
 *  Created on: Oct 28, 2020
 *      Author: Administrator
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "planner.h"

#define A(CODE) " " CODE "\n\t"

  int32_t __attribute__((used)) bezier_A __asm__("bezier_A");    // A coefficient in Bézier speed curve with alias for assembler
  int32_t __attribute__((used)) bezier_B __asm__("bezier_B");    // B coefficient in Bézier speed curve with alias for assembler
  int32_t __attribute__((used)) bezier_C __asm__("bezier_C");    // C coefficient in Bézier speed curve with alias for assembler
  uint32_t __attribute__((used)) bezier_F __asm__("bezier_F");   // F coefficient in Bézier speed curve with alias for assembler
  uint32_t __attribute__((used)) bezier_AV __asm__("bezier_AV"); // AV coefficient in Bézier speed curve with alias for assembler
  bool bezier_2nd_half;    // =false If Bézier curve has been initialized or not

// For all the other 32bit CPUs
void calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av) {
  // Calculate the Bézier coefficients
  bezier_A =  768 * (v1 - v0);
  bezier_B = 1920 * (v0 - v1);
  bezier_C = 1280 * (v1 - v0);
  bezier_F =  128 * v0;
  bezier_AV = av;
}

int32_t eval_bezier_curve(const uint32_t curr_step)
{
  int32_t alo = bezier_F;

#if defined(__arm__) || defined(__thumb__)

  // For ARM Cortex M3/M4 CPUs, we have the optimized assembler version, that takes 43 cycles to execute
  uint32_t flo = 0;
  uint32_t fhi = bezier_AV * curr_step;
  uint32_t t = fhi;
  int32_t ahi = 0;
  int32_t A = bezier_A;
  int32_t B = bezier_B;
  int32_t C = bezier_C;

  __asm__ __volatile__(
	  ".syntax unified" "\n\t"              // is to prevent CM0,CM1 non-unified syntax
	  A("lsrs  %[ahi],%[alo],#1")           // a  = F << 31      1 cycles
	  A("lsls  %[alo],%[alo],#31")          //                   1 cycles
	  A("umull %[flo],%[fhi],%[fhi],%[t]")  // f *= t            5 cycles [fhi:flo=64bits]
	  A("umull %[flo],%[fhi],%[fhi],%[t]")  // f>>=32; f*=t      5 cycles [fhi:flo=64bits]
	  A("lsrs  %[flo],%[fhi],#1")           //                   1 cycles [31bits]
	  A("smlal %[alo],%[ahi],%[flo],%[C]")  // a+=(f>>33)*C;     5 cycles
	  A("umull %[flo],%[fhi],%[fhi],%[t]")  // f>>=32; f*=t      5 cycles [fhi:flo=64bits]
	  A("lsrs  %[flo],%[fhi],#1")           //                   1 cycles [31bits]
	  A("smlal %[alo],%[ahi],%[flo],%[B]")  // a+=(f>>33)*B;     5 cycles
	  A("umull %[flo],%[fhi],%[fhi],%[t]")  // f>>=32; f*=t      5 cycles [fhi:flo=64bits]
	  A("lsrs  %[flo],%[fhi],#1")           // f>>=33;           1 cycles [31bits]
	  A("smlal %[alo],%[ahi],%[flo],%[A]")  // a+=(f>>33)*A;     5 cycles
	  A("lsrs  %[alo],%[ahi],#6")           // a>>=38            1 cycles
	  : [alo]"+r"( alo ) ,
		[flo]"+r"( flo ) ,
		[fhi]"+r"( fhi ) ,
		[ahi]"+r"( ahi ) ,
		[A]"+r"( A ) ,  // <== Note: Even if A, B, C, and t registers are INPUT ONLY
		[B]"+r"( B ) ,  //  GCC does bad optimizations on the code if we list them as
		[C]"+r"( C ) ,  //  such, breaking this function. So, to avoid that problem,
		[t]"+r"( t )    //  we list all registers as input-outputs.
	  :
	  : "cc"
	);
#endif

  return alo;
}
