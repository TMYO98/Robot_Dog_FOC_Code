/*
 * foc_math.h
 *
 * Fixed-point helpers for the fast loop. The STM32F042 (Cortex-M0) has no FPU,
 * so floats cost ~50-100 cycles per operation and libm sinf/cosf ~1000+.
 * The fast loop uses integers only:
 *
 *   angle   uint16_t, 65536 = one electrical revolution (wraps for free)
 *   sin/cos Q15 (32767 ~= 1.0), 256-entry table + linear interpolation
 *
 *      Author: CT
 */

#ifndef INC_FOC_MATH_H_
#define INC_FOC_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_Q15_ONE_OVER_SQRT3   18919   /* 1/sqrt(3) */
#define FOC_Q15_SQRT3_OVER_2     28378   /* sqrt(3)/2 */
#define FOC_ANGLE_QUARTER        16384u

/** sin(2*pi*i/256) in Q15, i = 0..256 (last entry repeats the first). */
extern const int16_t k_foc_sin_q15[257];

/**
 * sin(angle) in Q15, where the angle is 0..65535 for 0..2*pi.
 * The top 8 bits pick a table entry, the low 8 bits interpolate linearly to the
 * next one: s0 + (s1 - s0) * frac / 256. Maximum error is about 0.01 % of full
 * scale, much finer than the ADC.
 */
static inline int32_t foc_sin_q15(uint16_t angle)
{
  const uint32_t i = (uint32_t)angle >> 8;
  const int32_t frac = (int32_t)(angle & 0xFFu);
  const int32_t s0 = k_foc_sin_q15[i];
  const int32_t s1 = k_foc_sin_q15[i + 1u];
  return s0 + (((s1 - s0) * frac) >> 8);
}

/** sin and cos together, using cos(a) = sin(a + 90 deg) (a quarter turn = 16384). */
static inline void foc_sincos_q15(uint16_t angle, int32_t *s, int32_t *c)
{
  *s = foc_sin_q15(angle);
  *c = foc_sin_q15((uint16_t)(angle + FOC_ANGLE_QUARTER));
}

/** Limits x to the range [lo, hi]. */
static inline int32_t foc_clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/**
 * 1 if the 16-bit word has an even number of set bits (AS5147P frames use even
 * parity). Each XOR-fold halves the width: after folding 16 -> 8 -> 4 -> 2 -> 1,
 * bit 0 holds the XOR of all 16 bits, which is 1 for an odd count.
 */
static inline uint32_t foc_parity_even16(uint32_t x)
{
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return (~x) & 1u;
}

/** floor(sqrt(x)), integer only. */
uint32_t foc_isqrt32(uint32_t x);

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_MATH_H_ */
