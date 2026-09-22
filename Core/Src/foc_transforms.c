/*
 * foc_transforms.c
 */

#include "foc_transforms.h"

#include <math.h>

#ifndef M_SQRT3
#define M_SQRT3 1.7320508075688772f
#endif

static const float ONE_OVER_SQRT3 = 1.0f / M_SQRT3;

void FOC_ClarkeTransform(float ia, float ib, float ic, float *ialpha, float *ibeta)
{
  (void)ic;
  *ialpha = ia;
  *ibeta = (ia + 2.0f * ib) * ONE_OVER_SQRT3;
}

void FOC_ParkTransform(float ialpha, float ibeta, float theta, float *id, float *iq)
{
  float c = cosf(theta);
  float s = sinf(theta);
  *id = ialpha * c + ibeta * s;
  *iq = -ialpha * s + ibeta * c;
}

void FOC_InverseParkTransform(float id, float iq, float theta, float *ialpha, float *ibeta)
{
  float c = cosf(theta);
  float s = sinf(theta);
  *ialpha = id * c - iq * s;
  *ibeta = id * s + iq * c;
}

void FOC_InverseClarkeTransform(float valpha, float vbeta, float *va, float *vb, float *vc)
{
  const float half = 0.5f;
  const float sqrt3_2 = 0.5f * M_SQRT3;
  *va = valpha;
  *vb = -half * valpha + sqrt3_2 * vbeta;
  *vc = -half * valpha - sqrt3_2 * vbeta;
}
