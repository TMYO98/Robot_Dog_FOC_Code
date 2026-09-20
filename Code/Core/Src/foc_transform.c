/**
 * @file foc_transform.c
 */
#include "foc_transform.h"
#include "tim.h"
#include "stm32f0xx_hal.h"
#include <math.h>

#ifndef M_SQRT3
#define M_SQRT3 1.7320508075688772f
#endif

static const float ONE_OVER_SQRT3 = 0.5773502691896258f;

static float s_theta_rad;
static float s_omega_e_rad_s;
static float s_sin_theta;
static float s_cos_theta;
static float s_vd_open;
static float s_vq_open;

void Foc_Init(void)
{
  s_theta_rad = 0.0f;
  s_omega_e_rad_s = 0.0f;
  s_sin_theta = sinf(s_theta_rad);
  s_cos_theta = cosf(s_theta_rad);
  s_vd_open = 0.0f;
  s_vq_open = 0.0f;
}


void FocClarke(float ia, float ib, float ic, float *ialpha, float *ibeta)
{
  (void)ic;
  *ialpha = ia;
  *ibeta = (ia + 2.0f * ib) * ONE_OVER_SQRT3;
}

void FocPark(float ialpha, float ibeta, float theta, float *id, float *iq)
{
  float c = cosf(theta);
  float s = sinf(theta);
  *id = ialpha * c + ibeta * s;
  *iq = -ialpha * s + ibeta * c;
}

void FocInversePark(float vd, float vq, float theta, float *valpha, float *vbeta)
{
  float c = cosf(theta);
  float s = sinf(theta);
  *valpha = vd * c - vq * s;
  *vbeta = vd * s + vq * c;
}

void FocInverseClarke(float valpha, float vbeta, float *va, float *vb, float *vc)
{
  const float half = 0.5f;
  const float sqrt3_2 = 0.5f * M_SQRT3;
  *va = vbeta;
  *vb = -half * vbeta + sqrt3_2 * valpha;
  *vc = -half * vbeta - sqrt3_2 * valpha;
}

void FocAbcToDq(float ia, float ib, float ic, float *id, float *iq)
{
  float ialpha, ibeta;
  FocClarke(ia, ib, ic, &ialpha, &ibeta);
  FocPark(ialpha, ibeta, s_theta_rad, id, iq);
}

void FocDqToAbc(float vd, float vq, float theta, float *va, float *vb, float *vc)
{
  float valpha, vbeta;
  FocInversePark(vd, vq, theta, &valpha, &vbeta);
  FocInverseClarke(valpha, vbeta, va, vb, vc);
}

void FocDqToAbcVirtual(float vd, float vq, float *va, float *vb, float *vc)
{
  FocDqToAbc(vd, vq, s_theta_rad, va, vb, vc);
}

static void foc_wrap_theta_rad(float *theta)
{
  const float two_pi = (float)(2.0 * M_PI);
  while (*theta >= two_pi)
  {
    *theta -= two_pi;
  }
  while (*theta < 0.0f)
  {
    *theta += two_pi;
  }
}

void FocVirtualAngle_SetOmegaElectrical(float omega_e_rad_s)
{
  s_omega_e_rad_s = omega_e_rad_s;
}

void FocVirtualAngle_SetThetaRad(float theta_rad)
{
  s_theta_rad = theta_rad;
  foc_wrap_theta_rad(&s_theta_rad);
  s_sin_theta = sinf(s_theta_rad);
  s_cos_theta = cosf(s_theta_rad);
}

void FocOpenLoop_SetVdq(float vd, float vq)
{
  s_vd_open = vd;
  s_vq_open = vq;
}

void FocVirtualAngle_Step(float dt_s)
{
  s_theta_rad += s_omega_e_rad_s * dt_s;
  foc_wrap_theta_rad(&s_theta_rad);
  s_sin_theta = sinf(s_theta_rad);
  s_cos_theta = cosf(s_theta_rad);
}

float FocVirtualAngle_GetThetaRad(void)
{
  return s_theta_rad;
}

void FocVirtualAngle_GetSinCos(float *sin_theta, float *cos_theta)
{
  if (sin_theta != NULL)
  {
    *sin_theta = s_sin_theta;
  }
  if (cos_theta != NULL)
  {
    *cos_theta = s_cos_theta;
  }
}

void Foc_OnTim1Update(void)
{
  FocVirtualAngle_Step(FOC_ANGLE_DT_S);
}
