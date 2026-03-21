/**
 * @file foc_transform.h
 * @brief Clarke / Park / inverse Park / inverse Clarke + virtual electrical angle.
 *
 * Angle theta is the electrical angle (rad) for the rotor flux frame.
 * Virtual encoder: theta integrates at omega_e * dt (open-loop); replace with
 * real encoder theta later without changing the transforms.
 *
 * Convention (common ST style):
 *   Park:      id =  iα*cos(θ) + iβ*sin(θ)
 *              iq = -iα*sin(θ) + iβ*cos(θ)
 *   Inv Park:  vα =  vd*cos(θ) - vq*sin(θ)
 *              vβ =  vd*sin(θ) + vq*cos(θ)
 *   Clarke (ia+ib+ic=0): iα = ia, iβ = (ia + 2*ib) / sqrt(3)
 *   Inv Clarke: va = vβ, vb = -vβ/2 + (√3/2)*vα, vc = -vβ/2 - (√3/2)*vα
 */
#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * TIM1 center-aligned: PWM carrier = f_tim / (2*ARR); HAL update (UIF) fires twice per carrier
 * (overflow + underflow). TRGO=UPDATE therefore strobes TIM2/3 twice per PWM period — slave
 * trigger/reset on those timers corrupts PWM (buzz, no torque). Slaves must be disabled.
 */
#ifndef FOC_PWM_CARRIER_HZ
#define FOC_PWM_CARRIER_HZ (20000.0f)
#endif
#ifndef FOC_TIM1_UPDATE_ISR_HZ
#define FOC_TIM1_UPDATE_ISR_HZ (2.0f * FOC_PWM_CARRIER_HZ)
#endif

/** Electrical angle step per TIM1 PeriodElapsed ISR: theta += omega_e * FOC_ANGLE_DT_S */
#define FOC_ANGLE_DT_S (1.0f / FOC_TIM1_UPDATE_ISR_HZ)

void Foc_Init(void);

/** Open-loop electrical speed (rad/s); e.g. 2*pi*5 for 5 electrical Hz. */
void FocVirtualAngle_SetOmegaElectrical(float omega_e_rad_s);

/** Set electrical angle (rad), wrap [0,2pi); updates internal sin/cos. Use for alignment before omega_e > 0. */
void FocVirtualAngle_SetThetaRad(float theta_rad);

/** Open-loop dq voltage (same units as per-unit half-bus on phases); Vd=0, Vq>0 + positive omega_e => one direction. */
void FocOpenLoop_SetVdq(float vd, float vq);

/** Integrate angle by dt (seconds); normally called from ISR via Foc_OnTim1Update. */
void FocVirtualAngle_Step(float dt_s);

float FocVirtualAngle_GetThetaRad(void);
void FocVirtualAngle_GetSinCos(float *sin_theta, float *cos_theta);

void FocClarke(float ia, float ib, float ic, float *ialpha, float *ibeta);
void FocPark(float ialpha, float ibeta, float theta, float *id, float *iq);
void FocInversePark(float vd, float vq, float theta, float *valpha, float *vbeta);
void FocInverseClarke(float valpha, float vbeta, float *va, float *vb, float *vc);

/** Clarke + Park using the virtual angle (for current feedback + logging). */
void FocAbcToDq(float ia, float ib, float ic, float *id, float *iq);

/** Inverse Park + inverse Clarke: dq -> phase voltages (same units as vd/vq). */
void FocDqToAbc(float vd, float vq, float theta, float *va, float *vb, float *vc);

/** dq -> abc using the current virtual angle. */
void FocDqToAbcVirtual(float vd, float vq, float *va, float *vb, float *vc);

/** Called from HAL_TIM_PeriodElapsedCallback when htim is TIM1. */
void Foc_OnTim1Update(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRANSFORM_H */
