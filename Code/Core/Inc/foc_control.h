/**
 * @file foc_control.h
 * @brief Simple dq current-vector FOC: Clarke + Park, PI on id/iq, inverse Park + inverse Clarke -> PWM.
 *
 * Runs from TIM1 update ISR (same electrical angle as voltage). Tune gains and I refs for your motor.
 */
#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void FocControl_Init(void);

/** Electrical speed for open-loop angle (rad/s); e.g. 2*pi*3 for 3 Hz electrical. */
void FocControl_SetElectricalSpeedRadS(float omega_e_rad_s);

/** dq current references (A). id=0, iq>0 is typical torque command. */
void FocControl_SetIdqRef(float id_ref_a, float iq_ref_a);

/** When zero, PWM is forced to neutral (no torque); ISR still services TIM1. */
void FocControl_SetEnable(uint8_t enable);

/** Call once per TIM1 update (after Foc_OnTim1Update advances angle, or call alone — see main). */
void FocControl_StepIsr(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_H */
