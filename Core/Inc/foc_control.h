/*
 * foc_control.h
 *
 * Closed-loop dq current-vector FOC: measured phase currents -> Clarke + Park
 * -> PI on id/iq -> inverse Park + inverse Clarke -> PWM duty on TIM1 CH1..3.
 *
 * The electrical angle is a virtual open-loop ramp (FOC_Control_SetElectricalSpeedRadS),
 * used for both the measurement Park and the output inverse Park so the current
 * regulators always work in the same rotating frame they were measured in.
 *
 * Call FOC_Control_Step() once per TIM1 update event (matching FOC_CONTROL_LOOP_HZ).
 *
 * Created on: Sep 21, 2026
 *      Author: CT
 */

#ifndef INC_FOC_CONTROL_H_
#define INC_FOC_CONTROL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Call rate of FOC_Control_Step() in Hz (must match how it's wired to the TIM1 update ISR). */
#ifndef FOC_CONTROL_LOOP_HZ
#define FOC_CONTROL_LOOP_HZ (20000.0f)
#endif

/** DC bus voltage (V); sets the 0..1 duty mapping for the phase voltages. */
#ifndef FOC_VBUS_V
#define FOC_VBUS_V (12.0f)
#endif

/** Id/Iq PI gains (V per A). Tune for your motor. */
#ifndef FOC_ID_KP
#define FOC_ID_KP (1.0f)
#endif
#ifndef FOC_ID_KI
#define FOC_ID_KI (100.0f)
#endif
#ifndef FOC_IQ_KP
#define FOC_IQ_KP (1.0f)
#endif
#ifndef FOC_IQ_KI
#define FOC_IQ_KI (100.0f)
#endif

void FOC_Control_Init(void);

/** Electrical speed for the open-loop angle (rad/s); e.g. 2*pi*3 for 3 Hz electrical. */
void FOC_Control_SetElectricalSpeedRadS(float omega_e_rad_s);

/** dq current references (A). id=0, iq!=0 is a typical torque command. */
void FOC_Control_SetIdqRef(float id_ref_a, float iq_ref_a);

/** When zero, PWM is forced to neutral (50% all phases, no torque) and PI state is reset. */
void FOC_Control_SetEnable(uint8_t enable);

/** Runs one control update: measure -> regulate -> drive PWM. Call from the TIM1 update ISR. */
void FOC_Control_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_CONTROL_H_ */
