/*
 * motor_current.h
 *
 * Three-shunt phase current sampling (ADC_IN0..2 -> Ia, Ib, Ic on PA0..PA2).
 *
 * ADC is hardware-triggered by TIM1 TRGO (falling edge, see the .ioc) so each
 * scan happens on its own without CPU intervention; this module just reads
 * whatever the DMA last wrote. Requires ADC scan of 3 channels (rank order
 * IN0, IN1, IN2) into a circular DMA buffer, and TIM1's Trigger Output (TRGO)
 * set to "Update Event" so the trigger lands away from switching transients.
 *
 * Created on: Sep 21, 2026
 *      Author: CT
 */

#ifndef INC_MOTOR_CURRENT_H_
#define INC_MOTOR_CURRENT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** VDDA in volts (ADC full-scale reference). */
#ifndef MOTOR_CURRENT_VREF_V
#define MOTOR_CURRENT_VREF_V (3.3f)
#endif

/** Shunt resistance (ohms). */
#ifndef MOTOR_CURRENT_R_SHUNT_OHM
#define MOTOR_CURRENT_R_SHUNT_OHM (0.010f)
#endif

/** Total sense path gain: volts at the ADC pin per amp of phase current. */
#ifndef MOTOR_CURRENT_V_PER_AMP
#define MOTOR_CURRENT_V_PER_AMP (MOTOR_CURRENT_R_SHUNT_OHM * 20.0f)
#endif

/** Starts the ADC+DMA scan; conversions then happen on the TIM1 TRGO trigger. */
void MotorCurrent_Init(void);

/** Averages `samples` readings as the zero-current offset. PWM must be at neutral (no current flowing). */
void MotorCurrent_CalibrateOffsets(uint16_t samples);

/** Latest raw counts [0..4095], order A, B, C. */
void MotorCurrent_GetRaw(uint16_t *ia, uint16_t *ib, uint16_t *ic);

/** Amperes after offset and scaling (sign depends on shunt wiring). */
void MotorCurrent_GetAmps(float *ia, float *ib, float *ic);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_CURRENT_H_ */
