/**
 * @file motor_current.h
 * @brief Three-shunt phase current sampling (ADC CH0..2 -> Ia, Ib, Ic).
 *
 * DMA circular buffer order matches Cube scan: IN0=A, IN1=B, IN2=C (PA0..PA2).
 * Tune MOTOR_CURRENT_* defines to your sense gain and shunt.
 */
#ifndef MOTOR_CURRENT_H
#define MOTOR_CURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** VDDA in volts (used for ADC full-scale). */
#ifndef MOTOR_CURRENT_VREF_V
#define MOTOR_CURRENT_VREF_V (3.3f)
#endif

/** Shunt resistance (ohms). */
#ifndef MOTOR_CURRENT_R_SHUNT_OHM
#define MOTOR_CURRENT_R_SHUNT_OHM (0.010f)
#endif

/** Total sense path: V_adc per amp (R_shunt * INA gain, or V/A from datasheet). */
#ifndef MOTOR_CURRENT_V_PER_AMP
#define MOTOR_CURRENT_V_PER_AMP (MOTOR_CURRENT_R_SHUNT_OHM * 20.0f)
#endif

void MotorCurrent_Init(void);

/**
 * @brief Sample N times and store midpoint as zero-current offset (no PWM / no torque).
 */
void MotorCurrent_CalibrateOffsets(uint16_t samples);

/** Latest raw counts [0..4095], same order as ADC scan: A, B, C. */
void MotorCurrent_GetRaw(uint16_t *ia, uint16_t *ib, uint16_t *ic);

/** Amperes after offset and scaling (sign depends on wiring). */
void MotorCurrent_GetAmps(float *ia, float *ib, float *ic);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CURRENT_H */
