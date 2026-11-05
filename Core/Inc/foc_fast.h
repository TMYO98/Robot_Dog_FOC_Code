/*
 * foc_fast.h
 *
 * Fast loop: runs once per PWM period (20 kHz) from the ADC DMA
 * transfer-complete interrupt, i.e. right after this period's phase currents
 * have been sampled. Integer math only.
 *
 * Per step:
 *   phase currents (drop the phase with the highest duty, rebuild it from the
 *   other two) -> overcurrent check -> AS5147P angle over SPI (one 16-bit
 *   frame, pipelined) -> multi-turn position -> electrical angle
 *   -> Clarke/Park -> d/q PI -> vector voltage limit -> inverse Park
 *   -> SVPWM (min/max injection) -> TIM1 CCR1..3.
 *
 * Units used in g_foc_fast:
 *   current  ADC counts (FOC_COUNTS_PER_AMP per amp), sign-corrected
 *   voltage  millivolts
 *   angle    uint16_t, 65536 = one electrical revolution
 *   position encoder counts, 16384 per motor revolution, raw encoder direction
 *
 * Ownership: "cmd"/"param" fields are written by the slow loop, everything
 * else by the fast loop. Each field is a single word, so reads are atomic.
 *
 *      Author: CT
 */

#ifndef INC_FOC_FAST_H_
#define INC_FOC_FAST_H_

#include <stdint.h>

#include "stm32f0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  FOC_FAST_MODE_OFF = 0,     /* bridge off (MOE = 0), still sampling + tracking the encoder */
  FOC_FAST_MODE_OFFSET_CAL,  /* averaging zero-current ADC offsets, then back to OFF */
  FOC_FAST_MODE_VOLTAGE,     /* open-loop vd/vq (calibration) */
  FOC_FAST_MODE_CURRENT      /* closed-loop id/iq */
} FocFastMode_t;

/* Fault bits (also sent on CAN as one byte) */
#define FOC_FAULT_OVERCURRENT   0x01u
#define FOC_FAULT_ENCODER       0x02u
#define FOC_FAULT_UNDERVOLTAGE  0x04u
#define FOC_FAULT_OVERVOLTAGE   0x08u
#define FOC_FAULT_TIMEOUT       0x10u
#define FOC_FAULT_CALIBRATION   0x20u
#define FOC_FAULT_SYSTEM        0x40u   /* DMA error, init failure */
#define FOC_FAULT_MAGNET        0x80u   /* AS5147P reports magnet too weak / strong */

typedef struct
{
  /* ---- cmd (slow loop -> fast loop) ---- */
  volatile uint8_t  mode;             /* FocFastMode_t; use FocFast_SetMode() */
  volatile uint8_t  angle_override;   /* 1: use theta_override instead of the encoder */
  volatile uint16_t theta_override;
  volatile int32_t  id_ref;           /* counts */
  volatile int32_t  iq_ref;           /* counts */
  volatile int32_t  vd_cmd;           /* mV, voltage mode */
  volatile int32_t  vq_cmd;           /* mV, voltage mode */

  /* ---- param (slow loop -> fast loop) ---- */
  volatile int32_t  kp_q12;           /* mV per count, Q12 */
  volatile int32_t  ki_q12;           /* mV per count per sample, Q12 */
  volatile int32_t  vmax_mv;          /* max |Vdq| */
  volatile int32_t  duty_scale_q16;   /* ARR * 65536 / Vbus_mV */
  volatile int32_t  theta_adv_meas;   /* angle advance for the Park transform */
  volatile int32_t  theta_adv_out;    /* angle advance for the inverse Park */
  volatile int32_t  oc_trip_counts;
  volatile uint16_t enc_offset;       /* electrical angle at encoder zero */
  volatile int8_t   enc_dir;          /* +1 / -1 */
  volatile uint8_t  pole_pairs;

  /* ---- state (fast loop -> everyone) ---- */
  volatile uint32_t faults;           /* FOC_FAULT_* bits; any bit forces the bridge off */
  volatile uint16_t adc_offset[3];
  volatile uint8_t  offset_cal_done;
  volatile uint8_t  enc_valid;
  volatile uint16_t enc_angle;        /* last good 14-bit angle */
  volatile uint16_t theta_enc;        /* electrical angle from the encoder (no advance) */
  volatile int32_t  pos_raw;          /* multi-turn encoder counts */
  volatile int32_t  ia, ib, ic;       /* counts */
  volatile int32_t  id, iq;           /* counts */
  volatile int32_t  vd, vq;           /* mV, last output */
  volatile uint16_t vbus_raw;

  /* ---- diagnostics ---- */
  volatile uint32_t step_count;
  volatile uint32_t enc_error_count;
  volatile uint32_t overrun_count;    /* fast step still running when the next sample landed */
  volatile uint32_t slow_overrun_count;
  volatile uint16_t isr_end_ticks;    /* step end, in TIM1 ticks after the PWM peak (must stay < 2*ARR) */
  volatile uint16_t isr_end_ticks_max;
} FocFast_t;

extern FocFast_t g_foc_fast;

/** Set by the slow loop while it runs; used for overrun detection. */
extern volatile uint8_t g_foc_slow_busy;

/** Configures TIM1 CH4 / ADC trigger / outputs and starts sampling (bridge stays off).
 *  Must be called after the MX_*_Init() functions and before anything else uses SPI1. */
void FocFast_Init(void);

/** Switches mode. Entering VOLTAGE/CURRENT resets the PI state and turns the bridge on
 *  (unless a fault is latched); OFF/OFFSET_CAL turn it off. */
void FocFast_SetMode(FocFastMode_t mode);

/** Latches fault bits and turns the bridge off. Safe from any context. */
void FocFast_Trip(uint32_t fault_bits);

/** Clears all latched faults (the bridge stays off). */
void FocFast_ClearFaults(void);

/** Hook for DMA1_Channel1_IRQHandler. Returns 1 if the interrupt was handled. */
uint8_t FocFast_DmaIrqHandler(void);

/** Immediately disables the bridge outputs (MOE = 0). For fault/exception handlers. */
static inline void FocFast_EmergencyOff(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_FAST_H_ */
