/*
 * foc_fast.c
 *
 * See foc_fast.h. This file is always compiled with -O2 so the 20 kHz loop
 * keeps its timing even in the Debug configuration.
 *
 * Timing (TIM1 center-aligned, ARR = 1199, 48 MHz):
 *
 *   CNT  0 ............ ARR (peak) ............ 0 (valley) ............ ARR
 *                 CH4 ^ ADC trigger, LEAD ticks before the peak
 *                       [IN0][IN1][IN2] ~4.3 us, centred on the peak
 *                                 ^ DMA TC -> FocFast_Step()
 *                                   must finish before the next peak, where
 *                                   the new CCR values are loaded (RCR = 1)
 *
 * PWM mode 1: the high side is on while CNT < CCR, so every low-side FET is on
 * around the peak; that is where the low-side shunts carry the phase currents.
 *
 *      Author: CT
 */

#pragma GCC optimize ("O2")

#include "foc_fast.h"

#include "foc_config.h"
#include "foc_math.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"

#define FOC_HALF_ARR            (FOC_TIM_ARR / 2)
#define FOC_OFFSET_CAL_SHIFT    11u                    /* 2048 samples, ~100 ms */
#define FOC_PI_ERR_LIMIT        4096                   /* counts, keeps the Q12 products in int32 */
#define FOC_PI_GAIN_MAX         (1 << 18)              /* Q12, 64 mV/count */

/* AS5147P: read ANGLECOM (0x3FFF) with the read bit and even parity = 0xFFFF */
#define ENC_CMD_READ_ANGLECOM   0xFFFFu
#define ENC_ERROR_FLAG          0x4000u
#define ENC_SKIP_FRAMES         2u
#define ENC_SPI_TIMEOUT_LOOPS   200u

#define ENC_CS_PORT             CS1_GPIO_Port
#define ENC_CS_PIN              CS1_Pin
#define SPI_DR16                (*(volatile uint16_t *)&SPI1->DR)

#if FOC_VBUS_SENSE_ENABLE
#define FOC_ADC_NCH             4u
#else
#define FOC_ADC_NCH             3u
#endif

FocFast_t g_foc_fast;

static volatile uint16_t s_adc[FOC_ADC_NCH];

static uint32_t s_off_sum[3];
static uint32_t s_off_n;

static int32_t s_integ_d;          /* mV, Q12 */
static int32_t s_integ_q;          /* mV, Q12 */

static uint8_t  s_drop_phase;      /* phase whose shunt reading is not trusted next sample */
static uint16_t s_ccr_prev[3] = {FOC_HALF_ARR, FOC_HALF_ARR, FOC_HALF_ARR};

static uint16_t s_enc_prev;
static uint8_t  s_enc_skip;
static uint8_t  s_enc_consec_err;
static uint8_t  s_slow_div;

/* ------------------------------------------------------------------------- */

/**
 * Start of a short critical section.
 * Saves the current PRIMASK (interrupts already masked or not) and masks all
 * interrupts. Pair with irq_restore() so nesting works: the inner restore puts
 * back "masked" instead of re-enabling interrupts too early.
 */
static inline uint32_t irq_save(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

/** End of a critical section: restores the PRIMASK value returned by irq_save(). */
static inline void irq_restore(uint32_t primask)
{
  __set_PRIMASK(primask);
}

/**
 * Latches one or more FOC_FAULT_* bits and shuts the bridge down.
 *
 * How it works:
 *  1. Clears MOE in TIM1->BDTR. With OSSI = 1 (set in FocFast_Init) the six gate
 *     outputs go to their idle level (low) after the dead time: all FETs off.
 *  2. Forces the fast loop mode to OFF so the next step stops regulating
 *     (except during the boot offset calibration, which runs with the bridge
 *     off anyway and must finish to give valid offsets).
 *  3. ORs the bits into g_foc_fast.faults. Any non-zero value blocks
 *     FocFast_SetMode() from turning the bridge back on until
 *     FocFast_ClearFaults() is called.
 *
 * Runs under a critical section so a slow-loop call can't interleave with a
 * fast-loop call (both read-modify-write BDTR and faults). Callable from any
 * context: fast loop, slow loop, main.
 */
void FocFast_Trip(uint32_t fault_bits)
{
  uint32_t p = irq_save();
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  /* Let a running offset calibration finish: it needs the bridge off anyway, and
   * aborting it would leave bogus offsets (and a spurious overcurrent) behind. */
  if (g_foc_fast.mode != FOC_FAST_MODE_OFFSET_CAL)
  {
    g_foc_fast.mode = FOC_FAST_MODE_OFF;
  }
  g_foc_fast.faults |= fault_bits;
  irq_restore(p);
}

/**
 * Clears every latched fault bit. The bridge stays off: the caller (slow loop)
 * has to enable a mode again with FocFast_SetMode(). If the cause is still
 * present (e.g. encoder unplugged) the fast loop re-trips on its next step.
 */
void FocFast_ClearFaults(void)
{
  uint32_t p = irq_save();
  g_foc_fast.faults = 0u;
  irq_restore(p);
}

/**
 * Changes what the fast loop does each PWM period.
 *
 * VOLTAGE / CURRENT (bridge on):
 *   Inside a critical section, and only if no fault is latched: zero both PI
 *   integrators (so the loop starts from 0 V, not from a stale state), publish
 *   the new mode, then set MOE. Checking faults and setting MOE atomically
 *   prevents a race where the fast loop trips between the check and the
 *   enable, and the slow loop then turns the bridge back on.
 *   The CCRs are already at 50 % (written every step while OFF), so the bridge
 *   starts with zero average voltage on every phase.
 *
 * OFF / OFFSET_CAL (bridge off):
 *   Clear MOE first, then change the mode, so there is never a step where the
 *   bridge is on with the loop not regulating.
 *
 * Called from the slow loop (PendSV). The fast loop never calls it.
 */
void FocFast_SetMode(FocFastMode_t mode)
{
  if ((mode == FOC_FAST_MODE_VOLTAGE) || (mode == FOC_FAST_MODE_CURRENT))
  {
    uint32_t p = irq_save();
    if (g_foc_fast.faults == 0u)
    {
      s_integ_d = 0;
      s_integ_q = 0;
      g_foc_fast.mode = (uint8_t)mode;
      TIM1->BDTR |= TIM_BDTR_MOE;
    }
    irq_restore(p);
  }
  else
  {
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    g_foc_fast.mode = (uint8_t)mode;
  }
}

/* ------------------------------------------------------------------------- */

/**
 * One step of a fixed-point PI current regulator. Returns the voltage in mV.
 *
 *   err   current error in ADC counts (reference - measurement)
 *   kp    proportional gain, mV per count, Q12 (4096 = 1 mV/count)
 *   ki    integral gain per sample, mV per count, Q12
 *   integ integrator state, mV in Q12 (kept by the caller, one per axis)
 *
 * How it works:
 *  1. Clamp the error to +/-4096 counts. With kp, ki <= 2^18 this guarantees
 *     err * k fits in an int32 (<= 2^30), so no 64-bit math is needed.
 *  2. integ += err * ki, then clamp integ to +/-lim (anti-windup by clamping:
 *     the integrator can never store more voltage than the loop may output).
 *  3. out = (err * kp + integ) >> 12 converts Q12 back to mV.
 *  4. Clamp the output to +/-lim.
 *
 * The caller may tighten the q-axis limit afterwards (vector limit) and then
 * re-clamps the integrator itself.
 */
static inline int32_t foc_pi(int32_t *integ, int32_t err, int32_t kp, int32_t ki, int32_t lim_mv)
{
  err = foc_clamp_i32(err, -FOC_PI_ERR_LIMIT, FOC_PI_ERR_LIMIT);
  const int32_t lim_q12 = lim_mv << 12;
  int32_t i = *integ + err * ki;
  i = foc_clamp_i32(i, -lim_q12, lim_q12);
  *integ = i;
  int32_t out = (err * kp + i) >> 12;
  return foc_clamp_i32(out, -lim_mv, lim_mv);
}

/**
 * Turns a voltage vector in the rotor frame into three PWM compare values.
 *
 *   vd, vq          voltage command in mV (rotor frame)
 *   theta           electrical angle to use (already advanced for the
 *                   ~1.5 period delay until this PWM output is applied)
 *   duty_scale_q16  ARR * 65536 / Vbus_mV, from the slow loop
 *
 * How it works:
 *  1. Inverse Park: rotate (vd, vq) by theta into the stator frame (valpha, vbeta).
 *     sin/cos are Q15, so each product is shifted right by 15.
 *  2. Inverse Clarke: split (valpha, vbeta) into phase voltages va, vb, vc
 *     (va = valpha, vb/vc = -valpha/2 +/- sqrt(3)/2 * vbeta).
 *  3. SVPWM by min/max injection: subtract (max + min) / 2 from all three.
 *     This common-mode shift does not change the line-to-line voltages the
 *     motor sees, but centres them in the bus, which gives 15 % more usable
 *     voltage than plain sine PWM (Vbus/sqrt(3) instead of Vbus/2).
 *  4. Convert to compare counts: CCR = ARR/2 + v * ARR / Vbus, clamp to 0..ARR.
 *     With center-aligned PWM mode 1, duty = CCR / ARR.
 *  5. Write CCR1..3. They are preloaded, so the timer applies them at the
 *     next counter peak, and the pulse stays symmetric.
 *  6. Choose which phase to ignore at the next current sample: the one with
 *     the highest duty has the shortest low-side ON window around the peak,
 *     so its shunt reading may not have settled. The next sample sees the old
 *     duty before the peak and the new one after it, so the larger of the two
 *     is used for each phase.
 */
static inline void foc_output(int32_t vd, int32_t vq, uint16_t theta, int32_t duty_scale_q16)
{
  int32_t s, c;
  foc_sincos_q15(theta, &s, &c);

  const int32_t valpha = (vd * c - vq * s) >> 15;
  const int32_t vbeta  = (vd * s + vq * c) >> 15;

  const int32_t t = (vbeta * FOC_Q15_SQRT3_OVER_2) >> 15;
  const int32_t h = valpha >> 1;
  int32_t va = valpha;
  int32_t vb = t - h;
  int32_t vc = -t - h;

  int32_t vmax = va, vmin = va;
  if (vb > vmax) vmax = vb;
  if (vb < vmin) vmin = vb;
  if (vc > vmax) vmax = vc;
  if (vc < vmin) vmin = vc;
  const int32_t off = (vmax + vmin) >> 1;

  int32_t ca = FOC_HALF_ARR + (((va - off) * duty_scale_q16) >> 16);
  int32_t cb = FOC_HALF_ARR + (((vb - off) * duty_scale_q16) >> 16);
  int32_t cc = FOC_HALF_ARR + (((vc - off) * duty_scale_q16) >> 16);
  ca = foc_clamp_i32(ca, 0, FOC_TIM_ARR);
  cb = foc_clamp_i32(cb, 0, FOC_TIM_ARR);
  cc = foc_clamp_i32(cc, 0, FOC_TIM_ARR);

  TIM1->CCR1 = (uint32_t)ca;
  TIM1->CCR2 = (uint32_t)cb;
  TIM1->CCR3 = (uint32_t)cc;

  /* The next sample straddles the peak where these values get loaded, so the
   * window before it uses the previous duties: take the worst of both. */
  int32_t pa = (ca > s_ccr_prev[0]) ? ca : s_ccr_prev[0];
  int32_t pb = (cb > s_ccr_prev[1]) ? cb : s_ccr_prev[1];
  int32_t pc = (cc > s_ccr_prev[2]) ? cc : s_ccr_prev[2];
  s_drop_phase = (pa >= pb) ? ((pa >= pc) ? 0u : 2u) : ((pb >= pc) ? 1u : 2u);
  s_ccr_prev[0] = (uint16_t)ca;
  s_ccr_prev[1] = (uint16_t)cb;
  s_ccr_prev[2] = (uint16_t)cc;
}

/**
 * Puts all three phases at 50 % duty (zero average voltage across the motor).
 * Used every step while the bridge is off, so that the next time MOE is set
 * the bridge starts from a neutral output instead of the last duty cycle.
 */
static inline void foc_output_neutral(void)
{
  TIM1->CCR1 = FOC_HALF_ARR;
  TIM1->CCR2 = FOC_HALF_ARR;
  TIM1->CCR3 = FOC_HALF_ARR;
  s_ccr_prev[0] = s_ccr_prev[1] = s_ccr_prev[2] = FOC_HALF_ARR;
}

/* ------------------------------------------------------------------------- */

/**
 * The 20 kHz control step. Runs once per PWM period from the ADC DMA
 * transfer-complete interrupt, a couple of microseconds after the counter
 * peak, and must finish before the next peak (2*ARR ticks, ~50 us).
 *
 * Order of work, chosen so the SPI frame overlaps with computation:
 *  1. Pull the encoder CS low.
 *  2. Read the three ADC results the DMA just wrote.
 *     - OFFSET_CAL mode: add them to running sums; after 2048 samples
 *       (~100 ms) store the averages as zero-current offsets and go to OFF.
 *     - Otherwise: subtract offsets, apply FOC_CURRENT_SIGN, and replace the
 *       unreliable phase (see foc_output) with minus the sum of the other two
 *       (Kirchhoff: ia + ib + ic = 0).
 *  3. Start the 16-bit SPI frame (READ ANGLECOM). It shifts out in ~2.7 us
 *     while the overcurrent check runs.
 *  4. Overcurrent check: any phase beyond oc_trip_counts trips the bridge.
 *  5. Wait for the SPI answer and raise CS. The AS5147P answers each frame
 *     with the data requested by the previous frame, so this is the angle
 *     from one period ago (compensated by theta_adv_*).
 *  6. Check the answer (even parity, error flag clear). If it is valid, add
 *     the wrapped 14-bit difference to the multi-turn counter pos_raw. If it
 *     is bad, keep the last angle; several bad frames in a row trip ENCODER.
 *  7. Electrical angle = mechanical angle * 4 (14 -> 16 bit) * pole pairs
 *     - offset. uint16 overflow does the modulo 2*pi for free. Calibration
 *     can override the angle instead.
 *  8. VOLTAGE / CURRENT mode:
 *     - Clarke + Park: phase currents -> (id, iq) in the rotor frame.
 *     - CURRENT: d/q PI regulators. The voltage vector is limited to vmax
 *       giving d priority: |vq| <= sqrt(vmax^2 - vd^2), and the q integrator
 *       is clamped to the same limit so it doesn't wind up.
 *     - VOLTAGE: use vd_cmd / vq_cmd directly (calibration).
 *     - foc_output(): inverse transforms + SVPWM + CCR write.
 *     OFF mode: 50 % on all phases.
 *  9. Every FOC_SLOW_DIV steps, pend PendSV to run the slow loop after this
 *     interrupt returns (it is the lowest priority, so it's preempted by
 *     this function, never the other way round).
 * 10. Timing diagnostics: where in the PWM period we finished, and whether
 *     the next DMA transfer already completed (overrun).
 */
static void FocFast_Step(void)
{
  FocFast_t *const f = &g_foc_fast;

  /* Select the encoder now: the ADC work below covers the 350 ns CSn->CLK setup time. */
  ENC_CS_PORT->BRR = ENC_CS_PIN;

  const int32_t ra = s_adc[0];
  const int32_t rb = s_adc[1];
  const int32_t rc = s_adc[2];
#if FOC_VBUS_SENSE_ENABLE
  f->vbus_raw = s_adc[3];
#endif
  const uint8_t mode = f->mode;

  int32_t ia = 0, ib = 0, ic = 0;
  if (mode == FOC_FAST_MODE_OFFSET_CAL)
  {
    s_off_sum[0] += (uint32_t)ra;
    s_off_sum[1] += (uint32_t)rb;
    s_off_sum[2] += (uint32_t)rc;
    if (++s_off_n >= (1u << FOC_OFFSET_CAL_SHIFT))
    {
      f->adc_offset[0] = (uint16_t)(s_off_sum[0] >> FOC_OFFSET_CAL_SHIFT);
      f->adc_offset[1] = (uint16_t)(s_off_sum[1] >> FOC_OFFSET_CAL_SHIFT);
      f->adc_offset[2] = (uint16_t)(s_off_sum[2] >> FOC_OFFSET_CAL_SHIFT);
      s_off_sum[0] = s_off_sum[1] = s_off_sum[2] = 0u;
      s_off_n = 0u;
      f->offset_cal_done = 1u;
      f->mode = FOC_FAST_MODE_OFF;
    }
  }
  else
  {
    ia = (ra - (int32_t)f->adc_offset[0]) * FOC_CURRENT_SIGN;
    ib = (rb - (int32_t)f->adc_offset[1]) * FOC_CURRENT_SIGN;
    ic = (rc - (int32_t)f->adc_offset[2]) * FOC_CURRENT_SIGN;
    /* The phase with the highest duty has the shortest low-side window: rebuild it. */
    if (s_drop_phase == 0u)      { ia = -ib - ic; }
    else if (s_drop_phase == 1u) { ib = -ia - ic; }
    else                         { ic = -ia - ib; }
  }

  /* Encoder: start the 16-bit frame; it shifts out while we do the checks below. */
  SPI_DR16 = ENC_CMD_READ_ANGLECOM;

  f->ia = ia;
  f->ib = ib;
  f->ic = ic;

  const int32_t trip = f->oc_trip_counts;
  if (f->offset_cal_done
      && ((ia > trip) || (ia < -trip) || (ib > trip) || (ib < -trip) || (ic > trip) || (ic < -trip)))
  {
    FocFast_Trip(FOC_FAULT_OVERCURRENT);
  }

  /* Encoder: finish the frame. The answer is the angle requested by the previous frame. */
  uint32_t guard = 0u;
  while (((SPI1->SR & SPI_SR_RXNE) == 0u) && (++guard < ENC_SPI_TIMEOUT_LOOPS))
  {
  }
  const uint8_t rx_ok = (SPI1->SR & SPI_SR_RXNE) != 0u;
  const uint16_t rx = rx_ok ? SPI_DR16 : 0u;
  ENC_CS_PORT->BSRR = ENC_CS_PIN;

  if (s_enc_skip != 0u)
  {
    s_enc_skip--;   /* pipeline still holds answers to init-time commands */
  }
  else if (rx_ok && foc_parity_even16(rx) && ((rx & ENC_ERROR_FLAG) == 0u))
  {
    const uint16_t m = rx & 0x3FFFu;
    if (!f->enc_valid)
    {
      s_enc_prev = m;
      f->pos_raw = m;
      f->enc_valid = 1u;
    }
    int32_t d = (int32_t)m - (int32_t)s_enc_prev;
    if (d > 8192)       { d -= 16384; }
    else if (d < -8192) { d += 16384; }
    f->pos_raw += d;
    s_enc_prev = m;
    f->enc_angle = m;
    s_enc_consec_err = 0u;
  }
  else
  {
    f->enc_error_count++;
    if (++s_enc_consec_err >= FOC_ENC_MAX_CONSEC_ERRORS)
    {
      s_enc_consec_err = FOC_ENC_MAX_CONSEC_ERRORS;
      FocFast_Trip(FOC_FAULT_ENCODER);
    }
  }

  /* Electrical angle: 14-bit mechanical angle * 4 = 16-bit, * pole pairs wraps for free. */
  const uint32_t m_dir = (f->enc_dir >= 0) ? s_enc_prev : (16384u - s_enc_prev);
  const uint16_t theta_enc = (uint16_t)((uint16_t)(m_dir * 4u * f->pole_pairs) - f->enc_offset);
  f->theta_enc = theta_enc;

  uint16_t theta_meas, theta_out;
  if (f->angle_override)
  {
    theta_meas = f->theta_override;
    theta_out = theta_meas;
  }
  else
  {
    theta_meas = (uint16_t)(theta_enc + (uint16_t)f->theta_adv_meas);
    theta_out = (uint16_t)(theta_enc + (uint16_t)f->theta_adv_out);
  }

  const uint8_t run_mode = f->mode;   /* may have just been tripped to OFF */
  if ((run_mode == FOC_FAST_MODE_CURRENT) || (run_mode == FOC_FAST_MODE_VOLTAGE))
  {
    /* Clarke (amplitude invariant) + Park */
    const int32_t ialpha = ia;
    const int32_t ibeta = ((ia + 2 * ib) * FOC_Q15_ONE_OVER_SQRT3) >> 15;
    int32_t s, c;
    foc_sincos_q15(theta_meas, &s, &c);
    const int32_t id = (ialpha * c + ibeta * s) >> 15;
    const int32_t iq = (ibeta * c - ialpha * s) >> 15;
    f->id = id;
    f->iq = iq;

    const int32_t vlim = f->vmax_mv;
    int32_t vd, vq;
    if (run_mode == FOC_FAST_MODE_CURRENT)
    {
      const int32_t kp = f->kp_q12;
      const int32_t ki = f->ki_q12;
      vd = foc_pi(&s_integ_d, f->id_ref - id, kp, ki, vlim);
      vq = foc_pi(&s_integ_q, f->iq_ref - iq, kp, ki, vlim);
      /* Vector limit, d-axis first: |Vq| <= sqrt(Vmax^2 - Vd^2). */
      const uint32_t vmax2 = (uint32_t)(vlim * vlim);
      const uint32_t vd2 = (uint32_t)(vd * vd);
      if (vd2 + (uint32_t)(vq * vq) > vmax2)
      {
        const int32_t vq_lim = (int32_t)foc_isqrt32(vmax2 - vd2);
        vq = foc_clamp_i32(vq, -vq_lim, vq_lim);
        s_integ_q = foc_clamp_i32(s_integ_q, -(vq_lim << 12), vq_lim << 12);
      }
    }
    else
    {
      vd = foc_clamp_i32(f->vd_cmd, -vlim, vlim);
      vq = foc_clamp_i32(f->vq_cmd, -vlim, vlim);
    }
    f->vd = vd;
    f->vq = vq;
    foc_output(vd, vq, theta_out, f->duty_scale_q16);
  }
  else
  {
    f->vd = 0;
    f->vq = 0;
    foc_output_neutral();
  }

  /* Slow loop runs in PendSV, below this interrupt. */
  if (++s_slow_div >= FOC_SLOW_DIV)
  {
    s_slow_div = 0u;
    if (g_foc_slow_busy)
    {
      f->slow_overrun_count++;
    }
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  }

  /* Diagnostics: where in the PWM period did we finish? */
  f->step_count++;
  const uint32_t cnt = TIM1->CNT;
  const uint16_t ticks = (uint16_t)((TIM1->CR1 & TIM_CR1_DIR) ? (FOC_TIM_ARR - cnt) : (FOC_TIM_ARR + cnt));
  f->isr_end_ticks = ticks;
  if (ticks > f->isr_end_ticks_max)
  {
    f->isr_end_ticks_max = ticks;
  }
  if (DMA1->ISR & DMA_ISR_TCIF1)
  {
    f->overrun_count++;
  }
}

/**
 * Entry point from DMA1_Channel1_IRQHandler (stm32f0xx_it.c), called before
 * the HAL handler.
 *
 *  - Transfer error: the DMA hardware has disabled the channel, so no more
 *    samples (and no more fast or slow loop) will come. Trip the bridge; the
 *    slow loop stops refreshing the watchdog, which then resets the MCU.
 *  - Transfer complete: clear the flags and run FocFast_Step() directly.
 *    This skips HAL_DMA_IRQHandler and its callback chain, saving about 100
 *    cycles per period.
 *
 * Returns 1 if the interrupt was handled here (the caller then returns without
 * calling HAL), 0 to let HAL deal with anything else.
 */
uint8_t FocFast_DmaIrqHandler(void)
{
  const uint32_t isr = DMA1->ISR;
  if (isr & DMA_ISR_TEIF1)
  {
    DMA1->IFCR = DMA_IFCR_CGIF1;
    FocFast_Trip(FOC_FAULT_SYSTEM);   /* the channel is now disabled: the IWDG will reset us */
    return 1u;
  }
  if (isr & DMA_ISR_TCIF1)
  {
    DMA1->IFCR = DMA_IFCR_CGIF1;
    FocFast_Step();
    return 1u;
  }
  return 0u;
}

/* ------------------------------------------------------------------------- */

/**
 * Hardware setup for the fast loop, called once from FocControl_Init() after
 * all MX_*_Init() functions. Leaves the bridge OFF and the loop running in
 * OFFSET_CAL mode.
 *
 * Steps:
 *  1. Bridge off with gates driven low when off (OSSI) and when the debugger
 *     halts the core (DBG_TIM1_STOP).
 *  2. TIM1 CH4 (no pin) in PWM mode 2 with CCR4 = ARR - LEAD, and TRGO =
 *     OC4REF: one rising trigger per period, LEAD ticks before the peak.
 *  3. CH1..3 at 50 %, complementary outputs enabled (still gated by MOE).
 *  4. ADC: rising-edge trigger, optional Vbus channel, self-calibration,
 *     circular DMA into s_adc. The half-transfer and overrun interrupts are
 *     turned off, since only transfer-complete is used.
 *  5. SPI1: enable it, empty the RX FIFO, CS high, and skip the first two
 *     encoder answers (they belong to commands sent by the HAL during init).
 *  6. Priorities: DMA (fast loop) highest, PendSV (slow loop) lowest.
 *  7. Generate an update event to load all preload registers, then start
 *     the counter. From now on the fast loop runs every 50 us.
 *
 * Settings that the .ioc also holds (CH4, TRGO, trigger edge) are applied here
 * too, so a CubeMX regeneration with stale settings cannot break the timing.
 */
void FocFast_Init(void)
{
  FocFast_t *const f = &g_foc_fast;

  /* Bridge off. OSSI = 1: with MOE = 0 the outputs are driven to their idle
   * level (all gates low) instead of floating. DBG_TIM1_STOP: when the core is
   * halted by the debugger the outputs are disabled the same way. */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->BDTR |= TIM_BDTR_OSSI;
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  __HAL_DBGMCU_FREEZE_TIM1();

  /* CH4 (no pin) is the ADC trigger: PWM mode 2 -> OC4REF rises while counting
   * up through CCR4, LEAD ticks before the peak. TRGO = OC4REF. These are also
   * applied here so a CubeMX regeneration cannot silently break them. */
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM2;
  oc.Pulse = FOC_TIM_ARR - FOC_ADC_TRIGGER_LEAD_TICKS;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  TIM_MasterConfigTypeDef master = {0};
  master.MasterOutputTrigger = TIM_TRGO_OC4REF;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK)
  {
    Error_Handler();
  }

  /* Phase outputs at 50 %, enabled but gated by MOE. */
  foc_output_neutral();
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE
              | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  /* ADC: convert on the rising edge of TRGO into a circular DMA buffer. */
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  MODIFY_REG(ADC1->CFGR1, ADC_CFGR1_EXTEN, ADC_EXTERNALTRIGCONVEDGE_RISING);
#if FOC_VBUS_SENSE_ENABLE
  ADC_ChannelConfTypeDef ch = {0};
  ch.Channel = FOC_VBUS_ADC_CHANNEL;
  ch.Rank = ADC_RANK_CHANNEL_NUMBER;
  ch.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &ch) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)s_adc, FOC_ADC_NCH) != HAL_OK)
  {
    Error_Handler();
  }
  /* Only transfer-complete is used; the rest would just cost CPU at 20 kHz. */
  __HAL_DMA_DISABLE_IT(hadc.DMA_Handle, DMA_IT_HT);
  __HAL_ADC_DISABLE_IT(&hadc, ADC_IT_OVR);

  /* SPI1 is driven by register access from here on (no HAL calls after this). */
  SPI1->CR1 |= SPI_CR1_SPE;
  while (SPI1->SR & SPI_SR_RXNE)
  {
    (void)SPI_DR16;
  }
  ENC_CS_PORT->BSRR = ENC_CS_PIN;
  s_enc_skip = ENC_SKIP_FRAMES;

  /* Fast loop at the top priority, slow loop (PendSV) at the bottom. */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 3, 0);

  /* Measure current offsets with the bridge off, then idle. */
  f->offset_cal_done = 0u;
  f->mode = FOC_FAST_MODE_OFFSET_CAL;

  /* Load the preloads and start counting. RCR (=1) was written before the
   * counter started, so the update event (CCR load) happens at the peak. */
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);
}
