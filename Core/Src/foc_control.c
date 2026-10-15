/*
 * foc_control.c
 *
 * Slow loop: see foc_control.h. Floats are fine here (FOC_SLOW_HZ = 2 kHz).
 *
 *      Author: CT
 */

#include "foc_control.h"

#include "foc_config.h"
#include "foc_fast.h"
#include "nv_config.h"
#include "AS5147P.h"
#include "main.h"

#define TWO_PI_F            6.28318530718f
#define ENC_COUNTS_PER_REV  16384
#define RAD_PER_COUNT       (TWO_PI_F / (float)ENC_COUNTS_PER_REV)
#define MS_TICKS(ms)        ((uint32_t)(((uint32_t)(ms) * FOC_SLOW_HZ) / 1000u))
/** First-order low-pass coefficient for cutoff fc at sample rate fs. */
#define LPF_ALPHA(fc, fs)   ((TWO_PI_F * (fc) / (float)(fs)) / (1.0f + TWO_PI_F * (fc) / (float)(fs)))
#define PI_GAIN_MAX_Q12     (1 << 18)

#define CAL_STEP_Q32        ((uint32_t)(FOC_CAL_SPEED_ELEC_REV_S * 4294967296.0f / (float)FOC_SLOW_HZ))
#define CAL_SWEEP_TICKS     ((uint32_t)((float)FOC_CAL_ELEC_REVS * (float)FOC_SLOW_HZ / FOC_CAL_SPEED_ELEC_REV_S))

typedef enum
{
  CAL_SIGN = 0,     /* open-loop voltage ramp at theta = 0, check current sign */
  CAL_ALIGN_LO,     /* lock rotor with id = I/2, record vd */
  CAL_ALIGN_HI,     /* id = I, record vd -> phase resistance */
  CAL_SWEEP_FWD,    /* turn the field forward: direction + pole pairs */
  CAL_SETTLE_1,
  CAL_SWEEP_BACK,   /* sample the electrical offset going back... */
  CAL_SETTLE_2,
  CAL_SWEEP_FWD_2   /* ...and forward again (averages out friction lag) */
} CalPhase_t;

volatile uint8_t g_foc_slow_busy;

static volatile uint8_t s_state = FOC_STATE_INIT;
static NvConfig_t s_cfg;
static volatile uint8_t s_save_pending;

static volatile uint8_t s_req_motor;
static volatile uint8_t s_req_idle;
static volatile uint8_t s_req_zero;
static volatile uint8_t s_req_cal;
static volatile uint8_t s_req_clear;

static FocCommand_t s_cmd;
static volatile uint32_t s_cmd_age;
static FocFeedback_t s_fb;

static int32_t s_zero;              /* output zero, motor-direction counts (multi-turn) */
static int32_t s_prev_pos_raw;
static float s_vel_counts_s;
static float s_iq_filt;
static float s_p_out;
static float s_v_out;
static uint32_t s_state_ticks;
static uint8_t s_cal_error;
#if FOC_VBUS_SENSE_ENABLE
static float s_vbus_v;
#endif

/* calibration */
static uint8_t  s_cal_phase;
static uint32_t s_cal_ticks;
static uint32_t s_cal_theta;        /* Q16.16: upper 16 bits = electrical angle */
static float    s_cal_sum_v;
static float    s_cal_sum_i;
static uint32_t s_cal_n;
static float    s_cal_v1;
static float    s_cal_i1;
static float    s_cal_r;
static int32_t  s_cal_pos_start;
static int8_t   s_cal_dir;
static uint8_t  s_cal_pp;
static uint16_t s_cal_off_ref;
static int32_t  s_cal_off_sum;
static uint32_t s_cal_off_n;

/* ------------------------------------------------------------------------- */

/** Masks interrupts and returns the previous PRIMASK (see irq_restore). Used by the
 *  main-loop API so the slow loop never sees a half-written command or config. */
static inline uint32_t irq_save(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

/** Restores the PRIMASK saved by irq_save(). */
static inline void irq_restore(uint32_t primask)
{
  __set_PRIMASK(primask);
}

/**
 * Converts a gain that is already scaled to Q12 (float) into the integer the fast
 * loop uses: rounded, and clamped to 0..2^18 so err * gain can't overflow int32
 * there (the fast loop clamps the error to +/-4096 counts).
 */
static int32_t gain_to_q12(float g)
{
  if (g < 0.0f)
  {
    return 0;
  }
  if (g > (float)PI_GAIN_MAX_Q12)
  {
    return PI_GAIN_MAX_Q12;
  }
  return (int32_t)(g + 0.5f);
}

/**
 * Computes the current-loop PI gains from the motor model and hands them to the
 * fast loop.
 *
 * The motor winding is an R-L circuit, so its pole is at R/L. Choosing
 * Kp = L * wbw and Ki = R * wbw puts the PI zero (Ki/Kp = R/L) on top of that
 * pole. The two cancel, and the closed loop becomes first order with bandwidth
 * wbw = 2*pi*FOC_CURRENT_BW_HZ.
 *
 * Units: Kp in V/A -> mV per ADC count -> Q12. Ki is per fast-loop sample,
 * so it is also divided by FOC_FAST_HZ.
 * Called at init and whenever calibration measures a new resistance.
 */
static void update_gains(void)
{
  const float w = TWO_PI_F * FOC_CURRENT_BW_HZ;
  const float v_per_a_to_q12 = 1000.0f / FOC_COUNTS_PER_AMP * 4096.0f;   /* V/A -> mV/count, Q12 */
  g_foc_fast.kp_q12 = gain_to_q12(FOC_MOTOR_L_H * w * v_per_a_to_q12);
  g_foc_fast.ki_q12 = gain_to_q12(s_cfg.r_ohm * w / (float)FOC_FAST_HZ * v_per_a_to_q12);
}

/**
 * Copies the stored calibration (encoder offset, direction, pole pairs) and the
 * overcurrent threshold into g_foc_fast, then recomputes the PI gains.
 * Only called while the bridge is off, so the fast loop never sees a mix of
 * old and new values while regulating.
 */
static void push_params(void)
{
  g_foc_fast.enc_offset = s_cfg.enc_offset;
  g_foc_fast.enc_dir = s_cfg.enc_dir;
  g_foc_fast.pole_pairs = s_cfg.pole_pairs;
  g_foc_fast.oc_trip_counts = (int32_t)(FOC_I_TRIP_A * FOC_COUNTS_PER_AMP);
  update_gains();
}

/**
 * Updates the bus-voltage-dependent fast-loop parameters, every slow step.
 *
 * With FOC_VBUS_SENSE_ENABLE: converts the raw ADC value through the divider,
 * low-pass filters it (50 Hz), and trips UNDER/OVERVOLTAGE outside
 * FOC_VBUS_MIN_V..MAX_V (not during INIT, while the filter settles).
 * Without it, FOC_VBUS_NOMINAL_V is assumed.
 *
 * Then it derives:
 *   vmax_mv        largest |Vdq| the current loop may ask for: the linear SVPWM
 *                  limit Vbus/sqrt(3), times FOC_MAX_MODULATION for margin.
 *   duty_scale_q16 ARR * 65536 / Vbus_mV, so the fast loop converts mV to
 *                  compare counts with one multiply and shift. Because it
 *                  tracks the measured Vbus, a sagging battery doesn't change
 *                  the voltage actually applied (bus voltage compensation).
 */
static void update_vbus(void)
{
#if FOC_VBUS_SENSE_ENABLE
  const float v = (float)g_foc_fast.vbus_raw * (FOC_ADC_VREF_V / 4095.0f) * FOC_VBUS_DIVIDER;
  if (s_state == FOC_STATE_INIT)
  {
    s_vbus_v = v;
  }
  else
  {
    s_vbus_v += LPF_ALPHA(50.0f, FOC_SLOW_HZ) * (v - s_vbus_v);
    if (s_vbus_v < FOC_VBUS_MIN_V)
    {
      FocFast_Trip(FOC_FAULT_UNDERVOLTAGE);
    }
    else if (s_vbus_v > FOC_VBUS_MAX_V)
    {
      FocFast_Trip(FOC_FAULT_OVERVOLTAGE);
    }
  }
  const float vbus = (s_vbus_v < 1.0f) ? 1.0f : s_vbus_v;
#else
  const float vbus = FOC_VBUS_NOMINAL_V;
#endif
  const uint32_t vbus_mv = (uint32_t)(vbus * 1000.0f);
  g_foc_fast.vmax_mv = (int32_t)((float)vbus_mv * FOC_MAX_MODULATION * 0.57735027f);
  g_foc_fast.duty_scale_q16 = (int32_t)(((uint32_t)FOC_TIM_ARR << 16) / vbus_mv);
}

/**
 * Position, velocity and torque feedback, every slow step.
 *
 *  - Velocity: change in the multi-turn encoder count since the last slow step,
 *    times FOC_SLOW_HZ, through a first-order low-pass (FOC_VEL_FILTER_HZ).
 *    At 2 kHz one count of change is ~0.77 rad/s, so the filter is what makes
 *    the velocity usable for the kd term.
 *  - iq is filtered the same way for the torque reported on CAN.
 *  - Output position = (direction * raw count - zero) in radians / gear ratio.
 *    Multiplying by the calibrated direction makes positive torque produce
 *    positive position, whatever way the encoder is mounted.
 *  - Angle advance for the fast loop: electrical speed (angle units per fast
 *    sample) times the delay in samples. theta_adv_meas makes up for the
 *    encoder frame being one period old (used for the Park transform).
 *    theta_adv_out also covers the time until the new PWM is applied (used for
 *    the inverse Park). Without this the field lags the rotor at speed and
 *    torque per amp drops.
 */
static void update_kinematics(void)
{
  const int32_t pos_raw = g_foc_fast.pos_raw;
  const int32_t d = pos_raw - s_prev_pos_raw;
  s_prev_pos_raw = pos_raw;

  s_vel_counts_s += LPF_ALPHA(FOC_VEL_FILTER_HZ, FOC_SLOW_HZ) * ((float)d * (float)FOC_SLOW_HZ - s_vel_counts_s);
  s_iq_filt += LPF_ALPHA(FOC_IQ_FILTER_HZ, FOC_SLOW_HZ) * ((float)g_foc_fast.iq - s_iq_filt);

  const float dir = (float)s_cfg.enc_dir;
  const int32_t motor_counts = (int32_t)s_cfg.enc_dir * pos_raw - s_zero;
  s_p_out = (float)motor_counts * (RAD_PER_COUNT / FOC_GEAR_RATIO);
  s_v_out = dir * s_vel_counts_s * (RAD_PER_COUNT / FOC_GEAR_RATIO);

  /* Electrical speed in angle units (65536/rev) per fast sample, for delay compensation. */
  const float w_e = dir * s_vel_counts_s * 4.0f * (float)s_cfg.pole_pairs / (float)FOC_FAST_HZ;
  g_foc_fast.theta_adv_meas = (int32_t)(w_e * FOC_ENC_DELAY_SAMPLES);
  g_foc_fast.theta_adv_out = (int32_t)(w_e * (FOC_ENC_DELAY_SAMPLES + FOC_PWM_DELAY_SAMPLES));
}

/**
 * Picks the whole number of motor turns that puts the output closest to zero.
 *
 * The encoder is absolute over one motor turn only. At boot pos_raw starts at
 * that single-turn angle, and the stored zero is also single-turn. This shifts
 * s_zero by whole turns (16384 counts) until the reported position is in
 * [-half turn, +half turn). So after power-up the joint reads correctly as
 * long as it is within half a motor turn of its zero.
 */
static void align_zero(void)
{
  int32_t mp = (int32_t)s_cfg.enc_dir * g_foc_fast.pos_raw - s_zero;
  while (mp >= ENC_COUNTS_PER_REV / 2)
  {
    s_zero += ENC_COUNTS_PER_REV;
    mp -= ENC_COUNTS_PER_REV;
  }
  while (mp < -ENC_COUNTS_PER_REV / 2)
  {
    s_zero -= ENC_COUNTS_PER_REV;
    mp += ENC_COUNTS_PER_REV;
  }
}

/** Bridge off, encoder angle back in use, current references zeroed, state IDLE. */
static void enter_idle(void)
{
  FocFast_SetMode(FOC_FAST_MODE_OFF);
  g_foc_fast.angle_override = 0u;
  g_foc_fast.id_ref = 0;
  g_foc_fast.iq_ref = 0;
  s_state = FOC_STATE_IDLE;
  s_state_ticks = 0u;
}

/* ------------------------------------------------------------------------- */
/* Motor mode                                                                */
/* ------------------------------------------------------------------------- */

/**
 * Impedance controller, every slow step in MOTOR state.
 *
 *  1. Command timeout: s_cmd_age counts slow ticks since the last CAN command
 *     (FocControl_SetCommand resets it). Past FOC_CAN_TIMEOUT_MS the bridge
 *     trips with FAULT_TIMEOUT, so a dead master can't leave the joint powered.
 *  2. tau = kp * (p_des - p) + kd * (v_des - v) + t_ff   (output side, Nm).
 *     kp acts as a spring, kd as a damper, t_ff as feed-forward torque. With
 *     kp = kd = 0 it is a pure torque command.
 *  3. Motor current iq = tau / (Kt * gear ratio), clamped to +/-FOC_I_MAX_A,
 *     converted to ADC counts for the fast loop. id = 0 (no field weakening).
 */
static void motor_step(void)
{
  if ((FOC_CAN_TIMEOUT_MS > 0u) && (++s_cmd_age > MS_TICKS(FOC_CAN_TIMEOUT_MS)))
  {
    FocFast_Trip(FOC_FAULT_TIMEOUT);
    return;
  }

  const FocCommand_t c = s_cmd;   /* main can't preempt us: consistent copy */
  const float tau = c.kp * (c.p_des - s_p_out) + c.kd * (c.v_des - s_v_out) + c.t_ff;
  float iq_a = tau / (FOC_KT_NM_PER_A * FOC_GEAR_RATIO);
  if (iq_a > FOC_I_MAX_A)
  {
    iq_a = FOC_I_MAX_A;
  }
  else if (iq_a < -FOC_I_MAX_A)
  {
    iq_a = -FOC_I_MAX_A;
  }
  g_foc_fast.id_ref = 0;
  g_foc_fast.iq_ref = (int32_t)(iq_a * FOC_COUNTS_PER_AMP);
}

/* ------------------------------------------------------------------------- */
/* Calibration                                                               */
/* ------------------------------------------------------------------------- */

/**
 * Aborts calibration: records why (reported in FocFeedback_t.cal_error), returns
 * to the encoder angle, and trips FAULT_CALIBRATION, which turns the bridge off.
 * Nothing is written to flash, so the previous calibration stays valid.
 */
static void cal_fail(uint8_t err)
{
  s_cal_error = err;
  g_foc_fast.angle_override = 0u;
  FocFast_Trip(FOC_FAULT_CALIBRATION);
}

/** Moves calibration to the next phase: resets its tick timer and averaging sums. */
static void cal_next(uint8_t phase)
{
  s_cal_phase = phase;
  s_cal_ticks = 0u;
  s_cal_sum_v = 0.0f;
  s_cal_sum_i = 0.0f;
  s_cal_n = 0u;
}

/**
 * Starts calibration from IDLE: the fast loop switches to an overridden
 * electrical angle of 0 and open-loop VOLTAGE mode with 0 V, and the first
 * phase (current sign check) begins. See cal_step() for the sequence.
 */
static void cal_start(void)
{
  s_cal_error = FOC_CAL_ERR_NONE;
  s_cal_theta = 0u;
  g_foc_fast.theta_override = 0u;
  g_foc_fast.angle_override = 1u;
  g_foc_fast.vd_cmd = 0;
  g_foc_fast.vq_cmd = 0;
  g_foc_fast.id_ref = 0;
  g_foc_fast.iq_ref = 0;
  cal_next(CAL_SIGN);
  FocFast_SetMode(FOC_FAST_MODE_VOLTAGE);
  s_state = FOC_STATE_CALIBRATING;
}

/**
 * One sample of the electrical offset during the offset sweeps.
 *
 * The rotor follows the commanded field angle theta, so at every instant
 *   offset = (encoder angle * 4 * pole pairs, in the detected direction) - theta.
 * Angles wrap at 65536, so a plain average would break near 0/65535. Instead,
 * each sample is stored as a signed difference from the first one (int16 wraps
 * correctly), the differences are averaged, and the mean is added back to the
 * first sample in cal_finish().
 */
static void cal_sample_offset(void)
{
  const uint32_t m = g_foc_fast.enc_angle;
  const uint32_t md = (s_cal_dir > 0) ? m : (16384u - m);
  const uint16_t raw = (uint16_t)(md * 4u * s_cal_pp);
  const uint16_t sample = (uint16_t)(raw - (uint16_t)(s_cal_theta >> 16));
  if (s_cal_off_n == 0u)
  {
    s_cal_off_ref = sample;
  }
  s_cal_off_sum += (int16_t)(uint16_t)(sample - s_cal_off_ref);
  s_cal_off_n++;
}

/**
 * Calibration succeeded: turn the bridge off, store offset / direction / pole
 * pairs / resistance in s_cfg, push them to the fast loop (new resistance ->
 * new Ki), recompute the zero (the direction may have flipped), and ask the
 * main loop to write flash. Back to IDLE.
 */
static void cal_finish(void)
{
  const uint16_t offset = (uint16_t)(s_cal_off_ref + (int32_t)(s_cal_off_sum / (int32_t)s_cal_off_n));

  FocFast_SetMode(FOC_FAST_MODE_OFF);
  g_foc_fast.angle_override = 0u;
  g_foc_fast.id_ref = 0;

  s_cfg.enc_offset = offset;
  s_cfg.enc_dir = s_cal_dir;
  s_cfg.pole_pairs = s_cal_pp;
  s_cfg.r_ohm = s_cal_r;
  s_cfg.calibrated = 1u;
  push_params();

  /* The direction may have changed: re-derive the runtime zero. */
  s_zero = s_cfg.zero_counts;
  align_zero();
  s_prev_pos_raw = g_foc_fast.pos_raw;
  s_vel_counts_s = 0.0f;

  s_save_pending = 1u;
  s_state = FOC_STATE_IDLE;
  s_state_ticks = 0u;
}

/**
 * Calibration state machine, one call per slow step (0.5 ms). The motor must be
 * free to turn. Sequence:
 *
 *  CAL_SIGN      Open-loop vd ramps from 0 to FOC_CAL_MAX_VOLTAGE_V over 1 s at
 *                angle 0, so current should flow into phase A and out of B and
 *                C. Once |ia| reaches half the calibration current, it checks
 *                ia > 0 and ib, ic < 0. A wrong sign (or a dead channel) fails
 *                here, before the closed current loop could run away with the
 *                wrong feedback sign. No current by the end of the ramp means
 *                the motor or sensing is disconnected.
 *
 *  CAL_ALIGN_LO  Closed-loop id ramps to I/2 at angle 0, which pulls the rotor
 *                into alignment. Hold, then average vd and id over the last
 *                100 ms.
 *  CAL_ALIGN_HI  Same at full current I. R = (vd2 - vd1) / (id2 - id1). Using
 *                the slope of two points cancels the constant voltage error
 *                from dead time and FET drops. Rotor position is recorded.
 *
 *  CAL_SWEEP_FWD The field turns FOC_CAL_ELEC_REVS electrical revolutions
 *                forward, and the rotor follows.
 *  CAL_SETTLE_1  After 300 ms of settling: the sign of the encoder change gives
 *                the direction, and revs * 16384 / |change| must be (close to)
 *                a whole number, the pole-pair count.
 *  CAL_SWEEP_BACK, CAL_SETTLE_2, CAL_SWEEP_FWD_2
 *                Sweep back and forward again while sampling the offset
 *                (cal_sample_offset). Friction makes the rotor lag the field
 *                by the same angle in each direction, so averaging both
 *                directions cancels it.
 *
 * Total time is about 1.5 s plus 3 x 2 s of sweeps.
 */
static void cal_step(void)
{
  const int32_t i_cal = (int32_t)(FOC_CAL_CURRENT_A * FOC_COUNTS_PER_AMP);
  s_cal_ticks++;

  switch (s_cal_phase)
  {
    case CAL_SIGN:
    {
      /* Ramp vd at theta = 0: current flows into A and out of B and C. */
      int32_t vmax = (int32_t)(FOC_CAL_MAX_VOLTAGE_V * 1000.0f);
      if (vmax > g_foc_fast.vmax_mv)
      {
        vmax = g_foc_fast.vmax_mv;
      }
      const uint32_t ramp = MS_TICKS(1000);
      g_foc_fast.vd_cmd = (vmax * (int32_t)s_cal_ticks) / (int32_t)ramp;

      const int32_t ia = g_foc_fast.ia;
      if ((ia > i_cal / 2) || (ia < -i_cal / 2))
      {
        if ((ia > 0) && (g_foc_fast.ib < 0) && (g_foc_fast.ic < 0))
        {
          g_foc_fast.id_ref = 0;
          FocFast_SetMode(FOC_FAST_MODE_CURRENT);
          cal_next(CAL_ALIGN_LO);
        }
        else
        {
          cal_fail(FOC_CAL_ERR_CURRENT_SIGN);
        }
      }
      else if (s_cal_ticks > ramp)
      {
        cal_fail(FOC_CAL_ERR_NO_CURRENT);
      }
      break;
    }

    case CAL_ALIGN_LO:
    case CAL_ALIGN_HI:
    {
      const int32_t target = (s_cal_phase == CAL_ALIGN_LO) ? (i_cal / 2) : i_cal;
      if ((s_cal_phase == CAL_ALIGN_LO) && (s_cal_ticks < MS_TICKS(200)))
      {
        g_foc_fast.id_ref = (target * (int32_t)s_cal_ticks) / (int32_t)MS_TICKS(200);
      }
      else
      {
        g_foc_fast.id_ref = target;
      }

      const uint32_t end = (s_cal_phase == CAL_ALIGN_LO) ? MS_TICKS(500) : MS_TICKS(300);
      if (s_cal_ticks > end - MS_TICKS(100))
      {
        s_cal_sum_v += (float)g_foc_fast.vd;
        s_cal_sum_i += (float)g_foc_fast.id;
        s_cal_n++;
      }
      if (s_cal_ticks >= end)
      {
        const float v = s_cal_sum_v / (float)s_cal_n;
        const float i = s_cal_sum_i / (float)s_cal_n;
        if (s_cal_phase == CAL_ALIGN_LO)
        {
          s_cal_v1 = v;
          s_cal_i1 = i;
          cal_next(CAL_ALIGN_HI);
        }
        else
        {
          /* Two-point slope cancels the dead-time voltage error. */
          const float di_a = (i - s_cal_i1) / FOC_COUNTS_PER_AMP;
          const float dv = (v - s_cal_v1) / 1000.0f;
          const float r = (di_a > 0.2f * FOC_CAL_CURRENT_A) ? (dv / di_a) : -1.0f;
          if ((r < 0.005f) || (r > 50.0f))
          {
            cal_fail(FOC_CAL_ERR_RESISTANCE);
            break;
          }
          s_cal_r = r;
          s_cal_pos_start = g_foc_fast.pos_raw;
          s_cal_theta = 0u;
          cal_next(CAL_SWEEP_FWD);
        }
      }
      break;
    }

    case CAL_SWEEP_FWD:
    case CAL_SWEEP_BACK:
    case CAL_SWEEP_FWD_2:
    {
      if (s_cal_phase == CAL_SWEEP_BACK)
      {
        s_cal_theta -= CAL_STEP_Q32;
      }
      else
      {
        s_cal_theta += CAL_STEP_Q32;
      }
      g_foc_fast.theta_override = (uint16_t)(s_cal_theta >> 16);
      if (s_cal_phase != CAL_SWEEP_FWD)
      {
        cal_sample_offset();
      }
      if (s_cal_ticks >= CAL_SWEEP_TICKS)
      {
        if (s_cal_phase == CAL_SWEEP_FWD)
        {
          cal_next(CAL_SETTLE_1);
        }
        else if (s_cal_phase == CAL_SWEEP_BACK)
        {
          cal_next(CAL_SETTLE_2);
        }
        else
        {
          cal_finish();
        }
      }
      break;
    }

    case CAL_SETTLE_1:
    {
      if (s_cal_ticks < MS_TICKS(300))
      {
        break;
      }
      const int32_t d = g_foc_fast.pos_raw - s_cal_pos_start;
      const int32_t ad = (d < 0) ? -d : d;
      if (ad < 16)
      {
        cal_fail(FOC_CAL_ERR_NO_MOTION);
        break;
      }
      const float ppf = (float)FOC_CAL_ELEC_REVS * (float)ENC_COUNTS_PER_REV / (float)ad;
      const uint32_t pp = (uint32_t)(ppf + 0.5f);
      const float err = ppf - (float)pp;
      if ((pp < 1u) || (pp > FOC_CAL_MAX_POLE_PAIRS) || (err > 0.2f) || (err < -0.2f))
      {
        cal_fail(FOC_CAL_ERR_POLE_PAIRS);
        break;
      }
      s_cal_dir = (d > 0) ? 1 : -1;
      s_cal_pp = (uint8_t)pp;
      s_cal_off_sum = 0;
      s_cal_off_n = 0u;
      cal_next(CAL_SWEEP_BACK);
      break;
    }

    case CAL_SETTLE_2:
      if (s_cal_ticks >= MS_TICKS(300))
      {
        cal_next(CAL_SWEEP_FWD_2);
      }
      break;

    default:
      cal_fail(FOC_CAL_ERR_NONE);
      break;
  }
}

/* ------------------------------------------------------------------------- */
/* Slow loop                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * Slow loop body: FOC_SLOW_HZ (2 kHz), called from PendSV_Handler, which the
 * fast loop pends every FOC_SLOW_DIV steps. PendSV has the lowest priority, so
 * the fast loop can interrupt this function at any point, but main-loop code
 * can't.
 *
 * Every call:
 *  1. Mark busy (the fast loop counts overruns) and refresh the watchdog. If
 *     either loop stops, the refresh stops and the MCU resets.
 *  2. Vbus scaling/limits and position/velocity feedback.
 *  3. Any fault latched by either loop -> FAULT state (bridge already off).
 *  4. Clear-faults and go-idle requests, valid from several states.
 *  5. The state machine:
 *       INIT         wait for the current offsets and a valid encoder angle
 *                    (1 s timeout), set up the zero, go to IDLE.
 *       IDLE         handle zero / calibrate / motor-mode requests. Motor
 *                    mode needs a stored calibration and starts with an
 *                    all-zero command (no torque) until the master sends one.
 *       MOTOR        motor_step(): impedance control.
 *       CALIBRATING  cal_step().
 *       FAULT        stay off until a clear-faults request.
 *  6. Requests that didn't apply are dropped (not queued), so a stale "enter
 *     motor mode" can't fire later by surprise.
 *  7. Publish the feedback snapshot that CAN replies are built from.
 */
void FocControl_SlowStep(void)
{
  g_foc_slow_busy = 1u;

#if FOC_IWDG_ENABLE
  IWDG->KR = 0xAAAAu;
#endif

  update_vbus();
  update_kinematics();
  s_state_ticks++;

  if ((g_foc_fast.faults != 0u) && (s_state != FOC_STATE_FAULT))
  {
    FocFast_SetMode(FOC_FAST_MODE_OFF);
    g_foc_fast.angle_override = 0u;
    g_foc_fast.id_ref = 0;
    g_foc_fast.iq_ref = 0;
    s_state = FOC_STATE_FAULT;
  }

  if (s_req_clear)
  {
    s_req_clear = 0u;
    if (s_state == FOC_STATE_FAULT)
    {
      FocFast_ClearFaults();
      s_cal_error = FOC_CAL_ERR_NONE;
      enter_idle();
    }
  }
  if (s_req_idle)
  {
    s_req_idle = 0u;
    if ((s_state == FOC_STATE_MOTOR) || (s_state == FOC_STATE_CALIBRATING))
    {
      enter_idle();
    }
  }

  switch (s_state)
  {
    case FOC_STATE_INIT:
      if (g_foc_fast.offset_cal_done && g_foc_fast.enc_valid)
      {
        s_prev_pos_raw = g_foc_fast.pos_raw;
        s_vel_counts_s = 0.0f;
        align_zero();
        enter_idle();
      }
      else if (s_state_ticks > MS_TICKS(1000))
      {
        FocFast_Trip(g_foc_fast.offset_cal_done ? FOC_FAULT_ENCODER : FOC_FAULT_SYSTEM);
      }
      break;

    case FOC_STATE_IDLE:
      if (s_req_zero)
      {
        s_zero = (int32_t)s_cfg.enc_dir * g_foc_fast.pos_raw;
        s_cfg.zero_counts = (uint16_t)(s_zero & (ENC_COUNTS_PER_REV - 1));
        s_save_pending = 1u;
      }
      if (s_req_cal)
      {
        cal_start();
      }
      else if (s_req_motor && s_cfg.calibrated)
      {
        const FocCommand_t zero_cmd = {0};
        s_cmd = zero_cmd;
        s_cmd_age = 0u;
        g_foc_fast.id_ref = 0;
        g_foc_fast.iq_ref = 0;
        g_foc_fast.angle_override = 0u;
        FocFast_SetMode(FOC_FAST_MODE_CURRENT);
        s_state = FOC_STATE_MOTOR;
        s_state_ticks = 0u;
      }
      break;

    case FOC_STATE_MOTOR:
      motor_step();
      break;

    case FOC_STATE_CALIBRATING:
      cal_step();
      break;

    case FOC_STATE_FAULT:
    default:
      break;
  }

  /* Requests that did not apply in the current state are dropped, not queued. */
  s_req_motor = 0u;
  s_req_zero = 0u;
  s_req_cal = 0u;

  s_fb.p = s_p_out;
  s_fb.v = s_v_out;
  s_fb.t = s_iq_filt * (FOC_KT_NM_PER_A * FOC_GEAR_RATIO / FOC_COUNTS_PER_AMP);
  s_fb.state = s_state;
  s_fb.calibrated = s_cfg.calibrated;
  s_fb.faults = (uint8_t)g_foc_fast.faults;
  s_fb.cal_error = s_cal_error;

  g_foc_slow_busy = 0u;
}

/* ------------------------------------------------------------------------- */
/* Init / main-loop API                                                      */
/* ------------------------------------------------------------------------- */

/**
 * Starts the independent watchdog (runs from the ~40 kHz LSI, independent of
 * the main clock). Once started it can't be stopped. The slow loop reloads it
 * every 0.5 ms, and if it isn't reloaded within ~FOC_IWDG_TIMEOUT_MS the MCU
 * resets (and the bridge outputs go Hi-Z / off with it). It is frozen while
 * the debugger halts the core, so breakpoints don't cause resets.
 */
static void iwdg_start(void)
{
#if FOC_IWDG_ENABLE
  __HAL_DBGMCU_FREEZE_IWDG();         /* don't reset while halted at a breakpoint */
  IWDG->KR = 0xCCCCu;                 /* start (also starts LSI) */
  IWDG->KR = 0x5555u;                 /* unlock PR/RLR */
  IWDG->PR = 1u;                      /* /8 -> ~5 kHz with the nominal 40 kHz LSI */
  IWDG->RLR = FOC_IWDG_TIMEOUT_MS * 5u;
  uint32_t guard = 0u;
  while ((IWDG->SR != 0u) && (++guard < 100000u))
  {
  }
  IWDG->KR = 0xAAAAu;
#endif
}

/**
 * Called once from main() after the MX_*_Init() functions.
 *  1. Load the calibration / CAN id from flash (defaults if blank or corrupt)
 *     and push it to the fast loop. Fill in the Vbus-dependent scaling.
 *  2. Encoder bring-up through the HAL driver: flush its SPI pipeline, then
 *     read DIAAGC. Magnet too weak/strong or CORDIC overflow -> FAULT_MAGNET.
 *     No valid answer -> FAULT_ENCODER. The firmware keeps running so the
 *     fault can be reported over CAN.
 *  3. FocFast_Init(): start PWM timing, ADC sampling and the fast loop (bridge
 *     off, current offsets being measured).
 *  4. Start the watchdog.
 */
void FocControl_Init(void)
{
  NvConfig_Load(&s_cfg);
  s_zero = s_cfg.zero_counts;
  s_state = FOC_STATE_INIT;
  push_params();
  update_vbus();

  /* Encoder bring-up with the HAL driver (the fast loop takes SPI1 over afterwards). */
  AS5147P_Init(&AS5147P_handle_1, &hspi1, CS1_GPIO_Port, CS1_Pin);
  uint16_t diag = 0u;
  if ((AS5147P_ReadRegister(&AS5147P_handle_1, AS5147P_REG_DIAAGC, &diag) == HAL_OK)
      && AS5147P_DataParityOk(diag) && !AS5147P_ErrorFlag(diag))
  {
    if (diag & (AS5147P_DIAAGC_MAGH | AS5147P_DIAAGC_MAGL | AS5147P_DIAAGC_COF))
    {
      FocFast_Trip(FOC_FAULT_MAGNET);
    }
  }
  else
  {
    FocFast_Trip(FOC_FAULT_ENCODER);
  }

  FocFast_Init();
  iwdg_start();
}

/**
 * Main-loop housekeeping: writes the config to flash when the slow loop asked
 * for it (after calibration, zeroing or a CAN id change).
 *
 * A flash page erase stalls the whole CPU (the code runs from the same flash)
 * for up to ~40 ms, so the fast loop stops during it. That is only safe with
 * the bridge off, hence the IDLE/FAULT + mode OFF condition. The watchdog
 * (100 ms) tolerates the stall. The config is copied under a critical section
 * first so the slow loop can't change it halfway through the write.
 */
void FocControl_Background(void)
{
  if (s_save_pending && ((s_state == FOC_STATE_IDLE) || (s_state == FOC_STATE_FAULT))
      && (g_foc_fast.mode == FOC_FAST_MODE_OFF))
  {
    uint32_t p = irq_save();
    NvConfig_t copy = s_cfg;
    s_save_pending = 0u;
    irq_restore(p);

    /* Stalls the CPU (and the fast loop) for a page erase; the bridge is off. */
    if (!NvConfig_Save(&copy))
    {
      s_cal_error = FOC_CAL_ERR_FLASH;
      FocFast_Trip(FOC_FAULT_SYSTEM);
    }
  }
}

/* Request flags: the main loop (CAN) sets them, and the slow loop acts on them at
 * its next step and clears them. Each is a single byte, so setting one is
 * atomic. What each request does in each state is in FocControl_SlowStep(). */
void FocControl_RequestMotorMode(void)    { s_req_motor = 1u; }
void FocControl_RequestIdle(void)         { s_req_idle = 1u; }
void FocControl_RequestZero(void)         { s_req_zero = 1u; }
void FocControl_RequestCalibration(void)  { s_req_cal = 1u; }
void FocControl_RequestClearFaults(void)  { s_req_clear = 1u; }

/**
 * Stores a new impedance setpoint and resets the command timeout. The struct is
 * copied with interrupts masked, so the slow loop (which can preempt main) never
 * sees half of an old command and half of a new one.
 */
void FocControl_SetCommand(const FocCommand_t *cmd)
{
  uint32_t p = irq_save();
  s_cmd = *cmd;
  s_cmd_age = 0u;
  irq_restore(p);
}

/** Copies the latest feedback snapshot (written by the slow loop) atomically. */
void FocControl_GetFeedback(FocFeedback_t *fb)
{
  uint32_t p = irq_save();
  *fb = s_fb;
  irq_restore(p);
}

/** CAN id from the stored config (FOC_CAN_ID_DEFAULT on a blank board). */
uint8_t FocControl_GetCanId(void)
{
  return s_cfg.can_id;
}

/**
 * Changes the CAN id and schedules a flash write. Only accepted in IDLE and for
 * ids 1..127 that aren't the master's id. Returns 1 if accepted. The caller
 * (can_protocol.c) then moves its receive filter to the new id.
 */
uint8_t FocControl_SetCanId(uint8_t id)
{
  if ((s_state != FOC_STATE_IDLE) || (id == 0u) || (id == FOC_CAN_MASTER_ID) || (id > 0x7Fu))
  {
    return 0u;
  }
  uint32_t p = irq_save();
  s_cfg.can_id = id;
  s_save_pending = 1u;
  irq_restore(p);
  return 1u;
}
