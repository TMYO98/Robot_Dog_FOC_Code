/*
 * foc_config.h
 *
 * Every board / motor / protocol parameter in one place.
 * Values marked "TODO" are placeholders: check them against your hardware.
 * Values marked "(calibrated)" are only defaults; the encoder calibration
 * measures them and stores the result in flash (see nv_config.c).
 *
 *      Author: CT
 */

#ifndef INC_FOC_CONFIG_H_
#define INC_FOC_CONFIG_H_

/* ------------------------------------------------------------------------- */
/* Timing                                                                    */
/* ------------------------------------------------------------------------- */

/** TIM1 auto-reload. Must match TIM1 Period in the .ioc (center-aligned:
 *  f_pwm = 48 MHz / (2 * ARR) = 20 kHz). Duty = CCR / ARR. */
#define FOC_TIM_ARR               1199

/** Fast (current) loop rate: once per PWM period, driven by the ADC DMA
 *  transfer-complete interrupt. TIM1 RCR = 1 so the CCR preload is loaded
 *  once per period (at the counter peak). */
#define FOC_FAST_HZ               20000u

/** Slow loop (state machine, impedance control, kinematics) runs every
 *  FOC_SLOW_DIV fast steps, in the PendSV handler at the lowest priority. */
#define FOC_SLOW_DIV              10u
#define FOC_SLOW_HZ               (FOC_FAST_HZ / FOC_SLOW_DIV)

/** ADC trigger (TIM1 CH4 / OC4REF -> TRGO) fires this many timer ticks
 *  (1/48 MHz) before the counter peak, so the three-channel scan
 *  (~4.3 us) is centred on the peak, where all low-side FETs are on. */
#define FOC_ADC_TRIGGER_LEAD_TICKS 80

/* ------------------------------------------------------------------------- */
/* Current sensing (3 shunts + INA241x4 amplifiers on ADC IN0..IN2)           */
/* ------------------------------------------------------------------------- */

#define FOC_ADC_VREF_V            3.3f
#define FOC_SHUNT_OHM             0.003f   /* 3 mOhm */
#define FOC_CSA_GAIN              100.0f   /* INA241A4 / INA241B4: 100 V/V */

/** +1 or -1. If calibration fails with FOC_CAL_ERR_CURRENT_SIGN, flip this. */
#define FOC_CURRENT_SIGN          (+1)

/** ADC counts per amp of phase current: 0.3 V/A -> ~372 counts/A, ~2.7 mA per count.
 *  Full scale is +/-1.65 V around the INA241 REF voltage: +/-5.5 A with REF at 1.65 V.
 *  The zero-current offset is measured at boot, so the actual REF level is handled,
 *  but a REF away from mid-rail shrinks the range on one side. */
#define FOC_COUNTS_PER_AMP        (4096.0f / FOC_ADC_VREF_V * FOC_SHUNT_OHM * FOC_CSA_GAIN)

/* ------------------------------------------------------------------------- */
/* DC bus                                                                    */
/* ------------------------------------------------------------------------- */

/** Set to 1 if the PCB has a Vbus divider on a free ADC pin. With 0, the
 *  firmware assumes FOC_VBUS_NOMINAL_V and under/over-voltage checks are off. */
#define FOC_VBUS_SENSE_ENABLE     0
#define FOC_VBUS_ADC_CHANNEL      ADC_CHANNEL_3   /* PA3, must be after IN2 in scan order */
#define FOC_VBUS_DIVIDER          11.0f           /* Vbus / Vpin, TODO */
#define FOC_VBUS_NOMINAL_V        6.0f
#define FOC_VBUS_MIN_V            5.0f
#define FOC_VBUS_MAX_V            7.5f            /* fixed-point math is sized for <= 40 V */

/* ------------------------------------------------------------------------- */
/* Motor                                                                     */
/* ------------------------------------------------------------------------- */

/* RS2205 (12N14P outrunner), direct drive. */
#define FOC_POLE_PAIRS_DEFAULT    7u       /* 14 magnets (calibration verifies) */
#define FOC_MOTOR_R_OHM_DEFAULT   0.05f    /* (calibrated) phase resistance */
/** Phase inductance. Kp is computed from it: overestimating L makes the current loop
 *  unstable, underestimating it only makes it slower, so this is kept on the low side.
 *  TODO: measure (LCR meter across two phases, divide by 2). */
#define FOC_MOTOR_L_H             10e-6f

/** KV (rpm/V). TODO: set to your RS2205 variant (2300 / 2600 are common). */
#define FOC_MOTOR_KV              2300.0f
/** Torque constant for the amplitude-invariant iq used here, from KV:
 *  Kt = 1.5 * pp * flux_linkage = (sqrt(3)/2) / KV[rad/s/V]  (~0.0036 Nm/A at 2300 KV). */
#define FOC_KT_NM_PER_A           (0.8660254f / (FOC_MOTOR_KV * 0.10471976f))
/** Output = motor / gear ratio. Positions/velocities/torques on CAN are output-side. */
#define FOC_GEAR_RATIO            1.0f     /* direct drive */

/* ------------------------------------------------------------------------- */
/* Current loop                                                              */
/* ------------------------------------------------------------------------- */

/** PI gains are derived from the motor: Kp = L * wbw, Ki = R * wbw. */
#define FOC_CURRENT_BW_HZ         1000.0f
#define FOC_I_MAX_A               4.0f     /* iq command limit */
#define FOC_I_TRIP_A              5.0f     /* any phase above this -> overcurrent fault (keep < ADC full scale, 5.5 A) */
/** Fraction of the linear SVPWM range (Vbus/sqrt(3)) the loop may use. */
#define FOC_MAX_MODULATION        0.95f

/** Angle delay compensation, in fast-loop samples:
 *  encoder frame is ~1 sample old, PWM output lands ~1.5 samples later. */
#define FOC_ENC_DELAY_SAMPLES     1.0f
#define FOC_PWM_DELAY_SAMPLES     1.5f

/** Velocity / torque feedback low-pass filters (slow loop). */
#define FOC_VEL_FILTER_HZ         150.0f
#define FOC_IQ_FILTER_HZ          300.0f

/** Encoder frames with bad parity / error flag tolerated in a row before faulting. */
#define FOC_ENC_MAX_CONSEC_ERRORS 5u

/* ------------------------------------------------------------------------- */
/* Calibration (CAN command 0xFB, motor must be free to turn)                */
/* ------------------------------------------------------------------------- */

#define FOC_CAL_CURRENT_A         2.0f     /* d-axis current used to lock the rotor */
#define FOC_CAL_MAX_VOLTAGE_V     2.0f     /* voltage ramp limit for the current-sign check */
#define FOC_CAL_SPEED_ELEC_REV_S  1.0f     /* electrical revolutions per second during sweeps */
#define FOC_CAL_ELEC_REVS         2u       /* electrical revolutions per sweep */
#define FOC_CAL_MAX_POLE_PAIRS    40u

/* ------------------------------------------------------------------------- */
/* CAN (MIT mini-cheetah style protocol, 1 Mbit/s)                            */
/* ------------------------------------------------------------------------- */

#define FOC_CAN_ID_DEFAULT        1u
#define FOC_CAN_MASTER_ID         0u
/** Motor mode drops to idle (fault TIMEOUT) if no command arrives for this long. 0 = off. */
#define FOC_CAN_TIMEOUT_MS        100u

/** Packing ranges (output side). Master and motor must agree on these.
 *  Sized for the RS2205: max torque = FOC_I_MAX_A * Kt ~= 0.0144 Nm, so the torque and
 *  gain ranges are small, otherwise the 12-bit fields would only use a few codes.
 *  T: 0.04 Nm / 4095 ~= 10 uNm per code, KP: 0.24 mNm/rad per code. */
#define FOC_P_MIN                 (-12.5f)
#define FOC_P_MAX                 (12.5f)
#define FOC_V_MIN                 (-200.0f)
#define FOC_V_MAX                 (200.0f)
#define FOC_KP_MIN                (0.0f)
#define FOC_KP_MAX                (1.0f)
#define FOC_KD_MIN                (0.0f)
#define FOC_KD_MAX                (0.02f)
#define FOC_T_MIN                 (-0.02f)
#define FOC_T_MAX                 (0.02f)

/* ------------------------------------------------------------------------- */
/* Safety                                                                    */
/* ------------------------------------------------------------------------- */

/** Independent watchdog, refreshed by the slow loop. Approximate (LSI 30..50 kHz). */
#define FOC_IWDG_ENABLE           1
#define FOC_IWDG_TIMEOUT_MS       100u

#endif /* INC_FOC_CONFIG_H_ */
