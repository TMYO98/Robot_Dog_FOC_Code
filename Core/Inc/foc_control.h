/*
 * foc_control.h
 *
 * Slow loop (FOC_SLOW_HZ, PendSV, lowest interrupt priority) on top of the
 * fast current loop in foc_fast.c:
 *
 *   - state machine: INIT -> IDLE <-> MOTOR / CALIBRATING, any -> FAULT
 *   - position / velocity (output side, through FOC_GEAR_RATIO)
 *   - impedance control  tau = kp*(p_des - p) + kd*(v_des - v) + tau_ff
 *     -> iq reference for the fast loop
 *   - encoder calibration (current sign, phase resistance, direction,
 *     pole pairs, electrical offset), stored in flash
 *   - Vbus limits, command timeout, watchdog refresh
 *
 * The request / command functions are meant to be called from the main loop
 * (CAN handling); they only set flags that the slow loop acts on.
 *
 *      Author: CT
 */

#ifndef INC_FOC_CONTROL_H_
#define INC_FOC_CONTROL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  FOC_STATE_INIT = 0,      /* measuring current offsets, waiting for the encoder */
  FOC_STATE_IDLE,          /* bridge off */
  FOC_STATE_MOTOR,         /* impedance control from CAN commands */
  FOC_STATE_CALIBRATING,
  FOC_STATE_FAULT          /* bridge off until faults are cleared */
} FocState_t;

typedef enum
{
  FOC_CAL_ERR_NONE = 0,
  FOC_CAL_ERR_NO_CURRENT,     /* voltage ramp produced no current: motor / sensing disconnected */
  FOC_CAL_ERR_CURRENT_SIGN,   /* measured current has the wrong sign: flip FOC_CURRENT_SIGN */
  FOC_CAL_ERR_RESISTANCE,     /* measured resistance out of range */
  FOC_CAL_ERR_NO_MOTION,      /* rotor did not follow the field */
  FOC_CAL_ERR_POLE_PAIRS,     /* motion not consistent with an integer pole-pair count */
  FOC_CAL_ERR_FLASH
} FocCalError_t;

typedef struct
{
  float p_des;    /* rad (output) */
  float v_des;    /* rad/s */
  float kp;       /* Nm/rad */
  float kd;       /* Nm*s/rad */
  float t_ff;     /* Nm */
} FocCommand_t;

typedef struct
{
  float p;        /* rad (output) */
  float v;        /* rad/s */
  float t;        /* Nm, from measured iq */
  uint8_t state;  /* FocState_t */
  uint8_t calibrated;
  uint8_t faults; /* FOC_FAULT_* */
  uint8_t cal_error;
} FocFeedback_t;

/** Loads the flash config, checks the encoder and starts the fast loop. Call after MX_*_Init(). */
void FocControl_Init(void);

/** Slow loop body, called from PendSV_Handler. */
void FocControl_SlowStep(void);

/** Main-loop housekeeping (flash writes). */
void FocControl_Background(void);

void FocControl_RequestMotorMode(void);
void FocControl_RequestIdle(void);
void FocControl_RequestZero(void);         /* idle only: current position becomes 0, saved */
void FocControl_RequestCalibration(void);  /* idle only: motor must be free to turn */
void FocControl_RequestClearFaults(void);

/** New impedance setpoint; also feeds the command timeout. */
void FocControl_SetCommand(const FocCommand_t *cmd);

void FocControl_GetFeedback(FocFeedback_t *fb);

uint8_t FocControl_GetCanId(void);
/** Changes and saves the CAN id (idle only). Returns 1 if accepted. */
uint8_t FocControl_SetCanId(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_CONTROL_H_ */
