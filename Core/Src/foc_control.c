/*
 * foc_control.c
 */

#include "foc_control.h"

#include "foc_transforms.h"
#include "motor_current.h"
#include "tim.h"
#include "main.h"

#include <math.h>

extern TIM_HandleTypeDef htim1;

#define FOC_DT_S (1.0f / FOC_CONTROL_LOOP_HZ)
#define FOC_VDQ_LIMIT_V (0.5f * FOC_VBUS_V)

typedef struct
{
  float kp;
  float ki;
  float integral;
  float limit;
} Foc_Pi_t;

static Foc_Pi_t s_id_pi;
static Foc_Pi_t s_iq_pi;

static float s_theta_rad;
static float s_omega_e_rad_s;
static float s_id_ref_a;
static float s_iq_ref_a;
static uint8_t s_enabled;

static void foc_pi_reset(Foc_Pi_t *pi)
{
  pi->integral = 0.0f;
}

static float foc_pi_update(Foc_Pi_t *pi, float error)
{
  pi->integral += pi->ki * error * FOC_DT_S;
  if (pi->integral > pi->limit)
  {
    pi->integral = pi->limit;
  }
  else if (pi->integral < -pi->limit)
  {
    pi->integral = -pi->limit;
  }

  float out = pi->kp * error + pi->integral;
  if (out > pi->limit)
  {
    out = pi->limit;
  }
  else if (out < -pi->limit)
  {
    out = -pi->limit;
  }
  return out;
}

static void foc_wrap_theta_rad(float *theta)
{
  const float two_pi = (float)(2.0 * M_PI);
  while (*theta >= two_pi)
  {
    *theta -= two_pi;
  }
  while (*theta < 0.0f)
  {
    *theta += two_pi;
  }
}

static void foc_write_duty(float duty_a, float duty_b, float duty_c)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

  if (duty_a < 0.0f) duty_a = 0.0f; else if (duty_a > 1.0f) duty_a = 1.0f;
  if (duty_b < 0.0f) duty_b = 0.0f; else if (duty_b > 1.0f) duty_b = 1.0f;
  if (duty_c < 0.0f) duty_c = 0.0f; else if (duty_c > 1.0f) duty_c = 1.0f;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)arr));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)arr));
}

void FOC_Control_Init(void)
{
  s_id_pi.kp = FOC_ID_KP;
  s_id_pi.ki = FOC_ID_KI;
  s_id_pi.limit = FOC_VDQ_LIMIT_V;
  s_iq_pi.kp = FOC_IQ_KP;
  s_iq_pi.ki = FOC_IQ_KI;
  s_iq_pi.limit = FOC_VDQ_LIMIT_V;
  foc_pi_reset(&s_id_pi);
  foc_pi_reset(&s_iq_pi);

  s_theta_rad = 0.0f;
  s_omega_e_rad_s = 0.0f;
  s_id_ref_a = 0.0f;
  s_iq_ref_a = 0.0f;
  s_enabled = 0U;

  foc_write_duty(0.5f, 0.5f, 0.5f);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  MotorCurrent_Init();
  /* Outputs are at neutral (no current) above, so this reads a clean zero-current offset. */
  MotorCurrent_CalibrateOffsets(200);

  HAL_TIM_Base_Start_IT(&htim1);
}

void FOC_Control_SetElectricalSpeedRadS(float omega_e_rad_s)
{
  s_omega_e_rad_s = omega_e_rad_s;
}

void FOC_Control_SetIdqRef(float id_ref_a, float iq_ref_a)
{
  s_id_ref_a = id_ref_a;
  s_iq_ref_a = iq_ref_a;
}

void FOC_Control_SetEnable(uint8_t enable)
{
  s_enabled = enable;
  if (!enable)
  {
    foc_pi_reset(&s_id_pi);
    foc_pi_reset(&s_iq_pi);
    foc_write_duty(0.5f, 0.5f, 0.5f);
  }
}

void FOC_Control_Step(void)
{
  if (!s_enabled)
  {
    return;
  }

  float ia, ib, ic;
  MotorCurrent_GetAmps(&ia, &ib, &ic);

  float ialpha, ibeta;
  FOC_ClarkeTransform(ia, ib, ic, &ialpha, &ibeta);

  float id_meas, iq_meas;
  FOC_ParkTransform(ialpha, ibeta, s_theta_rad, &id_meas, &iq_meas);

  float vd = foc_pi_update(&s_id_pi, s_id_ref_a - id_meas);
  float vq = foc_pi_update(&s_iq_pi, s_iq_ref_a - iq_meas);

  float valpha, vbeta;
  FOC_InverseParkTransform(vd, vq, s_theta_rad, &valpha, &vbeta);

  float va, vb, vc;
  FOC_InverseClarkeTransform(valpha, vbeta, &va, &vb, &vc);

  const float inv_vbus = 1.0f / FOC_VBUS_V;
  foc_write_duty(0.5f + va * inv_vbus, 0.5f + vb * inv_vbus, 0.5f + vc * inv_vbus);

  s_theta_rad += s_omega_e_rad_s * FOC_DT_S;
  foc_wrap_theta_rad(&s_theta_rad);
}

/** TIM1 update fires at FOC_CONTROL_LOOP_HZ; drive the control loop from it. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    FOC_Control_Step();
  }
}
