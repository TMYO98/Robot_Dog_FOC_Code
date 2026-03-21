/**
 * @file motor_current.c
 * @brief Phase current sampling via ADC1 scan + DMA circular (PA0/1/2).
 */
#include "motor_current.h"
#include "adc.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_adc_ex.h"

extern ADC_HandleTypeDef hadc;

/* DMA writes here in order: CH0, CH1, CH2 == Ia, Ib, Ic */
static volatile uint16_t s_adc_dma[3];

static uint16_t s_offset[3] = {2048U, 2048U, 2048U};

void MotorCurrent_Init(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)s_adc_dma, 3) != HAL_OK)
  {
    Error_Handler();
  }
}

void MotorCurrent_CalibrateOffsets(uint16_t samples)
{
  if (samples == 0U)
  {
    return;
  }
  uint32_t sum[3] = {0, 0, 0};
  for (uint16_t n = 0; n < samples; n++)
  {
    uint16_t a, b, c;
    __disable_irq();
    a = s_adc_dma[0];
    b = s_adc_dma[1];
    c = s_adc_dma[2];
    __enable_irq();
    sum[0] += a;
    sum[1] += b;
    sum[2] += c;
    HAL_Delay(1);
  }
  s_offset[0] = (uint16_t)(sum[0] / samples);
  s_offset[1] = (uint16_t)(sum[1] / samples);
  s_offset[2] = (uint16_t)(sum[2] / samples);
}

void MotorCurrent_GetRaw(uint16_t *ia, uint16_t *ib, uint16_t *ic)
{
  __disable_irq();
  uint16_t a = s_adc_dma[0];
  uint16_t b = s_adc_dma[1];
  uint16_t c = s_adc_dma[2];
  __enable_irq();
  *ia = a;
  *ib = b;
  *ic = c;
}

void MotorCurrent_GetAmps(float *ia, float *ib, float *ic)
{
  uint16_t ra, rb, rc;
  MotorCurrent_GetRaw(&ra, &rb, &rc);

  const float scale = MOTOR_CURRENT_VREF_V / 4095.0f;
  const float inv_vpa = 1.0f / MOTOR_CURRENT_V_PER_AMP;

  *ia = ((float)ra - (float)s_offset[0]) * scale * inv_vpa;
  *ib = ((float)rb - (float)s_offset[1]) * scale * inv_vpa;
  *ic = ((float)rc - (float)s_offset[2]) * scale * inv_vpa;
}
