/*
 * AS5147P.c
 *
 * Created on: Apr 11, 2026
 *      Author: CT
 */

#include "AS5147P.h"

#include "stm32f0xx_hal.h"

/** Bit 14 = 1 selects read (datasheet command frame). */
#define AS5147P_READ_BIT (1u << 14)

/** Even parity for bits 14:0; returns full 16-bit command with PARC in bit 15. */
static uint16_t as5147p_cmd_with_parc(uint16_t bits14_0)
{
  uint16_t low = (uint16_t)(bits14_0 & 0x7FFFu);
  unsigned ones = 0u;
  for (unsigned i = 0u; i < 15u; i++)
  {
    ones += (unsigned)((low >> i) & 1u);
  }
  return (ones & 1u) ? (uint16_t)(low | 0x8000u) : low;
}

static uint16_t as5147p_read_command(uint16_t reg_addr)
{
  uint16_t addr = (uint16_t)(reg_addr & 0x3FFFu);
  return as5147p_cmd_with_parc((uint16_t)(AS5147P_READ_BIT | addr));
}

static HAL_StatusTypeDef as5147p_xfer_16(const AS5147P_Handle_t *handle, uint16_t tx,
                                         uint16_t *rx)
{
  if (handle->hspi == NULL || rx == NULL)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef st =  HAL_SPI_TransmitReceive(handle->hspi, (uint8_t *)&tx, (uint8_t *)rx, 1u, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
  return st;
}

void AS5147P_Init(AS5147P_Handle_t *handle, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
{
  handle->hspi = hspi;
  handle->cs_port = cs_port;
  handle->cs_pin = cs_pin;
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

  /* Flush the SPI pipeline: MISO EF flags an error in the *previous* command, which is
   * undefined at power-up, so the first angle frames would otherwise look "invalid". */
  uint16_t scratch = 0u;
  AS5147P_ReadRegister(handle, AS5147P_REG_ERRFL, &scratch);
  AS5147P_ReadRegister(handle, AS5147P_REG_NOP, &scratch);
  AS5147P_ReadRegister(handle, AS5147P_REG_NOP, &scratch);
}

HAL_StatusTypeDef AS5147P_ReadRegister(const AS5147P_Handle_t *handle, uint16_t reg_addr,
                                       uint16_t *value_out)
{
  if (value_out == NULL)
  {
    return HAL_ERROR;
  }

  uint16_t cmd = as5147p_read_command(reg_addr);
  uint16_t ignore = 0u;
  HAL_StatusTypeDef st = as5147p_xfer_16(handle, cmd, &ignore);
  if (st != HAL_OK)
  {
    return st;
  }

  cmd = as5147p_read_command(AS5147P_REG_NOP);
  return as5147p_xfer_16(handle, cmd, value_out);
}

HAL_StatusTypeDef AS5147P_ReadAngleUncorrected(const AS5147P_Handle_t *handle, uint16_t *angle14)
{
  return AS5147P_ReadRegister(handle, AS5147P_REG_ANGLEUNC, angle14);
}

HAL_StatusTypeDef AS5147P_ReadAngleDAEC(const AS5147P_Handle_t *handle, uint16_t *angle14)
{
  return AS5147P_ReadRegister(handle, AS5147P_REG_ANGLECOM, angle14);
}

uint8_t AS5147P_DataParityOk(uint16_t raw_frame)
{
  unsigned ones = 0u;
  for (unsigned i = 0u; i < 16u; i++)
  {
    ones += (unsigned)((raw_frame >> i) & 1u);
  }
  return (ones & 1u) == 0u;
}

uint8_t AS5147P_ErrorFlag(uint16_t raw_frame)
{
  return (raw_frame & (1u << 14)) != 0u;
}

uint16_t AS5147P_DataBits(uint16_t raw_frame)
{
  return (uint16_t)(raw_frame & 0x3FFFu);
}
