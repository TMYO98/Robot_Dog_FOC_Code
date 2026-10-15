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

/**
 * Adds the parity bit to a command frame. The AS5147P requires even parity over
 * all 16 bits: count the ones in bits 14..0, and if the count is odd set bit 15
 * (PARC) so the total is even. A frame with bad parity is ignored by the chip,
 * which then sets its error flag in the next answer.
 */
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

/** Builds a read command: 14-bit register address, read bit (14) set, parity in bit 15. */
static uint16_t as5147p_read_command(uint16_t reg_addr)
{
  uint16_t addr = (uint16_t)(reg_addr & 0x3FFFu);
  return as5147p_cmd_with_parc((uint16_t)(AS5147P_READ_BIT | addr));
}

/**
 * One SPI frame: CS low, shift out 16 bits while shifting in 16 bits (SPI mode 1,
 * MSB first), CS high. The chip latches the command at the CS rising edge, so
 * every frame is a separate transaction. Blocking HAL call, so it's only used
 * during init. The fast loop uses its own register-level transfer.
 */
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

/**
 * Stores the SPI handle and CS pin, parks CS high, and does a few dummy reads.
 * Reading ERRFL also clears the chip's error flags, and the NOP reads push any
 * power-up leftovers out of the one-frame answer pipeline, so later answers
 * are clean.
 */
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

/**
 * Reads one register. The AS5147P answers each frame with the data requested by
 * the previous frame, so a single read takes two frames:
 *   frame 1: send "read reg_addr"; the answer belongs to an older command and is ignored
 *   frame 2: send "read NOP";       the answer is the content of reg_addr
 * value_out gets the raw 16-bit answer: data in bits 13..0, error flag in bit
 * 14, parity in bit 15 (check them with the helpers below).
 */
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

/** Raw angle (ANGLEUNC) without the chip's dynamic angle error compensation. Raw frame. */
HAL_StatusTypeDef AS5147P_ReadAngleUncorrected(const AS5147P_Handle_t *handle, uint16_t *angle14)
{
  return AS5147P_ReadRegister(handle, AS5147P_REG_ANGLEUNC, angle14);
}

/**
 * Angle with dynamic angle error compensation (ANGLECOM). The chip extrapolates
 * for its own internal delay, which matters at speed. This is the register the
 * fast loop reads. Raw frame.
 */
HAL_StatusTypeDef AS5147P_ReadAngleDAEC(const AS5147P_Handle_t *handle, uint16_t *angle14)
{
  return AS5147P_ReadRegister(handle, AS5147P_REG_ANGLECOM, angle14);
}

/** 1 if the answer frame has even parity over all 16 bits (bit 15 = PARD), i.e. it wasn't corrupted. */
uint8_t AS5147P_DataParityOk(uint16_t raw_frame)
{
  unsigned ones = 0u;
  for (unsigned i = 0u; i < 16u; i++)
  {
    ones += (unsigned)((raw_frame >> i) & 1u);
  }
  return (ones & 1u) == 0u;
}

/** Bit 14 (EF): the chip rejected the previous command (bad parity / framing); read ERRFL for details. */
uint8_t AS5147P_ErrorFlag(uint16_t raw_frame)
{
  return (raw_frame & (1u << 14)) != 0u;
}

/** The 14 data bits (angle 0..16383 = 0..360 deg, or register content). */
uint16_t AS5147P_DataBits(uint16_t raw_frame)
{
  return (uint16_t)(raw_frame & 0x3FFFu);
}

/**
 * Bring-up helper (no longer called by the firmware): reads ANGLECOM and decodes
 * it into degrees, parity and error flag, for viewing in the debugger.
 * Don't call it after FocControl_Init(): the fast loop owns SPI1 from then on.
 */
HAL_StatusTypeDef AS5147P_Test_ReadPosition(const AS5147P_Handle_t *handle, AS5147P_Position_t *pos)
{
  if (pos == NULL)
  {
    return HAL_ERROR;
  }

  uint16_t raw = 0u;
  pos->spi_status = AS5147P_ReadAngleDAEC(handle, &raw);
  pos->raw_frame = raw;
  pos->parity_ok = AS5147P_DataParityOk(raw);
  pos->error_flag = AS5147P_ErrorFlag(raw);
  pos->angle14 = AS5147P_DataBits(raw);
  pos->angle_deg = (float)pos->angle14 * (360.0f / 16384.0f);

  return pos->spi_status;
}
