/*
 * AS5147P.h
 *
 * SPI driver for ams OSRAM AS5147P magnetic encoder (14-bit angle).
 * Datasheet: SPI mode 1 (CPOL=0, CPHA=1), 16-bit frames, even parity on command/data.
 *
 * Created on: Apr 11, 2026
 *      Author: CT
 */

#ifndef INC_AS5147P_H_
#define INC_AS5147P_H_

#include <stdbool.h>
#include <stdint.h>

#include "spi.h"

/** SPI CS for the encoder: set to CS1 (PF11) or CS2 (PB8) to match the PCB. */
#define AS5147P_CS_GPIO_Port (CS1_GPIO_Port)
#define AS5147P_CS_Pin       (CS1_Pin)

/** Volatile register addresses (bits 13:0 of SPI command). */
#define AS5147P_REG_NOP       0x0000u
#define AS5147P_REG_ERRFL     0x0001u
#define AS5147P_REG_PROG      0x0003u
#define AS5147P_REG_DIAAGC    0x3FFCu
#define AS5147P_REG_MAGN      0x3FFDu
#define AS5147P_REG_ANGLEUNC  0x3FFEu
#define AS5147P_REG_ANGLECOM  0x3FFFu

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} AS5147P_Handle_t;

extern AS5147P_Handle_t AS5147P_handle_1;

void AS5147P_Init(AS5147P_Handle_t *handle, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

HAL_StatusTypeDef AS5147P_ReadRegister(const AS5147P_Handle_t *handle, uint16_t reg_addr,
                                       uint16_t *value_out);

HAL_StatusTypeDef AS5147P_ReadAngleUncorrected(const AS5147P_Handle_t *handle, uint16_t *angle14);
HAL_StatusTypeDef AS5147P_ReadAngleDAEC(const AS5147P_Handle_t *handle, uint16_t *angle14);

uint8_t AS5147P_DataParityOk(uint16_t raw_frame);
/** EF bit: set if the *previous* SPI command frame was bad (not necessarily this data). */
uint8_t AS5147P_ErrorFlag(uint16_t raw_frame);
uint16_t AS5147P_DataBits(uint16_t raw_frame);

/** Decoded snapshot of a single angle read, for quick bring-up/debug (e.g. Live Expressions). */
typedef struct
{
  HAL_StatusTypeDef spi_status;
  uint16_t raw_frame;
  uint16_t angle14;
  float angle_deg;
  uint8_t parity_ok;
  uint8_t error_flag;
} AS5147P_Position_t;

/** Simple tester: reads the DAEC-compensated angle and decodes it into pos. */
HAL_StatusTypeDef AS5147P_Test_ReadPosition(const AS5147P_Handle_t *handle, AS5147P_Position_t *pos);

#endif /* INC_AS5147P_H_ */
