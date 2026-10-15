/*
 * nv_config.h
 *
 * Calibration / identity data kept in the last 1 KB flash page (0x08007C00).
 * The linker script reserves that page (FLASH LENGTH = 31K).
 *
 * Saving erases a page, which stalls the CPU for up to ~40 ms (code runs from
 * the same flash): only do it with the bridge off (see FocControl_Background()).
 *
 *      Author: CT
 */

#ifndef INC_NV_CONFIG_H_
#define INC_NV_CONFIG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t enc_offset;     /* electrical angle at encoder zero (uint16 angle units) */
  int8_t   enc_dir;        /* +1 / -1 */
  uint8_t  pole_pairs;
  uint8_t  calibrated;     /* 1 after a successful encoder calibration */
  uint8_t  can_id;
  uint16_t zero_counts;    /* output zero, motor-direction encoder counts (0..16383) */
  uint16_t reserved;
  float    r_ohm;          /* measured phase resistance */
  uint32_t checksum;
} NvConfig_t;

/** Loads the stored config; fills defaults and returns 0 if it is missing or corrupt. */
uint8_t NvConfig_Load(NvConfig_t *cfg);

/** Erases the page and writes cfg. Returns 1 on success (read-back verified). */
uint8_t NvConfig_Save(NvConfig_t *cfg);

void NvConfig_Defaults(NvConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* INC_NV_CONFIG_H_ */
