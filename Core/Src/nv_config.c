/*
 * nv_config.c
 *
 *      Author: CT
 */

#include "nv_config.h"

#include "foc_config.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#define NV_PAGE_ADDR   0x08007C00u        /* last 1 KB page of the 32 KB part */
#define NV_MAGIC       0x31434F46u        /* "FOC1" */
#define NV_VERSION     1u

_Static_assert((sizeof(NvConfig_t) % 4u) == 0u, "NvConfig_t must be a whole number of words");

static uint32_t nv_checksum(const NvConfig_t *cfg)
{
  /* FNV-1a over everything except the checksum itself */
  const uint8_t *p = (const uint8_t *)cfg;
  uint32_t h = 2166136261u;
  for (uint32_t i = 0u; i < offsetof(NvConfig_t, checksum); i++)
  {
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

void NvConfig_Defaults(NvConfig_t *cfg)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->magic = NV_MAGIC;
  cfg->version = NV_VERSION;
  cfg->enc_offset = 0u;
  cfg->enc_dir = 1;
  cfg->pole_pairs = FOC_POLE_PAIRS_DEFAULT;
  cfg->calibrated = 0u;
  cfg->can_id = FOC_CAN_ID_DEFAULT;
  cfg->zero_counts = 0u;
  cfg->r_ohm = FOC_MOTOR_R_OHM_DEFAULT;
}

uint8_t NvConfig_Load(NvConfig_t *cfg)
{
  memcpy(cfg, (const void *)NV_PAGE_ADDR, sizeof(*cfg));
  if ((cfg->magic != NV_MAGIC) || (cfg->version != NV_VERSION) || (cfg->checksum != nv_checksum(cfg))
      || (cfg->pole_pairs == 0u) || ((cfg->enc_dir != 1) && (cfg->enc_dir != -1)))
  {
    NvConfig_Defaults(cfg);
    return 0u;
  }
  return 1u;
}

uint8_t NvConfig_Save(NvConfig_t *cfg)
{
  cfg->magic = NV_MAGIC;
  cfg->version = NV_VERSION;
  cfg->checksum = nv_checksum(cfg);

  uint8_t ok = 1u;
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = NV_PAGE_ADDR;
  erase.NbPages = 1u;
  uint32_t page_error = 0u;
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    ok = 0u;
  }

  const uint32_t *src = (const uint32_t *)cfg;
  for (uint32_t i = 0u; ok && (i < sizeof(*cfg) / 4u); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, NV_PAGE_ADDR + 4u * i, src[i]) != HAL_OK)
    {
      ok = 0u;
    }
  }

  HAL_FLASH_Lock();

  return ok && (memcmp((const void *)NV_PAGE_ADDR, cfg, sizeof(*cfg)) == 0);
}
