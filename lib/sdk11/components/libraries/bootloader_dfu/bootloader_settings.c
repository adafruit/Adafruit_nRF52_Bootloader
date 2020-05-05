/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "bootloader_settings.h"
#include <stdint.h>
#include <dfu_types.h>
#include "nrfx_nvmc.h"

/**< This variable reserves a codepage for bootloader specific settings, to ensure the compiler doesn't locate any code or variables at his location. */
__attribute__ ((section(".bootloaderSettings")))
uint8_t m_boot_settings[CODE_PAGE_SIZE];

/**< This variable ensures that the linker script will write the bootloader start address to the UICR register. This value will be written in the HEX file and thus written to UICR when the bootloader is flashed into the chip. */
 __attribute__ ((section(".uicrBootStartAddress")))
volatile uint32_t m_uicr_bootloader_start_address = BOOTLOADER_REGION_START;

/**< This variable reserves a codepage for mbr parameters, to ensure the compiler doesn't locate any code or variables at his location. */
  __attribute__ ((section(".mbrParamsPage")))
uint8_t m_mbr_params_page[CODE_PAGE_SIZE];

/**< This variable makes the linker script write the mbr parameters page address to the UICR register. This value will be written in the HEX file and thus written to the UICR when the bootloader is flashed into the chip */
__attribute__ ((section(".uicrMbrParamsPageAddress")))
volatile uint32_t m_uicr_mbr_params_page_address = BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS;

void bootloader_util_settings_get(const bootloader_settings_t ** pp_bootloader_settings)
{
    // Read only pointer to bootloader settings in flash. 
    bootloader_settings_t const * const p_bootloader_settings =
        (bootloader_settings_t *)&m_boot_settings[0];        

    *pp_bootloader_settings = p_bootloader_settings;
}

void bootloader_mbr_addrs_populate(void)
{
  if (*(const uint32_t *)MBR_BOOTLOADER_ADDR == 0xFFFFFFFF)
  {
    nrfx_nvmc_word_write(MBR_BOOTLOADER_ADDR, BOOTLOADER_REGION_START);
  }

  if (*(const uint32_t *)MBR_PARAM_PAGE_ADDR == 0xFFFFFFFF)
  {
    nrfx_nvmc_word_write(MBR_PARAM_PAGE_ADDR, BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS);
  }
}
