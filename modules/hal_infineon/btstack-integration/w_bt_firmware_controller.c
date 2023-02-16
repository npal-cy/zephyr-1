/*
 * $ Copyright (C) 2020-2021 Infineon Technologies AG $
 */

#include <stdint.h>

const uint8_t brcm_patchram_format = 0x01;
/* Configuration Data Records (Write_RAM) */
#ifndef FW_DATBLOCK_SEPARATE_FROM_APPLICATION
const uint8_t brcm_patchram_buf[] =
{
   #include <bt_firmware.hcd.inc>
};

const int brcm_patch_ram_length = sizeof(brcm_patchram_buf);
#endif /* FW_DATBLOCK_SEPARATE_FROM_APPLICATION */
