/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* This is enpty/stub file used in WHD */
#define CYBSP_SDIO_INTERFACE (0)
#define CYBSP_SPI_INTERFACE (1)

// you can define CYBSP_WIFI_INTERFACE_TYPE based on Kconfig, assuming you make these kconfig options
#ifdef CONFIG_AIROC_WIFI_BUS_SDIO
#define CYBSP_WIFI_INTERFACE_TYPE CYBSP_SDIO_INTERFACE
#elif CONFIG_AIROC_WIFI_BUS_SPI
#define CYBSP_WIFI_INTERFACE_TYPE CYBSP_SPI_INTERFACE
#endif
