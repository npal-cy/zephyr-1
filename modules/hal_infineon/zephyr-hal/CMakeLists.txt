# Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation
#
# SPDX-License-Identifier: Apache-2.0

# Add local header overrides
zephyr_include_directories(include)

# Add core-lib
zephyr_include_directories(${ZEPHYR_HAL_INFINEON_MODULE_DIR}/core-lib/include)

zephyr_library_sources_ifdef(CONFIG_AIROC_WIFI_BUS_SPI	cyhal_system.c)

# DEBUG
# zephyr_compile_definitions_ifdef(CONFIG_INFINEON_ZEPHYR_WIFI CY_USING_HAL)
# DEBUG END
