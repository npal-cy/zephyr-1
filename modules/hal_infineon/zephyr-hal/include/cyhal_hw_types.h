/***************************************************************************//**
* \file cyhal_hw_types.h
*
* \brief
* Provides a struct definitions for configuration resources in the Zephyr PDL.
*
********************************************************************************
* \copyright
* Copyright 2018-2022 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/**
* \addtogroup group_hal_impl Zephyr Implementation Specific
* \{
* This section provides details about the ZEPHYR implementation of the Cypress HAL.
* All information within this section is platform specific and is provided for reference.
* Portable application code should depend only on the APIs and types which are documented
* in the @ref group_hal section.
*
* \section group_hal_impl_mapping HAL Resource Hardware Mapping
* The following table shows a mapping of each HAL driver to the lower level firmware driver
* and the corresponding hardware resource. This is intended to help understand how the HAL
* is implemented for ZEPHYR and what features the underlying hardware supports.
*/

#pragma once

/* Includes */

#if defined( __cplusplus)
extern "C" {
#endif

/* Set the Infineon API version for Zephyr */
#define CYHAL_API_VERSION                   (2)

/**
  * @brief SPI object
  *
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases.
  */
typedef struct {
    void *empty;
} cyhal_spi_t;

/* Fake the definition for the GPIO type */
typedef uint32_t cyhal_gpio_t;

/** @brief Clock object
  * Application code should not rely on the specific contents of this struct.
  * They are considered an implementation detail which is subject to change
  * between platforms and/or HAL releases. */
typedef struct
{
    uint32_t                block;
    uint8_t                 channel;
    bool                    reserved;
    const void*             funcs;
} cyhal_clock_t;

/**
  * @brief SPI configurator struct
  *
  * This struct allows a configurator to provide block configuration information
  * to the HAL. Because configurator-generated configurations are platform
  * specific, the contents of this struct is subject to change between platforms
  * and/or HAL releases.
  */
typedef struct {
#if defined(CY_IP_MXSCB) || defined(CY_IP_MXS22SCB)
    const cyhal_resource_inst_t*            resource;
    const cy_stc_scb_spi_config_t*          config;
    const cyhal_clock_t*                    clock;
    struct
    {
        cyhal_gpio_t                        sclk;
        cyhal_gpio_t                        ssel[4];
        cyhal_gpio_t                        mosi;
        cyhal_gpio_t                        miso;
    } gpios;
#else
    void *empty;
#endif /* defined(CY_IP_MXSCB) || defined(CY_IP_MXS22SCB) */
} cyhal_spi_configurator_t;

/** Fake the definition of the trigger source enum */
typedef uint32_t cyhal_source_t;

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/** \} group_hal_impl_hw_types */
/** \} group_hal_impl */
