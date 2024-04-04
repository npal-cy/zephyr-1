/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <whd_buffer_api.h>
#if defined(CONFIG_AIROC_WIFI_BUS_SDIO)
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>
#endif
#if defined(CONFIG_AIROC_WIFI_BUS_SPI)
#include <zephyr/drivers/spi.h>
#endif
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/wifi_mgmt.h>
#include <cy_utils.h>

/** Defines the amount of stack memory available for the wifi thread. */
#if !defined(CY_WIFI_THREAD_STACK_SIZE)
#define CY_WIFI_THREAD_STACK_SIZE (5120)
#endif

/** Defines the priority of the thread that services wifi packets. Legal values are defined by the
 *  RTOS being used.
 */
#if !defined(CY_WIFI_THREAD_PRIORITY)
#define CY_WIFI_THREAD_PRIORITY (CY_RTOS_PRIORITY_HIGH)
#endif

/** Defines the country this will operate in for wifi initialization parameters. See the
 *  wifi-host-driver's whd_country_code_t for legal options.
 */
#if !defined(CY_WIFI_COUNTRY)
#define CY_WIFI_COUNTRY (WHD_COUNTRY_AUSTRALIA)
#endif

/** Defines the priority of the interrupt that handles out-of-band notifications from the wifi
 *  chip. Legal values are defined by the MCU running this code.
 */
#if !defined(CY_WIFI_OOB_INTR_PRIORITY)
#define CY_WIFI_OOB_INTR_PRIORITY (2)
#endif

/** Defines whether to use the out-of-band pin to allow the WIFI chip to wake up the MCU. */
#if defined(CY_WIFI_HOST_WAKE_SW_FORCE)
#define CY_USE_OOB_INTR (CY_WIFI_HOST_WAKE_SW_FORCE)
#else
#define CY_USE_OOB_INTR (1u)
#endif /* defined(CY_WIFI_HOST_WAKE_SW_FORCE) */

#define CY_WIFI_HOST_WAKE_IRQ_EVENT GPIO_INT_TRIG_LOW
#define DEFAULT_OOB_PIN             (0)
#define WLAN_POWER_UP_DELAY_MS      (250)
#define WLAN_CBUCK_DISCHARGE_MS     (10)

struct airoc_wifi_data {
#if defined(CONFIG_AIROC_WIFI_BUS_SDIO)
	struct sd_card card;
	struct sdio_func sdio_func1;
	struct sdio_func sdio_func2;
#elif defined(CONFIG_AIROC_WIFI_BUS_SPI)
	struct spi_dt_spec card;
#endif
	struct net_if *iface;
	bool second_interface_init;
	bool is_ap_up;
	bool is_sta_connected;
	uint8_t mac_addr[6];
	scan_result_cb_t scan_rslt_cb;
	whd_ssid_t ssid;
	whd_scan_result_t scan_result;
	struct k_sem sema_common;
	struct k_sem sema_scan;
#if defined(CONFIG_NET_STATISTICS_WIFI)
	struct net_stats_wifi stats;
#endif
	whd_driver_t whd_drv;
	struct gpio_callback host_oob_pin_cb;
	uint8_t frame_buf[NET_ETH_MAX_FRAME_SIZE];
};

struct airoc_wifi_config {
	const struct device *bus_dev;
	struct gpio_dt_spec wifi_reg_on_gpio;
	struct gpio_dt_spec wifi_host_wake_gpio;
	struct gpio_dt_spec wifi_dev_wake_gpio;
};

/**
 * \brief This function returns pointer type to handle instance
 *        of whd interface (whd_interface_t) which allocated in
 *        Zephyr AIROC driver (drivers/wifi/infineon/airoc_wifi.c)
 */

whd_interface_t airoc_wifi_get_whd_interface(void);

/**
 * \brief Initialize the configured bus interface.
*/

int airoc_wifi_init_bus(const struct device *dev, whd_interface_t *interface,
			    whd_netif_funcs_t *netif_funcs, whd_buffer_funcs_t *buffer_if);
