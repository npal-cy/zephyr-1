/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <airoc_wifi.h>

#include <bus_protocols/whd_bus.h>

#define DT_DRV_COMPAT infineon_airoc_wifi

LOG_MODULE_REGISTER(infineon_airoc, CONFIG_WIFI_LOG_LEVEL);

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                 Function
 ******************************************************/

int airoc_wifi_power_on(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, wifi_reg_on_gpios)
	int ret;
	const struct airoc_wifi_config *config = dev->config;

	/* Check WIFI REG_ON gpio instance */
	if (!device_is_ready(config->wifi_reg_on_gpio.port)) {
		LOG_ERR("Error: failed to configure wifi_reg_on %s pin %d",
			config->wifi_reg_on_gpio.port->name, config->wifi_reg_on_gpio.pin);
		return -EIO;
	}

	/* Configure wifi_reg_on as output  */
	ret = gpio_pin_configure_dt(&config->wifi_reg_on_gpio, GPIO_OUTPUT);
	if (ret) {
		LOG_ERR("Error %d: failed to configure wifi_reg_on %s pin %d", ret,
			config->wifi_reg_on_gpio.port->name, config->wifi_reg_on_gpio.pin);
		return ret;
	}
	ret = gpio_pin_set_dt(&config->wifi_reg_on_gpio, 0);
	if (ret) {
		return ret;
	}

	/* Allow CBUCK regulator to discharge */
	k_msleep(WLAN_CBUCK_DISCHARGE_MS);

	/* WIFI power on */
	ret = gpio_pin_set_dt(&config->wifi_reg_on_gpio, 1);
	if (ret) {
		return ret;
	}
	k_msleep(WLAN_POWER_UP_DELAY_MS);
#endif /* DT_INST_NODE_HAS_PROP(0, reg_on_gpios) */

	return 0;
}

int airoc_wifi_init_primary(const struct device *dev, whd_interface_t *interface,
			    whd_netif_funcs_t *netif_funcs, whd_buffer_funcs_t *buffer_if)
{
	int ret;

	if (airoc_wifi_power_on(dev)) {
		LOG_ERR("airoc_wifi_power_on returns fail");
		return -ENODEV;
	}

	/* Initialize the configured bus interface */
	ret = airoc_wifi_init_bus(dev, interface, netif_funcs, buffer_if);

	return ret;
}

/*
 * Implement WHD memory wrappers
 */

void *whd_mem_malloc(size_t size)
{
	return k_malloc(size);
}

void *whd_mem_calloc(size_t nitems, size_t size)
{
	return k_calloc(nitems, size);
}

void whd_mem_free(void *ptr)
{
	k_free(ptr);
}

#ifdef __cplusplus
} /* extern "C" */
#endif
