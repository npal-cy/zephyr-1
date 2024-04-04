/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <airoc_wifi.h>

#include <bus_protocols/whd_bus_spi_protocol.h>
#include <bus_protocols/whd_bus.h>
#include <bus_protocols/whd_spi.h>
#include <zephyr/drivers/spi.h>

#define DT_DRV_COMPAT infineon_airoc_wifi

LOG_MODULE_DECLARE(infineon_airoc, CONFIG_WIFI_LOG_LEVEL);

#ifdef __cplusplus
extern "C" {
#endif

extern whd_resource_source_t resource_ops;

struct whd_bus_priv {
    whd_spi_config_t spi_config;
    whd_spi_t spi_obj;
};

static whd_init_config_t init_config_default = {
	.thread_stack_size = CY_WIFI_THREAD_STACK_SIZE,
	.thread_stack_start = NULL,
	.thread_priority = (uint32_t)CY_WIFI_THREAD_PRIORITY,
	.country = CY_WIFI_COUNTRY
};

/******************************************************
 *                 Function
 ******************************************************/

int airoc_wifi_init_bus(const struct device *dev, whd_interface_t *interface,
			    whd_netif_funcs_t *netif_funcs, whd_buffer_funcs_t *buffer_if)
{
	struct airoc_wifi_data *data = dev->data;
	const struct airoc_wifi_config *config = dev->config;

	whd_spi_config_t whd_spi_config = {
		.is_spi_normal_mode = WHD_FALSE,
	};

#if DT_INST_NODE_HAS_PROP(0, wifi_host_wake_gpios)
	whd_oob_config_t oob_config = {
		.host_oob_pin = (void *)&config->wifi_host_wake_gpio,
		.dev_gpio_sel = DEFAULT_OOB_PIN,
		.is_falling_edge =
			(CY_WIFI_HOST_WAKE_IRQ_EVENT == GPIO_INT_TRIG_LOW) ? WHD_TRUE : WHD_FALSE,
		.intr_priority = CY_WIFI_OOB_INTR_PRIORITY
	};
	whd_spi_config.oob_config = oob_config;
#endif

	if (!device_is_ready(config->bus_dev)) {
		LOG_ERR("SPI device is not ready");
		return -ENODEV;
	}

	/* Init wifi host driver (whd) */
	cy_rslt_t whd_ret = whd_init(&data->whd_drv, &init_config_default, &resource_ops, buffer_if,
				     netif_funcs);
	if (whd_ret == CY_RSLT_SUCCESS) {
		whd_ret = whd_bus_spi_attach(data->whd_drv, &whd_spi_config,
					      (whd_spi_t)&data->card);

		if (whd_ret == CY_RSLT_SUCCESS) {
			whd_ret = whd_wifi_on(data->whd_drv, interface);
		}

		if (whd_ret != CY_RSLT_SUCCESS) {
			whd_deinit(*interface);
			return -ENODEV;
		}
	}
	return 0;
}

/*
 * Implement SPI Transfer wrapper
 */

whd_result_t whd_bus_spi_transfer(whd_driver_t whd_driver, const uint8_t *tx, size_t tx_length,
			uint8_t *rx, size_t rx_length, uint8_t write_fill)
{
	const struct spi_dt_spec *spi_obj = whd_driver->bus_priv->spi_obj; 
	int ret;
	const struct spi_buf tx_buf = {
		.buf = (uint8_t *)tx,
		.len = tx_length
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf = {
		.buf = rx,
		.len = rx_length
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1
	};

	ret = spi_transceive_dt(spi_obj, &tx_set, &rx_set);
	if (ret) {
		LOG_DBG("spi_transceive FAIL %d\n", ret);
			return ret;
	}

	return 0;
}

/*
 * Is SPI interrupt required?
 */

whd_result_t whd_bus_spi_irq_register(whd_driver_t whd_driver)
{
	// DEBUG
	printf("whd_bus_spi_irq_register() called\n");
	// DEBUG END

	/* Nothing to do here, all handles by whd_bus_spi_irq_enable function */
	return WHD_SUCCESS;
}

whd_result_t whd_bus_spi_irq_enable(whd_driver_t whd_driver, whd_bool_t enable)
{
	// DEBUG
	printf("whd_bus_spi_irq_enable() called - enable=%d\n", enable);
	// DEBUG END

	/* Nothing to do here, all handles by whd_bus_spi_irq_enable function */
	return WHD_SUCCESS;
}

/*
 * Implement OOB functionality
 */

void whd_bus_spi_oob_irq_handler(const struct device *port, struct gpio_callback *cb,
				  gpio_port_pins_t pins)
{
#if DT_INST_NODE_HAS_PROP(0, wifi_host_wake_gpios)
	struct airoc_wifi_data *data = CONTAINER_OF(cb, struct airoc_wifi_data, host_oob_pin_cb);

	/* Get OOB pin info */
	const whd_oob_config_t *oob_config = &data->whd_drv->bus_priv->spi_config.oob_config;
	const struct gpio_dt_spec *host_oob_pin = oob_config->host_oob_pin;

	/* Check OOB state is correct */
	int expected_event = (oob_config->is_falling_edge == WHD_TRUE) ? 0 : 1;

	if (!(pins & BIT(host_oob_pin->pin)) || (gpio_pin_get_dt(host_oob_pin) != expected_event)) {
		WPRINT_WHD_ERROR(("Unexpected interrupt event %d\n", expected_event));
		WHD_BUS_STATS_INCREMENT_VARIABLE(data->whd_drv->bus_priv, error_intrs);
		return;
	}

	WHD_BUS_STATS_INCREMENT_VARIABLE(data->whd_drv->bus_priv, oob_intrs);

	/* Call thread notify to wake up WHD thread */
	whd_thread_notify_irq(data->whd_drv);

#endif /* DT_INST_NODE_HAS_PROP(0, wifi-host-wake-gpios) */
}

#ifdef __cplusplus
} /* extern "C" */
#endif
