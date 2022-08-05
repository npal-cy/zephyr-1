/* Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
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
 */

/**
 * @brief UART driver for Infineon CAT1 MCU family.
 *
 * Note:
 * - Uart ASYNC functionality is not implemented in current
 *   version of Uart CAT1 driver.
 */

#define DT_DRV_COMPAT infineon_mtbhal_uart

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include "cyhal_uart.h"

/* Default UART interrupt priority */
#define UART_CAT1_INTERRUPT_PRIORITY      (5u)

/* Map UART pins in Pinctrl array */
#define UART_TX_PIN   (CAT1_PIN_SIGNAL_SCB_UART_TX)     /* TX pin */
#define UART_RX_PIN   (CAT1_PIN_SIGNAL_SCB_UART_RX)     /* RX pin */
#define UART_RTS_PIN  (CAT1_PIN_SIGNAL_SCB_UART_RTS)    /* RTS pin */
#define UART_CTS_PIN  (CAT1_PIN_SIGNAL_SCB_UART_CTS)    /* CTS pin */

/* Data structure */
struct uart_cat1_data {
	cyhal_uart_t obj;                          /* UART CYHAL object */
	struct uart_config cfg;

	#if CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_cb;           /* Interrupt Callback */
	void *irq_cb_data;                              /* Interrupt Callback Arg */
	#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	#ifdef CONFIG_UART_ASYNC_API
	uart_callback_t async_cb;
	void *async_cb_data;
	void *async_rx_next_buf;
	size_t async_rx_next_buf_len;
	bool async_rx_enabled;
	#endif /* CONFIG_UART_ASYNC_API */
};

/* Device config structure */
struct uart_cat1_config {
	const struct pinctrl_dev_config *pcfg;
	struct uart_config z_cfg;
};

#define DEV_DATA(dev) \
	((struct uart_cat1_data *const)(dev)->data)

#define DEV_CFG(dev) \
	((struct uart_cat1_config *const)(dev)->config)

/* Internal function to get cyhal_gpio_t from pinctrl array by defined index. */
extern cyhal_gpio_t cat1_pinctrl_get_cyhal_gpio(const struct pinctrl_dev_config *config,
						uint8_t signal_name);

/* Helper API */
static cyhal_uart_parity_t _convert_uart_parity_z_to_cyhal(uint8_t z_parity)
{
	cyhal_uart_parity_t parity;

	switch ((enum uart_config_parity) z_parity) {
	case UART_CFG_PARITY_NONE:
		parity = CYHAL_UART_PARITY_NONE;
		break;
	case UART_CFG_PARITY_ODD:
		parity = CYHAL_UART_PARITY_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		parity = CYHAL_UART_PARITY_EVEN;
		break;
	default:
		parity = CYHAL_UART_PARITY_NONE;
	}
	return (parity);
}

static uint32_t _convert_uart_stop_bits_z_to_cyhal(uint8_t z_stop_bits)
{
	uint32_t stop_bits;

	switch ((enum uart_config_stop_bits) z_stop_bits) {
	case UART_CFG_STOP_BITS_1:
		stop_bits = 1u;
		break;

	case UART_CFG_STOP_BITS_2:
		stop_bits = 2u;
		break;
	default:
		stop_bits = 1u;
	}
	return (stop_bits);
}

static uint32_t _convert_uart_data_bits_z_to_cyhal(uint8_t z_data_bits)
{
	uint32_t data_bits;

	switch ((enum uart_config_data_bits) z_data_bits) {
	case UART_CFG_DATA_BITS_5:
		data_bits = 1u;
		break;

	case UART_CFG_DATA_BITS_6:
		data_bits = 6u;
		break;

	case UART_CFG_DATA_BITS_7:
		data_bits = 7u;
		break;

	case UART_CFG_DATA_BITS_8:
		data_bits = 8u;
		break;

	case UART_CFG_DATA_BITS_9:
		data_bits = 9u;
		break;

	default:
		data_bits = 1u;
	}
	return (data_bits);
}


/* UART CONSOLE AND CONFIGURATION API */
static int uart_cat1_poll_in(const struct device *dev, unsigned char *c)
{
	cy_rslt_t rec;

	rec = cyhal_uart_getc(&DEV_DATA(dev)->obj, c, 0u);

	return ((rec == CY_SCB_UART_RX_NO_DATA) ? -1 : 0);
}

static void uart_cat1_poll_out(const struct device *dev, unsigned char c)
{
	(void) cyhal_uart_putc(&DEV_DATA(dev)->obj, (uint32_t)c);
}

static int uart_cat1_err_check(const struct device *dev)
{
	uint32_t status = Cy_SCB_UART_GetRxFifoStatus(DEV_DATA(dev)->obj.base);
	int errors = 0;

	if (status & CY_SCB_UART_RX_OVERFLOW) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (status & CY_SCB_UART_RX_ERR_PARITY) {
		errors |= UART_ERROR_PARITY;
	}

	if (status & CY_SCB_UART_RX_ERR_FRAME) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}

static int uart_cat1_configure(const struct device *dev,
			       const struct uart_config *cfg)
{
	cy_rslt_t result;
	struct uart_cat1_data *data = DEV_DATA(dev);

	bool enable_cts = (data->obj.pin_cts != NC);
	bool enable_rts = (data->obj.pin_rts != NC);

	if (cfg == NULL) {
		return -EINVAL;
	}

	cyhal_uart_cfg_t uart_cfg = {
		.data_bits = _convert_uart_data_bits_z_to_cyhal(cfg->data_bits),
		.stop_bits = _convert_uart_stop_bits_z_to_cyhal(cfg->stop_bits),
		.parity = _convert_uart_parity_z_to_cyhal(cfg->parity)
	};

	/* Store Uart Zephyr configuration (uart config) into data structure */
	data->cfg = *cfg;

	/* Configure parity, data and stop bits */
	result = cyhal_uart_configure(&data->obj, &uart_cfg);

	/* Configure the baud rate */
	if (result == CY_RSLT_SUCCESS) {
		result = cyhal_uart_set_baud(&data->obj, cfg->baudrate, NULL);
	}

	/* Enable flow control */
	if ((result == CY_RSLT_SUCCESS) && (enable_cts || enable_rts)) {
		result = cyhal_uart_enable_flow_control(&data->obj, enable_cts,
							enable_rts);
	}
	return (result == CY_RSLT_SUCCESS) ? 0 : -1;
};

static int uart_cat1_config_get(const struct device *dev,
				struct uart_config *cfg)
{
	ARG_UNUSED(dev);

	if (cfg == NULL) {
		return -EINVAL;
	}

	*cfg = DEV_DATA(dev)->cfg;
	return 0;
}

/* Asynchronous UART API */
#ifdef CONFIG_UART_ASYNC_API

static int uart_cat1_async_cb_set(const struct device *dev,
				  uart_callback_t callback, void *user_data)
{
	return -ENOSYS;
}

static int uart_cat1_async_tx(const struct device *dev,
			      const uint8_t *tx_data, size_t buf_size, int32_t timeout)
{
	return -ENOSYS;
}

static int uart_cat1_async_tx_abort(const struct device *dev)
{
	return -ENOSYS;
}

static int uart_cat1_async_rx_enable(const struct device *dev, uint8_t *rx_buf,
				     size_t buf_size, int32_t timeout)
{
	return -ENOSYS;
}

static int uart_cat1_async_rx_disable(const struct device *dev)
{
	return -ENOSYS;
}

static int uart_cat1_async_rx_buf_rsp(const struct device *dev, uint8_t *buf,
				      size_t len)
{
	return -ENOSYS;
}
#endif /* CONFIG_UART_ASYNC_API */


/* UART INTERRUPT DRIVEN API */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN

/* Uart event callback for Interrupt driven mode */
static void _uart_event_callback_irq_mode(void *arg, cyhal_uart_event_t event)
{
	ARG_UNUSED(event);

	const struct device *dev = (const struct device *) arg;
	struct uart_cat1_data *const data = DEV_DATA(dev);

	if (data->irq_cb != NULL) {
		data->irq_cb(dev, data->irq_cb_data);
	}
}

/* Fill FIFO with data. */
static int uart_cat1_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data, int size)
{
	size_t _size = (size_t) size;

	(void)cyhal_uart_write(&DEV_DATA(dev)->obj, (uint8_t *) tx_data,  &_size);
	return (int) _size;
}

/* Read data from FIFO. */
static int uart_cat1_fifo_read(const struct device *dev,
			       uint8_t *rx_data, const int size)
{
	size_t _size = (size_t) size;

	(void)cyhal_uart_read(&DEV_DATA(dev)->obj, rx_data, &_size);
	return (int) _size;
}

/* Enable TX interrupt */
static void uart_cat1_irq_tx_enable(const struct device *dev)
{
	cyhal_uart_enable_event(&DEV_DATA(dev)->obj,
				(cyhal_uart_event_t) CYHAL_UART_IRQ_TX_EMPTY,
				UART_CAT1_INTERRUPT_PRIORITY, 1);
}

/* Disable TX interrupt */
static void uart_cat1_irq_tx_disable(const struct device *dev)
{
	cyhal_uart_enable_event(&DEV_DATA(dev)->obj,
				(cyhal_uart_event_t) CYHAL_UART_IRQ_TX_EMPTY,
				UART_CAT1_INTERRUPT_PRIORITY, 0);
}

/* Check if UART TX buffer can accept a new char */
static int uart_cat1_irq_tx_ready(const struct device *dev)
{
	uint32_t mask = Cy_SCB_GetTxInterruptStatusMasked(DEV_DATA(dev)->obj.base);

	return (((mask & (CY_SCB_UART_TX_NOT_FULL | SCB_INTR_TX_EMPTY_Msk)) != 0u) ? 1 : 0);
}

/* Check if UART TX block finished transmission */
static int uart_cat1_irq_tx_complete(const struct device *dev)
{
	return (int) !(cyhal_uart_is_tx_active(&DEV_DATA(dev)->obj));
}

/* Enable RX interrupt */
static void uart_cat1_irq_rx_enable(const struct device *dev)
{
	cyhal_uart_enable_event(&DEV_DATA(dev)->obj,
				(cyhal_uart_event_t) CYHAL_UART_IRQ_RX_NOT_EMPTY,
				UART_CAT1_INTERRUPT_PRIORITY, 1);
}

/* Disable TX interrupt */
static void uart_cat1_irq_rx_disable(const struct device *dev)
{
	cyhal_uart_enable_event(&DEV_DATA(dev)->obj,
				(cyhal_uart_event_t) CYHAL_UART_IRQ_RX_NOT_EMPTY,
				UART_CAT1_INTERRUPT_PRIORITY, 0);
}

/* Check if UART RX buffer has a received char */
static int uart_cat1_irq_rx_ready(const struct device *dev)
{
	return cyhal_uart_readable(&DEV_DATA(dev)->obj) ? 1 : 0;
}

/* Enable Error interrupts */
static void uart_cat1_irq_err_enable(const struct device *dev)
{
	cyhal_uart_enable_event(&DEV_DATA(dev)->obj, (cyhal_uart_event_t)
				(CYHAL_UART_IRQ_TX_ERROR | CYHAL_UART_IRQ_RX_ERROR),
				UART_CAT1_INTERRUPT_PRIORITY, 1);
}

/* Disable Error interrupts */
static void uart_cat1_irq_err_disable(const struct device *dev)
{
	cyhal_uart_enable_event(&DEV_DATA(dev)->obj, (cyhal_uart_event_t)
				(CYHAL_UART_IRQ_TX_ERROR | CYHAL_UART_IRQ_RX_ERROR),
				UART_CAT1_INTERRUPT_PRIORITY, 0);
}

/* Check if any IRQs is pending */
static int uart_cat1_irq_is_pending(const struct device *dev)
{
	uint32_t intcause = Cy_SCB_GetInterruptCause(DEV_DATA(dev)->obj.base);

	return (int) (intcause & (CY_SCB_TX_INTR | CY_SCB_RX_INTR));
}

/* Start processing interrupts in ISR.
 * This function should be called the first thing in the ISR. Calling
 * uart_irq_rx_ready(), uart_irq_tx_ready(), uart_irq_tx_complete()
 * allowed only after this.
 */
static int uart_cat1_irq_update(const struct device *dev)
{
	int status = 1;

	if (((uart_cat1_irq_is_pending(dev) & CY_SCB_RX_INTR) != 0u) &&
	    (Cy_SCB_UART_GetNumInRxFifo(DEV_DATA(dev)->obj.base) == 0u)) {
		status = 0;
	}

	return status;
}

static void uart_cat1_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_cat1_data *data = DEV_DATA(dev);
	cyhal_uart_t *uart_obj = &data->obj;

	/* Store user callback info */
	data->irq_cb = cb;
	data->irq_cb_data = cb_data;

	/* Register a uart general callback handler  */
	cyhal_uart_register_callback(uart_obj, _uart_event_callback_irq_mode, (void *) dev);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_cat1_init(const struct device *dev)
{
	struct uart_cat1_config *const config = DEV_CFG(dev);

	cy_rslt_t result;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Initialize the UART peripheral */
	result = cyhal_uart_init(&DEV_DATA(dev)->obj,
				 cat1_pinctrl_get_cyhal_gpio(config->pcfg, UART_TX_PIN),
				 cat1_pinctrl_get_cyhal_gpio(config->pcfg, UART_RX_PIN),
				 cat1_pinctrl_get_cyhal_gpio(config->pcfg, UART_CTS_PIN),
				 cat1_pinctrl_get_cyhal_gpio(config->pcfg, UART_RTS_PIN),
				 NULL, NULL);

	if (result == CY_RSLT_SUCCESS) {
		ret = uart_cat1_configure(dev, &config->z_cfg);
	}

	return ret;
}

/* UART API STRUCTURE */
static const struct uart_driver_api uart_cat1_driver_api = {
	.poll_in = uart_cat1_poll_in,
	.poll_out = uart_cat1_poll_out,
	.err_check = uart_cat1_err_check,

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_cat1_configure,
	.config_get = uart_cat1_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_cat1_fifo_fill,
	.fifo_read = uart_cat1_fifo_read,
	.irq_tx_enable = uart_cat1_irq_tx_enable,
	.irq_tx_disable = uart_cat1_irq_tx_disable,
	.irq_tx_ready = uart_cat1_irq_tx_ready,
	.irq_rx_enable = uart_cat1_irq_rx_enable,
	.irq_rx_disable = uart_cat1_irq_rx_disable,
	.irq_tx_complete = uart_cat1_irq_tx_complete,
	.irq_rx_ready = uart_cat1_irq_rx_ready,
	.irq_err_enable = uart_cat1_irq_err_enable,
	.irq_err_disable = uart_cat1_irq_err_disable,
	.irq_is_pending = uart_cat1_irq_is_pending,
	.irq_update = uart_cat1_irq_update,
	.irq_callback_set = uart_cat1_irq_callback_set,
#endif    /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_cat1_async_cb_set,
	.tx = uart_cat1_async_tx,
	.tx_abort = uart_cat1_async_tx_abort,
	.rx_enable = uart_cat1_async_rx_enable,
	.rx_disable = uart_cat1_async_rx_disable,
	.rx_buf_rsp = uart_cat1_async_rx_buf_rsp,
#endif  /* CONFIG_UART_ASYNC_API */
};

/* UART DRIVER INIT MACROS */
#define INFINEON_CAT1_UART_INIT(n)					   \
	PINCTRL_DT_INST_DEFINE(n);					   \
	static struct uart_cat1_data infineon_cat1_uart##n##_data = { 0 }; \
									   \
	static struct uart_cat1_config infineon_cat1_uart##n##_cfg = {	   \
		.z_cfg.baudrate = DT_INST_PROP(n, current_speed),	   \
		.z_cfg.parity = DT_ENUM_IDX(DT_DRV_INST(n), parity),	   \
		.z_cfg.stop_bits = DT_INST_PROP(n, stop_bits),		   \
		.z_cfg.data_bits = DT_INST_PROP(n, data_bits),		   \
		.z_cfg.flow_ctrl = DT_INST_PROP(n, hw_flow_control),	   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)		   \
	};								   \
									   \
	DEVICE_DT_INST_DEFINE(n,					   \
			      &uart_cat1_init, NULL,			   \
			      &infineon_cat1_uart##n##_data,		   \
			      &infineon_cat1_uart##n##_cfg, PRE_KERNEL_1,  \
			      CONFIG_SERIAL_INIT_PRIORITY,		   \
			      &uart_cat1_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INFINEON_CAT1_UART_INIT)
