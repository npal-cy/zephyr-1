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
 * @brief Counter driver for Infineon CAT1 MCU family.
 */

#define DT_DRV_COMPAT infineon_mtbhal_counter

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/counter.h>
#include "cyhal_timer.h"
#include <zephyr/device.h>
#define LOG_LEVEL CONFIG_COUNTER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_infineon_cat1);

/* Default Counter interrupt priority */
#if !defined(COUNTER_CAT1_INTERRUPT_PRIORITY)
	#define COUNTER_CAT1_INTERRUPT_PRIORITY      (3u)
#endif

/* Device config structure */
struct counter_cat1_config {
	struct counter_config_info counter_info;
	cyhal_resource_inst_t hw_resource;
	uint32_t irqn;
};

/* Data structure */
struct counter_cat1_data {
	cyhal_timer_t counter_obj;
	cyhal_timer_cfg_t counter_cfg;
	struct counter_alarm_cfg alarm_cfg_counter;
	struct counter_top_cfg top_value_cfg_counter;
	uint32_t guard_period;
	bool alarm_irq_flag;
};

#define DEV_DATA(dev) \
	((struct counter_cat1_data *const)((dev)->data))

#define DEV_CFG(dev) \
	((struct counter_cat1_config *const)((dev)->config))


static void counter_event_callback(void *callback_arg, cyhal_timer_event_t event)
{
	const struct device *dev = (const struct device *) callback_arg;
	struct counter_cat1_data *data = DEV_DATA(dev);

	/* Alarm compare/capture event */
	if ((data->alarm_cfg_counter.callback != NULL) &&
	    (((CYHAL_TIMER_IRQ_CAPTURE_COMPARE & event) == CYHAL_TIMER_IRQ_CAPTURE_COMPARE) ||
	     data->alarm_irq_flag)) {
		/* Alarm works as one-shot, so disable event */
		cyhal_timer_enable_event(&data->counter_obj,
					 CYHAL_TIMER_IRQ_CAPTURE_COMPARE,
					 COUNTER_CAT1_INTERRUPT_PRIORITY,
					 false);

		/* Call User callback for Alarm */
		data->alarm_cfg_counter.callback(dev, 1, cyhal_timer_read(&data->counter_obj),
						 data->alarm_cfg_counter.user_data);
		data->alarm_irq_flag = false;
	}

	/* Top_value terminal count event */
	if ((data->top_value_cfg_counter.callback != NULL) &&
	    ((CYHAL_TIMER_IRQ_TERMINAL_COUNT & event) == CYHAL_TIMER_IRQ_TERMINAL_COUNT)) {

		/* Call User callback for top value */
		data->top_value_cfg_counter.callback(dev, data->top_value_cfg_counter.user_data);
	}

	/* NOTE: cyhal handles cleaning of interrupts */
}

static void counter_cat1_set_int_pending(const struct device *dev)
{
	__ASSERT_NO_MSG(dev != NULL);

	const struct counter_cat1_config *config = DEV_CFG(dev);
	struct counter_cat1_data *data = DEV_DATA(dev);

	cyhal_timer_enable_event(&data->counter_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE,
				 COUNTER_CAT1_INTERRUPT_PRIORITY, true);
	NVIC_SetPendingIRQ((IRQn_Type)config->irqn);
}

static int counter_cat1_init(const struct device *dev)
{
	__ASSERT_NO_MSG(dev != NULL);

	cy_rslt_t rslt;
	struct counter_cat1_data *data = DEV_DATA(dev);
	const struct counter_cat1_config *config = DEV_CFG(dev);

	/* Default Counter configuration structure */
	const cy_stc_tcpwm_counter_config_t _cyhal_timer_default_config = {
		.period = 32768,
		.clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
		.runMode = CY_TCPWM_COUNTER_CONTINUOUS,
		.countDirection = CY_TCPWM_COUNTER_COUNT_UP,
		.compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE,
		.compare0 = 16384,
		.compare1 = 16384,
		.enableCompareSwap = false,
		.interruptSources = CY_TCPWM_INT_NONE,
		.captureInputMode = 0x3U,
		.captureInput = CY_TCPWM_INPUT_0,
		.reloadInputMode = 0x3U,
		.reloadInput = CY_TCPWM_INPUT_0,
		.startInputMode = 0x3U,
		.startInput = CY_TCPWM_INPUT_0,
		.stopInputMode = 0x3U,
		.stopInput = CY_TCPWM_INPUT_0,
		.countInputMode = 0x3U,
		.countInput = CY_TCPWM_INPUT_1,
	};

	cyhal_timer_configurator_t timer_configurator = {
		.resource = &config->hw_resource,
		.config = &_cyhal_timer_default_config,
	};

	/* Initialize timer */
	rslt = cyhal_timer_init_cfg(&data->counter_obj, &timer_configurator);
	if (rslt != CY_RSLT_SUCCESS) {
		return -EIO;
	}

	/* Initialize counter structure */
	data->alarm_irq_flag = false;
	data->counter_cfg.compare_value = 0;
	data->counter_cfg.period = config->counter_info.max_top_value;
	data->counter_cfg.direction = CYHAL_TIMER_DIR_UP;
	data->counter_cfg.is_compare = true;
	data->counter_cfg.is_continuous = true;
	data->counter_cfg.value = 0;

	/* Configure timer */
	rslt = cyhal_timer_configure(&data->counter_obj, &data->counter_cfg);
	if (rslt != CY_RSLT_SUCCESS) {
		return -EIO;
	}

	/* Configure frequency */
	rslt = cyhal_timer_set_frequency(&data->counter_obj, config->counter_info.freq);
	if (rslt != CY_RSLT_SUCCESS) {
		return -EIO;
	}

	/* Register timer event callback */
	cyhal_timer_register_callback(&data->counter_obj, counter_event_callback, (void *) dev);

	return 0;
}

static int counter_cat1_start(const struct device *dev)
{
	__ASSERT_NO_MSG(dev != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);

	if (cyhal_timer_start(&data->counter_obj) != CY_RSLT_SUCCESS) {
		return -EIO;
	}
	return 0;
}

static int counter_cat1_stop(const struct device *dev)
{
	__ASSERT_NO_MSG(dev != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);

	if (cyhal_timer_stop(&data->counter_obj) != CY_RSLT_SUCCESS) {
		return -EIO;
	}
	return 0;
}

static int counter_cat1_get_value(const struct device *dev, uint32_t *ticks)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(ticks != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);

	*ticks = cyhal_timer_read(&data->counter_obj);

	return 0;
}

static int counter_cat1_set_top_value(const struct device *dev,
				      const struct counter_top_cfg *cfg)
{
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(cfg != NULL);

	cy_rslt_t rslt;
	struct counter_cat1_data *data = DEV_DATA(dev);
	const struct counter_cat1_config *config = DEV_CFG(dev);
	bool ticks_gt_period = false;

	data->top_value_cfg_counter = *cfg;
	data->counter_cfg.period = cfg->ticks;

	/* Check new top value limit */
	if (cfg->ticks > config->counter_info.max_top_value) {
		return -ENOTSUP;
	}

	ticks_gt_period = cfg->ticks > data->counter_cfg.period;
	/* Checks if new period value is not less then old period value */
	if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		data->counter_cfg.value = 0u;
	} else if (ticks_gt_period && (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE)) {
		data->counter_cfg.value = 0u;
	} else {
		/* cyhal_timer_configure resets timer counter register to value
		 * defined in config structure 'counter_cfg.value', so update
		 * counter value with current value of counter (read by
		 * cyhal_timer_read function).
		 */
		data->counter_cfg.value = cyhal_timer_read(&data->counter_obj);
	}

	if ((ticks_gt_period == false) ||
	    ((ticks_gt_period == true) && (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE))) {
		/* Reconfigure timer */
		rslt = cyhal_timer_configure(&data->counter_obj, &data->counter_cfg);
		if (rslt != CY_RSLT_SUCCESS) {
			return -EIO;
		}

		/* Register an top_value terminal count event callback handler if
		 * callback is not NULL.
		 */
		if (cfg->callback != NULL) {
			cyhal_timer_enable_event(&data->counter_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
						 COUNTER_CAT1_INTERRUPT_PRIORITY, true);
		}
	}
	return 0;
}

static uint32_t counter_cat1_get_top_value(const struct device *dev)
{
	__ASSERT_NO_MSG(dev != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);

	return data->counter_cfg.period;
}

static inline bool counter_is_bit_mask(uint32_t val)
{
	/* Return true if value equals 2^n - 1 */
	return !(val & (val + 1U));
}

static uint32_t counter_cat1_ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
{
	uint32_t to_top;

	/* refer to https://tbrindus.ca/how-builtin-expect-works/ for 'likely' usage */
	if (likely(counter_is_bit_mask(top))) {
		return (val1 + val2) & top;
	}

	to_top = top - val1;

	return (val2 <= to_top) ? (val1 + val2) : (val2 - to_top - 1U);
}

static uint32_t counter_cat1_ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	/* refer to https://tbrindus.ca/how-builtin-expect-works/ for 'likely' usage */
	if (likely(counter_is_bit_mask(top))) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : (val + top + 1U - old);
}

static int counter_cat1_set_alarm(const struct device *dev, uint8_t chan_id,
				  const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan_id);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(alarm_cfg != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);
	uint32_t val = alarm_cfg->ticks;
	uint32_t top_val = counter_cat1_get_top_value(dev);
	uint32_t flags = alarm_cfg->flags;
	uint32_t max_rel_val;
	bool absolute = ((flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) ? false : true;
	bool irq_on_late;
	int err = 0;

	/* Checks if compare value is not less then period value */
	if (alarm_cfg->ticks > top_val) {
		return -EINVAL;
	}

	if (absolute) {
		max_rel_val = top_val - data->guard_period;
		irq_on_late = ((flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) == 0) ? false : true;
	} else {
		/* If relative value is smaller than half of the counter range it is assumed
		 * that there is a risk of setting value too late and late detection algorithm
		 * must be applied. When late setting is detected, interrupt shall be
		 * triggered for immediate expiration of the timer. Detection is performed
		 * by limiting relative distance between CC and counter.
		 *
		 * Note that half of counter range is an arbitrary value.
		 */
		irq_on_late = val < (top_val / 2U);

		/* limit max to detect short relative being set too late. */
		max_rel_val = irq_on_late ? (top_val / 2U) : top_val;
		val = counter_cat1_ticks_add(cyhal_timer_read(&data->counter_obj), val, top_val);
	}

	/* Decrement value to detect also case when val == counter_read(dev). Otherwise,
	 * condition would need to include comparing diff against 0.
	 */
	uint32_t curr = cyhal_timer_read(&data->counter_obj);
	uint32_t diff = counter_cat1_ticks_sub((val - 1), curr, top_val);

	if ((absolute && (val < curr)) || (diff > max_rel_val)) {
		if (absolute) {
			err = -ETIME;
		}

		/* Interrupt is triggered always for relative alarm and for absolute depending
		 * on the flag.
		 */
		if (irq_on_late) {
			data->alarm_irq_flag = true;
			counter_cat1_set_int_pending(dev);
		}
	} else {
		/* Setting new compare value */
		cy_rslt_t rslt;

		data->alarm_cfg_counter = *alarm_cfg;
		data->counter_cfg.compare_value = val;

		/* cyhal_timer_configure resets timer counter register to value
		 * defined in config structure 'counter_cfg.value', so update
		 * counter value with current value of counter (read by
		 * cyhal_timer_read function).
		 */
		data->counter_cfg.value = cyhal_timer_read(&data->counter_obj);

		/* Reconfigure timer */
		rslt = cyhal_timer_configure(&data->counter_obj, &data->counter_cfg);
		if (rslt != CY_RSLT_SUCCESS) {
			return -EINVAL;
		}

		cyhal_timer_enable_event(&data->counter_obj,
					 CYHAL_TIMER_IRQ_CAPTURE_COMPARE,
					 COUNTER_CAT1_INTERRUPT_PRIORITY, true);
	}

	return err;
}

static int counter_cat1_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);
	__ASSERT_NO_MSG(dev != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);

	cyhal_timer_enable_event(&data->counter_obj, CYHAL_TIMER_IRQ_CAPTURE_COMPARE,
				 COUNTER_CAT1_INTERRUPT_PRIORITY, false);
	return 0;
}

static uint32_t counter_cat1_get_pending_int(const struct device *dev)
{
	__ASSERT_NO_MSG(dev != NULL);

	const struct counter_cat1_config *config = DEV_CFG(dev);

	return NVIC_GetPendingIRQ((IRQn_Type)config->irqn);
}

static uint32_t counter_cat1_get_guard_period(const struct device *dev, uint32_t flags)
{
	ARG_UNUSED(flags);
	__ASSERT_NO_MSG(dev != NULL);

	struct counter_cat1_data *data = DEV_DATA(dev);

	return data->guard_period;
}

static int counter_cat1_set_guard_period(const struct device *dev, uint32_t guard,
					 uint32_t flags)
{
	ARG_UNUSED(flags);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(guard < counter_cat1_get_top_value(dev));

	struct counter_cat1_data *data = DEV_DATA(dev);

	data->guard_period = guard;
	return 0;
}

/* Counter API structure */
static const struct counter_driver_api counter_api = {
	.start = counter_cat1_start,
	.stop = counter_cat1_stop,
	.get_value = counter_cat1_get_value,
	.set_alarm = counter_cat1_set_alarm,
	.cancel_alarm = counter_cat1_cancel_alarm,
	.set_top_value = counter_cat1_set_top_value,
	.get_pending_int = counter_cat1_get_pending_int,
	.get_top_value = counter_cat1_get_top_value,
	.get_guard_period = counter_cat1_get_guard_period,
	.set_guard_period = counter_cat1_set_guard_period,
};

/* Counter driver init macros */
#define INFINEON_CAT1_COUNTER_INIT(n)							 \
											 \
	static struct counter_cat1_data counter_cat1_data_##n = { 0 };			 \
											 \
	static const struct counter_cat1_config counter_cat1_config_##n = {		 \
		.counter_info = {							 \
				.max_top_value = (DT_INST_PROP(n, resolution) == 32)	 \
				? UINT32_MAX : UINT16_MAX,				 \
				.freq = DT_INST_PROP(n, clock_frequency),		 \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,			 \
				.channels = 1						 \
			},								 \
											 \
		.hw_resource = {							 \
				.type = CYHAL_RSC_TCPWM,				 \
				.block_num = DT_INST_PROP_BY_IDX(n, peripheral_id, 0),	 \
				.channel_num = DT_INST_PROP_BY_IDX(n, peripheral_id, 0), \
			},								 \
		.irqn = DT_INST_IRQN(n)							 \
	};										 \
											 \
	DEVICE_DT_INST_DEFINE(n,							 \
			      counter_cat1_init,					 \
			      NULL, &counter_cat1_data_##n,				 \
			      &counter_cat1_config_##n, PRE_KERNEL_1,			 \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_api);

DT_INST_FOREACH_STATUS_OKAY(INFINEON_CAT1_COUNTER_INIT);
