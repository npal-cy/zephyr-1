/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define SLEEP_TIME_MS   2000
#define ADV_TIME_MS	500

 /* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device* const h4_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_uart));


/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
    BT_DATA_BYTES(BT_DATA_SVC_DATA16,
              0xaa, 0xfe, /* Eddystone UUID */
              0x10, /* Eddystone-URL frame type */
              0x00, /* Calibrated Tx power at 0m */
              0x00, /* URL Scheme Prefix http://www. */
              'z', 'e', 'p', 'h', 'y', 'r',
              'p', 'r', 'o', 'j', 'e', 'c', 't',
              0x08) /* .org */
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/** Non-connectable advertising with @ref BT_LE_ADV_OPT_USE_IDENTITY */
#define BT_LE_ADV_NCONN_IDENTITY2 BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
						 BT_GAP_ADV_FAST_INT_MIN_1, \
						 BT_GAP_ADV_FAST_INT_MAX_1, \
						 NULL)

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
}

int main(void)
{
    int err;

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        return 0;
    }

    printk("Starting Beacon Demo\n");

    /* Block PM */
    pm_device_busy_set(h4_dev);

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    /* Wait when BT stack will start */
    while (!bt_is_ready())
    {
        k_msleep(10);
    }

    while (1) {
        char addr_s[BT_ADDR_LE_STR_LEN];
        bt_addr_le_t addr = { 0 };
        size_t count = 1;
        bool led_state = true;

        /* Block PM */
        pm_device_busy_set(h4_dev);
	k_msleep(10);

        /* Start advertising */
        err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY2, ad, ARRAY_SIZE(ad),
            sd, ARRAY_SIZE(sd));
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
            return;
        }

        /* For connectable advertising you would use
         * bt_le_oob_get_local().  For non-connectable non-identity
         * advertising an non-resolvable private address is used;
         * there is no API to retrieve that.
         */

        bt_id_get(&addr, &count);
        bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));
        printk("Beacon started, advertising as %s\n", addr_s);

        /* Wait some time for advertising... */
        k_msleep(ADV_TIME_MS);

        /* Stop advertising */
        bt_le_adv_stop();
        printk("Beacon stoped\n");


        err = gpio_pin_toggle_dt(&led);
        if (err < 0) {
            return 0;
        }
        led_state = !led_state;

        /* Unblock PM to keed system in lowpower...during long sleep*/
        pm_device_busy_clear(h4_dev);
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
