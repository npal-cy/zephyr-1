/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief AIROC Wi-Fi driver.
 */

#define DT_DRV_COMPAT infineon_airoc_wifi

#include <zephyr/logging/log.h>
#include <airoc_wifi.h>

LOG_MODULE_REGISTER(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

#ifndef AIROC_WIFI_TX_PACKET_POOL_COUNT
#define AIROC_WIFI_TX_PACKET_POOL_COUNT (10)
#endif

#ifndef AIROC_WIFI_RX_PACKET_POOL_COUNT
#define AIROC_WIFI_RX_PACKET_POOL_COUNT (10)
#endif

#ifndef AIROC_WIFI_PACKET_POOL_SIZE
#define AIROC_WIFI_PACKET_POOL_SIZE (1600)
#endif

#define AIROC_WIFI_PACKET_POOL_COUNT                                                               \
	(AIROC_WIFI_TX_PACKET_POOL_COUNT + AIROC_WIFI_RX_PACKET_POOL_COUNT)

/* This macro is copy of NET_BUF_POOL_FIXED_DEFINE with aligning net_buf_data_##_name
 * WHD requires that network buffers is aligned, NET_BUF_POOL_FIXED_DEFINE does not
 * guarantees aligned.
 */
#define NET_BUF_POOL_FIXED_DEFINE_ALIGN(_name, _count, _data_size, _ud_size, _destroy)             \
	_NET_BUF_ARRAY_DEFINE(_name, _count, _ud_size);                                            \
	static uint8_t __noinit net_buf_data_##_name[_count][_data_size] __net_buf_align;          \
	static const struct net_buf_pool_fixed net_buf_fixed_##_name = {                           \
		.data_size = _data_size,                                                           \
		.data_pool = (uint8_t *)net_buf_data_##_name,                                      \
	};                                                                                         \
	static const struct net_buf_data_alloc net_buf_fixed_alloc_##_name = {                     \
		.cb = &net_buf_fixed_cb,                                                           \
		.alloc_data = (void *)&net_buf_fixed_##_name,                                      \
	};                                                                                         \
	static STRUCT_SECTION_ITERABLE(net_buf_pool, _name) = NET_BUF_POOL_INITIALIZER(            \
		_name, &net_buf_fixed_alloc_##_name, _net_buf_##_name, _count, _ud_size, _destroy)

#define AIROC_WIFI_WAIT_SEMA_MS (30 * 1000)

/* Allocate network pool */
NET_BUF_POOL_FIXED_DEFINE_ALIGN(airoc_pool, AIROC_WIFI_PACKET_POOL_COUNT,
				AIROC_WIFI_PACKET_POOL_SIZE, 0, NULL);

static uint16_t ap_event_handler_index = 0xFF;

/* Use global iface pointer to support any Ethernet driver */
/* necessary for wifi callback functions */
static struct net_if *airoc_wifi_iface;

struct airoc_wifi_event_t {
	uint8_t is_ap_event;
	uint32_t event_type;
};

whd_interface_t airoc_if;
whd_interface_t airoc_ap_if;

static const whd_event_num_t sta_link_events[] = {
	WLC_E_LINK,    WLC_E_DEAUTH_IND,       WLC_E_DISASSOC_IND,
	WLC_E_PSK_SUP, WLC_E_CSA_COMPLETE_IND, WLC_E_NONE};

static const whd_event_num_t ap_link_events[] = {WLC_E_DISASSOC_IND, WLC_E_DEAUTH_IND,
						 WLC_E_ASSOC_IND,    WLC_E_REASSOC_IND,
						 WLC_E_AUTHORIZED,   WLC_E_NONE};

static uint16_t sta_event_handler_index = 0xFF;
static void airoc_event_task(void);
static struct airoc_wifi_data airoc_wifi_data = {0};

static struct airoc_wifi_config airoc_wifi_config = {
	.sdhc_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.wifi_reg_on_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wifi_reg_on_gpios, {0}),
	.wifi_host_wake_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wifi_host_wake_gpios, {0}),
	.wifi_dev_wake_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wifi_dev_wake_gpios, {0}),
};

K_MSGQ_DEFINE(airoc_wifi_msgq, sizeof(whd_event_header_t), 10, 4);
K_THREAD_STACK_DEFINE(airoc_wifi_event_stack, CONFIG_AIROC_WIFI_EVENT_TASK_STACK_SIZE);

static struct k_thread airoc_wifi_event_thread;

whd_interface_t airoc_wifi_get_whd_interface(void)
{
	return airoc_if;
}

/*
 * Implement WHD network buffers functions
 */
whd_result_t cy_host_buffer_get(whd_buffer_t *buffer, whd_buffer_dir_t direction, uint16_t size,
				uint32_t timeout_ms)
{
	ARG_UNUSED(direction);
	ARG_UNUSED(timeout_ms);
	struct net_buf *buf;

	buf = net_buf_alloc_len(&airoc_pool, size, K_NO_WAIT);
	if (buf == NULL) {
		return WHD_BUFFER_ALLOC_FAIL;
	}
	*buffer = buf;
	return WHD_SUCCESS;
}

void cy_buffer_release(whd_buffer_t buffer, whd_buffer_dir_t direction)
{
	CY_UNUSED_PARAMETER(direction);
	(void)net_buf_destroy((struct net_buf *)buffer);
}

uint8_t *cy_buffer_get_current_piece_data_pointer(whd_buffer_t buffer)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf *buf = (struct net_buf *)buffer;

	return (uint8_t *)buf->data;
}

uint16_t cy_buffer_get_current_piece_size(whd_buffer_t buffer)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf *buf = (struct net_buf *)buffer;

	return (uint16_t)buf->size;
}

whd_result_t cy_buffer_set_size(whd_buffer_t buffer, unsigned short size)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf *buf = (struct net_buf *)buffer;

	buf->size = size;
	return CY_RSLT_SUCCESS;
}

whd_result_t cy_buffer_add_remove_at_front(whd_buffer_t *buffer, int32_t add_remove_amount)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf **buf = (struct net_buf **)buffer;

	if (add_remove_amount > 0) {
		(*buf)->len = (*buf)->size;
		(*buf)->data = net_buf_pull(*buf, add_remove_amount);
	} else {
		(*buf)->data = net_buf_push(*buf, -add_remove_amount);
		(*buf)->len = (*buf)->size;
	}
	return WHD_SUCCESS;
}

static int airoc_mgmt_send(const struct device *dev, struct net_pkt *pkt)
{
	struct airoc_wifi_data *data = dev->data;
	cy_rslt_t ret = CY_RSLT_SUCCESS;
	size_t pkt_len = net_pkt_get_len(pkt);
	struct net_buf *buf = NULL;

	/* Allocate Network Buffer from pool with Packet Length + Data Header */
	buf = net_buf_alloc_len(&airoc_pool, pkt_len + sizeof(data_header_t), K_NO_WAIT);
	if (buf == NULL) {
		return -EIO;
	}

	/* Reserve the buffer Headroom for WHD Data header */
	net_buf_reserve(buf, sizeof(data_header_t));

	/* Read the packet payload */
	if (net_pkt_read(pkt, buf->data, pkt_len) < 0) {
		LOG_ERR("net_pkt_read failed");
		return -EIO;
	}

	/* Call WHD API to send out the Packet */
	ret = whd_network_send_ethernet_data(airoc_if, (void *)buf);
	if (ret != CY_RSLT_SUCCESS) {
		LOG_ERR("whd_network_send_ethernet_data failed");
#if defined(CONFIG_NET_STATISTICS_WIFI)
		data->stats.errors.tx++;
#endif
		return -EIO;
	}

#if defined(CONFIG_NET_STATISTICS_WIFI)
	data->stats.bytes.sent += pkt_len;
	data->stats.pkts.tx++;
#endif

	return 0;
}

void cy_network_process_ethernet_data(whd_interface_t interface, whd_buffer_t buffer)
{
	struct net_pkt *pkt;
	uint8_t *data = whd_buffer_get_current_piece_data_pointer(interface->whd_driver, buffer);
	uint32_t len = whd_buffer_get_current_piece_size(interface->whd_driver, buffer);
	bool net_pkt_unref_flag = false;

	if ((interface->role == WHD_STA_ROLE) &&
	    (net_if_flag_is_set(airoc_wifi_iface, NET_IF_UP))) {
		if (airoc_wifi_iface == NULL) {
			LOG_ERR("network interface unavailable");
			cy_buffer_release(buffer, WHD_NETWORK_RX);
			return;
		}

		pkt = net_pkt_rx_alloc_with_buffer(airoc_wifi_iface, len, AF_UNSPEC, 0, K_NO_WAIT);

		if (pkt != NULL) {
			if (net_pkt_write(pkt, data, len) < 0) {
				LOG_ERR("Failed to write pkt");
				net_pkt_unref_flag = true;
			}

			if ((net_pkt_unref_flag) || (net_recv_data(airoc_wifi_iface, pkt) < 0)) {
				LOG_ERR("Failed to push received data");
				net_pkt_unref_flag = true;
			}
		} else {
			LOG_ERR("Failed to get net buffer");
		}
	}
	/* Release a packet buffer */
	cy_buffer_release(buffer, WHD_NETWORK_RX);

#if defined(CONFIG_NET_STATISTICS_WIFI)
	airoc_wifi_data.stats.bytes.received += len;
	airoc_wifi_data.stats.pkts.rx++;
#endif

	if (net_pkt_unref_flag) {
		net_pkt_unref(pkt);
#if defined(CONFIG_NET_STATISTICS_WIFI)
		airoc_wifi_data.stats.errors.rx++;
#endif
	}
}

static void *link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header,
				 const uint8_t *event_data, void *handler_user_data)
{
	ARG_UNUSED(ifp);
	ARG_UNUSED(event_data);
	ARG_UNUSED(handler_user_data);

	(void)k_msgq_put(&airoc_wifi_msgq, event_header, K_FOREVER);
	return NULL;
}

static void airoc_event_task(void)
{
	whd_event_header_t event_header;

	while (1) {
		(void)k_msgq_get(&airoc_wifi_msgq, &event_header, K_FOREVER);

		switch ((whd_event_num_t)event_header.event_type) {
		case WLC_E_LINK:
			break;

		case WLC_E_DEAUTH_IND:
		case WLC_E_DISASSOC_IND:
			net_if_dormant_on(airoc_wifi_iface);
			break;

		default:
			break;
		}
	}
}

static void airoc_mgmt_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct airoc_wifi_data *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	data->iface = iface;
	airoc_wifi_iface = iface;

	/* Read WLAN MAC Address */
	(void)whd_wifi_get_mac_address(airoc_if, &airoc_if->mac_addr);
	(void)memcpy(&data->mac_addr, &airoc_if->mac_addr, sizeof(airoc_if->mac_addr));

	/* Assign link local address. */
	(void)net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET);

	(void)ethernet_init(iface);

	/* Not currently connected to a network */
	(void)net_if_dormant_on(iface);

	/* L1 network layer (physical layer) is up */
	(void)net_if_carrier_on(data->iface);
}

whd_result_t cy_buffer_pool_init(void *tx_packet_pool, void *rx_packet_pool)
{
	CY_UNUSED_PARAMETER(tx_packet_pool);
	CY_UNUSED_PARAMETER(rx_packet_pool);

	return 0;
}

static void scan_cb_search(whd_scan_result_t **result_ptr, void *user_data,
			   whd_scan_status_t status)
{
	if (status == WHD_SCAN_ABORTED) {
		k_sem_give(&airoc_wifi_data.sema_scan);
		return;
	}
	if (status == WHD_SCAN_COMPLETED_SUCCESSFULLY) {
		k_sem_give(&airoc_wifi_data.sema_scan);
	} else if (status == WHD_SCAN_INCOMPLETE) {
		if (user_data != NULL) {
			if ((**result_ptr).SSID.length > 0) {
				if (strcmp((**result_ptr).SSID.value,
					   ((whd_scan_result_t *)user_data)->SSID.value) == 0) {
					memcpy(user_data, *result_ptr, sizeof(whd_scan_result_t));
				}
			}
		}
	}
}

static void parse_scan_result(whd_scan_result_t *p_whd_result, struct wifi_scan_result *p_zy_result)
{
	if (p_whd_result->SSID.length != 0) {
		p_zy_result->ssid_length = p_whd_result->SSID.length;
		sprintf(p_zy_result->ssid, "%s", p_whd_result->SSID.value);
		p_zy_result->channel = p_whd_result->channel;
		p_zy_result->security = (p_whd_result->security == 0) ? WIFI_SECURITY_TYPE_NONE
								      : WIFI_SECURITY_TYPE_PSK;
		p_zy_result->rssi = (int8_t)p_whd_result->signal_strength;
		p_zy_result->mac_length = 6;
		memcpy(p_zy_result->mac, &p_whd_result->BSSID, 6);
	}
}

static void scan_callback(whd_scan_result_t **result_ptr, void *user_data, whd_scan_status_t status)
{
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)user_data;
	whd_scan_result_t whd_scan_result;
	struct wifi_scan_result zephyr_scan_result;

	if (status == WHD_SCAN_COMPLETED_SUCCESSFULLY || status == WHD_SCAN_ABORTED) {
		data->scan_rslt_cb(data->iface, 0, NULL);

		return;
	}
	if ((result_ptr != NULL) || (*result_ptr != NULL)) {
		memcpy(&whd_scan_result, *result_ptr, sizeof(whd_scan_result_t));
		parse_scan_result(&whd_scan_result, &zephyr_scan_result);
		data->scan_rslt_cb(data->iface, 0, &zephyr_scan_result);
	}
	memset(*result_ptr, 0, sizeof(whd_scan_result_t));
}

static int airoc_mgmt_scan(const struct device *dev, struct wifi_scan_params *params,
			   scan_result_cb_t cb)
{
	int ret = 0;
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;

	data->scan_rslt_cb = cb;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	ret = whd_wifi_scan(airoc_if, params->scan_type, WHD_BSS_TYPE_ANY, &(data->ssid), NULL,
			    NULL, NULL, scan_callback, &(data->scan_result), data);

	k_sem_give(&data->sema_common);
	return ret;
}

static int airoc_mgmt_connect(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;
	whd_ssid_t ssid = {0};
	int ret = 0;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (data->is_sta_connected) {
		LOG_ERR("Already connected");
		ret = -EALREADY;
		goto error;
	}

	ssid.length = params->ssid_length;
	memcpy(ssid.value, params->ssid, params->ssid_length);

	whd_scan_result_t scan_result;
	whd_scan_result_t usr_result = {0};

	usr_result.SSID.length = ssid.length;
	memcpy(usr_result.SSID.value, ssid.value, ssid.length);

	if (whd_wifi_scan(airoc_if, WHD_SCAN_TYPE_ACTIVE, WHD_BSS_TYPE_ANY, NULL, NULL, NULL, NULL,
			  scan_cb_search, &scan_result, &(usr_result)) != WHD_SUCCESS) {
		LOG_ERR("Failed start scan");
		ret = -EAGAIN;
		goto error;
	}

	if (k_sem_take(&airoc_wifi_data.sema_scan, K_MSEC(12 * 1000)) != 0) {
		whd_wifi_stop_scan(airoc_if);
		ret = -EAGAIN;
		goto error;
	}

	if (usr_result.security == 0) {
		ret = -EAGAIN;
		goto error;
	}

	/* Connect to the network */
	if (whd_wifi_join(airoc_if, &usr_result.SSID, usr_result.security, params->psk,
			  params->psk_length) != WHD_SUCCESS) {
		LOG_ERR("Failed to connect with network");
		ret = -EAGAIN;
		goto error;
	}

error:
	if (ret < 0) {
		net_if_dormant_on(data->iface);
	} else {
		net_dhcpv4_start(data->iface);
		net_if_dormant_off(data->iface);
		data->is_sta_connected = 1;
	}

	wifi_mgmt_raise_connect_result_event(data->iface, ret);
	k_sem_give(&data->sema_common);
	return ret;
}

static int airoc_mgmt_disconnect(const struct device *dev)
{
	int ret = 0;
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (whd_wifi_leave(airoc_if) != WHD_SUCCESS) {
		k_sem_give(&data->sema_common);
		ret = -EAGAIN;
	} else {
		data->is_sta_connected = 0;
		net_dhcpv4_stop(data->iface);
		net_if_dormant_on(data->iface);
	}

	wifi_mgmt_raise_disconnect_result_event(data->iface, ret);
	k_sem_give(&data->sema_common);

	return ret;
}

static void *ap_link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header,
				    const uint8_t *event_data, void *handler_user_data)
{
	struct airoc_wifi_event_t airoc_event = {
		.is_ap_event = 1,
		.event_type = event_header->event_type
	};

	k_msgq_put(&airoc_wifi_msgq, &airoc_event, K_FOREVER);
	return NULL;
}

static int airoc_mgmt_ap_enable(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;
	whd_security_t security;
	whd_ssid_t ssid = {0};
	uint8_t channel = 0;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (data->second_interface_init == 0) {
		if (whd_add_secondary_interface(data->whd_drv, NULL, &airoc_ap_if) !=
		    CY_RSLT_SUCCESS) {
			LOG_ERR("Error Unable to bring up the whd secondary interface");
			k_sem_give(&data->sema_common);
			return -EAGAIN;
		}
		data->second_interface_init = 1;
	}

	if (data->is_ap_up) {
		LOG_ERR("Already AP is on - first disconnect");
		k_sem_give(&data->sema_common);
		return -EAGAIN;
	}

	ssid.length = params->ssid_length;
	memcpy((void *)ssid.value, (void *)params->ssid, ssid.length);

	if (((params->channel > 0) && (params->channel < 12)) ||
	    ((params->channel > 35) && (params->channel < 166))) {
		channel = params->channel;
	} else {
		channel = 1;
	}

	if (params->psk_length == 0) {
		security = WHD_SECURITY_OPEN;
	} else {
		security = WHD_SECURITY_WPA2_AES_PSK;
	}

	if (whd_wifi_init_ap(airoc_ap_if, &ssid, security, (const uint8_t *)params->psk,
			     params->psk_length, channel) != 0) {
		LOG_ERR("Failed to init whd ap interface");
		k_sem_give(&data->sema_common);
		return -EAGAIN;
	}

	if (whd_wifi_start_ap(airoc_ap_if) != 0) {
		LOG_ERR("Failed to start whd ap interface");
		k_sem_give(&data->sema_common);
		return -EAGAIN;
	}

	/* set event handler */
	if (whd_management_set_event_handler(airoc_ap_if, ap_link_events, ap_link_events_handler,
					     NULL, &ap_event_handler_index) != 0) {
		whd_wifi_stop_ap(airoc_ap_if);
		k_sem_give(&data->sema_common);
		return -EAGAIN;
	}

	data->is_ap_up = 1;

	k_sem_give(&data->sema_common);
	return 0;
}

#if defined(CONFIG_NET_STATISTICS_WIFI)
static int airoc_mgmt_wifi_stats(const struct device *dev, struct net_stats_wifi *stats)
{
	struct airoc_wifi_data *data = dev->data;

	stats->bytes.received = data->stats.bytes.received;
	stats->bytes.sent = data->stats.bytes.sent;
	stats->pkts.rx = data->stats.pkts.rx;
	stats->pkts.tx = data->stats.pkts.tx;
	stats->errors.rx = data->stats.errors.rx;
	stats->errors.tx = data->stats.errors.tx;
	stats->broadcast.rx = data->stats.broadcast.rx;
	stats->broadcast.tx = data->stats.broadcast.tx;
	stats->multicast.rx = data->stats.multicast.rx;
	stats->multicast.tx = data->stats.multicast.tx;
	stats->sta_mgmt.beacons_rx = data->stats.sta_mgmt.beacons_rx;
	stats->sta_mgmt.beacons_miss = data->stats.sta_mgmt.beacons_miss;

	return 0;
}
#endif

static int airoc_mgmt_ap_disable(const struct device *dev)
{
	int ret = 0;
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	whd_wifi_deregister_event_handler(airoc_ap_if, ap_event_handler_index);
	whd_wifi_stop_ap(airoc_ap_if);
	data->is_ap_up = 0;

	k_sem_give(&data->sema_common);
	return ret;
}

extern int airoc_wifi_init_primary(const struct device *dev, whd_interface_t *interface);

static int airoc_init(const struct device *dev)
{
	cy_rslt_t ret = CY_RSLT_SUCCESS;
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;

	k_tid_t tid = k_thread_create(
		&airoc_wifi_event_thread, airoc_wifi_event_stack,
		CONFIG_AIROC_WIFI_EVENT_TASK_STACK_SIZE, (k_thread_entry_t)airoc_event_task, NULL,
		NULL, NULL, CONFIG_AIROC_WIFI_EVENT_TASK_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	k_thread_name_set(tid, "airoc_event");

	ret = airoc_wifi_init_primary(dev, &airoc_if);
	if (ret != CY_RSLT_SUCCESS) {
		LOG_ERR("airoc_wifi_init_primary failed ret = %d \r\n", ret);
	}

	ret = whd_management_set_event_handler(airoc_if, sta_link_events, link_events_handler, NULL,
					       &sta_event_handler_index);
	if (ret != CY_RSLT_SUCCESS) {
		LOG_ERR("whd_management_set_event_handler failed ret = %d \r\n", ret);
	}

	k_sem_init(&data->sema_common, 1, 1);
	k_sem_init(&data->sema_scan, 0, 1);

	return ret;
}

static const struct wifi_mgmt_ops airoc_wifi_mgmt = {
	.scan = airoc_mgmt_scan,
	.connect = airoc_mgmt_connect,
	.disconnect = airoc_mgmt_disconnect,
	.ap_enable = airoc_mgmt_ap_enable,
	.ap_disable = airoc_mgmt_ap_disable,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = airoc_mgmt_wifi_stats,
#endif
};

static const struct net_wifi_mgmt_offload airoc_api = {
	.wifi_iface.iface_api.init = airoc_mgmt_init,
	.wifi_iface.send = airoc_mgmt_send,
	.wifi_mgmt_api = &airoc_wifi_mgmt,
};

NET_DEVICE_DT_INST_DEFINE(0, airoc_init, NULL, &airoc_wifi_data, &airoc_wifi_config,
			  CONFIG_WIFI_INIT_PRIORITY, &airoc_api, ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2), WHD_LINK_MTU);
