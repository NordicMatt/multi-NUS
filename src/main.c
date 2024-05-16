/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */
#include "uart_async_adapter.h"//WRC
#include <zephyr/usb/usb_device.h> //WRC
#include <errno.h> 
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h> 
#include <zephyr/sys/printk.h> 

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h> 

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <bluetooth/conn_ctx.h> 
#include <stdlib.h>
#include <stdio.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* UART payload buffer element size. */
#define UART_BUF_SIZE 20

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define NUS_WRITE_TIMEOUT K_MSEC(500)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work; 

K_SEM_DEFINE(nus_write_sem, 0, 1);

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};
//WRC
#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct bt_conn *default_conn;

BT_CONN_CTX_DEF(conns, CONFIG_BT_MAX_CONN, sizeof(struct bt_nus_client));

static bool routedMessage = false;
static bool messageStart = true;

#define ROUTED_MESSAGE_CHAR '*'
#define BROADCAST_INDEX 99

static void ble_data_sent(struct bt_nus_client *nus,uint8_t err, const uint8_t *const data, uint16_t len)
{

	k_sem_give(&nus_write_sem);

	if (err) {
		LOG_WRN("ATT error code: 0x%02X", err);
	}
}

/*	New function for sending data into the multi-NUS
* 	Extensions to the behavior of message routing can be made here.
*	If the first character is *, this indicates a routed message.
*	If the first character is not *, then this is a broadcast message sent to all peers.
* 	If the message is routed, the two characters after the * will be read as the peer number
*	and the message will be sent only to that peer. Numbers must be written as two digits, i.e 01 for 1.
*	The default behavior will be to broadcast in the case of failure of message parsing.
*/
static int multi_nus_send(struct uart_data_t *buf){
	
	int err = 0;
	char * message = buf->data;
	int length = buf->len;
	
	static bool broadcast = false;
	static int nus_index = 0;

	LOG_INF("Multi-Nus Send");

	/*How many connections are there in the Connection Context Library?*/
	const size_t num_nus_conns = bt_conn_ctx_count(&conns_ctx_lib);

	/*Handle the routing of the message only at the beginning of the message*/
	if (messageStart) {
		messageStart = false;

		/*Check if it's a routed message*/
		if (message[0] == ROUTED_MESSAGE_CHAR) {
			
			routedMessage = true;
			/*Determine who the intended recipient is*/
			char str[2];
			str[0] = message[1];
			str[1] = message[2];
			nus_index = atoi(str);

			/*Is this a number that makes sense?*/
			if ((nus_index >= 0) && (nus_index < num_nus_conns)){
				broadcast = false;

				/*Move the data buffer pointer to after the recipient info and 
				shorten the length*/
				message =  &message[3];
				length = length - 3;
			} else if (nus_index == BROADCAST_INDEX) {
				broadcast = true;
				message =  &message[3];
				length = length - 3;
			}
		} else {
			broadcast = true;
		}
	}



	/*	If it's a routed message, send it to that guy. 
	*	If it's not, broadcast it to everyone.
	*/
	if (broadcast == false){
		const struct bt_conn_ctx *ctx =
				bt_conn_ctx_get_by_id(&conns_ctx_lib, nus_index);
		LOG_INF("Trying to send to server %d", nus_index);

		
		if (ctx) {
			struct bt_nus_client *nus_client = ctx->data;

			if (nus_client) {
				err = bt_nus_client_send(nus_client,
								     message,
								     length);
				if (err) {
					LOG_WRN("Failed to send data over BLE connection"
						"(err %d)",
						err);
				}else{
					LOG_INF("Sent to server %d: %s", nus_index, buf->data);
				}
			
			
				err = k_sem_take(&nus_write_sem,
							NUS_WRITE_TIMEOUT);
				if (err) {
					LOG_WRN("NUS send timeout");
				}
			}
			bt_conn_ctx_release(&conns_ctx_lib,
							(void *)ctx->data);

		}

	}else{//Broadcast message
		LOG_INF("Broadcast");
		for (size_t i = 0; i < num_nus_conns; i++) {
			const struct bt_conn_ctx *ctx =
				bt_conn_ctx_get_by_id(&conns_ctx_lib, i);

			if (ctx) {
				struct bt_nus_client *nus_client = ctx->data;

				if (nus_client != NULL) {
					err = bt_nus_client_send(nus_client,
								     message,
								     length);
					if (err) {
						LOG_WRN("Failed to send data over BLE connection"
							"(err %d)",
							err);
					}else{
						LOG_INF("Sent to server %d: %s", nus_index, buf->data);
					}

					bt_conn_ctx_release(&conns_ctx_lib,
							    (void *)ctx->data);

					err = k_sem_take(&nus_write_sem,
							 NUS_WRITE_TIMEOUT);
					if (err) {
						LOG_WRN("NUS send timeout");
					}
				}
			}
		}
	}
	if ( (message[length-1] == '\n') || (message[length-1] == '\r') ) {
		messageStart = true;
		routedMessage = false;
	}

	return err;
}

/*	This function has been updated to add the ability for a peer to route a message by
*	appending a '*' as in the multi-NUS send function. So a peer could send the message
*	*00 to send a message to peer 0. If the peer sends a *99, that message is broadcast to 
*	all peers
*/

static uint8_t ble_data_received(struct bt_nus_client *nus,const uint8_t *const data, uint16_t len)
{
	int err;

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return BT_GATT_ITER_CONTINUE;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		/*	Routed messages. See the comments above. 
		*	Check for *, if there's a star, send it over to the multi-nus send function
		*/
		if (( data[0] == '*') || (routedMessage == true) ) {
			multi_nus_send(tx);
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static uint8_t *current_buf;
	static size_t aborted_len;
	static bool buf_release;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf; 

	switch (evt->type) {
	case UART_TX_DONE:
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf,
					   struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;
		buf_release = false;

		if (buf->len == UART_BUF_SIZE) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			  (evt->data.rx.buf[buf->len - 1] == '\r')) {
			k_fifo_put(&fifo_uart_rx_data, buf);
			current_buf = evt->data.rx.buf;
			buf_release = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_schedule(&uart_work,
					      UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_RX_TIMEOUT);

		break;

	case UART_RX_BUF_REQUEST:
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);
		if (buf_release && (current_buf != evt->data.rx_buf.buf)) {
			k_free(buf);
			buf_release = false;
			current_buf = NULL;
		}

		break;

	case UART_TX_ABORTED:
			if (!aborted_buf) {
				aborted_buf = (uint8_t *)evt->data.tx.buf;
			}

			aborted_len += evt->data.tx.len;
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);

			uart_tx(uart, &buf->data[aborted_len],
				buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_schedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT);
}

//WRC
static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	struct uart_data_t *rx;

	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}
	//WRC

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}


	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);
	//WRC
	
	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data),
			      UART_RX_TIMEOUT);
}

static void discovery_complete(struct bt_gatt_dm *dm,
			       void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("Service discovery completed");

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);

	bt_gatt_dm_data_release(dm);

	int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
	} else {
		LOG_INF("Scanning started");
	}

	/*	Send a message to the new NUS server informing it of its ID in this 
	*	mini-network
	*	The new NUS will have been added to the connection context library and so
	* 	will be the highest index as they are added incrementally upwards.
	*	This is a bit of a workaround because in this function, I don't know
	*	the ID of this connection which is the piece of info I want to transmit.
	*/
	size_t num_nus_conns = bt_conn_ctx_count(&conns_ctx_lib);
	size_t nus_index = 99;

	/*	This is a little inelegant but we must get the index of the device to
	* 	convey it
	*/
	for (size_t i = 0; i < num_nus_conns; i++) {
		const struct bt_conn_ctx *ctx =
			bt_conn_ctx_get_by_id(&conns_ctx_lib, i);

		if (ctx) {
			if (ctx->data == nus) {
				nus_index = i;
				char message[3];
				sprintf(message, "%d", nus_index);
				message[2] = '\r';
				int length = 3;

				err = bt_nus_client_send(nus, message, length);
				if (err) {
					LOG_WRN("Failed to send data over BLE connection"
						"(err %d)",
						err);
				} else {
					LOG_INF("Sent to server %d: %s",
						nus_index, message);
				}

				bt_conn_ctx_release(&conns_ctx_lib,
						    (void *)ctx->data);
				break;
			} else {
				bt_conn_ctx_release(&conns_ctx_lib,
						    (void *)ctx->data);
			}
		}
	}
}

static void discovery_service_not_found(struct bt_conn *conn,
					void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
			    int err,
			    void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	struct bt_nus_client *nus_client =
		bt_conn_ctx_get(&conns_ctx_lib, conn);

	if (!nus_client) {
		return;
	}

	err = bt_gatt_dm_start(conn,
			       BT_UUID_NUS_SERVICE,
			       &discovery_cb,
			       nus_client);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error "
			"code: %d", err);
	}

	bt_conn_ctx_release(&conns_ctx_lib, (void *) nus_client);

}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;
/*	struct bt_nus_client_init_param init = {
		.cb = {
			.received = ble_data_received,
			.sent = ble_data_sent,
		}
	};
*/
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr,conn_err);

		if (default_conn == conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			if (err) {
				LOG_ERR("Scanning failed to start (err %d)",
					err);
			}
		}

		return;
	}

	LOG_INF("Connected: %s", addr);

	/*Allocate memory for this connection using the connection context library. For reference,
	this code was taken from hids.c
	*/
	struct bt_nus_client *nus_client =bt_conn_ctx_alloc(&conns_ctx_lib, conn);

	if (!nus_client) {
		LOG_WRN("There is no free memory to "
			"allocate the connection context");
	}
	
	struct bt_nus_client_init_param init = {
		.cb = {
			.received = ble_data_received,
			.sent = ble_data_sent,
		}
	};

	memset(nus_client, 0, bt_conn_ctx_block_size_get(&conns_ctx_lib));

	err = bt_nus_client_init(nus_client, &init);

	bt_conn_ctx_release(&conns_ctx_lib, (void *)nus_client);
	
	if (err) {
		LOG_ERR("NUS Client initialization failed (err %d)", err);
	}else{
		LOG_INF("NUS Client module initialized");
	}

	gatt_discover(conn);

	/*Stop scanning during the discovery*/
	err = bt_scan_stop();
	if ((!err) && (err != -EALREADY)) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr,reason);

	err = bt_conn_ctx_free(&conns_ctx_lib, conn);

	if (err) {
		LOG_WRN("The memory was not allocated for the context of this "
			"connection.");
	}

	bt_conn_unref(conn);
	default_conn = NULL;

	// err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	// if (err) {
	// 	LOG_ERR("Scanning failed to start (err %d)",
	// 		err);
	// }
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr,level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,level, err);
	}

	gatt_discover(conn);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d",addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
}

// static int nus_client_init(void)
// {
// 	int err;
// 	struct bt_nus_client_init_param init = {
// 		.cb = {
// 			.received = ble_data_received,
// 			.sent = ble_data_sent,
// 		}
// 	};

// 	err = bt_nus_client_init(&nus_client, &init);
// 	if (err) {
// 		LOG_ERR("NUS Client initialization failed (err %d)", err);
// 		return err;
// 	}

// 	LOG_INF("NUS Client module initialized");
// 	return err;
// }

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	LOG_INF("Scan module initialized");
	return err;
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_auth_pairing_confirm(conn);

	LOG_INF("Pairing confirmed: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr,bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("Pairing failed conn: %s, reason %d", addr,	reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
	.pairing_confirm = pairing_confirm
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};



int main(void)
{
	int err;

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bt_conn_cb_register(&conn_callbacks);

	int (*module_init[])(void) = {uart_init, scan_init};//, nus_client_init};
	for (size_t i = 0; i < ARRAY_SIZE(module_init); i++) {
		err = (*module_init[i])();
		if (err) {
			return 0;
		}
	}

	printk("Starting Bluetooth Central UART example\n");


	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return 0;
	}

	LOG_INF("Scanning successfully started");

	for (;;) {
		/* Wait indefinitely for data to be sent over Bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		multi_nus_send(buf);					
		
	}
}