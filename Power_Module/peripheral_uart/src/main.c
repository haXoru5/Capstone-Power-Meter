/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

//FROM IMU Sample
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
 
 static struct sensor_trigger data_trigger;
 
 /* Flag set from IMU device irq handler */
 static volatile int irq_from_device;
 

//ADC init
/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(adc0)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)

/* Data of ADC device specified in devicetree. */
static const struct device *adc = DEVICE_DT_GET(ADC_NODE);

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))};

/* Data array of ADC channel voltage references. */
static uint32_t vrefs_mv[] = {DT_FOREACH_CHILD_SEP(ADC_NODE, CHANNEL_VREF, (,))};

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)
#define CONFIG_SEQUENCE_SAMPLES 32

/*Cal Data*/
#define CAL_SLOPE -0.35
#define CAL_OFFSET 827

/*RPM conversion constant*/
#define RADS_TO_RPM 9.5493

/*Battery range*/
#define BATTERY_MIN 3400
#define BATTERY_MAX 4200
#define DIVIDER_RATIO 5.5455

 /*
  * Get a device structure from a devicetree node from alias
  * "6dof_motion_drdy0".
  */
 static const struct device *get_6dof_motion_device(void)
 {
         const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(6dof_motion_drdy0));
 
         if (!device_is_ready(dev)) {
                 printk("\nError: Device \"%s\" is not ready; "
                        "check the driver initialization logs for errors.\n",
                        dev->name);
                 return NULL;
         }
 
         printk("Found device \"%s\", getting sensor data\n", dev->name);
         return dev;
 }
 
 static const char *now_str(void)
 {
         static char buf[16]; /* ...HH:MM:SS.MMM */
         uint32_t now = k_uptime_get_32();
         unsigned int ms = now % MSEC_PER_SEC;
         unsigned int s;
         unsigned int min;
         unsigned int h;
 
         now /= MSEC_PER_SEC;
         s = now % 60U;
         now /= 60U;
         min = now % 60U;
         now /= 60U;
         h = now;
 
         snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
         return buf;
 }
 
 static void handle_6dof_motion_drdy(const struct device *dev, const struct sensor_trigger *trig)
 {
         if (trig->type == SENSOR_TRIG_DATA_READY) {
                 int rc = sensor_sample_fetch_chan(dev, trig->chan);
 
                 if (rc < 0) {
                         printf("sample fetch failed: %d\n", rc);
                         printf("cancelling trigger due to failure: %d\n", rc);
                         (void)sensor_trigger_set(dev, trig, NULL);
                         return;
                 } else if (rc == 0) {
                         irq_from_device = 1;
                 }
         }
 }
 //END IMU Sample

//#define LOG_MODULE_NAME peripheral_uart
//LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

//USER DEFINED
#define MESSAGE "Hello from Peripheral!\n"

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		printk("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			printk("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		printk("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		printk("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			printk("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		printk("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			printk("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		printk("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		printk("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
				   data);

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
		printk("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}


static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			printk("Failed to enable USB");
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


	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		printk("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		printk("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		printk("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			printk("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			printk("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			printk("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		printk("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		printk("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);

	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u", addr, level);
	} else {
		printk("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u", addr, passkey);
}
/*
static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX)) {
		printk("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		printk("Press Button 1 to confirm, Button 2 to reject.");
	}
}
*/

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    printf("Auto-confirming passkey: %06u", passkey);
    bt_conn_auth_passkey_confirm(conn);
}



static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	printk("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			printk("Not able to allocate UART send data buffer");
			return;
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

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED

/*
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		printk("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		printk("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

*/

static void num_comp_reply(bool accept)
{
    bt_conn_auth_passkey_confirm(auth_conn);
    printf("Auto-confirming Numeric Match, conn %p", (void *)auth_conn);

    bt_conn_unref(auth_conn);
    auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)", err);
		printf("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		printk("Cannot init LEDs (err: %d)", err);
		printf("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = uart_init();
	if (err) {
		error();
		printf("Failed to init UART");
	}

	//ADC init
	//IMU init
	const struct device *dev = get_6dof_motion_device();
	//struct sensor_value accel[3];
	struct sensor_value gyro[3];
	//struct sensor_value temperature;
	uint32_t counter = 0;

	if (dev == NULL) {
			return 0;
	}

	data_trigger = (struct sensor_trigger){
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_6dof_motion_drdy) < 0) {
			printf("Cannot configure data trigger!!!\n");
			return 0;
	}

   //ADC init
	//uint32_t count = 0;
	uint16_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];
	uint16_t avg_val_chans[CHANNEL_COUNT];
	double gyro_y = 0.0;
	double torque;
	double power;
	double rpm;
	double battery_mv;
	double battery_percentage;

	/* Options for the sequence sampling. */
	const struct adc_sequence_options options = {
			.extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1,
			.interval_us = 0,
	};

	/* Configure the sampling sequence to be made. */
	struct adc_sequence sequence = {
			.buffer = channel_reading,
			/* buffer size in bytes, not number of samples */
			.buffer_size = sizeof(channel_reading),
			.resolution = CONFIG_SEQUENCE_RESOLUTION,
			.options = &options,
	};

	if (!device_is_ready(adc)) {
			printf("ADC controller device %s not ready\n", adc->name);
			return 0;
	}

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < CHANNEL_COUNT; i++) {
			sequence.channels |= BIT(channel_cfgs[i].channel_id);
			err = adc_channel_setup(adc, &channel_cfgs[i]);
			if (err < 0) {
					printf("Could not setup channel #%d (%d)\n", i, err);
					return 0;
			}
			if (channel_cfgs[i].reference == ADC_REF_INTERNAL) {
					vrefs_mv[i] = adc_ref_internal(adc);
			}
	}

	//k_sleep(K_MSEC(1000));
	//end ADC init



	//BLE init
	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	printk("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		printk("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)", err);
		printf("init error\n");
		return 0;
	}

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
		
		counter ++;
                uint32_t now = k_uptime_get_32();
                static uint32_t last_time = 0;
                 /*if (irq_from_device) {
                         //sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
                         sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
                         sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
 
                         LOG_INF("[%s]: temp %.2f Cel "
                                "  accel %f %f %f m/s/s "
                                "  gyro  %f %f %f rad/s\n",
                                now_str(), sensor_value_to_double(&temperature),
                                sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
                                sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
                                sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
                           
                                irq_from_device = 0;
                 }
                */
                 uint32_t elapsed = now - last_time;
                if (elapsed >= 1000) {
                        last_time = now;
                        //printf("Time elapsed: %u ms\n", elapsed);
                        //printf("Time elapsed: %s\n", now_str());
                        if(irq_from_device) {
                                //sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
                                sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
                                gyro_y = sensor_value_to_double(&gyro[1]);
                                irq_from_device = 0;
                        }

                        err = adc_read(adc, &sequence);
                        if (err < 0) {
                                printf("Could not read (%d)\n", err);
                                continue;
                        }

                        for (size_t channel_index = 0U; channel_index < CHANNEL_COUNT; channel_index++) {
                                int32_t val_mv;
                                int32_t avg_val_mv = 0;
                                /*printf("- %s, channel %" PRId32 ", %" PRId32 " sequence samples:\n",
                                       adc->name, channel_cfgs[channel_index].channel_id,
                                       CONFIG_SEQUENCE_SAMPLES);
                                */
                                for (size_t sample_index = 0U; sample_index < CONFIG_SEQUENCE_SAMPLES;
                                     sample_index++) {
        
                                        val_mv = channel_reading[sample_index][channel_index];
                                        //printf("- - %" PRId32, val_mv);
                                        err = adc_raw_to_millivolts(vrefs_mv[channel_index],
                                                                    channel_cfgs[channel_index].gain,
                                                                    CONFIG_SEQUENCE_RESOLUTION, &val_mv);
                                        
                                        /* conversion to mV may not be supported, skip if not */
                                        if ((err < 0) || vrefs_mv[channel_index] == 0) {
                                                printf(" (value in mV not available)\n");
                                        } else {
                                                avg_val_mv += val_mv;
                                                
                                                //printf(" = %" PRId32 "mV\n", val_mv);
                                        }
                                }
                                avg_val_mv /= CONFIG_SEQUENCE_SAMPLES;
                                avg_val_chans[channel_index] = avg_val_mv;
                                //printf("Average value: %" PRId32 "mV\n", avg_val_mv);
                        }
                        torque = avg_val_chans[0] * CAL_SLOPE + CAL_OFFSET;
                        power = abs(torque) * gyro_y;
                        rpm = gyro_y * RADS_TO_RPM;

                        /*Battery percentage calculation*/
                        battery_mv = avg_val_chans[1] * DIVIDER_RATIO;
                        battery_percentage = (battery_mv - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100;
                        if (battery_percentage < 0) {
                                battery_percentage = 0;
                        } else if (battery_percentage > 100) {
                                battery_percentage = 100;
                        }

						char output[32];
                        snprintf(output, sizeof(output), "%.1f \n%.1f\n%.1f\n%.1f\n",torque, power, rpm, battery_mv);
						bt_nus_send(NULL, output, sizeof(output));
                        LOG_INF("%s", output);
                        //LOG_INF("[%s]\nTorque: %.1f mV\n Power: %.1f W\n Cadence %.1f RPM\n Battery: %.1f mV\n", now_str(), torque, power, rpm, battery_mv);
                        //printf("Time: %s\n", now_str());
                        //LOG_INF("Temp: %.2f Cel",sensor_value_to_double(&temperature));

                }
		
	}
	
	
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0) {
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;

			if (nus_data.len >= sizeof(nus_data.data) ||
			   (nus_data.data[nus_data.len - 1] == '\n') ||
			   (nus_data.data[nus_data.len - 1] == '\r')) {
				if (bt_nus_send(NULL, nus_data.data, nus_data.len)) {
					printk("Failed to send data over BLE connection");
				}
				nus_data.len = 0;
			}

			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
