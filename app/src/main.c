/*
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led_strip.h>

LOG_MODULE_REGISTER(App);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* 1000 msec = 1 sec */
#define BLINK_SLEEP_TIME_MS 500
#define BLINK_STACK_SIZE 512
#define BLINK_PRIORITY 5

/* The devicetree node identifier for the "sw0" alias. */
#define SW0_NODE	DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button_cb_data;

/* The devicetree node identifier for the "led_strip" alias. */
#define STRIP_NODE		DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_SLEEP_TIME_MS 1000
#define RGB_LED_STACK_SIZE 512
#define RGB_LED_PRIORITY 5
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }


#define SHTC3_SENSOR_STACK_SIZE 512
#define SHTC3_SENSOR_PRIORITY 5
#define SHTC3_SLEEP_TIME_MS 1000


/* Thread structures */
struct k_thread blink_thread;
struct k_thread rgb_led_thread;
struct k_thread shtc3_sensor_thread;

K_THREAD_STACK_DEFINE(rgb_led_stack, RGB_LED_STACK_SIZE);
K_THREAD_STACK_DEFINE(blink_stack, BLINK_STACK_SIZE);
K_THREAD_STACK_DEFINE(shtc3_sensor_stack, SHTC3_SENSOR_STACK_SIZE);

void shtc3_sensor(void)
{
	const struct device *const sensor = DEVICE_DT_GET(DT_INST(0, sensirion_shtc3));
	if (!device_is_ready(sensor)) {
		LOG_ERR("SHTC3 sensor device %s is not ready", sensor->name);
		return;
	}

	while (1) {
		struct sensor_value temp;
		struct sensor_value hum;
		int rc = sensor_sample_fetch(sensor);
		if (rc) {
			LOG_ERR("Error %d: failed to fetch sample", rc);
			return;
		}

		rc = sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (rc) {
			LOG_ERR("Error %d: failed to get temperature", rc);
			return;
		}

		rc = sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &hum);
		if (rc) {
			LOG_ERR("Error %d: failed to get humidity", rc);
			return;
		}

		LOG_INF("Temperature: %d.%06d C", temp.val1, temp.val2);
		LOG_INF("Humidity: %d.%06d %%", hum.val1, hum.val2);

		k_msleep(SHTC3_SLEEP_TIME_MS);
	}
}

void rgb_led(void)
{
	size_t color = 0;
	int rc;
	struct led_rgb pixel;
	struct led_rgb colors[] = {
		RGB(0x0f, 0x00, 0x00), /* red */
		RGB(0x00, 0x0f, 0x00), /* green */
		RGB(0x00, 0x00, 0x0f), /* blue */
	};
	const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

	if (device_is_ready(strip)) {
		LOG_INF("Found LED strip device %s", strip->name);
	} else {
		LOG_ERR("LED strip device %s is not ready", strip->name);
		return;
	}

	while (1) {
		memcpy(&pixel, &colors[color], sizeof(struct led_rgb));
		rc = led_strip_update_rgb(strip, &pixel, STRIP_NUM_PIXELS);
		if (rc)
		{
			LOG_ERR("Error %d: failed to update LED strip", rc);
			return;
		}

		k_msleep(STRIP_SLEEP_TIME_MS);
		color = (color + 1) % ARRAY_SIZE(colors);
	}
}

void blink(void)
{
	int ret;
	bool led_state = true;

	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

	if (!gpio_is_ready_dt(&led))
	{
		LOG_ERR("LED device is not ready");
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR("LED configure error");
		return;
	}

	while (1)
	{
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
			LOG_ERR("LED toggle error");
			return;
		}

		led_state = !led_state;
		LOG_INF("LED state: %s", led_state ? "ON" : "OFF");
		k_msleep(BLINK_SLEEP_TIME_MS);
	}
}

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	/* Create blink thread */
	k_tid_t blink_thread_id = k_thread_create(&blink_thread, blink_stack,
											  K_THREAD_STACK_SIZEOF(blink_stack),
											  (k_thread_entry_t)blink, NULL, NULL, NULL,
											  BLINK_PRIORITY, 0, K_NO_WAIT);
	if (blink_thread_id == 0) {
		LOG_ERR("Error spawning blink thread");
		return -1;
	}

	LOG_INF("Blink thread created");

	/* Create RGB thread*/
	k_tid_t rgb_led_thread_id = k_thread_create(&rgb_led_thread, rgb_led_stack,
											  K_THREAD_STACK_SIZEOF(rgb_led_stack),
											  (k_thread_entry_t)rgb_led, NULL, NULL, NULL,
											  RGB_LED_PRIORITY, 0, K_NO_WAIT);
	if (rgb_led_thread_id == 0) {
		LOG_ERR("Error spawning RGB LED thread");
		return -1;
	}

	LOG_INF("RGB LED thread created");

	/* Create shtc3 sensor thread */
	k_tid_t shtc3_sensor_thread_id = k_thread_create(&shtc3_sensor_thread, shtc3_sensor_stack,
											  K_THREAD_STACK_SIZEOF(shtc3_sensor_stack),
											  (k_thread_entry_t)shtc3_sensor, NULL, NULL, NULL,
											  SHTC3_SENSOR_PRIORITY, 0, K_NO_WAIT);

	return 0;
}
