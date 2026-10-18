/*
 * Copyright (c) 2026 Neoventor AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "../test_work.h"
#include <udc_common.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/net_buf.h>
#include <zephyr/usb/usbd.h>

static bool fail_enqueue;
static bool refuse_always;
static bool resume_on_refusal;
static uint32_t enqueue_failures;
static uint32_t out_armed;

static bool fail_alloc;
static uint32_t alloc_failures;

/* Hold only test-side allocation, after the driver's enabled check. */
static K_SEM_DEFINE(notification_entered, 0, 1);
static K_SEM_DEFINE(notification_release, 0, 1);
static K_SEM_DEFINE(notification_queued, 0, 1);
static bool notification_test;
static bool notification_hold;
static bool notification_accept;
static bool notification_detached;
static uint32_t notification_allocations;
static uint8_t notification_ep;

static struct net_buf *notification_alloc(const struct usbd_class_data *c_data,
					 uint8_t ep, size_t size)
{
	if (notification_test && USB_EP_DIR_IS_IN(ep)) {
		notification_ep = ep;
		notification_allocations++;
		if (notification_hold) {
			k_sem_give(&notification_entered);
			(void)k_sem_take(&notification_release, K_MSEC(250));
		}
		if (c_data->uds_ctx == NULL) {
			/* Observe the bug without dereferencing the detached context. */
			notification_detached = true;
			return NULL;
		}
	}
	return usbd_ep_buf_alloc(c_data, ep, size);
}

static struct net_buf *alloc_seam(const struct usbd_class_data *c_data,
				uint8_t ep, size_t size)
{
	if (USB_EP_DIR_IS_OUT(ep) && fail_alloc) {
		fail_alloc = false;
		alloc_failures++;
		return NULL;
	}
	return notification_alloc(c_data, ep, size);
}

static int enqueue_seam(const struct usbd_class_data *c_data, struct net_buf *buf)
{
	bool out = USB_EP_DIR_IS_OUT(udc_get_buf_info(buf)->ep);
	int ret;

	if (out && (fail_enqueue || refuse_always)) {
		fail_enqueue = false;
		enqueue_failures++;
		if (resume_on_refusal) {
			resume_on_refusal = false;
			c_data->api->resumed((struct usbd_class_data *)c_data);
		}
		return -EIO;
	}

	if (notification_test && udc_get_buf_info(buf)->ep == notification_ep) {
		k_sem_give(&notification_queued);
		if (notification_accept) {
			/* The test controller owns it; completion is released separately. */
			net_buf_unref(buf);
			return 0;
		}
		return -EIO;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (out && ret == 0) {
		out_armed++;
	}
	return ret;
}

/* Compile the complete driver once, in its usual library. */
#define usbd_ep_buf_alloc alloc_seam
#define usbd_ep_enqueue enqueue_seam
#define k_work_submit_to_queue test_submit_to_queue
#define k_work_schedule_for_queue test_schedule_for_queue
#define k_work_reschedule_for_queue test_reschedule_for_queue
#define k_work_cancel test_cancel
#define k_work_cancel_sync test_cancel_sync
#define k_work_cancel_delayable_sync test_cancel_delayable_sync
#include <usbd_cdc_acm.c>
#undef k_work_cancel_delayable_sync
#undef k_work_cancel_sync
#undef k_work_cancel
#undef k_work_reschedule_for_queue
#undef k_work_schedule_for_queue
#undef k_work_submit_to_queue
#undef usbd_ep_enqueue
#undef usbd_ep_buf_alloc

USBD_CONFIGURATION_DEFINE(rx_fs, 0, 50, NULL);
USBD_CONFIGURATION_DEFINE(rx_hs, 0, 50, NULL);
USBD_DEVICE_DEFINE(rx_device, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x2fe3, 0xffff);

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static struct usbd_class_data *rx_class(void)
{
	const struct cdc_acm_uart_config *cfg = uart->config;

	return cfg->c_data;
}

static struct k_work *rx_work(void)
{
	struct cdc_acm_uart_data *data = uart->data;

#if RX_WORK_DELAYABLE
	return &data->rx_fifo_work.work;
#else
	return &data->rx_fifo_work;
#endif
}

static struct net_buf *take_out(void)
{
	struct udc_ep_config *cfg;

	cfg = udc_get_ep_cfg(rx_device.dev, cdc_acm_get_bulk_out(rx_class()));
	zassert_not_null(cfg);
	return udc_buf_get(cfg);
}

static void release_endpoint(uint8_t ep)
{
	struct udc_ep_config *cfg = udc_get_ep_cfg(rx_device.dev, ep);
	struct net_buf *buf;

	zassert_not_null(cfg);
	while ((buf = udc_buf_get(cfg)) != NULL) {
		zassert_ok(usbd_cdc_acm_request(rx_class(), buf, -ECONNABORTED));
	}
}

static void cleanup(void)
{
	struct cdc_acm_uart_data *data = uart->data;

	usbd_cdc_acm_disable(rx_class());
	test_work_reset();
	cdc_acm_irq_callback_set(uart, NULL, NULL);
	cdc_acm_irq_rx_disable(uart);
	cdc_acm_irq_tx_disable(uart);
	release_endpoint(cdc_acm_get_bulk_out(rx_class()));
	release_endpoint(cdc_acm_get_bulk_in(rx_class()));
	ring_buf_reset(data->rx_fifo.rb);
	/* Only fixture teardown resets ownership after controller cleanup. */
	atomic_clear_bit(&data->state, CDC_ACM_RX_FIFO_BUSY);
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	cleanup();
	fail_enqueue = false;
	refuse_always = false;
	resume_on_refusal = false;
	enqueue_failures = 0;
	out_armed = 0;
	fail_alloc = false;
	alloc_failures = 0;
	zassert_ok(usbd_cdc_acm_init(rx_class()));
	usbd_cdc_acm_enable(rx_class());
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);
	cleanup();
}

static void *setup(void)
{
	zassert_true(device_is_ready(uart));
	zassert_ok(usbd_add_configuration(&rx_device, USBD_SPEED_FS, &rx_fs));
	zassert_ok(usbd_add_configuration(&rx_device, USBD_SPEED_HS, &rx_hs));
	zassert_ok(usbd_register_class(&rx_device, "cdc_acm_0", USBD_SPEED_FS, 1));
	zassert_ok(usbd_register_class(&rx_device, "cdc_acm_0", USBD_SPEED_HS, 1));
	zassert_ok(usbd_init(&rx_device));
	zassert_ok(usbd_enable(&rx_device));
	zassert_ok(udc_ep_enable(rx_device.dev, cdc_acm_get_bulk_out(rx_class()),
				 USB_EP_TYPE_BULK, cdc_acm_get_bulk_mps(rx_class()), 0));
	zassert_ok(udc_ep_enable(rx_device.dev, cdc_acm_get_bulk_in(rx_class()),
				 USB_EP_TYPE_BULK, cdc_acm_get_bulk_mps(rx_class()), 0));
	return NULL;
}

ZTEST(cdc_rx, test_retry_backoff_and_success_reset)
{
	static const uint32_t delays[] = {1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U, 128U};
	struct net_buf *buf;
	unsigned char byte;
	uint32_t failures;

	refuse_always = true;
	test_advance(K_NO_WAIT);
	for (size_t i = 0; i < ARRAY_SIZE(delays); i++) {
		zassert_equal(enqueue_failures, i + 1U);
		zassert_equal(test_work_delay(rx_work()), K_MSEC(delays[i]).ticks,
			      "retry delay did not follow capped backoff");
		if (i + 1U < ARRAY_SIZE(delays)) {
			test_advance(K_MSEC(delays[i]));
		}
	}

	refuse_always = false;
	test_advance(K_MSEC(128));
	zassert_true(out_armed > 0, "retry did not recover");
	buf = take_out();
	zassert_not_null(buf);
	net_buf_add_u8(buf, 0x5a);
	zassert_ok(usbd_cdc_acm_request(rx_class(), buf, 0));
	refuse_always = true;
	zassert_ok(cdc_acm_poll_in(uart, &byte));
	test_advance(K_NO_WAIT);
	zassert_equal(test_work_delay(rx_work()), K_MSEC(1).ticks,
		      "successful enqueue did not reset backoff");
	usbd_cdc_acm_resumed(rx_class());
	zassert_equal(test_work_delay(rx_work()), 0, "resume waited for backoff");
	/* A resume can arrive before the controller returns its refusal. */
	resume_on_refusal = true;
	failures = enqueue_failures;
	test_advance(K_NO_WAIT);
	zassert_equal(enqueue_failures, failures + 2U,
		      "refusal replaced an immediate lifecycle event with backoff");
}

/* A bounded stand-in for usbd_wakeup_request() taking the same mutex. */
static K_SEM_DEFINE(device_locked, 0, 1);
static K_SEM_DEFINE(callback_entered, 0, 1);
static K_SEM_DEFINE(shutdown_done, 0, 1);
static K_SEM_DEFINE(release_device_lock, 0, 1);
static K_THREAD_STACK_DEFINE(shutdown_stack, 2048);
static struct k_thread shutdown_thread;
static uint32_t blocked_callbacks;

static void blocking_callback(const struct device *dev, void *user_data)
{
	struct usbd_context *ctx = user_data;
	uint8_t byte = 0x5a;
	int ret;

	blocked_callbacks++;
	k_sem_give(&callback_entered);
	/* A timeout also frees the red control's synchronous canceller. */
	ret = k_mutex_lock(&ctx->mutex, K_MSEC(250));
	if (ret == 0) {
		k_mutex_unlock(&ctx->mutex);
	}
	uart_irq_tx_enable(dev);
	uart_irq_rx_enable(dev);
	(void)uart_fifo_fill(dev, &byte, 1);
	uart_poll_out(dev, byte);
}

static void run_shutdown(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	(void)k_mutex_lock(&rx_device.mutex, K_FOREVER);
	k_sem_give(&device_locked);
	if (k_sem_take(&callback_entered, K_SECONDS(1)) == 0) {
		rx_class()->api->shutdown(rx_class());
		k_sem_give(&shutdown_done);
	}
	(void)k_sem_take(&release_device_lock, K_SECONDS(1));
	k_mutex_unlock(&rx_device.mutex);
}

ZTEST(cdc_rx, test_shutdown_does_not_wait_for_application)
{
	struct cdc_acm_uart_data *data = uart->data;
	struct usbd_context *ctx = rx_class()->uds_ctx;
	struct k_work_sync sync;
	uint32_t runs;
	int ret;
	int shutdown_join;

	zassert_not_null(rx_class()->api->shutdown, "class shutdown is not registered");
	refuse_always = true;
	test_advance(K_NO_WAIT);
	blocked_callbacks = 0U;
	k_sem_reset(&device_locked);
	k_sem_reset(&callback_entered);
	k_sem_reset(&shutdown_done);
	k_sem_reset(&release_device_lock);
	cdc_acm_irq_callback_set(uart, blocking_callback, ctx);
	/* Run the callback on the dedicated queue, leaving UHC events free. */
	test_real_work = &data->irq_cb_work;
	k_thread_create(&shutdown_thread, shutdown_stack,
			K_THREAD_STACK_SIZEOF(shutdown_stack), run_shutdown,
			NULL, NULL, NULL, 1, 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&device_locked, K_SECONDS(1)));
	cdc_acm_irq_tx_enable(uart);
	ret = k_sem_take(&shutdown_done, K_MSEC(50));
	if (ret == 0) {
		/* Detach only in green; the callback still waits for this lock. */
		rx_class()->uds_ctx = NULL;
	}
	k_sem_give(&release_device_lock);
	shutdown_join = k_thread_join(&shutdown_thread, K_SECONDS(1));
	(void)k_work_flush(&data->irq_cb_work, &sync);
	test_real_work = NULL;
	rx_class()->uds_ctx = ctx;
	zassert_ok(shutdown_join);
	zassert_ok(ret, "shutdown waited for application code under the device lock");
	zassert_equal(blocked_callbacks, 1U);
	zassert_false(test_work_pending(&data->irq_cb_work));
	zassert_false(test_work_pending(&data->tx_fifo_work.work));
	zassert_false(test_work_pending(rx_work()));
	runs = test_work_runs;
	rx_class()->uds_ctx = NULL;
	/* Late producers and already dequeued handlers must also refuse. */
	usbd_cdc_acm_enable(rx_class());
	usbd_cdc_acm_resumed(rx_class());
	cdc_acm_irq_tx_enable(uart);
	cdc_acm_irq_rx_enable(uart);
	cdc_acm_tx_fifo_handler(&data->tx_fifo_work.work);
	cdc_acm_rx_fifo_handler(rx_work());
	cdc_acm_irq_cb_handler(&data->irq_cb_work);
	test_advance(K_SECONDS(1));
	rx_class()->uds_ctx = ctx;
	zassert_equal(test_work_runs, runs, "class work ran after shutdown");
	zassert_equal(blocked_callbacks, 1U, "callback restarted after shutdown");
}

ZTEST(cdc_rx, test_shutdown_cancels_pending_retry)
{
	struct usbd_class_data *c_data = rx_class();
	struct usbd_context *ctx = c_data->uds_ctx;
	struct cdc_acm_uart_data *data = uart->data;
	uint32_t runs;

	refuse_always = true;
	test_advance(K_NO_WAIT);
	zassert_equal(enqueue_failures, 1U);
	zassert_true(test_work_pending(rx_work()));
	if (c_data->api->shutdown != NULL) {
		c_data->api->shutdown(c_data);
	}
	/* Assert before detaching: a missing shutdown handler must not fault. */
	zassert_false(test_work_pending(rx_work()), "shutdown left RX retry pending");
	zassert_false(test_work_pending(&data->tx_fifo_work.work));
	zassert_false(test_work_pending(&data->irq_cb_work));
	runs = test_work_runs;
	c_data->uds_ctx = NULL;
	test_advance(K_SECONDS(1));
	c_data->uds_ctx = ctx;
	zassert_equal(test_work_runs, runs, "work ran after shutdown returned");
	zassert_equal(enqueue_failures, 1U);
}

static void interface_reenable(bool completion_first)
{
	const struct cdc_acm_uart_config *cfg = uart->config;
	struct net_buf *cancelled[16];
	size_t count = 0U;
	uint32_t armed_before;

	test_advance(K_NO_WAIT);
	zassert_false(test_work_pending(rx_work()));
	/* Hold cancelled buffers until before or after the update callback. */
	while (count < ARRAY_SIZE(cancelled)) {
		cancelled[count] = take_out();
		if (cancelled[count] == NULL) {
			break;
		}
		count++;
	}
	zassert_true(count > 0U && count < ARRAY_SIZE(cancelled));
	armed_before = out_armed;
	if (!completion_first) {
		usbd_cdc_acm_update(rx_class(), cfg->desc->if1.bInterfaceNumber, 0);
		test_advance(K_NO_WAIT);
	}
	for (size_t i = 0; i < count; i++) {
		zassert_ok(usbd_cdc_acm_request(rx_class(), cancelled[i], -ECONNABORTED));
	}
	if (completion_first) {
		/* Run before update too: a refused enqueue may already be pending. */
		test_advance(K_NO_WAIT);
		usbd_cdc_acm_update(rx_class(), cfg->desc->if1.bInterfaceNumber, 0);
	}
	test_advance(K_NO_WAIT);
	zassert_true(out_armed > armed_before, "interface re-enable left RX unarmed");
}

ZTEST(cdc_rx, test_interface_update_before_cancel_completion)
{
	interface_reenable(false);
}

ZTEST(cdc_rx, test_interface_update_after_cancel_completion)
{
	interface_reenable(true);
}

ZTEST(cdc_rx, test_interface_update_skips_disabled_endpoint_backoff)
{
	const struct cdc_acm_uart_config *cfg = uart->config;
	struct net_buf *cancelled[16];
	size_t count = 0U;
	uint32_t armed_before;
	int64_t update_ticks;
	uint8_t ep = cdc_acm_get_bulk_out(rx_class());

	test_advance(K_NO_WAIT);
	zassert_false(test_work_pending(rx_work()));
	while (count < ARRAY_SIZE(cancelled)) {
		cancelled[count] = take_out();
		if (cancelled[count] == NULL) {
			break;
		}
		count++;
	}
	zassert_true(count > 0U && count < ARRAY_SIZE(cancelled));
	armed_before = out_armed;
	/* Return every held cancellation while the endpoint is still off. */
	zassert_ok(udc_ep_disable(rx_device.dev, ep));
	for (size_t i = 0; i < count; i++) {
		zassert_ok(usbd_cdc_acm_request(rx_class(), cancelled[i], -ECONNABORTED));
	}
	test_advance(K_NO_WAIT);
	zassert_equal(out_armed, armed_before, "disabled endpoint accepted RX");
	zassert_ok(udc_ep_enable(rx_device.dev, ep, USB_EP_TYPE_BULK,
				 cdc_acm_get_bulk_mps(rx_class()), 0));
	update_ticks = test_ticks;
	usbd_cdc_acm_update(rx_class(), cfg->desc->if1.bInterfaceNumber, 0);
	/* Run immediate work only: the refused enqueue's retry is still later. */
	test_advance(K_NO_WAIT);
	zassert_equal(test_ticks, update_ticks, "interface update advanced retry time");
	zassert_true(out_armed > armed_before, "interface update waited for RX backoff");
}

ZTEST(cdc_rx, test_enqueue_failure_rearms)
{
	fail_enqueue = true;
	test_advance(K_NO_WAIT);
	zassert_equal(enqueue_failures, 1, "enqueue seam was not reached");
	zassert_equal(out_armed, 0);
	test_advance(K_MSEC(1));
	zassert_true(out_armed > 0, "failed enqueue left RX unarmed");
}

ZTEST(cdc_rx, test_resume_rearms)
{
	/* Consume the initial work while suspended: there is no pending retry. */
	usbd_cdc_acm_suspended(rx_class());
	test_advance(K_NO_WAIT);
	zassert_equal(out_armed, 0);
	zassert_false(test_work_pending(rx_work()));
	usbd_cdc_acm_resumed(rx_class());
	test_advance(K_NO_WAIT);
	zassert_true(out_armed > 0, "resume did not rearm RX");
}

ZTEST(cdc_rx, test_retry_consumed_during_suspend)
{
	fail_enqueue = true;
	test_advance(K_NO_WAIT);
	zassert_equal(enqueue_failures, 1);
	usbd_cdc_acm_suspended(rx_class());
	test_advance(K_MSEC(1));
	zassert_equal(out_armed, 0, "RX enqueued while suspended");
	zassert_false(test_work_pending(rx_work()));
	usbd_cdc_acm_resumed(rx_class());
	test_advance(K_NO_WAIT);
	zassert_true(out_armed > 0, "suspended retry lost recovery on resume");
}

ZTEST(cdc_rx, test_disable_stops_retry)
{
	fail_enqueue = true;
	test_advance(K_NO_WAIT);
	zassert_equal(enqueue_failures, 1);
	usbd_cdc_acm_disable(rx_class());
	test_advance(K_MSEC(1));
	zassert_equal(out_armed, 0);
	zassert_false(test_work_pending(rx_work()));
	usbd_cdc_acm_resumed(rx_class());
	test_advance(K_NO_WAIT);
	zassert_equal(out_armed, 0, "resume armed a disabled class");
}

ZTEST(cdc_rx, test_no_fault_arms)
{
	test_advance(K_NO_WAIT);
	zassert_true(out_armed > 0);
	zassert_equal(enqueue_failures, 0);
}


ZTEST(cdc_rx, test_alloc_failure_rearms)
{
	fail_alloc = true;
	test_advance(K_NO_WAIT);
	zassert_equal(alloc_failures, 1, "allocator seam was not reached");
	zassert_equal(out_armed, 0);
	test_advance(K_MSEC(1));
	zassert_true(out_armed > 0, "allocation failure left RX busy or unarmed");
}

static void fill_rx_ring(void)
{
	struct net_buf *buf;
	uint8_t bytes[64] = {0};

	test_advance(K_NO_WAIT);
	/* Fill the ring through real production OUT completions. */
	for (size_t step = 0; step < 256; step++) {
		buf = take_out();
		if (buf == NULL) {
			zassert_false(test_work_pending(rx_work()),
				      "normal ring pressure must not poll");
			return;
		}
		net_buf_add_mem(buf, bytes, sizeof(bytes));
		zassert_ok(usbd_cdc_acm_request(rx_class(), buf, 0));
		test_advance(K_NO_WAIT);
	}
	zassert_unreachable("receive ring never filled");
}

ZTEST(cdc_rx, test_poll_drain_rearms_without_polling)
{
	uint32_t armed_before;
	unsigned char byte;

	fill_rx_ring();
	armed_before = out_armed;
	test_advance(K_MSEC(10));
	zassert_equal(out_armed, armed_before);
	for (size_t i = 0; i < cdc_acm_get_bulk_mps(rx_class()); i++) {
		zassert_ok(cdc_acm_poll_in(uart, &byte));
	}
	test_advance(K_NO_WAIT);
	zassert_true(out_armed > armed_before, "poll drain did not rearm RX");
}

static void drain_callback(const struct device *dev, void *user_data)
{
	uint8_t bytes[64];

	ARG_UNUSED(user_data);
	cdc_acm_irq_update(dev);
	if (cdc_acm_irq_rx_ready(dev) != 0) {
		zassert_equal(cdc_acm_fifo_read(dev, bytes, sizeof(bytes)), sizeof(bytes));
	}
}

ZTEST(cdc_rx, test_irq_drain_rearms_without_polling)
{
	uint32_t armed_before;

	fill_rx_ring();
	armed_before = out_armed;
	cdc_acm_irq_callback_set(uart, drain_callback, NULL);
	cdc_acm_irq_rx_enable(uart);
	test_advance(K_NO_WAIT);
	zassert_true(out_armed > armed_before, "IRQ drain did not rearm RX");
}


static K_SEM_DEFINE(notification_done, 0, 1);
static K_SEM_DEFINE(notification_shutdown_started, 0, 1);
static K_SEM_DEFINE(notification_shutdown_done, 0, 1);
static K_THREAD_STACK_DEFINE(notification_stack, 2048);
static K_THREAD_STACK_DEFINE(notification_shutdown_stack, 2048);
static struct k_thread notification_thread;
static struct k_thread notification_shutdown_thread;
static int notification_result;

static void run_notification(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	notification_result = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1U);
	k_sem_give(&notification_done);
}

static void shutdown_notification(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	(void)k_mutex_lock(&rx_device.mutex, K_FOREVER);
	k_sem_give(&notification_shutdown_started);
	if (rx_class()->api->shutdown != NULL) {
		rx_class()->api->shutdown(rx_class());
	}
	/* Match class removal: detach as soon as shutdown returns. */
	rx_class()->uds_ctx = NULL;
	k_sem_give(&notification_shutdown_done);
	k_mutex_unlock(&rx_device.mutex);
}

static void notification_shutdown_case(bool hold_allocation)
{
	struct usbd_context *ctx = rx_class()->uds_ctx;
	struct cdc_acm_uart_data *data = uart->data;
	uint32_t allocations;
	int shutdown_ret;
	int notify_join;
	int shutdown_join;
	int disabled_ret;

	notification_test = true;
	notification_hold = hold_allocation;
	notification_accept = !hold_allocation;
	notification_detached = false;
	notification_allocations = 0U;
	k_sem_reset(&notification_entered);
	k_sem_reset(&notification_release);
	k_sem_reset(&notification_queued);
	k_sem_reset(&notification_done);
	k_sem_reset(&notification_shutdown_started);
	k_sem_reset(&notification_shutdown_done);
	k_sem_reset(&data->notif_sem);
	k_thread_create(&notification_thread, notification_stack,
			K_THREAD_STACK_SIZEOF(notification_stack), run_notification,
			NULL, NULL, NULL, 1, 0, K_NO_WAIT);
	if (hold_allocation) {
		zassert_ok(k_sem_take(&notification_entered, K_SECONDS(1)));
	} else {
		/* Enqueue succeeds, but the test controller withholds completion. */
		zassert_ok(k_sem_take(&notification_queued, K_SECONDS(1)));
	}
	k_thread_create(&notification_shutdown_thread, notification_shutdown_stack,
			K_THREAD_STACK_SIZEOF(notification_shutdown_stack), shutdown_notification,
			NULL, NULL, NULL, 1, 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&notification_shutdown_started, K_SECONDS(1)));
	shutdown_ret = k_sem_take(&notification_shutdown_done, K_MSEC(50));
	/* Release either seam before asserting, including in a broken driver. */
	k_sem_give(&notification_release);
	k_sem_give(&data->notif_sem);
	notify_join = k_thread_join(&notification_thread, K_SECONDS(1));
	shutdown_join = k_thread_join(&notification_shutdown_thread, K_SECONDS(1));
	allocations = notification_allocations;
	/* Still detached: no new notification may even enter allocation. */
	if (atomic_test_bit(&data->state, CDC_ACM_CLASS_ENABLED)) {
		/* Old controls have no shutdown: do not look up an endpoint through NULL. */
		disabled_ret = 0;
	} else {
		disabled_ret = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 0U);
	}
	rx_class()->uds_ctx = ctx;
	notification_test = false;
	test_work_reset();
	zassert_ok(usbd_cdc_acm_init(rx_class()));
	usbd_cdc_acm_enable(rx_class());
	zassert_ok(notify_join);
	zassert_ok(shutdown_join);
	zassert_false(notification_detached, "shutdown detached a notification section");
	if (hold_allocation) {
		zassert_equal(shutdown_ret, -EAGAIN,
			      "shutdown did not join notification allocation");
		zassert_equal(notification_result, -EIO);
	} else {
		zassert_ok(shutdown_ret, "shutdown waited for notification completion");
		zassert_ok(notification_result);
	}
	zassert_equal(disabled_ret, -EACCES, "detached notification was not refused");
	zassert_equal(notification_allocations, allocations,
		      "late notification reached allocation");
}

ZTEST(cdc_rx, test_notification_shutdown_joins_section)
{
	notification_shutdown_case(true);
	notification_shutdown_case(false);
}

ZTEST_SUITE(cdc_rx, NULL, setup, before, after, NULL);
