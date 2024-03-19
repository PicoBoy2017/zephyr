/*
 * Copyright (c) 2024 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief HCI driver for max32xxx single-chip bluetooth configuration.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth.h>

#include <ll_btctrl.h>
#include <chci_tr.h>
#include <chci_api.h>

#define DT_DRV_COMPAT adi_max32_hci

struct hci_data {
	bt_hci_recv_t recv;
};

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);

#define MAX32_BTCTRL_STACK_SIZE 1280

static K_KERNEL_STACK_DEFINE(max32_ctrl_stack, MAX32_BTCTRL_STACK_SIZE);
static struct k_thread max32_ctrl_thread;

static void max32_btctrl_run(void *p1, void *p2, void *p3)
{
	ll_btctrl_run();
}

static void max32_btctrl_recv(uint8_t prot, uint8_t type, uint16_t len, uint8_t *data)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct hci_data *hci = dev->data;
	struct net_buf *buf;
	struct bt_hci_evt_hdr hdr;

	memcpy((void *)&hdr, data, sizeof(hdr));
	data += sizeof(hdr);
	len -= sizeof(hdr);

	switch (type) {
	case CHCI_TR_TYPE_EVT:
		buf = bt_buf_get_evt(hdr.evt, false, K_FOREVER);
		break;
	case CHCI_TR_TYPE_ACL:
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
		break;
	case CHCI_TR_TYPE_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_FOREVER);
			break;
		}
		__fallthrough;
	default:
		buf = NULL;
		LOG_ERR("Unknown HCI type %u", type);
	}

	if (buf) {
		net_buf_add_mem(buf, &hdr, sizeof(hdr));
		net_buf_add_mem(buf, data, len);

		hci->recv(dev, buf);
	}
}

/* Chci implementations for packetcraft CHCI custom layer */
void ChciTrInit(uint16_t maxAclLen, uint16_t maxIsoSduLen)
{
	(void)maxAclLen;
	(void)maxIsoSduLen;
}

void ChciTrWrite(uint8_t prot, uint8_t type, uint16_t len, uint8_t *pData)
{
	max32_btctrl_recv(prot, type, len, pData);
	chciTrSendComplete();
}

static int max32_bt_send(const struct device *dev, struct net_buf *buf)
{
	int ret = 0;

	ARG_UNUSED(dev);

	LOG_DBG("%s: buf %p type %u len %u", __func__, buf, bt_buf_get_type(buf), buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		ll_btctrl_recv(CHCI_TR_PROT_BLE, CHCI_TR_TYPE_ACL, buf->data, buf->len);
		break;
	case BT_BUF_CMD:
		ll_btctrl_recv(CHCI_TR_PROT_BLE, CHCI_TR_TYPE_CMD, buf->data, buf->len);
		break;
	case BT_BUF_ISO_OUT:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			ll_btctrl_recv(CHCI_TR_PROT_BLE, CHCI_TR_TYPE_ISO, buf->data, buf->len);
			break;
		}
		__fallthrough;
	default:
		LOG_ERR("Unknown buffer type");
		ret = -EINVAL;
	}

	net_buf_unref(buf);
	return ret;
}

static int max32_bt_open(const struct device *dev, bt_hci_recv_t recv)
{
	struct hci_data *hci = dev->data;

	ll_btctrl_setup();

	/*
	 * Start controller thread;
	 * Process events set by the application send, or by BTLE & TMR ISRs.
	 */
	k_thread_create(&max32_ctrl_thread, max32_ctrl_stack,
			K_KERNEL_STACK_SIZEOF(max32_ctrl_stack), max32_btctrl_run, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO), 0, K_NO_WAIT);

	k_thread_name_set(&max32_ctrl_thread, "BT CTLR");

	hci->recv = recv;

	LOG_DBG("MAX32 BT started");

	return 0;
}

static const struct bt_hci_driver_api drv = {
	.open = max32_bt_open,
	.send = max32_bt_send,
};

#define HCI_DEVICE_INIT(inst)                                                                      \
	static struct hci_data hci_data_##inst = {};                                               \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &hci_data_##inst, NULL, POST_KERNEL,               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported right now */
HCI_DEVICE_INIT(0)
