/*
 * Copyright (c) 2026 SlimeVR Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "dfu_diag.h"
#include "system/system.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dfu_diag, LOG_LEVEL_INF);

#define DFU_DIAG_MAGIC 0x44465544 /* "DFUD" */

struct dfu_diag_record_s {
	uint32_t magic;
	uint32_t uptime_ms; /* uptime at the moment DFU entry was requested */
	uint8_t reason;     /* DFU_DIAG_REASON_* */
	uint8_t detail;
	uint16_t count;     /* total programmatic DFU entries over device life */
};

static struct dfu_diag_record_s last_record;

static const char *dfu_diag_reason_str(uint8_t reason)
{
	switch (reason) {
	case DFU_DIAG_REASON_WATCHDOG:
		return "watchdog reset threshold";
	case DFU_DIAG_REASON_REQUEST:
		return "user/remote request";
	default:
		return "unknown";
	}
}

void dfu_diag_record(uint8_t reason, uint8_t detail)
{
	struct dfu_diag_record_s record = {
		.magic = DFU_DIAG_MAGIC,
		.uptime_ms = k_uptime_get_32(),
		.reason = reason,
		.detail = detail,
		.count = (uint16_t)(last_record.count + 1),
	};
	LOG_WRN("Recording DFU entry: %s (detail %u) at uptime %u ms",
		dfu_diag_reason_str(reason), detail, record.uptime_ms);
	/* No retained mirror; NVS only, so the record survives POR and DFU */
	sys_write(DFU_DIAG_NVS_ID, NULL, &record, sizeof(record));
	last_record = record;
}

static void dfu_diag_log_status(void)
{
	if (last_record.magic == DFU_DIAG_MAGIC) {
		LOG_WRN("Last programmatic DFU entry: %s (detail %u) at uptime %u ms, %u total",
			dfu_diag_reason_str(last_record.reason), last_record.detail,
			last_record.uptime_ms, last_record.count);
	} else {
		LOG_INF("No programmatic DFU entries recorded (dfu_diag active)");
	}
}

/* Repeat the status periodically so it can be read over the USB console at
 * any time, instead of only in the earliest boot messages (which are often
 * emitted before USB has enumerated). */
static void dfu_diag_report_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dfu_diag_report_work, dfu_diag_report_work_fn);

static void dfu_diag_report_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	dfu_diag_log_status();
	k_work_schedule(&dfu_diag_report_work, K_SECONDS(60));
}

/* sys_read zeroes the buffer when no entry exists, failing the magic check */
static int dfu_diag_report(void)
{
	sys_read(DFU_DIAG_NVS_ID, &last_record, sizeof(last_record));
	if (last_record.magic != DFU_DIAG_MAGIC) {
		last_record = (struct dfu_diag_record_s){0};
	}
	dfu_diag_log_status();
	k_work_schedule(&dfu_diag_report_work, K_SECONDS(15));
	return 0;
}

/* After sys_retained_init (CONFIG_APPLICATION_INIT_PRIORITY) so NVS is up */
SYS_INIT(dfu_diag_report, APPLICATION, 99);
