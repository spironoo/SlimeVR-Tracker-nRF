/*
 * Copyright (c) 2026 SlimeVR Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Persistent record of programmatic DFU/bootloader entries.
 *
 * Whenever the firmware itself decides to reboot into the bootloader
 * (watchdog recovery, console/remote command, button action), a small
 * record is written to NVS first. Because NVS lives in flash it survives
 * power-on resets and the DFU round trip, so an unexpected visit to the
 * bootloader can be diagnosed after the fact: the reason is logged at the
 * next boot.
 *
 * If the device is found sitting in DFU and the next boot does NOT report
 * a recent record, the entry was made by the bootloader on its own
 * (button pin, double reset, invalid app) rather than by the firmware.
 */

#pragma once

#include <stdint.h>

/* NVS ID for the diagnostic record; keep clear of IDs in system.h (1-8, 29-39) */
#define DFU_DIAG_NVS_ID 45

#define DFU_DIAG_REASON_NONE 0
#define DFU_DIAG_REASON_WATCHDOG 1 /* repeated WDT resets reached threshold */
#define DFU_DIAG_REASON_REQUEST 2  /* console command, button action, or remote command */

/**
 * @brief Persist the reason for an imminent programmatic DFU entry.
 *
 * Call immediately before setting the bootloader magic / rebooting.
 * Safe to call from any thread context; writes through the NVS layer.
 *
 * @param reason DFU_DIAG_REASON_*
 * @param detail reason-specific detail (watchdog: failing WDT channel;
 *               request: 1 for OTA, 0 for UF2/serial)
 */
void dfu_diag_record(uint8_t reason, uint8_t detail);
