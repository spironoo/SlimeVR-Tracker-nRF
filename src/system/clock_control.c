/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "clock_control.h"
#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>
#include <zephyr/logging/log.h>

// clock_control already has a log module defined in nrf_clock_control, so we define our own for this file
LOG_MODULE_REGISTER(clock_switch, LOG_LEVEL_INF);

/*
 * IMPORTANT: while the LF clock is being switched it may be stopped or dead.
 * The kernel tick (RTC) and the hardware watchdog are both clocked by LFCLK,
 * so neither k_usleep()/k_msleep() nor k_busy_wait() may be used in the
 * switching path: on nRF5 all of them are backed by the RTC, and if the new
 * source produces no edges they never return, freezing the entire timebase -
 * watchdog included. All waiting below uses a CPU-cycle delay loop instead,
 * and every wait is bounded, so a missing or gated oscillator degrades to
 * the internal RC oscillator instead of hanging the system.
 */

#define LFCLK_POLL_STEP_US 50
#define LFCLK_STOP_TIMEOUT_US 100000 /* 100 ms */
#define LFCLK_RC_START_TIMEOUT_US 100000 /* 100 ms; RC typically starts in <1 ms */
/* Crystal LFXO startup is typically ~250 ms and can approach 1 s */
#define LFCLK_EXT_START_TIMEOUT_US 1000000 /* 1 s */
/* Must span several 32.768 kHz periods (30.5 us each) */
#define LFCLK_EDGE_CHECK_US 300

/* CPU-cycle based delay, independent of the LF clock. Calibrated
 * conservatively for a 64 MHz Cortex-M4: the volatile counter loop takes
 * 3-6 cycles per iteration, so 22 iterations/us guarantees at least the
 * requested delay (and at most ~2x, which only lengthens timeouts). */
static void lfclk_cpu_delay_us(uint32_t us)
{
	for (volatile uint32_t i = us * 22U; i != 0U; i--) {
	}
}

/* Helper to normalize XTAL variants for comparison */
static inline nrf_clock_lfclk_t normalize_source(nrf_clock_lfclk_t source)
{
	/* XTAL_FULL_SWING and XTAL_LOW_SWING report as XTAL in actual source */
	if (false
#ifdef NRF_CLOCK_LFCLK_XTAL_FULL_SWING
		|| source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING
#endif
#ifdef NRF_CLOCK_LFCLK_XTAL_LOW_SWING
		|| source == NRF_CLOCK_LFCLK_XTAL_LOW_SWING
#endif
	) {
		return NRF_CLOCK_LFCLK_XTAL;
	}
	return source;
}

static bool lfclk_running_source_get(nrf_clock_lfclk_t *source)
{
	nrf_clock_lfclk_t active_source = NRF_CLOCK_LFCLK_RC;
	bool running = nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &active_source);

	if (source != NULL) {
		*source = normalize_source(active_source);
	}

	return running;
}

static bool lfclk_wait_started(uint32_t timeout_us)
{
	uint32_t waited_us = 0;
	while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
		if (waited_us >= timeout_us) {
			return false;
		}
		lfclk_cpu_delay_us(LFCLK_POLL_STEP_US);
		waited_us += LFCLK_POLL_STEP_US;
	}
	return true;
}

static void lfclk_stop_bounded(void)
{
	if (!lfclk_running_source_get(NULL)) {
		return;
	}
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
	uint32_t waited_us = 0;
	while (lfclk_running_source_get(NULL)) {
		if (waited_us >= LFCLK_STOP_TIMEOUT_US) {
			LOG_ERR("clock_switch: LFCLK did not stop");
			return;
		}
		lfclk_cpu_delay_us(LFCLK_POLL_STEP_US);
		waited_us += LFCLK_POLL_STEP_US;
	}
}

/* The kernel cycle counter is driven by the RTC, which counts LFCLK edges.
 * LFCLKSTARTED alone is not proof of a usable clock: in external full-swing
 * (bypass) mode it can fire while the driving oscillator is still gated off,
 * leaving a "running" LF clock that never ticks. Verify real edges. */
static bool lfclk_producing_edges(void)
{
	uint32_t start = k_cycle_get_32();
	lfclk_cpu_delay_us(LFCLK_EDGE_CHECK_US);
	return k_cycle_get_32() != start;
}

static bool lfclk_start_source(nrf_clock_lfclk_t source, uint32_t timeout_us)
{
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	nrf_clock_lf_src_set(NRF_CLOCK, source);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
	if (!lfclk_wait_started(timeout_us)) {
		LOG_ERR("clock_switch: source=%d did not start within %u us", source, timeout_us);
		return false;
	}
	if (!lfclk_producing_edges()) {
		LOG_ERR("clock_switch: source=%d started but produces no edges", source);
		return false;
	}
	return true;
}

// Safely switch LF clock source.
// Returns true when the requested source ends up running; on failure the
// clock is restored to the internal RC oscillator and false is returned.
bool clock_switch(nrf_clock_lfclk_t source)
{
	LOG_INF("clock_switch: requesting source=%d", source);

#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	/*
	 * Avoid switching to XTAL when the board does not have an external LFXO.
	 * Note: switching to RC is always safe.
	 */
	bool xtal_source = source == NRF_CLOCK_LFCLK_XTAL;
#ifdef NRF_CLOCK_LFCLK_XTAL_FULL_SWING
	xtal_source = xtal_source || source == NRF_CLOCK_LFCLK_XTAL_FULL_SWING;
#endif
#ifdef NRF_CLOCK_LFCLK_XTAL_LOW_SWING
	xtal_source = xtal_source || source == NRF_CLOCK_LFCLK_XTAL_LOW_SWING;
#endif
	if (!IS_ENABLED(CONFIG_CLOCK_USE_LFXO) && xtal_source) {
		LOG_INF("clock_switch: skipping XTAL, CONFIG_CLOCK_USE_LFXO disabled");
		return false;
	}
#endif

	/* Check if already running with the requested source */
	nrf_clock_lfclk_t current_source;
	bool running = lfclk_running_source_get(&current_source);
	nrf_clock_lfclk_t normalized_requested = normalize_source(source);

	if (running && current_source == normalized_requested) {
		LOG_INF("clock_switch: already running with source=%d", current_source);
		return true;
	}

	LOG_INF("clock_switch: %d -> %d", current_source, normalized_requested);

	lfclk_stop_bounded();

	uint32_t timeout_us = source == NRF_CLOCK_LFCLK_RC ? LFCLK_RC_START_TIMEOUT_US
							   : LFCLK_EXT_START_TIMEOUT_US;
	if (!lfclk_start_source(source, timeout_us)) {
		if (source == NRF_CLOCK_LFCLK_RC) {
			/* RC itself failed to start: nothing left to fall back to */
			return false;
		}
		/* Fall back to the internal RC oscillator so the kernel tick and
		 * the watchdog keep running instead of freezing the system. */
		LOG_ERR("clock_switch: falling back to RC oscillator");
		lfclk_stop_bounded();
		if (!lfclk_start_source(NRF_CLOCK_LFCLK_RC, LFCLK_RC_START_TIMEOUT_US)) {
			LOG_ERR("clock_switch: RC fallback failed");
		}
		return false;
	}

	/* Verify the actual clock source matches what we requested */
	nrf_clock_lfclk_t actual_source;
	bool actual_running = lfclk_running_source_get(&actual_source);
	if (!actual_running || actual_source != normalized_requested) {
		LOG_ERR("clock_switch: source mismatch! requested=%d, actual=%d", normalized_requested, actual_source);
		return false;
	}
	LOG_INF("clock_switch: switched to source=%d successfully", actual_source);
	return true;
}

// Switch to RC clock before shut down to avoid any problems with the bootloader
void clock_pre_shutdown(void)
{
	nrf_clock_lfclk_t current_source;
	bool running = lfclk_running_source_get(&current_source);

	if (running && current_source != NRF_CLOCK_LFCLK_RC) {
		clock_switch(NRF_CLOCK_LFCLK_RC);
	}
}

// Switch to external oscillator for LF clock for good TDMA precision
void clock_init_external(void)
{
#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES) || defined(__NRFX_DOXYGEN__)
	if (IS_ENABLED(CONFIG_CLOCK_USE_LFXO)) {
		nrf_clock_lfclk_t source = NRF_CLOCK_LFCLK_XTAL;
#ifdef NRF_CLOCK_LFCLK_XTAL_FULL_SWING
		if (IS_ENABLED(CONFIG_CLOCK_LFXO_FULL_SWING)) {
			/* XL1 is driven by an active oscillator (rail-to-rail square
			 * wave), not a crystal: use external full-swing (bypass) mode */
			source = NRF_CLOCK_LFCLK_XTAL_FULL_SWING;
		}
#endif
		/* The oscillator may still be powering up (MEMS parts need ~150 ms,
		 * and on some boards it is gated by a GPIO asserted during init),
		 * so retry before giving up. Between attempts the LF clock runs on
		 * RC (clock_switch falls back on failure), so sleeping is safe. */
		for (int attempt = 0; attempt < 4; attempt++) {
			if (attempt > 0) {
				if (!lfclk_running_source_get(NULL)) {
					break; /* no timebase to sleep on; should not happen */
				}
				k_msleep(500);
			}
			if (clock_switch(source)) {
				return;
			}
		}
		LOG_ERR("clock_init_external: LFXO unavailable, staying on RC oscillator");
	} else if (IS_ENABLED(CONFIG_CLOCK_USE_LF_SYNTH)) {
		/* Use LF synthesizer (derived from HFXO) for TDMA timing precision
		 * when LFXO is not available on the board */
#ifdef NRF_CLOCK_LFCLK_SYNTH
		clock_switch(NRF_CLOCK_LFCLK_SYNTH);
#else
		LOG_WRN("clock_init_external: LF_SYNTH requested but not supported");
#endif
	}
#endif
}

// Async version of clock_init_external
static struct k_thread clock_init_thread_id;
static K_THREAD_STACK_DEFINE(clock_init_thread_stack, 512);

#ifndef CLOCK_INIT_THREAD_PRIORITY
#define CLOCK_INIT_THREAD_PRIORITY 8
#endif

static void clock_init_external_async_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	clock_init_external();
}

void clock_init_external_async(void)
{
	k_thread_create(
		&clock_init_thread_id,
		clock_init_thread_stack,
		K_THREAD_STACK_SIZEOF(clock_init_thread_stack),
		clock_init_external_async_thread,
		NULL,
		NULL,
		NULL,
		CLOCK_INIT_THREAD_PRIORITY,
		0,
		K_NO_WAIT
	);
}
