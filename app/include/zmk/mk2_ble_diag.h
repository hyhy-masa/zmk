/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * MK2 BLE diagnostics (Phase 0). Disabled unless CONFIG_MK2_BLE_DIAG=y.
 *
 * Purpose: attribute the BLE-only mouse-lag / cursor-jump / stuck-key symptoms.
 * Captures HOG notify() outcomes per pipe and caches the active peripheral
 * connection interval so USB-unplugged (customer-condition) sessions can be
 * analyzed after the fact from the trackball data-logger dump. Adds no GATT
 * characteristics and changes no connection behavior, so it needs no
 * settings_reset. When the config is off, every call compiles to a no-op.
 */
#pragma once

#include <stdint.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HID notify pipes tracked by the diagnostics. */
#define MK2_BLE_DIAG_PIPE_KEYBOARD 0
#define MK2_BLE_DIAG_PIPE_CONSUMER 1
#define MK2_BLE_DIAG_PIPE_MOUSE    2

#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)

/* Record one bt_gatt_notify_cb() outcome for the given pipe. err == 0 => success. */
void mk2_ble_diag_note_notify(uint8_t pipe, int err);

/* Cache the active peripheral connection parameters (interval/timeout in 1.25ms
 * and 10ms units respectively, per Zephyr bt_conn_info). */
void mk2_ble_diag_set_conn_params(uint16_t interval, uint16_t latency, uint16_t timeout);

/* Last known connection interval in 1.25ms units (0 if unknown/disconnected). */
uint16_t mk2_ble_diag_interval_units(void);

/* Reset all counters. Call at the start of a measurement window (dlog clear). */
void mk2_ble_diag_reset(void);

/* Print the counter footer via printk. Call at dlog dump. */
void mk2_ble_diag_dump(void);

#else

static inline void mk2_ble_diag_note_notify(uint8_t pipe, int err) {
    (void)pipe;
    (void)err;
}
static inline void mk2_ble_diag_set_conn_params(uint16_t interval, uint16_t latency,
                                                uint16_t timeout) {
    (void)interval;
    (void)latency;
    (void)timeout;
}
static inline uint16_t mk2_ble_diag_interval_units(void) { return 0; }
static inline void mk2_ble_diag_reset(void) {}
static inline void mk2_ble_diag_dump(void) {}

#endif /* CONFIG_MK2_BLE_DIAG */

#ifdef __cplusplus
}
#endif
