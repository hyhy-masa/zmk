/*
 * Copyright (c) 2026 The minimal-keys Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_data_logger

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <dt-bindings/zmk/data-logger.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Driver-side APIs implemented in zmk-pmw3610-driver/src/data_logger.c.
 * Linked at build time; weak fallback below if logger is compiled out. */
extern void pmw3610_dlog_freeze(void) __attribute__((weak));
extern void pmw3610_dlog_clear(void) __attribute__((weak));
extern void pmw3610_dlog_set_marker(uint8_t marker_id) __attribute__((weak));

/* BLE dump trigger lives in the data-logger BLE service (added next). */
extern void pmw3610_dlog_request_dump(void) __attribute__((weak));

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    uint32_t cmd = binding->param1;

    switch (cmd) {
    case DLOG_FREEZE:
        if (pmw3610_dlog_freeze) {
            pmw3610_dlog_freeze();
        }
        LOG_INF("dlog: freeze");
        break;
    case DLOG_DUMP:
        if (pmw3610_dlog_request_dump) {
            pmw3610_dlog_request_dump();
        }
        LOG_INF("dlog: dump requested");
        break;
    case DLOG_CLEAR:
        if (pmw3610_dlog_clear) {
            pmw3610_dlog_clear();
        }
        LOG_INF("dlog: clear");
        break;
    case DLOG_MARKER_A:
        if (pmw3610_dlog_set_marker) {
            pmw3610_dlog_set_marker(0x01);
        }
        LOG_INF("dlog: marker A");
        break;
    case DLOG_MARKER_B:
        if (pmw3610_dlog_set_marker) {
            pmw3610_dlog_set_marker(0x02);
        }
        LOG_INF("dlog: marker B");
        break;
    default:
        LOG_WRN("dlog: unknown command %u", cmd);
        break;
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_data_logger_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_data_logger_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
