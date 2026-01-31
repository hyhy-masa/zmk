/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>

#include <zmk/pm.h>

#include <zmk/activity.h>

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_POINTING)
#include <zephyr/input/input.h>
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

bool is_usb_power_present(void) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    return zmk_usb_is_powered();
#else
    return false;
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */
}

static enum zmk_activity_state activity_state;

static uint32_t activity_last_uptime;

#define MAX_IDLE_MS CONFIG_ZMK_IDLE_TIMEOUT

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
#define MAX_SLEEP_MS CONFIG_ZMK_IDLE_SLEEP_TIMEOUT
#else
#define MAX_SLEEP_MS 0
#endif

struct activity_setting_state {
    uint32_t sleep_ms;
    uint32_t idle_ms;
};

struct activity_setting_state activity_settings = {
    .sleep_ms = MAX_SLEEP_MS,
    .idle_ms = MAX_IDLE_MS,
};

#if IS_ENABLED(CONFIG_SETTINGS)
static int activity_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb,
                                     void *cb_arg) {
    if (strcmp(name, "s") == 0 && len == sizeof(activity_settings)) {
        int rc = read_cb(cb_arg, &activity_settings, sizeof(activity_settings));
        return MIN(rc, 0);
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(activity, "activity", NULL, activity_settings_load_cb, NULL, NULL);

static void activity_settings_save_work_handler(struct k_work *work) {
    settings_save_one("activity/s", &activity_settings, sizeof(activity_settings));
}

K_WORK_DELAYABLE_DEFINE(activity_settings_save_work, activity_settings_save_work_handler);

#endif

int raise_event(void) {
    return raise_zmk_activity_state_changed(
        (struct zmk_activity_state_changed){.state = activity_state});
}

int set_state(enum zmk_activity_state state) {
    if (activity_state == state)
        return 0;

    activity_state = state;
    return raise_event();
}

enum zmk_activity_state zmk_activity_get_state(void) { return activity_state; }

uint32_t zmk_activity_get_sleep_ms(void) { return activity_settings.sleep_ms; }

uint32_t zmk_activity_get_idle_ms(void) { return activity_settings.idle_ms; }

bool zmk_activity_set_sleep_ms(uint32_t sleep_ms) {
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    activity_settings.sleep_ms = sleep_ms;
    LOG_INF("Updated sleep timeout to %d ms", sleep_ms);
    k_work_schedule(&activity_settings_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return true;
#else
    LOG_WRN("Sleep functionality is not enabled");
    return false;
#endif
}

bool zmk_activity_set_idle_ms(uint32_t idle_ms) {
    activity_settings.idle_ms = idle_ms;
    LOG_INF("Updated idle timeout to %d ms", idle_ms);
    k_work_schedule(&activity_settings_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return true;
}

static int note_activity(void) {
    activity_last_uptime = k_uptime_get();

    return set_state(ZMK_ACTIVITY_ACTIVE);
}

static int activity_event_listener(const zmk_event_t *eh) { return note_activity(); }

void activity_work_handler(struct k_work *work) {
    int32_t current = k_uptime_get();
    int32_t inactive_time = current - activity_last_uptime;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    if (activity_settings.sleep_ms > 0 && inactive_time > activity_settings.sleep_ms &&
        !is_usb_power_present()) {
        // Put devices in suspend power mode before sleeping
        set_state(ZMK_ACTIVITY_SLEEP);

        if (zmk_pm_suspend_devices() < 0) {
            LOG_ERR("Failed to suspend all the devices");
            zmk_pm_resume_devices();
            return;
        }

        sys_poweroff();
    } else
#endif /* IS_ENABLED(CONFIG_ZMK_SLEEP) */
        if (activity_settings.idle_ms > 0 && inactive_time > activity_settings.idle_ms) {
            set_state(ZMK_ACTIVITY_IDLE);
        }
}

K_WORK_DEFINE(activity_work, activity_work_handler);

void activity_expiry_function(struct k_timer *_timer) { k_work_submit(&activity_work); }

K_TIMER_DEFINE(activity_timer, activity_expiry_function, NULL);

static int activity_init(void) {
    activity_last_uptime = k_uptime_get();

    k_timer_start(&activity_timer, K_SECONDS(1), K_SECONDS(1));
    return 0;
}

ZMK_LISTENER(activity, activity_event_listener);
ZMK_SUBSCRIPTION(activity, zmk_position_state_changed);
ZMK_SUBSCRIPTION(activity, zmk_sensor_event);

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static void note_activity_work_cb(struct k_work *_work) { note_activity(); }

K_WORK_DEFINE(note_activity_work, note_activity_work_cb);

static void activity_input_listener(struct input_event *ev) { k_work_submit(&note_activity_work); }

INPUT_CALLBACK_DEFINE(NULL, activity_input_listener);

#endif

SYS_INIT(activity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
