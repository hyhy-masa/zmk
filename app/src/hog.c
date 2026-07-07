/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/settings/settings.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include <zmk/ble.h>
#include <zmk/endpoints_types.h>
#include <zmk/hog.h>
#include <zmk/hid.h>
#include <zmk/mk2_ble_diag.h>
#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#include <zmk/pointing/resolution_multipliers.h>
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version; /* version number of base USB HID Specification */
    uint8_t code;     /* country HID Device hardware is localized for. */
    uint8_t flags;
} __packed;

struct hids_report {
    uint8_t id;   /* report id */
    uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE | HIDS_REMOTE_WAKE,
};

enum {
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
    .id = ZMK_HID_REPORT_ID_KEYBOARD,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static struct hids_report led_indicators = {
    .id = ZMK_HID_REPORT_ID_LEDS,
    .type = HIDS_OUTPUT,
};

#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static struct hids_report consumer_input = {
    .id = ZMK_HID_REPORT_ID_CONSUMER,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static struct hids_report mouse_input = {
    .id = ZMK_HID_REPORT_ID_MOUSE,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

static struct hids_report mouse_feature = {
    .id = ZMK_HID_REPORT_ID_MOUSE,
    .type = HIDS_FEATURE,
};

#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static bool host_requests_notification = false;
static uint8_t ctrl_point;
// static uint8_t proto_mode;

static ssize_t read_hids_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_info));
}

static ssize_t read_hids_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_report));
}

static ssize_t read_hids_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, zmk_hid_report_desc,
                             sizeof(zmk_hid_report_desc));
}

static ssize_t read_hids_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset) {
    struct zmk_hid_keyboard_report_body *report_body = &zmk_hid_get_keyboard_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_keyboard_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static ssize_t write_hids_leds_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(struct zmk_hid_led_report_body)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct zmk_hid_led_report_body *report = (struct zmk_hid_led_report_body *)buf;
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {.transport = ZMK_TRANSPORT_BLE,
                                             .ble = {
                                                 .profile_index = profile,
                                             }};
    zmk_hid_indicators_process_report(report, endpoint);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static ssize_t read_hids_consumer_input_report(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr, void *buf,
                                               uint16_t len, uint16_t offset) {
    struct zmk_hid_consumer_report_body *report_body = &zmk_hid_get_consumer_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_consumer_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static ssize_t read_hids_mouse_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len, uint16_t offset) {
    struct zmk_hid_mouse_report_body *report_body = &zmk_hid_get_mouse_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_mouse_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

static ssize_t read_hids_mouse_feature_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                              void *buf, uint16_t len, uint16_t offset) {

    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        LOG_DBG("   BT_ATT_ERR_UNLIKELY");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {
        .transport = ZMK_TRANSPORT_BLE,
        .ble = {.profile_index = profile},
    };

    struct zmk_pointing_resolution_multipliers mult =
        zmk_pointing_resolution_multipliers_get_profile(endpoint);

    struct zmk_hid_mouse_resolution_feature_report_body report = {
        .wheel_res = mult.wheel,
        .hwheel_res = mult.hor_wheel,
    };

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &report,
                             sizeof(struct zmk_hid_mouse_resolution_feature_report_body));
}

static ssize_t write_hids_mouse_feature_report(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr, const void *buf,
                                               uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(struct zmk_hid_mouse_resolution_feature_report_body)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct zmk_hid_mouse_resolution_feature_report_body *report =
        (struct zmk_hid_mouse_resolution_feature_report_body *)buf;
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {.transport = ZMK_TRANSPORT_BLE,
                                             .ble = {
                                                 .profile_index = profile,
                                             }};
    zmk_pointing_resolution_multipliers_process_report(report, endpoint);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

// static ssize_t write_proto_mode(struct bt_conn *conn,
//                                 const struct bt_gatt_attr *attr,
//                                 const void *buf, uint16_t len, uint16_t offset,
//                                 uint8_t flags)
// {
//     printk("PROTO CHANGED\n");
//     return 0;
// }

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    host_requests_notification = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(ctrl_point)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    return len;
}

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(
    hog_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    //    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_PROTOCOL_MODE, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    //                           BT_GATT_PERM_WRITE, NULL, write_proto_mode, &proto_mode),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_hids_info,
                           NULL, &info),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,
                           read_hids_report_map, NULL, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &input),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_consumer_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &consumer_input),

#if IS_ENABLED(CONFIG_ZMK_POINTING)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_mouse_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &mouse_input),

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_hids_mouse_feature_report, write_hids_mouse_feature_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &mouse_feature),
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           write_hids_leds_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &led_indicators),
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point));

K_THREAD_STACK_DEFINE(hog_q_stack, CONFIG_ZMK_BLE_THREAD_STACK_SIZE);

struct k_work_q hog_work_q;

K_MSGQ_DEFINE(zmk_hog_keyboard_msgq, sizeof(struct zmk_hid_keyboard_report_body),
              CONFIG_ZMK_BLE_KEYBOARD_REPORT_QUEUE_SIZE, 4);

void send_keyboard_report_callback(struct k_work *work) {
    struct zmk_hid_keyboard_report_body report;

    while (k_msgq_get(&zmk_hog_keyboard_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[5],
            .data = &report,
            .len = sizeof(report),
        };

#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        uint32_t mk2_notify_start = k_cycle_get_32();
#endif
        int err = bt_gatt_notify_cb(conn, &notify_params);
#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        mk2_ble_diag_note_block(MK2_BLE_DIAG_PIPE_KEYBOARD,
                                k_cyc_to_us_floor32(k_cycle_get_32() - mk2_notify_start));
#endif
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }
        mk2_ble_diag_note_notify(MK2_BLE_DIAG_PIPE_KEYBOARD, err);

        bt_conn_unref(conn);
    }
}

K_WORK_DEFINE(hog_keyboard_work, send_keyboard_report_callback);

int zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *report) {
    int err = k_msgq_put(&zmk_hog_keyboard_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Keyboard message queue full, popping first message and queueing again");
            struct zmk_hid_keyboard_report_body discarded_report;
            k_msgq_get(&zmk_hog_keyboard_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_keyboard_report(report);
        }
        default:
            LOG_WRN("Failed to queue keyboard report to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&hog_work_q, &hog_keyboard_work);

    return 0;
};

K_MSGQ_DEFINE(zmk_hog_consumer_msgq, sizeof(struct zmk_hid_consumer_report_body),
              CONFIG_ZMK_BLE_CONSUMER_REPORT_QUEUE_SIZE, 4);

void send_consumer_report_callback(struct k_work *work) {
    struct zmk_hid_consumer_report_body report;

    while (k_msgq_get(&zmk_hog_consumer_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[9],
            .data = &report,
            .len = sizeof(report),
        };

#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        uint32_t mk2_notify_start = k_cycle_get_32();
#endif
        int err = bt_gatt_notify_cb(conn, &notify_params);
#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        mk2_ble_diag_note_block(MK2_BLE_DIAG_PIPE_CONSUMER,
                                k_cyc_to_us_floor32(k_cycle_get_32() - mk2_notify_start));
#endif
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }
        mk2_ble_diag_note_notify(MK2_BLE_DIAG_PIPE_CONSUMER, err);

        bt_conn_unref(conn);
    }
};

K_WORK_DEFINE(hog_consumer_work, send_consumer_report_callback);

int zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *report) {
    int err = k_msgq_put(&zmk_hog_consumer_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Consumer message queue full, popping first message and queueing again");
            struct zmk_hid_consumer_report_body discarded_report;
            k_msgq_get(&zmk_hog_consumer_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_consumer_report(report);
        }
        default:
            LOG_WRN("Failed to queue consumer report to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&hog_work_q, &hog_consumer_work);

    return 0;
};

#if IS_ENABLED(CONFIG_ZMK_POINTING)

#if IS_ENABLED(CONFIG_MK2_HOG_MOUSE_ACC)

/*
 * MK2 Phase 4: mouse delta accumulator.
 * The 250 Hz trackball outruns BLE delivery (~66-100 reports/s at a 15 ms conn
 * interval with 3 shared TX buffers). The stock msgq backs up, the K_FOREVER PDU
 * alloc stalls the HOG thread, backpressure freezes the sensor poll, and the
 * drained backlog replays stale deltas as cursor warp (measured: 11.6% of D0
 * notifies blocked >=15 ms, q_hwm=20). Instead we coalesce pending deltas into a
 * small button-segmented ring and send ONE fresh report per send opportunity,
 * self-paced by the TX-buffer wait. Distinct button states open new segments so
 * a click's press/release is preserved while the 4-slot ring has capacity; a
 * full ring (>4 button transitions in one stall -- practically unreachable)
 * merges into the tail with the newest buttons and is counted as a diag drop.
 * Deltas are int64 while pending and clamped-with-carry to the int16 HID range
 * on send. The
 * accumulator is shared between the input-enqueue thread (producer) and the
 * hog_work_q thread (consumer), so it is guarded by a spinlock; the blocking
 * notify is always issued outside the lock.
 */

#define MK2_MOUSE_ACC_SEGMENTS 4

struct mk2_mouse_seg {
    zmk_mouse_button_flags_t buttons;
    /* int64 so the producer's unchecked += can never overflow while pending,
     * even under a long undrained stall (clamped to int16 on send). */
    int64_t d_x, d_y, d_scroll_y, d_scroll_x;
};

static struct mk2_mouse_seg mk2_mouse_segs[MK2_MOUSE_ACC_SEGMENTS];
static uint8_t mk2_mouse_seg_head;  /* oldest pending segment */
static uint8_t mk2_mouse_seg_count; /* pending segments, 0..MK2_MOUSE_ACC_SEGMENTS */
static struct k_spinlock mk2_mouse_lock;

static inline int16_t mk2_clamp_i16(int64_t v) {
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
}

void send_mouse_report_callback(struct k_work *work) {
    while (true) {
        /* Acquire the connection BEFORE draining a segment: on a NULL conn we
         * must leave the accumulator intact rather than lose the deltas. */
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct zmk_hid_mouse_report_body report;
        bool have = false;
        k_spinlock_key_t key = k_spin_lock(&mk2_mouse_lock);
        if (mk2_mouse_seg_count > 0) {
            struct mk2_mouse_seg *seg = &mk2_mouse_segs[mk2_mouse_seg_head];
            report.buttons = seg->buttons;
            report.d_x = mk2_clamp_i16(seg->d_x);
            report.d_y = mk2_clamp_i16(seg->d_y);
            report.d_scroll_y = mk2_clamp_i16(seg->d_scroll_y);
            report.d_scroll_x = mk2_clamp_i16(seg->d_scroll_x);
            /* Clamp-and-carry: keep the residual, retire only once fully drained. */
            seg->d_x -= report.d_x;
            seg->d_y -= report.d_y;
            seg->d_scroll_y -= report.d_scroll_y;
            seg->d_scroll_x -= report.d_scroll_x;
            if (seg->d_x == 0 && seg->d_y == 0 && seg->d_scroll_y == 0 &&
                seg->d_scroll_x == 0) {
                mk2_mouse_seg_head = (mk2_mouse_seg_head + 1) % MK2_MOUSE_ACC_SEGMENTS;
                mk2_mouse_seg_count--;
            }
            have = true;
        }
        k_spin_unlock(&mk2_mouse_lock, key);

        if (!have) {
            bt_conn_unref(conn);
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[13],
            .data = &report,
            .len = sizeof(report),
        };

#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        uint32_t mk2_notify_start = k_cycle_get_32();
#endif
        int err = bt_gatt_notify_cb(conn, &notify_params);
#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        mk2_ble_diag_note_block(MK2_BLE_DIAG_PIPE_MOUSE,
                                k_cyc_to_us_floor32(k_cycle_get_32() - mk2_notify_start));
#endif
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }
        mk2_ble_diag_note_notify(MK2_BLE_DIAG_PIPE_MOUSE, err);

        bt_conn_unref(conn);
    }
};

K_WORK_DEFINE(hog_mouse_work, send_mouse_report_callback);

void zmk_hog_clear_mouse_queue(void) {
    k_spinlock_key_t key = k_spin_lock(&mk2_mouse_lock);
    mk2_mouse_seg_head = 0;
    mk2_mouse_seg_count = 0;
    k_spin_unlock(&mk2_mouse_lock, key);
}

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
    k_spinlock_key_t key = k_spin_lock(&mk2_mouse_lock);
    struct mk2_mouse_seg *tail = NULL;
    if (mk2_mouse_seg_count > 0) {
        uint8_t tail_idx =
            (mk2_mouse_seg_head + mk2_mouse_seg_count - 1) % MK2_MOUSE_ACC_SEGMENTS;
        tail = &mk2_mouse_segs[tail_idx];
    }
    if (tail != NULL && tail->buttons == report->buttons) {
        /* Same button state: coalesce into the tail segment. */
        tail->d_x += report->d_x;
        tail->d_y += report->d_y;
        tail->d_scroll_y += report->d_scroll_y;
        tail->d_scroll_x += report->d_scroll_x;
    } else if (mk2_mouse_seg_count < MK2_MOUSE_ACC_SEGMENTS) {
        /* Button transition: open a new segment to preserve click ordering. */
        uint8_t idx = (mk2_mouse_seg_head + mk2_mouse_seg_count) % MK2_MOUSE_ACC_SEGMENTS;
        struct mk2_mouse_seg *seg = &mk2_mouse_segs[idx];
        seg->buttons = report->buttons;
        seg->d_x = report->d_x;
        seg->d_y = report->d_y;
        seg->d_scroll_y = report->d_scroll_y;
        seg->d_scroll_x = report->d_scroll_x;
        mk2_mouse_seg_count++;
    } else {
        /* Ring full (>2 click cycles inside one stall -- rare): merge into the
         * tail and take the newest button state. Counted for the diagnostics. */
        tail->d_x += report->d_x;
        tail->d_y += report->d_y;
        tail->d_scroll_y += report->d_scroll_y;
        tail->d_scroll_x += report->d_scroll_x;
        tail->buttons = report->buttons;
        mk2_ble_diag_note_drop(MK2_BLE_DIAG_PIPE_MOUSE);
    }
    uint32_t used = mk2_mouse_seg_count;
    k_spin_unlock(&mk2_mouse_lock, key);

    mk2_ble_diag_note_queue(MK2_BLE_DIAG_PIPE_MOUSE, used);
    k_work_submit_to_queue(&hog_work_q, &hog_mouse_work);

    return 0;
};

#else /* !CONFIG_MK2_HOG_MOUSE_ACC: stock fixed-depth msgq (rollback path) */

K_MSGQ_DEFINE(zmk_hog_mouse_msgq, sizeof(struct zmk_hid_mouse_report_body),
              CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE, 4);

void send_mouse_report_callback(struct k_work *work) {
    struct zmk_hid_mouse_report_body report;
    while (k_msgq_get(&zmk_hog_mouse_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[13],
            .data = &report,
            .len = sizeof(report),
        };

#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        uint32_t mk2_notify_start = k_cycle_get_32();
#endif
        int err = bt_gatt_notify_cb(conn, &notify_params);
#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
        mk2_ble_diag_note_block(MK2_BLE_DIAG_PIPE_MOUSE,
                                k_cyc_to_us_floor32(k_cycle_get_32() - mk2_notify_start));
#endif
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }
        mk2_ble_diag_note_notify(MK2_BLE_DIAG_PIPE_MOUSE, err);

        bt_conn_unref(conn);
    }
};

K_WORK_DEFINE(hog_mouse_work, send_mouse_report_callback);

void zmk_hog_clear_mouse_queue(void) {
    k_msgq_purge(&zmk_hog_mouse_msgq);
}

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
    int err = k_msgq_put(&zmk_hog_mouse_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Mouse message queue full, popping first message and queueing again");
            struct zmk_hid_mouse_report_body discarded_report;
            k_msgq_get(&zmk_hog_mouse_msgq, &discarded_report, K_NO_WAIT);
            mk2_ble_diag_note_drop(MK2_BLE_DIAG_PIPE_MOUSE);
            return zmk_hog_send_mouse_report(report);
        }
        default:
            LOG_WRN("Failed to queue mouse report to send (%d)", err);
            return err;
        }
    }

#if IS_ENABLED(CONFIG_MK2_BLE_DIAG)
    mk2_ble_diag_note_queue(MK2_BLE_DIAG_PIPE_MOUSE, k_msgq_num_used_get(&zmk_hog_mouse_msgq));
#endif
    k_work_submit_to_queue(&hog_work_q, &hog_mouse_work);

    return 0;
};

#endif /* CONFIG_MK2_HOG_MOUSE_ACC */
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static int zmk_hog_init(void) {
    static const struct k_work_queue_config queue_config = {.name = "HID Over GATT Send Work"};
    k_work_queue_start(&hog_work_q, hog_q_stack, K_THREAD_STACK_SIZEOF(hog_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, &queue_config);

    return 0;
}

SYS_INIT(zmk_hog_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
