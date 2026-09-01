/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/init.h>

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <drivers/behavior.h>
#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/matrix.h>
#include <zmk/physical_layouts.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/split/bluetooth/service.h>
#if IS_ENABLED(CONFIG_MK2_SPLIT_NOTIFY_STATS)
#include <zmk/workqueue.h>
#include <zmk/mk2_kscan_stats.h>
#endif

#include "peripheral.h"

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
#include <zmk/events/hid_indicators_changed.h>
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

#include <zmk/events/sensor_event.h>
#include <zmk/sensors.h>

#if ZMK_KEYMAP_HAS_SENSORS
static struct sensor_event last_sensor_event;

static ssize_t split_svc_sensor_state(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                      void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attrs, buf, len, offset, &last_sensor_event,
                             sizeof(last_sensor_event));
}

static void split_svc_sensor_state_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#define POS_STATE_LEN 16

static uint8_t num_of_positions = ZMK_KEYMAP_LEN;
static uint8_t position_state[POS_STATE_LEN];

static struct zmk_split_run_behavior_payload behavior_run_payload;

static ssize_t split_svc_pos_state(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                   void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attrs, buf, len, offset, &position_state,
                             sizeof(position_state));
}

static ssize_t split_svc_run_behavior(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags);

static ssize_t split_svc_num_of_positions(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                          void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attrs, buf, len, offset, attrs->user_data, sizeof(uint8_t));
}

static void split_svc_pos_state_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)

static void split_svc_relay_event_from_central_callback(struct k_work *work);

K_MSGQ_DEFINE(relay_event_from_central_msgq, sizeof(struct zmk_split_relay_event_payload),
              CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_POSITION_QUEUE_SIZE, 4);
K_WORK_DEFINE(split_svc_relay_event_from_central_work, split_svc_relay_event_from_central_callback);

static void split_svc_relay_event_from_central_callback(struct k_work *work) {
    struct zmk_split_relay_event_payload payload;
    while (k_msgq_get(&relay_event_from_central_msgq, &payload, K_NO_WAIT) == 0) {
        struct zmk_relay_event_received relay_ev;
        relay_ev.source = 0; // Central is always source 0
        relay_ev.event_name = payload.event_type;
        relay_ev.event_data = payload.event_data;
        relay_ev.event_data_size = payload.header.event_data_size;
        raise_zmk_relay_event_received(relay_ev);
    }
}

static struct zmk_split_relay_event_payload last_relay_event_payload;

static ssize_t split_svc_relay_event_from_central(struct bt_conn *conn,
                                                  const struct bt_gatt_attr *attr, const void *buf,
                                                  uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset + len > sizeof(struct zmk_split_relay_event_payload) || len == 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    // TODO: offset support
    if (offset > 0) {
        LOG_ERR("Offset not supported for relay event characteristic");
    }
    struct relay_event_header *header = &last_relay_event_payload.header;
    memcpy(header, buf, sizeof(struct relay_event_header));

    if (header->event_type_size > CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN) {
        LOG_WRN("Event type name too long (%d), max is %d", header->event_type_size,
                CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN - 1);
        return len;
    }

    if (header->event_data_size > CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN) {
        LOG_WRN("Event data too large (%d), max is %d", header->event_data_size,
                CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN);
        return len;
    }

    if (sizeof(struct relay_event_header) + header->event_type_size + header->event_data_size !=
        len) {
        LOG_WRN("Malformed relay event: size mismatch (expected %d, got %d)",
                sizeof(struct relay_event_header) + header->event_type_size +
                    header->event_data_size,
                len);
        return len;
    }

    memcpy(last_relay_event_payload.event_type, buf + sizeof(struct relay_event_header),
           header->event_type_size);
    last_relay_event_payload.event_type[header->event_type_size] = '\0';
    memcpy(last_relay_event_payload.event_data,
           buf + sizeof(struct relay_event_header) + header->event_type_size,
           header->event_data_size);
    LOG_DBG("Received relay event: type='%s', data_len=%d (wire: %d bytes)",
            last_relay_event_payload.event_type, last_relay_event_payload.header.event_data_size,
            len);

    k_msgq_put(&relay_event_from_central_msgq, &last_relay_event_payload, K_NO_WAIT);
    k_work_submit(&split_svc_relay_event_from_central_work);

    return len;
}

static void split_svc_relay_event_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}

#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

static zmk_hid_indicators_t hid_indicators = 0;

static void split_svc_update_indicators_callback(struct k_work *work) {
    LOG_DBG("Raising HID indicators changed event: %x", hid_indicators);
    raise_zmk_hid_indicators_changed(
        (struct zmk_hid_indicators_changed){.indicators = hid_indicators});
}

static K_WORK_DEFINE(split_svc_update_indicators_work, split_svc_update_indicators_callback);

static ssize_t split_svc_update_indicators(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                           const void *buf, uint16_t len, uint16_t offset,
                                           uint8_t flags) {
    if (offset + len > sizeof(zmk_hid_indicators_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy((uint8_t *)&hid_indicators + offset, buf, len);

    k_work_submit(&split_svc_update_indicators_work);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

static uint8_t selected_phys_layout = 0;

static void split_svc_select_phys_layout_callback(struct k_work *work) {
    LOG_DBG("Selecting physical layout after GATT write of %d", selected_phys_layout);
    zmk_physical_layouts_select(selected_phys_layout);
}

static K_WORK_DEFINE(split_svc_select_phys_layout_work, split_svc_select_phys_layout_callback);

static ssize_t split_svc_select_phys_layout(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            const void *buf, uint16_t len, uint16_t offset,
                                            uint8_t flags) {
    if (offset + len > sizeof(uint8_t) || len == 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    selected_phys_layout = *(uint8_t *)buf;

    k_work_submit(&split_svc_select_phys_layout_work);

    return len;
}

static ssize_t split_svc_get_selected_phys_layout(struct bt_conn *conn,
                                                  const struct bt_gatt_attr *attrs, void *buf,
                                                  uint16_t len, uint16_t offset) {
    int selected_ret = zmk_physical_layouts_get_selected();
    if (selected_ret < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    uint8_t selected = (uint8_t)selected_ret;

    return bt_gatt_attr_read(conn, attrs, buf, len, offset, &selected, sizeof(selected));
}

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static void split_input_events_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}

// Duplicated from Zephyr, since it is internal there
struct gatt_cpf {
    uint8_t format;
    int8_t exponent;
    uint16_t unit;
    uint8_t name_space;
    uint16_t description;
} __packed;

ssize_t bt_gatt_attr_read_input_split_cpf(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset) {
    uint16_t reg = (uint16_t)(uint32_t)attr->user_data;
    struct gatt_cpf value;

    value.format = 0x1B; // Struct
    value.exponent = 0;
    value.unit = sys_cpu_to_le16(0x2700); // Unitless
    value.name_space = 0x01;              // Bluetooth SIG
    value.description = sys_cpu_to_le16(reg);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

#define INPUT_SPLIT_CHARS(node_id)                                                                 \
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID),                     \
                           BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),      \
        BT_GATT_CCC(split_input_events_ccc,                                                        \
                    BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),                       \
        BT_GATT_DESCRIPTOR(BT_UUID_GATT_CPF, BT_GATT_PERM_READ, bt_gatt_attr_read_input_split_cpf, \
                           NULL, (void *)DT_REG_ADDR(node_id)),

#endif

BT_GATT_SERVICE_DEFINE(
    split_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID),
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT,
                           split_svc_pos_state, NULL, &position_state),
    BT_GATT_CCC(split_svc_pos_state_ccc, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID),
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           split_svc_run_behavior, &behavior_run_payload),
    BT_GATT_DESCRIPTOR(BT_UUID_NUM_OF_DIGITALS, BT_GATT_PERM_READ, split_svc_num_of_positions, NULL,
                       &num_of_positions),
#if ZMK_KEYMAP_HAS_SENSORS
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_SENSOR_STATE_UUID),
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT,
                           split_svc_sensor_state, NULL, &last_sensor_event),
    BT_GATT_CCC(split_svc_sensor_state_ccc, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
#endif /* ZMK_KEYMAP_HAS_SENSORS */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_RELAY_EVENT_UUID),
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE_ENCRYPT, NULL, split_svc_relay_event_from_central,
                           NULL),
    BT_GATT_CCC(split_svc_relay_event_ccc, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
#endif
    DT_FOREACH_STATUS_OKAY(zmk_input_split, INPUT_SPLIT_CHARS)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_UPDATE_HID_INDICATORS_UUID),
                               BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                               split_svc_update_indicators, NULL),
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SELECT_PHYS_LAYOUT_UUID),
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                           BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT,
                           split_svc_get_selected_phys_layout, split_svc_select_phys_layout,
                           NULL), );

K_THREAD_STACK_DEFINE(service_q_stack, CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_STACK_SIZE);

struct k_work_q service_work_q;

K_MSGQ_DEFINE(position_state_msgq, sizeof(char[POS_STATE_LEN]),
              CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_POSITION_QUEUE_SIZE, 4);

#if IS_ENABLED(CONFIG_MK2_SPLIT_NOTIFY_STATS)

static uint32_t split_notify_ok;
static uint32_t split_notify_fail[5];
static int split_notify_last_err;
static uint32_t split_notify_evicted;

/* How long bt_gatt_notify() itself took, and when it last returned 0.
 *
 * ok above is the return value, not delivery, and the ATT notification path allocates its
 * PDU with K_FOREVER - so when the connection's TX buffers are exhausted the call does not
 * fail, it blocks. That produces a half that has stopped delivering while every error bucket
 * here stays at zero, which is exactly the readout that ended four sessions. A call that
 * takes seconds is that state, caught in the act.
 *
 * last_ok_ms answers the other half of it: if the operator reports dead keys and this is
 * minutes old, nothing was sent - the fault is upstream of here, in scanning. */
static uint32_t split_notify_last_ok_ms;
static uint32_t split_notify_call_max_us;
static uint32_t split_notify_call_max_at_ms;
/* Calls at or over this are the ones worth counting; normal ones are microseconds. */
#define MK2_NOTIFY_SLOW_US 150000
static uint32_t split_notify_slow_count;
static uint32_t split_notify_slow_last_ms;

static void mk2_notify_note_call(uint32_t start_cycles, int err) {
    uint32_t took_us = k_cyc_to_us_floor32(k_cycle_get_32() - start_cycles);
    uint32_t now = k_uptime_get_32();

    if (err == 0) {
        split_notify_last_ok_ms = now;
    }
    if (took_us > split_notify_call_max_us) {
        split_notify_call_max_us = took_us;
        split_notify_call_max_at_ms = now;
    }
    if (took_us >= MK2_NOTIFY_SLOW_US) {
        split_notify_slow_count++;
        split_notify_slow_last_ms = now;
    }
}

#if CONFIG_MK2_SPLIT_NOTIFY_STATS_SELFTEST > 0
static uint32_t split_notify_selftest_attempt;
#endif

static int split_notify_err_bucket(int err) {
    switch (err) {
    case -EPERM:
        return 0;
    case -ENOMEM:
        return 1;
    case -EAGAIN:
        return 2;
    case -ENOTCONN:
        return 3;
    default:
        return 4;
    }
}

/* DT_HAS_CHOSEN() alone is not enough: it is still true when the chosen console points at a
 * disabled UART, and DEVICE_DT_GET() on a disabled node fails to link (__device_dts_ord_N
 * undeclared). That is exactly how a previous peripheral USB-logging attempt broke the build,
 * so the node status is checked too. */
#if DT_HAS_CHOSEN(zephyr_console) && DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_console), okay) &&        \
    IS_ENABLED(CONFIG_UART_LINE_CTRL)

BUILD_ASSERT(POS_STATE_LEN == 16, "[NGUARD] pos= formatting expects a 16 byte position state");

static void split_notify_stats_dump_callback(struct k_work *work) {
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    if (device_is_ready(dev) && uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
        uint8_t state[POS_STATE_LEN];
        unsigned int key = irq_lock();

        memcpy(state, position_state, sizeof(state));
        irq_unlock(key);

        /* Split across two calls so neither spills a large argument list onto the stack. */
        printk("[NGUARD] ok=%u eperm=%u enomem=%u eagain=%u enotconn=%u other=%u "
               "last_err=%d evicted=%u up=%u\n",
               split_notify_ok, split_notify_fail[0], split_notify_fail[1], split_notify_fail[2],
               split_notify_fail[3], split_notify_fail[4], split_notify_last_err,
               split_notify_evicted, k_uptime_get_32());
        /* A third call for the same stack reason as the split above. */
        printk("[NGUARD] last_ok_ms=%u call_max_us=%u@%u slow_ge150ms=%u last_slow_ms=%u\n",
               split_notify_last_ok_ms, split_notify_call_max_us, split_notify_call_max_at_ms,
               split_notify_slow_count, split_notify_slow_last_ms);
        /* pos= is the peripheral's own view of which keys are held. If it shows a key down
         * that is physically released, the fault is in key scanning, not in the notify path. */
        printk("[NGUARD] pos=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
               state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7],
               state[8], state[9], state[10], state[11], state[12], state[13], state[14],
               state[15]);
        /* Key scanning on this half, one layer earlier than pos= above. pos= is what the
         * peripheral is willing to send; [KGUARD] held= is what its own scan queue actually
         * delivered. When they disagree the edge died between the two, and when they agree
         * but the fingers disagree with both, it died at or before the scan. */
        mk2_kscan_stats_dump();
    }

    /* Rescheduled onto the low priority queue, never service_work_q: that queue is the one
     * carrying the position notifications being measured, and a console write blocking on the
     * host would inject stalls into the very path under observation. */
    k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(), k_work_delayable_from_work(work),
                                K_MSEC(CONFIG_MK2_SPLIT_NOTIFY_STATS_DUMP_MS));
}

static K_WORK_DELAYABLE_DEFINE(split_notify_stats_dump_work, split_notify_stats_dump_callback);

#else

/* Fail loud: the counters would still tick here but nothing could ever read them, and an empty
 * readout reads exactly like "no failures". */
#warning "MK2_SPLIT_NOTIFY_STATS is on but no DTR-capable console is chosen: the [NGUARD] counters cannot be read out"

#endif /* chosen console is present and okay && CONFIG_UART_LINE_CTRL */

#if CONFIG_MK2_SPLIT_NOTIFY_STATS_SELFTEST > 0
#warning "MK2_SPLIT_NOTIFY_STATS_SELFTEST is a fault injection build that can stick keys - bench only, do not ship"
#endif

#endif /* CONFIG_MK2_SPLIT_NOTIFY_STATS */

void send_position_state_callback(struct k_work *work) {
    uint8_t state[POS_STATE_LEN];

    while (k_msgq_get(&position_state_msgq, &state, K_NO_WAIT) == 0) {
#if IS_ENABLED(CONFIG_MK2_SPLIT_NOTIFY_STATS)
        int err;

        uint32_t mk2_notify_start = k_cycle_get_32();

#if CONFIG_MK2_SPLIT_NOTIFY_STATS_SELFTEST > 0
        split_notify_selftest_attempt++;
        if (split_notify_selftest_attempt % CONFIG_MK2_SPLIT_NOTIFY_STATS_SELFTEST == 0) {
            err = -ENOMEM;
        } else {
            err = bt_gatt_notify(NULL, &split_svc.attrs[1], &state, sizeof(state));
        }
#else
        err = bt_gatt_notify(NULL, &split_svc.attrs[1], &state, sizeof(state));
#endif
        mk2_notify_note_call(mk2_notify_start, err);

        if (err == 0) {
            split_notify_ok++;
        } else {
            split_notify_fail[split_notify_err_bucket(err)]++;
            split_notify_last_err = err;
        }
#else
        int err = bt_gatt_notify(NULL, &split_svc.attrs[1], &state, sizeof(state));
#endif
        if (err) {
            LOG_DBG("Error notifying %d", err);
        }
    }
};

K_WORK_DEFINE(service_position_notify_work, send_position_state_callback);

int send_position_state() {
    int err = k_msgq_put(&position_state_msgq, position_state, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Position state message queue full, popping first message and queueing again");
            uint8_t discarded_state[POS_STATE_LEN];
            k_msgq_get(&position_state_msgq, &discarded_state, K_NO_WAIT);
#if IS_ENABLED(CONFIG_MK2_SPLIT_NOTIFY_STATS)
            /* Unlike the other counters this one is bumped from the calling (key event)
             * thread, not from service_work_q. It stays a single producer and a 32 bit
             * aligned store is indivisible on Cortex-M, so no lock is needed. */
            split_notify_evicted++;
#endif
            return send_position_state();
        }
        default:
            LOG_WRN("Failed to queue position state to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&service_work_q, &service_position_notify_work);

    return 0;
}

static int zmk_split_bt_position_pressed(uint8_t position) {
    WRITE_BIT(position_state[position / 8], position % 8, true);
    return send_position_state();
}

static int zmk_split_bt_position_released(uint8_t position) {
    WRITE_BIT(position_state[position / 8], position % 8, false);
    return send_position_state();
}

#if ZMK_KEYMAP_HAS_SENSORS
K_MSGQ_DEFINE(sensor_state_msgq, sizeof(struct sensor_event),
              CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_POSITION_QUEUE_SIZE, 4);

void send_sensor_state_callback(struct k_work *work) {
    while (k_msgq_get(&sensor_state_msgq, &last_sensor_event, K_NO_WAIT) == 0) {
        int err = bt_gatt_notify(NULL, &split_svc.attrs[8], &last_sensor_event,
                                 sizeof(last_sensor_event));
        if (err) {
            LOG_DBG("Error notifying %d", err);
        }
    }
};

K_WORK_DEFINE(service_sensor_notify_work, send_sensor_state_callback);

int send_sensor_state(struct sensor_event ev) {
    int err = k_msgq_put(&sensor_state_msgq, &ev, K_MSEC(100));
    if (err) {
        // retry...
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Sensor state message queue full, popping first message and queueing again");
            struct sensor_event discarded_state;
            k_msgq_get(&sensor_state_msgq, &discarded_state, K_NO_WAIT);
            return send_sensor_state(ev);
        }
        default:
            LOG_WRN("Failed to queue sensor state to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&service_work_q, &service_sensor_notify_work);
    return 0;
}

static int zmk_split_bt_sensor_triggered(uint8_t sensor_index,
                                         const struct zmk_sensor_channel_data channel_data[],
                                         size_t channel_data_size) {
    if (channel_data_size > ZMK_SENSOR_EVENT_MAX_CHANNELS) {
        return -EINVAL;
    }

    struct sensor_event ev =
        (struct sensor_event){.sensor_index = sensor_index, .channel_data_size = channel_data_size};
    memcpy(ev.channel_data, channel_data,
           channel_data_size * sizeof(struct zmk_sensor_channel_data));
    return send_sensor_state(ev);
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)

static struct zmk_split_relay_event_payload last_relay_event;
static uint8_t relay_event_buffer[sizeof(struct relay_event_header) +
                                  CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN +
                                  CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN];
K_MSGQ_DEFINE(relay_event_msgq, sizeof(struct zmk_split_relay_event_payload),
              CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_POSITION_QUEUE_SIZE, 4);

void send_relay_event_callback(struct k_work *work) {
    while (k_msgq_get(&relay_event_msgq, &last_relay_event, K_NO_WAIT) == 0) {
        // Find the relay event characteristic attribute index
        size_t relay_attr_index = 0;
        for (size_t i = 0; i < split_svc.attr_count; i++) {
            if (bt_uuid_cmp(split_svc.attrs[i].uuid,
                            BT_UUID_DECLARE_128(ZMK_SPLIT_BT_RELAY_EVENT_UUID)) == 0) {
                relay_attr_index = i;
                break;
            }
        }

        if (relay_attr_index > 0) {
            // pack to relay_event_buffer
            size_t total_size = 0;
            memcpy(relay_event_buffer, &last_relay_event.header, sizeof(struct relay_event_header));
            total_size += sizeof(struct relay_event_header);
            memcpy(relay_event_buffer + total_size, last_relay_event.event_type,
                   last_relay_event.header.event_type_size);
            total_size += last_relay_event.header.event_type_size;
            memcpy(relay_event_buffer + total_size, last_relay_event.event_data,
                   last_relay_event.header.event_data_size);
            total_size += last_relay_event.header.event_data_size;
            LOG_DBG("Prepared relay event of total size %d bytes (header %d, name %d, data %d)",
                    total_size, sizeof(struct relay_event_header),
                    last_relay_event.header.event_type_size,
                    last_relay_event.header.event_data_size);
            // Send only the actual bytes needed
            int err = bt_gatt_notify(NULL, &split_svc.attrs[relay_attr_index], relay_event_buffer,
                                     total_size);
            if (err) {
                LOG_WRN("Error notifying relay event %d", err);
            } else {
                LOG_DBG("Notified relay event of type %s", last_relay_event.event_type);
            }
        } else {
            LOG_WRN("Could not find relay event characteristic to send relay event");
        }
    }
};

K_WORK_DEFINE(service_relay_event_notify_work, send_relay_event_callback);

int zmk_split_bt_send_relay_event(const struct zmk_split_relay_event_payload *ev) {
    int err = k_msgq_put(&relay_event_msgq, ev, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Relay event message queue full, popping first message and queueing again");
            struct zmk_split_relay_event_payload discarded_ev;
            k_msgq_get(&relay_event_msgq, &discarded_ev, K_NO_WAIT);
            return zmk_split_bt_send_relay_event(ev);
        }
        default:
            LOG_WRN("Failed to queue relay event to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&service_work_q, &service_relay_event_notify_work);
    return 0;
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT) */

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static int zmk_split_bt_report_input(uint8_t reg, uint8_t type, uint16_t code, int32_t value,
                                     bool sync) {

    for (size_t i = 0; i < split_svc.attr_count; i++) {
        if (bt_uuid_cmp(split_svc.attrs[i].uuid,
                        BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID)) == 0 &&
            (uint8_t)(uint32_t)split_svc.attrs[i + 2].user_data == reg) {
            struct zmk_split_input_event_payload payload = {
                .type = type,
                .code = code,
                .value = value,
                .sync = sync ? 1 : 0,
            };

            return bt_gatt_notify(NULL, &split_svc.attrs[i], &payload, sizeof(payload));
        }
    }
    return -ENODEV;
}

#endif /* IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT) */

static int service_init(void) {
    static const struct k_work_queue_config queue_config = {
        .name = "Split Peripheral Notification Queue"};
    k_work_queue_start(&service_work_q, service_q_stack, K_THREAD_STACK_SIZEOF(service_q_stack),
                       CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_PRIORITY, &queue_config);

#if IS_ENABLED(CONFIG_MK2_SPLIT_NOTIFY_STATS) && DT_HAS_CHOSEN(zephyr_console) &&                  \
    DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_console), okay) && IS_ENABLED(CONFIG_UART_LINE_CTRL)
    k_work_schedule_for_queue(zmk_workqueue_lowprio_work_q(), &split_notify_stats_dump_work,
                              K_MSEC(CONFIG_MK2_SPLIT_NOTIFY_STATS_DUMP_MS));
#endif

    return 0;
}

SYS_INIT(service_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);

int zmk_split_transport_peripheral_bt_report_event(
    const struct zmk_split_transport_peripheral_event *ev) {
    switch (ev->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        if (ev->data.key_position_event.pressed) {
            zmk_split_bt_position_pressed(ev->data.key_position_event.position);
        } else {
            zmk_split_bt_position_released(ev->data.key_position_event.position);
        }
        break;
#if ZMK_KEYMAP_HAS_SENSORS
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        zmk_split_bt_sensor_triggered(ev->data.sensor_event.sensor_index,
                                      &ev->data.sensor_event.channel_data, 1);

        break;
#endif

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return zmk_split_bt_report_input(ev->data.input_event.reg, ev->data.input_event.type,
                                         ev->data.input_event.code, ev->data.input_event.value,
                                         ev->data.input_event.sync);
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        // The BLE transport uses standard BAS service for propagation, so just return success
        // here.
        return 0;
#endif
#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_RELAY_EVENT:
        struct zmk_split_relay_event_payload relay_ev = {
            .header = ev->data.relay_event.header,
        };
        relay_ev.header.event_type_size =
            strnlen(ev->data.relay_event.event_type, CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN);
        memcpy(relay_ev.event_type, ev->data.relay_event.event_type,
               relay_ev.header.event_type_size);
        memcpy(relay_ev.event_data, ev->data.relay_event.event_data,
               ev->data.relay_event.header.event_data_size);
        return zmk_split_bt_send_relay_event(&relay_ev);
#endif
    default:
        LOG_WRN("Unhandled event type %d", ev->type);
        return -ENOTSUP;
    }

    return 0;
}

static ssize_t split_svc_run_behavior(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags) {
    struct zmk_split_run_behavior_payload *payload = attrs->user_data;
    uint16_t end_addr = offset + len;

    LOG_DBG("offset %d len %d", offset, len);

    if (end_addr > sizeof(struct zmk_split_run_behavior_payload)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(payload + offset, buf, len);

    // We run if:
    // 1: We've gotten all the position/state/param data.
    // 2: We have a null terminated string for the behavior device label.
    const size_t behavior_dev_offset =
        offsetof(struct zmk_split_run_behavior_payload, behavior_dev);
    if ((end_addr > sizeof(struct zmk_split_run_behavior_data)) &&
        payload->behavior_dev[end_addr - behavior_dev_offset - 1] == '\0') {

        struct zmk_split_transport_central_command cmd = {
            .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR,
            .data = {.invoke_behavior = {
                         .param1 = payload->data.param1,
                         .param2 = payload->data.param2,
                         .position = payload->data.position,
                         .state = payload->data.state,
                     }}};

        const size_t payload_dev_size = sizeof(cmd.data.invoke_behavior.behavior_dev);
        if (strlcpy(cmd.data.invoke_behavior.behavior_dev, payload->behavior_dev,
                    payload_dev_size) >= payload_dev_size) {
            LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                    payload->behavior_dev, cmd.data.invoke_behavior.behavior_dev);
        }

        LOG_DBG("%s with params %d %d: pressed? %d", cmd.data.invoke_behavior.behavior_dev,
                cmd.data.invoke_behavior.param1, cmd.data.invoke_behavior.param2,
                cmd.data.invoke_behavior.state);

        int err = zmk_split_transport_peripheral_command_handler(
            zmk_split_transport_peripheral_bt(), cmd);

        if (err) {
            LOG_ERR("Failed to invoke behavior %s: %d", payload->behavior_dev, err);
        }
    }

    return len;
}
