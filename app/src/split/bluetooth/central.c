/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/ble.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/split/bluetooth/service.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/mk2_split_stats.h>
#include <zmk/physical_layouts.h>

static int start_scanning(void);

#define POSITION_STATE_DATA_LEN 16

enum peripheral_slot_state {
    PERIPHERAL_SLOT_STATE_OPEN,
    PERIPHERAL_SLOT_STATE_CONNECTING,
    PERIPHERAL_SLOT_STATE_CONNECTED,
};

struct peripheral_slot {
    enum peripheral_slot_state state;
    struct bt_conn *conn;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params subscribe_params;
    struct bt_gatt_discover_params sub_discover_params;
    struct bt_gatt_subscribe_params sensor_subscribe_params;
    struct bt_gatt_discover_params sensor_sub_discover_params;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
    struct bt_gatt_subscribe_params relay_event_subscribe_params;
    struct bt_gatt_discover_params relay_event_sub_discover_params;
#endif
    uint16_t run_behavior_handle;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    struct bt_gatt_subscribe_params batt_lvl_subscribe_params;
    struct bt_gatt_discover_params batt_lvl_sub_discover_params;
    struct bt_gatt_read_params batt_lvl_read_params;
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    uint16_t update_hid_indicators;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    uint16_t selected_physical_layout_handle;
    uint8_t position_state[POSITION_STATE_DATA_LEN];
    uint8_t changed_positions[POSITION_STATE_DATA_LEN];
};

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static const struct bt_uuid *gatt_ccc_uuid = BT_UUID_GATT_CCC;
static const struct bt_uuid *gatt_cpf_uuid = BT_UUID_GATT_CPF;

struct peripheral_input_slot {
    struct bt_conn *conn;
    struct bt_gatt_subscribe_params sub;
    uint8_t reg;
};

#define COUNT_INPUT_SPLIT(n) +1

static struct peripheral_input_slot
    peripheral_input_slots[(0 DT_FOREACH_STATUS_OKAY(zmk_input_split, COUNT_INPUT_SPLIT))];

static bool input_slot_is_open(size_t i) {
    return i < ARRAY_SIZE(peripheral_input_slots) && peripheral_input_slots[i].conn == NULL;
}

static bool input_slot_is_pending(size_t i) {
    return i < ARRAY_SIZE(peripheral_input_slots) && peripheral_input_slots[i].conn != NULL &&
           (!peripheral_input_slots[i].sub.value_handle ||
            !peripheral_input_slots[i].sub.ccc_handle || !peripheral_input_slots[i].reg);
}

static int reserve_next_open_input_slot(struct peripheral_input_slot **slot, struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (input_slot_is_open(i)) {
            peripheral_input_slots[i].conn = conn;

            // Clear out any previously set values
            peripheral_input_slots[i].sub.value_handle = 0;
            peripheral_input_slots[i].sub.ccc_handle = 0;
            peripheral_input_slots[i].reg = 0;
            *slot = &peripheral_input_slots[i];
            return i;
        }
    }

    return -ENOMEM;
}

static int find_pending_input_slot(struct peripheral_input_slot **slot, struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (peripheral_input_slots[i].conn == conn && input_slot_is_pending(i)) {
            *slot = &peripheral_input_slots[i];
            return i;
        }
    }

    return -ENODEV;
}

void release_peripheral_input_subs(struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (peripheral_input_slots[i].conn == conn) {
            peripheral_input_slots[i].conn = NULL;
            // memset(&peripheral_input_slots[i], 0, sizeof(struct peripheral_input_slot));
        }
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled;

#if IS_ENABLED(CONFIG_MK2_SPLIT_POS_DROP_STATS)

#include <errno.h>

#if CONFIG_MK2_SPLIT_POS_DROP_SELFTEST > 0
#warning "MK2_SPLIT_POS_DROP_SELFTEST is a fault injection build that can stick keys - bench only, do not ship"
#endif

#define MK2_SPLIT_LOSS_RING 8

enum mk2_split_loss_kind {
    /* Nothing could be queued, but the XOR state was rolled back so the next snapshot
     * from the peripheral regenerates the edge. */
    MK2_SPLIT_LOSS_ROLLED_BACK,
    /* Lost during slot release. The state is zeroed immediately afterwards, so unlike the
     * case above there is nothing left to regenerate it from. */
    MK2_SPLIT_LOSS_ON_DISCONNECT,
};

struct mk2_split_loss_rec {
    uint32_t ms;
    /* How long the consumer had been idle when this edge was lost. Large means the system
     * work queue was stalled, small means a genuine burst arrived. */
    uint32_t idle_ms;
    uint32_t position;
    int16_t err;
    uint8_t pressed;
    uint8_t kind;
};

static struct mk2_split_loss_rec split_pos_loss_ring[MK2_SPLIT_LOSS_RING];
static uint32_t split_pos_loss_next;
static uint32_t split_pos_drop_count;
/* Sensor, input, relay and battery events share this queue with key edges. Losing one
 * cannot strand a key, so they are not rolled back - but until now their loss was not
 * counted either, which meant the queue could be overflowing while the drop counter read
 * zero. A zero that means "nothing was measured" is the failure mode this whole exercise
 * keeps running into. */
static atomic_t split_other_drop_count;
/* Kept apart from the queue counter above on purpose. A short notification and a full queue
 * are different faults with different fixes, and a single number that can mean either is a
 * number nobody can act on. */
static atomic_t split_short_notify_count;
static uint32_t split_pos_drop_max_q;
static uint32_t split_pos_drain_last_ms;

#if CONFIG_MK2_SPLIT_POS_DROP_SELFTEST > 0
static uint32_t split_pos_drop_selftest_attempt;
#endif

static void mk2_split_pos_note_loss(uint32_t position, bool pressed, int err,
                                    enum mk2_split_loss_kind kind) {
    uint32_t now = k_uptime_get_32();
    /* Written from the BT RX context and read from a work queue, so a record can otherwise be
     * printed half-updated - and a garbled record is worst exactly when it finally matters. */
    unsigned int key = irq_lock();
    struct mk2_split_loss_rec *rec = &split_pos_loss_ring[split_pos_loss_next % MK2_SPLIT_LOSS_RING];

    rec->ms = now;
    rec->idle_ms = (split_pos_drain_last_ms == 0) ? 0 : (now - split_pos_drain_last_ms);
    rec->position = position;
    rec->err = (int16_t)err;
    rec->pressed = pressed ? 1 : 0;
    rec->kind = (uint8_t)kind;
    split_pos_loss_next++;
    split_pos_drop_count++;
    irq_unlock(key);
}

/* Only a timestamp now. The gap between drains used to be recorded as if it meant something,
 * but this consumer runs only when there is work, so the largest gap it can ever see is the
 * longest the operator went without typing - eight hours of sleep, on the readout that
 * prompted this. The number that does mean something is how long an event WAITED, measured
 * below from the stamp each one carries. */
static void mk2_split_pos_note_drain(void) { split_pos_drain_last_ms = k_uptime_get_32(); }

/* Longest an arrived key event sat before the consumer reached it. Single-digit milliseconds
 * in normal use; seconds is typing that was dead for that long. */
static uint32_t split_pos_max_wait_ms;
static uint32_t split_pos_max_wait_at_ms;

/* The maximum on its own cannot be matched to a report.
 *
 * A 4,795 ms stall measured one evening keeps the pair above for the rest of the boot, so a
 * five second stall the owner reports the next morning leaves no trace whatsoever: both
 * numbers are unchanged and the readout is indistinguishable from a keyboard that never
 * stalled at all. That happened, and it cost a night of measurement. Keep the recent long
 * waits as well, each with the moment it was taken, so a time the owner writes down can be
 * matched against them. */
#define MK2_SPLIT_WAIT_RING 8
/* Below this a wait is ordinary operation - single digit milliseconds is what a system work
 * queue that is keeping up looks like - and recording those would push the interesting
 * entries out of the ring within a second of typing. */
#define MK2_SPLIT_WAIT_MIN_MS 150

struct mk2_split_wait_rec {
    uint32_t at_ms;
    uint32_t waited_ms;
    uint8_t type;
    uint8_t position;
    uint8_t pressed;
};

static struct mk2_split_wait_rec split_pos_wait_ring[MK2_SPLIT_WAIT_RING];
/* Doubles as the total count of long waits. The ring holds eight, so without it there is no
 * way to tell "eight happened" from "eight kept out of ninety". */
static uint32_t split_pos_wait_next;

static void mk2_split_pos_note_wait(uint32_t queued_ms,
                                    const struct zmk_split_transport_peripheral_event *event) {
    uint32_t now = k_uptime_get_32();
    uint32_t waited = now - queued_ms;

    if (waited > split_pos_max_wait_ms) {
        split_pos_max_wait_ms = waited;
        split_pos_max_wait_at_ms = now;
    }

    if (waited < MK2_SPLIT_WAIT_MIN_MS) {
        return;
    }

    /* Same reason as the loss ring: written from the system work queue and read from the low
     * priority one, so a record can otherwise be printed half updated - and a garbled record
     * is worst exactly when it finally matters. */
    unsigned int key = irq_lock();
    struct mk2_split_wait_rec *rec =
        &split_pos_wait_ring[split_pos_wait_next % MK2_SPLIT_WAIT_RING];

    rec->at_ms = now;
    rec->waited_ms = waited;
    rec->type = (uint8_t)event->type;
    /* The payload is a union, so position and pressed hold anything only for a key edge.
     * Zeroed otherwise rather than copied across, because a plausible looking position read
     * out of a battery event is worse than no position at all. */
    if (event->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT) {
        rec->position = event->data.key_position_event.position;
        rec->pressed = event->data.key_position_event.pressed ? 1 : 0;
    } else {
        rec->position = 0;
        rec->pressed = 0;
    }
    split_pos_wait_next++;
    irq_unlock(key);
}

/* Notifications actually received from the peripheral, counted at the first instruction of
 * the subscribe callback.
 *
 * Every other counter on this half sits downstream of that callback, and the peripheral's
 * [NGUARD] ok is the return value of bt_gatt_notify() - not delivery. So "the left half sent
 * and nothing arrived" and "it arrived and something dropped it later" produced identical
 * readouts: all clean, on both halves. Four separate sessions ended there.
 *
 * rx is directly comparable to the peripheral's ok: one notify call should produce one
 * receive. A divergence between the two is delivery loss, and it can be read at any time
 * from the two cumulative numbers - the symptom does not have to be caught in the act,
 * which is what made the previous three attempts fail.
 *
 * unsub counts the callback being invoked with data == NULL. That path sets value_handle to
 * 0 and returns STOP, and nothing re-subscribes; the ACL stays up, so [SPLINK] disc never
 * moves. It is the one way the link can go quiet without a single counter noticing. */
static uint32_t split_rx_count;
static uint32_t split_rx_last_ms;
static uint32_t split_rx_unsub_count;
static uint32_t split_rx_unsub_last_ms;
/* The callback reached with no slot for the connection. Counted apart because it means the
 * notification arrived and was discarded here, which is a different fault from not arriving. */
static uint32_t split_rx_no_slot_count;

static void mk2_split_note_rx(void) {
    split_rx_count++;
    split_rx_last_ms = k_uptime_get_32();
}

static void mk2_split_note_rx_unsub(void) {
    split_rx_unsub_count++;
    split_rx_unsub_last_ms = k_uptime_get_32();
}

static void mk2_split_note_rx_no_slot(void) { split_rx_no_slot_count++; }

/* System work queue latency probe.
 *
 * Everything measured so far only sees a stall when a key happens to be in flight: the split
 * queue's wait is the gap between an event arriving and the consumer reaching it, so a stall
 * while nobody is typing leaves no trace, and a stall during typing is confounded with how
 * much was being typed. Thirty-six hours of measurement produced 26 samples that way.
 *
 * This submits one trivial item every 250 ms and records how long it waited before running.
 * The consumer of key events (peripheral_event_work) sits on the same queue, so this delay is
 * the delay those events would have seen - sampled four times a second whether or not anyone
 * is at the keyboard.
 *
 * The timer deliberately does NOT resubmit while the previous item is still pending. Doing so
 * would overwrite the submit timestamp every 250 ms and cap every measurement at one period,
 * turning a seven second stall into a string of 250 ms ones - the instrument would report
 * that nothing was wrong precisely when something was. */
#define MK2_SYSWQ_PROBE_MS 250
/* Below this the queue is keeping up; recording those would push the interesting entries out
 * of the ring within seconds. */
#define MK2_SYSWQ_STALL_MS 150
#define MK2_SYSWQ_RING 8

static uint32_t syswq_probe_submit_ms;
static uint32_t syswq_probe_runs;
static uint32_t syswq_probe_max_ms;
static uint32_t syswq_probe_max_at_ms;
static uint32_t syswq_stall_count;

struct mk2_syswq_stall_rec {
    uint32_t at_ms;
    uint32_t delay_ms;
};

static struct mk2_syswq_stall_rec syswq_stall_ring[MK2_SYSWQ_RING];
static uint32_t syswq_stall_next;

static void mk2_syswq_probe_handler(struct k_work *work) {
    uint32_t now = k_uptime_get_32();
    uint32_t delay = now - syswq_probe_submit_ms;

    syswq_probe_runs++;

    if (delay > syswq_probe_max_ms) {
        syswq_probe_max_ms = delay;
        syswq_probe_max_at_ms = now;
    }

    if (delay < MK2_SYSWQ_STALL_MS) {
        return;
    }

    /* Written here and read from the low priority dump, same as the other rings. */
    unsigned int key = irq_lock();
    struct mk2_syswq_stall_rec *rec = &syswq_stall_ring[syswq_stall_next % MK2_SYSWQ_RING];

    rec->at_ms = now;
    rec->delay_ms = delay;
    syswq_stall_next++;
    syswq_stall_count++;
    irq_unlock(key);
}

static K_WORK_DEFINE(mk2_syswq_probe_work, mk2_syswq_probe_handler);

static void mk2_syswq_probe_timer_fn(struct k_timer *timer) {
    /* Runs in interrupt context; both calls below are allowed there. */
    if (k_work_is_pending(&mk2_syswq_probe_work)) {
        /* Still stuck from the last submit. Leaving the timestamp alone is the whole point -
         * see the comment above. */
        return;
    }

    syswq_probe_submit_ms = k_uptime_get_32();
    k_work_submit(&mk2_syswq_probe_work);
}

static K_TIMER_DEFINE(mk2_syswq_probe_timer, mk2_syswq_probe_timer_fn, NULL);

static int mk2_syswq_probe_init(void) {
    k_timer_start(&mk2_syswq_probe_timer, K_MSEC(MK2_SYSWQ_PROBE_MS), K_MSEC(MK2_SYSWQ_PROBE_MS));
    return 0;
}

SYS_INIT(mk2_syswq_probe_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* atomic because the producers sit in different contexts - the BT RX path and the system
 * work queue both reach here. A plain ++ can lose an update, and a counter whose zero cannot
 * be trusted is the failure this whole exercise keeps circling back to. */
static void mk2_split_note_other_drop(void) { atomic_inc(&split_other_drop_count); }

static void mk2_split_note_short_notify(void) { atomic_inc(&split_short_notify_count); }

static void mk2_split_pos_note_queue(uint32_t used) {
    /* Sampled on every event, not just on failure: after a failed put the queue is by
     * definition full, so sampling only there would pin max_q to the queue size and tell us
     * nothing about the normal headroom. */
    if (used > split_pos_drop_max_q) {
        split_pos_drop_max_q = used;
    }
}

void mk2_split_pos_drop_dump(void) {
    /* now_ms closes the measurement window: without it a drop count cannot be told
     * apart from a stale one, and no rate can be derived. */
    printk("[MK2_DIAG] split_pos_drop=%u,other_drop=%u,short_notify=%u,max_q=%u/%u,"
           "max_wait_ms=%u@%u,wait_ge%u=%u,since=boot,now_ms=%u\n",
           split_pos_drop_count, (uint32_t)atomic_get(&split_other_drop_count),
           (uint32_t)atomic_get(&split_short_notify_count), split_pos_drop_max_q,
           (uint32_t)CONFIG_ZMK_SPLIT_BLE_CENTRAL_POSITION_QUEUE_SIZE, split_pos_max_wait_ms,
           split_pos_max_wait_at_ms, (uint32_t)MK2_SPLIT_WAIT_MIN_MS, split_pos_wait_next,
           k_uptime_get_32());

    /* The probe measures the same queue the key consumer runs on, but four times a second
     * regardless of typing - so a zero here means the queue really was keeping up, not that
     * nobody happened to press a key while it was stuck. */
    printk("[MK2_DIAG] syswq_runs=%u,max_ms=%u@%u,stall_ge%u=%u,since=boot,now_ms=%u\n",
           syswq_probe_runs, syswq_probe_max_ms, syswq_probe_max_at_ms,
           (uint32_t)MK2_SYSWQ_STALL_MS, syswq_stall_count, k_uptime_get_32());

    uint32_t stalls = MIN(syswq_stall_next, (uint32_t)MK2_SYSWQ_RING);

    for (uint32_t i = 0; i < stalls; i++) {
        const struct mk2_syswq_stall_rec *rec =
            &syswq_stall_ring[(syswq_stall_next - 1 - i) % MK2_SYSWQ_RING];

        printk("[MK2_DIAG] syswq_stall[%u] %ums at=%u\n", i, rec->delay_ms, rec->at_ms);
    }

    /* Printed next to the queue numbers on purpose: rx is what arrived, and everything else
     * on this line is what happened to it afterwards. Compare rx against the peripheral's
     * [NGUARD] ok - they count the same events at opposite ends of the link. */
    printk("[MK2_DIAG] split_rx=%u,last_rx_ms=%u,unsub=%u,last_unsub_ms=%u,no_slot=%u,"
           "since=boot,now_ms=%u\n",
           split_rx_count, split_rx_last_ms, split_rx_unsub_count, split_rx_unsub_last_ms,
           split_rx_no_slot_count, k_uptime_get_32());

    uint32_t shown = MIN(split_pos_loss_next, (uint32_t)MK2_SPLIT_LOSS_RING);

    for (uint32_t i = 0; i < shown; i++) {
        const struct mk2_split_loss_rec *rec =
            &split_pos_loss_ring[(split_pos_loss_next - 1 - i) % MK2_SPLIT_LOSS_RING];
        /* rolled_back means the next snapshot can regenerate the edge; on_disconnect means it
         * cannot, because position_state is zeroed immediately afterwards. */
        static const char *const kind_name[] = {"rolled_back", "on_disconnect"};

        /* RELEASE is shouted because that is the case that sticks a key. */
        printk("[MK2_DIAG] split_lost[%u] pos=%u %s %s err=%d ms=%u idle_ms=%u\n", i,
               rec->position, rec->pressed ? "press" : "RELEASE",
               kind_name[rec->kind <= MK2_SPLIT_LOSS_ON_DISCONNECT ? rec->kind : 0], rec->err,
               rec->ms, rec->idle_ms);
    }

    /* Newest first, same order as the losses above, so a time the owner reports is found by
     * reading down from the top rather than by hunting through the ring. */
    uint32_t waits = MIN(split_pos_wait_next, (uint32_t)MK2_SPLIT_WAIT_RING);

    for (uint32_t i = 0; i < waits; i++) {
        const struct mk2_split_wait_rec *rec =
            &split_pos_wait_ring[(split_pos_wait_next - 1 - i) % MK2_SPLIT_WAIT_RING];

        if (rec->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT) {
            printk("[MK2_DIAG] split_wait[%u] %ums at=%u pos=%u %s\n", i, rec->waited_ms,
                   rec->at_ms, rec->position, rec->pressed ? "press" : "RELEASE");
        } else {
            printk("[MK2_DIAG] split_wait[%u] %ums at=%u type=%u\n", i, rec->waited_ms, rec->at_ms,
                   rec->type);
        }
    }
}

/* Deliberately does nothing.
 *
 * These counters used to be cleared from mk2_ble_diag_reset() so they would share a window
 * with the BLE pipe counters. That cost more than it bought: the events being counted here
 * are rare and can predate any window the operator happens to open, and a zero from a
 * cleared counter is indistinguishable from a zero that means "this never happened". The
 * whole [SPLINK] instrument was read as broken for a day on exactly that confusion. The
 * dump prints since=boot so the reader is never left guessing which it is. */
void mk2_split_pos_drop_reset(void) {}

#else /* !CONFIG_MK2_SPLIT_POS_DROP_STATS */

static inline void mk2_split_pos_note_loss(uint32_t position, bool pressed, int err, int kind) {}
static inline void mk2_split_pos_note_drain(void) {}
static inline void
mk2_split_pos_note_wait(uint32_t queued_ms,
                        const struct zmk_split_transport_peripheral_event *event) {}
static inline void mk2_split_pos_note_queue(uint32_t used) {}
static inline void mk2_split_note_rx(void) {}
static inline void mk2_split_note_rx_unsub(void) {}
static inline void mk2_split_note_rx_no_slot(void) {}
static inline void mk2_split_note_other_drop(void) {}
static inline void mk2_split_note_short_notify(void) {}

#define MK2_SPLIT_LOSS_ROLLED_BACK 0
#define MK2_SPLIT_LOSS_ON_DISCONNECT 1

#endif /* CONFIG_MK2_SPLIT_POS_DROP_STATS */

#if IS_ENABLED(CONFIG_MK2_SPLIT_LINK_STATS)

/* The left half drops its link to the central and comes back about three seconds later,
 * on some host machines and not others. Three seconds means the link is being torn down
 * and re-established, not that the peripheral hung or rebooted - a reboot would take far
 * longer to reappear. What is missing is why it was torn down.
 *
 * A printk at the moment of disconnect (the July attempt) only helps if a host happens to
 * be attached to the console right then, which is why that attempt captured nothing. These
 * counters keep the answer until somebody asks for it, so the cable can be plugged in
 * afterwards - and the same readout works on a customer's keyboard, where nobody is going
 * to sit watching a serial console.
 *
 * reason is the HCI error code from the controller. 0x08 is a supervision timeout, meaning
 * the halves stopped hearing each other; 0x13/0x16 mean one side asked to close the link;
 * 0x3e means a connection attempt failed to complete. Which of those it is decides whether
 * this is a radio problem or a protocol one, and they call for opposite fixes. */
static uint32_t split_link_disc_count;
static uint32_t split_link_conn_count;
static uint8_t split_link_last_reason;
static uint32_t split_link_last_disc_ms;
static uint32_t split_link_last_conn_ms;
/* Shortest observed gap between a disconnect and the reconnect that followed it. A drop
 * nobody notices and one that interrupts typing look identical in a count; the gap is what
 * separates them. */
static uint32_t split_link_min_gap_ms;

/* Answered from the peripheral slot table, not from the connect callback, so the readout does
 * not depend on the very callback whose wiring is in question. A day went into reading
 * "conn=0" as "the link never came up" when the truth was that the counter had never been
 * reached at all; linked= is the independent axis that tells those two apart at a glance. */
static uint32_t mk2_split_link_connected_slots(void);

void mk2_split_link_dump(void) {
    printk("[SPLINK] linked=%u disc=%u conn=%u last_reason=0x%02x last_disc_ms=%u "
           "last_conn_ms=%u min_gap_ms=%u since=boot now_ms=%u\n",
           mk2_split_link_connected_slots(), split_link_disc_count, split_link_conn_count,
           split_link_last_reason, split_link_last_disc_ms, split_link_last_conn_ms,
           split_link_min_gap_ms, k_uptime_get_32());
}

/* Deliberately does nothing - see mk2_split_pos_drop_reset(). A link comes up once at boot
 * and stays up, so clearing this on DLOG_CLEAR guaranteed a zero no matter what the link had
 * actually done. */
void mk2_split_link_reset(void) {}

static void mk2_split_link_note_disconnect(uint8_t reason) {
    split_link_disc_count++;
    split_link_last_reason = reason;
    split_link_last_disc_ms = k_uptime_get_32();
}

static void mk2_split_link_note_connect(void) {
    uint32_t now = k_uptime_get_32();

    split_link_conn_count++;
    split_link_last_conn_ms = now;

    /* Only a reconnect that follows a disconnect measures an outage; the very first
     * connect after boot has nothing before it to measure against. */
    if (split_link_last_disc_ms != 0 && now > split_link_last_disc_ms) {
        uint32_t gap = now - split_link_last_disc_ms;

        if (split_link_min_gap_ms == 0 || gap < split_link_min_gap_ms) {
            split_link_min_gap_ms = gap;
        }
    }
}

#endif /* CONFIG_MK2_SPLIT_LINK_STATS */

static struct peripheral_slot peripherals[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

#if IS_ENABLED(CONFIG_MK2_SPLIT_LINK_STATS)

static uint32_t mk2_split_link_connected_slots(void) {
    uint32_t connected = 0;

    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state == PERIPHERAL_SLOT_STATE_CONNECTED) {
            connected++;
        }
    }

    return connected;
}

#endif /* CONFIG_MK2_SPLIT_LINK_STATS */

static bool is_scanning = false;

static const struct bt_uuid_128 split_service_uuid = BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);

struct peripheral_event_wrapper {
    uint8_t source;
    struct zmk_split_transport_peripheral_event event;
    /* Set when the event is queued; the consumer subtracts it. A stall on this side loses
     * nothing for a drop counter to see, so the wait is the only trace it leaves.
     * Unconditional for the same reason as the kscan event above: a member that exists in
     * only some configurations breaks the builds that do not enable them. */
    uint32_t queued_ms;
};

K_MSGQ_DEFINE(peripheral_event_msgq, sizeof(struct peripheral_event_wrapper),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_POSITION_QUEUE_SIZE, 4);

void peripheral_event_work_callback(struct k_work *work);

K_WORK_DEFINE(peripheral_event_work, peripheral_event_work_callback);

/* Hands one peripheral event to the consumer, and returns the put result.
 *
 * The caller has already committed the peripheral's new position state by the time this runs,
 * so an edge lost here cannot be rebuilt from a later snapshot: the XOR against the committed
 * state comes out zero and nothing fires again. A lost release leaves that key held for good,
 * which is what the host then turns into a stream of repeats.
 *
 * Nothing is sacrificed to make room. An earlier version evicted the oldest queued event so
 * the newest would fit, on the theory that the newest is the more likely release. That was
 * wrong twice over: the oldest is just as likely to be a release, and unlike the newest it
 * cannot be undone - its bit was committed and a newer edge for the same position may already
 * be queued behind it, so rolling it back could contradict that. The recovery could therefore
 * strand a key held forever, which is the exact defect being hunted. Failing the put instead
 * leaves the newest edge in the caller's hands, where it can be undone. */
static int split_enqueue_peripheral_event(struct peripheral_event_wrapper *ev) {
    ev->queued_ms = k_uptime_get_32();
    int err = k_msgq_put(&peripheral_event_msgq, ev, K_NO_WAIT);

    mk2_split_pos_note_queue(k_msgq_num_used_get(&peripheral_event_msgq));
    k_work_submit(&peripheral_event_work);

    return err;
}

/* For everything on this queue that is not a key edge: sensor, input, relay, battery.
 * Nothing to roll back - a lost battery reading is a stale percentage, not a stuck key -
 * but it is counted, so the queue's true pressure is visible rather than being read off
 * the key events alone. */
static void split_enqueue_other_event(struct peripheral_event_wrapper *ev) {
    if (split_enqueue_peripheral_event(ev) != 0) {
        mk2_split_note_other_drop();
    }
}

int peripheral_slot_index_for_conn(struct bt_conn *conn) {
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].conn == conn) {
            return i;
        }
    }

    return -EINVAL;
}

struct peripheral_slot *peripheral_slot_for_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return NULL;
    }

    return &peripherals[idx];
}

int release_peripheral_slot(int index) {
    if (index < 0 || index >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
        return -EINVAL;
    }

    struct peripheral_slot *slot = &peripherals[index];

    if (slot->state == PERIPHERAL_SLOT_STATE_OPEN) {
        return -EINVAL;
    }

    LOG_DBG("Releasing peripheral slot at %d", index);

    if (slot->conn != NULL) {
        bt_conn_unref(slot->conn);
        slot->conn = NULL;
    }
    slot->state = PERIPHERAL_SLOT_STATE_OPEN;

    /* There IS a race here, and it is deliberately left alone.
     *
     * conn and state above are cleared outside any lock, so an in-flight notification can
     * still commit a bit into this slot after it has been marked OPEN, and that bit has
     * nothing left to release it. An attempt to fix it by zeroing first and releasing from a
     * copy was written and then withdrawn: it moved the race rather than closing it, because
     * the notification would then queue its decoded press AFTER these releases and the key
     * would end up held anyway.
     *
     * Closing it properly means one synchronisation contract over conn, state, a slot
     * generation and the event ordering - not a lock around sixteen bytes. That is a
     * behaviour change, and it does not belong in a build whose job is to find out what is
     * actually happening. Instrument first, then fix.
     */
    // Raise events releasing any active positions from this peripheral
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->position_state[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                struct peripheral_event_wrapper ev = {
                    .source = index,
                    .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                              .data = {.key_position_event = {
                                           .position = position,
                                           .pressed = false,
                                       }}}};

                /* These are the releases that stop the halves parting company from leaving
                 * keys held. Losing one here is worse than losing one on the normal path:
                 * position_state is zeroed a few lines below, so there is no earlier value
                 * left to roll back to and nothing can regenerate the edge. Recorded under
                 * its own kind for that reason. */
                int put_err = split_enqueue_peripheral_event(&ev);

                if (put_err != 0) {
                    mk2_split_pos_note_loss(position, false, put_err,
                                            MK2_SPLIT_LOSS_ON_DISCONNECT);
                }
            }
        }
    }

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->position_state[i] = 0U;
        slot->changed_positions[i] = 0U;
    }

    // Clean up previously discovered handles;
    slot->subscribe_params.value_handle = 0;
    slot->run_behavior_handle = 0;
    slot->selected_physical_layout_handle = 0;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    slot->update_hid_indicators = 0;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

    return 0;
}

int reserve_peripheral_slot(const bt_addr_le_t *addr) {
    int i = zmk_ble_put_peripheral_addr(addr);
    if (i >= 0) {
        if (peripherals[i].state == PERIPHERAL_SLOT_STATE_OPEN) {
            // Be sure the slot is fully reinitialized.
            release_peripheral_slot(i);
            peripherals[i].state = PERIPHERAL_SLOT_STATE_CONNECTING;
            return i;
        }
    }

    return -ENOMEM;
}

int release_peripheral_slot_for_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return idx;
    }

    return release_peripheral_slot(idx);
}

int confirm_peripheral_slot_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return idx;
    }

    peripherals[idx].state = PERIPHERAL_SLOT_STATE_CONNECTED;
    return 0;
}

static void notify_transport_status(void);

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

#if ZMK_KEYMAP_HAS_SENSORS

static uint8_t split_central_sensor_notify_func(struct bt_conn *conn,
                                                struct bt_gatt_subscribe_params *params,
                                                const void *data, uint16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[SENSOR NOTIFICATION] data %p length %u", data, length);

    if (length < offsetof(struct sensor_event, channel_data)) {
        LOG_WRN("Ignoring sensor notify with insufficient data length (%d)", length);
        return BT_GATT_ITER_STOP;
    }

    struct sensor_event sensor_event;
    memcpy(&sensor_event, data, MIN(length, sizeof(sensor_event)));
    if (sensor_event.channel_data_size != 1) {
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_event_wrapper event_wrapper = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT,
                  .data = {.sensor_event = {
                               .channel_data = sensor_event.channel_data[0],
                               .sensor_index = sensor_event.sensor_index,
                           }}}};

    split_enqueue_other_event(&event_wrapper);

    return BT_GATT_ITER_CONTINUE;
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)

static void update_peripherals_relay_event_work_handler(struct k_work *_work);

K_MSGQ_DEFINE(relay_event_msgq, sizeof(struct zmk_split_relay_event_payload),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_POSITION_QUEUE_SIZE, 4);
K_WORK_DEFINE(update_peripherals_relay_event_work, update_peripherals_relay_event_work_handler);

int zmk_split_central_send_relay_event(struct zmk_split_relay_event_payload *payload) {
    int err = k_msgq_put(&relay_event_msgq, payload, K_NO_WAIT);
    if (err) {
        LOG_ERR("Failed to queue relay event to send (%d)", err);
        return err;
    }
    k_work_submit(&update_peripherals_relay_event_work);
    return 0;
}

static void update_peripherals_relay_event_work_handler(struct k_work *_work) {
    struct zmk_split_relay_event_payload payload;
    if (k_msgq_get(&relay_event_msgq, &payload, K_NO_WAIT) != 0) {
        return;
    }

    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        struct peripheral_slot *slot = &peripherals[i];
        if (slot->state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            continue;
        }

        if (slot->relay_event_subscribe_params.value_handle == 0) {
            // It appears that sometimes the peripheral is considered connected
            // before the GATT characteristics have been discovered. If this is
            // the case, the selected_physical_layout_handle will not yet be set.
            LOG_WRN("Peripheral relay event subscribe params not set, cannot send relay event");
            return;
        }

        if (bt_conn_get_security(slot->conn) < BT_SECURITY_L2) {
            LOG_WRN("Peripheral link not encrypted, cannot send relay event");
            return;
        }

        uint8_t relay_event_buffer[sizeof(struct relay_event_header) +
                                   CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN +
                                   CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN];
        size_t total_size = 0;
        memcpy(relay_event_buffer, &payload.header, sizeof(struct relay_event_header));
        total_size += sizeof(struct relay_event_header);
        memcpy(relay_event_buffer + total_size, payload.event_type, payload.header.event_type_size);
        total_size += payload.header.event_type_size;
        memcpy(relay_event_buffer + total_size, payload.event_data, payload.header.event_data_size);
        total_size += payload.header.event_data_size;
        LOG_DBG("Prepared relay event of total size %d bytes (header %d, name %d, data %d)",
                total_size, sizeof(struct relay_event_header), payload.header.event_type_size,
                payload.header.event_data_size);
        int err = bt_gatt_write_without_response(slot->conn,
                                                 slot->relay_event_subscribe_params.value_handle,
                                                 relay_event_buffer, total_size, true);

        if (err < 0) {
            LOG_ERR("Failed to write physical layout index to peripheral (err %d)", err);
        }
        if (k_msgq_num_used_get(&relay_event_msgq) > 0) {
            k_work_submit(&update_peripherals_relay_event_work);
        }
    }
    return;
}

static uint8_t split_central_relay_event_notify_func(struct bt_conn *conn,
                                                     struct bt_gatt_subscribe_params *params,
                                                     const void *data, uint16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[RELAY EVENT NOTIFICATION] data %p length %u", data, length);

    if (length < sizeof(struct relay_event_header)) {
        LOG_WRN("Relay event too small (%d), need at least %d bytes", length,
                sizeof(struct relay_event_header));
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_event_wrapper event_wrapper = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_RELAY_EVENT,
                  .data = {.relay_event = {}}}};
    struct relay_event_header *header = &event_wrapper.event.data.relay_event.header;
    // Unpack header struct
    memcpy(header, data, sizeof(struct relay_event_header));

    if (header->event_type_size > CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN) {
        LOG_WRN("Event type name too long (%d), max is %d", header->event_type_size,
                CONFIG_ZMK_SPLIT_RELAY_EVENT_TYPE_NAME_LEN - 1);
        return BT_GATT_ITER_STOP;
    }

    if (header->event_data_size > CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN) {
        LOG_WRN("Event data too large (%d), max is %d", header->event_data_size,
                CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN);
        return BT_GATT_ITER_STOP;
    }

    if (sizeof(struct relay_event_header) + header->event_type_size + header->event_data_size !=
        length) {
        LOG_WRN("Malformed relay event: size mismatch (expected %d, got %d)",
                sizeof(struct relay_event_header) + header->event_type_size +
                    header->event_data_size,
                length);
        return BT_GATT_ITER_STOP;
    }
    memcpy(event_wrapper.event.data.relay_event.event_type,
           data + sizeof(struct relay_event_header), header->event_type_size);
    event_wrapper.event.data.relay_event.event_type[header->event_type_size] = '\0';
    memcpy(event_wrapper.event.data.relay_event.event_data,
           data + sizeof(struct relay_event_header) + header->event_type_size,
           header->event_data_size);
    LOG_DBG("Received relay event: type='%s', data_len=%d (wire: %d bytes)",
            event_wrapper.event.data.relay_event.event_type,
            event_wrapper.event.data.relay_event.header.event_data_size, length);

    split_enqueue_other_event(&event_wrapper);

    return BT_GATT_ITER_CONTINUE;
}

#endif

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static uint8_t peripheral_input_event_notify_cb(struct bt_conn *conn,
                                                struct bt_gatt_subscribe_params *params,
                                                const void *data, uint16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[INPUT EVENT] data %p length %u", data, length);

    if (length != sizeof(struct zmk_split_input_event_payload)) {
        LOG_WRN("Ignoring input event notify with incorrect data length (%d)", length);
        return BT_GATT_ITER_STOP;
    }

    struct zmk_split_input_event_payload payload;
    memcpy(&payload, data, MIN(length, sizeof(struct zmk_split_input_event_payload)));

    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (&peripheral_input_slots[i].sub == params) {
            struct peripheral_event_wrapper event_wrapper = {
                .source = peripheral_slot_index_for_conn(conn),
                .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,
                          .data = {.input_event = {
                                       .reg = peripheral_input_slots[i].reg,
                                       .sync = payload.sync,
                                       .code = payload.code,
                                       .type = payload.type,
                                       .value = payload.value,
                                   }}}};

            split_enqueue_other_event(&event_wrapper);
            break;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

#endif

static uint8_t split_central_notify_func(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params, const void *data,
                                         uint16_t length) {
    /* First instruction: every existing counter on this half is downstream of here. */
    mk2_split_note_rx();

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);

    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        mk2_split_note_rx_no_slot();
        return BT_GATT_ITER_CONTINUE;
    }

    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        mk2_split_note_rx_unsub();
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[NOTIFICATION] data %p length %u", data, length);

    /* The loop below reads POSITION_STATE_DATA_LEN bytes whatever the notification's actual
     * size. A short one therefore read past the end of the buffer and XORed whatever
     * happened to follow it into the key state - inventing presses and releases for keys
     * nobody touched. Refusing it costs one snapshot, and the peripheral sends whole state,
     * so the next notification restores everything. */
    if (length < POSITION_STATE_DATA_LEN) {
        LOG_WRN("Ignoring short position notification (%u < %d)", length,
                POSITION_STATE_DATA_LEN);
        mk2_split_note_short_notify();
        return BT_GATT_ITER_CONTINUE;
    }

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->changed_positions[i] = ((uint8_t *)data)[i] ^ slot->position_state[i];
        slot->position_state[i] = ((uint8_t *)data)[i];
    }
    LOG_HEXDUMP_DBG(slot->position_state, POSITION_STATE_DATA_LEN, "data");

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->changed_positions[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = slot->position_state[i] & BIT(j);
                struct peripheral_event_wrapper ev = {
                    .source = peripheral_slot_index_for_conn(conn),
                    .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                              .data = {.key_position_event = {
                                           .position = position,
                                           .pressed = pressed,
                                       }}}};
                int put_err;

#if CONFIG_MK2_SPLIT_POS_DROP_SELFTEST > 0
                /* Fault injection: skipping the put loses a decoded press/release edge, which
                 * is precisely how a key or layer gets stuck. Bench use only. */
                split_pos_drop_selftest_attempt++;
                if (split_pos_drop_selftest_attempt % CONFIG_MK2_SPLIT_POS_DROP_SELFTEST == 0) {
                    put_err = -ENOMSG;
                } else {
                    put_err = split_enqueue_peripheral_event(&ev);
                }
#else
                put_err = split_enqueue_peripheral_event(&ev);
#endif

                if (put_err != 0) {
                    /* Undo the commit for this one bit.
                     *
                     * The peripheral sends whole-state snapshots, so putting the previous
                     * value back makes the next snapshot differ here again and the missing
                     * edge is regenerated on the very next key event - rather than staying
                     * lost until this particular key happens to move again.
                     *
                     * It is not free. If the edge that was lost was a press, its release will
                     * now cancel against the restored bit and the whole tap disappears: one
                     * character that never arrives. That is a much smaller fault than a key
                     * that never comes back up, which is the trade being made here.
                     *
                     * Not locked, and the reason is worth stating: a lock here would not be
                     * enough anyway. If a disconnect has zeroed the array in between, this
                     * XOR raises the bit again from zero and invents a held key. Guarding it
                     * needs the same slot-lifetime contract as the disconnect path, which is
                     * a separate piece of work. Until then the failure is at least recorded
                     * below, so it cannot happen invisibly. */
                    slot->position_state[i] ^= BIT(j);
                    mk2_split_pos_note_loss(position, pressed, put_err,
                                            MK2_SPLIT_LOSS_ROLLED_BACK);
                }
            }
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)

static uint8_t split_central_battery_level_notify_func(struct bt_conn *conn,
                                                       struct bt_gatt_subscribe_params *params,
                                                       const void *data, uint16_t length) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);

    if (!slot) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_CONTINUE;
    }

    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (length == 0) {
        LOG_ERR("Zero length battery notification received");
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_DBG("[BATTERY LEVEL NOTIFICATION] data %p length %u", data, length);
    uint8_t battery_level = ((uint8_t *)data)[0];
    LOG_DBG("Battery level: %u", battery_level);

    struct peripheral_event_wrapper ev = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                  .data = {.battery_event = {
                               .level = battery_level,
                           }}}};

    split_enqueue_other_event(&ev);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t split_central_battery_level_read_func(struct bt_conn *conn, uint8_t err,
                                                     struct bt_gatt_read_params *params,
                                                     const void *data, uint16_t length) {
    if (err > 0) {
        LOG_ERR("Error during reading peripheral battery level: %u", err);
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);

    if (!slot) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_CONTINUE;
    }

    if (!data) {
        LOG_DBG("[READ COMPLETED]");
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[BATTERY LEVEL READ] data %p length %u", data, length);

    if (length == 0) {
        LOG_ERR("Zero length battery notification received");
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t battery_level = ((uint8_t *)data)[0];

    LOG_DBG("Battery level: %u", battery_level);

    struct peripheral_event_wrapper ev = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                  .data = {.battery_event = {
                               .level = battery_level,
                           }}}};

    split_enqueue_other_event(&ev);

    return BT_GATT_ITER_CONTINUE;
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */

static int split_central_subscribe(struct bt_conn *conn, struct bt_gatt_subscribe_params *params) {
    atomic_set(params->flags, BT_GATT_SUBSCRIBE_FLAG_NO_RESUB);
    int err = bt_gatt_subscribe(conn, params);
    switch (err) {
    case -EALREADY:
        LOG_DBG("[ALREADY SUBSCRIBED]");
        break;
    case 0:
        LOG_DBG("[SUBSCRIBED]");
        break;
    default:
        LOG_ERR("Subscribe failed (err %d)", err);
        break;
    }

    return err;
}

static int update_peripheral_selected_layout(struct peripheral_slot *slot, uint8_t layout_idx) {
    if (slot->state != PERIPHERAL_SLOT_STATE_CONNECTED) {
        return -ENOTCONN;
    }

    if (slot->selected_physical_layout_handle == 0) {
        // It appears that sometimes the peripheral is considered connected
        // before the GATT characteristics have been discovered. If this is
        // the case, the selected_physical_layout_handle will not yet be set.
        return -EAGAIN;
    }

    if (bt_conn_get_security(slot->conn) < BT_SECURITY_L2) {
        return -EAGAIN;
    }

    int err = bt_gatt_write_without_response(slot->conn, slot->selected_physical_layout_handle,
                                             &layout_idx, sizeof(layout_idx), true);

    if (err < 0) {
        LOG_ERR("Failed to write physical layout index to peripheral (err %d)", err);
    }

    return err;
}

static void update_peripherals_selected_physical_layout(struct k_work *_work) {
    uint8_t layout_idx = zmk_physical_layouts_get_selected();
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            continue;
        }

        update_peripheral_selected_layout(&peripherals[i], layout_idx);
    }
}

K_WORK_DEFINE(update_peripherals_selected_layouts_work,
              update_peripherals_selected_physical_layout);

static uint8_t split_central_chrc_discovery_func(struct bt_conn *conn,
                                                 const struct bt_gatt_attr *attr,
                                                 struct bt_gatt_discover_params *params) {
    if (!attr) {
        LOG_DBG("Discover complete");
        return BT_GATT_ITER_STOP;
    }

    if (!attr->user_data) {
        LOG_ERR("Required user data not passed to discovery");
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);
    switch (params->type) {
    case BT_GATT_DISCOVER_CHARACTERISTIC: {
        const struct bt_uuid *chrc_uuid = ((struct bt_gatt_chrc *)attr->user_data)->uuid;

        if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID)) ==
            0) {
            LOG_DBG("Found position state characteristic");
            slot->subscribe_params.disc_params = &slot->sub_discover_params;
            slot->subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->subscribe_params.notify = split_central_notify_func;
            slot->subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->subscribe_params);
#if ZMK_KEYMAP_HAS_SENSORS
        } else if (bt_uuid_cmp(chrc_uuid,
                               BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_SENSOR_STATE_UUID)) == 0) {
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

            slot->sensor_subscribe_params.disc_params = &slot->sensor_sub_discover_params;
            slot->sensor_subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->sensor_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->sensor_subscribe_params.notify = split_central_sensor_notify_func;
            slot->sensor_subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->sensor_subscribe_params);
#endif /* ZMK_KEYMAP_HAS_SENSORS */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
        } else if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_RELAY_EVENT_UUID)) ==
                   0) {
            LOG_DBG("Found relay event characteristic");
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

            slot->relay_event_subscribe_params.disc_params = &slot->relay_event_sub_discover_params;
            slot->relay_event_subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->relay_event_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->relay_event_subscribe_params.notify = split_central_relay_event_notify_func;
            slot->relay_event_subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->relay_event_subscribe_params);
#endif
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        } else if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID)) ==
                   0) {
            LOG_DBG("Found an input characteristic");
            struct peripheral_input_slot *input_slot;
            int ret = reserve_next_open_input_slot(&input_slot, conn);
            if (ret < 0) {
                LOG_WRN("No available slot for peripheral input subscriptions (%d)", ret);

                slot->discover_params.uuid = NULL;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            } else {
                LOG_DBG("Reserved a slot for the input subscription");
                input_slot->sub.value_handle = bt_gatt_attr_value_handle(attr);

                slot->discover_params.uuid = gatt_ccc_uuid;
                slot->discover_params.start_handle = attr->handle;
                slot->discover_params.type = BT_GATT_DISCOVER_STD_CHAR_DESC;
            }
#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        } else if (bt_uuid_cmp(chrc_uuid,
                               BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID)) == 0) {
            LOG_DBG("Found run behavior handle");
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->run_behavior_handle = bt_gatt_attr_value_handle(attr);
        } else if (!bt_uuid_cmp(((struct bt_gatt_chrc *)attr->user_data)->uuid,
                                BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SELECT_PHYS_LAYOUT_UUID))) {
            LOG_DBG("Found select physical layout handle");
            slot->selected_physical_layout_handle = bt_gatt_attr_value_handle(attr);
            k_work_submit(&update_peripherals_selected_layouts_work);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        } else if (!bt_uuid_cmp(((struct bt_gatt_chrc *)attr->user_data)->uuid,
                                BT_UUID_DECLARE_128(ZMK_SPLIT_BT_UPDATE_HID_INDICATORS_UUID))) {
            LOG_DBG("Found update HID indicators handle");
            slot->update_hid_indicators = bt_gatt_attr_value_handle(attr);
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
        } else if (!bt_uuid_cmp(((struct bt_gatt_chrc *)attr->user_data)->uuid,
                                BT_UUID_BAS_BATTERY_LEVEL)) {
            LOG_DBG("Found battery level characteristics");
            slot->batt_lvl_subscribe_params.disc_params = &slot->batt_lvl_sub_discover_params;
            slot->batt_lvl_subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->batt_lvl_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->batt_lvl_subscribe_params.notify = split_central_battery_level_notify_func;
            slot->batt_lvl_subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->batt_lvl_subscribe_params);

            slot->batt_lvl_read_params.func = split_central_battery_level_read_func;
            slot->batt_lvl_read_params.handle_count = 1;
            slot->batt_lvl_read_params.single.handle = bt_gatt_attr_value_handle(attr);
            slot->batt_lvl_read_params.single.offset = 0;
            bt_gatt_read(conn, &slot->batt_lvl_read_params);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */
        }
        break;
    }
    case BT_GATT_DISCOVER_STD_CHAR_DESC:
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_GATT_CCC) == 0) {
            LOG_DBG("Found input CCC descriptor");
            struct peripheral_input_slot *input_slot;
            int ret = find_pending_input_slot(&input_slot, conn);
            if (ret < 0) {
                LOG_DBG("No pending input slot (%d)", ret);
                slot->discover_params.uuid = NULL;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            } else {
                LOG_DBG("Found pending input slot");
                input_slot->sub.ccc_handle = attr->handle;

                slot->discover_params.uuid = gatt_cpf_uuid;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_STD_CHAR_DESC;
            }
        } else if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_GATT_CPF) == 0) {
            LOG_DBG("Found input CPF descriptor");
            struct bt_gatt_cpf *cpf = attr->user_data;
            struct peripheral_input_slot *input_slot;
            int ret = find_pending_input_slot(&input_slot, conn);
            if (ret < 0) {
                LOG_DBG("No pending input slot (%d)", ret);
            } else {
                LOG_DBG("Found pending input slot");
                input_slot->reg = cpf->description;
                input_slot->sub.notify = peripheral_input_event_notify_cb;
                input_slot->sub.value = BT_GATT_CCC_NOTIFY;
                int err = split_central_subscribe(conn, &input_slot->sub);
                if (err < 0) {
                    LOG_WRN("Failed to subscribe to input notifications %d", err);
                }
            }

            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 1;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
        }
#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        break;
    }

    bool subscribed = slot->run_behavior_handle && slot->subscribe_params.value_handle &&
                      slot->selected_physical_layout_handle;

#if ZMK_KEYMAP_HAS_SENSORS
    subscribed = subscribed && slot->sensor_subscribe_params.value_handle;
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
    subscribed = subscribed && slot->relay_event_subscribe_params.value_handle;
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    subscribed = subscribed && slot->update_hid_indicators;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    subscribed = subscribed && slot->batt_lvl_subscribe_params.value_handle;
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (input_slot_is_open(i) || input_slot_is_pending(i)) {
            subscribed = false;
            break;
        }
    }
#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

    return subscribed ? BT_GATT_ITER_STOP : BT_GATT_ITER_CONTINUE;
}

static uint8_t split_central_service_discovery_func(struct bt_conn *conn,
                                                    const struct bt_gatt_attr *attr,
                                                    struct bt_gatt_discover_params *params) {
    if (!attr) {
        LOG_DBG("Discover complete");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_STOP;
    }

    if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)) !=
        0) {
        LOG_DBG("Found other service");
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_DBG("Found split service");
    slot->discover_params.uuid = NULL;
    slot->discover_params.func = split_central_chrc_discovery_func;
    slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &slot->discover_params);
    if (err) {
        LOG_ERR("Failed to start discovering split service characteristics (err %d)", err);
    }
    return BT_GATT_ITER_STOP;
}

static void split_central_process_connection(struct bt_conn *conn) {
    int err;

    LOG_DBG("Current security for connection: %d", bt_conn_get_security(conn));

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        return;
    }

    if (!slot->subscribe_params.value_handle) {
        slot->discover_params.uuid = &split_service_uuid.uuid;
        slot->discover_params.func = split_central_service_discovery_func;
        slot->discover_params.start_handle = 0x0001;
        slot->discover_params.end_handle = 0xffff;
        slot->discover_params.type = BT_GATT_DISCOVER_PRIMARY;

        err = bt_gatt_discover(slot->conn, &slot->discover_params);
        if (err) {
            LOG_ERR("Discover failed(err %d)", err);
            return;
        }
    }

    struct bt_conn_info info;

    bt_conn_get_info(conn, &info);

    LOG_DBG("New connection params: Interval: %d, Latency: %d, PHY: %d", info.le.interval,
            info.le.latency, info.le.phy->rx_phy);

    // Restart scanning if necessary.
    start_scanning();
}

static int stop_scanning(void) {
    LOG_DBG("Stopping peripheral scanning");
    is_scanning = false;

    int err = bt_le_scan_stop();
    if (err < 0) {
        LOG_ERR("Stop LE scan failed (err %d)", err);
        return err;
    }

    return 0;
}

static bool split_central_eir_found(const bt_addr_le_t *addr) {
    LOG_DBG("Found the split service");

    // Reserve peripheral slot. Once the central has bonded to its peripherals,
    // the peripheral MAC addresses will be validated internally and the slot
    // reservation will fail if there is a mismatch.
    int slot_idx = reserve_peripheral_slot(addr);
    if (slot_idx < 0) {
        LOG_INF("Unable to reserve peripheral slot (err %d)", slot_idx);
        return false;
    }
    struct peripheral_slot *slot = &peripherals[slot_idx];

    // Stop scanning so we can connect to the peripheral device.
    int err = stop_scanning();
    if (err < 0) {
        return false;
    }

    LOG_DBG("Initiating new connection");
    struct bt_le_conn_param *param =
        BT_LE_CONN_PARAM(CONFIG_ZMK_SPLIT_BLE_PREF_INT, CONFIG_ZMK_SPLIT_BLE_PREF_INT,
                         CONFIG_ZMK_SPLIT_BLE_PREF_LATENCY, CONFIG_ZMK_SPLIT_BLE_PREF_TIMEOUT);
    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &slot->conn);
    if (err < 0) {
        LOG_ERR("Create conn failed (err %d) (create conn? 0x%04x)", err, BT_HCI_OP_LE_CREATE_CONN);
        release_peripheral_slot(slot_idx);
        start_scanning();
    }

    return false;
}

static bool split_central_eir_parse(struct bt_data *data, void *user_data) {
    bt_addr_le_t *addr = user_data;
    int i;

    LOG_DBG("[AD]: %u data_len %u", data->type, data->data_len);

    switch (data->type) {
    case BT_DATA_UUID128_SOME:
    case BT_DATA_UUID128_ALL:
        if (data->data_len % 16 != 0U) {
            LOG_ERR("AD malformed");
            return true;
        }

        for (i = 0; i < data->data_len; i += 16) {
            struct bt_uuid_128 uuid;

            if (!bt_uuid_create(&uuid.uuid, &data->data[i], 16)) {
                LOG_ERR("Unable to load UUID");
                continue;
            }

            if (bt_uuid_cmp(&uuid.uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)) != 0) {
                char uuid_str[BT_UUID_STR_LEN];
                char service_uuid_str[BT_UUID_STR_LEN];

                bt_uuid_to_str(&uuid.uuid, uuid_str, sizeof(uuid_str));
                bt_uuid_to_str(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID), service_uuid_str,
                               sizeof(service_uuid_str));
                LOG_DBG("UUID %s does not match split UUID: %s", uuid_str, service_uuid_str);
                continue;
            }

            return split_central_eir_found(addr);
        }
    }

    return true;
}

static void split_central_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                                       struct net_buf_simple *ad) {
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));
    LOG_DBG("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i", dev, type, ad->len, rssi);

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND) {
        bt_data_parse(ad, split_central_eir_parse, (void *)addr);
    } else if (type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        split_central_eir_found(addr);
    }
}

static int start_scanning(void) {
    if (!is_enabled) {
        LOG_DBG("Not scanning, we're disabled");
        return 0;
    }

    // No action is necessary if central is already scanning.
    if (is_scanning) {
        LOG_DBG("Scanning already running");
        return 0;
    }

    // If all the devices are connected, there is no need to scan.
    bool has_unconnected = false;
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (peripherals[i].conn == NULL) {
            has_unconnected = true;
            break;
        }
    }
    if (!has_unconnected) {
        LOG_DBG("All devices are connected, scanning is unnecessary");
        return 0;
    }

    // Start scanning otherwise.
    is_scanning = true;
    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, split_central_device_found);
    if (err < 0) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }

    LOG_DBG("Scanning successfully started");
    return 0;
}

static void split_central_connected(struct bt_conn *conn, uint8_t conn_err) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_CENTRAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)", addr, conn_err);

        release_peripheral_slot_for_conn(conn);

        start_scanning();
        return;
    }

    LOG_DBG("Connected: %s", addr);

#if IS_ENABLED(CONFIG_MK2_SPLIT_LINK_STATS)
    /* Counted only on a real connection: the conn_err path above returns before reaching
     * here, so a failed attempt is not mistaken for a recovery. */
    mk2_split_link_note_connect();
    printk("[SPLINK] connect up=%u ms\n", k_uptime_get_32());
#endif

    confirm_peripheral_slot_conn(conn);
    split_central_process_connection(conn);
    k_work_submit(&notify_status_work);
}

static void split_central_disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Disconnected: %s (reason %d)", addr, reason);

#if IS_ENABLED(CONFIG_MK2_SPLIT_LINK_STATS)
    mk2_split_link_note_disconnect(reason);
    /* Printed as well as counted: when a console does happen to be attached, the line
     * carries a timestamp the counters cannot, which is what ties a drop to whatever the
     * host was doing at that moment. */
    printk("[SPLINK] disconnect reason=0x%02x up=%u ms\n", reason, k_uptime_get_32());
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    struct peripheral_event_wrapper ev = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                  .data = {.battery_event = {
                               .level = 0,
                           }}}};

    split_enqueue_other_event(&ev);
    // struct zmk_peripheral_battery_state_changed ev = {
    //     .source = peripheral_slot_index_for_conn(conn), .state_of_charge = 0};
    // k_msgq_put(&peripheral_batt_lvl_msgq, &ev, K_NO_WAIT);
    // k_work_submit(&peripheral_batt_lvl_work);
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    release_peripheral_input_subs(conn);
#endif

    err = release_peripheral_slot_for_conn(conn);

    if (err < 0) {
        LOG_WRN("Failed to release peripheral slot (%d)", err);
    }

    k_work_submit(&notify_status_work);

    start_scanning();
}

static void split_central_security_changed(struct bt_conn *conn, bt_security_t level,
                                           enum bt_security_err err) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (!slot || !slot->selected_physical_layout_handle) {
        return;
    }

    if (err > 0) {
        LOG_DBG("Skipping updating the physical layout for peripheral with security error");
        return;
    }

    if (level < BT_SECURITY_L2) {
        LOG_DBG("Skipping updating the physical layout for peripheral with insufficient security");
        return;
    }

    k_work_submit(&update_peripherals_selected_layouts_work);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = split_central_connected,
    .disconnected = split_central_disconnected,
    .security_changed = split_central_security_changed,
};

K_THREAD_STACK_DEFINE(split_central_split_run_q_stack,
                      CONFIG_ZMK_SPLIT_BLE_CENTRAL_SPLIT_RUN_STACK_SIZE);

struct k_work_q split_central_split_run_q;

struct central_cmd_wrapper {
    uint8_t source;
    struct zmk_split_transport_central_command cmd;
};

K_MSGQ_DEFINE(zmk_split_central_split_run_msgq, sizeof(struct central_cmd_wrapper),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_SPLIT_RUN_QUEUE_SIZE, 4);

void split_central_split_run_callback(struct k_work *work) {
    struct central_cmd_wrapper payload_wrapper;

    LOG_DBG("");

    while (k_msgq_get(&zmk_split_central_split_run_msgq, &payload_wrapper, K_NO_WAIT) == 0) {
        if (peripherals[payload_wrapper.source].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            LOG_ERR("Source not connected");
            continue;
        }

        switch (payload_wrapper.cmd.type) {
        case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR: {
            if (!peripherals[payload_wrapper.source].run_behavior_handle) {
                LOG_ERR("Run behavior handle not found");
                continue;
            }

            struct zmk_split_run_behavior_payload payload = {
                .data = {
                    .param1 = payload_wrapper.cmd.data.invoke_behavior.param1,
                    .param2 = payload_wrapper.cmd.data.invoke_behavior.param2,
                    .position = payload_wrapper.cmd.data.invoke_behavior.position,
                    .source = payload_wrapper.cmd.data.invoke_behavior.event_source,
                    .state = payload_wrapper.cmd.data.invoke_behavior.state ? 1 : 0,
                }};
            const size_t payload_dev_size = sizeof(payload.behavior_dev);
            if (strlcpy(payload.behavior_dev, payload_wrapper.cmd.data.invoke_behavior.behavior_dev,
                        payload_dev_size) >= payload_dev_size) {
                LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                        payload_wrapper.cmd.data.invoke_behavior.behavior_dev,
                        payload.behavior_dev);
            }

            int err = bt_gatt_write_without_response(
                peripherals[payload_wrapper.source].conn,
                peripherals[payload_wrapper.source].run_behavior_handle, &payload,
                sizeof(struct zmk_split_run_behavior_payload), true);

            if (err) {
                LOG_ERR("Failed to write the behavior characteristic (err %d)", err);
            }
            break;
        }
        case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
            update_peripheral_selected_layout(
                &peripherals[payload_wrapper.source],
                payload_wrapper.cmd.data.set_physical_layout.layout_idx);
            break;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
            LOG_WRN("do the indicators dance");
            if (peripherals[payload_wrapper.source].update_hid_indicators == 0) {
                // It appears that sometimes the peripheral is considered connected
                // before the GATT characteristics have been discovered. If this is
                // the case, the update_hid_indicators handle will not yet be set.
                LOG_WRN("NO HANDLE TO SET ON PERIPHERAL");
                break;
            }

            int err = bt_gatt_write_without_response(
                peripherals[payload_wrapper.source].conn,
                peripherals[payload_wrapper.source].update_hid_indicators,
                &payload_wrapper.cmd.data.set_hid_indicators.indicators,
                sizeof(payload_wrapper.cmd.data.set_hid_indicators.indicators), true);

            if (err) {
                LOG_ERR("Failed to write HID indicator characteristic (err %d)", err);
            }
            break;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        default:
            LOG_WRN("Unsupported wrapped central command type %d", payload_wrapper.cmd.type);
            return;
        }
    }
}

K_WORK_DEFINE(split_central_split_run_work, split_central_split_run_callback);

static int split_bt_invoke_behavior_payload(struct central_cmd_wrapper payload_wrapper) {
    LOG_DBG("");

    int err = k_msgq_put(&zmk_split_central_split_run_msgq, &payload_wrapper, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Run command message queue full, popping first message and queueing again");
            struct central_cmd_wrapper discarded_report;
            k_msgq_get(&zmk_split_central_split_run_msgq, &discarded_report, K_NO_WAIT);
            return split_bt_invoke_behavior_payload(payload_wrapper);
        }
        default:
            LOG_WRN("Failed to queue behavior to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&split_central_split_run_q, &split_central_split_run_work);

    return 0;
};

static int finish_init();

static bool settings_loaded = false;

#if IS_ENABLED(CONFIG_SETTINGS)

static int central_ble_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                  void *cb_arg) {
    return 0;
}

static struct settings_handler ble_central_settings_handler = {
    .name = "ble_central", .h_set = central_ble_handle_set, .h_commit = finish_init};

#endif // IS_ENABLED(CONFIG_SETTINGS)

static int zmk_split_bt_central_init(void) {
    k_work_queue_start(&split_central_split_run_q, split_central_split_run_q_stack,
                       K_THREAD_STACK_SIZEOF(split_central_split_run_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, NULL);
    bt_conn_cb_register(&conn_callbacks);

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_register(&ble_central_settings_handler);
    return 0;
#else
    return finish_init();
#endif // IS_ENABLED(CONFIG_SETTINGS)
}

SYS_INIT(zmk_split_bt_central_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);

static int zmk_split_bt_central_listener_cb(const zmk_event_t *eh) {
    if (as_zmk_physical_layout_selection_changed(eh)) {
        k_work_submit(&update_peripherals_selected_layouts_work);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_split_bt_central, zmk_split_bt_central_listener_cb);
ZMK_SUBSCRIPTION(zmk_split_bt_central, zmk_physical_layout_selection_changed);

static int split_central_bt_send_command(uint8_t source,
                                         struct zmk_split_transport_central_command cmd) {
    if (source >= ARRAY_SIZE(peripherals)) {
        return -EINVAL;
    }

    switch (cmd.type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR: {
        struct central_cmd_wrapper wrapper = {.source = source, .cmd = cmd};
        return split_bt_invoke_behavior_payload(wrapper);
    }
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return -ENOTSUP;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static int split_central_bt_get_available_source_ids(uint8_t *sources) {
    int count = 0;
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            continue;
        }

        sources[count++] = i;
    }

    return count;
}

static int split_central_bt_set_enabled(bool enabled) {
    is_enabled = enabled;
    if (enabled) {
        return start_scanning();
    } else {
        int err = stop_scanning();
        if (err < 0) {
            LOG_WRN("Failed to stop scanning for peripherals (%d)", err);
        }

        for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
            if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
                continue;
            }

            err = bt_conn_disconnect(peripherals[i].conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            if (err < 0) {
                LOG_WRN("Failed to disconnect a peripheral (%d)", err);
            }
        }

        return 0;
    }
}

static int
split_central_bt_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_bt_get_status() {
    uint8_t _source_ids[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

    int count = split_central_bt_get_available_source_ids(_source_ids);

    enum zmk_split_transport_connections_status conn_status;

    if (count == 0) {
        conn_status = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED;
    } else if (count == ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
        conn_status = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED;
    } else {
        conn_status = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_SOME_CONNECTED;
    }

    return (struct zmk_split_transport_status){
        .available = !IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START) && settings_loaded,
        .enabled = is_enabled,
        .connections = conn_status,
    };
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_bt_send_command,
    .get_available_source_ids = split_central_bt_get_available_source_ids,
    .set_enabled = split_central_bt_set_enabled,
    .set_status_callback = split_central_bt_set_status_callback,
    .get_status = split_central_bt_get_status,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(bt_central, &central_api, CONFIG_ZMK_SPLIT_BLE_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&bt_central, split_central_bt_get_status());
    }
}

static int finish_init() {
    settings_loaded = true;

    if (!transport_status_cb) {
        return 0;
    }

    return transport_status_cb(&bt_central, split_central_bt_get_status());
}

void peripheral_event_work_callback(struct k_work *work) {
    struct peripheral_event_wrapper ev;

    /* Timestamped on entry so the gap between drains is measurable. This queue only fills
     * when its consumer is not running, and the consumer is the shared system work queue -
     * so the gap is the difference between "a burst arrived" and "something else was holding
     * the work queue". Without it a drop count says nothing about why. */
    mk2_split_pos_note_drain();

    while (k_msgq_get(&peripheral_event_msgq, &ev, K_NO_WAIT) == 0) {
        mk2_split_pos_note_wait(ev.queued_ms, &ev.event);
        LOG_DBG("Trigger key position state change for %d",
                ev.event.data.key_position_event.position);
        zmk_split_transport_central_peripheral_event_handler(&bt_central, ev.source, ev.event);
    }
}
