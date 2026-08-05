/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * MK2 Matrix GPIO diagnostics — QC self-test for keyboard matrix wiring.
 *
 * Tests (Phase 1):
 *   A: GPIO output/readback — verify each col/row GPIO functions
 *   D: Pull-down integrity — verify all rows read LOW when cols are OFF
 *
 * Tests (Phase 2, to be added):
 *   B: Col-col short detection
 *   C: Col-row short detection
 *   E: Row-row short detection
 *
 * Trigger: USB serial command "DIAG_MATRIX\n" from host.
 * A low-priority polling thread reads CDC ACM UART RX.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zmk/mk2_matrix_diag.h>

#include <string.h>

/* ── DTS node references ── */

#define KSCAN_NODE DT_NODELABEL(kscan0)
#define NUM_COLS DT_PROP_LEN(KSCAN_NODE, col_gpios)
#define NUM_ROWS DT_PROP_LEN(KSCAN_NODE, row_gpios)

/* Build GPIO spec arrays from devicetree at compile time. */
#define COL_SPEC_ENTRY(idx, _) GPIO_DT_SPEC_GET_BY_IDX(KSCAN_NODE, col_gpios, idx)
#define ROW_SPEC_ENTRY(idx, _) GPIO_DT_SPEC_GET_BY_IDX(KSCAN_NODE, row_gpios, idx)

static const struct gpio_dt_spec diag_cols[] = {
    LISTIFY(NUM_COLS, COL_SPEC_ENTRY, (,))
};
static const struct gpio_dt_spec diag_rows[] = {
    LISTIFY(NUM_ROWS, ROW_SPEC_ENTRY, (,))
};

/* ── Counters ── */

static int diag_total;
static int diag_pass;
static int diag_warn;
static int diag_fail;

static void diag_result(const char *test, const char *item, const char *result) {
    printk("%s,%s,%s\n", test, item, result);
    diag_total++;
    if (strcmp(result, "PASS") == 0) {
        diag_pass++;
    } else if (strcmp(result, "WARN") == 0) {
        diag_warn++;
    } else {
        diag_fail++;
    }
}

/* ── Test A: GPIO self-test (output readback) ── */

static void test_a_gpio(void) {
    char label[16];

    /* Test each col GPIO: configure as output, set HIGH, read back, set LOW, read back. */
    for (int i = 0; i < NUM_COLS; i++) {
        const struct gpio_dt_spec *spec = &diag_cols[i];
        int err;

        snprintf(label, sizeof(label), "col%d", i);

        err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
        if (err) {
            diag_result("TEST_A_GPIO", label, "FAIL_CFG");
            continue;
        }

        /* Set HIGH and verify. */
        gpio_pin_set_dt(spec, 1);
        k_busy_wait(10);
        int val = gpio_pin_get_dt(spec);
        if (val != 1) {
            diag_result("TEST_A_GPIO", label, "FAIL_HIGH");
            gpio_pin_set_dt(spec, 0);
            continue;
        }

        /* Set LOW and verify. */
        gpio_pin_set_dt(spec, 0);
        k_busy_wait(10);
        val = gpio_pin_get_dt(spec);
        if (val != 0) {
            diag_result("TEST_A_GPIO", label, "FAIL_LOW");
            continue;
        }

        diag_result("TEST_A_GPIO", label, "PASS");
    }

    /* Test each row GPIO: configure as input with pull-down, verify reads LOW. */
    for (int i = 0; i < NUM_ROWS; i++) {
        const struct gpio_dt_spec *spec = &diag_rows[i];
        int err;

        snprintf(label, sizeof(label), "row%d", i);

        err = gpio_pin_configure_dt(spec, GPIO_INPUT | GPIO_PULL_DOWN);
        if (err) {
            diag_result("TEST_A_GPIO", label, "FAIL_CFG");
            continue;
        }

        k_busy_wait(50);
        int val = gpio_pin_get_dt(spec);
        if (val != 0) {
            diag_result("TEST_A_GPIO", label, "FAIL_FLOAT");
            continue;
        }

        diag_result("TEST_A_GPIO", label, "PASS");
    }
}

/* ── Test D: Pull-down integrity (all cols OFF, all rows should be LOW) ── */

static void test_d_pulldown(void) {
    char label[16];

    /* Ensure all cols are LOW. */
    for (int i = 0; i < NUM_COLS; i++) {
        gpio_pin_configure_dt(&diag_cols[i], GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&diag_cols[i], 0);
    }

    /* Ensure all rows are input with pull-down. */
    for (int i = 0; i < NUM_ROWS; i++) {
        gpio_pin_configure_dt(&diag_rows[i], GPIO_INPUT | GPIO_PULL_DOWN);
    }

    k_busy_wait(100);

    /* Check each row. */
    for (int i = 0; i < NUM_ROWS; i++) {
        snprintf(label, sizeof(label), "row%d", i);
        int val = gpio_pin_get_dt(&diag_rows[i]);
        diag_result("TEST_D_PULLDOWN", label, val == 0 ? "PASS" : "FAIL");
    }
}

/* ── Test B: Col-col short detection ── */

static void test_b_col_short(void) {
    char label[32];

    for (int i = 0; i < NUM_COLS; i++) {
        /* Set col[i] as output HIGH. */
        gpio_pin_configure_dt(&diag_cols[i], GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&diag_cols[i], 1);
        k_busy_wait(10);

        for (int j = 0; j < NUM_COLS; j++) {
            if (j == i) {
                continue;
            }
            /* Temporarily set col[j] as input with pull-down. */
            gpio_pin_configure_dt(&diag_cols[j], GPIO_INPUT | GPIO_PULL_DOWN);
            k_busy_wait(10);

            int val = gpio_pin_get_dt(&diag_cols[j]);
            snprintf(label, sizeof(label), "col%d-col%d", i, j);
            diag_result("TEST_B_COL_SHORT", label, val == 0 ? "PASS" : "FAIL");

            /* Restore col[j] as output LOW. */
            gpio_pin_configure_dt(&diag_cols[j], GPIO_OUTPUT_INACTIVE);
            gpio_pin_set_dt(&diag_cols[j], 0);
        }

        /* Restore col[i] as output LOW. */
        gpio_pin_set_dt(&diag_cols[i], 0);
        gpio_pin_configure_dt(&diag_cols[i], GPIO_OUTPUT_INACTIVE);
    }
}

/* ── Test C: Col-row short detection ── */

static void test_c_col_row_short(void) {
    char label[32];

    /* Ensure all rows are input with pull-down. */
    for (int i = 0; i < NUM_ROWS; i++) {
        gpio_pin_configure_dt(&diag_rows[i], GPIO_INPUT | GPIO_PULL_DOWN);
    }

    for (int i = 0; i < NUM_COLS; i++) {
        /* Set all cols LOW first. */
        for (int k = 0; k < NUM_COLS; k++) {
            gpio_pin_configure_dt(&diag_cols[k], GPIO_OUTPUT_INACTIVE);
            gpio_pin_set_dt(&diag_cols[k], 0);
        }

        /* Set col[i] HIGH. */
        gpio_pin_set_dt(&diag_cols[i], 1);
        k_busy_wait(20);

        for (int j = 0; j < NUM_ROWS; j++) {
            int val = gpio_pin_get_dt(&diag_rows[j]);
            snprintf(label, sizeof(label), "col%d-row%d", i, j);
            /* Key switch is open, so row should be LOW.
             * HIGH = either short or diode leakage (WARN, not definitive FAIL). */
            diag_result("TEST_C_COL_ROW", label, val == 0 ? "PASS" : "WARN");
        }

        gpio_pin_set_dt(&diag_cols[i], 0);
    }
}

/* ── Test E: Row-row short detection ── */

static void test_e_row_short(void) {
    char label[32];

    for (int i = 0; i < NUM_ROWS; i++) {
        /* Set row[i] as output HIGH. */
        gpio_pin_configure_dt(&diag_rows[i], GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&diag_rows[i], 1);
        k_busy_wait(10);

        for (int j = 0; j < NUM_ROWS; j++) {
            if (j == i) {
                continue;
            }
            /* row[j] should remain input with pull-down. */
            int val = gpio_pin_get_dt(&diag_rows[j]);
            snprintf(label, sizeof(label), "row%d-row%d", i, j);
            diag_result("TEST_E_ROW_SHORT", label, val == 0 ? "PASS" : "FAIL");
        }

        /* Restore row[i] as input with pull-down. */
        gpio_pin_set_dt(&diag_rows[i], 0);
        gpio_pin_configure_dt(&diag_rows[i], GPIO_INPUT | GPIO_PULL_DOWN);
        k_busy_wait(10);
    }
}

/* ── Restore kscan GPIO configuration ── */

static void restore_kscan_gpios(void) {
    /* Restore cols as output LOW (kscan default for col2row). */
    for (int i = 0; i < NUM_COLS; i++) {
        gpio_pin_configure_dt(&diag_cols[i], GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&diag_cols[i], 0);
    }
    /* Restore rows as input with pull-down. */
    for (int i = 0; i < NUM_ROWS; i++) {
        gpio_pin_configure_dt(&diag_rows[i], GPIO_INPUT | GPIO_PULL_DOWN);
    }
}

/* ── Main entry point ── */

void mk2_matrix_diag_run(void) {
    const struct device *kscan = DEVICE_DT_GET(KSCAN_NODE);

    diag_total = 0;
    diag_pass = 0;
    diag_warn = 0;
    diag_fail = 0;

    /* Determine side from col-offset presence in DTS. */
#if DT_NODE_HAS_PROP(DT_NODELABEL(default_transform), col_offset)
    const char *side = "R";
#else
    const char *side = "L";
#endif

    printk("[MATRIX_DIAG_START]\n");
    printk("side=%s,cols=%d,rows=%d\n", side, NUM_COLS, NUM_ROWS);

    /* Pause normal kscan scanning. */
    kscan_disable_callback(kscan);
    k_msleep(10);

    /* Run tests. */
    test_a_gpio();
    test_d_pulldown();
    test_b_col_short();
    test_c_col_row_short();
    test_e_row_short();

    /* Restore GPIO state and resume kscan. */
    restore_kscan_gpios();
    k_msleep(5);
    kscan_enable_callback(kscan);

    printk("SUMMARY,total=%d,pass=%d,warn=%d,fail=%d\n",
           diag_total, diag_pass, diag_warn, diag_fail);
    printk("[MATRIX_DIAG_END]\n");
}

/* ── USB serial command receiver (polling thread) ── */

#if DT_HAS_CHOSEN(zephyr_console)

#define CMD_BUF_SIZE 32

static void diag_cmd_thread_fn(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart)) {
        return;
    }

    char buf[CMD_BUF_SIZE];
    int pos = 0;

    while (1) {
        uint8_t c;
        int ret = uart_poll_in(uart, &c);
        if (ret == 0) {
            if (c == '\n' || c == '\r') {
                buf[pos] = '\0';
                if (pos > 0) {
                    if (strcmp(buf, "DIAG_MATRIX") == 0) {
                        mk2_matrix_diag_run();
                    }
                    /* Future: DIAG_KEY_START, DIAG_KEY_STOP, DIAG_BLE_STATUS */
                }
                pos = 0;
            } else if (pos < CMD_BUF_SIZE - 1) {
                buf[pos++] = c;
            }
        } else {
            k_msleep(50);
        }
    }
}

K_THREAD_DEFINE(mk2_diag_cmd_tid, 1024,
                diag_cmd_thread_fn, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

#endif /* DT_HAS_CHOSEN(zephyr_console) */
