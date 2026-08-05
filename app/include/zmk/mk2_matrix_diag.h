/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * MK2 Matrix GPIO diagnostics. Disabled unless CONFIG_MK2_MATRIX_DIAG=y.
 *
 * Purpose: QC check for keyboard matrix wiring after FW flash.
 * Tests GPIO output/input function, detects col-col/row-row/col-row shorts,
 * and verifies pull-down integrity — all without pressing any key.
 * Results are printed via printk in [MATRIX_DIAG_START]/[MATRIX_DIAG_END]
 * markers for host-side parsing (matrix_diag.py).
 *
 * Adds no GATT characteristics, no settings_reset required.
 * When the config is off, every call compiles to a no-op.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_MK2_MATRIX_DIAG)

/**
 * Run all matrix diagnostic tests and print results via printk.
 * Temporarily disables kscan scanning during the test (~200ms).
 *
 * Output format:
 *   [MATRIX_DIAG_START]
 *   side=R,cols=5,rows=4
 *   TEST_A_GPIO,col0,PASS
 *   ...
 *   SUMMARY,total=XX,pass=XX,warn=XX,fail=XX
 *   [MATRIX_DIAG_END]
 */
void mk2_matrix_diag_run(void);

#else

static inline void mk2_matrix_diag_run(void) {}

#endif /* CONFIG_MK2_MATRIX_DIAG */

#ifdef __cplusplus
}
#endif
