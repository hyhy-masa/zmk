/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#if IS_ENABLED(CONFIG_MK2_KSCAN_DROP_STATS)

/* Prints the [KGUARD] block for THIS half. Called from the central's [MK2_DIAG]
 * footer and from the peripheral's periodic [NGUARD] dump, so the same code
 * reports from both halves and the two readouts can be compared side by side. */
void mk2_kscan_stats_dump(void);

#else

static inline void mk2_kscan_stats_dump(void) {}

#endif /* CONFIG_MK2_KSCAN_DROP_STATS */

#ifdef __cplusplus
}
#endif
