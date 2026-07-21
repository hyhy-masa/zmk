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

#if IS_ENABLED(CONFIG_MK2_SPLIT_POS_DROP_STATS)

void mk2_split_pos_drop_dump(void);
void mk2_split_pos_drop_reset(void);

#else

static inline void mk2_split_pos_drop_dump(void) {}
static inline void mk2_split_pos_drop_reset(void) {}

#endif /* CONFIG_MK2_SPLIT_POS_DROP_STATS */

#ifdef __cplusplus
}
#endif
