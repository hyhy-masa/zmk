/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

enum zmk_activity_state { ZMK_ACTIVITY_ACTIVE, ZMK_ACTIVITY_IDLE, ZMK_ACTIVITY_SLEEP };

enum zmk_activity_state zmk_activity_get_state(void);

/**
 * Get the configured sleep timeout in milliseconds.
 * @return 0 if sleep is disabled, otherwise the sleep timeout in milliseconds.
 */
uint32_t zmk_activity_get_sleep_ms(void);
/**
 * Get the configured idle timeout in milliseconds.
 * @return The idle timeout in milliseconds.
 */
uint32_t zmk_activity_get_idle_ms(void);

/**
 * Set the sleep timeout in milliseconds.
 * @param sleep_ms The sleep timeout in milliseconds. Set to 0 to disable sleep.
 */
bool zmk_activity_set_sleep_ms(uint32_t sleep_ms);
/**
 * Set the idle timeout in milliseconds.
 * @param idle_ms The idle timeout in milliseconds. Set to 0 to disable idle state.
 */
bool zmk_activity_set_idle_ms(uint32_t idle_ms);
