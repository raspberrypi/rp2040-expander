/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RP2040EXP_PORT_API_H_
#define _RP2040EXP_PORT_API_H_

#define NUM_ELES(a)                 (sizeof(a) / sizeof(*(a)))


rpexp_err_t port_init_swd_gpios(void);

void port_set_swdio_as_output(bool yes);
void port_set_swdio(bool high);
void port_set_swclk(bool high);

bool port_get_swdio(void);

void port_delay_half_clock(void);

#ifdef DEBUG_SWD_ON_GPIOS  // debug GPIO helpers
void port_dbg_gpio_init(void);
void port_dbg_gpio_set(bool pin, bool high);
#endif


uint64_t port_get_time_us_64(void);
void port_sleep_us_32(uint32_t time_us);


#endif  // _RP2040EXP_PORT_API_H_
