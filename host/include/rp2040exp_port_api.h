/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RP2040EXP_PORT_API_H_
#define _RP2040EXP_PORT_API_H_

typedef struct {

    rpexp_err_t (*init_swd_gpios)(void);

    void (*set_swdio_as_output)(bool yes);
    void (*set_swdio)(bool high);
    void (*set_swclk)(bool high);

    bool (*get_swdio)(void);

    void (*delay_half_clock)(void);

#ifdef DEBUG_SWD_ON_GPIOS  // debug GPIO helpers
    void (*dbg_gpio_init)(void);
    void (*dbg_gpio_set)(bool pin, bool high);
#endif

} swdbb_helpers_t;


const swdbb_helpers_t *port_get_swdbb_helpers(void);

uint64_t port_get_time_us_64(void);

#endif  // _RP2040EXP_PORT_API_H_
