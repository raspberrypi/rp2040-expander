/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Port specific functions for an RP2040 host
 */

#include "pico/stdlib.h"

#include <rp2040exp_port.h>
#include <rp2040exp.h>
#include <rp2040exp_port_api.h>


// Timing definitions
#define HALF_CLOCK_CYCLE_US     ((1000 / SWD_CLOCK_RATE_KHZ) / 2)
static_assert(HALF_CLOCK_CYCLE_US != 0, "SWD clock rate is too fast for: sleep_us()");


static void inline set_swdio_as_output(bool out) {
    if (out) {
        gpio_set_dir(PIN_SWDIO, GPIO_OUT);
    } else {
        gpio_set_dir(PIN_SWDIO, GPIO_IN);
    }
}   

static void inline set_swdio(bool swdio) {
    if (swdio) {
        gpio_put(PIN_SWDIO, 1);
    } else {
        gpio_put(PIN_SWDIO, 0);
    }
}    

static void inline set_swclk(bool swclk) {
    if (swclk) {
        gpio_put(PIN_SWCLK, 1);
    } else {
        gpio_put(PIN_SWCLK, 0);
    }
}    

static bool inline get_swdio(void) {
    return gpio_get(PIN_SWDIO);
}

// things are slow enough that we don't need any [extra] delay here
#define delay_half_clock() ((void) 0)

//----------------------------------------------------------------------------

// Module API

rpexp_err_t port_swd_init_gpios(void) {

    gpio_init(PIN_SWCLK);
    gpio_init(PIN_SWDIO);

    gpio_put(PIN_SWCLK, 0);
    gpio_put(PIN_SWDIO, 0);

    gpio_set_dir(PIN_SWCLK, GPIO_OUT);
    gpio_set_dir(PIN_SWDIO, GPIO_OUT);

    return RPEXP_OK;
}


void port_swd_put_bits(const uint8_t *txb, int n_bits) {
    uint8_t shifter;

    set_swdio_as_output(1);

    for (unsigned int i = 0; i < n_bits; i++) {
        if (i % 8 == 0) {
            shifter = txb[i / 8];
        } else {
            shifter >>= 1;
        }

        set_swdio(shifter & 1u);
        delay_half_clock();
        set_swclk(1);
        delay_half_clock();
        set_swclk(0);
    }
}


void port_swd_get_bits(uint8_t *rxb, int n_bits) {
    uint8_t shifter;

    set_swdio_as_output(0);

    for (unsigned int i = 0; i < n_bits; i++) {

        delay_half_clock();
        uint8_t sample = get_swdio();
        set_swclk(1);

        delay_half_clock();
        set_swclk(0);

        shifter >>= 1;
        if (sample) {
            shifter |= (1 << 7);
        }
        if (i % 8 == 7)
            rxb[i / 8] = shifter;
    }

    if (n_bits % 8 != 0) {
        rxb[n_bits / 8] = shifter >> (8 - n_bits % 8);
    }
}


void port_swd_hiz_clocks(int n_bits) {

    set_swdio_as_output(0);

    for (unsigned int i = 0; i < n_bits; i++) {
        delay_half_clock();
        set_swclk(1);
        delay_half_clock();
        set_swclk(0);
    }
}


// Use SDK timer read function
uint64_t port_get_time_us_64(void) {
    return time_us_64();
}
