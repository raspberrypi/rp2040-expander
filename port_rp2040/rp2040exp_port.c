/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Helper functions for an RP2040 host
 */

#include "pico/stdlib.h"

#include <rp2040exp_port.h>
#include <rp2040exp.h>
#include <rp2040exp_port_api.h>


// Timing definitions
#define HALF_CLOCK_CYCLE_US     ((1000 / SWD_CLOCK_RATE_KHZ) / 2)
static_assert(HALF_CLOCK_CYCLE_US != 0, "SWD clock rate is too fast for: sleep_us()");


// private helper functions for an RP2040 host (using the Pico-SDK)
static rpexp_err_t init_swd_gpios(void) {
    gpio_init(PIN_SWCLK);
    gpio_init(PIN_SWDIO);

    gpio_put(PIN_SWCLK, 0);
    gpio_put(PIN_SWDIO, 0);

    gpio_set_dir(PIN_SWCLK, GPIO_OUT);
    gpio_set_dir(PIN_SWDIO, GPIO_OUT);

    return RPEXP_OK;
}   

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

static void inline delay_half_clock(void) {
  //sleep_us(HALF_CLOCK_CYCLE_US);
}


#ifdef DEBUG_SWD_ON_GPIOS  // debug GPIO helpers
static void inline dbg_gpio_init(void) {

    gpio_init(DBG_GPIO_SPI_CSN);
    gpio_init(DBG_GPIO_RXED);

    gpio_put(DBG_GPIO_SPI_CSN, 1);
    gpio_put(DBG_GPIO_RXED, 0);

    gpio_set_dir(DBG_GPIO_SPI_CSN, GPIO_OUT);
    gpio_set_dir(DBG_GPIO_RXED, GPIO_OUT);
}

static void inline dbg_gpio_set(bool pin, bool high) {
    if (high) {
        gpio_put(pin, 1);
    } else {
        gpio_put(pin, 0);
    }
}
#endif  // DEBUG_SWD_ON_GPIOS

//----------------------------------------------------------------------------

static const swdbb_helpers_t swdbb_helpers =
{
    .init_swd_gpios      = init_swd_gpios,
    .set_swdio_as_output = set_swdio_as_output,
    .set_swdio           = set_swdio,
    .set_swclk           = set_swclk,
    .get_swdio           = get_swdio,
    .delay_half_clock    = delay_half_clock

#ifdef DEBUG_SWD_ON_GPIOS  // debug GPIO helpers
  , .dbg_gpio_init      = dbg_gpio_init
  , .dbg_gpio_set       = dbg_gpio_set
#endif
};

//----------------------------------------------------------------------------

// Module API

// The function returns the address of the structure with the SWD helper function pointers in it.
const swdbb_helpers_t *port_get_swdbb_helpers(void) {
    return &swdbb_helpers;
}


// Use SDK timer read function
uint64_t port_get_time_us_64(void) {
    return time_us_64();
}


void port_sleep_us_32(uint32_t time_us) {
    sleep_us((uint64_t) time_us);
}
