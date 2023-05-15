/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Helper functions for an RPi host
 */

#include <picoexp_port.h>
#include <picoexp.h>
#include <picoexp_port_api.h>

// This code uses the 'libgpiod' library to access the Pi's GPIOs
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>


static const char *chipname = "gpiochip0";

static struct gpiod_chip *chip = NULL;

static struct gpiod_line *hswclk;
static struct gpiod_line *hswdio;

static pexp_err_t init_swd_gpios(void) {

    if (!chip) {
        chip = gpiod_chip_open_by_name(chipname);
        if (!chip) {
            return PEXP_ERR_NO_SWDBB;
        }
    }

    hswclk = gpiod_chip_get_line(chip, PIN_SWCLK);
    hswdio = gpiod_chip_get_line(chip, PIN_SWDIO);

    if (!hswclk || !hswdio) {
        return PEXP_ERR_NO_SWDBB;
    }

    if (0 != gpiod_line_request_output(hswclk, "swclk", 0)) {
        return PEXP_ERR_NO_SWDBB;
    }

    // initially this is an output although it swaps during operation
    if (0 != gpiod_line_request_output(hswdio, "swdio", 0)) {
        return PEXP_ERR_NO_SWDBB;
    }

    return PEXP_OK;
}


static void inline set_swdio_as_output(bool out) {
    gpiod_line_release(hswdio);
    if (out) {
        (void) gpiod_line_request_output(hswdio, "swdio", 0);
    } else {
        (void) gpiod_line_request_input(hswdio, "swdio");
    }
}


static void inline set_swdio(bool swdio) {
    (void) gpiod_line_set_value(hswdio, swdio);
}


static void inline set_swclk(bool swclk) {
    (void) gpiod_line_set_value(hswclk, swclk);
}


static bool inline get_swdio(void) {
    return gpiod_line_get_value(hswdio);
}


static void inline delay_half_clock(void) {
    // libgpiod is complicated enough so we don't need any [extra] delay here
}


#ifdef DEBUG_SWD_ON_GPIOS  // debug GPIO helpers
static struct gpiod_line *hspicsn = NULL;
static struct gpiod_line *hrxed   = NULL;

static void inline dbg_gpio_init(void) {

    if (!chip) {
        chip = gpiod_chip_open_by_name(chipname);
        if (!chip) {
            return;
        }
    }

    hspicsn = gpiod_chip_get_line(chip, DBG_GPIO_SPI_CSN);
    hrxed   = gpiod_chip_get_line(chip, DBG_GPIO_RXED);

    if (!hspicsn || !hrxed) {
        return;
    }

    if (0 != gpiod_line_request_output(hspicsn, "spicsn", 1)) {
        return;
    }

    if (0 != gpiod_line_request_output(hrxed, "rxed", 0)) {
        return;
    }
}


static void inline dbg_gpio_set(bool pin, bool high) {
    struct gpiod_line *hpin = NULL;

    if (pin == DBG_GPIO_SPI_CSN) {
        hpin = hspicsn;
    } else if (pin == DBG_GPIO_RXED) {
        hpin = hrxed;
    }

    if (!hpin) {
        return;
    }

    if (high) {
        (void) gpiod_line_set_value(hpin, 1);
    } else {
        (void) gpiod_line_set_value(hpin, 0);
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


// For a unix host, mock up RP2040 SDK timing function
uint64_t port_get_time_us_64(void) {

    struct timespec ts;

    if (0 != clock_gettime(CLOCK_MONOTONIC, &ts)){
        return (uint64_t) 0ull;
    }

    uint64_t time_us64 = (uint64_t) 1000000ull * (uint64_t) ts.tv_sec;
    time_us64 += (uint64_t) ts.tv_nsec / (uint64_t) 1000ull;

    return time_us64;
}
