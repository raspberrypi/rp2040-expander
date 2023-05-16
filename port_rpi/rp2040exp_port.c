/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Port specific functions for an RPi host
 */

#include <rp2040exp_port.h>
#include <rp2040exp.h>
#include <rp2040exp_port_api.h>

// This code uses the 'libgpiod' library to access the Pi's GPIOs
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>


static const char *chipname = "gpiochip0";
static struct gpiod_chip *chip = NULL;
static struct gpiod_line *hswclk;
static struct gpiod_line *hswdio;


static void inline swdio_as_output(bool out) {
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

// libgpiod is complicated enough so we don't need any [extra] delay here
#define delay_half_clock() ((void) 0)

//----------------------------------------------------------------------------

// Module API

rpexp_err_t port_swd_init_gpios(void) {

    if (!chip) {
        chip = gpiod_chip_open_by_name(chipname);
        if (!chip) {
            return RPEXP_ERR_NO_SWDBB;
        }
    }

    hswclk = gpiod_chip_get_line(chip, PIN_SWCLK);
    hswdio = gpiod_chip_get_line(chip, PIN_SWDIO);

    if (!hswclk || !hswdio) {
        return RPEXP_ERR_NO_SWDBB;
    }

    if (0 != gpiod_line_request_output(hswclk, "swclk", 0)) {
        return RPEXP_ERR_NO_SWDBB;
    }

    // initially this is an output although it swaps during operation
    if (0 != gpiod_line_request_output(hswdio, "swdio", 0)) {
        return RPEXP_ERR_NO_SWDBB;
    }

    return RPEXP_OK;
}


void port_swd_put_bits(const uint8_t *txb, int n_bits) {
    uint8_t shifter;

    swdio_as_output(1);

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

    swdio_as_output(0);

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

    swdio_as_output(0);

    for (unsigned int i = 0; i < n_bits; i++) {
        delay_half_clock();
        set_swclk(1);
        delay_half_clock();
        set_swclk(0);
    }
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
