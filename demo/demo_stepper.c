/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef PICO_BUILD
#include "pico/stdlib.h"
#endif
#ifdef __linux__
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#endif

#include <stdio.h>
#include <rp2040exp_port.h>
#include <rp2040exp.h>
#include <inttypes.h>
#include <rp2040exp_port_api.h>
#include "demo_stepper.h"


#define NUM_OF_MOTOR_STEPS          4
#define NUM_DRIVES_PER_MOTOR        4
#define STEP_TIME_US                100000ul

typedef struct {
    uint8_t gpios[NUM_DRIVES_PER_MOTOR];
} motor_gpios_t;

const uint8_t unipolar_step_seq[NUM_OF_MOTOR_STEPS] = { 9, 3, 6, 12 };

// VITAL:  The code in this module assumes the GPIOs for each motor are contiguous!!
const uint8_t motor_gpios_lsb[NUM_OF_MOTORS] = { 8, 12, 16 };
static int8_t motor_poss[NUM_OF_MOTORS];


rpexp_err_t stepper_init(const int8_t *pmotors) {

    if (!pmotors) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = rpexp_gpio_block_enable(true);

    for ( ; *pmotors >= 0 && rpexp_err == RPEXP_OK; pmotors++) {

        if (*pmotors >= NUM_OF_MOTORS) {
            return RPEXP_ERR_API_ARG;
        }

        motor_poss[*pmotors] = 0;

        uint32_t gpio = motor_gpios_lsb[*pmotors];

        // The code in this module assumes the GPIOs for each motor are contiguous!!
        for (int p = 0; p < 4; p++, gpio += 1ul) {

            rpexp_err = rpexp_gpio_init(gpio);

            if (rpexp_err == RPEXP_OK) {
                rpexp_err = rpexp_gpio_set_dir(gpio, GPIO_DIR_OUT);
            }

            if (rpexp_err == RPEXP_OK) {
                rpexp_err = rpexp_gpio_clr(gpio);
            }
        }
    }

    return rpexp_err;
}


rpexp_err_t stepper_step(const uint8_t motor, int16_t count) {

    if (motor >= NUM_OF_MOTORS) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = RPEXP_OK;
    int8_t step = 1;

    if (count < 0) {
        step = -1;
        count = 0 - count;
    }

    uint8_t gpios_shift = motor_gpios_lsb[motor];
    uint32_t gpios_mask = 0xFul << gpios_shift;

    for ( ; count && rpexp_err == RPEXP_OK; count--) {

        motor_poss[motor] += step;
        motor_poss[motor] %= NUM_OF_MOTOR_STEPS;

        uint32_t drive_bits = unipolar_step_seq[motor_poss[motor]];
        drive_bits <<= gpios_shift;

        uint32_t gpios = rpexp_gpio_get_all();
        if (gpios == (uint32_t)-1) {
            return PREXP_ERR_READ;
        }

        gpios &= ~gpios_mask;
        gpios |= drive_bits;

        rpexp_err = rpexp_gpio_set_all(gpios);
        port_sleep_us_32(STEP_TIME_US);

        gpios &= gpios_mask;
        rpexp_err = rpexp_gpio_set_all(gpios);
    }

    return rpexp_err;
}
