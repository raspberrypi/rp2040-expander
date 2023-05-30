/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DEMO_STEPPER_H_
#define _DEMO_STEPPER_H_

#include <rp2040exp_port.h>
#include <rp2040exp.h>

#define NUM_OF_MOTORS               2

rpexp_err_t stepper_init(const int8_t *pmotors);
rpexp_err_t stepper_step(const uint8_t motor, int16_t count);

#endif // _DEMO_STEPPER_H_
