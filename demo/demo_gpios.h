/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DEMO_GPIOS_H_
#define _DEMO_GPIOS_H_

#define GPIO_UART_0_TXD     0
#define GPIO_UART_0_RXD     1

#define GPIO_RENC_0         2
#define GPIO_RENC_1         3
#define GPIO_RENC_2         4

// VITAL: The code in the stepper module assumes the GPIOs for each motor are *contiguous*!!
#define GPIO_STEPPER_0_0    6
#define GPIO_STEPPER_0_1    7
#define GPIO_STEPPER_0_2    8
#define GPIO_STEPPER_0_3    9

#define GPIO_EXTRA_LED_PIN  15

// VITAL: The code in the stepper module assumes the GPIOs for each motor are *contiguous*!!
#define GPIO_STEPPER_1_0    16
#define GPIO_STEPPER_1_1    17
#define GPIO_STEPPER_1_2    18
#define GPIO_STEPPER_1_3    19

#define GPIO_CLOCKOUT       21

#define GPIO_7SEG_CLK       22

#define GPIO_PICO_LED       25

#define GPIO_26_IS_ADC_0    26
#define GPIO_ADC_0          (GPIO_26_IS_ADC_0 - 26)

#define GPIO_7SEG_DAT       27
#define GPIO_7SEG_CS        28

#endif // _DEMO_GPIOS_H_