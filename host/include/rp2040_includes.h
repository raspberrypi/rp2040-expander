/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RP2040_INCLUDES_H_
#define _RP2040_INCLUDES_H_

#include <hardware/platform_defs.h>
#include <hardware/regs/addressmap.h>
#include <hardware/address_mapped.h>

#include <hardware/structs/ioqspi.h>
#include <hardware/structs/clocks.h>
#include <hardware/structs/iobank0.h>
#include <hardware/structs/psm.h>
#include <hardware/structs/sio.h>
#include <hardware/structs/resets.h>
#include <hardware/structs/adc.h>
#include <hardware/structs/padsbank0.h>
#include <hardware/structs/rosc.h>
#include <hardware/structs/systick.h>
#include <hardware/structs/watchdog.h>
#include <hardware/structs/timer.h>


// Arm Cortex 'Debug Halting Control and Status Register'
#define DCB_DHCSR                   0xE000EDF0

// helper macro use when using target registers
#define _reg(r)                     ((volatile uintptr_t)&(r))

enum gpio_function {
    GPIO_FUNC_XIP = 0,
    GPIO_FUNC_SPI = 1,
    GPIO_FUNC_UART = 2,
    GPIO_FUNC_I2C = 3,
    GPIO_FUNC_PWM = 4,
    GPIO_FUNC_SIO = 5,
    GPIO_FUNC_PIO0 = 6,
    GPIO_FUNC_PIO1 = 7,
    GPIO_FUNC_GPCK = 8,
    GPIO_FUNC_USB = 9,
    GPIO_FUNC_NULL = 0x1f,
};

#ifndef bool_to_bit
#define bool_to_bit(x) ((uint32_t)!!(x))
#endif

#endif // _RP2040_INCLUDES_H_
