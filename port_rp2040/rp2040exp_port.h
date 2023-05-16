/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RPEXP_PORT_H_
#define _RPEXP_PORT_H_

// Needed for type definitions
#include <stdint.h>
#include <stdbool.h>

// SWD interface GPIO pin definitions, also as used by Raspberry Pi's Picoprobe
#define PIN_SWCLK                   2
#define PIN_SWDIO                   3

// Uncomment this to enable GPIO instrumentation of the SWD transactions
//#define DEBUG_SWD_ON_GPIOS

#ifdef DEBUG_SWD_ON_GPIOS
#define DBG_GPIO_SPI_CSN            12
#define DBG_GPIO_RXED               13
#endif

#endif // _RPEXP_PORT_H_
