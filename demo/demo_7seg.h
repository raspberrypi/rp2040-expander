/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DEMO_7SEG_H_
#define _DEMO_7SEG_H_

#include <rp2040exp_port.h>
#include <rp2040exp.h>

void s7_led_display_init(void);
void s7_insert_char_rh(uint8_t c);
void s7_print_text_string(const uint8_t *p);
void s7_print_decimal_number(uint32_t base10number);
void s7_print_hex_number(uint32_t number);
void s7_print_decimal_points(uint8_t dps);

#endif // _DEMO_7SEG_H_