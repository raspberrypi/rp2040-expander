/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RP2040EXP_PORT_API_H_
#define _RP2040EXP_PORT_API_H_

rpexp_err_t port_swd_init_gpios(void);
void port_swd_put_bits(const uint8_t *txb, int n_bits);
void port_swd_get_bits(uint8_t *rxb, int n_bits);
void port_swd_hiz_clocks(int n_bits);
uint64_t port_get_time_us_64(void);

#endif  // _RP2040EXP_PORT_API_H_
