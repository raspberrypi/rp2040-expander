/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RP2040EXP_DAP_H_
#define _RP2040EXP_DAP_H_

#include <rp2040exp_port.h>
#include <rp2040exp.h>


// DAP control field and register address constants are defined below
// NOTE:  The bits used are straight from the ADIv6 spec so can be used
// directly to assemble the SWD header for transmission.
#define DAP_DP              (0 << 1)
#define DAP_AP              (1 << 1)

#define DAP_WR              (0 << 2)
#define DAP_RD              (1 << 2)

#define DP_REG_DPIDR        (DAP_DP | (0 << 3))
#define DP_REG_ABORT        (DAP_DP | (0 << 3))
#define DP_REG_CTRL_STAT    (DAP_DP | (1 << 3))
#define DP_REG_DLCR         (DAP_DP | (1 << 3))
#define DP_REG_TARGETID     (DAP_DP | (1 << 3))
#define DP_REG_DLPIDR       (DAP_DP | (1 << 3))
#define DP_REG_EVENTSTAT    (DAP_DP | (1 << 3))
#define DP_REG_RESEND       (DAP_DP | (2 << 3))
#define DP_REG_SELECT       (DAP_DP | (2 << 3))
#define DP_REG_RDBUF        (DAP_DP | (3 << 3))
#define DP_REG_TARGETSEL    (DAP_DP | (3 << 3))

#define DP_BANK_CTRL_STAT   0
#define DP_BANK_DLCR        1
#define DP_BANK_TARGETID    2
#define DP_BANK_DLPIDR      3
#define DP_BANK_EVENTSTAT   4

#define AP_REG_CSW          (DAP_AP | (0 << 3))
#define AP_REG_TAR          (DAP_AP | (1 << 3))
#define AP_REG_TAR64        (DAP_AP | (2 << 3))
#define AP_REG_DRW          (DAP_AP | (3 << 3))
#define AP_REG_CFG          (DAP_AP | (0 << 3))
#define AP_REG_BASE         (DAP_AP | (2 << 3))
#define AP_REG_IDR          (DAP_AP | (3 << 3))
#define AP_REG_IDR_WR       (DAP_AP | (3 << 3))

#define AP_BANK_CSW         (0 << 4)
#define AP_BANK_TAR         (0 << 4)
#define AP_BANK_DRW         (0 << 4)
#define AP_BANK_BDx         (1 << 4)
#define AP_BANK_CFG         (0xf << 4)
#define AP_BANK_BASE        (0xf << 4)
#define AP_BANK_IDR         (0xf << 4)

typedef enum {
    NO_AUTO_INC = 0,
    USE_AUTO_INC
} auto_inc_t;

// initialisation functions
rpexp_err_t dap_if_init(void);
rpexp_err_t dap_reset_chip_halt_cpus(void);
rpexp_err_t dap_initial_chip_config(void);
rpexp_err_t dap_set_target_rd_wr_address(uint32_t address, auto_inc_t inc_mode);

// DAP level transactions via SWD - *everything* is built on top of these
rpexp_err_t dap_write(uint8_t dap_and_reg, uint32_t data);
rpexp_err_t dap_read(uint8_t dap_and_reg, uint32_t *pdata);

#endif // _RP2040EXP_DAP_H_