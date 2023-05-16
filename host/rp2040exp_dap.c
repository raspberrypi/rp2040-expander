/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <rp2040exp_port.h>
#include <rp2040exp_dap.h>
#include <rp2040exp.h>
#include <rp2040exp_port_api.h>
#include <rp2040_includes.h>


 // Retries are needed if the DAP replies with a WAIT status
#define NUM_DAP_RETRIES             4

#define DAP_STATUS_OK               (1 << 0)
#define DAP_STATUS_WAIT             (1 << 1)
#define DAP_STATUS_FAULT            (1 << 2)

#define DLPIDR_TARGETID_0           0x01002927ul
#define DLPIDR_TARGETID_RESCUE_DP   0xf1002927ul

#define EXPECTED_DPIDR_0            0x0BC12477ul
#define EXPECTED_DPIDR_RESCUE_DP    0x10212927ul

#define NUM_ELES(a)                 (sizeof(a) / sizeof(*(a)))

typedef struct {
    uint8_t rd_wr;
    uint8_t dap_reg;
    uint32_t cdata;
} dap_reg_rd_wr_seq_t;

typedef struct {
    uint32_t last_csw;
    uint32_t last_tar;
} dap_reg_cache_t;

// store a little context to avoid unnecessary transactions
static dap_reg_cache_t dap_reg_cache;

// sequences of bits used to reset and control the SWD at 'line level'
static const uint8_t bit_seq_reset_to_dormant[] = {
    // reset sequence (56 > 50 :)
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // enter domant state
    0xbc, 0xe3,
    // reset sequence  54 + two idle
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f
};
#define NUM_SEQ_RESET_TO_DORMANT_BITS   128

static const uint8_t bit_seq_dormant_to_swd[] = {
    // Resync the LFSR (which is 7 bits, can't produce 8 1s)
    0xff,
    // A 0-bit, then 127 bits of LFSR output
    0x92, 0xf3, 0x09, 0x62,
    0x95, 0x2d, 0x85, 0x86,
    0xe9, 0xaf, 0xdd, 0xe3,
    0xa2, 0x0e, 0xbc, 0x19,
    // Four zero-bits, 8 bits of select sequence
    0xa0, 0x01
    // Total 148 bits (not the last 4!)
};
#define NUM_SEQ_DORMANT_TO_SWD_BITS     148

static const uint8_t bit_seq_line_reset[] = {
    // 50 * 1s and 2 * idle 0s.
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03
};
#define NUM_SEQ_LINE_RESET_BITS         52

static const dap_reg_rd_wr_seq_t use_rescue_dp_to_reset_chip[] = {

    {DAP_WR, DP_REG_ABORT,  0x1Eul},    // clean up any inital errors
    {DAP_WR, DP_REG_SELECT, 0},         // Init bank select, undefined before

    {DAP_RD, DP_REG_CTRL_STAT, 0},      // Read status

    // set CDBGPWRUPREQ which performs a chip reset in the rescue DP
    {DAP_WR, DP_REG_CTRL_STAT, (1ul << 28)},

    {DAP_RD, DP_REG_CTRL_STAT, 0},      // Read status
    {DAP_WR, DP_REG_CTRL_STAT, (0ul << 28)}     // clear reset
};

// Note: The reads in this are for debug (when enabled) and some are
// also required for correct operation, such as the AP_REG_IDR read.
static const dap_reg_rd_wr_seq_t setup_dap_instance_0[] = {

    {DAP_RD, DP_REG_DPIDR,  0},         // Read Device ID register

    {DAP_WR, DP_REG_ABORT,  0x1Eul},    // clean up any inital errors
    {DAP_WR, DP_REG_SELECT, 0},         // Init bank select, undefined before
    {DAP_RD, DP_REG_DPIDR,  0},         // Read Device ID register

    {DAP_WR, DP_REG_CTRL_STAT, 0x50000020ul},   // power up AP
    {DAP_WR, DP_REG_CTRL_STAT, 0x50000000ul},
    {DAP_RD, DP_REG_CTRL_STAT, 0},      // Read status

    {DAP_WR, DP_REG_SELECT, DP_BANK_DLPIDR},    // Read Device instance register
    {DAP_RD, DP_REG_DLPIDR, 0},

    {DAP_WR, DP_REG_SELECT, 0x000000F0ul},  // AP bank to access the AP IDR
    {DAP_RD, AP_REG_IDR,    0},  // Dummy read
    {DAP_RD, DP_REG_RDBUF,  0},  // AP ID

    {DAP_WR, DP_REG_SELECT, 0}  // Bank selects back to 0
};


static void send_dap_header(uint8_t rnw_dap_and_reg) {

    uint8_t parity =
        ((rnw_dap_and_reg >> 1) & 1) ^
        ((rnw_dap_and_reg >> 2) & 1) ^
        ((rnw_dap_and_reg >> 3) & 1) ^
        ((rnw_dap_and_reg >> 4) & 1);

    uint8_t header =
        1u << 0             | // Start
        rnw_dap_and_reg     | // APnDP, RnW, reg addr: bits 1 to 4
        parity << 5         | // Parity
        0u << 6             | // Stop
        1u << 7;              // Park

    port_swd_put_bits(&header, 8);

    port_swd_hiz_clocks(1);  // always send a turnaround after a header
}


static rpexp_err_t send_dap_header_read_status(uint8_t rnw_dap_and_reg) {

    uint8_t i;
    uint8_t status;

    for (i = 0; i < NUM_DAP_RETRIES; i++) {

        send_dap_header(rnw_dap_and_reg);

        port_swd_get_bits(&status, 3);

        switch (status) {
            case DAP_STATUS_OK:
                return RPEXP_OK;
                break;

            // FIXME HERE TODO The needs to be explicitly tested, I've not yet seen it!
            case DAP_STATUS_WAIT:
                port_swd_hiz_clocks(1);
                continue;
                break;

            case DAP_STATUS_FAULT:
                port_swd_hiz_clocks(1);
                return RPEXP_ERR_DAP_FAULT;
                break;

            default:
                port_swd_hiz_clocks(1);
                // Guess at not connected
                return RPEXP_ERR_DAP_DISCONNECTED;
                break;
        }
    }

    return RPEXP_ERR_DAP_TIMEOUT;
}


static void send_dap_word(uint32_t dapw) {

    uint8_t parity = 0;

    for (unsigned int i = 0; i < 32; i++) {
        parity ^= (dapw >> i) & 0x1;
    }

    port_swd_put_bits((uint8_t *)&dapw, 32);

    port_swd_put_bits(&parity, 1);
}


static rpexp_err_t read_dap_word(uint32_t *pdata) {
    uint32_t parity;
    uint32_t mask;

    port_swd_get_bits((uint8_t *)pdata, 32);

    // calculate parity
    for (parity = 0ul, mask = 1ul; mask != 0; mask <<= 1) {
        if (mask & *pdata) {
            parity ^= 1;
        }
    }

    // read parity from target
    port_swd_get_bits((uint8_t *)&mask, 1);

    if (parity != mask) {
        return RPEXP_ERR_DAP_PARITY;
    }

    return RPEXP_OK;
}


static void send_targetselect(uint32_t targetsel_id) {

    send_dap_header(DAP_WR | DP_REG_TARGETSEL);

    // There is no useful response to TARGETSEL, ignore the
    // 'status' and send a turnaround since sending more
    port_swd_hiz_clocks(3 + 1);

    send_dap_word(targetsel_id);
}

static void send_reset_to_dormant(void) {
    port_swd_put_bits(bit_seq_reset_to_dormant, NUM_SEQ_RESET_TO_DORMANT_BITS);
}

static void send_dormant_to_swd(void) {
    port_swd_put_bits(bit_seq_dormant_to_swd, NUM_SEQ_DORMANT_TO_SWD_BITS);
}

static void swd_line_reset(void) {
    port_swd_put_bits(bit_seq_line_reset, NUM_SEQ_LINE_RESET_BITS);
}


static rpexp_err_t table_driven_dap_reg_setup(const dap_reg_rd_wr_seq_t *dap_rd_wr_seq, uint32_t table_length) {

    rpexp_err_t rpexp_err = RPEXP_OK;
    uint32_t dummy;

    for (unsigned int i = 0; i < table_length && rpexp_err == RPEXP_OK; i++) {

        if (dap_rd_wr_seq[i].rd_wr == DAP_WR) {
            rpexp_err = dap_write(dap_rd_wr_seq[i].dap_reg, dap_rd_wr_seq[i].cdata);
        } else {
            rpexp_err = dap_read(dap_rd_wr_seq[i].dap_reg, &dummy);
        }
    }

    return rpexp_err;
}


static void reset_register_caching(void) {

    dap_reg_cache.last_csw = (uint32_t) -1;
    dap_reg_cache.last_tar = (uint32_t) -1;
}


static rpexp_err_t connect_to_a_dp_instance(uint32_t target_id, uint32_t expected_dpidr) {

    rpexp_err_t rpexp_err;
    uint32_t read_dpidr;

    reset_register_caching();  // caching is only one DP instance deep

    send_reset_to_dormant();
    send_dormant_to_swd();

    swd_line_reset();  // always a line reset before a target select

    send_targetselect(target_id);

    rpexp_err = dap_read(DP_REG_DPIDR, &read_dpidr);

    if ((rpexp_err == RPEXP_OK) && (read_dpidr != expected_dpidr)) {
        rpexp_err = RPEXP_ERR_DAP_TARGET;
    }

    return rpexp_err;
}

//----------------------------------------------------------------------------

// Module API

rpexp_err_t dap_if_init(void) {

    return port_swd_init_gpios();
}


rpexp_err_t dap_reset_chip_halt_cpus(void) {

    rpexp_err_t rpexp_err;

    rpexp_err = connect_to_a_dp_instance(DLPIDR_TARGETID_RESCUE_DP, EXPECTED_DPIDR_RESCUE_DP);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = table_driven_dap_reg_setup(use_rescue_dp_to_reset_chip, NUM_ELES(use_rescue_dp_to_reset_chip));
    }

    return rpexp_err;
}


rpexp_err_t dap_initial_chip_config(void) {

    rpexp_err_t rpexp_err;

    rpexp_err = connect_to_a_dp_instance(DLPIDR_TARGETID_0, EXPECTED_DPIDR_0);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = table_driven_dap_reg_setup(setup_dap_instance_0, NUM_ELES(setup_dap_instance_0));
    }

    if (rpexp_err == RPEXP_OK) {
        // This can't be done until after the above
        // The top 16-bits are a magic unlock pattern, the bottom 2 bits halt the CPU
        rpexp_err = rpexp_write32(DCB_DHCSR,  0xA05F0003ul);
    }

    return rpexp_err;
}


rpexp_err_t dap_set_target_rd_wr_address(uint32_t address, auto_inc_t inc_mode) {

    uint32_t new_csw;
    rpexp_err_t rpexp_err = RPEXP_OK;

  //rpexp_err = dap_write(DP_REG_SELECT, 0);  // NOTE: We leave the Bank selects at 0

    if (inc_mode == NO_AUTO_INC) {
        new_csw = 0xA2000002ul;   // Word read, no inc
    } else {
        new_csw = 0xA2000012ul;   // Word read, inc +1 word
    }

    if (dap_reg_cache.last_csw != new_csw) {
        rpexp_err = dap_write(AP_REG_CSW, new_csw);
        dap_reg_cache.last_csw = new_csw;
    }

    if (rpexp_err == RPEXP_OK) {
        if (dap_reg_cache.last_tar != address) {
            rpexp_err = dap_write(AP_REG_TAR, address);  // set address
            dap_reg_cache.last_tar = address;
        }
    }

    return rpexp_err;
}


// SWD transactions - *everything* is built on top of these
rpexp_err_t dap_read(uint8_t dap_and_reg, uint32_t *pdata) {

    rpexp_err_t rpexp_err = send_dap_header_read_status(DAP_RD | dap_and_reg);

    if (rpexp_err != RPEXP_OK) {
        *pdata = 0;
        return rpexp_err;
    }

    rpexp_err = read_dap_word(pdata);

    // Turnaround for next packet header to be sent
    port_swd_hiz_clocks(1);

    return rpexp_err;
}


rpexp_err_t dap_write(uint8_t dap_and_reg, uint32_t data) {

    rpexp_err_t rpexp_err = send_dap_header_read_status(DAP_WR | dap_and_reg);

    if (rpexp_err != RPEXP_OK) {
        return rpexp_err;
    }

    port_swd_hiz_clocks(1);

    send_dap_word(data);

    return RPEXP_OK;
}
