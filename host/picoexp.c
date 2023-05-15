/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <picoexp_dap.h>
#include <picoexp.h>
#include <rp2040_includes.h>
#include <picoexp_port_api.h>

#define DAP_BLOCK_XFER_MAX_WORD_LENGTH  256

#define RESET_ESCAPE_COUNT              10
#define ADC_ESCAPE_COUNT                10

#define ROSC_MAX_FREQ                   125000000ul  // could probably be more


// struct for ROSC frequency measurement
typedef struct {
    uint64_t time_us;
    uint64_t tick_count;
} rosc_time_n_count_t;

static rosc_time_n_count_t snapshot_rosc_time_n_count = { 0 };


static pexp_err_t reset_blocks(uint32_t mask) {

    return pexp_hw_set_bits(_reg(resets_hw->reset), mask);
}


static pexp_err_t unreset_blocks(uint32_t mask) {

    return pexp_hw_clear_bits(_reg(resets_hw->reset), mask);
}


static pexp_err_t unreset_blocks_wait(uint32_t mask) {

    pexp_err_t pexp_err = unreset_blocks(mask);

    for (uint32_t escape = 0; pexp_err == PEXP_OK; escape++) {

        uint32_t reset_done;

        pexp_err = pexp_read32(_reg(resets_hw->reset_done), &reset_done);

        if (pexp_err == PEXP_OK && (reset_done & mask)) {
            break;
        }
        if (escape > RESET_ESCAPE_COUNT) {
            pexp_err = PEXP_ERR_RESET_TIMEOUT;
        }
    }

    return pexp_err;
}


static pexp_err_t peripheral_enable(bool enable, uint32_t mask) {

    if (enable) {
        return unreset_blocks_wait(mask);
    } else {
        return reset_blocks(mask);
    }
}


static pexp_err_t set_gpio_function(uint32_t gpio, enum gpio_function new_func) {

    pexp_err_t pexp_err = PEXP_OK;
    uint32_t current_func;

    if (new_func != GPIO_FUNC_NULL) {
        // Assigning a new function: First check if the GPIO is free
        pexp_err = pexp_read32(_reg(iobank0_hw->io[(gpio)].ctrl), &current_func);

        if (pexp_err == PEXP_OK) {
            if (current_func != GPIO_FUNC_NULL) {
                pexp_err = PEXP_ERR_IN_USE;
            }
        }
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_write32(_reg(iobank0_hw->io[(gpio)].ctrl), new_func);
    }

    return pexp_err;
}


static pexp_err_t _clock_gpio_init_int_frac(uint32_t gpio, pexp_clkout_t clk_src, uint32_t div_int, uint8_t div_frac) {

    uint32_t gpclk;

    if      (gpio == 21) gpclk = clk_gpout0;
    else if (gpio == 23) gpclk = clk_gpout1;
    else if (gpio == 24) gpclk = clk_gpout2;
    else if (gpio == 25) gpclk = clk_gpout3;
    else {
        return PEXP_ERR_API_ARG;
    }

    // Set up the gpclk generator
    pexp_err_t pexp_err = pexp_write32(_reg(clocks_hw->clk[(gpclk)].ctrl),
        (clk_src << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB) | CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_write32(_reg(clocks_hw->clk[(gpclk)].div),
            (div_int << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) | div_frac);
    }

    // Set the gpio pin to gpclock function
    if (pexp_err == PEXP_OK) {
        pexp_err = set_gpio_function(gpio, GPIO_FUNC_GPCK);
    }

    return pexp_err;
}


static pexp_err_t set_gpio_hi_function(uint32_t higpio, enum gpio_function new_func) {

    if (higpio >= NUM_QSPI_GPIOS) {
        return PEXP_ERR_API_ARG;
    }

    pexp_err_t pexp_err = PEXP_OK;
    uint32_t current_func;

    if (new_func != GPIO_FUNC_NULL) {
        // Assigning a new function: First check if the GPIO is free
        pexp_err = pexp_read32(_reg(ioqspi_hw->io[(higpio)].ctrl), &current_func);

        if (pexp_err == PEXP_OK) {
            if (current_func != GPIO_FUNC_NULL) {
                pexp_err = PEXP_ERR_IN_USE;
            }
        }
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_write32(_reg(ioqspi_hw->io[(higpio)].ctrl), new_func);
    }

    return pexp_err;
}


static pexp_err_t adc_clock_init(bool enable) {

    pexp_err_t pexp_err = pexp_hw_clear_bits(_reg(clocks_hw->clk[clk_adc].ctrl), CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);

    if (enable) {
        if (pexp_err == PEXP_OK) {
            pexp_err = pexp_write32(_reg(clocks_hw->clk[clk_adc].ctrl), // select source
            (CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH << CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB));
        }
        if (pexp_err == PEXP_OK) {
            pexp_err = pexp_write32(_reg(clocks_hw->clk[clk_adc].div), 0x100);  // divide by 1.0
        }
        if (pexp_err == PEXP_OK) {
            pexp_err = pexp_hw_set_bits(_reg(clocks_hw->clk[clk_adc].ctrl), CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);
        }
    }

    return pexp_err;
}


static pexp_err_t timer_enable(void) {

    pexp_err_t pexp_err = peripheral_enable(true, RESETS_RESET_TIMER_BITS);

    if (pexp_err == PEXP_OK) {
        // The expander is accessed using DAP/SWD: The timers will freeze unless we clear these
        pexp_err = pexp_write32(_reg(timer_hw->dbgpause), 0);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_write32(_reg(watchdog_hw->tick),
            (TICK_GENERATOR_CYCLES | WATCHDOG_TICK_ENABLE_BITS));
    }

    return pexp_err;
}


static pexp_err_t get_timespec_and_tickcount(rosc_time_n_count_t *tm_tk) {

    tm_tk->time_us = port_get_time_us_64();

    return pexp_time_get_ticks64(&tm_tk->tick_count);
}


static pexp_err_t initial_clock_set(void) {

    // Set the ROSC range, afirm enable value (overrite reset default)
    pexp_err_t pexp_err = pexp_write_hw_mask(_reg(rosc_hw->ctrl),
                                            ((ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB) |
                                             (ROSC_CTRL_FREQ_RANGE_VALUE_LOW << ROSC_CTRL_FREQ_RANGE_LSB)),
                                             (ROSC_CTRL_ENABLE_BITS | ROSC_CTRL_FREQ_RANGE_BITS));

    if (pexp_err == PEXP_OK) {
        pexp_err = timer_enable();
    }

    if (pexp_err == PEXP_OK) {
        // start timing for ROSC frequency measurement
        pexp_err = get_timespec_and_tickcount(&snapshot_rosc_time_n_count);
    }

    // FIXME TODO HERE Work In Progress

    return pexp_err;
}


static pexp_err_t chip_init(void) {

    pexp_err_t pexp_err;

    pexp_err = dap_reset_chip_halt_cpus();

    if (pexp_err == PEXP_OK) {
        pexp_err = dap_initial_chip_config();
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = initial_clock_set();
    }

    return pexp_err;
}


static pexp_err_t set_gpio_input_enabled(uint32_t gpio, bool enable) {

    if (enable) {
        return pexp_hw_set_bits(_reg(padsbank0_hw->io[gpio]), PADS_BANK0_GPIO0_IE_BITS);
    } else {
        return pexp_hw_clear_bits(_reg(padsbank0_hw->io[gpio]), PADS_BANK0_GPIO0_IE_BITS);
    }
}

//----------------------------------------------------------------------------

// Public API

pexp_err_t pexp_init(void) {

    pexp_err_t pexp_err;

    pexp_err = dap_if_init();

    if (pexp_err == PEXP_OK) {
        pexp_err = chip_init();
    }

    return pexp_err;
}

//----------------------------------------------------------------------------

pexp_err_t pexp_gpio_block_enable(bool enable) {

    return peripheral_enable(enable, RESETS_RESET_PADS_BANK0_BITS | RESETS_RESET_IO_BANK0_BITS);
}


pexp_err_t pexp_gpio_init(uint32_t gpio) {

    uint32_t mask = 1ul << gpio;
    pexp_err_t pexp_err = pexp_hw_clear_bits(_reg(sio_hw->gpio_out), mask);

    // set as output
    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_gpio_set_dir_out_masked(mask);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = set_gpio_function(gpio, GPIO_FUNC_SIO);
    }

    return pexp_err;
}


pexp_err_t pexp_gpio_init_mask(uint32_t mask) {

    pexp_err_t pexp_err = PEXP_OK;

    for (uint32_t bit = 0; bit < NUM_BANK0_GPIOS && pexp_err == PEXP_OK; bit++) {

        if (mask & (1ul << bit)) {
            pexp_err = pexp_gpio_init(bit);
        }
    }

    return pexp_err;
}


pexp_err_t pexp_gpio_deinit(uint32_t gpio) {
    return set_gpio_function(gpio, GPIO_FUNC_NULL);
}


pexp_err_t pexp_gpio_deinit_mask(uint32_t mask) {

    pexp_err_t pexp_err = PEXP_OK;

    for (uint32_t bit = 0; bit < NUM_BANK0_GPIOS && pexp_err == PEXP_OK; bit++) {

        if (mask & (1ul << bit)) {
            pexp_err = pexp_gpio_deinit(bit);
        }
    }

    return pexp_err;
}


pexp_err_t pexp_gpio_set_dir(uint32_t gpio, gpio_dir_t gpio_dir) {

    uint32_t mask = 1ul << gpio;

    if (gpio_dir == GPIO_DIR_IN) {
        return pexp_gpio_set_dir_in_masked(mask);
    } else {
        return pexp_gpio_set_dir_out_masked(mask);
    }
}


pexp_err_t pexp_gpio_set_dir_out_masked(uint32_t mask) {
    return pexp_write32(_reg(sio_hw->gpio_oe_set), mask);
}


pexp_err_t pexp_gpio_set_dir_in_masked(uint32_t mask) {
    return pexp_write32(_reg(sio_hw->gpio_oe_clr), mask);
}


pexp_err_t pexp_gpio_set_mask(uint32_t mask) {
    return pexp_write32(_reg(sio_hw->gpio_set), mask);
}


pexp_err_t pexp_gpio_clr_mask(uint32_t mask) {
    return pexp_write32(_reg(sio_hw->gpio_clr), mask);
}


pexp_err_t pexp_gpio_toggle_mask(uint32_t mask) {
    return pexp_write32(_reg(sio_hw->gpio_togl), mask);
}


pexp_err_t pexp_gpio_set(uint32_t gpio) {
    return pexp_gpio_set_mask(1ul << gpio);
}


pexp_err_t pexp_gpio_clr(uint32_t gpio) {
    return pexp_gpio_clr_mask(1ul << gpio);
}


pexp_err_t pexp_gpio_toggle(uint32_t gpio) {
    return pexp_gpio_toggle_mask(1ul << gpio);
}


uint32_t pexp_gpio_get_all(void) {

    uint32_t read_value;

    if (PEXP_OK != pexp_read32(_reg(sio_hw->gpio_in), &read_value)) {
        return 0xFFFFFFFFul;
    }

    return read_value;
}


pexp_err_t pexp_gpio_set_all(uint32_t mask) {
    return pexp_write32(_reg(sio_hw->gpio_out), mask);
}


pexp_err_t pexp_clock_gpio_init(uint32_t gpio, pexp_clkout_t clk_src, float div)
{
    uint32_t div_int = (uint32_t)div;
    uint8_t frac = (uint8_t)((div - (float)div_int) * (1u << CLOCKS_CLK_GPOUT0_DIV_INT_LSB));
    return _clock_gpio_init_int_frac(gpio, clk_src, div_int, frac);
}


// Simple clock init code, with some parameters to set up the PLL (etc.)
pexp_err_t pexp_set_rosc(void) {
    return PEXP_ERR_UNSUPPORTED;
}

pexp_err_t pexp_set_xosc(void) {
    return PEXP_ERR_UNSUPPORTED;
}

//----------------------------------------------------------------------------

pexp_err_t pexp_read32(uint32_t address, uint32_t *pdata) {

    if (!pdata) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t dummy;
    pexp_err_t pexp_err = dap_set_target_rd_wr_address(address, NO_AUTO_INC);  // set address, inc mode

    if (pexp_err == PEXP_OK) {
        pexp_err = dap_read(AP_REG_IDR, &dummy);    // read dummy word
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = dap_read(DP_REG_RDBUF, pdata);     // read actual data
    }

    return pexp_err;
}


pexp_err_t pexp_write32(uint32_t address, uint32_t data) {

    uint32_t dummy;
    pexp_err_t pexp_err = dap_set_target_rd_wr_address(address, NO_AUTO_INC);  // set address, inc mode

    if (pexp_err == PEXP_OK) {
        pexp_err = dap_write(AP_REG_IDR, data);     // write desired data
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = dap_read(DP_REG_RDBUF, &dummy);    // dummy READ to flush the WRITE!!
    }

    return pexp_err;
}

//----------------------------------------------------------------------------

pexp_err_t pexp_block_read32(uint32_t address, uint32_t *pdata, uint32_t length) {

    if (!pdata) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t dummy;
    pexp_err_t pexp_err = PEXP_OK;

    while (pexp_err == PEXP_OK && length) {
        uint32_t xfer_length;

        if (length > DAP_BLOCK_XFER_MAX_WORD_LENGTH) {
             xfer_length = DAP_BLOCK_XFER_MAX_WORD_LENGTH;
        } else {
            xfer_length = length;
        }

        pexp_err = dap_set_target_rd_wr_address(address, USE_AUTO_INC);

        address += (xfer_length * 4);
        length -= xfer_length;

        if (pexp_err == PEXP_OK) {
            pexp_err = dap_read(AP_REG_IDR, &dummy);    // read dummy word
        }

        while (pexp_err == PEXP_OK && xfer_length--) {
            pexp_err = dap_read(AP_REG_DRW, pdata++);     // read actual data
        }
    }

    return pexp_err;
}


pexp_err_t pexp_block_write32(uint32_t address, const uint32_t *pdata, uint32_t length) {

    if (!pdata) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t dummy = 0;
    pexp_err_t pexp_err = PEXP_OK;

    while (pexp_err == PEXP_OK && length) {
        uint32_t xfer_length;

        if (length > DAP_BLOCK_XFER_MAX_WORD_LENGTH) {
             xfer_length = DAP_BLOCK_XFER_MAX_WORD_LENGTH;
        } else {
            xfer_length = length;
        }

        pexp_err = dap_set_target_rd_wr_address(address, USE_AUTO_INC);

        address += (xfer_length * 4);
        length -= xfer_length;

        while (pexp_err == PEXP_OK && xfer_length--) {
            pexp_err = dap_write(AP_REG_DRW, *pdata++);
        }

        if (pexp_err == PEXP_OK) {
            pexp_err = dap_read(DP_REG_RDBUF, &dummy);  // dummy READ to flush the WRITE!!
        }
    }

    return pexp_err;
}

//----------------------------------------------------------------------------

// Note:  To 'speed things up' we could use inlining, remove some checking etc.
// but it won't make much difference.  The time consuming part is doing the DAP
// transactions which the use of these helpers reduces to a minimum already.

pexp_err_t pexp_write_hw_mask(uint32_t addr, uint32_t values, uint32_t write_mask) {

    if (addr < SYSINFO_BASE) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t temp;
    pexp_err_t pexp_err = pexp_read32(addr, &temp);

    if (pexp_err == PEXP_OK) {
        temp &= ~write_mask;
        temp |= (write_mask & values);

        pexp_err = pexp_write32(addr, temp);
    }

    return pexp_err;
}


pexp_err_t pexp_hw_set_bits(uint32_t addr, uint32_t mask) {

    if (addr < SYSINFO_BASE) {
        return PEXP_ERR_API_ARG;
    }

    return pexp_write32(REG_ALIAS_SET_BITS | addr, mask);
}


pexp_err_t pexp_hw_clear_bits(uint32_t addr, uint32_t mask) {

    if (addr < SYSINFO_BASE) {
        return PEXP_ERR_API_ARG;
    }

    return pexp_write32(REG_ALIAS_CLR_BITS | addr, mask);
}


pexp_err_t pexp_hw_xor_bits(uint32_t addr, uint32_t mask) {

    if (addr < SYSINFO_BASE) {
        return PEXP_ERR_API_ARG;
    }

    return pexp_write32(REG_ALIAS_XOR_BITS | addr, mask);
}

//----------------------------------------------------------------------------

pexp_err_t pexp_gpio_hi_block_enable(bool enable) {

    return peripheral_enable(enable, RESETS_RESET_PADS_QSPI_BITS | RESETS_RESET_IO_QSPI_BITS);
}


pexp_err_t pexp_gpio_hi_init(uint32_t higpio) {

    if (higpio >= NUM_QSPI_GPIOS) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t himask = 1ul << higpio;
    pexp_err_t pexp_err = pexp_hw_clear_bits(_reg(sio_hw->gpio_hi_out), himask);

    // set as output
    if (pexp_err == PEXP_OK) {
        pexp_err =  pexp_gpio_hi_set_dir_out_masked(himask);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = set_gpio_hi_function(higpio, GPIO_FUNC_SIO);
    }

    return pexp_err;
}


pexp_err_t pexp_gpio_hi_init_mask(uint32_t himask) {

    pexp_err_t pexp_err = PEXP_OK;

    for (uint32_t bit = 0; bit < NUM_BANK0_GPIOS && pexp_err == PEXP_OK; bit++) {

        if (himask & (1ul << bit)) {
            pexp_err =  pexp_gpio_hi_init(bit);
        }
    }

    return pexp_err;
}


pexp_err_t pexp_gpio_hi_deinit(uint32_t higpio) {

    return set_gpio_hi_function(higpio, GPIO_FUNC_NULL);
}


pexp_err_t pexp_gpio_hi_deinit_mask(uint32_t higpio_mask) {

    pexp_err_t pexp_err = PEXP_OK;

    for (uint32_t bit = 0; bit < NUM_QSPI_GPIOS && pexp_err == PEXP_OK; bit++) {

        if (higpio_mask & (1ul << bit)) {
            pexp_err =  pexp_gpio_hi_deinit(bit);
        }
    }

    return pexp_err;
}


pexp_err_t pexp_gpio_hi_set_dir(uint32_t higpio, gpio_dir_t gpio_dir) {

    if (higpio >= NUM_QSPI_GPIOS) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t himask = 1ul << higpio;

    if (gpio_dir == GPIO_DIR_IN) {
        return  pexp_gpio_hi_set_dir_in_masked(himask);
    } else {
        return  pexp_gpio_hi_set_dir_out_masked(himask);
    }
}


pexp_err_t pexp_gpio_hi_set_dir_out_masked(uint32_t himask) {
    return pexp_write32(_reg(sio_hw->gpio_hi_oe_set), himask);
}


pexp_err_t pexp_gpio_hi_set_dir_in_masked(uint32_t himask) {
    return pexp_write32(_reg(sio_hw->gpio_hi_oe_clr), himask);
}


pexp_err_t pexp_gpio_hi_set_mask(uint32_t himask) {
    return pexp_write32(_reg(sio_hw->gpio_hi_set), himask);
}


pexp_err_t pexp_gpio_hi_clr_mask(uint32_t himask) {
    return pexp_write32(_reg(sio_hw->gpio_hi_clr), himask);
}


pexp_err_t pexp_gpio_hi_toggle_mask(uint32_t himask) {
    return pexp_write32(_reg(sio_hw->gpio_hi_togl), himask);
}


uint32_t pexp_gpio_hi_get_all(void) {

    uint32_t hiread_value;

    if (PEXP_OK != pexp_read32(_reg(sio_hw->gpio_hi_in), &hiread_value)) {
        return 0xFFFFFFFFul;
    }

    return hiread_value;
}


pexp_err_t pexp_gpio_hi_set_all(uint32_t himask) {
    return pexp_write32(_reg(sio_hw->gpio_hi_out), himask);
}


pexp_err_t pexp_gpio_set_pulls(uint32_t gpio, bool up, bool down) {

    return pexp_write_hw_mask(_reg(padsbank0_hw->io[gpio]),
        (bool_to_bit(up) << PADS_BANK0_GPIO0_PUE_LSB) | (bool_to_bit(down) << PADS_BANK0_GPIO0_PDE_LSB),
        PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

//----------------------------------------------------------------------------


pexp_err_t pexp_adc_block_enable(bool enable) {

    pexp_err_t pexp_err = adc_clock_init(enable);

    if (pexp_err == PEXP_OK) {
        pexp_err = peripheral_enable(enable, RESETS_RESET_ADC_BITS);
    }

    return pexp_err;
}


pexp_err_t pexp_adc_init(void) {

    pexp_err_t pexp_err = pexp_adc_block_enable(false);

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_adc_block_enable(true);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_write32(_reg(adc_hw->cs),  ADC_CS_EN_BITS);
    }

    for (uint32_t escape = 0; pexp_err == PEXP_OK; escape++) {

        uint32_t adc_hw_cs;

        pexp_err = pexp_read32(_reg(adc_hw->cs), &adc_hw_cs);

        if (pexp_err == PEXP_OK && (adc_hw_cs & ADC_CS_READY_BITS)) {
            break;
        }

        if (escape > ADC_ESCAPE_COUNT) {
           pexp_err = PEXP_ERR_ADC_TIMEOUT;
        }
    }

    return pexp_err;
}


pexp_err_t pexp_adc_gpio_init(uint32_t gpio) {

    if ((gpio >= NUM_BANK0_GPIOS) ||
        (gpio <= (NUM_BANK0_GPIOS - NUM_ADC_CHANNELS))) {
        return PEXP_ERR_API_ARG;
    }

    // Select NULL function to make output driver hi-Z
    pexp_err_t pexp_err = set_gpio_function(gpio, GPIO_FUNC_NULL);

    // Disable digital pulls and digital receiver
    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_disable_gpio_pulls(gpio);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = set_gpio_input_enabled(gpio, 0);
    }

    return pexp_err;
}


pexp_err_t pexp_adc_select_input(uint32_t input) {

    if (input >= NUM_ADC_CHANNELS) {
        return PEXP_ERR_API_ARG;
    }

    return pexp_write_hw_mask(_reg(adc_hw->cs), input << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
}


pexp_err_t pexp_adc_get_selected_input(uint32_t *pinput) {

    if (!pinput) {
        return PEXP_ERR_API_ARG;
    }

    pexp_err_t pexp_err = pexp_read32(_reg(adc_hw->cs), pinput);

    *pinput  &= ADC_CS_AINSEL_BITS;
    *pinput >>= ADC_CS_AINSEL_LSB;

    if (pexp_err != PEXP_OK) {
        *pinput = 0;
    }

    return pexp_err;
}


pexp_err_t pexp_adc_set_temp_sensor_enabled(bool enable) {

    if (enable) {
        return pexp_hw_set_bits(_reg(adc_hw->cs), ADC_CS_TS_EN_BITS);
    } else {
        return pexp_hw_clear_bits(_reg(adc_hw->cs), ADC_CS_TS_EN_BITS);
    }
}


pexp_err_t pexp_adc_read(uint16_t *padc) {

    uint32_t escape;
    uint32_t adc_rd_regs;
    pexp_err_t pexp_err = pexp_hw_set_bits(_reg(adc_hw->cs), ADC_CS_START_ONCE_BITS);

    for (escape = 0; escape <= ADC_ESCAPE_COUNT && pexp_err == PEXP_OK; escape++) {

        pexp_err = pexp_read32(_reg(adc_hw->cs), &adc_rd_regs);

        if (pexp_err == PEXP_OK && (adc_rd_regs & ADC_CS_READY_BITS)) {

            pexp_err = pexp_read32(_reg(adc_hw->result), &adc_rd_regs);

            if (pexp_err == PEXP_OK) {
                *padc = (uint16_t) (adc_rd_regs  & 0x0000FFFFul);
            }
            break;
        }
    }

    if (escape > ADC_ESCAPE_COUNT) {
        pexp_err = PEXP_ERR_ADC_TIMEOUT;
    }

    if (pexp_err != PEXP_OK) {
        *padc = 0;
    }

    return pexp_err;
}


pexp_err_t pexp_time_get_ticks32(uint32_t *pticks) {

    if (!pticks) {
        return PEXP_ERR_API_ARG;
    }

    return pexp_read32(_reg(timer_hw->timerawl), pticks);
}


pexp_err_t pexp_time_get_ticks64(uint64_t *pticks) {

    if (!pticks) {
        return PEXP_ERR_API_ARG;
    }

    uint32_t lo, hi;

    // Reading low latches the high value
    pexp_err_t pexp_err = pexp_read32(_reg(timer_hw->timelr), &lo);

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_read32(_reg(timer_hw->timehr), &hi);
    }

    if (pexp_err == PEXP_OK) {
        *pticks = ((uint64_t) hi << 32u) | lo;
    }

    return pexp_err;
}


pexp_err_t pexp_rosc_set_div(uint32_t rosc_div) {

    if (rosc_div == 0 || rosc_div > 32) {
        return PEXP_ERR_API_ARG;
    }

    if (rosc_div == 32) {
        rosc_div = 0;
    }

    return pexp_write32(_reg(rosc_hw->div), (rosc_div + ROSC_DIV_VALUE_PASS));
}


pexp_err_t pexp_rosc_get_div(uint32_t *pdiv) {

    pexp_err_t pexp_err = pexp_read32(_reg(rosc_hw->div), pdiv);

    if (pexp_err == PEXP_OK) {

        *pdiv -= ROSC_DIV_VALUE_PASS;
        if (*pdiv == 0) {
            *pdiv = 32;
        }
    }
    else {
        *pdiv = 0;
    }

    return pexp_err;
}


pexp_err_t pexp_rosc_get_freq_ab_bits(uint32_t *pfreq) {

    uint32_t freqa, freqb;

    pexp_err_t pexp_err = pexp_read32(_reg(rosc_hw->freqa), &freqa);

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_read32(_reg(rosc_hw->freqb), &freqb);
    }

    if (pexp_err == PEXP_OK) {
        *pfreq = ((freqb << 16)  | (freqa & 0xfffful));
    }

    return pexp_err;
}


pexp_err_t pexp_rosc_set_freq_ab_bits(uint32_t freq32) {

    pexp_err_t pexp_err = pexp_write32(_reg(rosc_hw->freqa), (ROSC_FREQA_PASSWD_VALUE_PASS << ROSC_FREQA_PASSWD_LSB) | (freq32 & 0xfffful));

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_write32(_reg(rosc_hw->freqb), (ROSC_FREQB_PASSWD_VALUE_PASS << ROSC_FREQB_PASSWD_LSB) | (freq32 >> 16ul));
    }

    return PEXP_OK;
}


uint32_t pexp_rosc_inc_freq_ab_bits(uint32_t freq32) {

    // Note:  We set bits working *down* from the MS side since, with shorter
    // ROSC chain lengths (MEDIUM, HIGH) the LS sections are the ones removed.
    for (unsigned int bit = 30; bit >= 0; bit--) {

        if ((bit % 4) == 3) {
            continue;  // there are no active bits at positions 3, 7, 11 ... 31
        }

        uint32_t bit_mask = 1ul << bit;

        if (freq32 & bit_mask) {
            continue; // already set
        }

        freq32 |= bit_mask;  // set it!!
        break;
    }

    return freq32;
}


uint32_t pexp_rosc_dec_freq_ab_bits(uint32_t freq32) {

    // Work up, LS->MS, see comment in: pexp_rosc_inc_freq_ab_bits()
    for (unsigned int bit = 0; bit < 31; bit++) {

        if ((bit % 4) == 3) {
            continue;  // there are no active bits at positions 3, 7, 11 ... 31
        }

        uint32_t bit_mask = 1ul << bit;

        if (0 == (freq32 & bit_mask)) {
            continue; // already clear
        }

        freq32 &= ~bit_mask;  // clear it!!
        break;
    }

    return freq32;
}


pexp_err_t pexp_rosc_measure_postdiv_clock_freq(uint32_t *rosc_freq_hz, uint32_t min_sample_us) {

    pexp_err_t pexp_err;
    uint64_t delta_time_us;
    rosc_time_n_count_t current_rosc_time_n_count;

    *rosc_freq_hz = 0;

    do {
        pexp_err = get_timespec_and_tickcount(&current_rosc_time_n_count);

        if (pexp_err != PEXP_OK) {
            break;
        }

        delta_time_us = current_rosc_time_n_count.time_us - snapshot_rosc_time_n_count.time_us;

        if (delta_time_us > (uint64_t) min_sample_us) {
            break;
        }

    } while (min_sample_us);

    if (pexp_err == PEXP_OK) {

        uint64_t delta_tick_count = current_rosc_time_n_count.tick_count - snapshot_rosc_time_n_count.tick_count;
        *rosc_freq_hz = (uint32_t) ((delta_tick_count * (uint64_t) 1000000ull * (uint64_t) TICK_GENERATOR_CYCLES) / delta_time_us);

        // update the static datapoint for any subsequent calculation(s)
        snapshot_rosc_time_n_count = current_rosc_time_n_count;
    }

    return pexp_err;
}


static pexp_err_t rosc_trim(uint32_t target_rosc_postdiv_clock_hz,  uint32_t *measured_rosc_postdiv_clock_hz) {

    pexp_err_t pexp_err = PEXP_OK;
    uint32_t freq_bits, last_bits_setting;
    uint32_t freq_delta;
    uint32_t last_delta = (uint32_t) -1; // set improbably bad "last" delta

    freq_bits = 0;

    do {
        pexp_err = pexp_rosc_set_freq_ab_bits(freq_bits);

        if (pexp_err == PEXP_OK) {
            last_bits_setting = freq_bits;
            pexp_err = pexp_rosc_measure_postdiv_clock_freq(measured_rosc_postdiv_clock_hz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
        }

        if (pexp_err == PEXP_OK) {
            if (*measured_rosc_postdiv_clock_hz > target_rosc_postdiv_clock_hz) {
                freq_delta = *measured_rosc_postdiv_clock_hz - target_rosc_postdiv_clock_hz;
            } else {
                freq_delta = target_rosc_postdiv_clock_hz - *measured_rosc_postdiv_clock_hz;
            }

            if (freq_delta >= last_delta) {
                break;
            }

            last_delta = freq_delta;

            freq_bits = pexp_rosc_inc_freq_ab_bits(freq_bits);
        }

    } while(1);

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_rosc_set_freq_ab_bits(last_bits_setting);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_rosc_measure_postdiv_clock_freq(measured_rosc_postdiv_clock_hz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
    }

    return pexp_err;
}


pexp_err_t pexp_rosc_set_faster_postdiv_clock_freq(uint32_t target_rosc_postdiv_clock_hz, uint32_t *measured_rosc_postdiv_clock_hz) {

    uint32_t freq_bits;
    uint32_t rosc_div;

    uint32_t rosc_freq_hz;
    uint32_t new_rosc_freq_hz;

    pexp_err_t pexp_err = pexp_rosc_get_freq_ab_bits(&freq_bits);

    if (pexp_err == PEXP_OK && freq_bits) {
        // Firstly, if ROSC has been trimmed _up_ at all - trim it fully down
        pexp_err = pexp_rosc_set_freq_ab_bits(0);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_rosc_measure_postdiv_clock_freq(measured_rosc_postdiv_clock_hz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
    }

    if (pexp_err == PEXP_OK && (target_rosc_postdiv_clock_hz < *measured_rosc_postdiv_clock_hz)) {
        // it is not possible to slow things down further because the diver setting
        // can not be increased without [possibly] glitching the ROSC clock.
        // Return "OK" and leave the *caller* to decide if this is slow enough ...
        return PEXP_OK;
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_rosc_get_div(&rosc_div);
    }

    if (pexp_err == PEXP_OK) {

        uint32_t new_div;

        for (new_div = rosc_div; new_div > 1; new_div--) {

            new_rosc_freq_hz = target_rosc_postdiv_clock_hz * new_div;

            // this could be smarter for low target freqs (~10 MHz) but will [still]
            // work optimally for higher target frquencies (such as 48 MHz).
            if (new_rosc_freq_hz <= ROSC_MAX_FREQ) {
                break;
            }
        }

        pexp_err = pexp_rosc_set_div(new_div);
    }

    if (pexp_err == PEXP_OK) {
        pexp_err = rosc_trim(target_rosc_postdiv_clock_hz, measured_rosc_postdiv_clock_hz);
    }

    return pexp_err;
}
