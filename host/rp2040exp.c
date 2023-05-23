/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <rp2040exp_dap.h>
#include <rp2040exp.h>
#include <rp2040_includes.h>
#include <rp2040exp_port_api.h>

#define DAP_BLOCK_XFER_MAX_WORD_LENGTH  256

#define RESET_ESCAPE_COUNT              10
#define ADC_ESCAPE_COUNT                10

#define ROSC_MAX_FREQ_KHZ               125000ul  // could probably be more


// struct for ROSC frequency measurement
typedef struct {
    uint64_t time_us;
    uint64_t tick_count;
    uint32_t system_clock_frequency_khz;
} rosc_time_n_count_t;

static rosc_time_n_count_t rosc_time_count_freq = { 0 }; // vital


static rpexp_err_t reset_blocks(uint32_t mask) {

    return rpexp_hw_set_bits(_reg(resets_hw->reset), mask);
}


static rpexp_err_t unreset_blocks(uint32_t mask) {

    return rpexp_hw_clear_bits(_reg(resets_hw->reset), mask);
}


static rpexp_err_t unreset_blocks_wait(uint32_t mask) {

    rpexp_err_t rpexp_err = unreset_blocks(mask);

    for (uint32_t escape = 0; rpexp_err == RPEXP_OK; escape++) {

        uint32_t reset_done;

        rpexp_err = rpexp_read32(_reg(resets_hw->reset_done), &reset_done);

        if (rpexp_err == RPEXP_OK && (reset_done & mask)) {
            break;
        }
        if (escape > RESET_ESCAPE_COUNT) {
            rpexp_err = RPEXP_ERR_RESET_TIMEOUT;
        }
    }

    return rpexp_err;
}


static rpexp_err_t peripheral_enable(bool enable, uint32_t mask) {

    if (enable) {
        return unreset_blocks_wait(mask);
    } else {
        return reset_blocks(mask);
    }
}


static rpexp_err_t set_gpio_function(uint32_t gpio, enum gpio_function new_func) {

    rpexp_err_t rpexp_err = RPEXP_OK;
    uint32_t current_func;

    if (new_func != GPIO_FUNC_NULL) {
        // Assigning a new function: First check if the GPIO is free
        rpexp_err = rpexp_read32(_reg(iobank0_hw->io[(gpio)].ctrl), &current_func);

        if (rpexp_err == RPEXP_OK) {
            if (current_func != GPIO_FUNC_NULL) {
                rpexp_err = RPEXP_ERR_IN_USE;
            }
        }
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(iobank0_hw->io[(gpio)].ctrl), new_func);
    }

    return rpexp_err;
}


static rpexp_err_t _clock_gpio_init_int_frac(uint32_t gpio, rpexp_clkout_t clk_src, uint32_t div_int, uint8_t div_frac) {

    uint32_t gpclk;

    if      (gpio == 21) gpclk = clk_gpout0;
    else if (gpio == 23) gpclk = clk_gpout1;
    else if (gpio == 24) gpclk = clk_gpout2;
    else if (gpio == 25) gpclk = clk_gpout3;
    else {
        return RPEXP_ERR_API_ARG;
    }

    // Set up the gpclk generator
    rpexp_err_t rpexp_err = rpexp_write32(_reg(clocks_hw->clk[(gpclk)].ctrl),
        (clk_src << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB) | CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(clocks_hw->clk[(gpclk)].div),
            (div_int << CLOCKS_CLK_GPOUT0_DIV_INT_LSB) | div_frac);
    }

    // Set the gpio pin to gpclock function
    if (rpexp_err == RPEXP_OK) {
        rpexp_err = set_gpio_function(gpio, GPIO_FUNC_GPCK);
    }

    return rpexp_err;
}


static rpexp_err_t set_gpio_hi_function(uint32_t higpio, enum gpio_function new_func) {

    if (higpio >= NUM_QSPI_GPIOS) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = RPEXP_OK;
    uint32_t current_func;

    if (new_func != GPIO_FUNC_NULL) {
        // Assigning a new function: First check if the GPIO is free
        rpexp_err = rpexp_read32(_reg(ioqspi_hw->io[(higpio)].ctrl), &current_func);

        if (rpexp_err == RPEXP_OK) {
            if (current_func != GPIO_FUNC_NULL) {
                rpexp_err = RPEXP_ERR_IN_USE;
            }
        }
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(ioqspi_hw->io[(higpio)].ctrl), new_func);
    }

    return rpexp_err;
}


static rpexp_err_t peripheral_clock_init(bool enable) {

    rpexp_err_t rpexp_err = rpexp_hw_clear_bits(_reg(clocks_hw->clk[clk_peri].ctrl), CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);

    if (enable) {
        if (rpexp_err == RPEXP_OK) {
            rpexp_err = rpexp_write32(_reg(clocks_hw->clk[clk_peri].ctrl), // select source
            (CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB));
        }
        if (rpexp_err == RPEXP_OK) {
            rpexp_err = rpexp_write32(_reg(clocks_hw->clk[clk_peri].div), 0x100);  // divide by 1.0
        }
        if (rpexp_err == RPEXP_OK) {
            rpexp_err = rpexp_hw_set_bits(_reg(clocks_hw->clk[clk_peri].ctrl), CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
        }
    }

    return rpexp_err;
}


static rpexp_err_t adc_clock_init(bool enable) {

    rpexp_err_t rpexp_err = rpexp_hw_clear_bits(_reg(clocks_hw->clk[clk_adc].ctrl), CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);

    if (enable) {
        if (rpexp_err == RPEXP_OK) {
            rpexp_err = rpexp_write32(_reg(clocks_hw->clk[clk_adc].ctrl), // select source
            (CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH << CLOCKS_CLK_ADC_CTRL_AUXSRC_LSB));
        }
        if (rpexp_err == RPEXP_OK) {
            rpexp_err = rpexp_write32(_reg(clocks_hw->clk[clk_adc].div), 0x100);  // divide by 1.0
        }
        if (rpexp_err == RPEXP_OK) {
            rpexp_err = rpexp_hw_set_bits(_reg(clocks_hw->clk[clk_adc].ctrl), CLOCKS_CLK_ADC_CTRL_ENABLE_BITS);
        }
    }

    return rpexp_err;
}


static rpexp_err_t timer_enable(void) {

    rpexp_err_t rpexp_err = peripheral_enable(true, RESETS_RESET_TIMER_BITS);

    if (rpexp_err == RPEXP_OK) {
        // The expander is accessed using DAP/SWD: The timers will freeze unless we clear these
        rpexp_err = rpexp_write32(_reg(timer_hw->dbgpause), 0);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(watchdog_hw->tick),
            (TICK_GENERATOR_CYCLES | WATCHDOG_TICK_ENABLE_BITS));
    }

    return rpexp_err;
}


static rpexp_err_t get_timespec_and_tickcount(rosc_time_n_count_t *tm_tk) {

    tm_tk->time_us = port_get_time_us_64();
    return rpexp_time_get_ticks64(&tm_tk->tick_count);
}


static rpexp_err_t initial_clock_set(void) {

    // Set the ROSC range, afirm enable value (overrite reset default)
    rpexp_err_t rpexp_err = rpexp_write_hw_mask(_reg(rosc_hw->ctrl),
                                            ((ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB) |
                                             (ROSC_CTRL_FREQ_RANGE_VALUE_LOW << ROSC_CTRL_FREQ_RANGE_LSB)),
                                             (ROSC_CTRL_ENABLE_BITS | ROSC_CTRL_FREQ_RANGE_BITS));

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = timer_enable();
    }

    if (rpexp_err == RPEXP_OK) {
        // start timing for ROSC frequency measurement
        rpexp_err = get_timespec_and_tickcount(&rosc_time_count_freq);
    }

    // FIXME TODO HERE Work In Progress

    return rpexp_err;
}


static rpexp_err_t chip_init(void) {

    rpexp_err_t rpexp_err;

    rpexp_err = dap_reset_chip_halt_cpus();

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = dap_initial_chip_config();
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = initial_clock_set();
    }

    return rpexp_err;
}


static rpexp_err_t set_gpio_input_enabled(uint32_t gpio, bool enable) {

    if (enable) {
        return rpexp_hw_set_bits(_reg(padsbank0_hw->io[gpio]), PADS_BANK0_GPIO0_IE_BITS);
    } else {
        return rpexp_hw_clear_bits(_reg(padsbank0_hw->io[gpio]), PADS_BANK0_GPIO0_IE_BITS);
    }
}


static rpexp_err_t rosc_frequency_trim_up(uint32_t target_rosc_clock_khz,  uint32_t *measured_rosc_clock_khz) {

    uint32_t freq_bits, last_bits_setting;
    uint32_t freq_delta, last_delta;

    // Firstly, trim ROSC fully _down_ in frequency
    rpexp_err_t rpexp_err = rpexp_rosc_zero_all_freq_ab_bits();

    freq_bits = last_bits_setting = 0;

    while (rpexp_err == RPEXP_OK) {

        rpexp_err = rpexp_rosc_measure_clock_freq_khz(measured_rosc_clock_khz, MIN_ROSC_FREQ_SAMPLE_TIME_US);

        if (rpexp_err == RPEXP_OK) {
            if (*measured_rosc_clock_khz > target_rosc_clock_khz) {
                freq_delta = *measured_rosc_clock_khz - target_rosc_clock_khz;
            } else {
                freq_delta = target_rosc_clock_khz - *measured_rosc_clock_khz;
            }

            if (freq_bits && freq_delta > last_delta) {
                // relies on monotonicity of settings
                break;
            }

            last_delta = freq_delta;
            last_bits_setting = freq_bits;

            freq_bits = rpexp_rosc_inc_freq_ab_bits(freq_bits);
            rpexp_err = rpexp_rosc_set_freq_ab_bits(freq_bits);
        }
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_rosc_set_freq_ab_bits(last_bits_setting);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_rosc_measure_clock_freq_khz(measured_rosc_clock_khz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
    }

    return rpexp_err;
}


static rpexp_err_t common_rosc_set_freq_ab_bits(uint32_t freq_bits32) {

    rpexp_err_t rpexp_err = rpexp_write32(_reg(rosc_hw->freqa), (ROSC_FREQA_PASSWD_VALUE_PASS << ROSC_FREQA_PASSWD_LSB) | (freq_bits32 & 0xfffful));

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(rosc_hw->freqb), (ROSC_FREQB_PASSWD_VALUE_PASS << ROSC_FREQB_PASSWD_LSB) | (freq_bits32 >> 16ul));
    }

    return RPEXP_OK;
}

//----------------------------------------------------------------------------

// Public API

rpexp_err_t rpexp_init(void) {

    rpexp_err_t rpexp_err;

    rpexp_err = dap_if_init();

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = chip_init();
    }

    return rpexp_err;
}

//----------------------------------------------------------------------------

rpexp_err_t rpexp_gpio_block_enable(bool enable) {

    return peripheral_enable(enable, RESETS_RESET_PADS_BANK0_BITS | RESETS_RESET_IO_BANK0_BITS);
}


rpexp_err_t rpexp_gpio_init(uint32_t gpio) {

    uint32_t mask = 1ul << gpio;
    rpexp_err_t rpexp_err = rpexp_hw_clear_bits(_reg(sio_hw->gpio_out), mask);

    // set as output
    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_gpio_set_dir_out_masked(mask);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = set_gpio_function(gpio, GPIO_FUNC_SIO);
    }

    return rpexp_err;
}


rpexp_err_t rpexp_gpio_init_mask(uint32_t mask) {

    rpexp_err_t rpexp_err = RPEXP_OK;

    for (uint32_t bit = 0; bit < NUM_BANK0_GPIOS && rpexp_err == RPEXP_OK; bit++) {

        if (mask & (1ul << bit)) {
            rpexp_err = rpexp_gpio_init(bit);
        }
    }

    return rpexp_err;
}


rpexp_err_t rpexp_gpio_deinit(uint32_t gpio) {
    return set_gpio_function(gpio, GPIO_FUNC_NULL);
}


rpexp_err_t rpexp_gpio_deinit_mask(uint32_t mask) {

    rpexp_err_t rpexp_err = RPEXP_OK;

    for (uint32_t bit = 0; bit < NUM_BANK0_GPIOS && rpexp_err == RPEXP_OK; bit++) {

        if (mask & (1ul << bit)) {
            rpexp_err = rpexp_gpio_deinit(bit);
        }
    }

    return rpexp_err;
}


rpexp_err_t rpexp_gpio_set_dir(uint32_t gpio, gpio_dir_t gpio_dir) {

    uint32_t mask = 1ul << gpio;

    if (gpio_dir == GPIO_DIR_IN) {
        return rpexp_gpio_set_dir_in_masked(mask);
    } else {
        return rpexp_gpio_set_dir_out_masked(mask);
    }
}


rpexp_err_t rpexp_gpio_set_dir_out_masked(uint32_t mask) {
    return rpexp_write32(_reg(sio_hw->gpio_oe_set), mask);
}


rpexp_err_t rpexp_gpio_set_dir_in_masked(uint32_t mask) {
    return rpexp_write32(_reg(sio_hw->gpio_oe_clr), mask);
}


rpexp_err_t rpexp_gpio_set_mask(uint32_t mask) {
    return rpexp_write32(_reg(sio_hw->gpio_set), mask);
}


rpexp_err_t rpexp_gpio_clr_mask(uint32_t mask) {
    return rpexp_write32(_reg(sio_hw->gpio_clr), mask);
}


rpexp_err_t rpexp_gpio_toggle_mask(uint32_t mask) {
    return rpexp_write32(_reg(sio_hw->gpio_togl), mask);
}


rpexp_err_t rpexp_gpio_set(uint32_t gpio) {
    return rpexp_gpio_set_mask(1ul << gpio);
}


rpexp_err_t rpexp_gpio_clr(uint32_t gpio) {
    return rpexp_gpio_clr_mask(1ul << gpio);
}


rpexp_err_t rpexp_gpio_toggle(uint32_t gpio) {
    return rpexp_gpio_toggle_mask(1ul << gpio);
}


uint32_t rpexp_gpio_get_all(void) {

    uint32_t read_value;

    if (RPEXP_OK != rpexp_read32(_reg(sio_hw->gpio_in), &read_value)) {
        return 0xFFFFFFFFul;
    }

    return read_value;
}


rpexp_err_t rpexp_gpio_set_all(uint32_t mask) {
    return rpexp_write32(_reg(sio_hw->gpio_out), mask);
}


rpexp_err_t rpexp_clock_gpio_init(uint32_t gpio, rpexp_clkout_t clk_src, float div)
{
    uint32_t div_int = (uint32_t)div;
    uint8_t frac = (uint8_t)((div - (float)div_int) * (1u << CLOCKS_CLK_GPOUT0_DIV_INT_LSB));
    return _clock_gpio_init_int_frac(gpio, clk_src, div_int, frac);
}


// Simple clock init code, with some parameters to set up the PLL (etc.)
rpexp_err_t rpexp_set_rosc(void) {
    return RPEXP_ERR_UNSUPPORTED;
}

rpexp_err_t rpexp_set_xosc(void) {
    return RPEXP_ERR_UNSUPPORTED;
}

//----------------------------------------------------------------------------

rpexp_err_t rpexp_read32(uint32_t address, uint32_t *pdata) {

    if (!pdata) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t dummy;
    rpexp_err_t rpexp_err = dap_set_target_rd_wr_address(address, NO_AUTO_INC);  // set address, inc mode

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = dap_read(AP_REG_IDR, &dummy);    // read dummy word
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = dap_read(DP_REG_RDBUF, pdata);     // read actual data
    }

    return rpexp_err;
}


rpexp_err_t rpexp_write32(uint32_t address, uint32_t data) {

    uint32_t dummy;
    rpexp_err_t rpexp_err = dap_set_target_rd_wr_address(address, NO_AUTO_INC);  // set address, inc mode

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = dap_write(AP_REG_IDR, data);     // write desired data
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = dap_read(DP_REG_RDBUF, &dummy);    // dummy READ to flush the WRITE!!
    }

    return rpexp_err;
}

//----------------------------------------------------------------------------

rpexp_err_t rpexp_block_read32(uint32_t address, uint32_t *pdata, uint32_t length) {

    if (!pdata) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t dummy;
    rpexp_err_t rpexp_err = RPEXP_OK;

    while (rpexp_err == RPEXP_OK && length) {
        uint32_t xfer_length;

        if (length > DAP_BLOCK_XFER_MAX_WORD_LENGTH) {
             xfer_length = DAP_BLOCK_XFER_MAX_WORD_LENGTH;
        } else {
            xfer_length = length;
        }

        rpexp_err = dap_set_target_rd_wr_address(address, USE_AUTO_INC);

        address += (xfer_length * 4);
        length -= xfer_length;

        if (rpexp_err == RPEXP_OK) {
            rpexp_err = dap_read(AP_REG_IDR, &dummy);    // read dummy word
        }

        while (rpexp_err == RPEXP_OK && xfer_length--) {
            rpexp_err = dap_read(AP_REG_DRW, pdata++);     // read actual data
        }
    }

    return rpexp_err;
}


rpexp_err_t rpexp_block_write32(uint32_t address, const uint32_t *pdata, uint32_t length) {

    if (!pdata) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t dummy = 0;
    rpexp_err_t rpexp_err = RPEXP_OK;

    while (rpexp_err == RPEXP_OK && length) {
        uint32_t xfer_length;

        if (length > DAP_BLOCK_XFER_MAX_WORD_LENGTH) {
             xfer_length = DAP_BLOCK_XFER_MAX_WORD_LENGTH;
        } else {
            xfer_length = length;
        }

        rpexp_err = dap_set_target_rd_wr_address(address, USE_AUTO_INC);

        address += (xfer_length * 4);
        length -= xfer_length;

        while (rpexp_err == RPEXP_OK && xfer_length--) {
            rpexp_err = dap_write(AP_REG_DRW, *pdata++);
        }

        if (rpexp_err == RPEXP_OK) {
            rpexp_err = dap_read(DP_REG_RDBUF, &dummy);  // dummy READ to flush the WRITE!!
        }
    }

    return rpexp_err;
}

//----------------------------------------------------------------------------

// Note:  To 'speed things up' we could use inlining, remove some checking etc.
// but it won't make much difference.  The time consuming part is doing the DAP
// transactions which the use of these helpers reduces to a minimum already.

rpexp_err_t rpexp_write_hw_mask(uint32_t addr, uint32_t values, uint32_t write_mask) {

    if (addr < SYSINFO_BASE) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t temp;
    rpexp_err_t rpexp_err = rpexp_read32(addr, &temp);

    if (rpexp_err == RPEXP_OK) {
        temp &= ~write_mask;
        temp |= (write_mask & values);

        rpexp_err = rpexp_write32(addr, temp);
    }

    return rpexp_err;
}


rpexp_err_t rpexp_hw_set_bits(uint32_t addr, uint32_t mask) {

    if (addr < SYSINFO_BASE) {
        return RPEXP_ERR_API_ARG;
    }

    return rpexp_write32(REG_ALIAS_SET_BITS | addr, mask);
}


rpexp_err_t rpexp_hw_clear_bits(uint32_t addr, uint32_t mask) {

    if (addr < SYSINFO_BASE) {
        return RPEXP_ERR_API_ARG;
    }

    return rpexp_write32(REG_ALIAS_CLR_BITS | addr, mask);
}


rpexp_err_t rpexp_hw_xor_bits(uint32_t addr, uint32_t mask) {

    if (addr < SYSINFO_BASE) {
        return RPEXP_ERR_API_ARG;
    }

    return rpexp_write32(REG_ALIAS_XOR_BITS | addr, mask);
}

//----------------------------------------------------------------------------

rpexp_err_t rpexp_gpio_hi_block_enable(bool enable) {

    return peripheral_enable(enable, RESETS_RESET_PADS_QSPI_BITS | RESETS_RESET_IO_QSPI_BITS);
}


rpexp_err_t rpexp_gpio_hi_init(uint32_t higpio) {

    if (higpio >= NUM_QSPI_GPIOS) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t himask = 1ul << higpio;
    rpexp_err_t rpexp_err = rpexp_hw_clear_bits(_reg(sio_hw->gpio_hi_out), himask);

    // set as output
    if (rpexp_err == RPEXP_OK) {
        rpexp_err =  rpexp_gpio_hi_set_dir_out_masked(himask);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = set_gpio_hi_function(higpio, GPIO_FUNC_SIO);
    }

    return rpexp_err;
}


rpexp_err_t rpexp_gpio_hi_init_mask(uint32_t himask) {

    rpexp_err_t rpexp_err = RPEXP_OK;

    for (uint32_t bit = 0; bit < NUM_BANK0_GPIOS && rpexp_err == RPEXP_OK; bit++) {

        if (himask & (1ul << bit)) {
            rpexp_err =  rpexp_gpio_hi_init(bit);
        }
    }

    return rpexp_err;
}


rpexp_err_t rpexp_gpio_hi_deinit(uint32_t higpio) {

    return set_gpio_hi_function(higpio, GPIO_FUNC_NULL);
}


rpexp_err_t rpexp_gpio_hi_deinit_mask(uint32_t higpio_mask) {

    rpexp_err_t rpexp_err = RPEXP_OK;

    for (uint32_t bit = 0; bit < NUM_QSPI_GPIOS && rpexp_err == RPEXP_OK; bit++) {

        if (higpio_mask & (1ul << bit)) {
            rpexp_err =  rpexp_gpio_hi_deinit(bit);
        }
    }

    return rpexp_err;
}


rpexp_err_t rpexp_gpio_hi_set_dir(uint32_t higpio, gpio_dir_t gpio_dir) {

    if (higpio >= NUM_QSPI_GPIOS) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t himask = 1ul << higpio;

    if (gpio_dir == GPIO_DIR_IN) {
        return  rpexp_gpio_hi_set_dir_in_masked(himask);
    } else {
        return  rpexp_gpio_hi_set_dir_out_masked(himask);
    }
}


rpexp_err_t rpexp_gpio_hi_set_dir_out_masked(uint32_t himask) {
    return rpexp_write32(_reg(sio_hw->gpio_hi_oe_set), himask);
}


rpexp_err_t rpexp_gpio_hi_set_dir_in_masked(uint32_t himask) {
    return rpexp_write32(_reg(sio_hw->gpio_hi_oe_clr), himask);
}


rpexp_err_t rpexp_gpio_hi_set_mask(uint32_t himask) {
    return rpexp_write32(_reg(sio_hw->gpio_hi_set), himask);
}


rpexp_err_t rpexp_gpio_hi_clr_mask(uint32_t himask) {
    return rpexp_write32(_reg(sio_hw->gpio_hi_clr), himask);
}


rpexp_err_t rpexp_gpio_hi_toggle_mask(uint32_t himask) {
    return rpexp_write32(_reg(sio_hw->gpio_hi_togl), himask);
}


uint32_t rpexp_gpio_hi_get_all(void) {

    uint32_t hiread_value;

    if (RPEXP_OK != rpexp_read32(_reg(sio_hw->gpio_hi_in), &hiread_value)) {
        return 0xFFFFFFFFul;
    }

    return hiread_value;
}


rpexp_err_t rpexp_gpio_hi_set_all(uint32_t himask) {
    return rpexp_write32(_reg(sio_hw->gpio_hi_out), himask);
}


rpexp_err_t rpexp_gpio_set_pulls(uint32_t gpio, bool up, bool down) {

    return rpexp_write_hw_mask(_reg(padsbank0_hw->io[gpio]),
        (bool_to_bit(up) << PADS_BANK0_GPIO0_PUE_LSB) | (bool_to_bit(down) << PADS_BANK0_GPIO0_PDE_LSB),
        PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

//----------------------------------------------------------------------------


rpexp_err_t rpexp_adc_block_enable(bool enable) {

    rpexp_err_t rpexp_err = adc_clock_init(enable);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = peripheral_enable(enable, RESETS_RESET_ADC_BITS);
    }

    return rpexp_err;
}


rpexp_err_t rpexp_adc_init(void) {

    rpexp_err_t rpexp_err = rpexp_adc_block_enable(false);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_adc_block_enable(true);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(adc_hw->cs),  ADC_CS_EN_BITS);
    }

    for (uint32_t escape = 0; rpexp_err == RPEXP_OK; escape++) {

        uint32_t adc_hw_cs;

        rpexp_err = rpexp_read32(_reg(adc_hw->cs), &adc_hw_cs);

        if (rpexp_err == RPEXP_OK && (adc_hw_cs & ADC_CS_READY_BITS)) {
            break;
        }

        if (escape > ADC_ESCAPE_COUNT) {
           rpexp_err = RPEXP_ERR_ADC_TIMEOUT;
        }
    }

    return rpexp_err;
}


rpexp_err_t rpexp_adc_gpio_init(uint32_t gpio) {

    if ((gpio >= NUM_BANK0_GPIOS) ||
        (gpio <= (NUM_BANK0_GPIOS - NUM_ADC_CHANNELS))) {
        return RPEXP_ERR_API_ARG;
    }

    // Select NULL function to make output driver hi-Z
    rpexp_err_t rpexp_err = set_gpio_function(gpio, GPIO_FUNC_NULL);

    // Disable digital pulls and digital receiver
    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_disable_gpio_pulls(gpio);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = set_gpio_input_enabled(gpio, 0);
    }

    return rpexp_err;
}


rpexp_err_t rpexp_adc_select_input(uint32_t channel) {

    if (channel >= NUM_ADC_CHANNELS) {
        return RPEXP_ERR_API_ARG;
    }

    return rpexp_write_hw_mask(_reg(adc_hw->cs), channel << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
}


rpexp_err_t rpexp_adc_get_selected_input(uint32_t *pinput) {

    if (!pinput) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = rpexp_read32(_reg(adc_hw->cs), pinput);

    *pinput  &= ADC_CS_AINSEL_BITS;
    *pinput >>= ADC_CS_AINSEL_LSB;

    if (rpexp_err != RPEXP_OK) {
        *pinput = 0;
    }

    return rpexp_err;
}


rpexp_err_t rpexp_adc_set_temp_sensor_enabled(bool enable) {

    if (enable) {
        return rpexp_hw_set_bits(_reg(adc_hw->cs), ADC_CS_TS_EN_BITS);
    } else {
        return rpexp_hw_clear_bits(_reg(adc_hw->cs), ADC_CS_TS_EN_BITS);
    }
}


rpexp_err_t rpexp_adc_read(uint32_t *padcreading) {

    uint32_t escape;

    if (!padcreading) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = rpexp_hw_set_bits(_reg(adc_hw->cs), ADC_CS_START_ONCE_BITS);

    for (escape = 0; escape <= ADC_ESCAPE_COUNT && rpexp_err == RPEXP_OK; escape++) {

        rpexp_err = rpexp_read32(_reg(adc_hw->cs), padcreading);

        if (rpexp_err == RPEXP_OK && (*padcreading & ADC_CS_READY_BITS)) {

            rpexp_err = rpexp_read32(_reg(adc_hw->result), padcreading);
            break;
        }
    }

    if (escape > ADC_ESCAPE_COUNT) {
        rpexp_err = RPEXP_ERR_ADC_TIMEOUT;
    }

    if (rpexp_err != RPEXP_OK) {
        *padcreading = 0ul;  // overwrite failed reads
    }

    return rpexp_err;
}


rpexp_err_t rpexp_time_get_ticks32(uint32_t *pticks) {

    if (!pticks) {
        return RPEXP_ERR_API_ARG;
    }

    return rpexp_read32(_reg(timer_hw->timerawl), pticks);
}


rpexp_err_t rpexp_time_get_ticks64(uint64_t *pticks) {

    if (!pticks) {
        return RPEXP_ERR_API_ARG;
    }

    uint32_t lo, hi;

    // Reading low latches the high value
    rpexp_err_t rpexp_err = rpexp_read32(_reg(timer_hw->timelr), &lo);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_read32(_reg(timer_hw->timehr), &hi);
    }

    if (rpexp_err == RPEXP_OK) {
        *pticks = ((uint64_t) hi << 32u) | lo;
    }

    return rpexp_err;
}


rpexp_err_t rpexp_rosc_set_div(uint32_t rosc_div) {

    if (rosc_div == 0 || rosc_div > 32) {
        return RPEXP_ERR_API_ARG;
    }

    if (rosc_div == 32) {
        rosc_div = 0;
    }

    rpexp_err_t rpexp_err = rpexp_write32(_reg(rosc_hw->div), (rosc_div + ROSC_DIV_VALUE_PASS));

    if (rpexp_err == RPEXP_OK) {
        // REstart timing for ROSC frequency measurement
        rpexp_err = get_timespec_and_tickcount(&rosc_time_count_freq);
    }
}


rpexp_err_t rpexp_rosc_get_div(uint32_t *pdiv) {

    rpexp_err_t rpexp_err = rpexp_read32(_reg(rosc_hw->div), pdiv);

    if (rpexp_err == RPEXP_OK) {

        *pdiv -= ROSC_DIV_VALUE_PASS;
        if (*pdiv == 0) {
            *pdiv = 32;
        }
    }
    else {
        *pdiv = 0;
    }

    return rpexp_err;
}


rpexp_err_t rpexp_rosc_get_freq_ab_bits(uint32_t *pfreq_bits32) {

    uint32_t freqa, freqb;

    if (!pfreq_bits32) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = rpexp_read32(_reg(rosc_hw->freqa), &freqa);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_read32(_reg(rosc_hw->freqb), &freqb);
    }

    if (rpexp_err == RPEXP_OK) {
        *pfreq_bits32 = ((freqb << 16)  | (freqa & 0xfffful));
    }

    return rpexp_err;
}


rpexp_err_t rpexp_rosc_set_freq_ab_bits(uint32_t freq_bits32) {

    rpexp_err_t rpexp_err = common_rosc_set_freq_ab_bits(freq_bits32);

    if (rpexp_err == RPEXP_OK) {
        // REstart timing for ROSC frequency measurement
        rpexp_err = get_timespec_and_tickcount(&rosc_time_count_freq);
    }

    return RPEXP_OK;
}


rpexp_err_t rpexp_rosc_zero_all_freq_ab_bits(void) {

    uint32_t freq_bits, starting_freq_bits;

    rpexp_err_t rpexp_err = rpexp_rosc_get_freq_ab_bits(&freq_bits);

    starting_freq_bits = freq_bits;

    // zero the freq a/b bits, one bit at a time to avoid glitches
    while (rpexp_err == RPEXP_OK && freq_bits) {
        freq_bits = rpexp_rosc_dec_freq_ab_bits(freq_bits);
        rpexp_err = common_rosc_set_freq_ab_bits(freq_bits);
    }

    if (rpexp_err == RPEXP_OK && freq_bits != starting_freq_bits) {
        // REstart timing for ROSC frequency measurement
        rpexp_err = get_timespec_and_tickcount(&rosc_time_count_freq);
    }

    return rpexp_err;
}


uint32_t rpexp_rosc_inc_freq_ab_bits(uint32_t freq32) {

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


uint32_t rpexp_rosc_dec_freq_ab_bits(uint32_t freq32) {

    // Work up, LS->MS, see comment in: rpexp_rosc_inc_freq_ab_bits()
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


rpexp_err_t rpexp_rosc_measure_clock_freq_khz(uint32_t *rosc_freq_khz, uint32_t min_sample_us) {

    rpexp_err_t rpexp_err;
    uint64_t delta_time_us;
    rosc_time_n_count_t current_rosc_time_n_count;

    *rosc_freq_khz = 0;

    do {
        rpexp_err = get_timespec_and_tickcount(&current_rosc_time_n_count);

        if (rpexp_err != RPEXP_OK) {
            break;
        }

        delta_time_us = current_rosc_time_n_count.time_us - rosc_time_count_freq.time_us;

        if (delta_time_us > (uint64_t) min_sample_us) {
            break;
        }

    } while (min_sample_us);

    if (rpexp_err == RPEXP_OK) {

        uint64_t delta_tick_count = current_rosc_time_n_count.tick_count - rosc_time_count_freq.tick_count;
        *rosc_freq_khz = (uint32_t) ((delta_tick_count * (uint64_t) 1000ull * (uint64_t) TICK_GENERATOR_CYCLES) / delta_time_us);

        // record frequency for any subsequent baudrate setting etc.
        rosc_time_count_freq.system_clock_frequency_khz = *rosc_freq_khz;

        // update the static datapoint for any subsequent calculation(s)
        rosc_time_count_freq = current_rosc_time_n_count;
    }

    return rpexp_err;
}


rpexp_err_t rpexp_rosc_set_faster_clock_freq(uint32_t target_rosc_clock_khz, uint32_t *measured_rosc_clock_khz) {

    uint32_t rosc_div;
    uint32_t rosc_freq_khz;
    uint32_t new_rosc_freq_khz;

    // Firstly, trim ROSC fully _down_ in frequency
    rpexp_err_t rpexp_err = rpexp_rosc_zero_all_freq_ab_bits();

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_rosc_measure_clock_freq_khz(measured_rosc_clock_khz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
    }

    if (rpexp_err == RPEXP_OK && (target_rosc_clock_khz < *measured_rosc_clock_khz)) {
        // it is not possible to slow things down further because the diver setting
        // can not be increased without [possibly] glitching the ROSC clock.
        // Return "OK" and leave the *caller* to decide if this is slow enough ...
        return RPEXP_OK;
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_rosc_get_div(&rosc_div);
    }

    if (rpexp_err == RPEXP_OK) {

        uint32_t new_div;

        for (new_div = rosc_div; new_div > 1; new_div--) {

            new_rosc_freq_khz = target_rosc_clock_khz * new_div;

            // this could be smarter for low target freqs, e.g. 10 MHz, to allow "centreing"
            // the freq a/b but settings to more easily allow for trimming but it will [still]
            // work optimally for higher target frquencies, such as 48 MHz, where there
            // is likely only one possible divider ratio setting anyway.
            if (new_rosc_freq_khz <= ROSC_MAX_FREQ_KHZ) {
                break;
            }
        }

        rpexp_err = rpexp_rosc_set_div(new_div);
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rosc_frequency_trim_up(target_rosc_clock_khz, measured_rosc_clock_khz);
    }

    return rpexp_err;
}

#if 0
rpexp_err_t rpexp_uart_enable(uart_hw_t *uart, bool enable) {

    if (uart != uart0 && uart != uart1) {
        return RPEXP_ERR_API_ARG;
    }

    rpexp_err_t rpexp_err = peripheral_clock_init(enable);

    if (rpexp_err == RPEXP_OK) {
        if (uart == uart0) {
            return peripheral_enable(enable, RESETS_RESET_UART0_BITS);
        }
    } else {
        return peripheral_enable(enable, RESETS_RESET_UART1_BITS);
    }
}


static rpexp_err_t uart_set_baudrate(uart_hw_t *uart, uint32_t req_baudrate, uint32_t *actual_baudrate) {

    if (!actual_baudrate) {
        return RPEXP_ERR_API_ARG;
    }

    *actual_baudrate = 0;

    if (!rosc_time_count_freq.system_clock_frequency_khz) {
        return RPEXP_ERR_CLOCK_FREQ_UNKNOWN;
    }

    uint32_t baud_rate_div = (8 * rosc_time_count_freq.system_clock_frequency_khz / req_baudrate);
    uint32_t baud_ibrd = baud_rate_div >> 7;
    uint32_t baud_fbrd;

    if (baud_ibrd == 0) {
        baud_ibrd = 1;
        baud_fbrd = 0;
    } else if (baud_ibrd >= 65535) {
        baud_ibrd = 65535;
        baud_fbrd = 0;
    }  else {
        baud_fbrd = ((baud_rate_div & 0x7f) + 1) / 2;
    }

    // Load PL011's baud divisor registers
    rpexp_err_t rpexp_err = rpexp_write32(_reg(uart->ibrd), baud_ibrd);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_write32(_reg(uart->fbrd), baud_fbrd);
    }

    // PL011 needs a (dummy) line control register write to latch in the
    // divisors. We don't want to actually change LCR contents here.
    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_hw_set_bits(_reg(uart->lcr_h), 0);
    }

    if (rpexp_err == RPEXP_OK) {
        *actual_baudrate = (4 * rosc_time_count_freq.system_clock_frequency_khz) / (64 * baud_ibrd + baud_fbrd);
    }

    return rpexp_err;
}


rpexp_err_t rpexp_uart_init(uart_hw_t *uart, uint32_t baudrate, uint32_t data_bits, uint32_t stop_bits) {

    uint32_t actual_baudrate;

    rpexp_err_t rpexp_err = uart_set_baudrate(uart, baudrate, &actual_baudrate);



}


rpexp_err_t rpexp_uart_deinit(uart_hw_t *uart) {

}


rpexp_err_t rpexp_uart_is_writable(uart_hw_t *uart) {

}

rpexp_err_t rpexp_uart_is_readable(uart_hw_t *uart) {

}

rpexp_err_t rpexp_uart_write_blocking(uart_hw_t *uart, const uint8_t *src, uint32_t len) {

}

rpexp_err_t rpexp_uart_read_blocking(uart_hw_t *uart, uint8_t *dst, uint32_t len) {

}

rpexp_err_t rpexp_uart_putc(uart_hw_t *uart, char c) {

}

rpexp_err_t rpexp_uart_puts(uart_hw_t *uart, const char *s) {

}

rpexp_err_t rpexp_uart_getc(uart_hw_t *uart) {

}

rpexp_err_t rpexp_uart_set_break(uart_hw_t *uart, bool en) {

}

#define uart0_hw ((uart_hw_t *)UART0_BASE)
#define uart1_hw ((uart_hw_t *)UART1_BASE)
#endif
