/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICOEXP_H_
#define _PICOEXP_H_


// Please consult documentation regarding this setting
#define SWD_CLOCK_RATE_KHZ      500

#define TICK_GENERATOR_CYCLES   2

// This affects the frequency measurement resolution 
#define MIN_ROSC_FREQ_SAMPLE_TIME_US 2500


// Return error status codes
typedef enum {
    PEXP_OK = 0,
    PEXP_ERR_NO_SWDBB = 1,
    PEXP_ERR_DAP_DISCONNECTED = 2,
    PEXP_ERR_DAP_TARGET = 3,
    PEXP_ERR_DAP_FAULT = 4,
    PEXP_ERR_DAP_TIMEOUT = 5,
    PEXP_ERR_DAP_PARITY = 6,
    PEXP_ERR_API_ARG = 7,
    PEXP_ERR_NOT_INITED = 8,
    PEXP_ERR_IN_USE = 9,
    PEXP_ERR_RESET_TIMEOUT = 10,
    PEXP_ERR_ADC_TIMEOUT = 11,
    PEXP_ERR_TEST = 12,
    PEXP_ERR_UNSUPPORTED = 143
} pexp_err_t;


typedef enum {
    GPIO_DIR_IN = 0,
    GPIO_DIR_OUT = 1
} gpio_dir_t;


// Clock output API, selection options
typedef enum {
    GPIO_CLKOUT_CLKSRC_PLL_SYS = 0,
    GPIO_CLKOUT_CLKSRC_GPIN0 = 1,
    GPIO_CLKOUT_CLKSRC_GPIN1 = 2,
    GPIO_CLKOUT_CLKSRC_PLL_USB = 3,
    GPIO_CLKOUT_ROSC_CLKSRC = 4,
    GPIO_CLKOUT_XOSC_CLKSRC = 5,
    GPIO_CLKOUT_CLK_SYS = 6,
    GPIO_CLKOUT_CLK_USB = 7,
    GPIO_CLKOUT_CLK_ADC = 8,
    GPIO_CLKOUT_CLK_RTC = 9,
    GPIO_CLKOUT_CLK_REF = 10
} pexp_clkout_t;

/*
typedef enum {
    GPIO_SLEWRATE_SLOW = 0,
    GPIO_SLEWRATE_FAST = 1
} gpio_slewrate_t;


typedef enum {
    GPIO_DRIVESTRENGTH_2MA = 0,
    GPIO_DRIVESTRENGTH_4MA = 1,
    GPIO_DRIVESTRENGTH_8MA = 2,
    GPIO_DRIVESTRENGTH_12MA = 3
} gpio_drivestrength_t;
*/

/*! \brief Initialise the Pico expander sub-system
 *
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_init(void);


/*! \brief Enable or disable the GPIO HW
 *
 * Enable or disable the GPIO hardware.
 * Note: After expander initialisation, all hardare blocks are disabled
 *       and held in a reset state.  To use the GPIO API functions below
 *       the GPIO hardware must first be enabled so it can operate.
 *
 * \param enable        Enable (true) or disable (false) the hardware GPIO block
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_block_enable(bool enable);


/*! \brief Initialise a GPIO for I/O
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param gpio          GPIO number between 0 and 29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_init(uint32_t gpio);


/*! \brief Initialise multiple GPIOs for I/O
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param mask          Mask with 1 bit per GPIO initialise, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_init_mask(uint32_t mask);


/*! \brief Resets a GPIO back to the NULL function, i.e. disables it.
 *
 * \param gpio          GPIO number between 0 and 29 to disable
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_deinit(uint32_t gpio);


/*! \brief Resets multiple GPIOs back to the NULL function, i.e. disables them
 *
 * \param mask          Mask with 1 bit per GPIO disable, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_deinit_mask(uint32_t mask);


/*! \brief Set a single GPIO direction
 *
 * \param gpio          GPIO number between 0 and 29
 * \param gpio_dir      GPIO direction, see enum
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_set_dir(uint32_t gpio, gpio_dir_t gpio_dir);


/*! \brief Set a number of GPIOs to output
 *
 * Switch all GPIOs in "mask" to output
 *
 * \param mask          Bitmask of GPIOs to set to output, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_set_dir_out_masked(uint32_t mask);


/*! \brief Set a number of GPIOs to input
 *
 * \param mask          Bitmask of GPIOs to set to input, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_set_dir_in_masked(uint32_t mask);


/*! \brief Drive a single GPIO high
 *
 * \param gpio          GPIO number between 0 and 29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_set(uint32_t gpio);


/*! \brief Drive a single GPIO low
 *
 * \param gpio          GPIO number between 0 and 29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_clr(uint32_t gpio);


/*! \brief Toggle a single GPIO
 *
 * \param gpio          GPIO number between 0 and 29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_toggle(uint32_t gpio);


/*! \brief Drive high every GPIO appearing in mask
 *
 * \param mask          Bitmask of GPIO values to set, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_set_mask(uint32_t mask);


/*! \brief Drive low every GPIO appearing in mask
 *
 * \param mask          Bitmask of GPIO values to clear, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_clr_mask(uint32_t mask);


/*! \brief Toggle every GPIO appearing in mask
 *
 * \param mask          Bitmask of GPIO values to toggle, bits 0-29
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_toggle_mask(uint32_t mask);


/*! \brief Drive all [output] GPIOs according to the mask
 *
 * \param mask          Drive all GPIOs, 0-29, according to mask
 * \returns pexp_err_t  Operation result
 */
void pexp_gpio_put_all(uint32_t mask);


/*! \brief Read the values of all [input] GPIOs
 *
 * \returns uint32_t    Input bits or 0xFFFFFFFF if failed
 */
uint32_t pexp_gpio_get_all(void);


/*! \brief Output an (optionally divided) clock to gpios: 21, 23, 24 or 25
 *
 * NOTE:  Unless you are using a crystal or and external reference clock supplied
 *        to the XIN pin, you will *not* be able to use the PLLs and so the highest
 *        frequency available will be the ROSC, please see comments for pexp_set_xosc()
 *
 * \param gpio          GPIO numbers: 21, 23, 24, 25 ONLY, other GPIOs do not support this!
 * \param clk_sel       Clock selection, see definition of: pexp_clkout_t
 * \param div           The amount to divide the source clock frequency by - as a float.
 *                      This is useful to not overwhelm the GPIO pin with a fast clock.
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_clock_gpio_init(uint32_t gpio, pexp_clkout_t clk_sel, float div);

//----------------------------------------------------------------------------

/*! \brief De/initialise the  Hi (QSPI) GPIO block
 *
 * Enable or disable the Hi GPIO (QSPI pins) hardware.
 * Note: After expander initialisation, all hardare blocks are disabled
 *       and held in a reset state.  To use the Hi GPIO API functions below
 *       the Hi GPIO hardware must first be enabled so it can operate.
 *
 * \param enable        Enable (true) or disable (false) the Hi GPIO block hardware
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_block_enable(bool enable);


/*! \brief Initialise a Hi GPIO for I/O
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param higpio        Hi GPIO number between 0 and 5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_init(uint32_t higpio);


/*! \brief Initialise multiple Hi GPIOs for I/O
 *
 * Clear the output enable (i.e. set to input).
 * Clear any output value.
 *
 * \param himask        Mask with 1 bit per Hi GPIO initialise, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_init_mask(uint32_t himask);


/*! \brief Resets a Hi GPIO back to the NULL function, i.e. disables it.
 *
 * \param gpio          Hi GPIO number  between 0 and 5 to disable
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_deinit(uint32_t higpio);


/*! \brief Resets multiple Hi GPIOs back to the NULL function, i.e. disables them
 *
 * \param himask        Mask with 1 bit per Hi GPIO disable, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_deinit_mask(uint32_t himask);


/*! \brief Set a single Hi GPIO direction
 *
 * \param higpio        Hi GPIO number between 0 and 5
 * \param gpio_dir      Hi GPIO direction, see enum
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_set_dir(uint32_t higpio, gpio_dir_t gpio_dir);


/*! \brief Set a number of Hi GPIOs to output
 *
 * Switch all Hi GPIOs in "mask" to output
 *
 * \param himask        Bitmask of Hi GPIOs to set to output, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_set_dir_out_masked(uint32_t himask);


/*! \brief Set a number of Hi GPIOs to input
 *
 * \param himask        Bitmask of Hi GPIOs to set to input, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_set_dir_in_masked(uint32_t himask);


/*! \brief Drive high every Hi GPIO appearing in mask
 *
 * \param himask        Bitmask of Hi GPIO values to set, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_set_mask(uint32_t himask);


/*! \brief Drive low every Hi GPIO appearing in mask
 *
 * \param himask        Bitmask of Hi GPIO values to clear, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_clr_mask(uint32_t himask);


/*! \brief Toggle every Hi GPIO appearing in mask
 *
 * \param himask        Bitmask of Hi GPIO values to toggle, bits 0-5
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_hi_toggle_mask(uint32_t himask);


/*! \brief Drive all [output] Hi GPIOs according to the mask
 *
 * \param himask        Drive all Hi GPIOs, 0-5, according to mask
 * \returns pexp_err_t  Operation result
 */
void  pexp_gpio_hi_put_all(uint32_t himask);


/*! \brief Read the values of all [input] Hi GPIOs
 *
 * \returns uint32_t    GPIO Hi Input bits or 0xFFFFFFFF if failed
 */
uint32_t  pexp_gpio_hi_get_all(void);


/*! \brief Enable/disable pull up and/or pull downs on specific GPIO
 *
 * Note: On the RP2040, setting both pulls enables a "bus keep" function,
 * i.e. a weak pull to whatever is current high/low state of GPIO.
 * 
 * \param gpio          GPIO number between 0 and 29
 * \param up            Enable (true) or disable (false) the pull up
 * \param down          Enable (true) or disable (false) the pull down
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_gpio_set_pulls(uint32_t gpio, bool up, bool down);


/*! \brief Disable any pull ups and/or pull downs on specific GPIO
 *
 * \param gpio          GPIO number between 0 and 29
 * \returns pexp_err_t  Operation result
 */
static inline pexp_err_t pexp_disable_gpio_pulls(uint32_t gpio) {
        return pexp_gpio_set_pulls(gpio, 0, 0);
}

//----------------------------------------------------------------------------

/*! \brief Read a 32-bit word from the expander's memory map
 *
 * Read a 32-bit word from the specified address, store the result
 * the location provided by the supplied pointer.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 * It is provided to allow access resources not covered by the API.
 *
 * \param address       Expander address to read (memory or peripheral)
 * \param pdata         Pointer to 32-bit word to receive read value
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_read32(uint32_t address, uint32_t *pdata);


/*! \brief Write a 32-bit word into the expander's memory map
 *
 * Write the supplied 32-bit word into the the specified address.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 * It is provided to allow access resources not covered by the API.
 *
 * \param address       Expander address to read (memory or peripheral)
 * \param data          32-bit data word to be written
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_write32(uint32_t address, uint32_t data);


/*! \brief Read a block of 32-bit words from the expander's memory map
 *
 * Read a block 32-bit words starting from the specified address,
 * store the results the buffer provided by the supplied pointer.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 * It is provided to allow access resources not covered by the API.
 *
 * \param address       Expander start address to read (memory or peripheral)
 * \param pdata         Pointer to buffer of 32-bits words to receive data
 * \param length        Number of 32-bit words to read into the buffer
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_block_read32(uint32_t address, uint32_t *pdata, uint32_t length);


/*! \brief Write a block of 32-bit words into the expander's memory map
 *
 * Write a block 32-bit words starting from the specified address,
 * read from the buffer provided by the supplied pointer.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 * It is provided to allow access resources not covered by the API.
 *
 * \param address       Expander start address to write (memory or peripheral)
 * \param pdata         Pointer to source data buffer of 32-bit words
 * \param length        Number of 32-bit words to write into the expander
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_block_write32(uint32_t address, const uint32_t *pdata, uint32_t length);


/*! \brief Set new values for a sub-set of the bits in a hardware register
 *
 * Sets destination bit(s) to specified values, *if* the corresponding bit(s)
 * are set in the mask.
 * Note:  This will only work on RP2040 registers, not memory locations!
 *
 * This function makes use of RP2040's built-in register alias addresses.
 * The alias addresses allow single cycle setting, clearing or toggling of
 * RP2040 registers _if_ accessed via the appropriate address, rather than
 * requiring a read, modify & write sequence.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 *
 * \param addr          Address of writable register
 * \param values        New bit value(s)
 * \param write_mask    Mask of bits to change
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_write_hw_mask(uint32_t addr, uint32_t values, uint32_t write_mask);


/*! \brief Sets bits a 32-bit register in the expander's hardware
 *
 * Sret the specified bit(s) in a 32-bit register address.
 * Note:  This will only work on RP2040 registers, not memory locations!
 *
 * This function makes use of RP2040's built-in register alias addresses.
 * The alias addresses allow single cycle setting, clearing or toggling of
 * RP2040 registers _if_ accessed via the appropriate address, rather than
 * requiring a read, modify & write sequence.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 *
 * \param address       (base) Register address to modify
 * \param mask          Bitmask of bits to set in the register
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_hw_set_bits(uint32_t addr, uint32_t mask);


/*! \brief Clears bits a 32-bit register in the expander's hardware
 *
 * Clear the specified bit(s) in a 32-bit register address.
 * Note:  This will only work on RP2040 registers, not memory locations!
 *
 * This function makes use of RP2040's built-in register alias addresses.
 * The alias addresses allow single cycle setting, clearing or toggling of
 * RP2040 registers _if_ accessed via the appropriate address, rather than
 * requiring a read, modify & write sequence.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 *
 * \param address       (base) Register address to modify
 * \param mask          Bitmask of bits to clear in the register
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_hw_clear_bits(uint32_t addr, uint32_t mask);


/*! \brief Toggle bits a 32-bit register in the expander's hardware
 *
 * Toggle the specified bit(s) in a 32-bit register address.
 * Note:  This will only work on RP2040 registers, not memory locations!
 *
 * This function makes use of RP2040's built-in register alias addresses.
 * The alias addresses allow single cycle setting, clearing or toggling of
 * RP2040 registers _if_ accessed via the appropriate address, rather than
 * requiring a read, modify & write sequence.
 *
 * Note:  To make use of this you will need to be familiar with the
 * RP2040 memory map and the operation of the peripherals.
 *
 * \param address       (base) Register address to modify
 * \param mask          Bitmask of bits to toggle in the register
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_hw_xor_bits(uint32_t addr, uint32_t mask);


/*! \brief De/initialise the ADC HW
 *
 * Enable or disable the ADC hardware.
 * Note: After expander initialisation, all hardware blocks are disabled
 *       and held in a reset state.  To use the ADC API functions below
 *       the ADC hardware must first be enabled so it can operate.
 *
 * \param enable        Enable (true) or disable (false) the hardware ADC block
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_block_enable(bool enable);


/*! \brief Initialise the ADC
 *
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_init(void);


/*! \brief Initialise gpio for use as an ADC pin
 *
 * Prepare a GPIO for use with ADC by disabling all digital functions.
 *
 * \param gpio          GPIO numbers: 26, 27, 28, 29 ONLY, other GPIOs do not support this!
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_gpio_init(uint32_t gpio);


/*! \brief  ADC input select
 *
 * Select an ADC input. 0...3 are GPIOs 26...29 respectively.
 * Input 4 is the onboard temperature sensor.
 *
 * \param input         ADC input to select
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_select_input(uint32_t input);


/*! \brief  Get the currently selected ADC input channel
 *
 *  Get a reading from the currently selected input channel.
 *  Channels 0...3 are GPIOs 26...29 respectively.
 *  Channel 4 is the onboard temperature sensor.
 *
 * \param pinput        Pointer to 32-bit word to receive input channel
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_get_selected_input(uint32_t *pinput);


/*! \brief Enable or disable the onchip temperature sensor
 *
 * \param enable        Enable (true) or disable (false) the onchip temperature sensor
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_set_temp_sensor_enabled(bool enable);


/*! \brief Perform a single ADC conversion
 *
 * \param enable        Enable (true) or disable (false) the onchip temperature sensor
 * \returns pexp_err_t  Operation result
 */


/*! \brief Perform a single conversion
 *
 *  Performs an ADC conversion, waits for the result, and then returns it.
 *
 * \param padc          Pointer to 16-bit word to receive the ADC reading
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_adc_read(uint16_t *padc);

//----------------------------------------------------------------------------

/*! \brief Read a 32-bit tick count from the timer's LS word
 *
 * Read a 32-bit tick count from the free-running timer, store the result
 * in the location provided by the supplied pointer.
 *
 * The timer hardware ticks from the expander's tick generator hardware.
 * Please see clock and divider details in tick generator documentation.
 *
 * The timer is enabled during the expander's initialisation sequence
 * and runs continuously from that point on.
 *
 * Note: The 32-bit tick count value can *wrap* during feasible execution times.
 *
 * \param pticks        Pointer to 32-bit word to receive the tick count value
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_time_get_ticks32(uint32_t *pticks);


/*! \brief Read a 64-bit tick count from the timer
 *
 * Read a 64-bit tick count from the free-running timer, store the result
 * in the location provided by the supplied pointer.
 *
 * The timer hardware ticks from the expander's tick generator hardware.
 * Please see clock and divider details in tick generator documentation.
 *
 * The timer is enabled during the expander's initialisation sequence
 * and runs continuously from that point on.
 *
 * \param pticks        Pointer to 64-bit word to receive the tick count value
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_time_get_ticks64(uint64_t *pticks);


/*! \brief Read the Ring Oscillator's divider ratio
 *
 * Read the ROSC 'div' register, calculate and return the division ratio
 * which will be between  1 to 32.
 *
 * \param pdiv          Pointer to 32-bit word to receive the ROSC divider ratio
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_rosc_get_div(uint32_t *pdiv);


/*! \brief Set the Ring Oscillator's divider ratio
 *
 * Sets the ROSC 'div' register with a value calculated from the requested
 * division ratio.
 *
 * Note: The divider ratio can NOT be *increased* without causing glitches
 * in the clock output to the rest of the system.  If the expander is running
 * from ROSC clock this may impair or totally halt subsequent operation.
 *
 * Note: The initial division ratio, from system reset, is 16.  In practical
 * terms, this means the division ration can only be reduced from 16 unless the
 * system is (a) completely re-initialised first or (b) the system is not running from
 * the ROSC clock when the divide ratio is increased.
 *
 * \param div           New ROSC divider ratio, 1..32  !Note comments above!
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_rosc_set_div(uint32_t div);


pexp_err_t pexp_rosc_set_range(uint32_t range);


pexp_err_t pexp_rosc_get_freq_ab_bits(uint32_t *pfreq);


pexp_err_t pexp_rosc_set_freq_ab_bits(uint32_t freq);


uint32_t pexp_rosc_inc_freq_ab_bits(uint32_t freq32);


uint32_t pexp_rosc_dec_freq_ab_bits(uint32_t freq32);


/*! \brief Calculate the ROSC system clock frequency, update time/count snapshot
 *
 * This function takes a reading of the current *host* system's time and the
 * expander's tick counter value.  Using those current values, and previously
 * snapshot host time and expander tick counter readings, the frequency at which
 * the tick counter is advancing can be calculated.
 *
 * Post calculation, this function updates the time and tick count snapshot values
 * with those just taken, so that a subsequent invocations will correctly track
 * any changes in the ROSC clock frequency.
 *
 * There are two reasons the ROSC system clock frequency might change:
 *
 * 1) If the ROSC's internal divider setting is changed, see further comments below.
 * 2) By a change in temperature and/or operating voltage, see RP2040 datasheet.
 *
 * Note: There is no requirement for 'time-accurate' use of this API because the
 * host time is measured each time it is invoked.  It can be scheduled at any
 * convenient point in the host's operation.
 *
 * But, if the function is invoked immediately after the initial snapshot is taken,
 * or in rapid succession after a previous call to it, the calculated frequency
 * may be inaccurate due to lack of precision.
 *
 * To avoid inaccurate results and ease the use of this API, the 'min_sample_us'
 * argument can be used to make the function delay and repeatedly sample host time
 * and tick count until a specified number of microseconds have elapsed since the
 * previous snapshot data was captured.  The frequency calculation then proceeds
 * and the snapshot is updated.
 *
 * A minimum sample period of at least MIN_ROSC_FREQ_SAMPLE_TIME_US is recommended.
 *
 * If this function is invoked periodically, at a period greater than the minimum
 * sample time it will never busy-wait since the minimum period will have already
 * elapsed by the time the function is invoked again.
 * This is the recommended mode of operation!
 *
 * If 'min_sample_us' is zero, this function will not busy-wait and will only make
 * a single time and tick count reading, perform the calculation from those and
 * update the snapshot with the latest reading.  This host must ensure that the
 * use of the API is timely!  Please see: MIN_ROSC_FREQ_SAMPLE_TIME_US
 *
 * The ROSC system clock frequency is the tick counter frequency multiplied by
 * TICK_GENERATOR_CYCLES since the tick generator prescales the (tick) timer;
 * this function takes the TICK_GENERATOR_CYCLES setting into account and returns
 * the ROSC system clock rate.
 *
 * Note: the 'ROSC system clock' itself is the frequency *after* the ROSC's internal
 * divider stage; the actual ROSC frequency can be higher the clock it generates
 * for system as a whole.
 *
 * The ROSC divider ratio can be read using pexp_rosc_get_div(), it will range
 * between 1 and 32, it has a power-up default value of 16.
 *
 * \param rosc_freq_hz  Pointer to 32-bit word to receive the ROSC clock freq in Hz
 * \param min_sample_us Observe minimum period (us) since start / last calculation.
 *                      If 0, do not observe any minimum sample time criteria.
 * \returns pexp_err_t  Operation result
 */
pexp_err_t pexp_rosc_measure_postdiv_clock_freq(uint32_t *rosc_freq_hz, uint32_t min_sample_us);


pexp_err_t pexp_rosc_set_faster_postdiv_clock_freq(uint32_t target_rosc_postdiv_clock_hz,
                                                   uint32_t *measured_rosc_postdiv_clock_hz);

//----------------------------------------------------------------------------

// Then a set of simple clock init code, with some parameters to set up the PLL (etc.)
pexp_err_t pexp_set_rosc(void);
pexp_err_t pexp_set_xosc(void);  // this can fail

#endif // _PICOEXP_H_
