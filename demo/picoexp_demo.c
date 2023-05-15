/**
 * Copyright (c) 2023 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef PICO_BUILD
#include "pico/stdlib.h"
#endif
#ifdef __linux__
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#endif

#include <stdio.h>
#include <picoexp_port.h>
#include <picoexp.h>
#include <inttypes.h>

// "Mock up" *nix like timing functions for an RP2040 host
#ifdef PICO_BUILD
#define usleep(u)               sleep_us(u)
#endif


#define RAM_START               0x20000000ul
#define TEST_TOGGLE_PIN         15ul
#define PUSH_BUTTON_INPUT_PIN   1 // HI GPIO 1


#define RDWR_BUFF_SIZE_WORDS    2048ul
// Note: This can't be longer than the RP2040 RAM size, expressed as a 32-bit word count
#define RAM_TEST_LENGTH_WORDS   (4ul * RDWR_BUFF_SIZE_WORDS)    // In all 32 kBytes

#define LFSR32_SEED             0x12345678ul    // anything but 0!
#define LFSR_POLY32             0xEDB88320ul    // maximal length L->R shift


static uint32_t lfsr32 = LFSR32_SEED;

static uint32_t prand32_galois(void) {

    if (lfsr32 & 1ul) {
        lfsr32 >>= 1;
        lfsr32 ^= LFSR_POLY32;
    }  else {
        lfsr32 >>= 1;
    }

    return lfsr32;
}

static pexp_err_t read_chip_temperature(void);


int main() {

    pexp_err_t pexp_err;
    uint32_t rdwr_buffer[RDWR_BUFF_SIZE_WORDS];
    uint32_t i, ram_offset;
    uint32_t data;
    uint32_t freq_bits;
    uint32_t rosc_postdiv_freq_hz;
    uint32_t rosc_div;

#ifdef PICO_BUILD
    stdio_init_all();
#endif

    printf("\npico_expander_demo, built: %s %s\n\n", __DATE__, __TIME__);

    uint32_t step = 0;

    pexp_err = pexp_init();
    if (pexp_err != PEXP_OK) {
        printf("Error - failed to initialise RP2040 expander device, error: %d\n", pexp_err);
        goto end_tests;
    }

    step = 1;
    pexp_err = pexp_gpio_block_enable(true);
    if (pexp_err) goto end_tests;

    step = 2;
    pexp_err = pexp_gpio_init(TEST_TOGGLE_PIN);
    if (pexp_err) goto end_tests;

    step = 3;
    pexp_err = pexp_gpio_set_dir(TEST_TOGGLE_PIN, GPIO_DIR_OUT);
    if (pexp_err) goto end_tests;

    step = 4;
    pexp_err = pexp_gpio_set(TEST_TOGGLE_PIN);
    if (pexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 5;
    pexp_err = pexp_clock_gpio_init(25, GPIO_CLKOUT_CLK_REF, 500000);
    if (pexp_err) goto end_tests;

    step = 6;
    pexp_err = pexp_clock_gpio_init(21, GPIO_CLKOUT_CLK_REF, 1);
    if (pexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 10;
    pexp_err = pexp_gpio_hi_block_enable(true);
    if (pexp_err) goto end_tests;

    step = 11;
    pexp_err = pexp_gpio_hi_init(PUSH_BUTTON_INPUT_PIN);
    if (pexp_err) goto end_tests;

    step = 12;
    pexp_err = pexp_gpio_hi_set_dir(PUSH_BUTTON_INPUT_PIN, GPIO_DIR_IN);
    if (pexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 15;
    lfsr32 = LFSR32_SEED;  // need predictable 'random numbers' :)

    for (ram_offset = 0; ram_offset < (4 * RAM_TEST_LENGTH_WORDS); ) {
        for (i = 0; i < RDWR_BUFF_SIZE_WORDS; i++) {
            rdwr_buffer[i] = prand32_galois();
        }
        pexp_err = pexp_block_write32(RAM_START + ram_offset, rdwr_buffer, RDWR_BUFF_SIZE_WORDS);
        if (pexp_err) goto end_tests;

        ram_offset += (4 * RDWR_BUFF_SIZE_WORDS);
    }

    step = 16;
    lfsr32 = LFSR32_SEED;  // re-seed for checking

    for (ram_offset = 0; ram_offset < (4 * RAM_TEST_LENGTH_WORDS); ) {
        pexp_err = pexp_block_read32(RAM_START + ram_offset, rdwr_buffer, RDWR_BUFF_SIZE_WORDS);
        if (pexp_err) goto end_tests;

        for (i = 0; i < RDWR_BUFF_SIZE_WORDS && pexp_err == PEXP_OK; i++) {
            if (rdwr_buffer[i] != prand32_galois()) {
                printf("read verify failed at ram_offset: %" PRIX32 ", index: %" PRIX32 "\n", ram_offset, i);
                pexp_err = PEXP_ERR_TEST;
                goto end_tests;
            }
        }
        ram_offset += (4 * RDWR_BUFF_SIZE_WORDS);
    }

    //------------------------------------------------------------------------

    step = 20;  // get ROSC freq measurement

    pexp_err = pexp_rosc_measure_postdiv_clock_freq(&rosc_postdiv_freq_hz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
    if (pexp_err) goto end_tests;

    pexp_err = pexp_rosc_get_div(&rosc_div);
    if (pexp_err) goto end_tests;

    printf("ROSC clock freq (Hz): %" PRId32 "\n", rosc_postdiv_freq_hz);
    printf("ROSC divider setting: %" PRId32 "\n", rosc_div);

    //------------------------------------------------------------------------

    step = 25;

    pexp_err = pexp_rosc_set_freq_ab_bits(0);
    if (pexp_err) goto end_tests;

    for (int i = 0; i < 25; i++) {

        pexp_err = pexp_rosc_get_freq_ab_bits(&freq_bits);
        if (pexp_err) goto end_tests;

        pexp_err = pexp_rosc_measure_postdiv_clock_freq(&rosc_postdiv_freq_hz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
        if (pexp_err) goto end_tests;

        printf("ROSC freq setting: 0x%08" PRIX32 ", ROSC clock freq (Hz): %" PRId32 "\n", freq_bits, rosc_postdiv_freq_hz*16);

        freq_bits = pexp_rosc_inc_freq_ab_bits(freq_bits);

        pexp_err = pexp_rosc_set_freq_ab_bits(freq_bits);
        if (pexp_err) goto end_tests;
    }

    step = 26;

    for (int i = 0; i < 25; i++) {

        pexp_err = pexp_rosc_get_freq_ab_bits(&freq_bits);
        if (pexp_err) goto end_tests;

        pexp_err = pexp_rosc_measure_postdiv_clock_freq(&rosc_postdiv_freq_hz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
        if (pexp_err) goto end_tests;

        printf("ROSC freq setting: 0x%08" PRIX32 ", ROSC clock freq (Hz): %" PRId32 "\n", freq_bits, rosc_postdiv_freq_hz*16);

        freq_bits = pexp_rosc_dec_freq_ab_bits(freq_bits);

        pexp_err = pexp_rosc_set_freq_ab_bits(freq_bits);
        if (pexp_err) goto end_tests;
    }

    //------------------------------------------------------------------------

    step = 30;  // set new ROSC clock freq

    pexp_err = pexp_rosc_set_faster_postdiv_clock_freq(48*1000*1000, &rosc_postdiv_freq_hz);
    if (pexp_err) goto end_tests;

    pexp_err = pexp_rosc_get_div(&rosc_div);
    if (pexp_err) goto end_tests;

    pexp_err = pexp_rosc_get_freq_ab_bits(&freq_bits);
    if (pexp_err) goto end_tests;

    printf("ROSC clock freq (Hz): %" PRId32 "\n", rosc_postdiv_freq_hz);
    printf("ROSC divider setting: %" PRId32 "\n", rosc_div);
    printf("ROSC freq bits a/b:   %" PRIX32 "\n", freq_bits);

    //------------------------------------------------------------------------

    step = 35;
    pexp_err = pexp_adc_block_enable(true);
    if (pexp_err) goto end_tests;

    step = 36;
    pexp_err = pexp_adc_init();
    if (pexp_err) goto end_tests;

    step = 37;
    read_chip_temperature();
    if (pexp_err) goto end_tests;

    //------------------------------------------------------------------------

    if (pexp_err == PEXP_OK) {
        printf("LED flash loop, press 'BOOTSEL' button to end tests...\n");
        step = 99;

        while (1) {

            pexp_err = pexp_gpio_toggle(TEST_TOGGLE_PIN);
            if (pexp_err) goto end_tests;

            (void) usleep(50000);

            data = pexp_gpio_hi_get_all();

            if (data == (uint32_t)-1) {
                pexp_err = PEXP_ERR_TEST;
            }
            else if (0 == ((1ul << PUSH_BUTTON_INPUT_PIN) & data)) {

                pexp_err = pexp_gpio_clr(TEST_TOGGLE_PIN);
                if (pexp_err) goto end_tests;
                printf("Button pressed!\n");
                break;
            }
        }
    }

    //------------------------------------------------------------------------

end_tests:
    if (pexp_err == PEXP_OK) {
        printf("Success - demo code completed all steps\n\n");
    }
    else {
        printf("Error - demo code failed on step %" PRId32 ", error: %d\n\n", step, pexp_err);
    }

    //------------------------------------------------------------------------

    return pexp_err;
}


// These functions borrowed from: pico-examples/adc/onboard_temperature
static float read_onboard_temperature(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    uint16_t adc_result;

    if (0 == pexp_adc_read(&adc_result)) {

        float adc = (float) adc_result * conversionFactor;
        float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

        if (unit == 'C') {
            return tempC;
        } else if (unit == 'F') {
            return tempC * 9 / 5 + 32;
        }
    }

    return -1.0f;
}

/* Choose 'C' for Celsius or 'F' for Fahrenheit. */
#define TEMPERATURE_UNITS 'C'

static pexp_err_t read_chip_temperature(void) {

    /* The hardware AD converter is already initialised, enable
     * the onboard temperature sensor and select its channel
     */
    pexp_err_t pexp_err = pexp_adc_set_temp_sensor_enabled(true);

    if (pexp_err == PEXP_OK) {
        pexp_err = pexp_adc_select_input(4);
    }

    if (pexp_err == PEXP_OK) {
        float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
        printf("Onboard temperature = %.02f %c\n", temperature, TEMPERATURE_UNITS);
    }

    return pexp_err;
}
