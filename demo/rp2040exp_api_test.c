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
#endif

#include <stdio.h>
#include <rp2040exp_port.h>
#include <rp2040exp.h>
#include <rp2040exp_port_api.h>
#include <inttypes.h>

#include "blink_prog.h"
#include "demo_stepper.h"


#define RAM_START               0x20000000ul
#define TEST_TOGGLE_PIN         15ul
#define PUSH_BUTTON_INPUT_PIN   1 // HI GPIO 1

#define TEMPERATURE_FORMAT      'C'

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

static rpexp_err_t read_adc_gpio_voltage(uint32_t channel);
static rpexp_err_t read_chip_temperature(float *ptemp);


int main() {

    rpexp_err_t rpexp_err;
    uint32_t rdwr_buffer[RDWR_BUFF_SIZE_WORDS];
    uint32_t i, ram_offset;
    uint32_t data;
    uint32_t freq_bits;
    uint32_t rosc_clock_freq_khz;
    uint32_t rosc_div;

#ifdef PICO_BUILD
    stdio_init_all();
#endif

    printf("\nrp2040_expander_demo, built: %s %s\n\n", __DATE__, __TIME__);

    uint32_t step = 0;

    rpexp_err = rpexp_init();
    if (rpexp_err != RPEXP_OK) {
        printf("Error - failed to initialise RP2040 expander device, error: %d\n", rpexp_err);
        goto end_tests;
    }

    step = 1;
    rpexp_err = rpexp_gpio_block_enable(true);
    if (rpexp_err) goto end_tests;

    step = 2;
    rpexp_err = rpexp_gpio_init(TEST_TOGGLE_PIN);
    if (rpexp_err) goto end_tests;

    step = 3;
    rpexp_err = rpexp_gpio_set_dir(TEST_TOGGLE_PIN, GPIO_DIR_OUT);
    if (rpexp_err) goto end_tests;

    step = 4;
    rpexp_err = rpexp_gpio_set(TEST_TOGGLE_PIN);
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 5;
    rpexp_err = rpexp_clock_gpio_init(25, GPIO_CLKOUT_CLK_REF, 5000000);
    if (rpexp_err) goto end_tests;

    step = 6;
    rpexp_err = rpexp_clock_gpio_init(21, GPIO_CLKOUT_CLK_REF, 10);
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 10;
    rpexp_err = rpexp_gpio_hi_block_enable(true);
    if (rpexp_err) goto end_tests;

    step = 11;
    rpexp_err = rpexp_gpio_hi_init(PUSH_BUTTON_INPUT_PIN);
    if (rpexp_err) goto end_tests;

    step = 12;
    rpexp_err = rpexp_gpio_hi_set_dir(PUSH_BUTTON_INPUT_PIN, GPIO_DIR_IN);
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 15;
    lfsr32 = LFSR32_SEED;  // need predictable 'random numbers' :)

    for (ram_offset = 0; ram_offset < (4 * RAM_TEST_LENGTH_WORDS); ) {
        for (i = 0; i < RDWR_BUFF_SIZE_WORDS; i++) {
            rdwr_buffer[i] = prand32_galois();
        }
        rpexp_err = rpexp_block_write32(RAM_START + ram_offset, rdwr_buffer, RDWR_BUFF_SIZE_WORDS);
        if (rpexp_err) goto end_tests;

        ram_offset += (4 * RDWR_BUFF_SIZE_WORDS);
    }

    step = 16;
    lfsr32 = LFSR32_SEED;  // re-seed for checking

    for (ram_offset = 0; ram_offset < (4 * RAM_TEST_LENGTH_WORDS); ) {
        rpexp_err = rpexp_block_read32(RAM_START + ram_offset, rdwr_buffer, RDWR_BUFF_SIZE_WORDS);
        if (rpexp_err) goto end_tests;

        for (i = 0; i < RDWR_BUFF_SIZE_WORDS && rpexp_err == RPEXP_OK; i++) {
            if (rdwr_buffer[i] != prand32_galois()) {
                printf("read verify failed at ram_offset: %" PRIX32 ", index: %" PRIX32 "\n", ram_offset, i);
                rpexp_err = RPEXP_ERR_TEST;
                goto end_tests;
            }
        }
        ram_offset += (4 * RDWR_BUFF_SIZE_WORDS);
    }

    //------------------------------------------------------------------------

    step = 19;  // get ROSC freq measurement
    const int8_t stepper_motor_init_list[] = {0, 1, 2, -1};

    rpexp_err = stepper_init(stepper_motor_init_list);
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 20;  // get ROSC freq measurement
    rpexp_err = rpexp_rosc_measure_clock_freq_khz(&rosc_clock_freq_khz, MIN_ROSC_FREQ_SAMPLE_TIME_US);
    if (rpexp_err) goto end_tests;

    printf("Boot ROSC clock frequency (kHz): %" PRId32 "\n", rosc_clock_freq_khz);

    step = 25;  // set new ROSC clock freq

    for (uint32_t new_rosc_clk_freq_mhz = 8; new_rosc_clk_freq_mhz <= 48; new_rosc_clk_freq_mhz += 8) {

        uint32_t new_rosc_clk_freq_khz = 1000ul * new_rosc_clk_freq_mhz;

        rpexp_err = rpexp_rosc_set_faster_clock_freq(new_rosc_clk_freq_khz, &rosc_clock_freq_khz);
        if (rpexp_err) goto end_tests;

        rpexp_err = rpexp_rosc_get_freq_ab_bits(&freq_bits);
        if (rpexp_err) goto end_tests;

        rpexp_err = rpexp_rosc_get_div(&rosc_div);
        if (rpexp_err) goto end_tests;

        printf("Requested ROSC clock freq (kHz): %5" PRId32 ", ", new_rosc_clk_freq_khz);
        printf("resulting freq: %5" PRId32 ", ", rosc_clock_freq_khz);

        // integer maths to create a 'part-per thousand' metric
        rosc_clock_freq_khz *= 20;
        uint32_t result_ppk = rosc_clock_freq_khz / (new_rosc_clk_freq_khz / 100);
        result_ppk += 1; // round
        result_ppk /= 2;

        printf("accuracy metric, parts per 1000: %4" PRId32 "\n", result_ppk);
    }

    //------------------------------------------------------------------------

    step = 30;  // Initialise UARTs
    rpexp_err = rpexp_uart_enable(UART0, true);
    if (rpexp_err) goto end_tests;

    step = 31;  // Initialise UARTs
    rpexp_err = rpexp_uart_enable(UART1, true);
    if (rpexp_err) goto end_tests;

    step = 32;  // Initialise UARTs
    rpexp_err = rpexp_uart_init(UART0, 115200, 8, 1, UART_NO_PARITY, false, false);
    if (rpexp_err) goto end_tests;

    step = 33;  // Initialise UARTs
    rpexp_err = rpexp_uart_init(UART1, 115200, 8, 1, UART_NO_PARITY, false, false);
    if (rpexp_err) goto end_tests;

    step = 34;
    const int8_t uart_gpio_list[] = {0, 1, 2, 3, 4, -1};
    rpexp_err = rpexp_uart_assign_gpios(uart_gpio_list);
    if (rpexp_err) goto end_tests;

    step = 35;
    rpexp_err = rpexp_uart_puts(UART0, "\r\nHello UART world!\r\nThis is a test message brought to you by expander - built: ");
    if (rpexp_err) goto end_tests;

    step = 36;
    rpexp_err = rpexp_uart_puts(UART0,  __DATE__);
    if (rpexp_err) goto end_tests;

    step = 37;
    rpexp_err = rpexp_uart_putc(UART0, (char)' ');
    if (rpexp_err) goto end_tests;

    step = 38;
    rpexp_err = rpexp_uart_puts(UART0,  __TIME__);
    if (rpexp_err) goto end_tests;

    step = 39;
    rpexp_err = rpexp_uart_puts(UART0, "\r\n");
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 40;
    rpexp_err = rpexp_uart_puts(UART0, "UART0 getc() test; typed characters will be printed - press 'Escape' to exit\r\n");

    do {
        int data;
        rpexp_err = rpexp_uart_getc(UART0, &data);

        if (!rpexp_err) {
            if (data == (int)'\e') {
                printf("\nEscape\n");
                break;
            }
            printf("%c", data);
        }

    } while (!rpexp_err);

    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 50;
    rpexp_err = rpexp_adc_block_enable(true);
    if (rpexp_err) goto end_tests;

    step = 51;
    rpexp_err = rpexp_adc_init();
    if (rpexp_err) goto end_tests;

    step = 52;
    rpexp_err = read_adc_gpio_voltage(0);  // GPIO26
    if (rpexp_err) goto end_tests;

    step = 53;
    float temperature;
    read_chip_temperature(&temperature);
    if (rpexp_err) goto end_tests;

    printf("Onboard temperature = %.01f %c\n", temperature, TEMPERATURE_FORMAT);

    read_chip_temperature(&temperature);
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    printf("LED flash loop, press 'BOOTSEL' button to continue ...\n");
    step = 55;

    while (1) {

        rpexp_err = rpexp_gpio_toggle(TEST_TOGGLE_PIN);
        if (rpexp_err) goto end_tests;

        port_sleep_us_32(50000);

        data = rpexp_gpio_hi_get_all();

        if (data == (uint32_t)-1) {
            rpexp_err = RPEXP_ERR_TEST;
        }
        else if (0 == ((1ul << PUSH_BUTTON_INPUT_PIN) & data)) {

            rpexp_err = rpexp_gpio_clr(TEST_TOGGLE_PIN);
            if (rpexp_err) goto end_tests;
            printf("Button pressed!\n");
            break;
        }
    }
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

    step = 60;
    printf("Run code from RAM ...\n");
    rpexp_err = rpexp_load_and_run_ram_program((const uint32_t *)blink_blink_bin,
                                              ((sizeof(blink_blink_bin) + 3) / 4));
    if (rpexp_err) goto end_tests;

    //------------------------------------------------------------------------

end_tests:
    if (rpexp_err == RPEXP_OK) {
        printf("Success - demo code completed all steps\n\n");
    }
    else {
        printf("Error - demo code failed on step %" PRId32 ", error: %d\n\n", step, rpexp_err);
    }

    return rpexp_err;

}  // main()


//----------------------------------------------------------------------------


static rpexp_err_t read_adc_gpio_voltage(uint32_t channel) {

    uint32_t adc_result;

    rpexp_err_t rpexp_err = rpexp_adc_select_input(channel);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_adc_read(&adc_result);
    }

    if (rpexp_err == RPEXP_OK) {
        uint32_t mV = adc_result * 3300ul / 4095ul;
        printf("ADC reading: %d, voltage (mV): %ld\n", adc_result, mV);
    }

    return rpexp_err;
}


// This functionality borrowed from: pico-examples/adc/onboard_temperature
static rpexp_err_t read_chip_temperature(float *ptemp) {

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversionFactor = 3.3f / (1 << 12);
    uint32_t adc_result;

    // The hardware AD converter should already be initialised,
    // enable the onboard temperature sensor and select its channel
    rpexp_err_t rpexp_err = rpexp_adc_set_temp_sensor_enabled(true);

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_adc_select_input(4);  // termperature
    }

    if (rpexp_err == RPEXP_OK) {
        rpexp_err = rpexp_adc_read(&adc_result);
    }

    if (rpexp_err == RPEXP_OK) {

        float adc = (float) adc_result * conversionFactor;
        *ptemp= 27.0f - (adc - 0.706f) / 0.001721f;

        if (TEMPERATURE_FORMAT == 'F') {
            *ptemp *= 9 / 5 + 32;
        }

        rpexp_err = rpexp_adc_set_temp_sensor_enabled(false);
    }

    return rpexp_err;
}
