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
#include <rp2040exp_port.h>
#include <rp2040exp.h>
#include <inttypes.h>
#include <rp2040exp_port_api.h>
#include "demo_7seg.h"


#define MAX7219_CLK                 22
#define MAX7219_DAT                 27
#define MAX7219_CS                  28
#define MAX7219_GPIOS               (MAX7219_CLK | MAX7219_DAT | MAX7219_CS)

#define NUM_OF_DIGITS               8

/* 16 bit format: xxxx CMDS DATADATA */
#define MAX7219_CMD_SHIFT           8
#define MAX7219_NOP                 ((uint16_t)(0x0 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_0               ((uint16_t)(0x1 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_1               ((uint16_t)(0x2 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_2               ((uint16_t)(0x3 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_3               ((uint16_t)(0x4 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_4               ((uint16_t)(0x5 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_5               ((uint16_t)(0x6 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_6               ((uint16_t)(0x7 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_7               ((uint16_t)(0x8 << MAX7219_CMD_SHIFT))
#define MAX7219_DIG_INC             ((uint16_t)(0x1 << MAX7219_CMD_SHIFT))
#define MAX7219_DECODE_MODE         ((uint16_t)(0x9 << MAX7219_CMD_SHIFT))
#define MAX7219_INTENSITY           ((uint16_t)(0xA << MAX7219_CMD_SHIFT))
#define MAX7219_SCAN_LIMIT          ((uint16_t)(0xB << MAX7219_CMD_SHIFT))
#define MAX7219_SHUTDOWN_MODE       ((uint16_t)(0xC << MAX7219_CMD_SHIFT))
#define MAX7219_DISPLAY_TEST        ((uint16_t)(0xF << MAX7219_CMD_SHIFT))
#define SEG_DP                      0x80
/* Maxim Semiconductor use some bespoke 7 seg defs... */
#define SEG_A                       0x40
#define SEG_B                       0x20
#define SEG_C                       0x10
#define SEG_D                       0x08
#define SEG_E                       0x04
#define SEG_F                       0x02
#define SEG_G                       0x01

/* NOTE:  This is a functional map of iluminated segments */
static const uint8_t character_map[] = {
    (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F        ),   // 0
    (        SEG_B | SEG_C                                ),   // 1
    (SEG_A | SEG_B |         SEG_D | SEG_E |         SEG_G),   // 2
    (SEG_A | SEG_B | SEG_C | SEG_D |                 SEG_G),   // 3
    (        SEG_B | SEG_C |                 SEG_F | SEG_G),   // 4
    (SEG_A |         SEG_C | SEG_D |         SEG_F | SEG_G),   // 5
    (SEG_A |         SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),   // 6
    (SEG_A | SEG_B | SEG_C                                ),   // 7
    (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),   // 8
    (SEG_A | SEG_B | SEG_C |                 SEG_F | SEG_G),   // 9
    (SEG_A | SEG_B | SEG_C |         SEG_E | SEG_F | SEG_G),   // A
    (                SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),   // b
    (SEG_A |                 SEG_D | SEG_E | SEG_F        ),   // C
    (        SEG_B | SEG_C | SEG_D | SEG_E |         SEG_G),   // d
    (SEG_A |                 SEG_D | SEG_E | SEG_F | SEG_G),   // E
    (SEG_A |                         SEG_E | SEG_F | SEG_G),   // F
    (SEG_A |         SEG_C | SEG_D | SEG_E | SEG_F        ),   // G
    (                SEG_C |         SEG_E | SEG_F | SEG_G),   // h
    (                                SEG_E | SEG_F        ),   // i
    (        SEG_B | SEG_C | SEG_D | SEG_E                ),   // j
    (        SEG_B | SEG_C |         SEG_E | SEG_F | SEG_G | SEG_DP),   // k
    (                        SEG_D | SEG_E | SEG_F        ),   // l
    (                SEG_C |         SEG_E |         SEG_G | SEG_DP),   // m
    (                SEG_C |         SEG_E |         SEG_G),   // n
    (                SEG_C | SEG_D | SEG_E |         SEG_G),   // o
    (SEG_A | SEG_B |                 SEG_E | SEG_F | SEG_G),   // p
    (SEG_A | SEG_B | SEG_C |                 SEG_F | SEG_G | SEG_DP),   // q
    (                                SEG_E |         SEG_G),   // r
    (SEG_A |         SEG_C | SEG_D |         SEG_F | SEG_G),   // s
    (                        SEG_D | SEG_E | SEG_F | SEG_G),   // t
    (        SEG_B | SEG_C | SEG_D | SEG_E | SEG_F        ),   // U
    (                SEG_C | SEG_D | SEG_E                ),   // v
    (        SEG_B | SEG_C | SEG_D | SEG_E | SEG_F |        SEG_DP),   // w
    (        SEG_B | SEG_C |         SEG_E | SEG_F | SEG_G),   // x
    (        SEG_B | SEG_C | SEG_D |         SEG_F | SEG_G),   // y
    (SEG_A | SEG_B |         SEG_D | SEG_E |         SEG_G),   // z
    (0)                                                        // SPACE
};

#define NUMBER_CHARACTERS       NUM_ELES(character_map)
#define CHAR_SPACE              (NUMBER_CHARACTERS - 1)

typedef struct
{
    uint8_t character;
    uint8_t segments;

} lookup_punctuation_t;


static const lookup_punctuation_t lookup_punctuation[] =
{
    { (uint8_t) '.',  (SEG_DP)},                          // .
    { (uint8_t) '!',  (SEG_B | SEG_DP)},                  // !
    { (uint8_t) '?',  (SEG_A | SEG_B | SEG_E | SEG_G)},   // ?
    { (uint8_t) '=',  (SEG_D | SEG_G)},                   // =
    { (uint8_t) '-',  (SEG_G)},                           // -
    { (uint8_t) '\'', (SEG_F)},                           // '
    { (uint8_t) '\"', (SEG_B | SEG_F)}                    // "
};

#define NUMBER_PUNCTUATION      NUM_ELES(lookup_punctuation)

static uint8_t segment_data[NUM_OF_DIGITS];


static uint8_t find_punctuation(uint8_t c)
{
    for (int i = 0; i < NUMBER_PUNCTUATION; i++)
    {
        if (lookup_punctuation[i].character == c)
        {
            return lookup_punctuation[i].segments;
        }
    }

    return 0;
}


static uint8_t lookup_character(uint8_t c)
{
    uint8_t r;

    if (0 == (r = find_punctuation(c)))
    {
        r = NUMBER_CHARACTERS;

        if (c >= (uint8_t)'0' && c <= (uint8_t)'9')
        {
            r = c - (uint8_t) '0';
        }
        else if (c >= (uint8_t)'a' && c <= (uint8_t)'z')
        {
            r = 10 + c - (uint8_t) 'a';
        }
        else if (c >= (uint8_t)'A' && c <= (uint8_t)'Z')
        {
            r = 10 + c - (uint8_t) 'A';
        }
        else if (c == (uint8_t)' ')
        {
            r = CHAR_SPACE;
        }

        if (r < NUMBER_CHARACTERS)
        {
            r = character_map[r];
        }
        else
        {
            r = (SEG_DP);
        }
    }

    return r;
}


static void drive_gpio(uint8_t gpio, bool on) {

    if (on) {
        (void) rpexp_gpio_set(gpio);
    } else {
        (void) rpexp_gpio_clr(gpio);
    }
}


static void max2719_send_u16(uint16_t word) {

    drive_gpio(MAX7219_CS, 0);

    for (int i = 0; i < 16; i++, word <<= 1) {

        if (word & 0x8000) {
            drive_gpio(MAX7219_DAT, 1);
        } else {
            drive_gpio(MAX7219_DAT, 0);
        }

        drive_gpio(MAX7219_CLK, 1);
        drive_gpio(MAX7219_CLK, 0);
    }

    drive_gpio(MAX7219_CS, 1);
}


static void display_init(void) {

    rpexp_gpio_block_enable(true);
    rpexp_gpio_init(MAX7219_GPIOS);
    rpexp_gpio_set_dir(MAX7219_GPIOS, GPIO_DIR_OUT);

    drive_gpio(MAX7219_CS, 1);
    drive_gpio(MAX7219_DAT, 0);
    drive_gpio(MAX7219_CLK, 0);

    max2719_send_u16(MAX7219_NOP           | 0);  // Send NOP, just to pipeclean
    max2719_send_u16(MAX7219_DISPLAY_TEST  | 0);  // NOT display test
    max2719_send_u16(MAX7219_DECODE_MODE   | 0);  // RAW segment data
    max2719_send_u16(MAX7219_SCAN_LIMIT    | (NUM_OF_DIGITS - 1));  // Scan all
    max2719_send_u16(MAX7219_INTENSITY     | 15); // Intensity 0-15
    max2719_send_u16(MAX7219_SHUTDOWN_MODE | 1); // NOT shutdown!
}


static void display_update(void) {

    for (int i = 0; i < NUM_OF_DIGITS; i++) {
        max2719_send_u16((MAX7219_DIG_0 + (i << MAX7219_CMD_SHIFT)) | (uint16_t)segment_data[i]);
    }
}


static void insert_seg_data_rh(uint8_t seg_data) {

    for (int i = NUM_OF_DIGITS; i > 0; i--) {
        segment_data[i] = segment_data[i - 1];
    }

    segment_data[0] = seg_data;

    display_update();
}

/*---------------------------------------------------------------------------*/

void s7_led_display_init(void) {

    for (int i = 0; i < NUM_OF_DIGITS; i++) {
        segment_data[i] = character_map[CHAR_SPACE];
    }


    display_init();

    display_update();
}


void s7_insert_char_rh(uint8_t c) {
    insert_seg_data_rh(lookup_character(c));
}


void s7_print_decimal_number(uint32_t number) {

    uint32_t base10number = number;

    /* This code will display the least significant
     * 8 decimal digits of the 32-bit argument. */
    for (int i = 0; i < NUM_OF_DIGITS; i++) {

        uint8_t digit = base10number % 10;

        /* Check if we should blank a leading zero */
        if (digit == 0 && base10number == 0) {
            segment_data[i] = 0;
        } else {
            segment_data[i] = character_map[digit];
        }

        base10number /= 10;
    }

    /* Handle special case: Just 0 _should_ be displayed */
    /* Comment OUT to reserve 0 as a "display blanking" option */
  //if (number == 0)
  //{
  //    segment_data[0] = character_map[0];
  //}

    display_update();
}


void s7_print_hex_number(uint32_t number) {

    // This code will display as much as it can
    // of the 32-bit argument in Hex
    for (int i = 0; i < NUM_OF_DIGITS; i++) {
        segment_data[i] = character_map[number % 16];
        number /= 16;
    }

    display_update();
}


void s7_print_text_string(const uint8_t *p) {

    uint8_t c;

    /* clear display */
    s7_print_decimal_number(0);

    while((c = *p++) != 0) {

        s7_insert_char_rh(c);

        if (*p == (uint8_t)'.') {
            segment_data[0] |= SEG_DP;
            display_update();
            p++;
        }
    }
}


void s7_print_decimal_points(uint8_t dps) {

    for (int i = 0; i < NUM_OF_DIGITS; i++, dps >>= 1) {

        if (dps & 1) {
            segment_data[i] |= SEG_DP;
        } else {
            segment_data[i] &= ~SEG_DP;
        }
    }

    display_update();
}
