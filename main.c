// WHY2025 coprocessor firmware
// Copyright Nicolai Electronics 2024

#include "ch32v003fun.h"
#include "i2c_slave.h"
#include <stdio.h>
#include <stdbool.h>

// Board revision
#define HW_REV 1

// Firmware version
#define FW_VERSION 1

// Pins
const uint8_t keyboard_rows[] = {PA8, PA9, PA10, PA4, PA3, PA1, PA6, PA5, PA2};
const uint8_t keyboard_columns[] = {PB14, PB12, PB1, PA7, PB15, PB13, PB2, PB0};
const uint8_t pin_c6_enable = PB8;
const uint8_t pin_c6_boot = PB9;
const uint8_t pin_display_backlight = PB4; // Note: change PWM timer configuration too when changing this pin
const uint8_t pin_keyboard_backlight = PB3; // Note: change PWM timer configuration too when changing this pin
const uint8_t pin_interrupt = PA0;
const uint8_t pin_sdcard_detect = PA15;
const uint8_t pin_headphone_detect = PB5;
const uint8_t pin_amplifier_enable = PD0;
//const uint8_t pin_power_on = PC13;

// Configuration
const uint16_t timer2_pwm_cycle_width = 255; // Amount of brightness steps for keyboard backlight
const uint16_t timer3_pwm_cycle_width = 255; // Amount of brightness steps for display backlight

const uint32_t keyboard_scan_interval = 1; // milliseconds (per row)
const uint32_t input_scan_interval = 50; // milliseconds

// I2C registers

typedef enum {
    I2C_REG_FW_VERSION_0 = 0, // LSB
    I2C_REG_FW_VERSION_1,  // MSB
    I2C_REG_KEYBOARD_0,
    I2C_REG_KEYBOARD_1,
    I2C_REG_KEYBOARD_2,
    I2C_REG_KEYBOARD_3,
    I2C_REG_KEYBOARD_4,
    I2C_REG_KEYBOARD_5,
    I2C_REG_KEYBOARD_6,
    I2C_REG_KEYBOARD_7,
    I2C_REG_KEYBOARD_8,
    I2C_REG_DISPLAY_BACKLIGHT_0, // LSB
    I2C_REG_DISPLAY_BACKLIGHT_1, // MSB
    I2C_REG_KEYBOARD_BACKLIGHT_0, // LSB
    I2C_REG_KEYBOARD_BACKLIGHT_1, // MSB
    I2C_REG_INPUT, // SD card detect (bit 0) & headphone detect (bit 1)
    I2C_REG_AMPLIFIER_ENABLE,
    I2C_REG_RADIO_CONTROL,
    I2C_REG_LAST, // End of list marker
} i2c_register_t;

volatile uint8_t i2c_registers[I2C_REG_LAST];

// Interrupt flags
volatile bool keyboard_interrupt = false;
volatile bool input_interrupt = false;

// Keyboard matrix
bool keyboard_step() {
    static uint8_t row = 0;
    static uint8_t previous_values[sizeof(keyboard_columns)] = {0};

    bool changed = false;

    uint8_t value = 0;
    for (uint8_t column = 0; column < sizeof(keyboard_columns); column++) {
        value |= funDigitalRead(keyboard_columns[column]) << column;
    }
    i2c_registers[I2C_REG_KEYBOARD_0 + row] = value;
    changed = previous_values[row] != value;
    previous_values[row] = value;

    // Next row
    funDigitalWrite(keyboard_rows[row], FUN_LOW);
    row++;
    if (row >= sizeof(keyboard_rows)) {
        row = 0;
    }
    funDigitalWrite(keyboard_rows[row], FUN_HIGH);

    return changed;
}

// Inputs
bool input_step() {
    static uint8_t previous_value = 0;

    uint8_t value = 0;
    value |= funDigitalRead(pin_sdcard_detect)    << 0;
    value |= funDigitalRead(pin_headphone_detect) << 1;

    i2c_registers[I2C_REG_INPUT] = value;

    bool changed = previous_value != value;
    previous_value = value;

    return changed;
}

// Timers for PWM output

void timer2_set(uint16_t value) {
    if (value > timer2_pwm_cycle_width) {
        value = timer2_pwm_cycle_width;
    }
    TIM2->CH2CVR = value;
    TIM2->SWEVGR |= TIM_UG; // Apply
}

void timer2_init() {
        RCC->APB1PCENR |= RCC_APB1Periph_TIM2; // Enable clock for timer 2
        AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1; // Partial mapping (PB3 as channel 2)
        funPinMode(pin_keyboard_backlight, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

        // Reset timer 3 peripheral
        RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
        RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

        TIM2->PSC = 0x0000; // Clock prescaler divider
        TIM2->ATRLR = timer2_pwm_cycle_width; // Total PWM cycle width

        TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE; // Enable channel 2

        TIM2->CTLR1 |= TIM_ARPE; // Enable auto-reload of preload

        TIM2->CCER |= TIM_CC1E | TIM_CC1P; // Enable channel 1 output, positive polarity

        timer2_set(0); // Load default target PWM dutycycle

        TIM2->CTLR1 |= TIM_CEN; // Enable timer
}

void timer3_set(uint16_t value) {
    if (value > timer3_pwm_cycle_width) {
        value = timer3_pwm_cycle_width;
    }
    TIM3->CH1CVR = value;
    TIM3->SWEVGR |= TIM_UG; // Apply
}

void timer3_init() {
        RCC->APB1PCENR |= RCC_APB1Periph_TIM3; // Enable clock for timer 3
        AFIO->PCFR1 |= AFIO_PCFR1_TIM3_REMAP_PARTIALREMAP; // Partial mapping (PB4 as channel 1)
        funPinMode(pin_display_backlight, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

        // Reset timer 3 peripheral
        RCC->APB1PRSTR |= RCC_APB1Periph_TIM3;
        RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM3;

        TIM3->PSC = 0x0000; // Clock prescaler divider
        TIM3->ATRLR = timer3_pwm_cycle_width; // Total PWM cycle width

        TIM3->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE; // Enable channel 1

        TIM3->CTLR1 |= TIM_ARPE; // Enable auto-reload of preload

        TIM3->CCER |= TIM_CC1E | TIM_CC1P; // Enable channel 1 output, positive polarity

        timer3_set(0); // Load default target PWM dutycycle

        TIM3->CTLR1 |= TIM_CEN; // Enable timer
}

// I2C write callback
void i2c_write_cb(uint8_t reg, uint8_t length) {
    while (length > 0) {
        switch(reg) {
            case I2C_REG_DISPLAY_BACKLIGHT_0:
                if (length > 1) break; // Fall through when only LSB register gets written
            case I2C_REG_DISPLAY_BACKLIGHT_1:
                timer3_set(i2c_registers[I2C_REG_DISPLAY_BACKLIGHT_0] + (i2c_registers[I2C_REG_DISPLAY_BACKLIGHT_1] << 8));
            case I2C_REG_KEYBOARD_BACKLIGHT_0:
                if (length > 1) break; // Fall through when only LSB register gets written
            case I2C_REG_KEYBOARD_BACKLIGHT_1:
                timer2_set(i2c_registers[I2C_REG_KEYBOARD_BACKLIGHT_0] + (i2c_registers[I2C_REG_KEYBOARD_BACKLIGHT_1] << 8));
                break;
            case I2C_REG_AMPLIFIER_ENABLE:
                funDigitalWrite(pin_amplifier_enable, i2c_registers[I2C_REG_AMPLIFIER_ENABLE] & 1);
                break;
            case I2C_REG_RADIO_CONTROL:
                funDigitalWrite(pin_c6_enable, (i2c_registers[I2C_REG_RADIO_CONTROL] >> 0) & 1);
                funDigitalWrite(pin_c6_boot, (i2c_registers[I2C_REG_RADIO_CONTROL] >> 1) & 1);
                break;
            default:
                break;
        }
        // Next register
        reg++;
        length--;
    }
}

// I2C read callback
void i2c_read_cb(uint8_t reg) {
    switch(reg) {
        case I2C_REG_KEYBOARD_0:
        case I2C_REG_KEYBOARD_1:
        case I2C_REG_KEYBOARD_2:
        case I2C_REG_KEYBOARD_3:
        case I2C_REG_KEYBOARD_4:
        case I2C_REG_KEYBOARD_5:
        case I2C_REG_KEYBOARD_6:
        case I2C_REG_KEYBOARD_7:
        case I2C_REG_KEYBOARD_8:
            keyboard_interrupt = false; // Clear keyboard interrupt flag
            break;
        case I2C_REG_INPUT:
            input_interrupt = false; // Clear input interrupt flag
            break;
        default:
            break;
    }
}

// Entry point
int main() {
    SystemInit();
    funGpioInitAll();

    // Keyboard pins
    for (uint8_t row = 0; row < sizeof(keyboard_rows); row++) {
        funPinMode(keyboard_rows[row], GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
        funDigitalWrite(keyboard_rows[row], FUN_LOW);
    }

    for (uint8_t column = 0; column < sizeof(keyboard_columns); column++) {
        funPinMode(keyboard_columns[column], GPIO_Speed_In | GPIO_CNF_IN_FLOATING);
    }

    // Initialize I2C slave
    funPinMode(PB7, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
    funPinMode(PB6, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
    SetupI2CSlave(0x5f, i2c_registers, sizeof(i2c_registers), i2c_write_cb, i2c_read_cb, false);

    // ESP32-C6
    funPinMode(pin_c6_enable, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_c6_enable, FUN_LOW);
    funPinMode(pin_c6_boot, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_c6_boot, FUN_LOW);

    // Display backlight
    timer3_init(); // Use timer 3 channel 1 as PWM output for controlling display backlight
    timer3_set(255); // Turn on display backlight at full brightness

    // Keyboard backlight
    timer2_init(); // Use timer 2 channel 2 as PWM output for controlling keyboard backlight

    // Interrupt
    funPinMode(pin_interrupt, GPIO_Speed_10MHz | GPIO_CNF_OUT_OD);
    funDigitalWrite(pin_interrupt, FUN_HIGH);

    // SD card detect
    funPinMode(pin_sdcard_detect, GPIO_Speed_In | GPIO_CNF_IN_FLOATING);

    // Headphone detect
    funPinMode(pin_headphone_detect, GPIO_Speed_In | GPIO_CNF_IN_FLOATING);

    // Amplifier enable
    funPinMode(pin_amplifier_enable, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_amplifier_enable, FUN_LOW);

    while (1) {
        i2c_registers[I2C_REG_FW_VERSION_0] = (FW_VERSION     ) & 0xFF;
        i2c_registers[I2C_REG_FW_VERSION_1] = (FW_VERSION >> 8) & 0xFF;

        uint32_t now = SysTick->CNT;

        static uint32_t keyboard_scan_previous = 0;
        if (now - keyboard_scan_previous >= keyboard_scan_interval * DELAY_MS_TIME) {
            keyboard_scan_previous = now;
            keyboard_interrupt |= keyboard_step(); // Scans one row when called
        }

        static uint32_t input_scan_previous = 0;
        if (now - input_scan_previous >= input_scan_interval * DELAY_MS_TIME) {
            input_scan_previous = now;
            input_interrupt |= input_step(); // Scans all inputs
        }

        funDigitalWrite(pin_interrupt, (keyboard_interrupt | input_interrupt) ? FUN_HIGH : FUN_LOW); // Update interrupt pin state
    }
}
