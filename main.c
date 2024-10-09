// Tanmatsu coprocessor firmware
// Copyright Nicolai Electronics 2024

#include "ch32v003fun.h"
#include "i2c_slave.h"
#include "i2c_master.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Board revision
#define HW_REV 1

// Firmware version
#define FW_VERSION 3

#define OVERRIDE_C6 false
#define HARDWARE_REV 2 // 1 for prototype 1, 2 for prototype 2

// Pins
const uint8_t keyboard_rows[] = {PA8, PA9, PA10, PA4, PA3, PA1, PA6, PA5, PA2};
const uint8_t keyboard_columns[] = {PB14, PB12, PB1, PA7, PB15, PB13, PB2, PB0};
const uint8_t pin_c6_enable = PB8;
const uint8_t pin_c6_boot = (HARDWARE_REV > 1 ? PD1 : PB9);
const uint8_t pin_display_backlight = PB4; // Note: change PWM timer configuration too when changing this pin
const uint8_t pin_keyboard_backlight = PB3; // Note: change PWM timer configuration too when changing this pin
const uint8_t pin_interrupt = PA0;
const uint8_t pin_sdcard_detect = PA15;
const uint8_t pin_headphone_detect = PB5;
const uint8_t pin_amplifier_enable = PD0;
//const uint8_t pin_power_out = PC13; // Output to power button (for powering on using the RTC alarm)
const uint8_t pin_sda = PB7; // Uses hardware I2C peripheral
const uint8_t pin_scl = PB6; // Uses hardware I2C peripheral

#if HARDWARE_REV > 1
const uint8_t pin_camera = PA11;
const uint8_t pin_power_in = PA12; // Input from power button
const uint8_t pin_pm_sda = PB11;
const uint8_t pin_pm_scl = PB10;
#endif

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
    I2C_REG_OUTPUT,
    I2C_REG_RADIO_CONTROL,
    I2C_REG_RTC_VALUE_0, // LSB
    I2C_REG_RTC_VALUE_1,
    I2C_REG_RTC_VALUE_2,
    I2C_REG_RTC_VALUE_3,
    I2C_REG_BACKUP_0,
    I2C_REG_BACKUP_1,
    I2C_REG_BACKUP_2,
    I2C_REG_BACKUP_3,
    I2C_REG_BACKUP_4,
    I2C_REG_BACKUP_5,
    I2C_REG_BACKUP_6,
    I2C_REG_BACKUP_7,
    I2C_REG_BACKUP_8,
    I2C_REG_BACKUP_9,
    I2C_REG_BACKUP_10,
    I2C_REG_BACKUP_11,
    I2C_REG_BACKUP_12,
    I2C_REG_BACKUP_13,
    I2C_REG_BACKUP_14,
    I2C_REG_BACKUP_15,
    I2C_REG_BACKUP_16,
    I2C_REG_BACKUP_17,
    I2C_REG_BACKUP_18,
    I2C_REG_BACKUP_19,
    I2C_REG_BACKUP_20,
    I2C_REG_BACKUP_21,
    I2C_REG_BACKUP_22,
    I2C_REG_BACKUP_23,
    I2C_REG_BACKUP_24,
    I2C_REG_BACKUP_25,
    I2C_REG_BACKUP_26,
    I2C_REG_BACKUP_27,
    I2C_REG_BACKUP_28,
    I2C_REG_BACKUP_29,
    I2C_REG_BACKUP_30,
    I2C_REG_BACKUP_31,
    I2C_REG_BACKUP_32,
    I2C_REG_BACKUP_33,
    I2C_REG_BACKUP_34,
    I2C_REG_BACKUP_35,
    I2C_REG_BACKUP_36,
    I2C_REG_BACKUP_37,
    I2C_REG_BACKUP_38,
    I2C_REG_BACKUP_39,
    I2C_REG_BACKUP_40,
    I2C_REG_BACKUP_41,
    I2C_REG_BACKUP_42,
    I2C_REG_BACKUP_43,
    I2C_REG_BACKUP_44,
    I2C_REG_BACKUP_45,
    I2C_REG_BACKUP_46,
    I2C_REG_BACKUP_47,
    I2C_REG_BACKUP_48,
    I2C_REG_BACKUP_49,
    I2C_REG_BACKUP_50,
    I2C_REG_BACKUP_51,
    I2C_REG_BACKUP_52,
    I2C_REG_BACKUP_53,
    I2C_REG_BACKUP_54,
    I2C_REG_BACKUP_55,
    I2C_REG_BACKUP_56,
    I2C_REG_BACKUP_57,
    I2C_REG_BACKUP_58,
    I2C_REG_BACKUP_59,
    I2C_REG_BACKUP_60,
    I2C_REG_BACKUP_61,
    I2C_REG_BACKUP_62,
    I2C_REG_BACKUP_63,
    I2C_REG_BACKUP_64,
    I2C_REG_BACKUP_65,
    I2C_REG_BACKUP_66,
    I2C_REG_BACKUP_67,
    I2C_REG_BACKUP_68,
    I2C_REG_BACKUP_69,
    I2C_REG_BACKUP_70,
    I2C_REG_BACKUP_71,
    I2C_REG_BACKUP_72,
    I2C_REG_BACKUP_73,
    I2C_REG_BACKUP_74,
    I2C_REG_BACKUP_75,
    I2C_REG_BACKUP_76,
    I2C_REG_BACKUP_77,
    I2C_REG_BACKUP_78,
    I2C_REG_BACKUP_79,
    I2C_REG_BACKUP_80,
    I2C_REG_BACKUP_81,
    I2C_REG_BACKUP_82,
    I2C_REG_BACKUP_83,
    I2C_REG_PMIC_BATTERY_CONTROL,
    I2C_REG_PMIC_BATTERY_STATUS,
    I2C_REG_LAST, // End of list marker
} i2c_register_t;

volatile uint8_t i2c_registers[I2C_REG_LAST];

// Interrupt flags
volatile bool keyboard_interrupt = false;
volatile bool input_interrupt = false;

// Keyboard matrix
bool keyboard_step() {
    static uint8_t row = 0;
    static uint8_t previous_values[sizeof(keyboard_rows)] = {0};

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
    value |= (!funDigitalRead(pin_sdcard_detect)) << 0;
    value |= funDigitalRead(pin_headphone_detect) << 1;

#if HARDWARE_REV > 1
    value |= funDigitalRead(pin_power_in) << 2;
#endif

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
    TIM2->CH2CVR = timer2_pwm_cycle_width - value;
    TIM2->SWEVGR |= TIM_UG; // Apply
}

void timer2_init() {
        RCC->APB1PCENR |= RCC_APB1Periph_TIM2; // Enable clock for timer 2
        AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_PARTIALREMAP1; // Partial mapping (PB3 as channel 2)
        funPinMode(pin_keyboard_backlight, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

        // Reset timer 3 peripheral
        RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
        RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

        TIM2->PSC = 0x2000; // Clock prescaler divider
        TIM2->ATRLR = timer2_pwm_cycle_width; // Total PWM cycle width

        TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE; // Enable channel 2

        TIM2->CTLR1 |= TIM_ARPE; // Enable auto-reload of preload

        TIM2->CCER |= TIM_CC2E | TIM_CC2P; // Enable channel 2 output, positive polarity

        timer2_set(0); // Load default target PWM dutycycle

        TIM2->CTLR1 |= TIM_CEN; // Enable timer
}

void timer3_set(uint16_t value) {
    if (value > timer3_pwm_cycle_width) {
        value = timer3_pwm_cycle_width;
    }
    TIM3->CH1CVR = timer3_pwm_cycle_width - value;
    TIM3->SWEVGR |= TIM_UG; // Apply
}

void timer3_init() {
        RCC->APB1PCENR |= RCC_APB1Periph_TIM3; // Enable clock for timer 3
        AFIO->PCFR1 |= AFIO_PCFR1_TIM3_REMAP_PARTIALREMAP; // Partial mapping (PB4 as channel 1)
        funPinMode(pin_display_backlight, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF);

        // Reset timer 3 peripheral
        RCC->APB1PRSTR |= RCC_APB1Periph_TIM3;
        RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM3;

        TIM3->PSC = 0x2000; // Clock prescaler divider
        TIM3->ATRLR = timer3_pwm_cycle_width; // Total PWM cycle width

        TIM3->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE; // Enable channel 1

        TIM3->CTLR1 |= TIM_ARPE; // Enable auto-reload of preload

        TIM3->CCER |= TIM_CC1E | TIM_CC1P; // Enable channel 1 output, positive polarity

        timer3_set(255); // Load default target PWM dutycycle

        TIM3->CTLR1 |= TIM_CEN; // Enable timer
}

// RTC timer

#define PWR_CTLR_R2KSTY   ((uint32_t)0x00010000) /* 2K/20K enable flag (standby) */
#define PWR_CTLR_R30KSTY  ((uint32_t)0x00020000) /* 30K RAM enable flag (standby) */
#define PWR_CTLR_R2KVBAT  ((uint32_t)0x00040000) /* 2K/20K RAM enable flag (vbat) */
#define PWR_CTLR_R30KVBAT ((uint32_t)0x00080000) /* 30K RAM enable flag (vbat) */
#define PWR_CTLR_RAMLV    ((uint32_t)0x00100000) /* RAM low voltage mode enable */

void rtc_disable_wp() {
    PWR->CTLR |= PWR_CTLR_DBP;
}

void rtc_enable_wp() {
    PWR->CTLR &= ~PWR_CTLR_DBP;
}

void rtc_enter_config() {
    RTC->CTLRL |= RTC_CTLRL_CNF;
}

void rtc_exit_config() {
    RTC->CTLRL &= (uint16_t) ~((uint16_t)RTC_CTLRL_CNF);
}

void rtc_wait_for_last_task(void) {
    while((RTC->CTLRL & RTC_FLAG_RTOFF) == 0) {}
}

void rtc_wait_for_sync() {
    RTC->CTLRL &= (uint16_t)~RTC_FLAG_RSF;
    while((RTC->CTLRL & RTC_FLAG_RSF) == 0) {}
}

void rtc_set_prescaler(uint32_t value) {
    rtc_enter_config();
    RTC->PSCRH = (value >> 16) & 0xF;
    RTC->PSCRL = value & 0xFFFF;
    rtc_exit_config();
}

uint32_t rtc_get_prescaler() {
    return ((RTC->PSCRH & 0xF) << 16) | RTC->PSCRL;
}

void rtc_set_counter(uint32_t value) {
    rtc_disable_wp();
    rtc_enter_config();
    rtc_wait_for_last_task();
    RTC->CNTH = value >> 16;
    RTC->CNTL = value & 0xFFFF;
    rtc_wait_for_last_task();
    rtc_exit_config();
    rtc_wait_for_last_task();
    rtc_enable_wp();
}

uint32_t rtc_get_counter() {
    uint16_t high1 = RTC->CNTH;
    uint16_t high2 = RTC->CNTH;
    uint16_t low = RTC->CNTL;
    if(high1 != high2) {
        return (((uint32_t)high2 << 16) | RTC->CNTL);
    } else {
        return (((uint32_t)high1 << 16) | low);
    }
}

uint32_t rtc_read_divider() {
    uint32_t tmp = 0x00;
    tmp = ((uint32_t)RTC->DIVH & (uint32_t)0x000F) << 16;
    tmp |= RTC->DIVL;
    return tmp;
}

void rtc_init() {
    RCC->APB1PCENR |= RCC_APB1Periph_PWR | RCC_APB1Periph_BKP;

    bool rtc_not_ready = false;
    rtc_not_ready |= !(RCC->BDCTLR | RCC_LSEON); // If LSE oscillator is not enabled
    rtc_not_ready |= !(RCC->BDCTLR & RCC_LSERDY); // If LSE oscillator is not running
    rtc_not_ready |= !(RCC->BDCTLR | RCC_RTCSEL_0); // If LSE is not selected as clock source
    rtc_not_ready |= !(RCC->BDCTLR | RCC_RTCEN); // If RTC is not enabled
    rtc_not_ready |= !(rtc_get_prescaler() == 32768 / 2); // If RTC is not set to tick once per second

    if (rtc_not_ready) {
        rtc_disable_wp(); // Disable backup domain write protection
        RCC->BDCTLR |= RCC_LSEON; // Enable LSE

        // Wait for LSE oscillator ready
        while (!(RCC->BDCTLR & RCC_LSERDY)) {}

        RCC->BDCTLR |= RCC_RTCSEL_0; // Use LSE oscillator as RTC clock source
        RCC->BDCTLR |= RCC_RTCEN; // Enable RTC

        rtc_wait_for_last_task();
        rtc_wait_for_last_task();
        rtc_set_prescaler(32768 / 2); // 1 tick per second
        rtc_wait_for_last_task();
        rtc_set_counter(0);
        rtc_wait_for_last_task();
        rtc_enable_wp(); // Enable backup domain write protection
    }
}

// Backup registers
uint16_t bkp_read(uint8_t position) {
    switch(position) {
        case 0:  return BKP->DATAR1;
        case 1:  return BKP->DATAR2;
        case 2:  return BKP->DATAR3;
        case 3:  return BKP->DATAR4;
        case 4:  return BKP->DATAR5;
        case 5:  return BKP->DATAR6;
        case 6:  return BKP->DATAR7;
        case 7:  return BKP->DATAR8;
        case 8:  return BKP->DATAR9;
        case 9:  return BKP->DATAR10;
        case 10:  return BKP->DATAR11;
        case 11:  return BKP->DATAR12;
        case 12:  return BKP->DATAR13;
        case 13:  return BKP->DATAR14;
        case 14:  return BKP->DATAR15;
        case 15:  return BKP->DATAR16;
        case 16:  return BKP->DATAR17;
        case 17:  return BKP->DATAR18;
        case 18:  return BKP->DATAR19;
        case 19:  return BKP->DATAR20;
        case 20:  return BKP->DATAR21;
        case 21:  return BKP->DATAR22;
        case 22:  return BKP->DATAR23;
        case 23:  return BKP->DATAR24;
        case 24:  return BKP->DATAR25;
        case 25:  return BKP->DATAR26;
        case 26:  return BKP->DATAR27;
        case 27:  return BKP->DATAR28;
        case 28:  return BKP->DATAR29;
        case 29:  return BKP->DATAR30;
        case 30:  return BKP->DATAR31;
        case 31:  return BKP->DATAR32;
        case 32:  return BKP->DATAR33;
        case 33:  return BKP->DATAR34;
        case 34:  return BKP->DATAR35;
        case 35:  return BKP->DATAR36;
        case 36:  return BKP->DATAR37;
        case 37:  return BKP->DATAR38;
        case 38:  return BKP->DATAR39;
        case 39:  return BKP->DATAR40;
        case 40:  return BKP->DATAR41;
        case 41:  return BKP->DATAR42;
        default: return 0;
    }
}

void bkp_read_all() {
    for (uint8_t index = 0; index < 42; index++) {
        uint16_t value = bkp_read(index);
        i2c_registers[I2C_REG_BACKUP_0 + index * 2 + 0] = (value >> 0) & 0xFF;
        i2c_registers[I2C_REG_BACKUP_0 + index * 2 + 1] = (value >> 8) & 0xFF;
    }
}

void bkp_write(uint8_t position, uint16_t value) {
    rtc_disable_wp();
    switch(position) {
        case 0:  BKP->DATAR1 = value;
        case 1:  BKP->DATAR2 = value;
        case 2:  BKP->DATAR3 = value;
        case 3:  BKP->DATAR4 = value;
        case 4:  BKP->DATAR5 = value;
        case 5:  BKP->DATAR6 = value;
        case 6:  BKP->DATAR7 = value;
        case 7:  BKP->DATAR8 = value;
        case 8:  BKP->DATAR9 = value;
        case 9:  BKP->DATAR10 = value;
        case 10:  BKP->DATAR11 = value;
        case 11:  BKP->DATAR12 = value;
        case 12:  BKP->DATAR13 = value;
        case 13:  BKP->DATAR14 = value;
        case 14:  BKP->DATAR15 = value;
        case 15:  BKP->DATAR16 = value;
        case 16:  BKP->DATAR17 = value;
        case 17:  BKP->DATAR18 = value;
        case 18:  BKP->DATAR19 = value;
        case 19:  BKP->DATAR20 = value;
        case 20:  BKP->DATAR21 = value;
        case 21:  BKP->DATAR22 = value;
        case 22:  BKP->DATAR23 = value;
        case 23:  BKP->DATAR24 = value;
        case 24:  BKP->DATAR25 = value;
        case 25:  BKP->DATAR26 = value;
        case 26:  BKP->DATAR27 = value;
        case 27:  BKP->DATAR28 = value;
        case 28:  BKP->DATAR29 = value;
        case 29:  BKP->DATAR30 = value;
        case 30:  BKP->DATAR31 = value;
        case 31:  BKP->DATAR32 = value;
        case 32:  BKP->DATAR33 = value;
        case 33:  BKP->DATAR34 = value;
        case 34:  BKP->DATAR35 = value;
        case 35:  BKP->DATAR36 = value;
        case 36:  BKP->DATAR37 = value;
        case 37:  BKP->DATAR38 = value;
        case 38:  BKP->DATAR39 = value;
        case 39:  BKP->DATAR40 = value;
        case 40:  BKP->DATAR41 = value;
        case 41:  BKP->DATAR42 = value;
        default: break;
    }
    rtc_enable_wp();
}

void bkp_write_byte(uint8_t position, uint8_t value) {
    uint16_t register_value = bkp_read(position >> 1);
    if (position & 1) {
        register_value &= 0x00FF;
        register_value |= value << 8;
    } else {
        register_value &= 0xFF00;
        register_value |= value;
    }
    bkp_write(position >> 1, register_value);
}

// I2C write callback
void i2c_write_cb(uint8_t reg, uint8_t length) {
    static uint32_t new_rtc_value = 0;

    while (length > 0) {
        switch(reg) {
            case I2C_REG_DISPLAY_BACKLIGHT_0:
                if (length > 1) break; // Fall through when only LSB register gets written
            case I2C_REG_DISPLAY_BACKLIGHT_1: {
                timer3_set(i2c_registers[I2C_REG_DISPLAY_BACKLIGHT_0] + (i2c_registers[I2C_REG_DISPLAY_BACKLIGHT_1] << 8));
                break;
            }
            case I2C_REG_KEYBOARD_BACKLIGHT_0:
                if (length > 1) break; // Fall through when only LSB register gets written
            case I2C_REG_KEYBOARD_BACKLIGHT_1:
                timer2_set(i2c_registers[I2C_REG_KEYBOARD_BACKLIGHT_0] + (i2c_registers[I2C_REG_KEYBOARD_BACKLIGHT_1] << 8));
                break;
            case I2C_REG_OUTPUT:
                funDigitalWrite(pin_amplifier_enable, i2c_registers[I2C_REG_OUTPUT] & 1);
#if HARDWARE_REV > 1
                funDigitalWrite(pin_camera, i2c_registers[I2C_REG_OUTPUT] & 2);
#endif
                break;
            case I2C_REG_RADIO_CONTROL:
#if OVERRIDE_C6 == false // Ignore radio control register if radio override is active
                funDigitalWrite(pin_c6_enable, ((i2c_registers[I2C_REG_RADIO_CONTROL] >> 0) & 1) ? FUN_HIGH : FUN_LOW);
                funDigitalWrite(pin_c6_boot, ((i2c_registers[I2C_REG_RADIO_CONTROL] >> 1) & 1) ? FUN_HIGH : FUN_LOW);
#endif
                break;
            case I2C_REG_RTC_VALUE_0:
                new_rtc_value &= 0xFFFFFF00;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_0];
                break;
            case I2C_REG_RTC_VALUE_1:
                new_rtc_value &= 0xFFFF00FF;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_1] << 8;
                break;
            case I2C_REG_RTC_VALUE_2:
                new_rtc_value &= 0xFF00FFFF;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_2] << 16;
                break;
            case I2C_REG_RTC_VALUE_3:
                new_rtc_value &= 0x00FFFFFF;
                new_rtc_value |= i2c_registers[I2C_REG_RTC_VALUE_3] << 24;
                rtc_set_counter(new_rtc_value);
                break;
            default:
                if (reg >= I2C_REG_BACKUP_0 && reg <= I2C_REG_BACKUP_83) {
                    bkp_write_byte(reg - I2C_REG_BACKUP_0, i2c_registers[reg]);
                }
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

// ---- PMIC ----

// REG00
void pmic_set_input_current_limit(uint16_t current, bool enable_ilim_pin) {
    const uint8_t reg = 0x00;
    uint8_t value = 0x00;

    if (current < 100) { // Minimum current is 100 mA
        current = 100;
    }
    current -= 100; // Offset is 100 mA

    if (enable_ilim_pin) {
        value |= 0b1000000; // Enable ILIM pin
    }
    if (current >= 1600) {
        current -= 1600;
        value |= 0b100000; // Add 1600 mA
    }
    if (current >= 800) {
        current -= 800;
        value |= 0b10000; // Add 800 mA
    }
    if (current >= 400) {
        current -= 400;
        value |= 0b1000; // Add 400 mA
    }
    if (current >= 200) {
        current -= 200;
        value |= 0b100; // Add 200 mA
    }
    if (current >= 100) {
        current -= 100;
        value |= 0b10; // Add 100 mA
    }
    if (current >= 50) {
        current -= 50;
        value |= 0b1; // Add 50 mA
    }

    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG02
void pmic_adc_control(bool enable, bool continuous) {
    const uint8_t reg = 0x02;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    if (enable) {
        value |= (1 << 7);
    } else {
        value &= ~(1 << 7);
    }
    if (continuous) {
        value |= (1 << 6);
    } else {
        value &= ~(1 << 6);
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

void pmic_ico(bool enable) {
    const uint8_t reg = 0x02;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    if (enable) {
        value |= (1 << 4);
    } else {
        value &= ~(1 << 4);
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG03
void pmic_otg_config(bool enable) {
    const uint8_t reg = 0x03;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    if (enable) {
        value |= (1 << 5);
    } else {
        value &= ~(1 << 5);
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

void pmic_chg_config(bool enable) {
    const uint8_t reg = 0x03;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    if (enable) {
        value |= (1 << 4);
    } else {
        value &= ~(1 << 4);
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

void pmic_batt_load_config(bool enable) {
    const uint8_t reg = 0x03;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    if (enable) {
        value |= (1 << 7);
    } else {
        value &= ~(1 << 7);
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

void pmic_set_minimum_system_voltage_limit(uint16_t voltage) {
    // Voltage in mV
    const uint8_t reg = 0x03;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);

    value &= 0b11110001; // Mask

    value |= 0b10000000; // Enable battery load

    if (voltage < 3000) { // Minimum voltage is 3V
        voltage = 3000;
    }
    voltage -= 3000; // Offset is 3V

    if (voltage >= 400) {
        voltage -= 400;
        value |= (0b100) << 1; // Add 0.4V
    }
    if (voltage >= 200) {
        voltage -= 200;
        value |= (0b010) << 1; // Add 0.2V
    }
    if (voltage >= 100) {
        voltage -= 100;
        value |= (0b001) << 1; // Add 0.1V
    }

    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG04

void pmic_set_fast_charge_current(uint16_t current, bool en_pumpx) {
    const uint8_t reg = 0x04;
    uint8_t value = 0;

    if (en_pumpx) {
        value |= (1 << 7);
    }

    if (current >= 4096) {
        current -= 4096;
        value |= (1 << 6); // Add 4096mA
    }
    if (current >= 2048) {
        current -= 2048;
        value |= (1 << 5); // Add 2048mA
    }
    if (current >= 1024) {
        current -= 1024;
        value |= (1 << 4); // Add 1024mA
    }
    if (current >= 512) {
        current -= 512;
        value |= (1 << 3); // Add 512mA
    }
    if (current >= 256) {
        current -= 256;
        value |= (1 << 2); // Add 256mA
    }
    if (current >= 128) {
        current -= 128;
        value |= (1 << 1); // Add 128mA
    }
    if (current >= 64) {
        current -= 64;
        value |= (1 << 0); // Add 64mA
    }

    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG06
void pmic_battery_threshold(uint16_t voltage_limit, bool batlowv, bool vrechg) {
    const uint8_t reg = 0x06;
    uint8_t value = 0;

    if (voltage_limit < 3840) {
        voltage_limit = 3840;
    }
    voltage_limit -= 3840; // Offset
    if (voltage_limit > 512) {
        voltage_limit -= 512;
        value |= (1 << 7); // Add 512mA
    }
    if (voltage_limit > 256) {
        voltage_limit -= 256;
        value |= (1 << 6); // Add 256mA
    }
    if (voltage_limit > 128) {
        voltage_limit -= 128;
        value |= (1 << 5); // Add 128mA
    }
    if (voltage_limit > 64) {
        voltage_limit -= 64;
        value |= (1 << 4); // Add 64mA
    }
    if (voltage_limit > 32) {
        voltage_limit -= 32;
        value |= (1 << 3); // Add 32mA
    }
    if (voltage_limit > 16) {
        voltage_limit -= 16;
        value |= (1 << 2); // Add 16mA
    }
    if (batlowv) {
        value |= (1 << 1); // Battery precharge to fast charge threshold: 1 is 3.0v (default), 0 is 2.8v
    }
    if (vrechg) {
        value |= (1 << 0); // Battery recharge threshold offset: 1 is 200mV below VREG, 0 is 100mV below VREG (default)
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG07
void pmic_watchdog(uint8_t watchdog_setting) {
    const uint8_t reg = 0x07;
    uint8_t value = 0x00;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    value &= ~(0b00110000);
    value |= (watchdog_setting & 3) << 4; // Watchdog
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG09
void pmic_power_on() {
    pmic_watchdog(0);
    const uint8_t reg = 0x09;
    uint8_t value = 0x00;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    value &= (~1 << 5); // Clear BATFET_DIS bit
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

void pmic_power_off() {
    pmic_watchdog(0);
    const uint8_t reg = 0x09;
    uint8_t value = 0x00;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    value |= (1 << 5); // Set BATFET_DIS bit
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

typedef struct pmic_faults {
    bool watchdog;
    bool boost;
    bool chrg_input;
    bool chrg_thermal;
    bool chrg_safety;
    bool batt_ovp;
    bool ntc_cold;
    bool ntc_hot;
    bool ntc_boost;
} pmic_faults_t;

// REG0C
void pmic_read_faults(pmic_faults_t* faults) {
    const uint8_t reg = 0x0C;
    uint8_t value = 0;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    faults->watchdog = (value >> 7) & 1;
    faults->boost = (value >> 6) & 1;
    faults->chrg_input = ((value >> 4) & 3) == 0b01;
    faults->chrg_thermal = ((value >> 4) & 3) == 0b10;
    faults->chrg_safety = ((value >> 4) & 3) == 0b11;
    faults->batt_ovp = (value >> 3) & 1;
    faults->ntc_cold = ((value >> 0) & 3) == 0b01;
    faults->ntc_hot = ((value >> 0) & 3) == 0b10;
    faults->ntc_boost = (value >> 2) & 1;
}

void pmic_vbus_test() {
    uint8_t reg0b;
    pm_i2c_read_reg(0x6a, 0x0b, &reg0b, 1);
    //printf("REG0B: %02x\r\n", reg0b);
}

void pmic_vbus_attached(bool* attached) {
    const uint8_t reg = 0x11;
    uint8_t value;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    *attached = (value >> 7) & 1;
    //printf("REG11: %02x = %u\r\n", value, *attached);
}

// REG0E
void pmic_adc_read_vbatt(uint16_t* vbatt, bool* treg) {
    const uint8_t reg = 0x0E;
    uint8_t value;
    pm_i2c_read_reg(0x6a, reg, &value, 1);

    // REG0E
    if (treg != NULL) {
        *treg = (value >> 7) & 1;
    }
    if (vbatt != NULL) {
    *vbatt = 2304;
        if ((value >> 6) & 1) *vbatt += 1280;
        if ((value >> 5) & 1) *vbatt += 640;
        if ((value >> 4) & 1) *vbatt += 320;
        if ((value >> 3) & 1) *vbatt += 160;
        if ((value >> 2) & 1) *vbatt += 80;
        if ((value >> 1) & 1) *vbatt += 40;
        if ((value >> 0) & 1) *vbatt += 20;
    }
}

void pmic_adc_test() {
    const uint8_t reg = 0x0E;
    uint8_t buffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    pm_i2c_read_reg(0x6a, reg, buffer, sizeof(buffer));

    // REG0E
    bool treg = (buffer[0] >> 7) & 1;
    uint16_t vbatt = 2304;
    if ((buffer[0] >> 6) & 1) vbatt += 1280;
    if ((buffer[0] >> 5) & 1) vbatt += 640;
    if ((buffer[0] >> 4) & 1) vbatt += 320;
    if ((buffer[0] >> 3) & 1) vbatt += 160;
    if ((buffer[0] >> 2) & 1) vbatt += 80;
    if ((buffer[0] >> 1) & 1) vbatt += 40;
    if ((buffer[0] >> 0) & 1) vbatt += 20;

    // REG0F
    uint16_t vsys = 2304;
    if ((buffer[1] >> 6) & 1) vsys += 1280;
    if ((buffer[1] >> 5) & 1) vsys += 640;
    if ((buffer[1] >> 4) & 1) vsys += 320;
    if ((buffer[1] >> 3) & 1) vsys += 160;
    if ((buffer[1] >> 2) & 1) vsys += 80;
    if ((buffer[1] >> 1) & 1) vsys += 40;
    if ((buffer[1] >> 0) & 1) vsys += 20;

    // REG10
    uint32_t ts = 21;
    if ((buffer[2] >> 6) & 1) ts += 29760;
    if ((buffer[2] >> 5) & 1) ts += 14880;
    if ((buffer[2] >> 4) & 1) ts += 7440;
    if ((buffer[2] >> 3) & 1) ts += 3720;
    if ((buffer[2] >> 2) & 1) ts += 1860;
    if ((buffer[2] >> 1) & 1) ts += 930;
    if ((buffer[2] >> 0) & 1) ts += 465;

    // REG11
    bool vbus_attached = (buffer[3] >> 7) & 1;
    uint16_t vbus = 2304;
    if ((buffer[3] >> 6) & 1) vbus += 6400;
    if ((buffer[3] >> 5) & 1) vbus += 3200;
    if ((buffer[3] >> 4) & 1) vbus += 1600;
    if ((buffer[3] >> 3) & 1) vbus += 800;
    if ((buffer[3] >> 2) & 1) vbus += 400;
    if ((buffer[3] >> 1) & 1) vbus += 200;
    if ((buffer[3] >> 0) & 1) vbus += 100;

    // REG12
    uint16_t charge_current = 0;
    if ((buffer[4] >> 6) & 1) charge_current += 3200;
    if ((buffer[4] >> 5) & 1) charge_current += 1600;
    if ((buffer[4] >> 4) & 1) charge_current += 800;
    if ((buffer[4] >> 3) & 1) charge_current += 400;
    if ((buffer[4] >> 2) & 1) charge_current += 200;
    if ((buffer[4] >> 1) & 1) charge_current += 100;
    if ((buffer[4] >> 0) & 1) charge_current += 50;

    if (!vbus_attached && vbus == 2304) {
        vbus = 0;
    }

    //printf("Treg: %s, Vbatt: %u mV, Vsys: %u mV, TS %lu%%, Vbus: %s %u mV, Ichrg: %u mA (%02x %02x %02x %02x %02x)\r\n", treg ? "Y" : "N", vbatt, vsys, ts / 100, vbus_attached ? "Y" : "N", vbus, charge_current, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
}

void pmic_battery_attached(bool* battery_attached, bool detect_empty_battery) {
    *battery_attached = false;

    // Algorithm 1: detect battery with normal voltage
    pmic_batt_load_config(true); // Apply 30mA load on battery
    Delay_Ms(5);
    uint16_t vbatt = 0;
    pmic_adc_read_vbatt(&vbatt, NULL);
    pmic_batt_load_config(false); // Disable 30mA load on battery
    if (vbatt >= 3000) {
        *battery_attached = true;
        return;
    }

    // Algorithm 2: detect empty battery
    if (detect_empty_battery) {
        //to-do
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
    funPinMode(pin_sda, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
    funPinMode(pin_scl, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
    SetupI2CSlave(0x5f, i2c_registers, sizeof(i2c_registers), i2c_write_cb, i2c_read_cb, false);

    // Initialize I2C master
#if HARDWARE_REV > 1
    funPinMode(pin_pm_sda, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
    funPinMode(pin_pm_scl, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL
    SetupI2CMaster();
    pmic_power_on(); // Enable battery
    pmic_set_input_current_limit(3250, false);
    pmic_otg_config(false); // Disable OTG booster
    pmic_chg_config(false); // Disable battery charging
    pmic_batt_load_config(false); // Disable 30mA load on battery
    pmic_set_minimum_system_voltage_limit(3500); // 3.5v (default)
    pmic_watchdog(0);
    pmic_adc_control(true, true);
    pmic_ico(false);
    pmic_battery_threshold(4200, true, false);
    pmic_set_fast_charge_current(512, false);
#endif

    // ESP32-C6
    funPinMode(pin_c6_enable, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_c6_enable, FUN_LOW);
    funPinMode(pin_c6_boot, GPIO_Speed_10MHz | GPIO_CNF_OUT_OD);
    funDigitalWrite(pin_c6_boot, FUN_HIGH);

#if OVERRIDE_C6
    funDigitalWrite(pin_c6_enable, FUN_HIGH); // Turn on radio
#endif

    // Display backlight
    timer3_init(); // Use timer 3 channel 1 as PWM output for controlling display backlight

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
    AFIO->PCFR1 |= AFIO_PCFR1_PD01_REMAP;

    funPinMode(pin_amplifier_enable, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(pin_amplifier_enable, FUN_LOW);

    // Real time clock
    rtc_init();

    // Backup registers
    bkp_read_all();

    bool power_button_latch = false;
    uint8_t power_button_counter = 0;

    uint16_t backlight_fade_value = 0;

    bool prev_vbus_attached = false;

    uint8_t empty_battery_delay = 4;

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

            if (!funDigitalRead(pin_power_in)) {
                if (power_button_latch && power_button_counter > 500 / input_scan_interval) {
                    pmic_power_off();
                }
                power_button_counter++;
            } else {
                power_button_latch = true;
                power_button_counter = 0;
            }
        }

        static uint32_t rtc_previous = 0;
        if (now - rtc_previous >= 1 * DELAY_MS_TIME) {
            rtc_previous = now;
            uint32_t value = rtc_get_counter();
            LockI2CSlave(true);
            i2c_registers[I2C_REG_RTC_VALUE_0] = (value >> 0) & 0xFF;
            i2c_registers[I2C_REG_RTC_VALUE_1] = (value >> 8) & 0xFF;
            i2c_registers[I2C_REG_RTC_VALUE_2] = (value >> 16) & 0xFF;
            i2c_registers[I2C_REG_RTC_VALUE_3] = (value >> 24) & 0xFF;
            LockI2CSlave(false);
        }

        /*static uint32_t bl_previous = 0;
        if (now - bl_previous >= 20 * DELAY_MS_TIME) {
            bl_previous = now;
            if (backlight_fade_value < timer3_pwm_cycle_width) {
                timer3_set(backlight_fade_value);
                backlight_fade_value++;
            }
        }*/

        static uint32_t pmic_previous = 0;
        if (now - pmic_previous >= 500 * DELAY_MS_TIME) {
            pmic_previous = now;

            if (empty_battery_delay > 0) {
                empty_battery_delay -= 1;
            }

            bool vbus_attached = false;
            pmic_vbus_attached(&vbus_attached);
            //printf("Tick: %u %u\r\n", prev_vbus_attached, vbus_attached);
            if (!prev_vbus_attached && vbus_attached) {
                //printf("USB ATTACHED\r\n");
                // Badge has been connected to USB supply
                pmic_chg_config(false); // Disable battery charging
                bool battery_attached = false;
                pmic_battery_attached(&battery_attached, empty_battery_delay == 0);
                if (battery_attached) {
                    //printf("BATTERY CHARGER ENABLED\r\n");
                    pmic_chg_config(true); // Enable battery charging
                } else {
                    //printf("BATTERY CHARGER DISABLED: no battery\r\n");
                }
            } else if (prev_vbus_attached && !vbus_attached) {
                // Badge has been disconnected from USB supply
                pmic_chg_config(false); // Disable battery charging
                pmic_batt_load_config(false); // Disable 30mA load on battery
                //printf("USB DETACHED\r\n");
            }
            prev_vbus_attached = vbus_attached;
        }

        funDigitalWrite(pin_interrupt, (keyboard_interrupt | input_interrupt) ? FUN_LOW : FUN_HIGH); // Update interrupt pin state
    }
}
