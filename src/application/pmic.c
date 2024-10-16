#include "pmic.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ch32v003fun.h"
#include "i2c_master.h"
#include "pmic_regs.h"

#define BQ25895_ADDR 0x6a

static pmic_result_t pmic_read_reg(uint8_t reg, uint8_t* out_value) {
    return (pm_i2c_read_reg(BQ25895_ADDR, reg, out_value, 1) == i2c_ok) ? pmic_ok : pmic_error;
}

static pmic_result_t pmic_write_reg(uint8_t reg, uint8_t value) {
    return (pm_i2c_write_reg(BQ25895_ADDR, reg, &value, 1) == i2c_ok) ? pmic_ok : pmic_error;
}

// REG00
pmic_result_t pmic_set_input_current_limit(uint16_t current, bool enable_ilim_pin, bool enable_hiz) {
    bq25895_reg00_t value = {0};

    if (current < 100) {  // Minimum current is 100 mA
        current = 100;
    }
    current -= 100;  // Offset is 100 mA

    value.en_hiz = enable_hiz;
    value.en_ilim = enable_ilim_pin;

    if (current >= 1600) {
        current -= 1600;
        value.iinlim |= (1 << 5);  // Add 1600 mA
    }
    if (current >= 800) {
        current -= 800;
        value.iinlim |= (1 << 4);  // Add 800 mA
    }
    if (current >= 400) {
        current -= 400;
        value.iinlim |= (1 << 3);  // Add 400 mA
    }
    if (current >= 200) {
        current -= 200;
        value.iinlim |= (1 << 2);  // Add 200 mA
    }
    if (current >= 100) {
        current -= 100;
        value.iinlim |= (1 << 1);  // Add 100 mA
    }
    if (current >= 50) {
        current -= 50;
        value.iinlim |= (1 << 0);  // Add 50 mA
    }

    return pmic_write_reg(0x00, value.raw);
}

pmic_result_t pmic_get_input_current_limit(uint16_t* out_current, bool* out_enable_ilim_pin, bool* out_enable_hiz) {
    bq25895_reg00_t value;
    pmic_result_t res = pmic_read_reg(0x00, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable_hiz) {
        *out_enable_hiz = value.en_hiz;
    }
    if (out_enable_ilim_pin) {
        *out_enable_ilim_pin = value.en_ilim;
    }
    if (out_current) {
        *out_current = 100;  // Offset is 100mA
        if (value.iinlim & (1 << 5)) *out_current += 1600;
        if (value.iinlim & (1 << 4)) *out_current += 800;
        if (value.iinlim & (1 << 3)) *out_current += 400;
        if (value.iinlim & (1 << 2)) *out_current += 200;
        if (value.iinlim & (1 << 1)) *out_current += 100;
        if (value.iinlim & (1 << 0)) *out_current += 50;
    }
    return pmic_ok;
}

// REG01
pmic_result_t pmic_set_input_voltage_limit_offset(uint8_t offset) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != pmic_ok) return res;
    value.vindpm_os = offset;
    return pmic_write_reg(0x01, value.raw);
}

pmic_result_t pmic_get_input_voltage_limit_offset(uint8_t* out_offset) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != pmic_ok) return res;
    if (out_offset) {
        *out_offset = value.vindpm_os;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_boost_mode_temperature_monitor_thresholds(bool cold, pmic_bhot_t hot) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != pmic_ok) return res;
    value.bcold = cold;
    value.bhot = (uint8_t)hot;
    return pmic_write_reg(0x01, value.raw);
}

pmic_result_t pmic_get_boost_mode_temperature_monitor_thresholds(bool* out_cold, pmic_bhot_t* out_hot) {
    bq25895_reg01_t value = {0};
    pmic_result_t res = pmic_read_reg(0x01, &value.raw);
    if (res != pmic_ok) return res;
    if (out_cold) {
        *out_cold = value.bcold;
    }
    if (out_hot) {
        *out_hot = (pmic_bhot_t)value.bhot;
    }
    return pmic_ok;
}

// REG02
pmic_result_t pmic_set_otg_boost_frequency(bool low_frequency) {
    // Low frequency: 500kHz, high frequency: 1.5MHz
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    value.boost_freq = low_frequency;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_otg_boost_frequency(bool* out_low_frequency) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    if (out_low_frequency) {
        *out_low_frequency = value.boost_freq;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_input_current_optimizer(bool enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    value.ico_en = enable;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_input_current_optimizer(bool* out_enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.ico_en;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_high_voltage_dcp(bool enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    value.hvdcp_en = enable;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_high_voltage_dcp(bool* out_enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.hvdcp_en;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_maxcharge(bool enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    value.maxc_en = enable;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_maxcharge(bool* out_enable) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.maxc_en;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_adc_configuration(bool start, bool continuous) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    value.conv_start = start;
    value.conv_rate = continuous;
    return pmic_write_reg(0x02, value.raw);
}

pmic_result_t pmic_get_adc_configuration(bool* out_busy, bool* out_continuous) {
    bq25895_reg02_t value;
    pmic_result_t res = pmic_read_reg(0x02, &value.raw);
    if (res != pmic_ok) return res;
    if (out_busy) {
        *out_busy = value.conv_start;
    }
    if (out_continuous) {
        *out_continuous = value.conv_rate;
    }
    return pmic_ok;
}

// REG03
pmic_result_t pmic_set_minimum_system_voltage_limit(uint16_t millivolt) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;

    value.sys_min = 0;

    if (millivolt < 3000) {
        millivolt = 3000;  // Minimum
    }
    millivolt -= 3000;  // Offset

    if (millivolt > 400) {
        millivolt -= 400;
        value.sys_min |= (1 << 2);
    }

    if (millivolt > 200) {
        millivolt -= 200;
        value.sys_min |= (1 << 1);
    }

    if (millivolt > 100) {
        millivolt -= 100;
        value.sys_min |= (1 << 0);
    }

    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_minimum_system_voltage_limit(uint16_t* out_millivolt) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;

    uint16_t millivolt = 3000;

    if (value.sys_min & (1 << 2)) {
        millivolt += 400;
    }

    if (value.sys_min & (1 << 1)) {
        millivolt += 200;
    }

    if (value.sys_min & (1 << 0)) {
        millivolt += 100;
    }

    if (out_millivolt) {
        *out_millivolt = millivolt;
    }

    return pmic_ok;
}

pmic_result_t pmic_set_charge_enable(bool enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    value.chg_config = enable;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_charge_enable(bool* out_enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.chg_config;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_otg_enable(bool enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    value.otg_config = enable;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_otg_enable(bool* out_enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.otg_config;
    }
    return pmic_ok;
}

pmic_result_t pmic_reset_watchdog(void) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    value.wd_rst = true;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_set_battery_load_enable(bool enable) {
    // Enabling this puts a 30mA load on the battery
    // for battery presence detection
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    value.bat_loaden = enable;
    return pmic_write_reg(0x03, value.raw);
}

pmic_result_t pmic_get_battery_load_enable(bool* out_enable) {
    bq25895_reg03_t value;
    pmic_result_t res = pmic_read_reg(0x03, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.bat_loaden;
    }
    return pmic_ok;
}

// REG04

pmic_result_t pmic_set_fast_charge_current(uint16_t current) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != pmic_ok) return res;
    value.ichg = 0;
    if (current >= 4096) {
        current -= 4096;
        value.ichg |= (1 << 6);  // Add 4096mA
    }
    if (current >= 2048) {
        current -= 2048;
        value.ichg |= (1 << 5);  // Add 2048mA
    }
    if (current >= 1024) {
        current -= 1024;
        value.ichg |= (1 << 4);  // Add 1024mA
    }
    if (current >= 512) {
        current -= 512;
        value.ichg |= (1 << 3);  // Add 512mA
    }
    if (current >= 256) {
        current -= 256;
        value.ichg |= (1 << 2);  // Add 256mA
    }
    if (current >= 128) {
        current -= 128;
        value.ichg |= (1 << 1);  // Add 128mA
    }
    if (current >= 64) {
        current -= 64;
        value.ichg |= (1 << 0);  // Add 64mA
    }
    return pmic_write_reg(0x04, value.raw);
}

pmic_result_t pmic_get_fast_charge_current(uint16_t* out_current) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != pmic_ok) return res;
    if (out_current) {
        uint16_t current = 0;
        if (value.ichg & (1 << 6)) {
            current += 4096;
        }
        if (value.ichg & (1 << 5)) {
            current += 2048;
        }
        if (value.ichg & (1 << 4)) {
            current += 1024;
        }
        if (value.ichg & (1 << 3)) {
            current += 512;
        }
        if (value.ichg & (1 << 2)) {
            current += 256;
        }
        if (value.ichg & (1 << 1)) {
            current += 128;
        }
        if (value.ichg & (1 << 0)) {
            current += 64;
        }
        *out_current = current;
    }
    return pmic_ok;
}

pmic_result_t pmic_set_pumpx_enable(bool enable) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != pmic_ok) return res;
    value.en_pumpx = enable;
    return pmic_write_reg(0x04, value.raw);
}

pmic_result_t pmic_get_pumpx_enable(bool* out_enable) {
    bq25895_reg04_t value;
    pmic_result_t res = pmic_read_reg(0x04, &value.raw);
    if (res != pmic_ok) return res;
    if (out_enable) {
        *out_enable = value.en_pumpx;
    }
    return pmic_ok;
}

// REG05
pmic_result_t pmic_set_termination_current(uint16_t current) {
    return pmic_error;
}

pmic_result_t pmic_get_termination_current(uint16_t* out_current) {
    return pmic_error;
}

pmic_result_t pmic_set_precharge_current(uint16_t current) {
    return pmic_error;
}

pmic_result_t pmic_get_precharge_current(uint16_t* out_current) {
    return pmic_error;
}

// REG06
pmic_result_t pmic_set_recharge_threshold_200mv_offset(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_recharge_threshold_200mv_offset(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_battery_precharge_threshold_3v(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_battery_precharge_threshold_3v(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_charge_voltage_limit(uint16_t voltage) {
    return pmic_error;
}

pmic_result_t pmic_get_charge_voltage_limit(uint16_t* out_voltage) {
    return pmic_error;
}

void pmic_battery_threshold(uint16_t voltage_limit, bool batlowv, bool vrechg) {
    const uint8_t reg = 0x06;
    uint8_t value = 0;

    if (voltage_limit < 3840) {
        voltage_limit = 3840;
    }
    voltage_limit -= 3840;  // Offset
    if (voltage_limit > 512) {
        voltage_limit -= 512;
        value |= (1 << 7);  // Add 512mA
    }
    if (voltage_limit > 256) {
        voltage_limit -= 256;
        value |= (1 << 6);  // Add 256mA
    }
    if (voltage_limit > 128) {
        voltage_limit -= 128;
        value |= (1 << 5);  // Add 128mA
    }
    if (voltage_limit > 64) {
        voltage_limit -= 64;
        value |= (1 << 4);  // Add 64mA
    }
    if (voltage_limit > 32) {
        voltage_limit -= 32;
        value |= (1 << 3);  // Add 32mA
    }
    if (voltage_limit > 16) {
        voltage_limit -= 16;
        value |= (1 << 2);  // Add 16mA
    }
    if (batlowv) {
        value |= (1 << 1);  // Battery precharge to fast charge threshold: 1 is 3.0v (default), 0 is 2.8v
    }
    if (vrechg) {
        value |= (1 << 0);  // Battery recharge threshold offset: 1 is 200mV below VREG, 0 is 100mV below VREG (default)
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG07
pmic_result_t pmic_set_charge_timer_limit(uint8_t hours) {
    return pmic_error;
}

pmic_result_t pmic_get_charge_timer_limit(uint8_t* out_hours) {
    return pmic_error;
}

pmic_result_t pmic_set_charge_timer_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_charge_timer_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_watchdog_timer_limit(uint8_t seconds) {
    return pmic_error;
}

pmic_result_t pmic_get_watchdog_timer_limit(uint8_t* out_seconds) {
    return pmic_error;
}

pmic_result_t pmic_set_status_pin_disable(bool disable) {
    return pmic_error;
}

pmic_result_t pmic_get_status_pin_disable(bool* out_disable) {
    return pmic_error;
}

pmic_result_t pmic_set_charge_termination_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_charge_termination_enable(bool* out_enable) {
    return pmic_error;
}

void pmic_watchdog(uint8_t watchdog_setting) {
    const uint8_t reg = 0x07;
    uint8_t value = 0x00;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    value &= ~(0b00110000);
    value |= (watchdog_setting & 3) << 4;  // Watchdog
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG08
pmic_result_t pmic_set_thermal_regulation_threshold(uint8_t temperature) {
    return pmic_error;
}

pmic_result_t pmic_get_thermal_regulation_threshold(uint8_t* out_temperature) {
    return pmic_error;
}

pmic_result_t pmic_set_ir_compensation_voltage_clamp(uint8_t millivolt) {
    return pmic_error;
}

pmic_result_t pmic_get_ir_compensation_voltage_clamp(uint8_t* out_millivolt) {
    return pmic_error;
}

pmic_result_t pmic_set_ir_compensation_resistor_setting(uint8_t milliohm) {
    return pmic_error;
}

pmic_result_t pmic_get_ir_compensation_resistor_setting(uint8_t* out_milliohm) {
    return pmic_error;
}

// REG09
pmic_result_t pmic_set_pumpx_down_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_pumpx_down_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_pumpx_up_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_pumpx_up_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_battery_full_system_reset_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_battery_full_system_reset_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_battery_disconnect_delay_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_battery_disconnect_delay_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_battery_disconnect_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_battery_disconnect_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_safety_timer_slow_2x_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_safety_timer_slow_2x_enable(bool* out_enable) {
    return pmic_error;
}

pmic_result_t pmic_set_force_ico_enable(bool enable) {
    return pmic_error;
}

pmic_result_t pmic_get_force_ico_enable(bool* out_enable) {
    return pmic_error;
}

void pmic_control_battery_connection(bool enable) {
    const uint8_t reg = 0x09;
    uint8_t value = 0x00;
    pm_i2c_read_reg(0x6a, reg, &value, 1);
    if (!enable) {
        value |= (1 << 5);  // Set BATFET_DIS bit
    } else {
        value &= (~1 << 5);  // Clear BATFET_DIS bit
    }
    pm_i2c_write_reg(0x6a, reg, &value, 1);
}

// REG0C
pmic_result_t pmic_read_faults(uint8_t* out_raw, pmic_faults_t* out_faults) {
    bq25895_reg0C_t value = {0};
    pmic_result_t res = pmic_read_reg(0x0C, &value.raw);
    if (res != pmic_ok) return res;

    if (out_raw) {
        *out_raw = value.raw;
    }

    if (out_faults) {
        out_faults->watchdog = value.watchdog_fault;
        out_faults->boost = value.boost_fault;
        out_faults->chrg_input = value.chrg_fault == 1;
        out_faults->chrg_thermal = value.chrg_fault == 2;
        out_faults->chrg_safety = value.chrg_fault == 3;
        out_faults->batt_ovp = value.bat_fault;
        out_faults->ntc_cold = (value.ntc_fault & 3) == 1;
        out_faults->ntc_hot = (value.ntc_fault & 3) == 2;
        out_faults->ntc_boost = (value.ntc_fault >> 2) & 1;
    }

    return pmic_ok;
}

// REG0E
pmic_result_t pmic_adc_read_batv(uint16_t* out_batv, bool* out_therm_stat) {
    bq25895_reg0E_t value;
    pmic_result_t res = pmic_read_reg(0x0E, &value.raw);
    if (res != pmic_ok) return res;

    if (out_batv) {
        uint16_t batv = 2304;
        if ((value.batv >> 6) & 1) batv += 1280;
        if ((value.batv >> 5) & 1) batv += 640;
        if ((value.batv >> 4) & 1) batv += 320;
        if ((value.batv >> 3) & 1) batv += 160;
        if ((value.batv >> 2) & 1) batv += 80;
        if ((value.batv >> 1) & 1) batv += 40;
        if ((value.batv >> 0) & 1) batv += 20;
        *out_batv = batv;
    }

    if (out_therm_stat) {
        *out_therm_stat = value.therm_stat;
    }

    return pmic_ok;
}

// REG0F
pmic_result_t pmic_adc_read_sysv(uint16_t* out_sysv) {
    bq25895_reg0F_t value;
    pmic_result_t res = pmic_read_reg(0x0F, &value.raw);
    if (res != pmic_ok) return res;

    if (out_sysv) {
        uint16_t sysv = 2304;
        if ((value.sysv >> 6) & 1) sysv += 1280;
        if ((value.sysv >> 5) & 1) sysv += 640;
        if ((value.sysv >> 4) & 1) sysv += 320;
        if ((value.sysv >> 3) & 1) sysv += 160;
        if ((value.sysv >> 2) & 1) sysv += 80;
        if ((value.sysv >> 1) & 1) sysv += 40;
        if ((value.sysv >> 0) & 1) sysv += 20;
        *out_sysv = sysv;
    }

    return pmic_ok;
}

// REG10
pmic_result_t pmic_adc_read_tspct(uint16_t* out_tspct) {
    // Divide the output of this function by 100 to get the percentage
    bq25895_reg10_t value;
    pmic_result_t res = pmic_read_reg(0x10, &value.raw);
    if (res != pmic_ok) return res;

    if (out_tspct) {
        uint16_t tspct = 2100;
        if ((value.tspct >> 6) & 1) tspct += 2976;
        if ((value.tspct >> 5) & 1) tspct += 1488;
        if ((value.tspct >> 4) & 1) tspct += 744;
        if ((value.tspct >> 3) & 1) tspct += 372;
        if ((value.tspct >> 2) & 1) tspct += 186;
        if ((value.tspct >> 1) & 1) tspct += 93;
        if ((value.tspct >> 0) & 1) tspct += 47;
        *out_tspct = tspct;
    }

    return pmic_ok;
}

// REG11
pmic_result_t pmic_adc_read_busv(uint16_t* out_busv, bool* out_busv_gd) {
    bq25895_reg11_t value;
    pmic_result_t res = pmic_read_reg(0x11, &value.raw);
    if (res != pmic_ok) return res;

    if (out_busv) {
        uint16_t busv = 2600;
        if ((value.busv >> 6) & 1) busv += 6400;
        if ((value.busv >> 5) & 1) busv += 3200;
        if ((value.busv >> 4) & 1) busv += 1600;
        if ((value.busv >> 3) & 1) busv += 800;
        if ((value.busv >> 2) & 1) busv += 400;
        if ((value.busv >> 1) & 1) busv += 200;
        if ((value.busv >> 0) & 1) busv += 100;
        *out_busv = busv;
    }

    if (out_busv_gd) {
        *out_busv_gd = value.busv_gd;
    }

    return pmic_ok;
}

// REG12
pmic_result_t pmic_adc_read_ichgr(uint16_t* out_ichgr) {
    bq25895_reg12_t value;
    pmic_result_t res = pmic_read_reg(0x12, &value.raw);
    if (res != pmic_ok) return res;

    if (out_ichgr) {
        uint16_t ichgr = 0;
        if ((value.ichgr >> 6) & 1) ichgr += 3200;
        if ((value.ichgr >> 5) & 1) ichgr += 1600;
        if ((value.ichgr >> 4) & 1) ichgr += 800;
        if ((value.ichgr >> 3) & 1) ichgr += 400;
        if ((value.ichgr >> 2) & 1) ichgr += 200;
        if ((value.ichgr >> 1) & 1) ichgr += 100;
        if ((value.ichgr >> 0) & 1) ichgr += 50;
        *out_ichgr = ichgr;
    }

    return pmic_ok;
}

/////////////////////////////

void pmic_power_off() {
    pmic_watchdog(0);
    pmic_set_adc_configuration(false, false);
    pmic_set_otg_enable(false);
    pmic_set_battery_load_enable(false);
    pmic_control_battery_connection(false);
}

void pmic_battery_attached(bool* battery_attached, bool detect_empty_battery) {
    *battery_attached = false;

    // Algorithm 1: detect battery with normal voltage
    pmic_set_battery_load_enable(true);  // Apply 30mA load on battery
    Delay_Ms(5);
    uint16_t vbatt = 0;
    pmic_adc_read_batv(&vbatt, NULL);
    pmic_set_battery_load_enable(false);  // Disable 30mA load on battery
    if (vbatt >= 3000) {
        *battery_attached = true;
        return;
    }

    // Algorithm 2: detect empty battery
    if (detect_empty_battery) {
        // to-do
    }
}
