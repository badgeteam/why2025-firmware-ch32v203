#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    pmic_ok = 0,
    pmic_error = 1,
} pmic_result_t;

typedef struct {
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

typedef enum {
    PMIC_BHOT_1 = 0,
    PMIC_BHOT_0 = 1,
    PMIC_BHOT_2 = 2,
    PMIC_BHOT_DISABLE = 3,
} pmic_bhot_t;

// REG00
pmic_result_t pmic_set_input_current_limit(uint16_t current, bool enable_ilim_pin, bool enable_hiz);
pmic_result_t pmic_get_input_current_limit(uint16_t* out_current, bool* out_enable_ilim_pin, bool* out_enable_hiz);

// REG01
pmic_result_t pmic_set_input_voltage_limit_offset(uint8_t offset);
pmic_result_t pmic_get_input_voltage_limit_offset(uint8_t* out_offset);
pmic_result_t pmic_set_boost_mode_temperature_monitor_thresholds(bool cold, pmic_bhot_t hot);
pmic_result_t pmic_get_boost_mode_temperature_monitor_thresholds(bool* out_cold, pmic_bhot_t* out_hot);

// REG02
pmic_result_t pmic_set_otg_boost_frequency(bool low_frequency);
pmic_result_t pmic_get_otg_boost_frequency(bool* out_low_frequency);
pmic_result_t pmic_set_input_current_optimizer(bool enable);
pmic_result_t pmic_get_input_current_optimizer(bool* out_enable);
pmic_result_t pmic_set_high_voltage_dcp(bool enable);
pmic_result_t pmic_get_high_voltage_dcp(bool* out_enable);
pmic_result_t pmic_set_maxcharge(bool enable);
pmic_result_t pmic_get_maxcharge(bool* out_enable);
pmic_result_t pmic_set_adc_configuration(bool start, bool continuous);
pmic_result_t pmic_get_adc_configuration(bool* out_busy, bool* out_continuous);

// REG03
pmic_result_t pmic_set_minimum_system_voltage_limit(uint16_t millivolt);
pmic_result_t pmic_get_minimum_system_voltage_limit(uint16_t* out_millivolt);
pmic_result_t pmic_set_charge_enable(bool enable);
pmic_result_t pmic_get_charge_enable(bool* out_enable);
pmic_result_t pmic_set_otg_enable(bool enable);
pmic_result_t pmic_get_otg_enable(bool* out_enable);
pmic_result_t pmic_reset_watchdog(void);
pmic_result_t pmic_set_battery_load_enable(bool enable);
pmic_result_t pmic_get_battery_load_enable(bool* out_enable);

// REG04
pmic_result_t pmic_set_fast_charge_current(uint16_t current);
pmic_result_t pmic_get_fast_charge_current(uint16_t* out_current);
pmic_result_t pmic_set_pumpx_enable(bool enable);
pmic_result_t pmic_get_pumpx_enable(bool* out_enable);

// REG05
pmic_result_t pmic_set_termination_current(uint16_t current);
pmic_result_t pmic_get_termination_current(uint16_t* out_current);
pmic_result_t pmic_set_precharge_current(uint16_t current);
pmic_result_t pmic_get_precharge_current(uint16_t* out_current);

// REG06
pmic_result_t pmic_set_recharge_threshold_200mv_offset(bool enable);
pmic_result_t pmic_get_recharge_threshold_200mv_offset(bool* out_enable);
pmic_result_t pmic_set_battery_precharge_threshold_3v(bool enable);
pmic_result_t pmic_get_battery_precharge_threshold_3v(bool* out_enable);
pmic_result_t pmic_set_charge_voltage_limit(uint16_t voltage);
pmic_result_t pmic_get_charge_voltage_limit(uint16_t* out_voltage);

void pmic_battery_threshold(uint16_t voltage_limit, bool batlowv, bool vrechg);

// REG07
pmic_result_t pmic_set_charge_timer_limit(uint8_t hours);
pmic_result_t pmic_get_charge_timer_limit(uint8_t* out_hours);
pmic_result_t pmic_set_charge_timer_enable(bool enable);
pmic_result_t pmic_get_charge_timer_enable(bool* out_enable);
pmic_result_t pmic_set_watchdog_timer_limit(uint8_t seconds);
pmic_result_t pmic_get_watchdog_timer_limit(uint8_t* out_seconds);
pmic_result_t pmic_set_status_pin_disable(bool disable);
pmic_result_t pmic_get_status_pin_disable(bool* out_disable);
pmic_result_t pmic_set_charge_termination_enable(bool enable);
pmic_result_t pmic_get_charge_termination_enable(bool* out_enable);

void pmic_watchdog(uint8_t watchdog_setting);

// REG08
pmic_result_t pmic_set_thermal_regulation_threshold(uint8_t temperature);
pmic_result_t pmic_get_thermal_regulation_threshold(uint8_t* out_temperature);
pmic_result_t pmic_set_ir_compensation_voltage_clamp(uint8_t millivolt);
pmic_result_t pmic_get_ir_compensation_voltage_clamp(uint8_t* out_millivolt);
pmic_result_t pmic_set_ir_compensation_resistor_setting(uint8_t milliohm);
pmic_result_t pmic_get_ir_compensation_resistor_setting(uint8_t* out_milliohm);

// REG09
pmic_result_t pmic_set_pumpx_down_enable(bool enable);
pmic_result_t pmic_get_pumpx_down_enable(bool* out_enable);
pmic_result_t pmic_set_pumpx_up_enable(bool enable);
pmic_result_t pmic_get_pumpx_up_enable(bool* out_enable);
pmic_result_t pmic_set_battery_full_system_reset_enable(bool enable);
pmic_result_t pmic_get_battery_full_system_reset_enable(bool* out_enable);
pmic_result_t pmic_set_battery_disconnect_delay_enable(bool enable);
pmic_result_t pmic_get_battery_disconnect_delay_enable(bool* out_enable);
pmic_result_t pmic_set_battery_disconnect_enable(bool enable);
pmic_result_t pmic_get_battery_disconnect_enable(bool* out_enable);
pmic_result_t pmic_set_safety_timer_slow_2x_enable(bool enable);
pmic_result_t pmic_get_safety_timer_slow_2x_enable(bool* out_enable);
pmic_result_t pmic_set_force_ico_enable(bool enable);
pmic_result_t pmic_get_force_ico_enable(bool* out_enable);

void pmic_control_battery_connection(bool enable);
void pmic_power_off();

// REG0A
pmic_result_t pmic_set_boost_mode_voltage(uint8_t millivolt);
pmic_result_t pmic_get_boost_mode_voltage(uint8_t* out_millivolt);

// REG0B
pmic_result_t pmic_get_vsys_regulation_status(bool* out_active);
pmic_result_t pmic_get_usb_input_sdp_status(bool* out_usb500);
pmic_result_t pmic_get_power_good_status(bool* out_active);
pmic_result_t pmic_get_charging_status(uint8_t* out_status);
pmic_result_t pmic_get_vbus_status(uint8_t* out_status);

// REG0C
pmic_result_t pmic_read_faults(uint8_t* out_raw, pmic_faults_t* out_faults);

// REG0E
pmic_result_t pmic_adc_read_batv(uint16_t* out_batv, bool* out_therm_stat);

// REG0F
pmic_result_t pmic_adc_read_sysv(uint16_t* out_sysv);

// REG10
pmic_result_t pmic_adc_read_tspct(uint16_t* out_tspct);

// REG11
pmic_result_t pmic_adc_read_busv(uint16_t* out_busv, bool* out_busv_gd);

// REG12
pmic_result_t pmic_adc_read_ichgr(uint16_t* out_ichgr);

// REG13
// pmic_result_t pmic_get_input_current_limit(bool* out_milliampere);
pmic_result_t pmic_get_input_current_limit_status(bool* out_active);
pmic_result_t pmic_get_voltage_input_limit_status(bool* out_active);

// REG14
pmic_result_t pmic_get_device_revision(uint8_t* out_revision);
pmic_result_t pmic_get_temperature_profile(bool* out_value);
pmic_result_t pmic_get_device_configuration(uint8_t* out_devcfg);
pmic_result_t pmic_get_ico_optimized_status(bool* out_active);
pmic_result_t pmic_register_reset(void);

void pmic_battery_attached(bool* battery_attached, bool detect_empty_battery);
