#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum pmic_result {
    pmic_ok = 0,
    pmic_error = 1,
} pmic_result_t;

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

// REG00
pmic_result_t pmic_set_input_current_limit(uint16_t current, bool enable_ilim_pin, bool enable_hiz);
pmic_result_t pmic_get_input_current_limit(uint16_t* out_current, bool* out_enable_ilim_pin, bool* out_enable_hiz);

// REG01

// REG02
pmic_result_t pmic_adc_control(bool enable, bool continuous);

void pmic_ico(bool enable);
void pmic_otg_config(bool enable);
void pmic_chg_config(bool enable);
void pmic_batt_load_config(bool enable);
void pmic_set_minimum_system_voltage_limit(uint16_t voltage);
void pmic_set_fast_charge_current(uint16_t current, bool en_pumpx);
void pmic_battery_threshold(uint16_t voltage_limit, bool batlowv, bool vrechg);
void pmic_watchdog(uint8_t watchdog_setting);
void pmic_control_battery_connection(bool enable);
void pmic_power_on();
void pmic_power_off();

// REG0C
pmic_result_t pmic_read_faults(uint8_t* out_raw, pmic_faults_t* out_faults);

void pmic_vbus_test();
void pmic_vbus_attached(bool* attached);

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

void pmic_adc_test();
void pmic_battery_attached(bool* battery_attached, bool detect_empty_battery);
