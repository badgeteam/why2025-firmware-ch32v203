#pragma once

#include <stdint.h>

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t iinlim : 6;   // Input current limit (limited by ILIM if enabled)
            uint8_t en_ilim : 1;  // Enable input current limit (ILIM) of 1.5A (260Ω)
            uint8_t en_hiz : 1;   // Disable power input from USB-C port
        };
    };
} bq25895_reg00_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t vindpm_os : 5;  // Input voltage threshold offset (see REG0D)
            uint8_t bcold : 1;      // Boost mode cold temperature monitor threshold
            uint8_t bhot : 2;       // Boost mode hot temperature monitor threshold
        };
    };
} bq25895_reg01_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t auto_dpdm_en : 1;  // Automatic D+/D- selection enable
            uint8_t force_dpdm : 1;    // Force D+/D- detection
            uint8_t maxc_en : 1;       // MaxCharge adapter enable
            uint8_t hvdcp_en : 1;      // High voltage DCP enable
            uint8_t ico_en : 1;        // Input current optimizer enable
            uint8_t boost_freq : 1;    // OTG booster frequency: 0 = 1.5MHz, 1 = 500kHz
            uint8_t conv_rate : 1;     // ADC continuous mode enable
            uint8_t conv_start : 1;    // ADC enable
        };
    };
} bq25895_reg02_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t unused : 1;
            uint8_t sys_min : 3;     // Minimum system voltage limit
            uint8_t chg_config : 1;  // Enable battery charger
            uint8_t otg_config : 1;  // Enable OTG booster
            uint8_t wd_rst : 1;      // Reset I2C watchdog timer
            uint8_t bat_loaden : 1;  // Apply 30mA load to battery
        };
    };
} bq25895_reg03_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t ichg : 7;      // Battery charging current limit (default 2048mA)
            uint8_t en_pumpx : 1;  // Current pulse control enable (default disabled)
        };
    };
} bq25895_reg04_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t iterm : 4;
            uint8_t iprechg : 4;
        };
    };
} bq25895_reg05_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t vrechg : 1;
            uint8_t batlowv : 1;
            uint8_t vreg : 6;
        };
    };
} bq25895_reg06_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t unused : 1;
            uint8_t chg_timer : 2;
            uint8_t en_timer : 1;
            uint8_t watchdog : 2;
            uint8_t stat_dis : 1;
            uint8_t en_term : 1;
        };
    };
} bq25895_reg07_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t treg : 2;
            uint8_t vclamp : 3;
            uint8_t bat_comp : 3;
        };
    };
} bq25895_reg08_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t pumpx_dn : 1;       // Current pulse control voltage down enable (default disabled) see REG04
            uint8_t pumpx_up : 1;       // Current pulse control voltage up enable (default disabled) see REG04
            uint8_t batfet_rst_en : 1;  // Batfet full system reset enable (default enabled)
            uint8_t batfet_dly : 1;     // Batfet delay (default turn off immediately on batfet disable)
            uint8_t unused : 1;
            uint8_t batfet_dis : 1;  // Disconnect battery
            uint8_t tmr2x_en : 1;    // Safety timer slowdown (default enabled)
            uint8_t force_ico : 1;   // Force start input current optimizer (ICO)
        };
    };
} bq25895_reg09_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t unused : 4;
            uint8_t boostv : 4;  // Boost mode voltage regulation (default 5.126v)
        };
    };
} bq25895_reg0A_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t vsys_stat : 1;  // Minimum system voltage regulation mode status
            uint8_t sdp_stat : 1;   // USB input status (100/500mA input)
            uint8_t pg_stat : 1;    // Power good status
            uint8_t chrg_stat : 2;  // Charging status
            uint8_t vbus_stat : 3;  // Input (vbus) status
        };
    };
} bq25895_reg0B_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t ntc_fault : 3;       // Temperature sensor fault
            uint8_t bat_fault : 1;       // Battery over voltage fault
            uint8_t chrg_fault : 2;      // Charge fault
            uint8_t boost_fault : 1;     // Boost mode fault
            uint8_t watchdog_fault : 1;  // Watchdog fault
        };
    };
} bq25895_reg0C_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t vindpm : 7;        // Input voltage lower threshold (default 4.4v + VINDPM_OS 0.5v
                                       // = 4.9v)
            uint8_t force_vindpm : 1;  // Use VINDPM as absolute threshold and don't add VINDPM_OS
        };
    };
} bq25895_reg0D_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t batv : 7;        // Battery voltage
            uint8_t therm_stat : 1;  // Thermal regulation status
        };
    };
} bq25895_reg0E_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t sysv : 7;  // System voltage
            uint8_t unused : 1;
        };
    };
} bq25895_reg0F_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t tspct : 7;  // TS voltage as percentage of REGN
            uint8_t unused : 1;
        };
    };
} bq25895_reg10_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t busv : 7;     // Input voltage (vbus)
            uint8_t busv_gd : 1;  // Input power good
        };
    };
} bq25895_reg11_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t ichgr : 7;  // Battery charging current
            uint8_t unused : 1;
        };
    };
} bq25895_reg12_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t idpm_lim : 6;   // Effective input current limit (ICO)
            uint8_t ipdm_stat : 1;  // Input current limited status
            uint8_t vpdm_stat : 1;  // Input voltage limited status
        };
    };
} bq25895_reg13_t;

typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t dev_rev : 2;        // Device revision (01)
            uint8_t ts_profile : 1;     // Temperature profile (0)
            uint8_t pn : 3;             // Device configuration (111)
            uint8_t ico_optimized : 1;  // ICO optimized status
            uint8_t reg_rst : 1;        // Register reset (write 1 to reset PMIC)
        };
    };
} bq25895_reg14_t;
