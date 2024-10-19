# Tanmatsu coprocessor firmware

Firmware for the CH32V203 microcontroller used as coprocessor on the Tanmatsu.

## Features

- I2C peripheral device presenting at address 0x5F for access from the host device
- Interrupt line for notifying host of important events
- Keyboard matrix scanning
- Real time clock
- Backup registers (data stored in the RTC power domain)
- PMIC (BQ25895) control and monitoring, including battery detection, ADC reading and and soft power control
- PWM backlight brightness control

## Compatibility

This firmware is compatible with both the Tanmatsu and the WHY2025 badge. Please be sure to correctly adjust the HW_REV define in [main.c](main.c) to configure the firmware for the correct board.

Currently supported values:

| Value | Device              |
|-------|---------------------|
| 1     | WHY2025 prototype 1 |
| 2     | WHY2025 prototype 2 |

## License

This firmware, copyright 2024 Nicolai Electronics, is made available under the terms of the MIT license, see [LICENSE](LICENSE) for the full license text. The platform files come from the [ch32v003fun](https://github.com/cnlohr/ch32v003fun) project by Cnlohr and are also licensed under the terms of the MIT license. Please see the [CH32V003 LICENSE](src/platform/LICENSE) for a list of copyright holders.
