// Tanmatsu coprocessor firmware
// SPDX-FileCopyrightText: 2024 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

void keyboard_init(void);
bool keyboard_step(volatile uint8_t* registers);
