# Build target
TARGET:=main
TARGET_MCU:=CH32V203
TARGET_MCU_PACKAGE:=CH32V203C8T6

ADDITIONAL_C_FILES+=i2c_master.c rtc.c pmic.c keyboard.c

# SDK
PREFIX := riscv64-elf
NEWLIB:=/usr/riscv64-elf/include
CH32V003FUN := ch32v003fun/ch32v003fun
include ch32v003fun/ch32v003fun/ch32v003fun.mk

# Commands
.PHONY: all
all: prepare build flash

.PHONY: prepare
prepare:
	git submodule update --init --recursive

.PHONY: flash
flash: cv_flash
	#$(MINICHLINK)/minichlink -D

.PHONY: clean
clean: cv_clean

.PHONY: bear
bear: clean
bear:
	bear -- $(MAKE) build
