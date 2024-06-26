# Build target
TARGET:=main
TARGET_MCU:=CH32V203
TARGET_MCU_PACKAGE:=CH32V203C8T6

#ADDITIONAL_C_FILES+=

# SDK
PREFIX := riscv64-elf
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

