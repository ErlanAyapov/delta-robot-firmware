SHELL := cmd
.SHELLFLAGS := /C

.DEFAULT_GOAL := cycle

PROJECT := TestChannels
TARGET := $(PROJECT)

ROOT_DIR := $(subst /,\,$(CURDIR))
MDK_DIR := $(ROOT_DIR)\MDK-ARM
UVPROJ := $(MDK_DIR)\$(PROJECT).uvprojx
BUILD_DIR := $(MDK_DIR)\$(PROJECT)
HEX_PATH := $(BUILD_DIR)\$(PROJECT).hex
BUILD_LOG := $(BUILD_DIR)\build.log

UV4 ?= C:\Users\wwwai\AppData\Local\Arm\Keil_v5\UV4\UV4.exe
STM32_PROG_CLI ?= C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe
STLINK_CLI ?= C:\Program Files (x86)\STMicroelectronics\ST-LINK Utility\ST-LINK_CLI.exe

.PHONY: cycle help build flash erase clean

cycle: clean flash

help:
	@echo Targets:
	@echo   make        - Clean + build + flash ^(default^)
	@echo   make build  - Build firmware ^($(PROJECT)^) via Keil UV4
	@echo   make flash  - Build + flash, or flash existing HEX if UV4 is unavailable
	@echo   make erase  - Mass erase flash
	@echo   make clean  - Remove build artifacts from $(BUILD_DIR)
	@echo.
	@echo Optional overrides:
	@echo   make flash UV4="C:\Keil_v5\UV4\UV4.exe"
	@echo   make flash STM32_PROG_CLI="C:\...\STM32_Programmer_CLI.exe"
	@echo   make flash STLINK_CLI="C:\...\ST-LINK_CLI.exe"

build:
	@if not exist "$(UV4)" (echo [error] UV4 not found: "$(UV4)" & exit /b 1)
	@if not exist "$(UVPROJ)" (echo [error] uVision project not found: "$(UVPROJ)" & exit /b 1)
	@echo [build] $(PROJECT)
	@"$(UV4)" -j0 -b "$(UVPROJ)" -t "$(TARGET)" -o "$(BUILD_LOG)"
	@if errorlevel 1 (echo [error] Build failed & exit /b 1)
	@if not exist "$(HEX_PATH)" (echo [error] HEX not generated: "$(HEX_PATH)" & exit /b 1)
	@echo [build] OK -^> "$(HEX_PATH)"

flash:
	@if exist "$(UV4)" ($(MAKE) --no-print-directory build) else (if exist "$(HEX_PATH)" (echo [flash] UV4 not found, using existing HEX: "$(HEX_PATH)") else (echo [error] UV4 not found and HEX missing: "$(HEX_PATH)" & exit /b 1))
	@if errorlevel 1 exit /b 1
	@if exist "$(STM32_PROG_CLI)" (echo [flash] STM32_Programmer_CLI && "$(STM32_PROG_CLI)" -c port=SWD mode=UR -w "$(HEX_PATH)" -v -rst) else (if exist "$(STLINK_CLI)" (echo [flash] ST-LINK_CLI && "$(STLINK_CLI)" -c SWD UR -P "$(HEX_PATH)" 0x08000000 -V -Rst) else (echo [error] No flash tool found. Set STM32_PROG_CLI or STLINK_CLI. & exit /b 1))
	@if errorlevel 1 (echo [error] Flash failed & exit /b 1)
	@echo [flash] Done

erase:
	@if exist "$(STM32_PROG_CLI)" (echo [erase] STM32_Programmer_CLI && "$(STM32_PROG_CLI)" -c port=SWD mode=UR -e all) else (if exist "$(STLINK_CLI)" (echo [erase] ST-LINK_CLI && "$(STLINK_CLI)" -c SWD UR -ME) else (echo [error] No flash tool found. Set STM32_PROG_CLI or STLINK_CLI. & exit /b 1))
	@if errorlevel 1 (echo [error] Erase failed & exit /b 1)
	@echo [erase] Done

clean:
	@if exist "$(BUILD_DIR)\*.axf" del /q "$(BUILD_DIR)\*.axf"
	@if exist "$(BUILD_DIR)\*.hex" del /q "$(BUILD_DIR)\*.hex"
	@if exist "$(BUILD_DIR)\*.map" del /q "$(BUILD_DIR)\*.map"
	@if exist "$(BUILD_DIR)\*.o" del /q "$(BUILD_DIR)\*.o"
	@if exist "$(BUILD_DIR)\*.d" del /q "$(BUILD_DIR)\*.d"
	@if exist "$(BUILD_DIR)\*.crf" del /q "$(BUILD_DIR)\*.crf"
	@if exist "$(BUILD_DIR)\*.dep" del /q "$(BUILD_DIR)\*.dep"
	@if exist "$(BUILD_DIR)\*.htm" del /q "$(BUILD_DIR)\*.htm"
	@if exist "$(BUILD_DIR)\*.lnp" del /q "$(BUILD_DIR)\*.lnp"
	@if exist "$(BUILD_LOG)" del /q "$(BUILD_LOG)"
	@echo [clean] Done
