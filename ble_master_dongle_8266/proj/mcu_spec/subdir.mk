################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../proj/mcu_spec/gpio_8263.c \
../proj/mcu_spec/gpio_8266.c \
../proj/mcu_spec/gpio_8267.c \
../proj/mcu_spec/gpio_8366.c 

S_UPPER_SRCS += \
../proj/mcu_spec/cstartup_8263.S \
../proj/mcu_spec/cstartup_8263_ram.S \
../proj/mcu_spec/cstartup_8266.S \
../proj/mcu_spec/cstartup_8266_ram.S \
../proj/mcu_spec/cstartup_8267.S \
../proj/mcu_spec/cstartup_8267_ram.S \
../proj/mcu_spec/cstartup_8366.S \
../proj/mcu_spec/cstartup_8366_ram.S 

OBJS += \
./proj/mcu_spec/cstartup_8263.o \
./proj/mcu_spec/cstartup_8263_ram.o \
./proj/mcu_spec/cstartup_8266.o \
./proj/mcu_spec/cstartup_8266_ram.o \
./proj/mcu_spec/cstartup_8267.o \
./proj/mcu_spec/cstartup_8267_ram.o \
./proj/mcu_spec/cstartup_8366.o \
./proj/mcu_spec/cstartup_8366_ram.o \
./proj/mcu_spec/gpio_8263.o \
./proj/mcu_spec/gpio_8266.o \
./proj/mcu_spec/gpio_8267.o \
./proj/mcu_spec/gpio_8366.o 


# Each subdirectory must supply rules for building sources it contributes
proj/mcu_spec/%.o: ../proj/mcu_spec/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 CC/Assembler'
	tc32-elf-gcc -DMCU_CORE_8266 -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

proj/mcu_spec/%.o: ../proj/mcu_spec/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_BLE_MASTER__=1 -DMASTER_DONGLE_TYPE_SEL=1 -DDEBUG_USB_MODE=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


