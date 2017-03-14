################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/light_switch/light_switch.c \
../vendor/light_switch/main.c \
../vendor/light_switch/vendor_att_switch.c \
../vendor/light_switch/vendor_switch.c 

OBJS += \
./vendor/light_switch/light_switch.o \
./vendor/light_switch/main.o \
./vendor/light_switch/vendor_att_switch.o \
./vendor/light_switch/vendor_switch.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/light_switch/%.o: ../vendor/light_switch/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_LIGHT_SWITCH__=1 -D__PROJECT_CHIP_TYPE_SEL__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


