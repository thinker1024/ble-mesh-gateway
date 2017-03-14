################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/ble_master/main.c \
../vendor/ble_master/rc_keyboard.c \
../vendor/ble_master/rc_master.c \
../vendor/ble_master/rc_mouse.c 

OBJS += \
./vendor/ble_master/main.o \
./vendor/ble_master/rc_keyboard.o \
./vendor/ble_master/rc_master.o \
./vendor/ble_master/rc_mouse.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/ble_master/%.o: ../vendor/ble_master/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_BLE_MASTER__=1 -DMASTER_DONGLE_TYPE_SEL=1 -DDEBUG_USB_MODE=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


