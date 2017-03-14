################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../proj/ext_drv/mpu6050.c 

OBJS += \
./proj/ext_drv/mpu6050.o 


# Each subdirectory must supply rules for building sources it contributes
proj/ext_drv/%.o: ../proj/ext_drv/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_BLE_MASTER__=1 -DMASTER_DONGLE_TYPE_SEL=1 -DDEBUG_USB_MODE=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


