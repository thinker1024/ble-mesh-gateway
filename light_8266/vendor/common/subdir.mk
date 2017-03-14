################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/common/app_att_light.c \
../vendor/common/common.c \
../vendor/common/crc.c \
../vendor/common/cust_irq.c \
../vendor/common/device_led.c \
../vendor/common/device_power.c \
../vendor/common/emi.c \
../vendor/common/factory_reset.c \
../vendor/common/pm_test.c \
../vendor/common/rtc.c \
../vendor/common/scene.c \
../vendor/common/tl_audio.c 

OBJS += \
./vendor/common/app_att_light.o \
./vendor/common/common.o \
./vendor/common/crc.o \
./vendor/common/cust_irq.o \
./vendor/common/device_led.o \
./vendor/common/device_power.o \
./vendor/common/emi.o \
./vendor/common/factory_reset.o \
./vendor/common/pm_test.o \
./vendor/common/rtc.o \
./vendor/common/scene.o \
./vendor/common/tl_audio.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/common/%.o: ../vendor/common/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_LIGHT_8266__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


