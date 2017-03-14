################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../proj/drivers/adc_8263.c \
../proj/drivers/adc_8266.c \
../proj/drivers/adc_8267.c \
../proj/drivers/airmouse.c \
../proj/drivers/airmouse_cali.c \
../proj/drivers/audio.c \
../proj/drivers/battery.c \
../proj/drivers/dfifo.c \
../proj/drivers/eth_hw.c \
../proj/drivers/ethernet.c \
../proj/drivers/flash.c \
../proj/drivers/i2c.c \
../proj/drivers/id.c \
../proj/drivers/ip_adapt.c \
../proj/drivers/keyboard.c \
../proj/drivers/mic.c \
../proj/drivers/pwm.c \
../proj/drivers/spi.c \
../proj/drivers/syshw.c \
../proj/drivers/uart.c \
../proj/drivers/usb.c \
../proj/drivers/usbdesc.c \
../proj/drivers/usbhw.c 

OBJS += \
./proj/drivers/adc_8263.o \
./proj/drivers/adc_8266.o \
./proj/drivers/adc_8267.o \
./proj/drivers/airmouse.o \
./proj/drivers/airmouse_cali.o \
./proj/drivers/audio.o \
./proj/drivers/battery.o \
./proj/drivers/dfifo.o \
./proj/drivers/eth_hw.o \
./proj/drivers/ethernet.o \
./proj/drivers/flash.o \
./proj/drivers/i2c.o \
./proj/drivers/id.o \
./proj/drivers/ip_adapt.o \
./proj/drivers/keyboard.o \
./proj/drivers/mic.o \
./proj/drivers/pwm.o \
./proj/drivers/spi.o \
./proj/drivers/syshw.o \
./proj/drivers/uart.o \
./proj/drivers/usb.o \
./proj/drivers/usbdesc.o \
./proj/drivers/usbhw.o 


# Each subdirectory must supply rules for building sources it contributes
proj/drivers/%.o: ../proj/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_LIGHT_8267_UART__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


