################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/light_8267_uart/main.c \
../vendor/light_8267_uart/main_light.c \
../vendor/light_8267_uart/vendor_att_light.c \
../vendor/light_8267_uart/vendor_light.c 

OBJS += \
./vendor/light_8267_uart/main.o \
./vendor/light_8267_uart/main_light.o \
./vendor/light_8267_uart/vendor_att_light.o \
./vendor/light_8267_uart/vendor_light.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/light_8267_uart/%.o: ../vendor/light_8267_uart/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_LIGHT_8267_UART__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


