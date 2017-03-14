################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/8266_mesh_config/main.c \
../vendor/8266_mesh_config/mesh_config.c \
../vendor/8266_mesh_config/vendor.c \
../vendor/8266_mesh_config/vendor_att.c 

OBJS += \
./vendor/8266_mesh_config/main.o \
./vendor/8266_mesh_config/mesh_config.o \
./vendor/8266_mesh_config/vendor.o \
./vendor/8266_mesh_config/vendor_att.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/8266_mesh_config/%.o: ../vendor/8266_mesh_config/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -D__PROJECT_8266_MESH_CONFIG__=1 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


