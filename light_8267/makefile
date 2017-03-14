################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include vendor/light_8267/subdir.mk
-include vendor/common/subdir.mk
-include proj/mcu_spec/subdir.mk
-include proj/mcu/subdir.mk
-include proj/ext_drv/subdir.mk
-include proj/drivers/subdir.mk
-include proj/common/subdir.mk
-include proj/app/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
light_8267.lst \

FLASH_IMAGE += \
light_8267.bin \

SIZEDUMMY += \
sizedummy \


# All Target
all: pre-build main-build

# Main-build Target
main-build: ble_mesh_light.elf secondary-outputs

# Tool invocations
ble_mesh_light.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 C Linker'
	tc32-elf-ld --gc-sections -L"Y:\ble_mesh_release\sdk\proj_lib" -T ../boot.link -o"ble_mesh_light.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

light_8267.lst: ble_mesh_light.elf
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S ble_mesh_light.elf  >"light_8267.lst"
	@echo 'Finished building: $@'
	@echo ' '

light_8267.bin: ble_mesh_light.elf
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy -O binary ble_mesh_light.elf  "light_8267.bin"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: ble_mesh_light.elf
	@echo 'Invoking: Print Size'
	tc32-elf-size -t ble_mesh_light.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(LST)$(SIZEDUMMY) ble_mesh_light.elf
	-@echo ' '

pre-build:
	-"Y:\ble_mesh_release\sdk/getver.sh"
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY: main-build pre-build

-include ../makefile.targets
