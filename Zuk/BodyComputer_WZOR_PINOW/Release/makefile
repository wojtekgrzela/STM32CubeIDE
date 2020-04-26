################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BodyComputer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BodyComputer.list \

OBJCOPY_BIN += \
BodyComputer.bin \


# All Target
all: BodyComputer.elf secondary-outputs

# Tool invocations
BodyComputer.elf: $(OBJS) $(USER_OBJS) C:\Users\bystr\Documents\STM32CubeIDE\Zuk\BodyComputer\STM32F103VETX_FLASH.ld
	arm-none-eabi-gcc -o "BodyComputer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\bystr\Documents\STM32CubeIDE\Zuk\BodyComputer\STM32F103VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BodyComputer.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BodyComputer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BodyComputer.list"
	@echo 'Finished building: $@'
	@echo ' '

BodyComputer.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BodyComputer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
