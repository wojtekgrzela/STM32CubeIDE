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
Tempomat.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Tempomat.list \

OBJCOPY_BIN += \
Tempomat.bin \


# All Target
all: Tempomat.elf secondary-outputs

# Tool invocations
Tempomat.elf: $(OBJS) $(USER_OBJS) C:\Users\Dell\Documents\STM32CubeIDE\Zuk\Tempomat\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Tempomat.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Dell\Documents\STM32CubeIDE\Zuk\Tempomat\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Tempomat.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Tempomat.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Tempomat.list"
	@echo 'Finished building: $@'
	@echo ' '

Tempomat.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Tempomat.bin"
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
