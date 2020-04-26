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
Sig_Gen_BLACK.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Sig_Gen_BLACK.list \


# All Target
all: Sig_Gen_BLACK.elf secondary-outputs

# Tool invocations
Sig_Gen_BLACK.elf: $(OBJS) $(USER_OBJS) C:\Users\bystr\Documents\STM32CubeIDE\Dla_Andrzeja\Sig_Gen_BLACK\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Sig_Gen_BLACK.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\bystr\Documents\STM32CubeIDE\Dla_Andrzeja\Sig_Gen_BLACK\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Sig_Gen_BLACK.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Sig_Gen_BLACK.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Sig_Gen_BLACK.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
