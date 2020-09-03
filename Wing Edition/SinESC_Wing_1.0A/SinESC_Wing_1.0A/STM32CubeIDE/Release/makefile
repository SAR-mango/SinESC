################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/MotorControl/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Application/User/subdir.mk
-include Application/Startup/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SinESC_Wing_1.0A.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SinESC_Wing_1.0A.list \

OBJCOPY_BIN += \
SinESC_Wing_1.0A.bin \


# All Target
all: SinESC_Wing_1.0A.elf secondary-outputs

# Tool invocations
SinESC_Wing_1.0A.elf: $(OBJS) $(USER_OBJS) C:\Users\esamp\Documents\SinESC\Wing\ Edition\SinESC_Wing_1.0A\SinESC_Wing_1.0A\STM32CubeIDE\STM32F303CBTX_FLASH.ld
	arm-none-eabi-gcc -o "SinESC_Wing_1.0A.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\esamp\Documents\SinESC\Wing Edition\SinESC_Wing_1.0A\SinESC_Wing_1.0A\STM32CubeIDE\STM32F303CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SinESC_Wing_1.0A.map" -Wl,--gc-sections -static -L../../MCSDK_v5.4.4/MotorControl/lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SinESC_Wing_1.0A.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SinESC_Wing_1.0A.list"
	@echo 'Finished building: $@'
	@echo ' '

SinESC_Wing_1.0A.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SinESC_Wing_1.0A.bin"
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
