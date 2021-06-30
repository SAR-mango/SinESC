################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Application/Startup/startup_stm32g431cbux.s 

OBJS += \
./Application/Startup/startup_stm32g431cbux.o 

S_DEPS += \
./Application/Startup/startup_stm32g431cbux.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Startup/startup_stm32g431cbux.o: ../Application/Startup/startup_stm32g431cbux.s Application/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Application/Startup/startup_stm32g431cbux.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

