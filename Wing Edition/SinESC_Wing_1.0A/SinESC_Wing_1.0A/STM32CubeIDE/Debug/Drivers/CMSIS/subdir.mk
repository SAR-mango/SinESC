################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/esamp/Documents/SinESC/Wing\ Edition/SinESC_Wing_1.0A/SinESC_Wing_1.0A/Src/system_stm32f3xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f3xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f3xx.o: C:/Users/esamp/Documents/SinESC/Wing\ Edition/SinESC_Wing_1.0A/SinESC_Wing_1.0A/Src/system_stm32f3xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DESC_BEEP_FEATURE -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.4/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.4/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.4/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../Inc -I../../MCSDK_v5.4.4/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/system_stm32f3xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

