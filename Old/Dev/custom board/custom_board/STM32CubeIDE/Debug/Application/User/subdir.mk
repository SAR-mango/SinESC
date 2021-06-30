################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/main.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_api.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_config.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_interface.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_math.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_parameters.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_tasks.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/motor_control_protocol.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/motorcontrol.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/regular_conversion_manager.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/stm32f30x_mc_it.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/stm32f3xx_hal_msp.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/stm32f3xx_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/ui_task.c \
C:/Users/esamp/Desktop/custom\ board/custom_board/Src/user_interface.c 

OBJS += \
./Application/User/main.o \
./Application/User/mc_api.o \
./Application/User/mc_config.o \
./Application/User/mc_interface.o \
./Application/User/mc_math.o \
./Application/User/mc_parameters.o \
./Application/User/mc_tasks.o \
./Application/User/motor_control_protocol.o \
./Application/User/motorcontrol.o \
./Application/User/regular_conversion_manager.o \
./Application/User/stm32f30x_mc_it.o \
./Application/User/stm32f3xx_hal_msp.o \
./Application/User/stm32f3xx_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/ui_task.o \
./Application/User/user_interface.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/mc_api.d \
./Application/User/mc_config.d \
./Application/User/mc_interface.d \
./Application/User/mc_math.d \
./Application/User/mc_parameters.d \
./Application/User/mc_tasks.d \
./Application/User/motor_control_protocol.d \
./Application/User/motorcontrol.d \
./Application/User/regular_conversion_manager.d \
./Application/User/stm32f30x_mc_it.d \
./Application/User/stm32f3xx_hal_msp.d \
./Application/User/stm32f3xx_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/ui_task.d \
./Application/User/user_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_api.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_api.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/mc_api.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/mc_config.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_interface.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/mc_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_math.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_math.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/mc_math.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_parameters.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_parameters.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/mc_parameters.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/mc_tasks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/mc_tasks.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motor_control_protocol.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/motor_control_protocol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/motor_control_protocol.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motorcontrol.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/motorcontrol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/motorcontrol.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/regular_conversion_manager.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/regular_conversion_manager.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/regular_conversion_manager.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f30x_mc_it.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/stm32f30x_mc_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stm32f30x_mc_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f3xx_hal_msp.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/stm32f3xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stm32f3xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f3xx_it.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/stm32f3xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stm32f3xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/syscalls.o: ../Application/User/syscalls.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/sysmem.o: ../Application/User/sysmem.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/ui_task.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/ui_task.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/ui_task.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/user_interface.o: C:/Users/esamp/Desktop/custom\ board/custom_board/Src/user_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DARM_MATH_CM4 -DDEBUG -c -I../../Drivers/CMSIS/Include -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../../Inc -I../../Drivers/STM32F3xx_HAL_Driver/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F3xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/user_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

