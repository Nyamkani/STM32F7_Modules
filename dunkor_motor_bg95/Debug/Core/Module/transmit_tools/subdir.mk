################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Module/transmit_tools/transmit_tools.cpp 

OBJS += \
./Core/Module/transmit_tools/transmit_tools.o 

CPP_DEPS += \
./Core/Module/transmit_tools/transmit_tools.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Module/transmit_tools/%.o Core/Module/transmit_tools/%.su: ../Core/Module/transmit_tools/%.cpp Core/Module/transmit_tools/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"/home/studio3s/STM32CubeIDE/workspace_1.11.0/dunkor_motor_bg95/Core/Module" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Module-2f-transmit_tools

clean-Core-2f-Module-2f-transmit_tools:
	-$(RM) ./Core/Module/transmit_tools/transmit_tools.d ./Core/Module/transmit_tools/transmit_tools.o ./Core/Module/transmit_tools/transmit_tools.su

.PHONY: clean-Core-2f-Module-2f-transmit_tools

