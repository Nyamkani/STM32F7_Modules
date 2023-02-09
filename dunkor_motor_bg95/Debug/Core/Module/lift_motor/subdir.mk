################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Module/lift_motor/bg95.cpp \
../Core/Module/lift_motor/bg95_command.cpp 

OBJS += \
./Core/Module/lift_motor/bg95.o \
./Core/Module/lift_motor/bg95_command.o 

CPP_DEPS += \
./Core/Module/lift_motor/bg95.d \
./Core/Module/lift_motor/bg95_command.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Module/lift_motor/%.o Core/Module/lift_motor/%.su: ../Core/Module/lift_motor/%.cpp Core/Module/lift_motor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"/home/studio3s/STM32CubeIDE/workspace_1.11.0/dunkor_motor_bg95/Core/Module" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Module-2f-lift_motor

clean-Core-2f-Module-2f-lift_motor:
	-$(RM) ./Core/Module/lift_motor/bg95.d ./Core/Module/lift_motor/bg95.o ./Core/Module/lift_motor/bg95.su ./Core/Module/lift_motor/bg95_command.d ./Core/Module/lift_motor/bg95_command.o ./Core/Module/lift_motor/bg95_command.su

.PHONY: clean-Core-2f-Module-2f-lift_motor

