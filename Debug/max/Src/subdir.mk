################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../max/Src/max30102.c 

OBJS += \
./max/Src/max30102.o 

C_DEPS += \
./max/Src/max30102.d 


# Each subdirectory must supply rules for building sources it contributes
max/Src/%.o max/Src/%.su: ../max/Src/%.c max/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/MONSTER/STM32CubeIDE/workspace_1.11.0/maxDeneme/max/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-max-2f-Src

clean-max-2f-Src:
	-$(RM) ./max/Src/max30102.d ./max/Src/max30102.o ./max/Src/max30102.su

.PHONY: clean-max-2f-Src

