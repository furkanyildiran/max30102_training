################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../max/Src/heartRate.c \
../max/Src/max30102.c \
../max/Src/spo2_algorithm.c 

OBJS += \
./max/Src/heartRate.o \
./max/Src/max30102.o \
./max/Src/spo2_algorithm.o 

C_DEPS += \
./max/Src/heartRate.d \
./max/Src/max30102.d \
./max/Src/spo2_algorithm.d 


# Each subdirectory must supply rules for building sources it contributes
max/Src/%.o max/Src/%.su max/Src/%.cyclo: ../max/Src/%.c max/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/MONSTER/Desktop/a/max30102_training/max/Inc" -I"C:/Users/MONSTER/Desktop/a/max30102_training/I2C_LIB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-max-2f-Src

clean-max-2f-Src:
	-$(RM) ./max/Src/heartRate.cyclo ./max/Src/heartRate.d ./max/Src/heartRate.o ./max/Src/heartRate.su ./max/Src/max30102.cyclo ./max/Src/max30102.d ./max/Src/max30102.o ./max/Src/max30102.su ./max/Src/spo2_algorithm.cyclo ./max/Src/spo2_algorithm.d ./max/Src/spo2_algorithm.o ./max/Src/spo2_algorithm.su

.PHONY: clean-max-2f-Src

