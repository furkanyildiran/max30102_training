################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../I2C_LIB/Src/i2c1.c 

OBJS += \
./I2C_LIB/Src/i2c1.o 

C_DEPS += \
./I2C_LIB/Src/i2c1.d 


# Each subdirectory must supply rules for building sources it contributes
I2C_LIB/Src/%.o I2C_LIB/Src/%.su I2C_LIB/Src/%.cyclo: ../I2C_LIB/Src/%.c I2C_LIB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/MONSTER/Desktop/max30102_training/max/Inc" -I"C:/Users/MONSTER/Desktop/max30102_training/I2C_LIB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-I2C_LIB-2f-Src

clean-I2C_LIB-2f-Src:
	-$(RM) ./I2C_LIB/Src/i2c1.cyclo ./I2C_LIB/Src/i2c1.d ./I2C_LIB/Src/i2c1.o ./I2C_LIB/Src/i2c1.su

.PHONY: clean-I2C_LIB-2f-Src

