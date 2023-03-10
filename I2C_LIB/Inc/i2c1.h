#ifndef __I2C1_LIB_H__
#define __I2C1_LIB_H__
#include "stm32wbxx_hal.h"
#include "main.h"
#define I2C1_TIMEOUT 	1000
#define I2C1_BYTE_SIZE 	1

void I2C1_init(I2C_HandleTypeDef *handle_ptr);
I2C_HandleTypeDef* I2C1_getPtr(void);
uint8_t I2C1_Read8(uint8_t dev_addr, uint8_t reg_addr);
void I2C1_Write8(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_val);
uint8_t I2C1_ReadBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t buff[], uint8_t buff_size);
uint8_t I2C1_WriteBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t buff[], uint8_t buff_size);

#endif
