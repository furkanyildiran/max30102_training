#include "i2c1.h"

static I2C_HandleTypeDef *hi2c1_ptr;
static uint8_t reading_buf;

void I2C1_init(I2C_HandleTypeDef *handle_ptr){
	hi2c1_ptr = handle_ptr;
}
I2C_HandleTypeDef* I2C1_getPtr(void){
	return hi2c1_ptr;
}
uint8_t I2C1_Read8(uint8_t dev_addr, uint8_t reg_addr){
	HAL_I2C_Mem_Read(hi2c1_ptr, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &reading_buf, I2C1_BYTE_SIZE, I2C1_TIMEOUT);
	return reading_buf;
}
void I2C1_Write8(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_val){
	HAL_I2C_Mem_Write(hi2c1_ptr, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_val, I2C1_BYTE_SIZE, I2C1_TIMEOUT);
}
uint8_t I2C1_ReadBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t buff[], uint8_t buff_size){
	return HAL_I2C_Mem_Read(hi2c1_ptr, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, buff_size, I2C1_TIMEOUT);
}
uint8_t I2C1_WriteBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t buff[], uint8_t buff_size){
	return HAL_I2C_Mem_Write(hi2c1_ptr, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, buff_size, I2C1_TIMEOUT);
}

