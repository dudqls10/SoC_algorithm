/*
 * tmp117.c
 *
 *  Created on: 2025. 8. 1.
 *      Author: Admin
 */

#include "tmp117.h"

/* Private Variables */
extern uint8_t data1_tmp[2];
extern uint8_t data2_tmp[2];


void TMP117_Init(I2C_HandleTypeDef* hi2c)
{
	TMP117_Write(hi2c, Configuration_Register, (Continuous_conversion|Conversion_3cycle|Average_Conversion_8));
}

void TMP117_Write(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint16_t reg_data)
{
	uint8_t temp_data[2];
	temp_data[0] = (uint8_t)(reg_data >> 8);
	temp_data[1] = (uint8_t)(reg_data & 0x00FF);

	HAL_I2C_Mem_Write(hi2c, TMP117_W, reg_addr, 1, &temp_data[0], 2, 1);
}

void TMP117_Read(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(hi2c, TMP117_R, reg_addr, 1, &data[0], 2, 1);
}

#if 0
void TMP117_Read_DMA(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read_DMA(hi2c, TMP117_R, reg_addr, 1, &data[0], 2);
}
#endif
