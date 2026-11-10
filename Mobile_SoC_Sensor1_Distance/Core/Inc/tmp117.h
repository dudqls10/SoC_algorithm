/*
 * tmp100.h
 *
 *  Created on: 2025. 8. 1.
 *      Author: Admin
 */

#ifndef INC_TMP117_H_
#define INC_TMP117_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "i2c.h"


/* Private Definition */
#define TMP117_Address                 	0x48
#define TMP117_W                       	(TMP117_Address << 1) + 1
#define TMP117_R                       	(TMP117_Address << 1) + 0
#define Temperature_Register			0x00
#define Configuration_Register			0x01
#define	Temp_Offset						0x07
#define Device_ID						0x0F
#define Continuous_conversion			0x0000
#define Conversion_3cycle				0x0181
#define Conversion_4cycle				0x0200
#define Average_Conversion_8			0x0020
#define Average_Conversion_32			0x0040


void TMP117_Init(I2C_HandleTypeDef* hi2c);
void TMP117_Write(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint16_t reg_data);
void TMP117_Read(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data);
#if 0
void TMP117_Read_DMA(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data);
#endif


#ifdef __cplusplus
}
#endif

#endif /* INC_TMP100_H_ */
