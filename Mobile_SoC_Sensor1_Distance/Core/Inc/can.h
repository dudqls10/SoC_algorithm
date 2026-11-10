/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_can.h"

extern CAN_HandleTypeDef hcan1;

extern CAN_FilterTypeDef canFilter1;
extern CAN_RxHeaderTypeDef can1RxHeader;
extern CAN_TxHeaderTypeDef can1TxHeader;
extern uint8_t can1Rx0Data[8];
extern uint32_t TxMailBox1;
extern uint8_t can1Tx0Data[8];

extern volatile uint8_t can1_rx0_flag;
extern volatile uint32_t lastCanError;
extern volatile uint32_t can_tx_fail_count;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Filter_Init(void);
HAL_StatusTypeDef CAN_Send(uint8_t ID, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3,
              uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);
void CAN_pSend(uint8_t ID, uint8_t* data_msb, uint8_t* data_lsb);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

