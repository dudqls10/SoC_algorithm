/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
// CAN1 관련 전역 변수 정의
CAN_FilterTypeDef canFilter1;
CAN_RxHeaderTypeDef can1RxHeader;
CAN_TxHeaderTypeDef can1TxHeader;
uint8_t can1Rx0Data[8];
uint32_t TxMailBox1;
uint8_t can1Tx0Data[8];

/* 수신 인터럽트 플래그 초기화 */
volatile uint8_t can1_rx0_flag = 0;

volatile uint32_t lastCanError;
volatile uint32_t can_tx_fail_count = 0U;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(void)
{
    /* ID 마스크와 필터를 설정하여 수신 메시지를 허용한다. */
    canFilter1.FilterMaskIdHigh = 0x7F3 << 5; // 5비트 이동
    canFilter1.FilterIdHigh     = 0x106 << 5;
    canFilter1.FilterMaskIdLow  = 0x7F3 << 5;
    canFilter1.FilterIdLow      = 0x106 << 5;
    canFilter1.FilterMode       = CAN_FILTERMODE_IDMASK;
    canFilter1.FilterScale      = CAN_FILTERSCALE_16BIT;
    canFilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilter1.FilterBank       = 0;
    canFilter1.FilterActivation = ENABLE;

    /* 송신 헤더 설정: 표준 ID, 데이터 프레임, DLC=8 */
    can1TxHeader.RTR = CAN_RTR_DATA;
    can1TxHeader.IDE = CAN_ID_STD;
    can1TxHeader.DLC = 8;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter1) != HAL_OK)
    {
        lastCanError = HAL_CAN_GetError(&hcan1);
        return;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        lastCanError = HAL_CAN_GetError(&hcan1);
        return;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        lastCanError = HAL_CAN_GetError(&hcan1);
    }

}

HAL_StatusTypeDef CAN_Send(uint8_t ID, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3,
                            uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        can_tx_fail_count++;
        return HAL_BUSY;
    }
    can1TxHeader.StdId = ID;
    can1Tx0Data[0] = data0;
    can1Tx0Data[1] = data1;
    can1Tx0Data[2] = data2;
    can1Tx0Data[3] = data3;
    can1Tx0Data[4] = data4;
    can1Tx0Data[5] = data5;
    can1Tx0Data[6] = data6;
    can1Tx0Data[7] = data7;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
            &hcan1, &can1TxHeader, can1Tx0Data, &TxMailBox1);
    if (status != HAL_OK)
    {
        can_tx_fail_count++;
        lastCanError = HAL_CAN_GetError(&hcan1);
    }
    return status;
}

void CAN_pSend(uint8_t ID, uint8_t* data_msb, uint8_t* data_lsb)
{
    /* 메시지 ID 설정 */
    can1TxHeader.StdId = ID;

    /* 상하위 바이트를 전송 버퍼에 채운다. */
    can1Tx0Data[7] = data_lsb[3];
    can1Tx0Data[6] = data_lsb[2];
    can1Tx0Data[5] = data_lsb[1];
    can1Tx0Data[4] = data_lsb[0];
    can1Tx0Data[3] = data_msb[3];
    can1Tx0Data[2] = data_msb[2];
    can1Tx0Data[1] = data_msb[1];
    can1Tx0Data[0] = data_msb[0];

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        lastCanError = HAL_CAN_ERROR_NOT_INITIALIZED;
        return;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, &can1Tx0Data[0], &TxMailBox1) != HAL_OK)
    {
        lastCanError = HAL_CAN_GetError(&hcan1);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, &can1Rx0Data[0]);
        can1_rx0_flag = 1;
    }
}
/* USER CODE END 1 */
