#ifndef MOBILE_SOC_EVALUATION_CLOCK_H
#define MOBILE_SOC_EVALUATION_CLOCK_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* TIM2 is unused by the project and needs no pin or interrupt. Unlike DWT,
 * this independent 1 MHz counter is not configured by a debug connection.
 * Sensor TIM7, CAN TIM6, and their interrupt rates remain unchanged. */
static inline void EvaluationClockInit(void)
{
    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) clock_hz *= 2U;
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    TIM2->PSC = clock_hz / 1000000U - 1U;
    TIM2->ARR = UINT32_MAX;
    TIM2->DIER = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CNT = 0U;
    TIM2->CR1 = TIM_CR1_CEN;
}

static inline uint32_t EvaluationNowUs(void)
{
    return TIM2->CNT;
}

#endif
