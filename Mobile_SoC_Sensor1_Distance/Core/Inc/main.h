/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/* TIMx Counter Period according to Frequency */
#define Freq0f2Hz		500000-1
#define Freq0f5Hz		200000-1
#define Freq1Hz			100000-1
#define Freq5Hz			20000-1
#define Freq10Hz		10000-1
#define Freq20Hz		5000-1
#define Freq25Hz		4000-1
#define Freq40Hz		2500-1
#define Freq50Hz		2000-1
#define Freq80Hz		1250-1
#define Freq100Hz		1000-1
#define Freq125Hz		800-1
#define Freq160Hz		625-1
#define Freq200Hz		500-1
#define Freq250Hz		400-1
#define Freq300Hz		333-1
#define Freq400Hz		250-1
#define Freq500Hz		200-1
#define	Freq1000Hz		100-1

typedef struct
{
    /* Transmitting CAN Signal */
    uint8_t canTransmit;

    /* CAN Protocol */
    uint8_t onOff;
    uint8_t onOff_ID;
    uint8_t onOff_Function;
    uint8_t onOff_sensing;
    uint8_t onOff_reset;
    uint8_t onOff_cftool;
    uint8_t onOff_rawData;

    uint8_t conversionTimeControl;

	/* FDC */
	volatile uint8_t idle_fdcQ1;
	volatile uint8_t idle_fdcQ2;
	uint8_t movingObject_fdc;
	uint8_t stationaryObject_fdc;

} states;


typedef struct
{
	// ch000 = ch0, ch100 = ch1
	uint8_t ovf_ch0;
	uint8_t ovf_ch1;
	uint8_t status_reg;
	uint8_t ch0_valid;
	uint8_t ch1_valid;
	uint8_t ch0_sequence;
	uint8_t ch0_reject_reason;
	uint8_t ch1_reject_reason;
	uint8_t ch0_filter_seeded;
	uint8_t ch1_filter_seeded;
	HAL_StatusTypeDef ch0_i2c_status;
	HAL_StatusTypeDef ch1_i2c_status;
	uint8_t config_reg;
	uint8_t conv_reg_ch0;
	uint8_t conv_reg_ch1;
	HAL_StatusTypeDef config_i2c_status;

	// ch300 sensing, ch302 compensation
	uint8_t temp_ch300[4];
	uint8_t temp_ch302[4];
	uint8_t temp_ch100[4];
	uint8_t temp_ch102[4];

	uint32_t ch300;
	uint32_t ch302;
	uint32_t ch100;
	uint32_t ch102;
	uint32_t raw_ch0_sample;
	uint32_t raw_ch1_sample;
	uint32_t count_ch0_sample_milli;
	uint32_t count_ch1_sample_milli;
	uint32_t frequency_ch0_sample_hz;
	uint32_t frequency_ch1_sample_hz;
	uint32_t raw_ch0;
	uint32_t raw_ch1;
	uint32_t frequency_ch0_hz;
	uint32_t frequency_ch1_hz;
	uint32_t ch0_rejected_samples;
	uint32_t ch1_rejected_samples;
	uint32_t ch0_saturation_samples;
	uint32_t ch1_saturation_samples;

}NEXTLAB_fdc;


typedef struct
{
	uint8_t temp_Q1[4];
	uint8_t temp_Q2[4];

	uint32_t Q1;
	uint32_t Q2;
	uint32_t Normalized_Q1;

}distance;


/* Variables for Drift Compensation Algorithm */
typedef struct
{
	uint32_t fdc_Q1;
	uint32_t fdc_Q2;

	int32_t drift_fdcQ1;
	int32_t drift_fdcQ2;

	uint32_t Vfdc_Q1;
	uint32_t Vfdc_Q2;

	uint32_t Tfdc_Q1;
	int32_t Tfdc_tmp[2];

	uint8_t temp_fdcQ1[4];
	uint8_t temp_fdcQ2[4];

	int64_t fdcCoeff_Q1;
	int64_t fdcCoeff_Q2;
	int64_t tmp117_fdcCoeff_Q1;

	int32_t PMfdcCoeff_Q1;
	int32_t PMfdcCoeff_Q2;

	int64_t fdcOffset_Q1;
	int64_t fdcOffset_Q2;
	int64_t tmp117_fdcOffset_Q1;

	int64_t PMfdcOffset_Q1;
	int64_t PMfdcOffset_Q2;

	int64_t sumNumnerator_fdcQ1;
	int64_t sumNumnerator_fdcQ2;

	int64_t sumNumnerator_PMfdcQ1;
	int64_t sumNumnerator_PMfdcQ2;

	int64_t sumDenominator_fdcQ1;
	int64_t sumDenominator_fdcQ2;

	int64_t sumDenominator_PMfdcQ1;
	int64_t sumDenominator_PMfdcQ2;

	int64_t sumNumnerator_tmp117;
	int64_t sumDenominator_tmp117;

}driftCompensation;

typedef struct
{
	/* Nextlab FDC */

	uint32_t fdc300[100];
	uint32_t fdc302[100];
	uint32_t fdc100[100];
	uint32_t fdc102[100];

	uint32_t PMfdc300[50];
	uint32_t PMfdc302[50];
	uint32_t PMfdc100[50];
	uint32_t PMfdc102[50];

	uint64_t sum_fdc300;
	uint64_t sum_fdc302;
	uint64_t sum_fdc100;
	uint64_t sum_fdc102;

	uint64_t sum_PMfdc300;
	uint64_t sum_PMfdc302;
	uint64_t sum_PMfdc100;
	uint64_t sum_PMfdc102;

	uint64_t mean_fdc300;
	uint64_t mean_fdc302;
	uint64_t mean_fdc100;
	uint64_t mean_fdc102;

	uint64_t mean_PMfdc300;
	uint64_t mean_PMfdc302;
	uint64_t mean_PMfdc100;
	uint64_t mean_PMfdc102;

	int64_t err_fdc300[100];
	int64_t err_fdc302[100];
	int64_t err_fdc100[100];
	int64_t err_fdc102[100];

	int32_t err_PMfdc300[50];
	int32_t err_PMfdc302[50];
	int32_t err_PMfdc100[50];
	int32_t err_PMfdc102[50];


	/* Temperature */

	uint32_t tmp117[50];
	int32_t sum_tmp117;
	int32_t mean_tmp117;
	int32_t err_tmp117[50];

	uint32_t tmp117_f302[50];
	uint64_t sum_tmp117_f302;
	uint64_t mean_tmp117_f302;
	uint32_t err_tmp117_f302[50];


	uint32_t DI_fdcQ1[100];
	uint32_t DI_fdcQ2[100];
	uint32_t DI_tmp117[50];

	uint32_t DI_RfdcQ1[50];
	uint32_t DI_RfdcQ2[50];

	int64_t Derivative_fdcQ1[100];
	int64_t Derivative_fdcQ2[100];
	int64_t Derivative_tmp117[50];

	int32_t Derivative_RfdcQ1[50];
	int32_t Derivative_RfdcQ2[50];

	int32_t Integral_fdcQ1[20];
	int32_t Integral_fdcQ2[20];
	int32_t Integral_tmp117[20];

	int32_t Integral_RfdcQ1[20];
	int32_t Integral_RfdcQ2[20];

	int64_t sumIntegral_fdcQ1;
	int64_t sumIntegral_fdcQ2;
	int64_t sumIntegral_tmp117;

	int64_t sumIntegral_RfdcQ1;
	int64_t sumIntegral_RfdcQ2;

	int32_t DOinit_fdcQ1[100];
	int32_t DOinit_fdcQ2[100];

	uint64_t DO_fdcQ1[100];
	uint64_t DO_fdcQ2[100];

	int32_t DO_RfdcQ1[100];
	int32_t DO_RfdcQ2[100];

	uint64_t sumDO_fdcQ1;
	uint64_t sumDO_fdcQ2;

	uint32_t sumDO_RfdcQ1;
	uint32_t sumDO_RfdcQ2;

	uint64_t sumSTDDO_fdcQ1;
	uint64_t sumSTDDO_fdcQ2;

	uint64_t sumSTDDO_RfdcQ1;
	uint64_t sumSTDDO_RfdcQ2;

	int32_t meanDOinit_fdcQ1;
	int32_t meanDOinit_fdcQ2;

	int32_t meanDO_fdcQ1;
	int32_t meanDO_fdcQ2;

	int32_t meanDO_RfdcQ1;
	int32_t meanDO_RfdcQ2;

	uint32_t stdDOinit_fdcQ1;
	uint32_t stdDOinit_fdcQ2;

	uint32_t stdDO_fdcQ1;
	uint32_t stdDO_fdcQ2;

	uint32_t stdDO_RfdcQ1;
	uint32_t stdDO_RfdcQ2;

}samples;


/* Variables for Kalman Filter */
typedef struct
{
	uint32_t fdc300;
	uint32_t fdc302;
	uint32_t fdc100;
	uint32_t fdc102;

	uint8_t A_Q1;
	uint8_t A_Q1cmp;
	uint8_t A_Q2;
	uint8_t A_Q2cmp;

	float H_Q1;
	float H_Q1cmp;
	float H_Q2;
	float H_Q2cmp;

	float Q_Q1;
	float Q_Q1cmp;
	float Q_Q2;
	float Q_Q2cmp;

	float Pp_Q1;
	float Pp_Q1cmp;
	float Pp_Q2;
	float Pp_Q2cmp;

	float K_Q1;
	float K_Q1cmp;
	float K_Q2;
	float K_Q2cmp;

	float R_Q1;
	float R_Q1cmp;
	float R_Q2;
	float R_Q2cmp;

	float x_Q1;
	float x_Q1cmp;
	float x_Q2;
	float x_Q2cmp;

	uint32_t xp_Q1;
	uint32_t xp_Q1cmp;
	uint32_t xp_Q2;
	uint32_t xp_Q2cmp;

	float P_Q1;
	float P_Q1cmp;
	float P_Q2;
	float P_Q2cmp;

	uint32_t result_Q1;
	uint32_t result_Q1cmp;
	uint32_t result_Q2;
	uint32_t result_Q2cmp;

	uint8_t temp_resultQ1[4];
	uint8_t temp_resultQ1cmp[4];
	uint8_t temp_resultQ2[4];
	uint8_t temp_resultQ2cmp[4];

}kalmanFilter;

typedef struct
{
	uint32_t prev_Q1;
	uint32_t prev_Q1cmp;
	uint32_t prev_Q2;
	uint32_t prev_Q2cmp;
	uint8_t initialized_Q1;
	uint8_t initialized_Q1cmp;
	uint8_t initialized_Q2;
	uint8_t initialized_Q2cmp;
}slewRateLimiterState;


/* Variables for Temperature Sensor */
typedef struct
{
	uint8_t temp_data[2];
	uint32_t data;
	float temp_data_celsius;

}temperature117;


/* Variables for ToF */
typedef struct
{
	uint8_t address[4];
	uint8_t state[4];
	uint16_t distance[4];
	uint16_t distanceCAN[4];
	uint8_t flag[4];
	uint8_t temp;

}ToF_vl53l1x;


/* Variables for Curve Fitting */
typedef struct
{
	int32_t distancePredict_Q1;
	int32_t distancePredict_Q2;

	int32_t FilteredDistance_Q1;
	int32_t FilteredDistance_Q2;

	uint32_t RefDistance_Q1;
	uint32_t RefDistance_Q2;

	volatile uint32_t dataQ1_curr[30];
	volatile uint32_t dataQ1_prev[30];
	volatile uint32_t dataQ1_filtered[30];

	volatile uint32_t dataQ2_curr[30];
	volatile uint32_t dataQ2_prev[30];
	volatile uint32_t dataQ2_filtered[30];

	volatile int32_t MeanData_Q1[30];
	volatile int32_t MeanData_Q2[30];

	double Sum_lnY_Q1;
	double Sum_lnY_Q2;

	double lnY_Q1[30];
	double lnY_Q2[30];

	double Mean_lnY_Q1;
	double Mean_lnY_Q2;

	/* Reference Distance Data */
	uint16_t X[30];
	int32_t Mean_X;
	int32_t Sum_X;
	int32_t err_X[30];

	double err_lnY_Q1[30];
	double err_lnY_Q2[30];

	double sumNumnerator_Q1;
	double sumNumnerator_Q2;

	double sumDenominator_Q1;
	double sumDenominator_Q2;

	double Coeff_Q1;
	uint64_t tmpCoeff_Q1;
	double Coeff_Q2;
	uint64_t tmpCoeff_Q2;

	double Offset_Q1;
	double Offset_Q2;

	int32_t logY_Q1;
	int32_t logY_Q2;

}Curvefitting;

/* Variables for Sensor Fusion */
typedef struct
{
	uint32_t std_CapRaw_Q1;
	float std_CapDist_Q1;
	uint32_t std_CapDist_Q1_u6;
	float std_ToF_Q1;
	float FusedNumerator_Q1;
	float FusedDenominator_Q1;
	int32_t FusedDistance_Q1;
	uint8_t FusedDistance_Q1_valid;

	uint32_t mean_CapRaw_Q1;
	uint32_t prev_CapRaw_Q1;
	uint64_t temp_b_uc_Q1;

}SensorFusion;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
