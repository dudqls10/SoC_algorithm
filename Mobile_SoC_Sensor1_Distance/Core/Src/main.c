/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* ToF */
#define ToF							1

/* Expansion Accessories */
#define Expansion					0

/* Set up only one operation of them: 90/180/360(Degree) */
#define Operation_Single			1
#define Operation_Double			0
#define Operation_Quadruple			0

/* NEXTLab FDC setting */
#define FDC							1

#define FDC_ADD						0

#define ConversionTime_Control		0
/* Setting for frequency of conversion time control : 25/50/100/200/500(Hz) */
#define ConversionTimeControl		Freq100Hz

/* CAN Communication Setting */
#define CANCommunication			1
/* Setting for frequency of CAN transmission : 100/200/250/500/1000(Hz) */
#define CANTransmit					Freq300Hz

/* CAN diagnostic protocol signature: ASCII "STM", version 2. */
#define CAN_DIAG_SIGNATURE_S			0x53U
#define CAN_DIAG_SIGNATURE_T			0x54U
#define CAN_DIAG_SIGNATURE_M			0x4DU
#define CAN_DIAG_VERSION				0x02U

/* CAN Communication Channel */
/* CNA_CH0 is the representative CAN ID of the sensor */
#define Standard_Sensor1					1
#define Standard_Sensor2					2
#define Standard_Sensor3					3
/* Select CAN Channel According to the Defines Above */
/* Standard_Sensor1-3 */
#define CanChannelVersion 	Standard_Sensor1


#if (CanChannelVersion == Standard_Sensor1)
#define CAN_CH0								0x41		// 1st
#define CAN_CH1								CAN_CH0+1	// 2nd
#define CAN_CH2								CAN_CH0+2	// 3rd
#define CAN_CH3								CAN_CH0+3	// 4th
#define CAN_CH4								CAN_CH0+4	// 5th
#define CAN_CH5								CAN_CH0+5	// 6th

#elif (CanChannelVersion == Standard_Sensor2)
#define CAN_CH0								0x51		// 1st
#define CAN_CH1								CAN_CH0+1	// 2nd
#define CAN_CH2								CAN_CH0+2	// 3rd
#define CAN_CH3								CAN_CH0+3	// 4th
#define CAN_CH4								CAN_CH0+4	// 5th
#define CAN_CH5								CAN_CH0+5	// 6th

#elif (CanChannelVersion == Standard_Sensor3)
#define CAN_CH0								0x61		// 1st
#define CAN_CH1								CAN_CH0+1	// 2nd
#define CAN_CH2								CAN_CH0+2	// 3rd
#define CAN_CH3								CAN_CH0+3	// 4th
#define CAN_CH4								CAN_CH0+4	// 5th
#define CAN_CH5								CAN_CH0+5	// 6th
#endif


/* Drift Compensation */
#define Drift_Compensation					1
#define n_samplingLR						80

/* Drift Compensation in the Proximity mode */
#define Drift_Compensation_Proximity		0

/* Drift Compensation with Temperature Measurement */
#define Drift_Compensation_Temperature		0

/* Coefficient & Offset */
#define Gain1st_Compensation1				0.4000
#define Offset1								2.00E+07
#define Gain1st_Compensation2				0.1000
#define Offset2								2.00E+07

/* Distance Calibration */
#define CalibrationDistance					1

/* Curve fitting */
#define CurveFitting 						1

/* Temperature sensor */
#define TMP117								0

/* Kalman Filter */
#define KalmanFilters						1

/* LowPass Filter */
#define LowPassFilter						0

/* V1 filtering stages */
#define EMA2Filter							1
#define EMA2_DELTA_LOW						5000.0f
#define EMA2_DELTA_HIGH						42000.0f
#define EMA2_ALPHA_SLOW						0.01f
#define EMA2_ALPHA_FAST						0.30f

/* Implemented for testing, disabled by default as requested. */
#define SlewRateLimiterFilter				0
#define SLEW_RATE_LIMIT_STEP				10000U

#define ToFFusion							1

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor1_distance_calibration.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "fdc.h"
#include "fdc_recovery.h"
#include "drift_gate.h"
#include "drift_noise_reference.h"
#include "drift_model_math.h"
#include "drift_learning_guard.h"
#include "drift_approach_gate.h"
#include "drift_model_audit.h"
#include "ema_precision.h"
#include "fusion_cap_noise.h"
#include "sensor1_v1_auto_sigma.h"
#include "evaluation_trace.h"
#include "evaluation_clock.h"

#if TMP117
#include "tmp117.h"
#endif

#if ToF
#include "VL53L1X_api.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

states state;
extern states state;

NEXTLAB_fdc fdc;
extern NEXTLAB_fdc fdc;

#if CalibrationDistance
distance dist;
extern distance dist;
#endif

driftCompensation dComps;
extern driftCompensation dComps;

samples sample;
extern samples sample;

kalmanFilter kalman;
extern kalmanFilter kalman;

#if ToF
ToF_vl53l1x tof;
extern ToF_vl53l1x tof;
static uint8_t tof_init_stage = 0U;
static int8_t tof_init_status = 0;
static int8_t tof_read_status = 0;
static uint16_t tof_read_count = 0U;
static uint16_t tof_error_count = 0U;
static uint8_t tof_line_before_request = 0U;
static uint8_t tof_line_after_request = 0U;
static uint8_t tof_line_after_shutdown = 0U;
static uint8_t tof_boot_hal_error = 0U;
static uint8_t tof_boot_hal_state = 0U;
static uint16_t tof_boot_isr = 0U;
extern volatile uint8_t vl53l1_diag_phase;
extern volatile uint8_t vl53l1_diag_status;
extern volatile uint8_t vl53l1_diag_error;
extern volatile uint8_t vl53l1_diag_state;
extern volatile uint16_t vl53l1_diag_cr2;
extern volatile uint8_t vl53l1_diag_write_ok;
extern volatile uint8_t vl53l1_diag_read_ok;
#endif

Curvefitting cvf;
extern Curvefitting cvf;

#if TMP117
temperature117 tmp117;
extern temperature117 tmp117;
#endif

SensorFusion fusion;
extern SensorFusion fusion;

slewRateLimiterState slewLimiter;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flag for TIM7 */
volatile uint8_t TIM7_flag = 0;
volatile uint8_t TIM6_flag = 0;

static uint8_t mobile_fdc_read_count = 0U;
volatile uint32_t fdc_recovery_retry_count = 0U;
volatile uint32_t fdc_recovery_retry_success_count = 0U;
#if ToF
static uint32_t mobile_tof_tick = 0U;
static uint8_t mobile_tof_seen = 0U;
static uint8_t mobile_tof_quality_valid = 0U;
volatile uint8_t mobile_tof_range_status = 255U;
#endif

/* Variables for CAN Receive Interrupt */
//volatile uint8_t can1_rx0_flag = 0;

/* Variables for Linear regression */
uint16_t count_sampling = 0;
uint8_t n_PMsamplingLR = 0;
uint8_t n_data = 0;
uint8_t n_PMdata = 0;
uint16_t peroid_sampling = 0;

/* Variables for derivative integration */
uint8_t flag_di = 0;
uint8_t n_samplingDI = 0;

/* Algorithm to detect an object */
uint8_t flag_algorithmDO = 0;
uint8_t flag_DOinit = 0;
uint8_t countDO = 0;

uint32_t ema2_fdc_Q1 = 0U;
uint8_t ema2_initialized = 0U;
uint32_t drift_only_fdc_Q1 = 0U;
FusionCapNoiseState fusion_cap_noise_Q1;
static double ema2_accumulator_Q1 = 0.0;
volatile float ema2_selected_alpha = 0.0f;
volatile uint32_t ema2_precision_self_test_failed = 0U;
volatile int32_t ema2_legacy_constant_bias = 0;
volatile uint32_t eval_tim7_interrupts = 0U;
volatile uint32_t eval_tim7_coalesced = 0U;
volatile uint32_t eval_tim6_interrupts = 0U;
volatile uint32_t eval_tim6_coalesced = 0U;
volatile uint32_t eval_trace_sequence = 0U;
volatile uint32_t eval_trace_depth = EVAL_TRACE_DEPTH;
volatile uint32_t eval_trace_clock_hz = 1000000U;
volatile uint32_t eval_trace_clock_kind = 2U; /* Independent TIM2 */
volatile uint32_t eval_clock_self_test_failed = 0U;
volatile EvaluationTraceEntry eval_trace[EVAL_TRACE_DEPTH];
static uint32_t eval_stage_elapsed_us[EVAL_STAGE_COUNT];

/* SWD-only evaluation counters; the CAN 0x41..0x44 contract is unchanged. */
volatile uint32_t sensor1_drift_variant = 0x56314533U; /* V1E3: Adaptive EMA + auto quiet sigma +3sigma */
volatile uint32_t sensor1_drift_audit_schema = 8U;
volatile uint32_t drift_eval_valid_cycles = 0U;
volatile uint32_t drift_eval_invalid_skips = 0U;
volatile uint32_t drift_eval_learning_updates = 0U;
volatile uint32_t drift_eval_frozen_samples = 0U;
volatile uint32_t drift_eval_zero_variance_skips = 0U;
volatile uint32_t drift_eval_last_us = 0U;
volatile uint32_t drift_eval_max_us = 0U;
volatile uint32_t drift_eval_self_test_failed = 0U;
volatile uint32_t drift_eval_object_reference = 10000000U;
volatile uint32_t drift_eval_noise_floor = 0U; /* No physical count floor. */
volatile uint32_t drift_model_math_self_test_failed = 0U;
volatile uint32_t drift_model_init_self_test_failed = 0U;
volatile uint32_t drift_model_reset_count = 0U;
volatile uint32_t drift_model_audit_sequence = 0U;
volatile uint32_t drift_model_audit_depth = DRIFT_MODEL_AUDIT_DEPTH;
volatile uint32_t drift_model_audit_entry_size = sizeof(DriftModelAuditEntry);
volatile uint32_t drift_model_audit_overwritten = 0U;
volatile uint32_t drift_model_gain_lower_hits = 0U;
volatile uint32_t drift_model_gain_upper_hits = 0U;
volatile uint32_t drift_model_object_learning_violations = 0U;
volatile uint32_t drift_model_large_step_events = 0U;
volatile int64_t drift_model_sensor_variance = 0;
volatile DriftModelAuditEntry drift_model_audit[DRIFT_MODEL_AUDIT_DEPTH];
volatile uint32_t drift_audit_kalman_x0_before_bits = 0U;
volatile uint32_t drift_audit_kalman_p0_before_bits = 0U;
volatile uint32_t drift_audit_kalman_gain0_bits = 0U;
/* Active RAW policy; the legacy guard retains fitted-model diagnostics only. */
DriftApproachGate drift_approach_gate;
static uint8_t drift_bootstrap_model_seeded;
static uint8_t drift_pair_tick_valid;
static uint32_t drift_pair_tick;
DriftLearningGuard drift_learning_guard;
DriftNoiseReference drift_noise_reference;
volatile uint32_t drift_detection_baseline_sigma = 0U;
volatile uint32_t drift_detection_recent_sigma = 0U;
volatile uint32_t drift_detection_entry_threshold = 150U;
volatile uint32_t drift_learning_input_ch0 = 0U;
volatile uint32_t drift_learning_input_ch1 = 0U;
volatile uint32_t drift_learning_candidate_rejects = 0U;

/* CAN 0x41 keeps the first received 24-bit counts, before any filtering. */
static uint32_t mobile_fdc_raw_ch0 = 0U;
static uint32_t mobile_fdc_raw_ch1 = 0U;
static uint8_t mobile_fdc_raw_status = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DeviceCheck(void);
void SystemConfiguration(void);
void InitializeCANProtocol(void);
static void StoreValidCapDistance(double candidate);
static uint8_t current_sample_received, tof_transport_error;
static int32_t CurrentDistanceToInt(double mm);
void InitializeCompensation(void);
void GetSensingData(void);
HAL_StatusTypeDef RecoverFDCI2C(void);
void GetFilteredData(void);
void DriftCompensation(void);
void ApplyEMA2Filter(void);
static void UpdateFusionCapSignal(void);
int64_t TwinSensorCompensation(uint32_t* input_sensor, uint32_t* input_compensation, double gain_1st, int64_t offset);
uint32_t DriftModelInitializationSelfTest(void);
void RecordDriftModelAudit(void);
void LinearRegression(void);
void CAN_transmit(void);
void CAN_receive(void);
void SplitData(uint32_t data_32bit, uint8_t *data_8bit);
uint32_t MergeData(uint8_t* temp_data);
uint32_t FDC_RawToFrequencyHz(uint32_t raw_data, uint8_t data_format,
							  uint32_t conversion_time_us);
uint32_t FDC_RawToCountMilli(uint32_t raw_data, uint8_t data_format);
uint32_t FDC_ConversionTimeUs(uint8_t conversion_reg);

float TMP117_RawToCelsius(uint8_t msb, uint8_t lsb);
//void TMP117_Celsius(uint8_t msb, uint8_t lsb, uint8_t temp_data_celsius[2]);

#if 0
void SplitData_8Byte(int64_t data_64bit, uint8_t *data_8bit);
#endif

#if Drift_Compensation_Temperature
int64_t TemperatureCompensation(uint32_t* input_sensor, uint32_t* temperature, float coefficient, int64_t offset);
#endif

#if (CalibrationDistance || CurveFitting)
void CalibrateDistance(void);
void CalibrateDistanceNormalized(void);
#endif

#if ToF
void InitializeToF(void);
void GetToFData(void);
#endif

#if CurveFitting
void InitializeCurveFitting(void);
void ExpCurvefitting(void);
#if ToF
void AchieveCurveFittingData(void);
#endif
#endif

#if (ToFFusion && ToF && CurveFitting)
void PerformSensorFusion(void);
#endif

uint32_t SlewRateLimiter(uint32_t input, uint32_t *prev,
						 uint8_t *initialized, uint32_t max_step);
void RecordEvaluationTrace(uint32_t cycle_start);

/* Filters */
/* Kalman Filter */
#if KalmanFilters
void InitializeKalmanFilters(void);
uint32_t KalmanFilter(uint32_t input, uint8_t A, float H, float Q,
					  float R, float *x, float *P);
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Confirming I2C Interface with FDC2214 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  EvaluationClockInit();
  uint32_t eval_clock_test_start = EvaluationNowUs();
  HAL_Delay(2U);
  uint32_t eval_clock_test_elapsed = EvaluationNowUs() - eval_clock_test_start;
  eval_clock_self_test_failed = (uint32_t)(
          (eval_clock_test_elapsed < 2000U) || (eval_clock_test_elapsed > 4000U));

  /* Initialize the System Configuration */
  SystemConfiguration();

  /* Confirming I2C Interface with Nextlab FDC */
  DeviceCheck();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Initialize the Parameters CAN Protocol */
  InitializeCANProtocol();

  /* Initialize the Parameters for Compensation */
  InitializeCompensation();
  drift_model_init_self_test_failed = DriftModelInitializationSelfTest();


  /* Initialize the Parameters for Kalman Filter */
  #if KalmanFilters
  InitializeKalmanFilters();
  #endif

  /* Initialize ToF */
  #if ToF
  InitializeToF();
  #endif

  /* Initialize the Parameter for Auto-Curve Fitting Algorithm */
  #if CurveFitting
  InitializeCurveFitting();
  #endif

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (TIM7_flag == 1)
      {
          TIM7_flag = 0;
          uint32_t eval_cycle_start = EvaluationNowUs();
          uint32_t eval_stage_start = eval_cycle_start;
          for (uint32_t i = 0U; i < EVAL_STAGE_COUNT; ++i) eval_stage_elapsed_us[i] = 0U;

		  /* Getting Raw Data */
		  /* state.onOff_senisng = 1 : Turning the sensing ON */
		  /* state.onOff_senisng = 0 : Turning the sensing OFF */
          if((state.onOff_sensing == 1))
          {
        	  GetSensingData();
        	  if ((fdc.ch0_i2c_status == HAL_OK) && (fdc.ch1_i2c_status == HAL_OK))
        	  {
        	      mobile_fdc_read_count++;
        	  }
          }
		  eval_stage_elapsed_us[0] = EvaluationNowUs() - eval_stage_start;

          uint8_t gate_pair_valid = (uint8_t)(state.onOff_sensing && current_sample_received);

          Sensor1V1AutoSigmaSetInputContext(HAL_GetTick(),gate_pair_valid);

		  /* Never train drift or update filters from a rejected/stale pair. */
          if (gate_pair_valid != 0U)
          {
		  /* Filters */
		  eval_stage_start = EvaluationNowUs();
          /* The v1 engine applies the input Kalman once inside compensation.
           * Adaptive EMA2 below supplies final output and detector feedback. */
		  eval_stage_elapsed_us[1] = EvaluationNowUs() - eval_stage_start;

		  /* Compensating Drift */
		  eval_stage_start = EvaluationNowUs();
		  /* Read the saved pre-EMA sample without restoring shared dComps. */
		  FusionCapNoiseStep(&fusion_cap_noise_Q1, drift_only_fdc_Q1,
		          flag_di, flag_algorithmDO);
		  DriftCompensation();
		  eval_stage_elapsed_us[2] = EvaluationNowUs() - eval_stage_start;
          /* Preserve the pre-EMA value for the independent fusion noise path.
           * Restore the original Adaptive EMA and feed its final value back. */
          eval_stage_start = EvaluationNowUs();
          #if EMA2Filter
          ApplyEMA2Filter();
          #endif
          Sensor1V1AutoSigmaSetFilteredOutput(dComps.fdc_Q1);
          eval_stage_elapsed_us[3] = EvaluationNowUs() - eval_stage_start;

		  UpdateFusionCapSignal();

		  /* Curve Fitting */
		  #if (CalibrationDistance || CurveFitting)
		  eval_stage_start = EvaluationNowUs();
		  CalibrateDistance();
		  CalibrateDistanceNormalized();
		  eval_stage_elapsed_us[4] = EvaluationNowUs() - eval_stage_start;
		  #endif
          }
          else
          {
              drift_eval_invalid_skips++;
              DriftLearningGuardPause(&drift_learning_guard, DRIFT_LEARN_INVALID);
              DriftNoiseReferenceReject(&drift_noise_reference);
              drift_learning_guard.residual_count = 0U;
              drift_learning_guard.residual_head = 0U;
              drift_learning_guard.residual_high_timer = 0U;
              drift_learning_guard.reference_tick = HAL_GetTick();
          }

		  #if ToF
		  eval_stage_start = EvaluationNowUs();
		  GetToFData();
		  eval_stage_elapsed_us[5] = EvaluationNowUs() - eval_stage_start;
		  #endif

		  /* Auto-Self Curve Fitting Algorithm */
		  #if (CurveFitting)
		  eval_stage_start = EvaluationNowUs();
		  #if ToF
		  if(((tof.state[0] != 0U) && (tof.distance[0] != 0U) &&
		      (tof.distance[0] <= 280U)) ||
		     ((tof.state[1] != 0U) && (tof.distance[1] != 0U) &&
		      (tof.distance[1] <= 280U)))
		  {
			  if((state.onOff_cftool == 1))
			  {
				  AchieveCurveFittingData();
			  }
			  ExpCurvefitting();

			  #if 1
			  //cvf.FilteredDistance_Q1 = (1-tof.distance[0]/220)*cvf.distancePredict_Q1 + (tof.distance[0]/220)*dist.Q1;
			  StoreValidCapDistance((0.02)*cvf.distancePredict_Q1 + (0.98)*dist.Q1);
			  #else
			  StoreValidCapDistance(dist.Q1);
			  #endif

			  #if FDC2214_ADD
			  #if 1
			  cvf.FilteredDistance_Q2 = (0.02)*cvf.distancePredict_Q2 + (0.98)*dist.Q2;
			  #else
			  cvf.FilteredDistance_Q2 = dist.Q2;
			  #endif

			  if(cvf.FilteredDistance_Q2 <= 0)
			  {
			  	  cvf.FilteredDistance_Q2 = 0;
			  }
			  #endif
		  }
		  else
		  #endif
		  {
			  ExpCurvefitting();

			  #if 1
			  //cvf.FilteredDistance_Q1 = (1-dist.Q1/220)*cvf.distancePredict_Q1 + (dist.Q1/220)*dist.Q1;
			  StoreValidCapDistance((0.02)*cvf.distancePredict_Q1 + (0.98)*dist.Q1);
			  #else
			  StoreValidCapDistance(dist.Q1);
			  #endif

			  #if FDC_ADD
			  #if 1
			  cvf.FilteredDistance_Q2 = (0.02)*cvf.distancePredict_Q2 + (0.98)*dist.Q2;
			  #else
			  cvf.FilteredDistance_Q2 = dist.Q2;
			  #endif

			  if(cvf.FilteredDistance_Q2 <= 0)
			  {
			 	  cvf.FilteredDistance_Q2 = 0;
			  }
			  #endif
		  }

		  #if (ToFFusion && ToF)
		  eval_stage_elapsed_us[6] = EvaluationNowUs() - eval_stage_start;
		  eval_stage_start = EvaluationNowUs();
		  PerformSensorFusion();
		  eval_stage_elapsed_us[7] = EvaluationNowUs() - eval_stage_start;
		  #endif
		  #endif

		  #if TMP117
		  TMP117_Read(&hi2c3, Temperature_Register, &tmp117.temp_data[0]);
//		  tmp117.temp_data_celsius = TMP117_Celsius(tmp117.temp_data[0], tmp117.temp_data[1]);
		  #endif
          RecordEvaluationTrace(eval_cycle_start);
      }

	  if(TIM6_flag == 1)
	  {
		  /* Reset Timer Flag */
		  TIM6_flag = 0;

		  /* Transmitting CAN data */
		  #if CANCommunication
		  CAN_transmit();
		  #endif

		  /* Receiving CAN data */
		  #if 1
		  CAN_receive();
		  #endif

	  }

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Mobile CAN: three data frames and one status frame, identical on ASIC/SoC.
 * Every data field is a 32-bit big-endian value. Snapshot once per four-frame
 * cycle so FDC, post-EMA, CVF, fusion and ToF belong to the same processing state. */
typedef struct
{
    uint32_t values[3][2];
    uint8_t debug[8];
} MobileCANTelemetry;

static MobileCANTelemetry mobile_can_snapshot;

static void CaptureMobileCANTelemetry(void)
{
    uint8_t flags = 0U;
    uint8_t lines = 0U;
    uint8_t tof_ready = 0U;
    uint8_t tof_fresh = 0U;
    uint8_t tof_reads = 0U;
    int8_t tof_status = 0;
    uint32_t tof_distance = 0U;
    #if ToF
    tof_ready = tof.state[0];
    tof_distance = tof_transport_error ? UINT32_MAX : (uint32_t)tof.distance[0];
    tof_fresh = (uint8_t)((tof_ready != 0U) && (mobile_tof_seen != 0U) &&
            (tof_read_status == 0) && ((HAL_GetTick() - mobile_tof_tick) <= 200U));
    tof_reads = (uint8_t)tof_read_count;
    tof_status = (tof_init_status != 0) ? tof_init_status : tof_read_status;
    #endif
    if (fdc.config_i2c_status == HAL_OK) flags |= 0x01U;
    if (fdc.ch0_valid != 0U) flags |= 0x02U;
    if (fdc.ch1_valid != 0U) flags |= 0x04U;
    if (tof_ready != 0U) flags |= 0x08U;
    if (tof_fresh != 0U) flags |= 0x10U;
    if (fusion.FusedDistance_Q1_valid != 0U) flags |= 0x20U;
    if (ema2_initialized != 0U) flags |= 0x40U;
    if (state.onOff_sensing != 0U) flags |= 0x80U;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) lines |= 0x01U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) lines |= 0x02U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) lines |= 0x04U;
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) lines |= 0x08U;
    if ((hi2c1.Instance->ISR & I2C_ISR_BUSY) != 0U) lines |= 0x10U;
    if ((hi2c3.Instance->ISR & I2C_ISR_BUSY) != 0U) lines |= 0x20U;
    /* SoC evaluation: previously unused line-byte bits expose the object gate
     * and an actual learner heartbeat without attaching a debugger in motion. */
    if (state.idle_fdcQ1 != 0U) lines |= 0x40U;
    if ((drift_eval_learning_updates & 1U) != 0U) lines |= 0x80U;

    mobile_can_snapshot.values[0][0] = mobile_fdc_raw_ch0;
    mobile_can_snapshot.values[0][1] = mobile_fdc_raw_ch1;
    mobile_can_snapshot.values[1][0] = dComps.fdc_Q1;
    mobile_can_snapshot.values[1][1] = (uint32_t)cvf.FilteredDistance_Q1;
    mobile_can_snapshot.values[2][0] = (uint32_t)fusion.FusedDistance_Q1;
    mobile_can_snapshot.values[2][1] = tof_distance;
    mobile_can_snapshot.debug[0] = flags;
    mobile_can_snapshot.debug[1] = lines;
    mobile_can_snapshot.debug[2] = (uint8_t)(0x80U |
            (((uint8_t)fdc.ch1_i2c_status & 0x03U) << 4) |
            (((uint8_t)fdc.ch0_i2c_status & 0x03U) << 2) |
            ((uint8_t)fdc.config_i2c_status & 0x03U));
    mobile_can_snapshot.debug[3] = mobile_fdc_raw_status;
    mobile_can_snapshot.debug[4] = (uint8_t)tof_status;
    mobile_can_snapshot.debug[5] = (uint8_t)can_tx_fail_count;
    mobile_can_snapshot.debug[6] = mobile_fdc_read_count;
    mobile_can_snapshot.debug[7] = tof_reads;
}

static HAL_StatusTypeDef CAN_SendUInt32Pair(uint8_t id, uint32_t first, uint32_t second)
{
    return CAN_Send(id,
            (uint8_t)(first >> 24), (uint8_t)(first >> 16),
            (uint8_t)(first >> 8), (uint8_t)first,
            (uint8_t)(second >> 24), (uint8_t)(second >> 16),
            (uint8_t)(second >> 8), (uint8_t)second);
}

void CAN_transmit(void)
{
    HAL_StatusTypeDef status;
    if (state.canTransmit > 3U) state.canTransmit = 0U;
    if (state.canTransmit == 0U) CaptureMobileCANTelemetry();
    if (state.canTransmit < 3U)
    {
        uint8_t page = state.canTransmit;
        status = CAN_SendUInt32Pair((uint8_t)(CAN_CH0 + page),
                mobile_can_snapshot.values[page][0],
                mobile_can_snapshot.values[page][1]);
    }
    else
    {
        status = CAN_Send(CAN_CH3,
                mobile_can_snapshot.debug[0], mobile_can_snapshot.debug[1],
                mobile_can_snapshot.debug[2], mobile_can_snapshot.debug[3],
                mobile_can_snapshot.debug[4], mobile_can_snapshot.debug[5],
                mobile_can_snapshot.debug[6], mobile_can_snapshot.debug[7]);
    }
    /* One frame per timer tick; do not skip a page when all mailboxes are full. */
    if (status == HAL_OK) state.canTransmit = (uint8_t)((state.canTransmit + 1U) & 3U);
}

/* Receiving CAN Data */
void CAN_receive(void)
{
	if(can1_rx0_flag == 1)
	{
		can1_rx0_flag = 0;

		state.onOff_Function 	= can1Rx0Data[1];
		state.onOff 			= can1Rx0Data[2];

		if((can1Rx0Data[0] == state.onOff_ID) | (can1Rx0Data[0] == 0xFF))
		{
			/* onOff_Function = 1 ---> Reset */
			/* onOff_Function = 2 ---> Sensing */
			/* onOff_Function = 3 ---> CurveFitting */
			/* onOff_Function = 4 ---> RawData */
			if(state.onOff_Function == 0x01)
			{
				/* Reset */
				/* state.onOff = 1 ---> Reboot */
				/* state.onOff = 2 ---> Reset */
				if(state.onOff == 0x01)
				{
					NVIC_SystemReset();
				}
				else if(state.onOff == 0x02)
				{
					InitializeCompensation();
				}
				else
				{
					/* Nothing */
				}
			}
			else if(state.onOff_Function == 0x02)
			{
				/* Sensing */
			}
			else if(state.onOff_Function == 0x03)
			{
				/* CurveFitting */
				/* state.onOff = 0 ---> CurveFitting ON */
				/* state.onOff = 1 ---> CurveFitting OFF */
				if(state.onOff == 0x00)
				{
					state.onOff_cftool = 0;
				}
				else if(state.onOff == 0x01)
				{
					state.onOff_cftool = 1;
				}
				else
				{
					/* Nothing */
				}
			}
			else if(state.onOff_Function == 0x04)
			{
				/* RawData */
			}
			else
			{
				/* Nothing */
			}

		}
		state.onOff_Function 	= 0;
		state.onOff 			= 0;
	}
	else
	{
		/* Nothing */
	}
}

/* System Configuration */
void SystemConfiguration(void)
{
	/* Delay */
	HAL_Delay(100);

	/* GPIO Initialization for LED */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	/* GPIO Initialization for Output_4 (1st ToF) */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
	/* GPIO Initialization for Output_3 (2nd ToF) */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
	/* GPIO Initialization for Output_2 (3rd ToF) */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
	/* GPIO Initialization for Output_1 (4th ToF) */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);

	/* Timer counter period initialization */
	TIM_Configuration(CANTransmit, ConversionTimeControl);

	/* I2C Interface With the Expansion Accessories */
	#if Expansion
	HAL_I2C_Init(&hi2c1);
	#endif

	/* CAN Communication Initialization */
	HAL_CAN_Init(&hcan1);
	CAN_Filter_Init();

	#if FDC
	fdc.config_reg = 0xFFU;
	fdc.conv_reg_ch0 = FDC_CH0_CONV_REG_VALUE;
	fdc.conv_reg_ch1 = FDC_CH1_CONV_REG_VALUE;
	fdc.config_i2c_status = FDC_InitWithRetry(&hi2c3);
	if (fdc.config_i2c_status == HAL_OK)
	{
		fdc.config_reg = FDC_ACTIVE_CONFIG_ON;
		fdc.config_i2c_status = FDC_WarmupAndDiscard(&hi2c3);
	}
	#endif
	#if FDC_ADD
	FDC_Init(&hi2c1);
	FDC_WriteReg(&hi2c1, FDC_REG_SHIELD_CTRL, FDC_SHIELD_DRIVE_ACTIVE);
	#endif

	/* TMP117 initialization */
	#if TMP117
	HAL_Delay(1);
	TMP117_Init(&hi2c3);
	HAL_Delay(1);
	#if 1
	TMP117_Init(&hi2c1);
	HAL_Delay(1);
	#endif
	#endif

 	/* Timer interrupt initialization */
	HAL_TIM_Base_Start_IT(&htim7);
 	HAL_TIM_Base_Start_IT(&htim6);
}

void DeviceCheck(void)
{
	/* Verify the active CH0 conversion register. */
	uint8_t reg_val = 0;

	#if FDC
	if (FDC_ReadReg(&hi2c3, FDC_REG_CONV_TIME_CH0, &reg_val) == HAL_OK)
	{
		fdc.conv_reg_ch0 = reg_val;
		if (FDC_ReadReg(&hi2c3, FDC_REG_CONV_TIME_CH1, &reg_val) == HAL_OK)
		{
			fdc.conv_reg_ch1 = reg_val;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		/* Keep CAN and the main loop running even when the FDC is absent. */
	}
	#endif

	#if FDC_ADD
	if (FDC_ReadReg(&hi2c1, FDC_REG_CONV_TIME_CH0, &reg_val) == HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		/* Keep CAN and the main loop running even when the FDC is absent. */
	}
	#endif
}


void InitializeCANProtocol(void)
{
	state.onOff_ID = CAN_CH0;
	state.onOff_sensing = 1;
	/* Continue automatic curve collection from the measured initial seeds. */
	state.onOff_cftool = 0; /* Preserve measured calibration; CAN can re-enable collection. */
	/* 0: DisatanceData, 1: RawData */
	state.onOff_rawData = 0;
	state.onOff_reset = 0;
}

static void StoreValidCapDistance(double candidate)
{
    cvf.FilteredDistance_Q1=(state.onOff_sensing && current_sample_received)?CurrentDistanceToInt(candidate):-1;
}

static int32_t CurrentDistanceToInt(double mm)
{
    if (!isfinite(mm)) return -1;
    if (mm <= 0.0) return 0;
    if (mm >= (double)INT32_MAX) return INT32_MAX;
    return (int32_t)mm;
}

void InitializeCompensation(void)
{
    current_sample_received=tof_transport_error=0U;
    cvf.FilteredDistance_Q1 = 0; /* Explicit reset, not a quality rejection. */
    Sensor1V1AutoSigmaReset();
    drift_bootstrap_model_seeded = 0U;
    drift_pair_tick_valid = 0U;
    drift_pair_tick = 0U;
	/* CAN compensation reset must clear the complete old learner/detector,
	 * including the old EMA history used by detection. Chip I2C state is separate. */
	memset(&sample, 0, sizeof(sample));
	memset(&dComps, 0, sizeof(dComps));
	count_sampling = 0U;
	countDO = 0U;
	flag_di = 0U;
	flag_algorithmDO = 0U;
	flag_DOinit = 0U;
	state.idle_fdcQ1 = 0U;
	state.idle_fdcQ2 = 0U;
	drift_model_sensor_variance = 0;
	drift_model_reset_count++;
	DriftLearningGuardReset(&drift_learning_guard);
	DriftNoiseReferenceReset(&drift_noise_reference);
	drift_detection_baseline_sigma = 0U;
	drift_detection_recent_sigma = 0U;
	drift_detection_entry_threshold = 0U;
	drift_learning_input_ch0 = 0U;
	drift_learning_input_ch1 = 0U;
	drift_model_math_self_test_failed = DriftModelMathSelfTest();
	slewLimiter = (slewRateLimiterState){0};
	ema2_fdc_Q1 = 0U;
	ema2_initialized = 0U;
	drift_only_fdc_Q1 = 0U;
	ema2_accumulator_Q1 = 0.0;
	ema2_selected_alpha = 0.0f;
	ema2_precision_self_test_failed = EMA2PrecisionSelfTest();
	ema2_legacy_constant_bias = EMA2LegacyConstantBias();
	fusion = (SensorFusion){0};
	FusionCapNoiseReset(&fusion_cap_noise_Q1);
	drift_eval_self_test_failed = DriftGateSelfTest();

	/* Parameter Initialization for Drift Compensation */
	peroid_sampling = 20;
	n_data = 0;
	n_PMdata = 0;
	n_PMsamplingLR = 40;
	n_samplingDI = 3;

	#if FDC
	state.idle_fdcQ1 = 0;
	sample.Integral_fdcQ1[2] = 0;
	sample.sumIntegral_fdcQ1 = 10000000;
	sample.meanDOinit_fdcQ1 = sample.sumIntegral_fdcQ1;

	#if Drift_Compensation_Proximity
	sample.Integral_RfdcQ1[2] = 0;
	sample.sumIntegral_RfdcQ1 = 10000000;
	#endif
	#endif

	#if FDC2_ADD
	state.idle_fdcQ2 = 0;
	sample.Integral_fdcQ2[2] = 0;
	sample.sumIntegral_fdcQ2 = 10000000;
	sample.meanDOinit_fdcQ2 = sample.sumIntegral_fdcQ2;
	#endif

	/* Drift Compensation Using the Temperature Measurement */
	#if Drift_Compensation_Temperature
	state.movingObject_fdc = 1;
	sample.Integral_tmp117[2] = 0;
	sample.sumIntegral_tmp117 = 10000000;
	#endif
}

uint32_t DriftModelInitializationSelfTest(void)
{
    /* Exercise the real warm compensation-reset path with an old object model.
     * No peripheral, motor, or chip reset is involved. Leave a clean learner. */
    count_sampling = 79U;
    countDO = 80U;
    flag_di = 1U;
    flag_algorithmDO = 1U;
    flag_DOinit = 1U;
    state.idle_fdcQ1 = 1U;
    sample.fdc300[0] = 123U;
    sample.fdc302[79] = 456U;
    sample.DI_fdcQ1[1] = 39820000U;
    sample.stdDOinit_fdcQ1 = 99U;
    dComps.fdcCoeff_Q1 = 9999;
    dComps.fdcOffset_Q1 = 7654321;
    (void)Sensor1V1AutoSigmaStep(10000000U,11000000U);
    Sensor1V1AutoSigmaSetFilteredOutput(12345678U);
    drift_bootstrap_model_seeded = drift_pair_tick_valid = 1U;
    drift_pair_tick = 123U;
    drift_noise_reference.ready = 1U;
    drift_noise_reference.sample_count = 501U;
    drift_noise_reference.sigma_counts = 123U;
    InitializeCompensation();
    uint32_t failures = 0U;
    if (count_sampling || countDO || flag_di || flag_algorithmDO || flag_DOinit ||
        state.idle_fdcQ1 || n_data || n_PMdata) failures |= 1U;
    if (sample.fdc300[0] || sample.fdc302[79] || sample.DI_fdcQ1[1] ||
        sample.stdDOinit_fdcQ1 || dComps.fdcCoeff_Q1 || dComps.fdcOffset_Q1)
        failures |= 2U;
    if (peroid_sampling != 20U || n_samplingDI != 3U ||
        sample.sumIntegral_fdcQ1 != 10000000) failures |= 4U;
    if (drift_noise_reference.ready || drift_noise_reference.sample_count ||
        drift_noise_reference.sigma_counts || drift_detection_recent_sigma ||
        drift_detection_baseline_sigma ||
        drift_detection_entry_threshold)
        failures |= 8U;
    const Sensor1V1AutoSigmaResult *reset_model = Sensor1V1AutoSigmaGet();
    if (reset_model->accepted_pairs || reset_model->n_data || reset_model->guard ||
        reset_model->final_count || reset_model->init_sigma ||
        drift_bootstrap_model_seeded || drift_pair_tick_valid || drift_pair_tick)
        failures |= 16U;
    return failures;
}

void GetSensingData(void)
{
    current_sample_received=0U;
	#if FDC
	static uint8_t consecutive_i2c_failures = 0U;
	static FDCRecoverySchedule recovery_schedule = {0};
	uint8_t channel0_data[FDC_CH0_DATA_SIZE] = {0U};
	uint8_t channel1_data[FDC_CH0_DATA_SIZE] = {0U};
	uint8_t status_reg = 0U;
	fdc.ch0_valid = 0U;
	fdc.ch1_valid = 0U;
	fdc.ch0_reject_reason = FDC_SAMPLE_REJECT_NONE;
	fdc.ch1_reject_reason = FDC_SAMPLE_REJECT_NONE;

	/* A failed bus clear must not permanently disable reads after the slave
	 * releases the lines. Retry only after sampled idle and a cooldown. */
	if (fdc.config_i2c_status != HAL_OK)
	{
		fdc.ch0_i2c_status = fdc.config_i2c_status;
		fdc.ch1_i2c_status = fdc.config_i2c_status;
		fdc.status_reg = 0U;
		fdc.raw_ch0_sample = 0U;
		fdc.raw_ch1_sample = 0U;
		fdc.count_ch0_sample_milli = 0U;
		fdc.count_ch1_sample_milli = 0U;
		fdc.frequency_ch0_sample_hz = 0U;
		fdc.frequency_ch1_sample_hz = 0U;
		fdc.ch0_reject_reason = FDC_SAMPLE_REJECT_I2C;
		fdc.ch1_reject_reason = FDC_SAMPLE_REJECT_I2C;

		uint8_t lines_high = (uint8_t)(
				(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) &&
				(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET));
		if (FDCRecoveryDue(&recovery_schedule, HAL_GetTick(), lines_high) != 0U)
		{
			fdc_recovery_retry_count++;
			fdc.config_i2c_status = RecoverFDCI2C();
			FDCRecoveryArm(&recovery_schedule, HAL_GetTick());
			if (fdc.config_i2c_status == HAL_OK)
			{
				consecutive_i2c_failures = 0U;
				fdc_recovery_retry_success_count++;
				FDCRecoveryReset(&recovery_schedule);
			}
		}
		/* This cycle is still invalid: recovery is not a fresh sample pair. */
		return;
	}
	FDCRecoveryReset(&recovery_schedule);

	fdc.ch0_i2c_status =
			FDC_ReadCH0Sample(&hi2c3, channel0_data, &status_reg);
	HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
	fdc.ch1_i2c_status =
			FDC_ReadCH1Sample(&hi2c3, channel1_data, &status_reg);

	/* Preserve each successful receive even if the paired sample is rejected.
	 * On an I2C failure, keep that channel's last received value and flags. */
	if (fdc.ch0_i2c_status == HAL_OK)
	{
		mobile_fdc_raw_ch0 = (uint32_t)channel0_data[0] |
				((uint32_t)channel0_data[1] << 8) |
				((uint32_t)channel0_data[2] << 16);
		mobile_fdc_raw_status = (uint8_t)(
				(mobile_fdc_raw_status & ~(FDC_STATUS_FR_CH0 | FDC_STATUS_ERR_CH0)) |
				(status_reg & (FDC_STATUS_FR_CH0 | FDC_STATUS_ERR_CH0)));
	}
	if (fdc.ch1_i2c_status == HAL_OK)
	{
		mobile_fdc_raw_ch1 = (uint32_t)channel1_data[0] |
				((uint32_t)channel1_data[1] << 8) |
				((uint32_t)channel1_data[2] << 16);
		mobile_fdc_raw_status = (uint8_t)(
				(mobile_fdc_raw_status & ~(FDC_STATUS_FR_CH1 | FDC_STATUS_ERR_CH1)) |
				(status_reg & (FDC_STATUS_FR_CH1 | FDC_STATUS_ERR_CH1)));
	}

	if ((fdc.ch0_i2c_status != HAL_OK) ||
		(fdc.ch1_i2c_status != HAL_OK))
	{
		if (consecutive_i2c_failures < 0xFFU)
		{
			consecutive_i2c_failures++;
		}
		fdc.status_reg = 0U;
		fdc.raw_ch0_sample = 0U;
		fdc.raw_ch1_sample = 0U;
		fdc.count_ch0_sample_milli = 0U;
		fdc.count_ch1_sample_milli = 0U;
		fdc.frequency_ch0_sample_hz = 0U;
		fdc.frequency_ch1_sample_hz = 0U;
		fdc.ch0_reject_reason = FDC_SAMPLE_REJECT_I2C;
		fdc.ch1_reject_reason = FDC_SAMPLE_REJECT_I2C;
		fdc.ch0_rejected_samples++;
		fdc.ch1_rejected_samples++;

		if (consecutive_i2c_failures >= FDC_I2C_RECOVERY_THRESHOLD)
		{
			fdc.config_i2c_status = RecoverFDCI2C();
			if (fdc.config_i2c_status == HAL_OK)
			{
				consecutive_i2c_failures = 0U;
			}
			else
			{
				FDCRecoveryArm(&recovery_schedule, HAL_GetTick());
			}
		}
		return;
	}
	consecutive_i2c_failures = 0U;

	/*
	 * NXP3110 CH0DR (sub-address 0x64) is read as one 32-bit little-endian
	 * register. The driver extracts bits23:0 and converts bit31/bit30 into
	 * the legacy application overflow/error flags.
	 */
	uint32_t raw_ch0 = (uint32_t)channel0_data[0] |
					   ((uint32_t)channel0_data[1] << 8) |
					   ((uint32_t)channel0_data[2] << 16);
	uint32_t raw_ch1 = (uint32_t)channel1_data[0] |
					   ((uint32_t)channel1_data[1] << 8) |
					   ((uint32_t)channel1_data[2] << 16);
	fdc.status_reg = status_reg;
    current_sample_received=1U;
	fdc.ovf_ch0 =
			(uint8_t)((fdc.status_reg & FDC_STATUS_FR_CH0) != 0U);
	fdc.ovf_ch1 =
			(uint8_t)((fdc.status_reg & FDC_STATUS_FR_CH1) != 0U);
	uint8_t data_format_ch0 = fdc.conv_reg_ch0 & FDC_CONV_FORMAT_Msk;
	uint8_t data_format_ch1 = fdc.conv_reg_ch1 & FDC_CONV_FORMAT_Msk;
	uint32_t conversion_time_ch0_us =
			FDC_ConversionTimeUs(fdc.conv_reg_ch0);
	uint32_t conversion_time_ch1_us =
			FDC_ConversionTimeUs(fdc.conv_reg_ch1);

	fdc.raw_ch0_sample = raw_ch0;
	fdc.raw_ch1_sample = raw_ch1;
	fdc.count_ch0_sample_milli =
			FDC_RawToCountMilli(raw_ch0, data_format_ch0);
	fdc.count_ch1_sample_milli =
			FDC_RawToCountMilli(raw_ch1, data_format_ch1);
	fdc.frequency_ch0_sample_hz = FDC_RawToFrequencyHz(
			raw_ch0, data_format_ch0, conversion_time_ch0_us);
	fdc.frequency_ch1_sample_hz = FDC_RawToFrequencyHz(
			raw_ch1, data_format_ch1, conversion_time_ch1_us);

	if ((fdc.status_reg & FDC_STATUS_FR_CH0) != 0U)
	{
		fdc.ch0_reject_reason |= FDC_SAMPLE_REJECT_FR;
	}
	if ((fdc.status_reg & FDC_STATUS_ERR_CH0) != 0U)
	{
		fdc.ch0_reject_reason |= FDC_SAMPLE_REJECT_ERR;
	}
	if (raw_ch0 >= FDC_CH0_SATURATION_GUARD)
	{
		fdc.ch0_reject_reason |= FDC_SAMPLE_REJECT_FULL_SCALE;
	}
	if ((fdc.status_reg & FDC_STATUS_FR_CH1) != 0U)
	{
		fdc.ch1_reject_reason |= FDC_SAMPLE_REJECT_FR;
	}
	if ((fdc.status_reg & FDC_STATUS_ERR_CH1) != 0U)
	{
		fdc.ch1_reject_reason |= FDC_SAMPLE_REJECT_ERR;
	}
	if (raw_ch1 >= FDC_CH0_SATURATION_GUARD)
	{
		fdc.ch1_reject_reason |= FDC_SAMPLE_REJECT_FULL_SCALE;
	}

	if ((fdc.ch0_reject_reason != FDC_SAMPLE_REJECT_NONE) ||
		(fdc.ch1_reject_reason != FDC_SAMPLE_REJECT_NONE))
	{
		if ((fdc.ch0_reject_reason & FDC_SAMPLE_REJECT_FULL_SCALE) != 0U)
		{
			fdc.ch0_saturation_samples++;
		}
		if ((fdc.ch1_reject_reason & FDC_SAMPLE_REJECT_FULL_SCALE) != 0U)
		{
			fdc.ch1_saturation_samples++;
		}
		fdc.ch0_rejected_samples++;
		fdc.ch1_rejected_samples++;
	}

	/* Successfully received data is allowed into the algorithm/filter input. */
	fdc.temp_ch300[0] = 0U;
	fdc.temp_ch300[1] = channel0_data[2];
	fdc.temp_ch300[2] = channel0_data[1];
	fdc.temp_ch300[3] = channel0_data[0];
	fdc.temp_ch302[0] = 0U;
	fdc.temp_ch302[1] = channel1_data[2];
	fdc.temp_ch302[2] = channel1_data[1];
	fdc.temp_ch302[3] = channel1_data[0];
	fdc.raw_ch0 = raw_ch0;
	fdc.raw_ch1 = raw_ch1;
	fdc.frequency_ch0_hz = FDC_RawToFrequencyHz(
			raw_ch0, data_format_ch0, conversion_time_ch0_us);
	fdc.frequency_ch1_hz = FDC_RawToFrequencyHz(
			raw_ch1, data_format_ch1, conversion_time_ch1_us);
	fdc.ch0_valid = (fdc.ch0_reject_reason==0);
	fdc.ch1_valid = (fdc.ch1_reject_reason==0);
	fdc.ch0_sequence++;

	#if KalmanFilters
	/* Seed the legacy filter with the first real NXP3110 sample. */
	if (fdc.ch0_filter_seeded == 0U)
	{
		kalman.x_Q1 = raw_ch0;
		kalman.P_Q1 = 1U;
		fdc.ch300 = raw_ch0;
		fdc.ch0_filter_seeded = 1U;
	}
	if (fdc.ch1_filter_seeded == 0U)
	{
		kalman.x_Q1cmp = raw_ch1;
		kalman.P_Q1cmp = 1U;
		fdc.ch302 = raw_ch1;
		fdc.ch1_filter_seeded = 1U;
	}
	#endif
	#endif
}

HAL_StatusTypeDef RecoverFDCI2C(void)
{
	HAL_StatusTypeDef status;

	/*
	 * Clear a stuck F7 I2C peripheral, check that the NXP3110 ACKs again, then
	 * replay the same atomic conversion trigger used by the reference UI.
	 */
	status = FDC_ClearI2C3Bus(&hi2c3);
	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_IsDeviceReady(&hi2c3, FDC_I2C_ADDR, 3U,
								  FDC_I2C_TIMEOUT_MS);
	FDC_RecordDiagnostic(&hi2c3, FDC_DIAG_OP_DEVICE_READY, 0U, status);
	if (status != HAL_OK)
	{
		return status;
	}

	status = FDC_RestartConversion(&hi2c3);
	if (status != HAL_OK)
	{
		return status;
	}

	status = FDC_WarmupAndDiscard(&hi2c3);
	if (status == HAL_OK)
	{
		status = FDC_CaptureConfigReadback(&hi2c3);
	}
	if (status == HAL_OK)
	{
		fdc.config_reg = FDC_ACTIVE_CONFIG_ON;
	}
	return status;
}

void GetFilteredData(void)
{
	#if FDC
	if (!current_sample_received)
	{
		return;
	}
	#endif

	/* Kalman Filter */
	#if (KalmanFilters)
	/* Merging data */
	#if FDC
	kalman.fdc300 = MergeData(fdc.temp_ch300);
	kalman.fdc302 = MergeData(fdc.temp_ch302);
	#if SlewRateLimiterFilter
	kalman.fdc300 = SlewRateLimiter(kalman.fdc300,
			&slewLimiter.prev_Q1, &slewLimiter.initialized_Q1,
			SLEW_RATE_LIMIT_STEP);
	kalman.fdc302 = SlewRateLimiter(kalman.fdc302,
			&slewLimiter.prev_Q1cmp, &slewLimiter.initialized_Q1cmp,
			SLEW_RATE_LIMIT_STEP);
	#endif
	#endif
	#if FDC_ADD
	kalman.fdc100 = MergeData(fdc.temp_ch100);
	kalman.fdc102 = MergeData(fdc.temp_ch102);
	#endif
	kalman.result_Q1 = KalmanFilter(kalman.fdc300, kalman.A_Q1,
			kalman.H_Q1, kalman.Q_Q1, kalman.R_Q1,
			&kalman.x_Q1, &kalman.P_Q1);
	kalman.result_Q1cmp = KalmanFilter(kalman.fdc302, kalman.A_Q1cmp,
			kalman.H_Q1cmp, kalman.Q_Q1cmp, kalman.R_Q1cmp,
			&kalman.x_Q1cmp, &kalman.P_Q1cmp);
	#if FDC_ADD
	kalman.result_Q2 = KalmanFilter(kalman.fdc100, kalman.A_Q2,
			kalman.H_Q2, kalman.Q_Q2, kalman.R_Q2,
			&kalman.x_Q2, &kalman.P_Q2);
	kalman.result_Q2cmp = KalmanFilter(kalman.fdc102, kalman.A_Q2cmp,
			kalman.H_Q2cmp, kalman.Q_Q2cmp, kalman.R_Q2cmp,
			&kalman.x_Q2cmp, &kalman.P_Q2cmp);
	#endif

	/* Debugging Kalman Filter */
	#if 0
	SplitData(kalman.result_Q1, kalman.temp_resultQ1);
	SplitData(kalman.result_Q1cmp, kalman.temp_resultQ1cmp);
	#endif
	#endif

	#if (KalmanFilters)
	#if FDC
	fdc.ch300 = kalman.result_Q1;
	fdc.ch302 = kalman.result_Q1cmp;
	#endif
	#if FDC_ADD
	fdc.ch100 = kalman.result_Q2;
	fdc.ch102 = kalman.result_Q2cmp;
	#endif
	#endif

	#if Drift_Compensation_Temperature
	tmp117.data = (tmp117.temp_data[0] << 8)|(tmp117.temp_data[1] << 0);
	#endif
}

/* Validate a delayed candidate on the current, held-out clear measurement.
 * A rejected candidate never mutates the committed window or model. */
static uint8_t TrainDelayedQ1Pair(uint32_t input0, uint32_t input1, uint32_t tick)
{
    uint32_t sensor[n_samplingLR], reference[n_samplingLR];
    uint64_t sum0 = input0, sum1 = input1;
    sensor[0] = input0;
    reference[0] = input1;
    for (uint32_t i = 1U; i < n_samplingLR; ++i)
    {
        sensor[i] = sample.fdc300[i - 1U];
        reference[i] = sample.fdc302[i - 1U];
        sum0 += sensor[i];
        sum1 += reference[i];
    }
    uint32_t mean0 = (uint32_t)(sum0 / n_samplingLR);
    uint32_t mean1 = (uint32_t)(sum1 / n_samplingLR);
    int64_t numerator = 0, denominator = 0, sensor_variance = 0;
    for (uint32_t i = 0U; i < n_samplingLR; ++i)
    {
        int64_t x = (int64_t)sensor[i] - mean0;
        int64_t y = (int64_t)reference[i] - mean1;
        numerator += x * y;
        denominator += y * y;
        sensor_variance += x * x;
    }
    int64_t coefficient = dComps.fdcCoeff_Q1;
    uint8_t identifiable = DriftFitGainScaled(numerator, denominator, &coefficient);
    int64_t offset = DriftOffsetFromMeans(mean0, coefficient, mean1);
    /* RAW gate owns approach/learning policy. Do not reintroduce the old
     * fixed-floor compensated-residual veto after the RAW quarantine. */
    double candidate = (double)fdc.ch300 - (double)coefficient * .0001 * fdc.ch302 - offset;
    if (!isfinite(candidate))
    {
        drift_learning_candidate_rejects++;
        return 0U;
    }
    memcpy(sample.fdc300, sensor, sizeof(sensor));
    memcpy(sample.fdc302, reference, sizeof(reference));
    sample.sum_fdc300 = sum0;
    sample.sum_fdc302 = sum1;
    sample.mean_fdc300 = mean0;
    sample.mean_fdc302 = mean1;
    dComps.sumNumnerator_fdcQ1 = numerator;
    dComps.sumDenominator_fdcQ1 = denominator;
    drift_model_sensor_variance = sensor_variance;
    dComps.fdcCoeff_Q1 = coefficient;
    dComps.fdcOffset_Q1 = offset;
    if (!identifiable) drift_eval_zero_variance_skips++;
    drift_eval_learning_updates++;
    drift_learning_input_ch0 = input0;
    drift_learning_input_ch1 = input1;
    peroid_sampling = 80U;
    flag_di = 1U;
    flag_algorithmDO = 1U;
    /* Record model readiness without invoking the old residual/ToF policy or
     * its extra learning pause. Only the RAW gate controls admission. */
    drift_learning_guard.ready = 1U;
    drift_learning_guard.coefficient = coefficient;
    drift_learning_guard.offset = offset;
    drift_learning_guard.reference_tick = tick;
    return 1U;
}

void LinearRegression(void)
{
	/* Derivative Integration Algorithm */
	if(flag_di == 0)
	{
		for(uint8_t i = 0; i < n_samplingDI-1; i ++)
		{
			#if (Operation_Single || Operation_Double)
			#if FDC
			sample.Integral_fdcQ1[2] = 0;
			sample.DI_fdcQ1[n_samplingDI-1-i] = sample.DI_fdcQ1[n_samplingDI-2-i];
			#endif
			#if FDC_ADD
			sample.Integral_fdcQ2[2] = 0;
			sample.DI_fdcQ2[n_samplingDI-1-i] = sample.DI_fdcQ2[n_samplingDI-2-i];
			#endif
			#if Drift_Compensation_Proximity
			sample.Integral_RfdcQ1[2] = 0;
			sample.DI_RfdcQ1[n_samplingDI-1-i] = sample.DI_RfdcQ1[n_samplingDI-2-i];
			#endif
			#if Drift_Compensation_Temperature
			sample.Integral_tmp117[2] = 0;
			sample.DI_tmp117[n_samplingDI-1-i] = sample.DI_tmp117[n_samplingDI-2-i];
			#endif
			#endif
		}
		#if (Operation_Single || Operation_Double)
		#if FDC
		sample.DI_fdcQ1[0] = dComps.fdc_Q1;
		#endif
		#if FDC_ADD
		sample.DI_fdcQ2[0] = dComps.fdc_Q2;
		#endif
		#if Drift_Compensation_Proximity
		sample.DI_RfdcQ1[0] = dComps.Vfdc_Q1;
		#endif
		#if Drift_Compensation_Temperature
		sample.DI_tmp117[0] = dComps.Tfdc_tmp[1];
		#endif
		#endif
	}

	if(flag_di == 1)
	{
		for(uint8_t i = 0; i < n_samplingDI-1; i ++)
		{
			#if (Operation_Single || Operation_Double)
			#if FDC
			sample.Integral_fdcQ1[2] = 0;
			sample.DI_fdcQ1[n_samplingDI-1-i] = sample.DI_fdcQ1[n_samplingDI-2-i];
			#endif

			#if FDC_ADD
			sample.Integral_fdcQ2[2] = 0;
			sample.DI_fdcQ2[n_samplingDI-1-i] = sample.DI_fdcQ2[n_samplingDI-2-i];
			#endif

			/* Compensation in the Proximity mode */
			#if Drift_Compensation_Proximity
			#if FDC
			sample.Integral_RfdcQ1[2] = 0;
			sample.DI_RfdcQ1[n_samplingDI-1-i] = sample.DI_RfdcQ1[n_samplingDI-2-i];
			#endif
			#endif

			#if Drift_Compensation_Temperature
			sample.Integral_tmp117[2] = 0;
			sample.DI_tmp117[n_samplingDI-1-i] = sample.DI_tmp117[n_samplingDI-2-i];
			#endif
			#endif
		}

		#if (Operation_Single || Operation_Double)
		#if FDC
		sample.DI_fdcQ1[0] = dComps.fdc_Q1;
		sample.Derivative_fdcQ1[0] = (int64_t)sample.DI_fdcQ1[2] - (int64_t)sample.DI_fdcQ1[1];
		sample.Integral_fdcQ1[2] = sample.Derivative_fdcQ1[0];
		sample.sumIntegral_fdcQ1 += sample.Integral_fdcQ1[2];
		#endif
		#if FDC_ADD
		sample.DI_fdcQ2[0] = dComps.fdc_Q2;
		sample.Derivative_fdcQ2[0] = sample.DI_fdcQ2[2] - sample.DI_fdcQ2[1];
		sample.Integral_fdcQ2[2] = sample.Derivative_fdcQ2[0];
		sample.sumIntegral_fdcQ2 += sample.Integral_fdcQ2[2];
		#endif

		/* Compensation in the Proximity mode */
		#if Drift_Compensation_Proximity
		#if FDC
		sample.DI_RfdcQ1[0] = dComps.Vfdc_Q1;
		sample.Derivative_RfdcQ1[0] = sample.DI_RfdcQ1[2] - sample.DI_RfdcQ1[1];
		sample.Integral_RfdcQ1[2] = sample.Derivative_RfdcQ1[0];
		sample.sumIntegral_RfdcQ1 += sample.Integral_RfdcQ1[2];
		#endif
		#endif

		#if Drift_Compensation_Temperature
		//sample.DI_tmp117[0] = dComps.Tfdc_Q1;
		sample.DI_tmp117[0] = dComps.Tfdc_tmp[1];
		sample.Derivative_tmp117[0] = sample.DI_tmp117[2] - sample.DI_tmp117[1];
		sample.Integral_tmp117[2] = sample.Derivative_tmp117[0];
		sample.sumIntegral_tmp117 += sample.Integral_tmp117[2];
		#endif
		#endif
	}

	/* Detecting an Object Algorithm */
	if(flag_algorithmDO == 1)
	{
		/* Mean value in initial state */
		if(countDO < n_samplingLR)
		{
			/* Shifting data to next arrays */
			for(uint8_t i = 0; i < n_samplingLR-1; i ++)
			{
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				sample.DOinit_fdcQ1[(n_samplingLR-1)-i] = sample.DOinit_fdcQ1[(n_samplingLR-2)-i];
				#endif
				#if FDC_ADD
				sample.DOinit_fdcQ2[(n_samplingLR-1)-i] = sample.DOinit_fdcQ2[(n_samplingLR-2)-i];
				#endif
				#endif
			}

			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			sample.DOinit_fdcQ1[0] = sample.sumIntegral_fdcQ1;
			#endif
			#if FDC_ADD
			sample.DOinit_fdcQ2[0] = sample.sumIntegral_fdcQ2;
			#endif
			#endif

			/* Mean value */
			for(uint8_t i = 0; i < n_samplingLR; i ++)
			{
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					#if FDC
					sample.sumDO_fdcQ1 = 0;
					#endif
					#if FDC_ADD
					sample.sumDO_fdcQ2 = 0;
					#endif
					#endif
				}
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				sample.sumDO_fdcQ1 += sample.DOinit_fdcQ1[i];
				#endif
				#if FDC_ADD
				sample.sumDO_fdcQ2 += sample.DOinit_fdcQ2[i];
				#endif
				#endif
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			sample.meanDOinit_fdcQ1 = sample.sumDO_fdcQ1*0.0125;
			#endif
			#if FDC_ADD
			sample.meanDOinit_fdcQ2 = sample.sumDO_fdcQ2*0.0125;
			#endif
			#endif

			countDO ++;

			/* STD in initial state */
			for(uint8_t i = 0; i < n_samplingLR; i ++)
			{
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					#if FDC
					sample.sumSTDDO_fdcQ1 = 0;
					#endif
					#if FDC_ADD
					sample.sumSTDDO_fdcQ2 = 0;
					#endif
					#endif
				}
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				sample.sumSTDDO_fdcQ1 += (sample.DOinit_fdcQ1[i]-sample.meanDOinit_fdcQ1)*(sample.DOinit_fdcQ1[i]-sample.meanDOinit_fdcQ1);
				#endif
				#if FDC_ADD
				sample.sumSTDDO_fdcQ2 += (sample.DOinit_fdcQ2[i]-sample.meanDOinit_fdcQ2)*(sample.DOinit_fdcQ2[i]-sample.meanDOinit_fdcQ2);
				#endif
				#endif
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			sample.stdDOinit_fdcQ1 = sqrt(sample.sumSTDDO_fdcQ1*0.0125);
			if (countDO == n_samplingLR)
			{
				flag_DOinit = DriftSeedDetectionWindow(sample.DOinit_fdcQ1,
						sample.DO_fdcQ1, &sample.meanDO_fdcQ1, &sample.sumDO_fdcQ1);
			}
			#endif
			#if FDC_ADD
			sample.stdDOinit_fdcQ2 = sqrt(sample.sumSTDDO_fdcQ2*0.0125);
			#endif
			#endif
		}
		else if(countDO >= n_samplingLR)
		{
			for(uint8_t i = 0; i < 10-1; i ++)
			{
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				sample.DO_fdcQ1[(10-1)-i] = sample.DO_fdcQ1[(10-2)-i];
				#endif
				#if FDC_ADD
				sample.DO_fdcQ2[(10-1)-i] = sample.DO_fdcQ2[(10-2)-i];
				#endif

				#if Drift_Compensation_Proximity
				sample.DO_RfdcQ1[(10-1)-i] = sample.DO_RfdcQ1[(10-2)-i];
				#endif
				#endif
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			sample.DO_fdcQ1[0] = sample.sumIntegral_fdcQ1;
			#endif
			#if FDC_ADD
			sample.DO_fdcQ2[0] = sample.sumIntegral_fdcQ2;
			#endif

			#if Drift_Compensation_Proximity
			sample.DO_RfdcQ1[0] = sample.sumIntegral_RfdcQ1;
			#endif
			#endif

			/* Mean value */
			for(uint8_t i = 0; i < 10; i ++)
			{
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					#if FDC
					sample.sumDO_fdcQ1 = 0;
					#endif
					#if FDC_ADD
					sample.sumDO_fdcQ2 = 0;
					#endif

					#if Drift_Compensation_Proximity
					sample.sumDO_RfdcQ1 = 0;
					#endif
					#endif
				}

				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				sample.sumDO_fdcQ1 += sample.DO_fdcQ1[i];
				#endif
				#if FDC_ADD
				sample.sumDO_fdcQ2 += sample.DO_fdcQ2[i];
				#endif
				#if Drift_Compensation_Proximity
				sample.sumDO_RfdcQ1 += sample.DO_RfdcQ1[i];
				#endif
				#endif
			}

			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			sample.meanDO_fdcQ1 = sample.sumDO_fdcQ1 * 0.1;
			#endif
			#if FDC_ADD
			sample.meanDO_fdcQ2 = sample.sumDO_fdcQ2 * 0.1;
			#endif

			#if Drift_Compensation_Proximity
			sample.meanDO_RfdcQ1 = sample.sumDO_RfdcQ1 * 0.1;
			#endif
			#endif
		}

		/* STD after initial state */
		for(uint8_t i = 0; i < 10; i ++)
		{
			if(i == 0)
			{
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				sample.sumSTDDO_fdcQ1 = 0;
				#endif
				#if FDC_ADD
				sample.sumSTDDO_fdcQ2 = 0;
				#endif
				#if Drift_Compensation_Proximity
				sample.sumSTDDO_RfdcQ1 = 0;
				#endif
				#endif
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			sample.sumSTDDO_fdcQ1 += (sample.DO_fdcQ1[i]-sample.meanDO_fdcQ1)*(sample.DO_fdcQ1[i]-sample.meanDO_fdcQ1);
			#endif
			#if FDC_ADD
			sample.sumSTDDO_fdcQ2 += (sample.DO_fdcQ2[i]-sample.meanDO_fdcQ2)*(sample.DO_fdcQ2[i]-sample.meanDO_fdcQ2);
			#endif
			#if Drift_Compensation_Proximity
			sample.sumSTDDO_RfdcQ1 += (sample.DO_RfdcQ1[i]-sample.meanDO_RfdcQ1)*(sample.DO_RfdcQ1[i]-sample.meanDO_RfdcQ1);
			#endif
			#endif
		}

		#if (Operation_Single || Operation_Double || Operation_Quadruple)
		#if FDC
		sample.stdDO_fdcQ1 = sqrt(sample.sumSTDDO_fdcQ1*0.1);
		#endif
		#if FDC_ADD
		sample.stdDO_fdcQ2 = sqrt(sample.sumSTDDO_fdcQ2*0.1);
		#endif
		#if Drift_Compensation_Proximity
		sample.stdDO_RfdcQ1 = sqrt(sample.sumSTDDO_RfdcQ1*0.1);
		#endif
		#endif

		/* Judging Proximity detection */
		#if (Operation_Single || Operation_Double || Operation_Quadruple)
		#if FDC
        /* RAW policy runs before filtering. Integral statistics remain for
         * legacy distance/fusion only and must never overwrite RAW idle. */
		#endif
		#if FDC_ADD
		if((sample.meanDO_fdcQ2 - 1.0E+07) >= 1.55*(sample.stdDO_fdcQ2 + sample.stdDOinit_fdcQ2))
		{
			/* Detecting an object */
			state.idle_fdcQ2 = 1;
		}
		else
		{
			/* Idle state */
			state.idle_fdcQ2 = 0;
		}
		#endif

		#if Drift_Compensation_Proximity
		//if(((sample.stdDO_RfdcQ1 < 5*(sample.stdDO_fdcQ1)) && (state.idle_fdc)))
		//if(((sample.meanDO_RfdcQ1 - 1.0E+07 >= 4.0*sample.stdDO_RfdcQ1) && (state.idle_fdc)))
		if((sample.stdDO_RfdcQ1 < 4*(sample.stdDO_fdcQ1)) && (sample.meanDO_RfdcQ1 - 1.0E+07 >= 4.0*sample.stdDO_RfdcQ1) && (state.idle_fdc))
		{
			/* Proximity in Constant distance */
			state.stationaryObject_fdc = 1;
		}
		else
		{
			/* Object moving */
			state.stationaryObject_fdc = 0;
		}
		#endif
		#endif
	}

	/* Linear Regression */
    uint32_t learning_tick = HAL_GetTick();
    /* Legacy periods are 100 Hz units, independent of acquisition rate. */
    if (!drift_pair_tick_valid ||
        (uint32_t)(learning_tick - drift_pair_tick) >= (uint32_t)peroid_sampling * 10U)
    {
        drift_pair_tick_valid = 1U;
        drift_pair_tick = learning_tick;
        count_sampling = 0;
        uint32_t learning_input0 = 0U, learning_input1 = 0U;
        uint8_t q1_learning_due = DriftApproachGateTakePair(&drift_approach_gate,
                learning_tick, fdc.ch300, fdc.ch302, &learning_input0, &learning_input1);
		uint8_t q1_committed = 0U;
		if (q1_learning_due && n_data >= n_samplingLR)
		{
		    q1_committed = TrainDelayedQ1Pair(learning_input0, learning_input1, learning_tick);
		    q1_learning_due = 0U;
		}

		if ((q1_learning_due != 0U) ||
			(FDC_ADD && (state.idle_fdcQ2 == 0U)) ||
			(Drift_Compensation_Temperature && (state.movingObject_fdc != 0U)))
		{
			/* Shifting data to next arrays */
			for(uint8_t i = 0; i < n_samplingLR-1; i ++)
			{
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				if((!(state.idle_fdcQ1)) == 1)
				{
					sample.fdc300[(n_samplingLR-1)-i] = sample.fdc300[(n_samplingLR-2)-i];
					sample.fdc302[(n_samplingLR-1)-i] = sample.fdc302[(n_samplingLR-2)-i];
				}
				#endif
				#if FDC_ADD
				if((!(state.idle_fdcQ2)) == 1)
				{
					sample.fdc100[(n_samplingLR-1)-i] = sample.fdc100[(n_samplingLR-2)-i];
					sample.fdc102[(n_samplingLR-1)-i] = sample.fdc102[(n_samplingLR-2)-i];
				}
				#endif
				#if Drift_Compensation_Temperature
				if(state.movingObject_fdc == 1)
				{
					sample.tmp117[(n_samplingLR-1)-i] = sample.tmp117[(n_samplingLR-2)-i];
					sample.tmp117_f302[(n_samplingLR-1)-i] = sample.tmp117_f302[(n_samplingLR-2)-i];
				}
				#endif
				#endif

				/*-----------*/
				if(n_data == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					#if FDC
					if((!(state.idle_fdcQ1)) == 1)
					{
						sample.fdc300[i] = learning_input0;
						sample.fdc302[i] = learning_input1;
					}
					#endif
					#if FDC_ADD
					if((!(state.idle_fdcQ2)) == 1)
					{
						sample.fdc100[i] = fdc.ch100;
						sample.fdc102[i] = fdc.ch102;
					}
					#endif
					#if Drift_Compensation_Temperature
					if(state.movingObject_fdc == 1)
					{
						sample.tmp117[i] = tmp117.data;
						sample.tmp117_f302[i] = fdc.ch302;
					}
					#endif
					#endif
				}
				/*-----------*/
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			if((!(state.idle_fdcQ1)) == 1)
			{
				sample.fdc300[0] = learning_input0;
				sample.fdc302[0] = learning_input1;
			}
			#endif
			#if FDC_ADD
			if((!(state.idle_fdcQ2)) == 1)
			{
				sample.fdc100[0] = fdc.ch100;
				sample.fdc102[0] = fdc.ch102;
			}
			#endif
			#if Drift_Compensation_Temperature
			if(state.movingObject_fdc == 1)
			{
				sample.tmp117[0] = tmp117.data;
				sample.tmp117_f302[0] = fdc.ch302;
			}
			#endif
			#endif

			/* Mean value */
			for(uint8_t i = 0; i < n_samplingLR; i ++)
			{
				/* Reset the SUM variables to zero */
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					#if FDC
					if((!(state.idle_fdcQ1)) == 1)
					{
						sample.sum_fdc300 = 0;
						sample.sum_fdc302 = 0;
					}
					#endif
					#if FDC_ADD
					if((!(state.idle_fdcQ2)) == 1)
					{
						sample.sum_fdc100 = 0;
						sample.sum_fdc102 = 0;
					}
					#endif
					#if Drift_Compensation_Temperature
					if(state.movingObject_fdc == 1)
					{
						sample.sum_tmp117 = 0;
						sample.sum_tmp117_f302 = 0;
					}
					#endif
					#endif
				}
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				if((!(state.idle_fdcQ1)) == 1)
				{
					sample.sum_fdc300 += sample.fdc300[i];
					sample.sum_fdc302 += sample.fdc302[i];
				}
				#endif
				#if FDC_ADD
				if((!(state.idle_fdcQ2)) == 1)
				{
					sample.sum_fdc100 += sample.fdc100[i];
					sample.sum_fdc102 += sample.fdc102[i];
				}
				#endif
				#if Drift_Compensation_Temperature
				if(state.movingObject_fdc == 1)
				{
					sample.sum_tmp117 += sample.tmp117[i];
					sample.sum_tmp117_f302 += sample.tmp117_f302[i];
				}
				#endif
				#endif
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			if((!(state.idle_fdcQ1)) == 1)
			{
				sample.mean_fdc300 = sample.sum_fdc300 * 0.0125;
				sample.mean_fdc302 = sample.sum_fdc302 * 0.0125;
			}
			#endif
			#if FDC2214_ADD
			if((!(state.idle_fdcQ2)) == 1)
			{
				sample.mean_fdc100 = sample.sum_fdc100 * 0.0125;
				sample.mean_fdc102 = sample.sum_fdc102 * 0.0125;
			}
			#endif
			#if Drift_Compensation_Temperature
			if(state.movingObject_fdc == 1)
			{
				sample.mean_tmp117 = sample.sum_tmp117 * 0.02;
				sample.mean_tmp117_f302 = sample.sum_tmp117_f302 * 0.02;
			}
			#endif
			#endif


			/* Error */
			for(uint8_t i = 0; i < n_samplingLR; i ++)
			{
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					#if FDC
					if((!(state.idle_fdcQ1)) == 1)
					{
						dComps.sumNumnerator_fdcQ1 = 0;
						dComps.sumDenominator_fdcQ1 = 0;
						drift_model_sensor_variance = 0;
					}
					#endif
					#if FDC_ADD
					if((!(state.idle_fdcQ2)) == 1)
					{
						dComps.sumNumnerator_fdcQ2 = 0;
						dComps.sumDenominator_fdcQ2 = 0;
					}
					#endif
					#if Drift_Compensation_Temperature
					if(state.movingObject_fdc == 1)
					{
						dComps.sumNumnerator_tmp117 = 0;
						dComps.sumDenominator_tmp117 = 0;
					}
					#endif
					#endif
				}

				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				if((!(state.idle_fdcQ1)) == 1)
				{
					#if 0
					sample.err_fdc300[i] = abs(sample.fdc300[i] - sample.mean_fdc300);
					sample.err_fdc302[i] = abs(sample.fdc302[i] - sample.mean_fdc302);
					#endif

					#if 1
						sample.err_fdc300[i] = (int64_t)sample.fdc300[i] - (int64_t)sample.mean_fdc300;
						sample.err_fdc302[i] = (int64_t)sample.fdc302[i] - (int64_t)sample.mean_fdc302;
					#endif
				}
				#endif
				#if FDC_ADD
				if((!(state.idle_fdcQ2)) == 1)
				{
					#if 0
					sample.err_fdc100[i] =  abs(sample.fdc100[i] - sample.mean_fdc100);
					sample.err_fdc102[i] =  abs(sample.fdc102[i] - sample.mean_fdc102);
					#endif

					#if 1
					sample.err_fdc100[i] = (sample.fdc100[i] - sample.mean_fdc100);
					sample.err_fdc102[i] = (sample.fdc102[i] - sample.mean_fdc102);
					#endif
				}
				#endif
				#if Drift_Compensation_Temperature
				if(state.movingObject_fdc == 1)
				{
					sample.err_tmp117[i] = abs(sample.tmp117[i] - sample.mean_tmp117);
					sample.err_tmp117_f302[i] = abs(sample.tmp117_f302[i] - sample.mean_tmp117_f302);
				}
				#endif
				#endif

				/* Necessary to fill array of sum_XXXX */
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				#if FDC
				if((!(state.idle_fdcQ1)) == 1)
				{
					dComps.sumNumnerator_fdcQ1 += (sample.err_fdc300[i]*sample.err_fdc302[i]);
					dComps.sumDenominator_fdcQ1 += (sample.err_fdc302[i]*sample.err_fdc302[i]);
						drift_model_sensor_variance += sample.err_fdc300[i]*sample.err_fdc300[i];
				}
				#endif
				#if FDC2214_ADD
				if((!(state.idle_fdcQ2)) == 1)
				{
					dComps.sumNumnerator_fdcQ2 += (sample.err_fdc100[i]*sample.err_fdc101[i]);
					dComps.sumDenominator_fdcQ2 += (sample.err_fdc102[i]*sample.err_fdc102[i]);
				}
				#endif
				#if Drift_Compensation_Temperature
				if(state.movingObject_fdc == 1)
				{
					dComps.sumNumnerator_tmp117 += (sample.err_tmp117_f302[i]*sample.err_tmp117[i]);
					dComps.sumDenominator_tmp117 += (sample.err_tmp117[i]*sample.err_tmp117[i]);
				}
				#endif
				#endif
			}

		}

		/* Linear Regression When an Object Approaches */
		#if Drift_Compensation_Proximity
		//state.stationaryObject_fdc = ((sample.stdDO_fdcQ1 < 0.7*(sample.stdDOinit_fdcQ1 + sample.stdDO_fdcQ1)) && state.idle_fdc);
		if((state.stationaryObject_fdc) == 1)
		{
			/* Shifting data to next arrays */
			for(uint8_t i = 0; i < n_PMsamplingLR-1; i ++)
			{
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				if(state.stationaryObject_fdc == 1)
				{
					sample.PMfdc300[(n_PMsamplingLR-1)-i] = sample.PMfdc300[(n_PMsamplingLR-2)-i];
					sample.PMfdc302[(n_PMsamplingLR-1)-i] = sample.PMfdc302[(n_PMsamplingLR-2)-i];
				}
				#endif

				/*-----------*/
				if(n_PMdata == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					if(state.stationaryObject_fdc == 1)
					{
						sample.PMfdc300[i] = fdc.ch300;
						sample.PMfdc302[i] = fdc.ch302;
					}
					#endif
				}
				/*-----------*/
			}
			n_PMdata = 1;


			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			if(state.stationaryObject_fdc == 1)
			{
				sample.PMfdc300[0] = fdc.ch300;
				sample.PMfdc302[0] = fdc.ch302;
			}
			#endif

			/* Mean value */
			for(uint8_t i = 0; i < n_PMsamplingLR; i ++)
			{
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					if(state.stationaryObject_fdc == 1)
					{
						sample.sum_PMfdc300 = 0;
						sample.sum_PMfdc302 = 0;
					}
					#endif
				}
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				if(state.stationaryObject_fdc == 1)
				{
					sample.sum_PMfdc300 += sample.PMfdc300[i];
					sample.sum_PMfdc302 += sample.PMfdc302[i];
				}
				#endif
			}
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			if(state.stationaryObject_fdc == 1)
			{
				sample.mean_PMfdc300 = sample.sum_PMfdc300 * 0.025;
				sample.mean_PMfdc302 = sample.sum_PMfdc302 * 0.025;
			}
			#endif


			/* Error */
			for(uint8_t i = 0; i < n_PMsamplingLR; i ++)
			{
				/* Reset the variables to zero */
				if(i == 0)
				{
					#if (Operation_Single || Operation_Double || Operation_Quadruple)
					if(state.stationaryObject_fdc == 1)
					{
						dComps.sumNumnerator_PMfdcQ1 = 0;
						dComps.sumDenominator_PMfdcQ1 = 0;
					}
					#endif
				}
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				if(state.stationaryObject_fdc == 1)
				{
					sample.err_PMfdc300[i] = abs(sample.PMfdc300[i] - sample.mean_PMfdc300);
					sample.err_PMfdc302[i] = abs(sample.PMfdc302[i] - sample.mean_PMfdc302);
				}
				#endif

				/* Necessary to fill array of sum_XXXX */
				#if (Operation_Single || Operation_Double || Operation_Quadruple)
				if(state.stationaryObject_fdc == 1)
				{
					dComps.sumNumnerator_PMfdcQ1 += (sample.err_PMfdc300[i]*sample.err_PMfdc300[i]);
					dComps.sumDenominator_PMfdcQ1 += (sample.err_PMfdc302[i]*sample.err_PMfdc302[i]);
				}
				#endif
			}

			dComps.PMfdcCoeff_Q1 = 1.5*(dComps.sumNumnerator_PMfdcQ1*10000/dComps.sumDenominator_PMfdcQ1);
			#if 1
			if(dComps.PMfdcCoeff_Q1 < 0)
			{
				dComps.PMfdcCoeff_Q1 = 0;
			}
			else if(dComps.PMfdcCoeff_Q1 > 9990)
			{
				dComps.PMfdcCoeff_Q1 = 9990;
			}
			#endif
			dComps.PMfdcOffset_Q1 = sample.mean_fdc300-((float)dComps.PMfdcCoeff_Q1*0.0001)*sample.mean_fdc302;
		}
		else if(state.stationaryObject_fdc == 0)
		{
			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			dComps.PMfdcCoeff_Q1 = dComps.fdcCoeff_Q1;
			dComps.PMfdcOffset_Q1 = sample.fdc300[0] - ((float)dComps.PMfdcCoeff_Q1*0.0001)*sample.fdc302[0];

			sample.sum_PMfdc300 = 0;
			sample.sum_PMfdc302 = 0;
			#endif

			n_PMdata = 0;
		}
		#endif

		/* Mobile SoC uses the CH0/CH1 pair only. Preserve its model while an
		 * object is present; a disabled Q2 must not advance Q1 learning. */
		if (q1_learning_due != 0U)
		{
		drift_eval_learning_updates++;
		drift_learning_input_ch0 = learning_input0;
		drift_learning_input_ch1 = learning_input1;
		if(n_data < n_samplingLR)
		{
			/* Before Sampling 50 data */
			n_data ++;

			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			dComps.fdcCoeff_Q1 = Gain1st_Compensation1*10000;
			dComps.fdcOffset_Q1 = DriftOffsetFromMeans(sample.fdc300[0], dComps.fdcCoeff_Q1, sample.fdc302[0]);
			//dComps.fdcOffset_Q1 = fdc.ch300-((float)dComps.fdcCoeff_Q1*0.0001)*fdc.ch302;
			#endif
			#if FDC_ADD
			dComps.fdcCoeff_Q2= Gain1st_Compensation2*10000;
			dComps.fdcOffset_Q2 = sample.fdc100[0]-((float)dComps.fdcCoeff_Q2*0.0001)*sample.fdc102[0];
			#endif
			#if Drift_Compensation_Temperature
			dComps.tmp117_fdcCoeff_Q1 = (dComps.sumNumnerator_tmp117*100000/dComps.sumDenominator_tmp117);
			dComps.tmp117_fdcOffset_Q1 = sample.fdc302[0]-((float)dComps.tmp117_fdcCoeff_Q1*0.00001)*sample.tmp117[0];
			#endif
			#endif

			//flag_di = 0;
		}
		else if(n_data >= n_samplingLR)
		{
			/* After Sampling 50 data */

			#if (Operation_Single || Operation_Double || Operation_Quadruple)
			#if FDC
			if (DriftFitGainScaled(dComps.sumNumnerator_fdcQ1,
					dComps.sumDenominator_fdcQ1, &dComps.fdcCoeff_Q1) == 0U)
			{
				drift_eval_zero_variance_skips++;
			}
			/* A constant reference prevents a new gain fit, not trusted idle
			 * baseline centering. Keep gain and update its paired offset. */
			dComps.fdcOffset_Q1 = DriftOffsetFromMeans(sample.mean_fdc300,
					dComps.fdcCoeff_Q1, sample.mean_fdc302);
			#endif
			#if FDC_ADD
			dComps.fdcCoeff_Q2 = (dComps.sumNumnerator_fdcQ2*10000/dComps.sumDenominator_fdcQ2);
			if(dComps.fdcCoeff_Q2 > 9999)
			{
				dComps.fdcCoeff_Q2 = 9999;
			}
			else if(dComps.fdcCoeff_Q2 < 0)
			{
				dComps.fdcCoeff_Q2 = 0;
			}
			dComps.fdcOffset_Q2 = sample.mean_fdc100-((float)dComps.fdcCoeff_Q2*0.0001)*sample.mean_fdc102;
			#endif
			#if Drift_Compensation_Temperature
			dComps.tmp117_fdcCoeff_Q1 = (dComps.sumNumnerator_tmp117*100000/dComps.sumDenominator_tmp117);
			dComps.tmp117_fdcOffset_Q1 = sample.mean_tmp117_f302-((float)dComps.tmp117_fdcCoeff_Q1*0.00001)*sample.tmp117[0];
			#endif
			#endif


			/* Low frequency */
			#if 0
			//peroid_sampling = 1000;
			#endif

			/* High frequency */
			#if 0
			peroid_sampling = 80;
			#endif

			#if 1
			if ((state.idle_fdcQ1 == 0U) ||
				(FDC_ADD && (state.idle_fdcQ2 == 0U)) ||
				(Drift_Compensation_Temperature && (state.movingObject_fdc != 0U)))
			{
				peroid_sampling = 80;
			}
			else
			{
				peroid_sampling = 400;
			}
			#endif

			/*  */
			flag_di = 1;
			flag_algorithmDO = 1;
		}
		}
		else
		{
			if (!q1_committed) drift_eval_frozen_samples++;
		}
	}
	count_sampling ++;

	#if (Operation_Single || Operation_Double || Operation_Quadruple)
	#if FDC
	dComps.fdc_Q1 = TwinSensorCompensation(&fdc.ch300, &fdc.ch302, (((double)dComps.fdcCoeff_Q1)*0.0001), dComps.fdcOffset_Q1) + 2*Offset1;
	//dComps.Vfdc_Q1 = TwinSensorCompensation(&fdc.ch300, &fdc.ch302, (((float)dComps.PMfdcCoeff_Q1)*0.0001), dComps.PMfdcOffset_Q1) + 2*Offset1;
	#endif
	#if FDC_ADD
	dComps.fdc_Q2 = TwinSensorCompensation(&fdc.ch100, &fdc.ch102, (((float)dComps.fdcCoeff_Q2)*0.0001), dComps.fdcOffset_Q2) + 2*Offset2;
	#endif
	#endif

	#if Drift_Compensation_Temperature
	dComps.Tfdc_tmp[1] = TemperatureCompensation(&fdc.ch300, &tmp117.data, (((float)dComps.tmp117_fdcCoeff_Q1)*0.00001), dComps.tmp117_fdcOffset_Q1 )+ 2*Offset1;
	//dComps.Tfdc_tmp[1] = fdc.ch300 + (fdc.ch302 - dComps.Tfdc_tmp[0]);
	dComps.Tfdc_Q1 = 2*Offset1 - (sample.sumIntegral_tmp117 - 1E+06);
	#endif

	/* Debugging the Compensated Value */
	#if 1
	SplitData(dComps.fdc_Q1, dComps.temp_fdcQ1);
	SplitData(dComps.fdc_Q2, dComps.temp_fdcQ2);
	#endif

	#if 1
	if(n_data >= n_samplingLR)
	{
		fusion.mean_CapRaw_Q1 = (uint32_t)(
				0.98f * (float)fusion.prev_CapRaw_Q1 +
				0.02f * (float)abs(cvf.MeanData_Q1[0] - dComps.fdc_Q1));
		fusion.prev_CapRaw_Q1 = fusion.mean_CapRaw_Q1;

		fusion.temp_b_uc_Q1 = (uint64_t)(
				abs(cvf.tmpCoeff_Q1 * 0.0001f * fusion.mean_CapRaw_Q1));
		if(fusion.temp_b_uc_Q1 > 0xFFFFFFFFULL)
		{
			fusion.temp_b_uc_Q1 = 0xFFFFFFFFULL;
		}
	}
	#endif
}



#if Drift_Compensation
int64_t TwinSensorCompensation(uint32_t* input_sensor, uint32_t* input_compensation, double gain_1st, int64_t offset)
{
	return DriftCompensatedResidual(*input_sensor, *input_compensation, gain_1st, offset);
}
#endif

#if Drift_Compensation_Temperature
int64_t TemperatureCompensation(uint32_t* input_sensor, uint32_t* temperature, float coefficient, int64_t offset)
{
	int32_t merge_sensor = 0;
	int32_t merge_compensation = 0;
	int64_t result = 0;

	merge_sensor = *input_sensor;
	merge_compensation = *temperature;

	result = (merge_sensor - coefficient*(merge_compensation) - offset);
	return result;
}
#endif

void DriftCompensation(void)
{
	uint32_t started = EvaluationNowUs();
    const Sensor1V1AutoSigmaResult *v1 = Sensor1V1AutoSigmaStep(fdc.raw_ch0,fdc.raw_ch1);
    fdc.ch300 = (uint32_t)v1->filtered0;
    fdc.ch302 = (uint32_t)v1->filtered1;
    kalman.fdc300 = v1->filter_input0;
    kalman.fdc302 = v1->filter_input1;
    drift_audit_kalman_x0_before_bits = v1->filter_x0_before;
    drift_audit_kalman_p0_before_bits = v1->filter_p0_before;
    memcpy((void *)&drift_audit_kalman_gain0_bits,&v1->filter_gain0,sizeof(uint32_t));
    dComps.fdc_Q1 = v1->pre_count;
    dComps.fdcCoeff_Q1 = v1->coefficient;
    dComps.fdcOffset_Q1 = v1->offset_count;
    dComps.sumNumnerator_fdcQ1 = v1->numerator;
    dComps.sumDenominator_fdcQ1 = v1->denominator;
    drift_only_fdc_Q1 = v1->pre_count;
    state.idle_fdcQ1 = (uint8_t)v1->object;
    n_data = (uint8_t)v1->n_data;
    count_sampling = (uint16_t)v1->sampling_count;
    peroid_sampling = (uint16_t)v1->period;
    flag_di = (uint8_t)v1->di_ready;
    flag_algorithmDO = (uint8_t)v1->algorithm_ready;
    countDO = (uint8_t)v1->do_count;
    sample.stdDOinit_fdcQ1 = (uint32_t)v1->init_sigma;
    sample.stdDO_fdcQ1 = (uint32_t)v1->recent_sigma;
    sample.meanDO_fdcQ1 = (int32_t)v1->detector_mean;
    sample.mean_fdc300 = v1->mean0;
    sample.mean_fdc302 = v1->mean1;
    drift_detection_baseline_sigma = (uint32_t)v1->sigma_used;
    drift_detection_recent_sigma = (uint32_t)v1->recent_sigma;
    drift_detection_entry_threshold = (uint32_t)v1->threshold;
    drift_eval_object_reference = 10000000U; /* normalized DO=5e7-comp */
    drift_learning_guard.allowed = (uint8_t)v1->allowed;
    drift_learning_guard.reason = v1->object ? DRIFT_LEARN_OBJECT : 0U;
    if (v1->source_event) {
        drift_eval_learning_updates++;
        drift_learning_input_ch0 = fdc.ch300;
        drift_learning_input_ch1 = fdc.ch302;
    }
    if (v1->object) drift_eval_frozen_samples++;
    drift_eval_zero_variance_skips = v1->zero_variance_skips;

	drift_eval_valid_cycles++;
	drift_eval_last_us = EvaluationNowUs() - started;
	if (drift_eval_last_us > drift_eval_max_us)
	{
		drift_eval_max_us = drift_eval_last_us;
	}
}

#if (CalibrationDistance || CurveFitting)
void CalibrateDistance(void)
{
    /* New fitted model followed by its affine correction, applied exactly once. */
    dist.Q1 = Sensor1CalibratedCapDistance(dComps.fdc_Q1, INT32_MAX);

    #if FDC_ADD
    dist.Q2 = (12.89*exp(-0.000007192*(dComps.fdc_Q2 - 2*Offset1)) + 220.1*exp(0.0001462*(dComps.fdc_Q2 - 2*Offset2)));
    #endif
}

void CalibrateDistanceNormalized(void)
{
    /* Match the active calibrated CAP model, retaining the existing350mm limit. */
    dist.Normalized_Q1 = Sensor1CalibratedCapDistance(dComps.fdc_Q1, 350U);

    #if FDC_ADD
    dist.Q2 = (12.89*exp(-0.000007192*(dComps.fdc_Q2 - 2*Offset1)) + 220.1*exp(0.0001462*(dComps.fdc_Q2 - 2*Offset2)));
    #endif
}
#endif

#if KalmanFilters
void InitializeKalmanFilters(void)
{
	/* Kalman Filter */
	#if KalmanFilters
	kalman.A_Q1 = 1;
	kalman.H_Q1 = 1;
	kalman.Q_Q1 = 0.1;
	kalman.R_Q1 = 1;
	kalman.x_Q1 = 2.74E+07;
	kalman.P_Q1 = 1;

	kalman.A_Q1cmp = 1;
	kalman.H_Q1cmp = 1;
	kalman.Q_Q1cmp = 0.1;
	kalman.R_Q1cmp = 1;
	kalman.x_Q1cmp = 4.85E+07;
	kalman.P_Q1cmp = 1;

	kalman.A_Q2 = 1;
	kalman.H_Q2 = 1;
	kalman.Q_Q2 = 0.01;
	kalman.R_Q2 = 1;
	kalman.x_Q2 = 5.6E+07;
	kalman.P_Q2 = 1;

	kalman.A_Q2cmp = 1;
	kalman.H_Q2cmp = 1;
	kalman.Q_Q2cmp = 0.1;
	kalman.R_Q2cmp = 1;
	kalman.x_Q2cmp = 3.79E+07;
	kalman.P_Q2cmp = 1;
	#endif
}
#endif

uint32_t KalmanFilter(uint32_t input, uint8_t A, float H, float Q,
					  float R, float *x, float *P)
{
	if ((x == NULL) || (P == NULL))
	{
		return 0U;
	}

	float x_local = *x;
	float P_local = *P;
	if (x == &kalman.x_Q1)
	{
		uint32_t bits;
		memcpy(&bits, &x_local, sizeof(bits));
		drift_audit_kalman_x0_before_bits = bits;
		memcpy(&bits, &P_local, sizeof(bits));
		drift_audit_kalman_p0_before_bits = bits;
	}
	float xp = (float)A * x_local;
	float Pp = (float)A * P_local * (float)A + Q;
	float denominator = H * Pp * H + R;
	float K = 0.0f;

	if (denominator > 1.0e-9f)
	{
		K = (Pp * H) / denominator;
	}
	if (x == &kalman.x_Q1)
	{
		uint32_t bits;
		memcpy(&bits, &K, sizeof(bits));
		drift_audit_kalman_gain0_bits = bits;
	}

	x_local = xp + K * ((float)input - H * xp);
	P_local = (1.0f - K * H) * Pp;

	if (!isfinite(x_local) || (x_local < 0.0f))
	{
		x_local = 0.0f;
	}
	else if (x_local > 4294967295.0f)
	{
		x_local = 4294967295.0f;
	}
	if (!isfinite(P_local) || (P_local < 0.0f))
	{
		P_local = 0.0f;
	}

	*x = x_local;
	*P = P_local;
	return (uint32_t)x_local;
}

uint32_t SlewRateLimiter(uint32_t input, uint32_t *prev,
						 uint8_t *initialized, uint32_t max_step)
{
	if ((prev == NULL) || (initialized == NULL))
	{
		return input;
	}
	if (*initialized == 0U)
	{
		*prev = input;
		*initialized = 1U;
		return input;
	}

	int64_t delta = (int64_t)input - (int64_t)(*prev);
	uint32_t limited = input;
	if (delta > (int64_t)max_step)
	{
		limited = *prev + max_step;
	}
	else if (delta < -(int64_t)max_step)
	{
		limited = (*prev > max_step) ? (*prev - max_step) : 0U;
	}
	*prev = limited;
	return limited;
}

static void UpdateFusionCapSignal(void)
{
    /* v3 backup: instantaneous post-EMA delta, on the same sample used for
     * CAP distance. Sensor1 has Q1 only; do not wait for a Q2 learner. */
    if (n_data < n_samplingLR) return;
    const int64_t delta =
            (int64_t)cvf.MeanData_Q1[0] - (int64_t)dComps.fdc_Q1;
    fusion.mean_CapRaw_Q1 = (uint32_t)((delta >= 0) ? delta : -delta);
    fusion.prev_CapRaw_Q1 = fusion.mean_CapRaw_Q1;
    fusion.temp_b_uc_Q1 = fusion.mean_CapRaw_Q1;
}

void ApplyEMA2Filter(void)
{
	if (ema2_initialized == 0U)
	{
		ema2_fdc_Q1 = dComps.fdc_Q1;
		ema2_accumulator_Q1 = (double)dComps.fdc_Q1;
		ema2_selected_alpha = EMA2_ALPHA_SLOW;
		ema2_initialized = 1U;
	}
	else
	{
		float alpha;
		ema2_fdc_Q1 = EMA2PrecisionStep(&ema2_accumulator_Q1, dComps.fdc_Q1,
				EMA2_DELTA_LOW, EMA2_DELTA_HIGH, EMA2_ALPHA_SLOW,
				EMA2_ALPHA_FAST, &alpha);
		ema2_selected_alpha = alpha;
	}
	dComps.fdc_Q1 = ema2_fdc_Q1;
}

void RecordEvaluationTrace(uint32_t cycle_start)
{
    uint32_t sequence = eval_trace_sequence + 1U;
    volatile EvaluationTraceEntry *entry = &eval_trace[(sequence - 1U) % EVAL_TRACE_DEPTH];
    entry->sequence = 0U;
    __DMB();
    entry->tick_ms = HAL_GetTick();
    entry->raw_ch0 = mobile_fdc_raw_ch0;
    entry->raw_ch1 = mobile_fdc_raw_ch1;
    entry->filtered_ch0 = fdc.ch300;
    entry->filtered_ch1 = fdc.ch302;
    entry->pre_ema = drift_only_fdc_Q1;
    entry->post_ema = dComps.fdc_Q1;
    entry->alpha_ppm = (uint32_t)(ema2_selected_alpha * 1000000.0f + 0.5f);
    entry->flags = ((uint32_t)(fdc.ch0_valid != 0U)) |
            ((uint32_t)(fdc.ch1_valid != 0U) << 1) |
            ((uint32_t)(state.idle_fdcQ1 != 0U) << 2) |
            ((uint32_t)(n_data >= n_samplingLR) << 3) |
            ((uint32_t)(fusion.FusedDistance_Q1_valid != 0U) << 4) |
            ((uint32_t)(sensor1_drift_audit_schema == 8U) << 30) |
            ((uint32_t)(sensor1_drift_variant == 0x56314533U) << 31);
    entry->learning_updates = drift_eval_learning_updates;
    entry->coefficient = (int32_t)dComps.fdcCoeff_Q1;
    entry->offset = (int32_t)dComps.fdcOffset_Q1;
    entry->cap_mm = cvf.FilteredDistance_Q1;
    entry->fusion_mm = fusion.FusedDistance_Q1;
    entry->tof_mm = tof.distance[0];
    entry->total_us = EvaluationNowUs() - cycle_start;
    for (uint32_t i = 0U; i < EVAL_STAGE_COUNT; ++i)
        entry->stage_us[i] = eval_stage_elapsed_us[i];
    __DMB();
    entry->sequence = sequence;
    __DMB();
    eval_trace_sequence = sequence;
    RecordDriftModelAudit();
}

void RecordDriftModelAudit(void)
{
    static uint32_t previous_learning = 0U;
    static uint32_t previous_flags = 0U;
    static uint32_t previous_reset = 0U;
    static uint32_t last_tick = 0U;
    static int64_t previous_coefficient = 0;
    static int64_t previous_offset = 0;
    static uint32_t previous_signals[6] = {0U};
    static uint8_t previous_pair_valid = 0U;
    static uint32_t previous_signal_reset = 0U;
    uint32_t tick = HAL_GetTick();
    uint32_t flags = ((uint32_t)(fdc.ch0_valid != 0U)) |
            ((uint32_t)(fdc.ch1_valid != 0U) << 1) |
            ((uint32_t)(state.idle_fdcQ1 != 0U) << 2) |
            ((uint32_t)(n_data >= n_samplingLR) << 3) |
            ((uint32_t)(flag_di != 0U) << 4) |
            ((uint32_t)drift_learning_guard.reason << 8) |
            ((uint32_t)Sensor1V1AutoSigmaGet()->do_ready << 14) |
            ((uint32_t)drift_learning_guard.allowed << 15) |
            ((uint32_t)Sensor1V1AutoSigmaGet()->sigma_ready << 16) |
            ((uint32_t)Sensor1V1AutoSigmaGet()->sigma_safe << 17);
    uint8_t new_learning = (uint8_t)(drift_eval_learning_updates != previous_learning);
    uint32_t signals[6] = {mobile_fdc_raw_ch0, mobile_fdc_raw_ch1,
                          fdc.ch300, fdc.ch302, drift_only_fdc_Q1, dComps.fdc_Q1};
    uint32_t large_step_flags = 0U;
    if ((flags & 3U) == 3U && previous_pair_valid &&
        previous_signal_reset == drift_model_reset_count)
    {
        for (uint32_t i = 0U; i < 6U; ++i)
        {
            int64_t delta = (int64_t)signals[i] - (int64_t)previous_signals[i];
            if (delta > (int64_t)DRIFT_AUDIT_LARGE_STEP_COUNTS ||
                delta < -(int64_t)DRIFT_AUDIT_LARGE_STEP_COUNTS)
                large_step_flags |= 1U << i;
        }
    }
    for (uint32_t i = 0U; i < 6U; ++i) previous_signals[i] = signals[i];
    previous_pair_valid = (uint8_t)((flags & 3U) == 3U);
    previous_signal_reset = drift_model_reset_count;
    if (large_step_flags) drift_model_large_step_events++;
    if ((flags & 0x17U) == 0x17U && previous_reset == drift_model_reset_count &&
        (new_learning || dComps.fdcCoeff_Q1 != previous_coefficient ||
         dComps.fdcOffset_Q1 != previous_offset)) drift_model_object_learning_violations++;
    previous_coefficient = dComps.fdcCoeff_Q1;
    previous_offset = dComps.fdcOffset_Q1;
    if (drift_model_audit_sequence && !new_learning && !large_step_flags && flags == previous_flags &&
        previous_reset == drift_model_reset_count && tick - last_tick < 1000U) return;
    if ((flags & 0x10U) != 0U && dComps.sumDenominator_fdcQ1 > 0)
    {
        double gain = (double)dComps.sumNumnerator_fdcQ1 / (double)dComps.sumDenominator_fdcQ1;
        if (gain < 0.0) { flags |= 0x20U; if (new_learning) drift_model_gain_lower_hits++; }
        if (gain > 0.9999) { flags |= 0x40U; if (new_learning) drift_model_gain_upper_hits++; }
    }
    else if ((flags & 0x10U) != 0U) flags |= 0x80U;
    uint32_t sequence = drift_model_audit_sequence + 1U;
    volatile DriftModelAuditEntry *entry = &drift_model_audit[(sequence - 1U) % DRIFT_MODEL_AUDIT_DEPTH];
    entry->sequence = 0U;
    __DMB();
    entry->tick_ms = tick;
    entry->filtered_ch0 = fdc.ch300;
    entry->filtered_ch1 = fdc.ch302;
    entry->pre_ema = drift_only_fdc_Q1;
    entry->post_ema = dComps.fdc_Q1;
    entry->numerator = dComps.sumNumnerator_fdcQ1;
    entry->denominator = dComps.sumDenominator_fdcQ1;
    entry->sensor_variance = drift_model_sensor_variance;
    entry->offset = dComps.fdcOffset_Q1;
    entry->coefficient = (int32_t)dComps.fdcCoeff_Q1;
    entry->learning_updates = drift_eval_learning_updates;
    entry->flags = flags;
    entry->mean_ch0 = (uint32_t)sample.mean_fdc300;
    entry->mean_ch1 = (uint32_t)sample.mean_fdc302;
    entry->object_mean = (uint32_t)sample.meanDO_fdcQ1;
    entry->object_reference = 10000000U; /* DO=5e7-comp, fixed comp anchor4e7 */
    entry->drift_us = drift_eval_last_us;
    entry->model_reset_count = drift_model_reset_count;
    entry->raw_ch0 = mobile_fdc_raw_ch0;
    entry->raw_ch1 = mobile_fdc_raw_ch1;
    entry->kalman_input_ch0 = kalman.fdc300;
    entry->kalman_input_ch1 = kalman.fdc302;
    entry->kalman_x0_before_bits = drift_audit_kalman_x0_before_bits;
    entry->kalman_p0_before_bits = drift_audit_kalman_p0_before_bits;
    entry->kalman_gain0_bits = drift_audit_kalman_gain0_bits;
    entry->large_step_flags = large_step_flags;
    entry->sample_diagnostics = (uint32_t)mobile_fdc_raw_status |
            (((uint32_t)fdc.config_i2c_status & 3U) << 8) |
            (((uint32_t)fdc.ch0_i2c_status & 3U) << 10) |
            (((uint32_t)fdc.ch1_i2c_status & 3U) << 12) |
            (((uint32_t)fdc.ch0_reject_reason & 15U) << 16) |
            (((uint32_t)fdc.ch1_reject_reason & 15U) << 20) |
            ((uint32_t)fdc_diag_last_error << 24);
    entry->learning_input_ch0 = drift_learning_input_ch0;
    entry->learning_input_ch1 = drift_learning_input_ch1;
    /* Audit schema8: baseline sigma and threshold words carry IEEE float32
     * bits to retain sub-count precision; CAN0x41..0x44 is unchanged. */
    float sigma_audit = (float)Sensor1V1AutoSigmaGet()->sigma_used;
    float threshold_audit = (float)Sensor1V1AutoSigmaGet()->threshold;
    uint32_t sigma_audit_bits,threshold_audit_bits;
    memcpy(&sigma_audit_bits,&sigma_audit,sizeof(uint32_t));
    memcpy(&threshold_audit_bits,&threshold_audit,sizeof(uint32_t));
    entry->detection_baseline_sigma = sigma_audit_bits;
    entry->detection_recent_sigma = drift_detection_recent_sigma;
    entry->detection_entry_threshold = threshold_audit_bits;
    entry->noise_reference_samples = Sensor1V1AutoSigmaGet()->noise_count;
    __DMB();
    entry->sequence = sequence;
    __DMB();
    drift_model_audit_sequence = sequence;
    if (sequence > DRIFT_MODEL_AUDIT_DEPTH) drift_model_audit_overwritten++;
    previous_learning = drift_eval_learning_updates;
    previous_flags = flags & ~0xE0U;
    previous_reset = drift_model_reset_count;
    last_tick = tick;
}

uint32_t MergeData(uint8_t* temp_data)
{
	uint32_t result = 0;

	result = (temp_data[0] << 24)|(temp_data[1] << 16)|(temp_data[2] << 8)|(temp_data[3] << 0);

	return result;
}

uint32_t FDC_RawToFrequencyHz(uint32_t raw_data, uint8_t data_format,
							  uint32_t conversion_time_us)
{
	if ((data_format > 15U) || (conversion_time_us == 0U))
	{
		return 0U;
	}

	/* Verified reference rule: fixed-point divisor = 2^(15 - FORMAT). */
	uint8_t fractional_bits = (uint8_t)(15U - data_format);
	uint32_t format_scale = 1UL << fractional_bits;
	uint64_t denominator = 2ULL * conversion_time_us * format_scale;

	return (uint32_t)(((uint64_t)raw_data * 1000000ULL) / denominator);
}

uint32_t FDC_RawToCountMilli(uint32_t raw_data, uint8_t data_format)
{
	if (data_format > 15U)
	{
		return 0U;
	}

	uint32_t scale = 1UL << (15U - data_format);
	return (uint32_t)((((uint64_t)raw_data * 1000ULL) + (scale / 2U)) /
					  scale);
}

uint32_t FDC_ConversionTimeUs(uint8_t conversion_reg)
{
	uint8_t time_code =
			(conversion_reg & FDC_CONV_TIME_Msk) >> FDC_CONV_TIME_Pos;

	/* Viewer definition: 256 us * 2^TIME, with codes 8..15 capped. */
	if (time_code >= 8U)
	{
		return 65536U;
	}

	return 256UL << time_code;
}

void SplitData(uint32_t data_32bit, uint8_t *data_8bit)
{
	data_8bit[3] = (uint8_t)((data_32bit >> 0) & 0xFF);
	data_8bit[2] = (uint8_t)((data_32bit >> 8) & 0xFF);
	data_8bit[1] = (uint8_t)((data_32bit >> 16) & 0xFF);
	data_8bit[0] = (uint8_t)((data_32bit >> 24) & 0xFF);
}

#if 0
void SplitData_8Byte(int64_t data_64bit, uint8_t *data_8bit)
{
	data_8bit[0] = (uint8_t)((data_64bit >> 0) & 0xFF);
	data_8bit[1] = (uint8_t)((data_64bit >> 8) & 0xFF);
	data_8bit[2] = (uint8_t)((data_64bit >> 16) & 0xFF);
	data_8bit[3] = (uint8_t)((data_64bit >> 24) & 0xFF);
	data_8bit[4] = (uint8_t)((data_64bit >> 32) & 0xFF);
	data_8bit[5] = (uint8_t)((data_64bit >> 40) & 0xFF);
	data_8bit[6] = (uint8_t)((data_64bit >> 48) & 0xFF);
	data_8bit[7] = (uint8_t)((data_64bit >> 56) & 0xFF);
}
#endif

#if ToF
static uint8_t ToF_GetI2C1LineState(void)
{
	uint8_t lines = 0U;
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) lines |= 0x02U;
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) lines |= 0x01U;
	return lines;
}

void InitializeToF(void)
{
	VL53L1X_ERROR status = 0;
	uint8_t booted = 0U;
	uint32_t started;
	const uint16_t address = 0x29U << 1;

	/* Logical ToF0 uses I2C1 (PB6=SCL, PB7=SDA). NPX3110 remains on
	 * I2C3. XSHUT is pulled up on the ToF board, with no MCU control wire. */
	tof.address[0] = 0x29U;
	tof.state[0] = 0U;
	tof.state[1] = 0U;
	tof.distance[0] = 0U;
	tof_init_stage = 1U;
	HAL_Delay(20U);
	tof_line_before_request = ToF_GetI2C1LineState();
	if ((tof_line_before_request & 0x03U) != 0x03U)
	{
		tof_init_stage = 7U;
		tof_init_status = -7;
		goto failed;
	}
	started = HAL_GetTick();
	do
	{
		status = VL53L1X_BootState(&hi2c1, address, &booted);
		tof_line_after_request = ToF_GetI2C1LineState();
		if (status != 0)
		{
			tof_boot_hal_error = (uint8_t)hi2c1.ErrorCode;
			tof_boot_hal_state = (uint8_t)hi2c1.State;
			tof_boot_isr = (uint16_t)I2C1->ISR;
			break;
		}
		if ((status == 0) && (booted != 0U))
		{
			break;
		}
		HAL_Delay(5U);
	} while ((HAL_GetTick() - started) < 300U);
	if ((status != 0) || (booted == 0U))
	{
		tof_init_status = (status != 0) ? status : -7;
		goto failed;
	}

	tof_init_stage = 2U;
	status = VL53L1X_SensorInit(&hi2c1, address);
	if (status != 0) { tof_init_status = status; goto failed; }
	tof_init_stage = 3U;
	status = VL53L1X_SetDistanceMode(&hi2c1, address, 2U);
	if (status != 0) { tof_init_status = status; goto failed; }
	tof_init_stage = 4U;
	status = VL53L1X_StartRanging(&hi2c1, address);
	if (status != 0) { tof_init_status = status; goto failed; }
	tof.state[0] = 1U;
	tof_init_stage = 5U;
	return;

failed:
	/* Disable ToF polling on failure; NPX3110 uses the independent I2C3.
	 * XSHUT remains at its external pull-up level. */
	tof.state[0] = 0U;
	tof_line_after_shutdown = ToF_GetI2C1LineState();
}
#endif

#if ToF
void GetToFData(void)
{
	static uint32_t last_read_tick = 0U;
	static uint8_t consecutive_errors = 0U;
	uint8_t ready = 0U;
	uint16_t distance = 0U;
	uint8_t range_status = 255U;
	VL53L1X_ERROR quality_status = -1;
	VL53L1X_ERROR status;
    VL53L1X_ERROR distance_status=-1;
	if ((tof.state[0] == 0U) ||
		((HAL_GetTick() - last_read_tick) < 50U))
	{
		return;
	}
	last_read_tick = HAL_GetTick();
	/* ToF0 transactions stay on I2C1, independent of the NPX3110 bus. */
	status = VL53L1X_CheckForDataReady(&hi2c1, tof.address[0] << 1, &ready);
	if ((status == 0) && (ready != 0U))
	{
		status = VL53L1X_GetDistance(&hi2c1, tof.address[0] << 1, &distance);
        distance_status=status;
		if (status == 0)
		{
			quality_status = VL53L1X_GetRangeStatus(&hi2c1, tof.address[0] << 1, &range_status);
			status = VL53L1X_ClearInterrupt(&hi2c1, tof.address[0] << 1);
		}
		if (distance_status == 0)
		{
            tof_transport_error=0U;
			uint16_t corrected_distance = Sensor1CorrectToFDistance(distance, 1U);
            tof.distance[0] = corrected_distance;
			tof_read_count++;
			mobile_tof_tick = HAL_GetTick();
			mobile_tof_seen = 1U;
			mobile_tof_range_status = range_status;
			mobile_tof_quality_valid = (uint8_t)(quality_status == 0 && range_status == 0U);
		}
	}
    if (status!=0 && distance_status!=0) tof_transport_error=1U;
	tof_read_status = status;
	if (status == 0)
	{
		consecutive_errors = 0U;
		return;
	}
	tof_error_count++;
	if (++consecutive_errors >= 3U)
	{
		tof.state[0] = 0U;
		tof_init_stage = 6U;
		tof_line_after_shutdown = ToF_GetI2C1LineState();
	}

}
#endif

#if CurveFitting
void InitializeCurveFitting(void)
{
	uint16_t d0 = 270;
	uint8_t d = 10;

	cvf.X[0] = 320;
	cvf.X[1] = d0;
	cvf.X[2] = d0 - 1*d;
	cvf.X[3] = d0 - 2*d;
	cvf.X[4] = d0 - 3*d;
	cvf.X[5] = d0 - 4*d;
	cvf.X[6] = d0 - 5*d;
	cvf.X[7] = d0 - 6*d;
	cvf.X[8] = d0 - 7*d;
	cvf.X[9] = d0 - 8*d;
	cvf.X[10] = d0 - 9*d;
	cvf.X[11] = d0 - 10*d;
	cvf.X[12] = d0 - 11*d;
	cvf.X[13] = d0 - 12*d;
	cvf.X[14] = d0 - 13*d;
	cvf.X[15] = d0 - 14*d;
	cvf.X[16] = d0 - 15*d;
	cvf.X[17] = d0 - 16*d;
	cvf.X[18] = d0 - 17*d;
	cvf.X[19] = d0 - 18*d;
	cvf.X[20] = d0 - 19*d;
	cvf.X[21] = d0 - 20*d;
	cvf.X[22] = d0 - 21*d;
	cvf.X[23] = d0 - 22*d;
	cvf.X[24] = d0 - 23*d;
	cvf.X[25] = d0 - 24*d;

	cvf.Sum_X = cvf.X[1]+cvf.X[2]+cvf.X[3]+cvf.X[4]+cvf.X[5]+cvf.X[6]+cvf.X[7]+cvf.X[8]+cvf.X[9]+cvf.X[10]
				+cvf.X[11]+cvf.X[12]+cvf.X[13]+cvf.X[14]+cvf.X[15]+cvf.X[16]+cvf.X[17]+cvf.X[18]+cvf.X[19]+cvf.X[20]
				+cvf.X[21]+cvf.X[22]+cvf.X[23]+cvf.X[24]+cvf.X[25];
	cvf.Mean_X = cvf.Sum_X / 25;

	cvf.MeanData_Q1[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.dataQ1_curr[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.dataQ1_prev[0] = SENSOR1_CAP_BASELINE_COUNT;

	/* Existing25 slots:27..3cm, NEW RB10 CSVs2026-10-18; cycle5 held out. */
	#if 1
	cvf.MeanData_Q1[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.MeanData_Q1[1] = 233; /* 27cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[2] = 355; /* 26cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[3] = 495; /* 25cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[4] = 635; /* 24cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[5] = 816; /* 23cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[6] = 1013; /* 22cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[7] = 1254; /* 21cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[8] = 1531; /* 20cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[9] = 1854; /* 19cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[10] = 2241; /* 18cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[11] = 2721; /* 17cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[12] = 3318; /* 16cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[13] = 4045; /* 15cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[14] = 4920; /* 14cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[15] = 6031; /* 13cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[16] = 7399; /* 12cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[17] = 9193; /* 11cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[18] = 11492; /* 10cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[19] = 14929; /* 9cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[20] = 19833; /* 8cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[21] = 27062; /* 7cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[22] = 37517; /* 6cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[23] = 54225; /* 5cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[24] = 81573; /* 4cm,NEW RB10 CSVs2026-10-18 */
	cvf.MeanData_Q1[25] = 134780; /* 3cm,NEW RB10 CSVs2026-10-18 */


	cvf.dataQ1_curr[0] = SENSOR1_CAP_BASELINE_COUNT;								cvf.dataQ1_prev[0] = SENSOR1_CAP_BASELINE_COUNT;
	cvf.dataQ1_curr[1] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[1];			cvf.dataQ1_prev[1] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[1];
	cvf.dataQ1_curr[2] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[2];			cvf.dataQ1_prev[2] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[2];
	cvf.dataQ1_curr[3] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[3];			cvf.dataQ1_prev[3] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[3];
	cvf.dataQ1_curr[4] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[4];			cvf.dataQ1_prev[4] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[4];
	cvf.dataQ1_curr[5] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[5];			cvf.dataQ1_prev[5] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[5];
	cvf.dataQ1_curr[6] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[6];			cvf.dataQ1_prev[6] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[6];
	cvf.dataQ1_curr[7] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[7];			cvf.dataQ1_prev[7] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[7];
	cvf.dataQ1_curr[8] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[8];			cvf.dataQ1_prev[8] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[8];
	cvf.dataQ1_curr[9] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[9];			cvf.dataQ1_prev[9] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[9];
	cvf.dataQ1_curr[10] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[10];		cvf.dataQ1_prev[10] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[10];
	cvf.dataQ1_curr[11] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[11];		cvf.dataQ1_prev[11] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[11];
	cvf.dataQ1_curr[12] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[12];		cvf.dataQ1_prev[12] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[12];
	cvf.dataQ1_curr[13] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[13];		cvf.dataQ1_prev[13] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[13];
	cvf.dataQ1_curr[14] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[14];		cvf.dataQ1_prev[14] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[14];
	cvf.dataQ1_curr[15] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[15];		cvf.dataQ1_prev[15] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[15];
	cvf.dataQ1_curr[16] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[16];		cvf.dataQ1_prev[16] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[16];
	cvf.dataQ1_curr[17] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[17];		cvf.dataQ1_prev[17] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[17];
	cvf.dataQ1_curr[18] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[18];		cvf.dataQ1_prev[18] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[18];
	cvf.dataQ1_curr[19] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[19];		cvf.dataQ1_prev[19] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[19];
	cvf.dataQ1_curr[20] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[20];		cvf.dataQ1_prev[20] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[20];
	cvf.dataQ1_curr[21] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[21];		cvf.dataQ1_prev[21] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[21];
	cvf.dataQ1_curr[22] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[22];		cvf.dataQ1_prev[22] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[22];
	cvf.dataQ1_curr[23] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[23];		cvf.dataQ1_prev[23] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[23];
	cvf.dataQ1_curr[24] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[24];		cvf.dataQ1_prev[24] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[24];
	cvf.dataQ1_curr[25] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[25];		cvf.dataQ1_prev[25] = SENSOR1_CAP_BASELINE_COUNT - cvf.MeanData_Q1[25];
	#endif


	#if FDC_ADD
	cvf.MeanData_Q2[0] = 4.00E+07;
	cvf.MeanData_Q2[1] = 80;
	cvf.MeanData_Q2[2] = 80;
	cvf.MeanData_Q2[3] = 80;
	cvf.MeanData_Q2[4] = 80;
	cvf.MeanData_Q2[5] = 80;
	cvf.MeanData_Q2[6] = 100;
	cvf.MeanData_Q2[7] = 377;
	cvf.MeanData_Q2[8] = 400;
	cvf.MeanData_Q2[9] = 506;
	cvf.MeanData_Q2[10] = 813;
	cvf.MeanData_Q2[11] = 1151;
	cvf.MeanData_Q2[12] = 1595;
	cvf.MeanData_Q2[13] = 1857;
	cvf.MeanData_Q2[14] = 2188;
	cvf.MeanData_Q2[15] = 2846;
	cvf.MeanData_Q2[16] = 4231;
	cvf.MeanData_Q2[17] = 5054;
	cvf.MeanData_Q2[18] = 6337;
	cvf.MeanData_Q2[19] = 7887;
	cvf.MeanData_Q2[20] = 10092;
	cvf.MeanData_Q2[21] = 12326;
	cvf.MeanData_Q2[22] = 14846;
	cvf.MeanData_Q2[23] = 19088;
	cvf.MeanData_Q2[24] = 25246;
	cvf.MeanData_Q2[25] = 35039;


	cvf.dataQ2_curr[0] = 4.00E+07;								cvf.dataQ2_prev[0] = 4.00E+07;
	cvf.dataQ2_curr[1] = 4.00E+07 - cvf.MeanData_Q2[1];			cvf.dataQ2_prev[1] = 4.00E+07 - cvf.MeanData_Q2[1];
	cvf.dataQ2_curr[2] = 4.00E+07 - cvf.MeanData_Q2[2];			cvf.dataQ2_prev[2] = 4.00E+07 - cvf.MeanData_Q2[2];
	cvf.dataQ2_curr[3] = 4.00E+07 - cvf.MeanData_Q2[3];			cvf.dataQ2_prev[3] = 4.00E+07 - cvf.MeanData_Q2[3];
	cvf.dataQ2_curr[4] = 4.00E+07 - cvf.MeanData_Q2[4];			cvf.dataQ2_prev[4] = 4.00E+07 - cvf.MeanData_Q2[4];
	cvf.dataQ2_curr[5] = 4.00E+07 - cvf.MeanData_Q2[5];			cvf.dataQ2_prev[5] = 4.00E+07 - cvf.MeanData_Q2[5];
	cvf.dataQ2_curr[6] = 4.00E+07 - cvf.MeanData_Q2[6];			cvf.dataQ2_prev[6] = 4.00E+07 - cvf.MeanData_Q2[6];
	cvf.dataQ2_curr[7] = 4.00E+07 - cvf.MeanData_Q2[7];			cvf.dataQ2_prev[7] = 4.00E+07 - cvf.MeanData_Q2[7];
	cvf.dataQ2_curr[8] = 4.00E+07 - cvf.MeanData_Q2[8];			cvf.dataQ2_prev[8] = 4.00E+07 - cvf.MeanData_Q2[8];
	cvf.dataQ2_curr[9] = 4.00E+07 - cvf.MeanData_Q2[9];			cvf.dataQ2_prev[9] = 4.00E+07 - cvf.MeanData_Q2[9];
	cvf.dataQ2_curr[10] = 4.00E+07 - cvf.MeanData_Q2[10];		cvf.dataQ2_prev[10] = 4.00E+07 - cvf.MeanData_Q2[10];
	cvf.dataQ2_curr[11] = 4.00E+07 - cvf.MeanData_Q2[11];		cvf.dataQ2_prev[11] = 4.00E+07 - cvf.MeanData_Q2[11];
	cvf.dataQ2_curr[12] = 4.00E+07 - cvf.MeanData_Q2[12];		cvf.dataQ2_prev[12] = 4.00E+07 - cvf.MeanData_Q2[12];
	cvf.dataQ2_curr[13] = 4.00E+07 - cvf.MeanData_Q2[13];		cvf.dataQ2_prev[13] = 4.00E+07 - cvf.MeanData_Q2[13];
	cvf.dataQ2_curr[14] = 4.00E+07 - cvf.MeanData_Q2[14];		cvf.dataQ2_prev[14] = 4.00E+07 - cvf.MeanData_Q2[14];
	cvf.dataQ2_curr[15] = 4.00E+07 - cvf.MeanData_Q2[15];		cvf.dataQ2_prev[15] = 4.00E+07 - cvf.MeanData_Q2[15];
	cvf.dataQ2_curr[16] = 4.00E+07 - cvf.MeanData_Q2[16];		cvf.dataQ2_prev[16] = 4.00E+07 - cvf.MeanData_Q2[16];
	cvf.dataQ2_curr[17] = 4.00E+07 - cvf.MeanData_Q2[17];		cvf.dataQ2_prev[17] = 4.00E+07 - cvf.MeanData_Q2[17];
	cvf.dataQ2_curr[18] = 4.00E+07 - cvf.MeanData_Q2[18];		cvf.dataQ2_prev[18] = 4.00E+07 - cvf.MeanData_Q2[18];
	cvf.dataQ2_curr[19] = 4.00E+07 - cvf.MeanData_Q2[19];		cvf.dataQ2_prev[19] = 4.00E+07 - cvf.MeanData_Q2[19];
	cvf.dataQ2_curr[20] = 4.00E+07 - cvf.MeanData_Q2[20];		cvf.dataQ2_prev[20] = 4.00E+07 - cvf.MeanData_Q2[20];
	cvf.dataQ2_curr[21] = 4.00E+07 - cvf.MeanData_Q2[21];		cvf.dataQ2_prev[21] = 4.00E+07 - cvf.MeanData_Q2[21];
	cvf.dataQ2_curr[22] = 4.00E+07 - cvf.MeanData_Q2[22];		cvf.dataQ2_prev[22] = 4.00E+07 - cvf.MeanData_Q2[22];
	cvf.dataQ2_curr[23] = 4.00E+07 - cvf.MeanData_Q2[23];		cvf.dataQ2_prev[23] = 4.00E+07 - cvf.MeanData_Q2[23];
	cvf.dataQ2_curr[24] = 4.00E+07 - cvf.MeanData_Q2[24];		cvf.dataQ2_prev[24] = 4.00E+07 - cvf.MeanData_Q2[24];
	cvf.dataQ2_curr[25] = 4.00E+07 - cvf.MeanData_Q2[25];		cvf.dataQ2_prev[25] = 4.00E+07 - cvf.MeanData_Q2[25];
	#endif
}
#endif

#if (CurveFitting && ToF)
void AchieveCurveFittingData(void)
{
	uint8_t bias = 1;

	/* Sample 1 */
	if((tof.distance[0] >= (cvf.X[0] - bias)) && (tof.distance[0] <= (cvf.X[0] + 2*bias)))
	{
		cvf.dataQ1_curr[0] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[0] = 0.98*cvf.dataQ1_prev[0] + 0.02*cvf.dataQ1_curr[0];
		cvf.dataQ1_prev[0] = cvf.dataQ1_filtered[0];

		cvf.MeanData_Q1[0] = cvf.dataQ1_filtered[0];
	}
	/* Sample 2 */
	else if((tof.distance[0] >= cvf.X[1] - bias) && (tof.distance[0] <= cvf.X[1] + bias))
	{
		cvf.dataQ1_curr[1] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[1] = 0.5*cvf.dataQ1_prev[1] + 0.5*cvf.dataQ1_curr[1];
		cvf.dataQ1_prev[1] = cvf.dataQ1_filtered[1];

		cvf.MeanData_Q1[1] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[1]);
		if(cvf.MeanData_Q1[1] <= 1)
		{
			cvf.MeanData_Q1[1] = 1;
		}
	}
	/* Sample 3 */
	else if((tof.distance[0] >= cvf.X[2] - bias) && (tof.distance[0] <= cvf.X[2] + bias))
	{
		cvf.dataQ1_curr[2] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[2] = 0.5*cvf.dataQ1_prev[2] + 0.5*cvf.dataQ1_curr[2];
		cvf.dataQ1_prev[2] = cvf.dataQ1_filtered[2];

		cvf.MeanData_Q1[2] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[2]);
		if(cvf.MeanData_Q1[2] <= 1)
		{
			cvf.MeanData_Q1[2] = 1;
		}
	}
	/* Sample 4 */
	else if((tof.distance[0] >= cvf.X[3] - bias) && (tof.distance[0] <= cvf.X[3] + bias))
	{
		cvf.dataQ1_curr[3] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[3] = 0.5*cvf.dataQ1_prev[3] + 0.5*cvf.dataQ1_curr[3];
		cvf.dataQ1_prev[3] = cvf.dataQ1_filtered[3];

		cvf.MeanData_Q1[3] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[3]);
		if(cvf.MeanData_Q1[3] <= 1)
		{
			cvf.MeanData_Q1[3] = 1;
		}
	}
	/* Sample 5 */
	else if((tof.distance[0] >= cvf.X[4] - bias) && (tof.distance[0] <= cvf.X[4] + bias))
	{
		cvf.dataQ1_curr[4] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[4] = 0.5*cvf.dataQ1_prev[4] + 0.5*cvf.dataQ1_curr[4];
		cvf.dataQ1_prev[4] = cvf.dataQ1_filtered[4];

		cvf.MeanData_Q1[4] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[4]);
		if(cvf.MeanData_Q1[4] <= 1)
		{
			cvf.MeanData_Q1[4] = 1;
		}
	}
	/* Sample 6 */
	else if((tof.distance[0] >= cvf.X[5] - bias) && (tof.distance[0] <= cvf.X[5] + bias))
	{
		cvf.dataQ1_curr[5] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[5] = 0.5*cvf.dataQ1_prev[5] + 0.5*cvf.dataQ1_curr[5];
		cvf.dataQ1_prev[5] = cvf.dataQ1_filtered[5];

		cvf.MeanData_Q1[5] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[5]);
		if(cvf.MeanData_Q1[5] <= 1)
		{
			cvf.MeanData_Q1[5] = 1;
		}
	}
	/* Sample 7 */
	else if((tof.distance[0] >= cvf.X[6] - bias) && (tof.distance[0] <= cvf.X[6] + bias))
	{
		cvf.dataQ1_curr[6] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[6] = 0.5*cvf.dataQ1_prev[6] + 0.5*cvf.dataQ1_curr[6];
		cvf.dataQ1_prev[6] = cvf.dataQ1_filtered[6];

		cvf.MeanData_Q1[6] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[6]);
		if(cvf.MeanData_Q1[6] <= 1)
		{
			cvf.MeanData_Q1[6] = 1;
		}
	}
	/* Sample 8 */
	else if((tof.distance[0] >= cvf.X[7] - bias) && (tof.distance[0] <= cvf.X[7] + bias))
	{
		cvf.dataQ1_curr[7] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[7] = 0.5*cvf.dataQ1_prev[7] + 0.5*cvf.dataQ1_curr[7];
		cvf.dataQ1_prev[7] = cvf.dataQ1_filtered[7];

		cvf.MeanData_Q1[7] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[7]);
		if(cvf.MeanData_Q1[7] <= 1)
		{
			cvf.MeanData_Q1[7] = 1;
		}
	}
	/* Sample 9 */
	else if((tof.distance[0] >= cvf.X[8] - bias) && (tof.distance[0] <= cvf.X[8] + bias))
	{
		cvf.dataQ1_curr[8] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[8] = 0.5*cvf.dataQ1_prev[8] + 0.5*cvf.dataQ1_curr[8];
		cvf.dataQ1_prev[8] = cvf.dataQ1_filtered[8];

		cvf.MeanData_Q1[8] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[8]);
		if(cvf.MeanData_Q1[8] <= 1)
		{
			cvf.MeanData_Q1[8] = 1;
		}
	}
	/* Sample 10 */
	else if((tof.distance[0] >= cvf.X[9] - bias) && (tof.distance[0] <= cvf.X[9] + bias))
	{
		cvf.dataQ1_curr[9] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[9] = 0.5*cvf.dataQ1_prev[9] + 0.5*cvf.dataQ1_curr[9];
		cvf.dataQ1_prev[9] = cvf.dataQ1_filtered[9];

		cvf.MeanData_Q1[9] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[9]);
		if(cvf.MeanData_Q1[9] <= 1)
		{
			cvf.MeanData_Q1[9] = 1;
		}
	}
	/* Sample 11 */
	else if((tof.distance[0] >= cvf.X[10] - bias) && (tof.distance[0] <= cvf.X[10] + bias))
	{
		cvf.dataQ1_curr[10] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[10] = 0.5*cvf.dataQ1_prev[10] + 0.5*cvf.dataQ1_curr[10];
		cvf.dataQ1_prev[10] = cvf.dataQ1_filtered[10];

		cvf.MeanData_Q1[10] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[10]);
		if(cvf.MeanData_Q1[10] <= 1)
		{
			cvf.MeanData_Q1[10] = 1;
		}
	}
	/* Sample 12 */
	else if((tof.distance[0] >= cvf.X[11] - bias) && (tof.distance[0] <= cvf.X[11] + bias))
	{
		cvf.dataQ1_curr[11] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[11] = 0.5*cvf.dataQ1_prev[11] + 0.5*cvf.dataQ1_curr[11];
		cvf.dataQ1_prev[11] = cvf.dataQ1_filtered[11];

		cvf.MeanData_Q1[11] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[11]);
		if(cvf.MeanData_Q1[11] <= 1)
		{
			cvf.MeanData_Q1[11] = 1;
		}
	}
	/* Sample 13 */
	else if((tof.distance[0] >= cvf.X[12] - bias) && (tof.distance[0] <= cvf.X[12] + bias))
	{
		cvf.dataQ1_curr[12] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[12] = 0.5*cvf.dataQ1_prev[12] + 0.5*cvf.dataQ1_curr[12];
		cvf.dataQ1_prev[12] = cvf.dataQ1_filtered[12];

		cvf.MeanData_Q1[12] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[12]);
		if(cvf.MeanData_Q1[12] <= 1)
		{
			cvf.MeanData_Q1[12] = 1;
		}
	}
	/* Sample 14 */
	else if((tof.distance[0] >= cvf.X[13] - bias) && (tof.distance[0] <= cvf.X[13] + bias))
	{
		cvf.dataQ1_curr[13] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[13] = 0.5*cvf.dataQ1_prev[13] + 0.5*cvf.dataQ1_curr[13];
		cvf.dataQ1_prev[13] = cvf.dataQ1_filtered[13];

		cvf.MeanData_Q1[13] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[13]);
		if(cvf.MeanData_Q1[13] <= 1)
		{
			cvf.MeanData_Q1[13] = 1;
		}
	}
	/* Sample 15 */
	else if((tof.distance[0] >= cvf.X[14] - bias) && (tof.distance[0] <= cvf.X[14] + bias))
	{
		cvf.dataQ1_curr[14] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[14] = 0.5*cvf.dataQ1_prev[14] + 0.5*cvf.dataQ1_curr[14];
		cvf.dataQ1_prev[14] = cvf.dataQ1_filtered[14];

		cvf.MeanData_Q1[14] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[14]);
		if(cvf.MeanData_Q1[14] <= 1)
		{
			cvf.MeanData_Q1[14] = 1;
		}
	}
	/* Sample 16 */
	else if((tof.distance[0] >= cvf.X[15] - bias) && (tof.distance[0] <= cvf.X[15] + bias))
	{
		cvf.dataQ1_curr[15] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[15] = 0.5*cvf.dataQ1_prev[15] + 0.5*cvf.dataQ1_curr[15];
		cvf.dataQ1_prev[15] = cvf.dataQ1_filtered[15];

		cvf.MeanData_Q1[15] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[15]);
		if(cvf.MeanData_Q1[15] <= 1)
		{
			cvf.MeanData_Q1[15] = 1;
		}
	}
	/* Sample 17 */
	else if((tof.distance[0] >= cvf.X[16] - bias) && (tof.distance[0] <= cvf.X[16] + bias))
	{
		cvf.dataQ1_curr[16] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[16] = 0.5*cvf.dataQ1_prev[16] + 0.5*cvf.dataQ1_curr[16];
		cvf.dataQ1_prev[16] = cvf.dataQ1_filtered[16];

		cvf.MeanData_Q1[16] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[16]);
	}
	/* Sample 18 */
	else if((tof.distance[0] >= cvf.X[17] - bias) && (tof.distance[0] <= cvf.X[17] + bias))
	{
		cvf.dataQ1_curr[17] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[17] = 0.5*cvf.dataQ1_prev[17] + 0.5*cvf.dataQ1_curr[17];
		cvf.dataQ1_prev[17] = cvf.dataQ1_filtered[17];

		cvf.MeanData_Q1[17] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[17]);
	}
	/* Sample 19 */
	else if((tof.distance[0] >= cvf.X[18] - bias) && (tof.distance[0] <= cvf.X[18] + bias))
	{
		cvf.dataQ1_curr[18] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[18] = 0.5*cvf.dataQ1_prev[18] + 0.5*cvf.dataQ1_curr[18];
		cvf.dataQ1_prev[18] = cvf.dataQ1_filtered[18];

		cvf.MeanData_Q1[18] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[18]);
	}
	/* Sample 20 */
	else if((tof.distance[0] >= cvf.X[19] - bias) && (tof.distance[0] <= cvf.X[19] + bias))
	{
		cvf.dataQ1_curr[19] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[19] = 0.5*cvf.dataQ1_prev[19] + 0.5*cvf.dataQ1_curr[19];
		cvf.dataQ1_prev[19] = cvf.dataQ1_filtered[19];

		cvf.MeanData_Q1[19] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[19]);
	}
	/* Sample 21 */
	else if((tof.distance[0] >= cvf.X[20] - bias) && (tof.distance[0] <= cvf.X[20] + bias))
	{
		cvf.dataQ1_curr[20] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[20] = 0.5*cvf.dataQ1_prev[20] + 0.5*cvf.dataQ1_curr[20];
		cvf.dataQ1_prev[20] = cvf.dataQ1_filtered[20];

		cvf.MeanData_Q1[20] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[20]);
	}
	/* Sample 21 */
	else if((tof.distance[0] >= cvf.X[21] - bias) && (tof.distance[0] <= cvf.X[21] + bias))
	{
		cvf.dataQ1_curr[21] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[21] = 0.5*cvf.dataQ1_prev[21] + 0.5*cvf.dataQ1_curr[21];
		cvf.dataQ1_prev[21] = cvf.dataQ1_filtered[21];

		cvf.MeanData_Q1[21] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[21]);
	}
	/* Sample 21 */
	else if((tof.distance[0] >= cvf.X[22] - bias) && (tof.distance[0] <= cvf.X[22] + bias))
	{
		cvf.dataQ1_curr[22] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[22] = 0.5*cvf.dataQ1_prev[22] + 0.5*cvf.dataQ1_curr[22];
		cvf.dataQ1_prev[22] = cvf.dataQ1_filtered[22];

		cvf.MeanData_Q1[22] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[22]);
	}
	/* Sample 21 */
	else if((tof.distance[0] >= cvf.X[23] - bias) && (tof.distance[0] <= cvf.X[23] + bias))
	{
		cvf.dataQ1_curr[23] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[23] = 0.5*cvf.dataQ1_prev[23] + 0.5*cvf.dataQ1_curr[23];
		cvf.dataQ1_prev[23] = cvf.dataQ1_filtered[23];

		cvf.MeanData_Q1[23] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[23]);
	}
	/* Sample 21 */
	else if((tof.distance[0] >= cvf.X[24] - bias) && (tof.distance[0] <= cvf.X[24] + bias))
	{
		cvf.dataQ1_curr[24] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[24] = 0.5*cvf.dataQ1_prev[24] + 0.5*cvf.dataQ1_curr[24];
		cvf.dataQ1_prev[24] = cvf.dataQ1_filtered[24];

		cvf.MeanData_Q1[24] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[24]);
	}
	/* Sample 21 */
	else if((tof.distance[0] >= cvf.X[25] - bias) && (tof.distance[0] <= cvf.X[25] + bias))
	{
		cvf.dataQ1_curr[25] = dComps.fdc_Q1;
		cvf.dataQ1_filtered[25] = 0.5*cvf.dataQ1_prev[25] + 0.5*cvf.dataQ1_curr[25];
		cvf.dataQ1_prev[25] = cvf.dataQ1_filtered[25];

		cvf.MeanData_Q1[25] = abs(cvf.MeanData_Q1[0] - cvf.dataQ1_filtered[25]);
	}
	else
	{
		/* Nothing */
	}

	#if FDC_ADD
	/* Sample 1 */
	if((tof.distance[1] >= (cvf.X[0] - bias)) && (tof.distance[1] <= (cvf.X[0] + 2*bias)))
	{
		cvf.dataQ2_curr[0] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[0] = 0.98*cvf.dataQ2_prev[0] + 0.02*cvf.dataQ2_curr[0];
		cvf.dataQ2_prev[0] = cvf.dataQ2_filtered[0];

		cvf.MeanData_Q2[0] = cvf.dataQ2_filtered[0];
	}
	/* Sample 2 */
	else if((tof.distance[1] >= cvf.X[1] - bias) && (tof.distance[1] <= cvf.X[1] + bias))
	{
		cvf.dataQ2_curr[1] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[1] = 0.5*cvf.dataQ2_prev[1] + 0.5*cvf.dataQ2_curr[1];
		cvf.dataQ2_prev[1] = cvf.dataQ2_filtered[1];

		cvf.MeanData_Q2[1] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[1]);
		if(cvf.MeanData_Q2[1] <= 1)
		{
			cvf.MeanData_Q2[1] = 1;
		}
	}
	/* Sample 3 */
	else if((tof.distance[1] >= cvf.X[2] - bias) && (tof.distance[1] <= cvf.X[2] + bias))
	{
		cvf.dataQ2_curr[2] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[2] = 0.5*cvf.dataQ2_prev[2] + 0.5*cvf.dataQ2_curr[2];
		cvf.dataQ2_prev[2] = cvf.dataQ2_filtered[2];

		cvf.MeanData_Q2[2] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[2]);
		if(cvf.MeanData_Q2[2] <= 1)
		{
			cvf.MeanData_Q2[2] = 1;
		}
	}
	/* Sample 4 */
	else if((tof.distance[1] >= cvf.X[3] - bias) && (tof.distance[1] <= cvf.X[3] + bias))
	{
		cvf.dataQ2_curr[3] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[3] = 0.5*cvf.dataQ2_prev[3] + 0.5*cvf.dataQ2_curr[3];
		cvf.dataQ2_prev[3] = cvf.dataQ2_filtered[3];

		cvf.MeanData_Q2[3] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[3]);
		if(cvf.MeanData_Q2[3] <= 1)
		{
			cvf.MeanData_Q2[3] = 1;
		}
	}
	/* Sample 5 */
	else if((tof.distance[1] >= cvf.X[4] - bias) && (tof.distance[1] <= cvf.X[4] + bias))
	{
		cvf.dataQ2_curr[4] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[4] = 0.5*cvf.dataQ2_prev[4] + 0.5*cvf.dataQ2_curr[4];
		cvf.dataQ2_prev[4] = cvf.dataQ2_filtered[4];

		cvf.MeanData_Q2[4] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[4]);
		if(cvf.MeanData_Q2[4] <= 1)
		{
			cvf.MeanData_Q2[4] = 1;
		}
	}
	/* Sample 6 */
	else if((tof.distance[1] >= cvf.X[5] - bias) && (tof.distance[1] <= cvf.X[5] + bias))
	{
		cvf.dataQ2_curr[5] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[5] = 0.5*cvf.dataQ2_prev[5] + 0.5*cvf.dataQ2_curr[5];
		cvf.dataQ2_prev[5] = cvf.dataQ2_filtered[5];

		cvf.MeanData_Q2[5] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[5]);
		if(cvf.MeanData_Q2[5] <= 1)
		{
			cvf.MeanData_Q2[5] = 1;
		}
	}
	/* Sample 7 */
	else if((tof.distance[1] >= cvf.X[6] - bias) && (tof.distance[1] <= cvf.X[6] + bias))
	{
		cvf.dataQ2_curr[6] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[6] = 0.5*cvf.dataQ2_prev[6] + 0.5*cvf.dataQ2_curr[6];
		cvf.dataQ2_prev[6] = cvf.dataQ2_filtered[6];

		cvf.MeanData_Q2[6] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[6]);
		if(cvf.MeanData_Q2[6] <= 1)
		{
			cvf.MeanData_Q2[6] = 1;
		}
	}
	/* Sample 8 */
	else if((tof.distance[1] >= cvf.X[7] - bias) && (tof.distance[1] <= cvf.X[7] + bias))
	{
		cvf.dataQ2_curr[7] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[7] = 0.5*cvf.dataQ2_prev[7] + 0.5*cvf.dataQ2_curr[7];
		cvf.dataQ2_prev[7] = cvf.dataQ2_filtered[7];

		cvf.MeanData_Q2[7] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[7]);
		if(cvf.MeanData_Q2[7] <= 1)
		{
			cvf.MeanData_Q2[7] = 1;
		}
	}
	/* Sample 9 */
	else if((tof.distance[1] >= cvf.X[8] - bias) && (tof.distance[1] <= cvf.X[8] + bias))
	{
		cvf.dataQ2_curr[8] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[8] = 0.5*cvf.dataQ2_prev[8] + 0.5*cvf.dataQ2_curr[8];
		cvf.dataQ2_prev[8] = cvf.dataQ2_filtered[8];

		cvf.MeanData_Q2[8] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[8]);
		if(cvf.MeanData_Q2[8] <= 1)
		{
			cvf.MeanData_Q2[8] = 1;
		}
	}
	/* Sample 10 */
	else if((tof.distance[1] >= cvf.X[9] - bias) && (tof.distance[1] <= cvf.X[9] + bias))
	{
		cvf.dataQ2_curr[9] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[9] = 0.5*cvf.dataQ2_prev[9] + 0.5*cvf.dataQ2_curr[9];
		cvf.dataQ2_prev[9] = cvf.dataQ2_filtered[9];

		cvf.MeanData_Q2[9] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[9]);
		if(cvf.MeanData_Q2[9] <= 1)
		{
			cvf.MeanData_Q2[9] = 1;
		}
	}
	/* Sample 11 */
	else if((tof.distance[1] >= cvf.X[10] - bias) && (tof.distance[1] <= cvf.X[10] + bias))
	{
		cvf.dataQ2_curr[10] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[10] = 0.5*cvf.dataQ2_prev[10] + 0.5*cvf.dataQ2_curr[10];
		cvf.dataQ2_prev[10] = cvf.dataQ2_filtered[10];

		cvf.MeanData_Q2[10] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[10]);
		if(cvf.MeanData_Q2[10] <= 1)
		{
			cvf.MeanData_Q2[10] = 1;
		}
	}
	/* Sample 12 */
	else if((tof.distance[1] >= cvf.X[11] - bias) && (tof.distance[1] <= cvf.X[11] + bias))
	{
		cvf.dataQ2_curr[11] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[11] = 0.5*cvf.dataQ2_prev[11] + 0.5*cvf.dataQ2_curr[11];
		cvf.dataQ2_prev[11] = cvf.dataQ2_filtered[11];

		cvf.MeanData_Q2[11] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[11]);
		if(cvf.MeanData_Q2[11] <= 1)
		{
			cvf.MeanData_Q2[11] = 1;
		}
	}
	/* Sample 13 */
	else if((tof.distance[1] >= cvf.X[12] - bias) && (tof.distance[1] <= cvf.X[12] + bias))
	{
		cvf.dataQ2_curr[12] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[12] = 0.5*cvf.dataQ2_prev[12] + 0.5*cvf.dataQ2_curr[12];
		cvf.dataQ2_prev[12] = cvf.dataQ2_filtered[12];

		cvf.MeanData_Q2[12] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[12]);
		if(cvf.MeanData_Q2[12] <= 1)
		{
			cvf.MeanData_Q2[12] = 1;
		}
	}
	/* Sample 14 */
	else if((tof.distance[1] >= cvf.X[13] - bias) && (tof.distance[1] <= cvf.X[13] + bias))
	{
		cvf.dataQ2_curr[13] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[13] = 0.5*cvf.dataQ2_prev[13] + 0.5*cvf.dataQ2_curr[13];
		cvf.dataQ2_prev[13] = cvf.dataQ2_filtered[13];

		cvf.MeanData_Q2[13] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[13]);
		if(cvf.MeanData_Q2[13] <= 1)
		{
			cvf.MeanData_Q2[13] = 1;
		}
	}
	/* Sample 15 */
	else if((tof.distance[1] >= cvf.X[14] - bias) && (tof.distance[1] <= cvf.X[14] + bias))
	{
		cvf.dataQ2_curr[14] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[14] = 0.5*cvf.dataQ2_prev[14] + 0.5*cvf.dataQ2_curr[14];
		cvf.dataQ2_prev[14] = cvf.dataQ2_filtered[14];

		cvf.MeanData_Q2[14] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[14]);
		if(cvf.MeanData_Q2[14] <= 1)
		{
			cvf.MeanData_Q2[14] = 1;
		}
	}
	/* Sample 16 */
	else if((tof.distance[1] >= cvf.X[15] - bias) && (tof.distance[1] <= cvf.X[15] + bias))
	{
		cvf.dataQ2_curr[15] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[15] = 0.5*cvf.dataQ2_prev[15] + 0.5*cvf.dataQ2_curr[15];
		cvf.dataQ2_prev[15] = cvf.dataQ2_filtered[15];

		cvf.MeanData_Q2[15] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[15]);
		if(cvf.MeanData_Q2[15] <= 1)
		{
			cvf.MeanData_Q2[15] = 1;
		}
	}
	/* Sample 17 */
	else if((tof.distance[1] >= cvf.X[16] - bias) && (tof.distance[1] <= cvf.X[16] + bias))
	{
		cvf.dataQ2_curr[16] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[16] = 0.5*cvf.dataQ2_prev[16] + 0.5*cvf.dataQ2_curr[16];
		cvf.dataQ2_prev[16] = cvf.dataQ2_filtered[16];

		cvf.MeanData_Q2[16] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[16]);
	}
	/* Sample 18 */
	else if((tof.distance[1] >= cvf.X[17] - bias) && (tof.distance[1] <= cvf.X[17] + bias))
	{
		cvf.dataQ2_curr[17] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[17] = 0.5*cvf.dataQ2_prev[17] + 0.5*cvf.dataQ2_curr[17];
		cvf.dataQ2_prev[17] = cvf.dataQ2_filtered[17];

		cvf.MeanData_Q2[17] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[17]);
	}
	/* Sample 19 */
	else if((tof.distance[1] >= cvf.X[18] - bias) && (tof.distance[1] <= cvf.X[18] + bias))
	{
		cvf.dataQ2_curr[18] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[18] = 0.5*cvf.dataQ2_prev[18] + 0.5*cvf.dataQ2_curr[18];
		cvf.dataQ2_prev[18] = cvf.dataQ2_filtered[18];

		cvf.MeanData_Q2[18] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[18]);
	}
	/* Sample 20 */
	else if((tof.distance[1] >= cvf.X[19] - bias) && (tof.distance[1] <= cvf.X[19] + bias))
	{
		cvf.dataQ2_curr[19] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[19] = 0.5*cvf.dataQ2_prev[19] + 0.5*cvf.dataQ2_curr[19];
		cvf.dataQ2_prev[19] = cvf.dataQ2_filtered[19];

		cvf.MeanData_Q2[19] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[19]);
	}
	/* Sample 21 */
	else if((tof.distance[1] >= cvf.X[20] - bias) && (tof.distance[1] <= cvf.X[20] + bias))
	{
		cvf.dataQ2_curr[20] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[20] = 0.5*cvf.dataQ2_prev[20] + 0.5*cvf.dataQ2_curr[20];
		cvf.dataQ2_prev[20] = cvf.dataQ2_filtered[20];

		cvf.MeanData_Q2[20] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[20]);
	}
	/* Sample 21 */
	else if((tof.distance[1] >= cvf.X[21] - bias) && (tof.distance[1] <= cvf.X[21] + bias))
	{
		cvf.dataQ2_curr[21] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[21] = 0.5*cvf.dataQ2_prev[21] + 0.5*cvf.dataQ2_curr[21];
		cvf.dataQ2_prev[21] = cvf.dataQ2_filtered[21];

		cvf.MeanData_Q2[21] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[21]);
	}
	/* Sample 21 */
	else if((tof.distance[1] >= cvf.X[22] - bias) && (tof.distance[1] <= cvf.X[22] + bias))
	{
		cvf.dataQ2_curr[22] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[22] = 0.5*cvf.dataQ2_prev[22] + 0.5*cvf.dataQ2_curr[22];
		cvf.dataQ2_prev[22] = cvf.dataQ2_filtered[22];

		cvf.MeanData_Q2[22] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[22]);
	}
	/* Sample 21 */
	else if((tof.distance[1] >= cvf.X[23] - bias) && (tof.distance[1] <= cvf.X[23] + bias))
	{
		cvf.dataQ2_curr[23] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[23] = 0.5*cvf.dataQ2_prev[23] + 0.5*cvf.dataQ2_curr[23];
		cvf.dataQ2_prev[23] = cvf.dataQ2_filtered[23];

		cvf.MeanData_Q2[23] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[23]);
	}
	/* Sample 21 */
	else if((tof.distance[1] >= cvf.X[24] - bias) && (tof.distance[1] <= cvf.X[24] + bias))
	{
		cvf.dataQ2_curr[24] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[24] = 0.5*cvf.dataQ2_prev[24] + 0.5*cvf.dataQ2_curr[24];
		cvf.dataQ2_prev[24] = cvf.dataQ2_filtered[24];

		cvf.MeanData_Q2[24] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[24]);
	}
	/* Sample 21 */
	else if((tof.distance[1] >= cvf.X[25] - bias) && (tof.distance[1] <= cvf.X[25] + bias))
	{
		cvf.dataQ2_curr[25] = dComps.fdc_Q2;
		cvf.dataQ2_filtered[25] = 0.5*cvf.dataQ2_prev[25] + 0.5*cvf.dataQ2_curr[25];
		cvf.dataQ2_prev[25] = cvf.dataQ2_filtered[25];

		cvf.MeanData_Q2[25] = abs(cvf.MeanData_Q2[0] - cvf.dataQ2_filtered[25]);
	}
	else
	{
		/* Nothing */
	}
	#endif
}
#endif

#if CurveFitting
void ExpCurvefitting(void)
{
	/* Log */
	for(uint8_t i = 1; i < 25+1; i ++)
	{
		if(i == 1)
		{
			cvf.Sum_lnY_Q1 = 0;

			#if FDC_ADD
			cvf.Sum_lnY_Q2 = 0;
			#endif
		}
		cvf.lnY_Q1[i] = log(cvf.MeanData_Q1[i]);
		cvf.Sum_lnY_Q1 += cvf.lnY_Q1[i];

		/**/
		#if FDC_ADD
		cvf.lnY_Q2[i] = log(cvf.MeanData_Q2[i]);
		cvf.Sum_lnY_Q2 += cvf.lnY_Q2[i];
		#endif
	}
	cvf.Mean_lnY_Q1 = cvf.Sum_lnY_Q1 *0.04;

	#if FDC_ADD
	cvf.Mean_lnY_Q2 = cvf.Sum_lnY_Q2 *0.04;
	#endif

	for(uint8_t i = 1; i < 25+1; i ++)
	{
		if(i == 1)
		{
			cvf.sumNumnerator_Q1 = 0;
			cvf.sumDenominator_Q1 = 0;

			#if FDC_ADD
			cvf.sumNumnerator_Q2 = 0;
			cvf.sumDenominator_Q2 = 0;
			#endif
		}
		cvf.err_X[i] = cvf.X[i] - cvf.Mean_X;

		cvf.err_lnY_Q1[i] = cvf.lnY_Q1[i] - cvf.Mean_lnY_Q1;
		cvf.sumNumnerator_Q1 += cvf.err_lnY_Q1[i]*cvf.err_X[i];
		cvf.sumDenominator_Q1 += cvf.err_X[i]*cvf.err_X[i];

		#if FDC_ADD
		cvf.err_lnY_Q2[i] = cvf.lnY_Q2[i] - cvf.Mean_lnY_Q2;
		cvf.sumNumnerator_Q2 += cvf.err_lnY_Q2[i]*cvf.err_X[i];
		cvf.sumDenominator_Q2 += cvf.err_X[i]*cvf.err_X[i];
		#endif
	}

	cvf.Coeff_Q1 = cvf.sumNumnerator_Q1 / cvf.sumDenominator_Q1;
	cvf.tmpCoeff_Q1 = abs((cvf.sumNumnerator_Q1*100000000) / cvf.sumDenominator_Q1);

	#if FDC_ADD
	cvf.Coeff_Q2 = cvf.sumNumnerator_Q2 / cvf.sumDenominator_Q2;
	cvf.tmpCoeff_Q2 = abs((cvf.sumNumnerator_Q2*100000000) / cvf.sumDenominator_Q2);
	#endif

	#if 0
	if(cvf.Coeff_Q1 > -0.008)
	{
		cvf.Coeff_Q1 = -0.008;
	}
	else if(cvf.Coeff_Q1 < -0.08)
	{
		cvf.Coeff_Q1 = -0.08;
	}
	#endif

	cvf.Offset_Q1 = cvf.Mean_lnY_Q1 - (cvf.Coeff_Q1*cvf.Mean_X);

	#if FDC_ADD
	cvf.Offset_Q2 = cvf.Mean_lnY_Q2 - (cvf.Coeff_Q2*cvf.Mean_X);
	#endif

	cvf.logY_Q1 = cvf.MeanData_Q1[0] - dComps.fdc_Q1;
	if(cvf.logY_Q1 <= 1)
	{
		cvf.logY_Q1 = 1;
	}

	#if FDC_ADD
	cvf.logY_Q2 = cvf.MeanData_Q2[0] - dComps.fdc_Q2;
	if(cvf.logY_Q2 <= 1)
	{
		cvf.logY_Q2 = 1;
	}
	#endif

	cvf.distancePredict_Q1 = CurrentDistanceToInt((1/cvf.Coeff_Q1)*(log(cvf.logY_Q1) - cvf.Offset_Q1));
	#if 1
	if(cvf.distancePredict_Q1 <= 0)
	{
		cvf.distancePredict_Q1 = 0;
	}
	#endif

	#if FDC_ADD
	cvf.distancePredict_Q2 = (1/cvf.Coeff_Q2)*(log(cvf.logY_Q2) - cvf.Offset_Q2);
	#if 1
	if(cvf.distancePredict_Q2 <= 0)
	{
		cvf.distancePredict_Q2 = 0;
	}
	else if(cvf.distancePredict_Q2 > 500)
	{
		cvf.distancePredict_Q2 = 500;
	}
	#endif
	#endif
}
#endif

#if (ToFFusion && ToF && CurveFitting)
void PerformSensorFusion(void)
{
    {int64_t d=(int64_t)cvf.MeanData_Q1[0]-(int64_t)dComps.fdc_Q1;fusion.mean_CapRaw_Q1=(uint32_t)(d>=0?d:-d);}
	const float minimum_std = 0.001f;
	const float coefficient = (float)cvf.Coeff_Q1;
	const float cap_mean = (float)fusion.mean_CapRaw_Q1;
	const float cap_distance = (float)cvf.FilteredDistance_Q1;
	const float tof_distance = (float)tof.distance[0];

	fusion.FusedDistance_Q1_valid = 0U;
	fusion.std_CapRaw_Q1 = fusion_cap_noise_Q1.std_count;
	fusion.std_ToF_Q1 = 5.0f;

    /* Quality/freshness/table range do not gate this calculation.
     * Reject only absent bytes or a mathematically undefined operation. */
    fusion.FusedDistance_Q1 = -1; /* UINT32_MAX; never silently hold. */
    if (!current_sample_received || tof_transport_error || !isfinite(coefficient) ||
            fabsf(coefficient) <= 1.0e-9f || cap_mean <= 0.0f ||
            !isfinite(cap_distance) || cap_distance < 0.0f) return;

	fusion.std_CapDist_Q1 = fabsf(
			((float)fusion.std_CapRaw_Q1) / (coefficient * cap_mean));
	if (!isfinite(fusion.std_CapDist_Q1))
	{
		return;
	}
	if (fusion.std_CapDist_Q1 < minimum_std)
	{
		fusion.std_CapDist_Q1 = minimum_std;
	}

	float scaled_std = fusion.std_CapDist_Q1 * 1000.0f;
	fusion.std_CapDist_Q1_u6 =
			(scaled_std >= 4294967295.0f) ?
			UINT32_MAX : (uint32_t)roundf(scaled_std);

	float var_cap = fusion.std_CapDist_Q1 * fusion.std_CapDist_Q1;
	float var_tof = fusion.std_ToF_Q1 * fusion.std_ToF_Q1;
	float weight_cap = 1.0f / var_cap;
	float weight_tof = 1.0f / var_tof;

	fusion.FusedNumerator_Q1 =
			weight_cap * cap_distance + weight_tof * tof_distance;
	fusion.FusedDenominator_Q1 = weight_cap + weight_tof;
	if (!isfinite(fusion.FusedNumerator_Q1) ||
		!isfinite(fusion.FusedDenominator_Q1) ||
		(fusion.FusedDenominator_Q1 <= 0.0f))
	{
		return;
	}

	float fused_distance =
			fusion.FusedNumerator_Q1 / fusion.FusedDenominator_Q1;
	if (!isfinite(fused_distance) || (fused_distance < 0.0f))
	{
		return;
	}
    /* Bound the integer representation, not a physical distance range. */
    if ((double)fused_distance >= (double)INT32_MAX)
        fusion.FusedDistance_Q1 = INT32_MAX;
    else fusion.FusedDistance_Q1 = (int32_t)roundf(fused_distance);
	fusion.FusedDistance_Q1_valid = 1U;
}
#endif

#if LowPassFilter
uint32_t LPF(uint32_t input, uint32_t prev_input , uint32_t prev_output, float Tsampling, float tau)
{
	uint32_t output;

	output = tau/(tau + Tsampling)*prev_output + Tsampling/(tau + Tsampling)*input;

	prev_output = output;
	prev_input = input;

	return output;
}
#endif

float TMP117_RawToCelsius(uint8_t msb, uint8_t lsb)
{
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return raw * 0.0078125f; // LSB 당 0.0078125°C
}

//float TMP117_Celsius(uint8_t msb, uint8_t lsb)
//{
//    int16_t raw = ((int16_t)msb << 8) | lsb;   // 2's complement
//    return (float)raw * 0.0078125f;            // 1 LSB = 0.0078125°C
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
