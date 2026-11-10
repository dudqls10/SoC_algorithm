/*
 * NXP3110 analog/FDC register bridge over I2C.
 *
 * The NXP3110 firmware exposes 0x4000_3000..0x4000_3078 as 32-bit,
 * little-endian registers at 4-byte aligned I2C sub-addresses 0x00..0x78.
 */
#ifndef INC_FDC_H_
#define INC_FDC_H_

#include "stm32f7xx_hal.h"

#define FDC_I2C_ADDR                    (0x70U << 1)
#define FDC_I2C_TIMEOUT_MS              20U
#define FDC_I2C_WRITE_TIMEOUT_MS        200U
#define FDC_SOC_STARTUP_DELAY_MS        5000U
#define FDC_SOC_READY_TIMEOUT_MS        5000U
#define FDC_SOC_READY_POLL_MS           100U
#define FDC_I2C_TRANSACTION_GAP_MS      2U
#define FDC_INIT_RETRY_COUNT            2U
#define FDC_INIT_RETRY_DELAY_MS         10U
#define FDC_WRITE_VERIFY_RETRY_COUNT    1U
#define FDC_CONVERSION_WARMUP_MS        100U
#define FDC_WARMUP_DISCARD_SAMPLES      5U
#define FDC_WARMUP_SAMPLE_DELAY_MS      2U
#define FDC_I2C_RECOVERY_THRESHOLD      3U

#define FDC_REGISTER_SIZE               4U
#define FDC_REGISTER_MAX_OFFSET         0x78U
#define FDC_CH0_DATA_SIZE               3U
#define FDC_SNAPSHOT_SIZE               19U
#define FDC_CH0_FULL_SCALE              0x00FFFFFFUL
#define FDC_CH0_SATURATION_GUARD        0x00FF0000UL

/* NXP3110 datasheet, chapter 15.2: offset from analog base 0x4000_3000. */
#define FDC_REG_PLL_CTRL                0x00U
#define FDC_REG_TEST_CTRL               0x04U
#define FDC_REG_BGR_CTRL1               0x08U
#define FDC_REG_BGR_CTRL2               0x0CU
#define FDC_REG_LDO_CTRL                0x10U
#define FDC_REG_OSC_CTRL                0x14U
#define FDC_REG_STEN1                   0x18U
#define FDC_REG_STEN2                   0x1CU
#define FDC_REG_STEN3                   0x20U
#define FDC_REG_CONV_TIME_CH0           0x24U
#define FDC_REG_CONV_TIME_CH1           0x28U
#define FDC_REG_CONV_TIME_CH2           0x2CU
#define FDC_REG_CONV_TIME_CH3           0x30U
#define FDC_REG_SETTLE_TIME_CH0         0x34U
#define FDC_REG_SETTLE_TIME_CH1         0x38U
#define FDC_REG_SETTLE_TIME_CH2         0x3CU
#define FDC_REG_SETTLE_TIME_CH3         0x40U
#define FDC_REG_OFFSET_CH0              0x44U
#define FDC_REG_OFFSET_CH1              0x48U
#define FDC_REG_OFFSET_CH2              0x4CU
#define FDC_REG_OFFSET_CH3              0x50U
#define FDC_REG_SWITCH_CTRL             0x54U
#define FDC_REG_DRIVE_CTRL              0x58U
#define FDC_REG_SHIELD_CTRL             0x5CU
#define FDC_REG_CONFIG                  0x60U
#define FDC_REG_CH0_DATA                0x64U
#define FDC_REG_CH1_DATA                0x68U
#define FDC_REG_CH2_DATA                0x6CU
#define FDC_REG_CH3_DATA                0x70U
#define FDC_REG_DELTA_CH01              0x74U
#define FDC_REG_DELTA_CH23              0x78U

/* TIME6 / FORMAT9: 16.384ms, same RAW frequency scale as TIME8/FORMAT11. */
#define FDC_CH0_CONV_REG_VALUE          0x69U
#define FDC_CH1_CONV_REG_VALUE          0x69U
#define FDC_CH0_DATA_FORMAT             (FDC_CH0_CONV_REG_VALUE & 0x0FU)
#define FDC_CH1_DATA_FORMAT             (FDC_CH1_CONV_REG_VALUE & 0x0FU)
#define FDC_CH0_CONV_TIME_US            16384U
#define FDC_CH1_CONV_TIME_US            16384U
#define FDC_CH0_SETTLE_VALUE            0x0EA6U
#define FDC_CH1_SETTLE_VALUE            0x0EA6U

#define FDC_SAMPLE_REJECT_NONE          0x00U
#define FDC_SAMPLE_REJECT_I2C           (1U << 0)
#define FDC_SAMPLE_REJECT_FR            (1U << 1)
#define FDC_SAMPLE_REJECT_ERR           (1U << 2)
#define FDC_SAMPLE_REJECT_FULL_SCALE    (1U << 3)

/* Virtual status layout used by the legacy application. */
#define FDC_STATUS_FR_CH0                (1U << 0)
#define FDC_STATUS_FR_CH1                (1U << 1)
#define FDC_STATUS_FR_CH2                (1U << 2)
#define FDC_STATUS_FR_CH3                (1U << 3)
#define FDC_STATUS_ERR_CH0               (1U << 4)
#define FDC_STATUS_ERR_CH1               (1U << 5)
#define FDC_STATUS_ERR_CH2               (1U << 6)
#define FDC_STATUS_ERR_CH3               (1U << 7)
#define FDC_STATUS_OVF_CH0               FDC_STATUS_FR_CH0
#define FDC_STATUS_OVF_CH1               FDC_STATUS_FR_CH1
#define FDC_STATUS_OVF_CH2               FDC_STATUS_FR_CH2
#define FDC_STATUS_OVF_CH3               FDC_STATUS_FR_CH3

/* CHxDR flags in the real 32-bit NXP3110 data register. */
#define FDC_DATA_VALUE_Msk               0x00FFFFFFUL
#define FDC_DATA_ERROR                   (1UL << 30)
#define FDC_DATA_OVERFLOW                (1UL << 31)

#define FDC_CONV_FORMAT_Pos              0U
#define FDC_CONV_FORMAT_Msk              (0x0FU << FDC_CONV_FORMAT_Pos)
#define FDC_CONV_TIME_Pos                4U
#define FDC_CONV_TIME_Msk                (0x0FU << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_250US              (0x00U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_500US              (0x01U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_1MS                (0x02U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_2MS                (0x03U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_4MS                (0x04U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_8MS                (0x05U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_16MS               (0x06U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_32MS               (0x07U << FDC_CONV_TIME_Pos)
#define FDC_CONV_TIME_64MS               (0x08U << FDC_CONV_TIME_Pos)
#define FDC_DATA_FORMAT(value)           ((uint8_t)((value) & 0x0FU))
#define FDC_CONV_CONFIG(time, format)    ((uint8_t)((time) | FDC_DATA_FORMAT(format)))

#define FDC_CONFIG_ACTEN                 (1UL << 0)
#define FDC_CONFIG_MODE_Pos              1U
#define FDC_CONFIG_MODE_Msk              (0x07UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_MODE_CH0              (0x00UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_MODE_CH1              (0x01UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_MODE_CH2              (0x02UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_MODE_CH3              (0x03UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_MODE_SCAN             (0x04UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_MODE_PARALLEL         (0x05UL << FDC_CONFIG_MODE_Pos)
#define FDC_CONFIG_DELDIR                (1UL << 4)
#define FDC_CONFIG_DEL_DIR               FDC_CONFIG_DELDIR
#define FDC_CONFIG_GAIN_Pos              5U
#define FDC_CONFIG_GAIN_Msk              (0x03UL << FDC_CONFIG_GAIN_Pos)
#define FDC_CONFIG_GAIN_X1               (0x00UL << FDC_CONFIG_GAIN_Pos)
#define FDC_CONFIG_GAIN_DIV4             (0x01UL << FDC_CONFIG_GAIN_Pos)
#define FDC_CONFIG_GAIN_DIV8             (0x02UL << FDC_CONFIG_GAIN_Pos)
#define FDC_CONFIG_GAIN_DIV16            (0x03UL << FDC_CONFIG_GAIN_Pos)

/*
 * CSCR controls the optional AC-shield input path; it does not select the
 * conversion channel. Active Shield output drive is configured separately
 * through SHCR.
 */
#define FDC_SWITCH_ACS_OFF               0x00UL
#define FDC_SWITCH_ACS_CH01_ON           0x08UL
#define FDC_SWITCH_ACS_ALL_ON            0x88UL
#define FDC_SWITCH_ACS_ACTIVE            FDC_SWITCH_ACS_CH01_ON
#define FDC_SHIELD_DRIVE_MINIMUM         0x00UL
#define FDC_SHIELD_DRIVE_ACTIVE          0x08UL
#define FDC_DRIVE_VALUE_INITIAL          0x0AUL
#define FDC_DRIVE_VALUE_KICK             0x0FUL
#define FDC_DRIVE_VALUE_NORMAL           0x0AUL
/* CH0 feeds fdc300 and CH1 feeds fdc302, so convert both together. */
#define FDC_ACTIVE_MODE                  FDC_CONFIG_MODE_PARALLEL
#define FDC_ACTIVE_CONFIG_OFF            FDC_ACTIVE_MODE
#define FDC_ACTIVE_CONFIG_ON             (FDC_ACTIVE_MODE | FDC_CONFIG_ACTEN)

/* CAN 0x47 configuration read-back mask. */
#define FDC_READBACK_RD0_OK               (1U << 0)
#define FDC_READBACK_ST0_OK               (1U << 1)
#define FDC_READBACK_O0_OK                (1U << 2)
#define FDC_READBACK_CSCR_OK              (1U << 3)
#define FDC_READBACK_DCCR_OK              (1U << 4)
#define FDC_READBACK_CCR_OK               (1U << 5)
#define FDC_READBACK_SHCR_OK              (1U << 6)
#define FDC_READBACK_ALL_OK               0x7FU

extern I2C_HandleTypeDef hi2c3;
extern volatile uint8_t fdc_init_last_reg;
extern volatile HAL_StatusTypeDef fdc_init_status;
extern volatile uint32_t fdc_init_i2c_error;

/* CAN-visible I2C diagnostics. Values survive later recovery attempts. */
#define FDC_DIAG_OP_NONE                  0x00U
#define FDC_DIAG_OP_INIT_WRITE            0x01U
#define FDC_DIAG_OP_INIT_READBACK         0x02U
#define FDC_DIAG_OP_CHANNEL_READ          0x03U
#define FDC_DIAG_OP_RESTART               0x04U
#define FDC_DIAG_OP_DEVICE_READY          0x05U
#define FDC_DIAG_OP_BUS_CLEAR             0x06U
#define FDC_DIAG_OP_VERIFY_MISMATCH       0x07U

extern volatile uint8_t fdc_diag_first_valid;
extern volatile uint8_t fdc_diag_first_op;
extern volatile uint8_t fdc_diag_first_reg;
extern volatile uint8_t fdc_diag_first_status;
extern volatile uint8_t fdc_diag_first_error;
extern volatile uint8_t fdc_diag_first_state;
extern volatile uint16_t fdc_diag_first_isr;
extern volatile uint16_t fdc_diag_first_xfer_count;
extern volatile uint16_t fdc_diag_first_xfer_size;
extern volatile uint16_t fdc_diag_first_elapsed_ms;
extern volatile uint8_t fdc_diag_first_lines;
extern volatile uint32_t fdc_diag_first_cr2;
extern volatile uint8_t fdc_diag_last_op;
extern volatile uint8_t fdc_diag_last_reg;
extern volatile uint8_t fdc_diag_last_status;
extern volatile uint8_t fdc_diag_last_error;
extern volatile uint8_t fdc_diag_last_state;
extern volatile uint16_t fdc_diag_last_isr;
extern volatile uint8_t fdc_config_readback_mask;
extern volatile uint8_t fdc_readback_rd0;
extern volatile uint16_t fdc_readback_st0;
extern volatile uint8_t fdc_readback_cscr;
extern volatile uint8_t fdc_readback_shcr;
extern volatile uint8_t fdc_readback_dccr;
extern volatile uint8_t fdc_readback_ccr;
extern volatile uint8_t fdc_bus_clear_count;
extern volatile uint8_t fdc_bus_clear_before;
extern volatile uint8_t fdc_bus_clear_released;
extern volatile uint8_t fdc_bus_clear_pulsed;
extern volatile uint8_t fdc_bus_clear_stopped;
extern volatile uint8_t fdc_bus_clear_after_init;
extern volatile uint8_t fdc_bus_clear_status;
extern volatile uint8_t fdc_soc_ready_seen;
extern volatile uint8_t fdc_soc_ready_status;
extern volatile uint8_t fdc_soc_ready_error;
extern volatile uint8_t fdc_soc_ready_attempts;
extern volatile uint16_t fdc_soc_ready_elapsed_ms;

void FDC_RecordDiagnostic(I2C_HandleTypeDef *hi2c, uint8_t operation,
                          uint8_t reg, HAL_StatusTypeDef status);

HAL_StatusTypeDef FDC_WriteReg32(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                 uint32_t value);
HAL_StatusTypeDef FDC_ReadReg32(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                uint32_t *value);
HAL_StatusTypeDef FDC_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
                               uint8_t data);
HAL_StatusTypeDef FDC_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
                              uint8_t *data);
HAL_StatusTypeDef FDC_ReadCH0Data(I2C_HandleTypeDef *hi2c,
                                 uint8_t data[FDC_CH0_DATA_SIZE]);
HAL_StatusTypeDef FDC_ReadCH1Data(I2C_HandleTypeDef *hi2c,
                                 uint8_t data[FDC_CH0_DATA_SIZE]);
HAL_StatusTypeDef FDC_ReadCH0Sample(I2C_HandleTypeDef *hi2c,
                                   uint8_t data[FDC_CH0_DATA_SIZE],
                                   uint8_t *status);
HAL_StatusTypeDef FDC_ReadCH1Sample(I2C_HandleTypeDef *hi2c,
                                   uint8_t data[FDC_CH0_DATA_SIZE],
                                   uint8_t *status);
HAL_StatusTypeDef FDC_ReadSnapshot(I2C_HandleTypeDef *hi2c,
                                  uint8_t snapshot[FDC_SNAPSHOT_SIZE]);
HAL_StatusTypeDef FDC_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FDC_InitWithRetry(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FDC_WarmupAndDiscard(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FDC_CaptureConfigReadback(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FDC_ClearI2C3Bus(I2C_HandleTypeDef *hi2c);
uint8_t FDC_GetI2C3LineState(void);
void FDC_SleepMode(I2C_HandleTypeDef *hi2c);
void FDC_ActiveMode(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FDC_RestartConversion(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef FDC_Raw_ReadBoth(I2C_HandleTypeDef *hi2c,
                                   uint8_t ch0_reg, uint8_t ch1_reg,
                                   uint8_t *ch0_at_0, uint8_t *ch0_at_2,
                                   uint8_t *ch1_at_0, uint8_t *ch1_at_2);
HAL_StatusTypeDef FDC_Raw_Read_CH0(I2C_HandleTypeDef *hi2c, uint8_t ch0_reg,
                                  uint8_t ch0_data[4]);
HAL_StatusTypeDef FDC_Raw_Read_CH1(I2C_HandleTypeDef *hi2c, uint8_t ch1_reg,
                                  uint8_t *ch1_at_0, uint8_t *ch1_at_2);

#endif
