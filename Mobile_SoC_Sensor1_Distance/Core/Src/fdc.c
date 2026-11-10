#include "fdc.h"
#include <string.h>

volatile uint8_t fdc_init_last_reg = 0U;
volatile HAL_StatusTypeDef fdc_init_status = HAL_OK;
volatile uint32_t fdc_init_i2c_error = HAL_I2C_ERROR_NONE;
volatile uint8_t fdc_diag_first_valid = 0U;
volatile uint8_t fdc_diag_first_op = FDC_DIAG_OP_NONE;
volatile uint8_t fdc_diag_first_reg = 0U;
volatile uint8_t fdc_diag_first_status = HAL_OK;
volatile uint8_t fdc_diag_first_error = 0U;
volatile uint8_t fdc_diag_first_state = 0U;
volatile uint16_t fdc_diag_first_isr = 0U;
volatile uint16_t fdc_diag_first_xfer_count = 0U;
volatile uint16_t fdc_diag_first_xfer_size = 0U;
volatile uint16_t fdc_diag_first_elapsed_ms = 0U;
volatile uint8_t fdc_diag_first_lines = 0U;
volatile uint32_t fdc_diag_first_cr2 = 0U;
volatile uint8_t fdc_diag_last_op = FDC_DIAG_OP_NONE;
volatile uint8_t fdc_diag_last_reg = 0U;
volatile uint8_t fdc_diag_last_status = HAL_OK;
volatile uint8_t fdc_diag_last_error = 0U;
volatile uint8_t fdc_diag_last_state = 0U;
volatile uint16_t fdc_diag_last_isr = 0U;
volatile uint8_t fdc_config_readback_mask = 0U;
volatile uint8_t fdc_readback_rd0 = 0U;
volatile uint16_t fdc_readback_st0 = 0U;
volatile uint8_t fdc_readback_cscr = 0U;
volatile uint8_t fdc_readback_shcr = 0U;
volatile uint8_t fdc_readback_dccr = 0U;
volatile uint8_t fdc_readback_ccr = 0U;
volatile uint8_t fdc_bus_clear_count = 0U;
volatile uint8_t fdc_bus_clear_before = 0U;
volatile uint8_t fdc_bus_clear_released = 0U;
volatile uint8_t fdc_bus_clear_pulsed = 0U;
volatile uint8_t fdc_bus_clear_stopped = 0U;
volatile uint8_t fdc_bus_clear_after_init = 0U;
volatile uint8_t fdc_bus_clear_status = HAL_OK;
volatile uint8_t fdc_soc_ready_seen = 0U;
volatile uint8_t fdc_soc_ready_status = HAL_ERROR;
volatile uint8_t fdc_soc_ready_error = 0U;
volatile uint8_t fdc_soc_ready_attempts = 0U;
volatile uint16_t fdc_soc_ready_elapsed_ms = 0U;

static uint8_t s_cached_status;
static uint16_t s_last_transfer_elapsed_ms;

static uint8_t FDC_I2C3LineState(uint8_t include_busy);

void FDC_RecordDiagnostic(I2C_HandleTypeDef *hi2c, uint8_t operation,
                          uint8_t reg, HAL_StatusTypeDef status)
{
  uint8_t error = 0U;
  uint8_t state = 0U;
  uint16_t isr = 0U;

  if (hi2c != NULL)
  {
    error = (uint8_t)HAL_I2C_GetError(hi2c);
    state = (uint8_t)HAL_I2C_GetState(hi2c);
    if (hi2c->Instance != NULL)
    {
      isr = (uint16_t)hi2c->Instance->ISR;
    }
  }

  fdc_diag_last_op = operation;
  fdc_diag_last_reg = reg;
  fdc_diag_last_status = (uint8_t)status;
  fdc_diag_last_error = error;
  fdc_diag_last_state = state;
  fdc_diag_last_isr = isr;

  if ((status != HAL_OK) && (fdc_diag_first_valid == 0U))
  {
    fdc_diag_first_valid = 1U;
    fdc_diag_first_op = operation;
    fdc_diag_first_reg = reg;
    fdc_diag_first_status = (uint8_t)status;
    fdc_diag_first_error = error;
    fdc_diag_first_state = state;
    fdc_diag_first_isr = isr;
    fdc_diag_first_elapsed_ms = s_last_transfer_elapsed_ms;
    if ((hi2c != NULL) && (hi2c->Instance != NULL))
    {
      fdc_diag_first_xfer_count = hi2c->XferCount;
      fdc_diag_first_xfer_size = hi2c->XferSize;
      fdc_diag_first_lines = FDC_I2C3LineState(1U);
      fdc_diag_first_cr2 = hi2c->Instance->CR2;
    }
  }
}

static uint8_t FDC_IsValidRegister(uint8_t reg)
{
  return (uint8_t)((reg <= FDC_REGISTER_MAX_OFFSET) &&
                   ((reg & (FDC_REGISTER_SIZE - 1U)) == 0U));
}

static uint32_t FDC_RegisterVerifyMask(uint8_t reg)
{
  switch (reg)
  {
    case FDC_REG_CONV_TIME_CH0:
    case FDC_REG_CONV_TIME_CH1: return 0x000000FFUL;
    case FDC_REG_SETTLE_TIME_CH0:
    case FDC_REG_SETTLE_TIME_CH1:
    case FDC_REG_OFFSET_CH0:
    case FDC_REG_OFFSET_CH1:    return 0x0000FFFFUL;
    case FDC_REG_SWITCH_CTRL:
    case FDC_REG_SHIELD_CTRL:   return 0x000000FFUL;
    case FDC_REG_DRIVE_CTRL:    return 0x0000008FUL;
    case FDC_REG_CONFIG:        return 0x0000007FUL;
    default:                    return 0xFFFFFFFFUL;
  }
}

static uint8_t FDC_I2C3LineState(uint8_t include_busy)
{
  uint8_t state = 0U;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
    state |= (1U << 0);
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET)
    state |= (1U << 1);
  if ((include_busy != 0U) && ((I2C3->ISR & I2C_ISR_BUSY) != 0U))
    state |= (1U << 2);
  return state;
}

uint8_t FDC_GetI2C3LineState(void)
{
  return FDC_I2C3LineState(1U);
}

static HAL_StatusTypeDef FDC_ReadBytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                       uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;
  uint32_t started;

  if ((hi2c == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  /*
   * Match the supplied F401 bridge: write the one-byte sub-address and use
   * a repeated START for the 32-bit register read.
   */
  started = HAL_GetTick();
  status = HAL_I2C_Mem_Read(hi2c, FDC_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, length,
                            FDC_I2C_TIMEOUT_MS);
  s_last_transfer_elapsed_ms = (uint16_t)(HAL_GetTick() - started);
  return status;
}

HAL_StatusTypeDef FDC_WriteReg32(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                 uint32_t value)
{
  uint8_t data[4];
  HAL_StatusTypeDef status;
  uint32_t started;

  if ((hi2c == NULL) || (FDC_IsValidRegister(reg) == 0U))
  {
    return HAL_ERROR;
  }

  /* Match the verified F401 bridge transaction and its sub-address phase. */
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
  started = HAL_GetTick();
  status = HAL_I2C_Mem_Write(hi2c, FDC_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                             FDC_I2C_WRITE_TIMEOUT_MS);
  s_last_transfer_elapsed_ms = (uint16_t)(HAL_GetTick() - started);
  return status;
}

HAL_StatusTypeDef FDC_ReadReg32(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                uint32_t *value)
{
  uint8_t rx[4];
  HAL_StatusTypeDef status;

  if ((value == NULL) || (FDC_IsValidRegister(reg) == 0U))
  {
    return HAL_ERROR;
  }

  status = FDC_ReadBytes(hi2c, reg, rx, sizeof(rx));
  if (status == HAL_OK)
  {
    *value = (uint32_t)rx[0] | ((uint32_t)rx[1] << 8) |
             ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 24);
  }
  return status;
}

HAL_StatusTypeDef FDC_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
                               uint8_t data)
{
  return FDC_WriteReg32(hi2c, reg, data);
}

HAL_StatusTypeDef FDC_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
                              uint8_t *data)
{
  uint32_t value;
  HAL_StatusTypeDef status;

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  status = FDC_ReadReg32(hi2c, reg, &value);
  if (status == HAL_OK)
  {
    *data = (uint8_t)value;
  }
  return status;
}

static HAL_StatusTypeDef FDC_ReadChannel(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                         uint8_t channel,
                                         uint8_t data[FDC_CH0_DATA_SIZE])
{
  uint32_t word;
  HAL_StatusTypeDef status;
  uint8_t overflow_bit = (uint8_t)(1U << channel);
  uint8_t error_bit = (uint8_t)(1U << (channel + 4U));

  if (data == NULL)
  {
    return HAL_ERROR;
  }
  status = FDC_ReadReg32(hi2c, reg, &word);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_CHANNEL_READ, reg, status);
  if (status != HAL_OK)
  {
    return status;
  }

  s_cached_status &= (uint8_t)~(overflow_bit | error_bit);
  if ((word & FDC_DATA_OVERFLOW) != 0U) s_cached_status |= overflow_bit;
  if ((word & FDC_DATA_ERROR) != 0U) s_cached_status |= error_bit;
  word &= FDC_DATA_VALUE_Msk;
  data[0] = (uint8_t)word;
  data[1] = (uint8_t)(word >> 8);
  data[2] = (uint8_t)(word >> 16);
  return HAL_OK;
}

HAL_StatusTypeDef FDC_ReadCH0Data(I2C_HandleTypeDef *hi2c,
                                  uint8_t data[FDC_CH0_DATA_SIZE])
{
  return FDC_ReadChannel(hi2c, FDC_REG_CH0_DATA, 0U, data);
}

HAL_StatusTypeDef FDC_ReadCH1Data(I2C_HandleTypeDef *hi2c,
                                  uint8_t data[FDC_CH0_DATA_SIZE])
{
  return FDC_ReadChannel(hi2c, FDC_REG_CH1_DATA, 1U, data);
}

HAL_StatusTypeDef FDC_ReadCH1Sample(I2C_HandleTypeDef *hi2c,
                                    uint8_t data[FDC_CH0_DATA_SIZE],
                                    uint8_t *status)
{
  HAL_StatusTypeDef result;

  if (status == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * NXP3110 has no separate STATUS register. CH1DR is one coherent 32-bit
   * register: bit31=overflow, bit30=error, bits23:0=conversion result.
   */
  result = FDC_ReadCH1Data(hi2c, data);
  if (result == HAL_OK)
  {
    *status = s_cached_status;
  }
  return result;
}

HAL_StatusTypeDef FDC_ReadCH0Sample(I2C_HandleTypeDef *hi2c,
                                    uint8_t data[FDC_CH0_DATA_SIZE],
                                    uint8_t *status)
{
  HAL_StatusTypeDef result;

  if (status == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * CH0DR is one coherent 32-bit little-endian register:
   * bit31=overflow, bit30=error, bits23:0=conversion result.
   */
  result = FDC_ReadCH0Data(hi2c, data);
  if (result == HAL_OK)
  {
    *status = s_cached_status;
  }
  return result;
}

HAL_StatusTypeDef FDC_ReadSnapshot(I2C_HandleTypeDef *hi2c,
                                   uint8_t snapshot[FDC_SNAPSHOT_SIZE])
{
  uint8_t channel[FDC_CH0_DATA_SIZE];
  HAL_StatusTypeDef status;

  if (snapshot == NULL) return HAL_ERROR;
  memset(snapshot, 0, FDC_SNAPSHOT_SIZE);
  for (uint8_t ch = 0U; ch < 4U; ++ch)
  {
    status = FDC_ReadChannel(hi2c, (uint8_t)(FDC_REG_CH0_DATA + 4U * ch),
                             ch, channel);
    if (status != HAL_OK) return status;
    memcpy(&snapshot[3U * ch], channel, FDC_CH0_DATA_SIZE);
    if (ch < 3U) HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
  }
  snapshot[18] = s_cached_status;
  return HAL_OK;
}

static HAL_StatusTypeDef FDC_InitWriteVerified(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg, uint32_t value)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t readback = 0U;
  uint32_t verify_mask = FDC_RegisterVerifyMask(reg);

  fdc_init_last_reg = reg;
  for (uint32_t attempt = 0U; attempt < FDC_WRITE_VERIFY_RETRY_COUNT; ++attempt)
  {
    status = FDC_WriteReg32(hi2c, reg, value);
    FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_WRITE, reg, status);
    if (status == HAL_OK)
    {
      /* NXP3110 slave needs an idle gap to close the previous STOP state. */
      HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
      status = FDC_ReadReg32(hi2c, reg, &readback);
      FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK, reg, status);
      if ((status == HAL_OK) &&
          ((readback & verify_mask) == (value & verify_mask)))
      {
        fdc_init_status = HAL_OK;
        fdc_init_i2c_error = HAL_I2C_ERROR_NONE;
        HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
        return HAL_OK;
      }
      if (status == HAL_OK)
      {
        status = HAL_ERROR;
        FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_VERIFY_MISMATCH, reg, status);
      }
    }
    fdc_init_status = status;
    fdc_init_i2c_error = HAL_I2C_GetError(hi2c);
  }
  return status;
}

HAL_StatusTypeDef FDC_ClearI2C3Bus(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio = {0};
  HAL_StatusTypeDef status;
  uint8_t scl_released;

  if ((hi2c == NULL) || (hi2c->Instance != I2C3))
  {
    return HAL_ERROR;
  }

  /*
   * Release a transaction interrupted by an MCU reset.  PA8=SCL and PC9=SDA
   * are temporarily driven open-drain, then a STOP is generated before the
   * I2C peripheral is re-enabled.
   */
  if (fdc_bus_clear_count < 0xFFU) fdc_bus_clear_count++;
  fdc_bus_clear_before = FDC_I2C3LineState(1U);
  (void)HAL_I2C_DeInit(hi2c);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
  fdc_bus_clear_released = FDC_I2C3LineState(0U);

  for (uint32_t pulse = 0U; pulse < 9U; ++pulse)
  {
    if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) &&
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET))
    {
      break;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1U);
  }
  fdc_bus_clear_pulsed = FDC_I2C3LineState(0U);

  /*
   * STOP is valid only after SCL was actually released.  If the slave still
   * stretches SCL, do not pretend that toggling the GPIO generated a STOP.
   */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
  scl_released =
      (uint8_t)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET);
  if (scl_released != 0U)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1U);
  }
  fdc_bus_clear_stopped = FDC_I2C3LineState(0U);

  status = HAL_I2C_Init(hi2c);
  if ((status == HAL_OK) && (scl_released == 0U))
  {
    status = HAL_TIMEOUT;
  }
  fdc_bus_clear_after_init = FDC_I2C3LineState(1U);
  fdc_bus_clear_status = (uint8_t)status;
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_BUS_CLEAR, 0U, status);
  return status;
}

void FDC_SleepMode(I2C_HandleTypeDef *hi2c)
{
  (void)FDC_WriteReg32(hi2c, FDC_REG_CONFIG, FDC_ACTIVE_CONFIG_OFF);
}

void FDC_ActiveMode(I2C_HandleTypeDef *hi2c)
{
  (void)FDC_RestartConversion(hi2c);
}

HAL_StatusTypeDef FDC_RestartConversion(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;

  /* Keep Active Shield enabled only for the CH0/CH1 pair. */
  status = FDC_WriteReg32(hi2c, FDC_REG_SWITCH_CTRL,
                          FDC_SWITCH_ACS_ACTIVE);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_RESTART,
                       FDC_REG_SWITCH_CTRL, status);
  if (status != HAL_OK) return status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
  status = FDC_WriteReg32(hi2c, FDC_REG_SHIELD_CTRL, FDC_SHIELD_DRIVE_ACTIVE);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_RESTART,
                       FDC_REG_SHIELD_CTRL, status);
  if (status != HAL_OK) return status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  /* Keep the verified trigger writes back-to-back. */
  status = FDC_WriteReg32(hi2c, FDC_REG_CONFIG, FDC_ACTIVE_CONFIG_ON);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_RESTART, FDC_REG_CONFIG, status);
  if (status != HAL_OK) return status;
  status = FDC_WriteReg32(hi2c, FDC_REG_DRIVE_CTRL,
                          FDC_DRIVE_VALUE_KICK);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_RESTART, FDC_REG_DRIVE_CTRL, status);
  if (status != HAL_OK) return status;
  status = FDC_WriteReg32(hi2c, FDC_REG_DRIVE_CTRL,
                          FDC_DRIVE_VALUE_NORMAL);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_RESTART, FDC_REG_DRIVE_CTRL, status);
  return status;
}

static uint8_t FDC_ReadbackMatches(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                   uint32_t value, uint32_t expected)
{
  uint32_t mask = FDC_RegisterVerifyMask(reg);

  if ((value & mask) == (expected & mask)) return 1U;
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_VERIFY_MISMATCH, reg, HAL_ERROR);
  return 0U;
}

HAL_StatusTypeDef FDC_CaptureConfigReadback(I2C_HandleTypeDef *hi2c)
{
  uint32_t value = 0U;
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef result = HAL_OK;

  if (hi2c == NULL) return HAL_ERROR;
  fdc_config_readback_mask = 0U;

  status = FDC_ReadReg32(hi2c, FDC_REG_CONV_TIME_CH0, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_CONV_TIME_CH0, status);
  if (status == HAL_OK)
  {
    fdc_readback_rd0 = (uint8_t)value;
    if (FDC_ReadbackMatches(hi2c, FDC_REG_CONV_TIME_CH0, value,
                            FDC_CH0_CONV_REG_VALUE) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_RD0_OK;
    else result = HAL_ERROR;
  }
  else result = status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  status = FDC_ReadReg32(hi2c, FDC_REG_SETTLE_TIME_CH0, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_SETTLE_TIME_CH0, status);
  if (status == HAL_OK)
  {
    fdc_readback_st0 = (uint16_t)value;
    if (FDC_ReadbackMatches(hi2c, FDC_REG_SETTLE_TIME_CH0, value,
                            FDC_CH0_SETTLE_VALUE) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_ST0_OK;
    else result = HAL_ERROR;
  }
  else result = status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  status = FDC_ReadReg32(hi2c, FDC_REG_OFFSET_CH0, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_OFFSET_CH0, status);
  if (status == HAL_OK)
  {
    if (FDC_ReadbackMatches(hi2c, FDC_REG_OFFSET_CH0, value, 0U) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_O0_OK;
    else result = HAL_ERROR;
  }
  else result = status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  status = FDC_ReadReg32(hi2c, FDC_REG_SWITCH_CTRL, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_SWITCH_CTRL, status);
  if (status == HAL_OK)
  {
    fdc_readback_cscr = (uint8_t)value;
    if (FDC_ReadbackMatches(hi2c, FDC_REG_SWITCH_CTRL, value,
                            FDC_SWITCH_ACS_ACTIVE) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_CSCR_OK;
    else result = HAL_ERROR;
  }
  else result = status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  status = FDC_ReadReg32(hi2c, FDC_REG_SHIELD_CTRL, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_SHIELD_CTRL, status);
  if (status == HAL_OK)
  {
    fdc_readback_shcr = (uint8_t)value;
    if (FDC_ReadbackMatches(hi2c, FDC_REG_SHIELD_CTRL, value,
                            FDC_SHIELD_DRIVE_ACTIVE) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_SHCR_OK;
    else result = HAL_ERROR;
  }
  else result = status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  status = FDC_ReadReg32(hi2c, FDC_REG_DRIVE_CTRL, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_DRIVE_CTRL, status);
  if (status == HAL_OK)
  {
    fdc_readback_dccr = (uint8_t)value;
    if (FDC_ReadbackMatches(hi2c, FDC_REG_DRIVE_CTRL, value,
                            FDC_DRIVE_VALUE_NORMAL) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_DCCR_OK;
    else result = HAL_ERROR;
  }
  else result = status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);

  status = FDC_ReadReg32(hi2c, FDC_REG_CONFIG, &value);
  FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_INIT_READBACK,
                       FDC_REG_CONFIG, status);
  if (status == HAL_OK)
  {
    fdc_readback_ccr = (uint8_t)value;
    if (FDC_ReadbackMatches(hi2c, FDC_REG_CONFIG, value,
                            FDC_ACTIVE_CONFIG_ON) != 0U)
      fdc_config_readback_mask |= FDC_READBACK_CCR_OK;
    else result = HAL_ERROR;
  }
  else result = status;

  return result;
}

HAL_StatusTypeDef FDC_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;

  if (hi2c == NULL) return HAL_ERROR;
  fdc_init_last_reg = 0U;
  fdc_init_status = HAL_OK;
  fdc_init_i2c_error = HAL_I2C_ERROR_NONE;
  s_cached_status = 0U;
  fdc_config_readback_mask = 0U;

  status = FDC_InitWriteVerified(hi2c, FDC_REG_CONFIG,
                                 FDC_ACTIVE_CONFIG_OFF);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_CONV_TIME_CH0,
                                 FDC_CH0_CONV_REG_VALUE);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_CONV_TIME_CH1,
                                 FDC_CH1_CONV_REG_VALUE);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_SETTLE_TIME_CH0,
                                 FDC_CH0_SETTLE_VALUE);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_SETTLE_TIME_CH1,
                                 FDC_CH1_SETTLE_VALUE);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_OFFSET_CH0, 0U);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_OFFSET_CH1, 0U);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_SWITCH_CTRL,
                                 FDC_SWITCH_ACS_ACTIVE);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_SHIELD_CTRL,
                                 FDC_SHIELD_DRIVE_ACTIVE);
  if (status != HAL_OK) return status;
  status = FDC_InitWriteVerified(hi2c, FDC_REG_DRIVE_CTRL,
                                 FDC_DRIVE_VALUE_INITIAL);
  if (status != HAL_OK) return status;
  status = FDC_RestartConversion(hi2c);
  if (status != HAL_OK) return status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
  return FDC_CaptureConfigReadback(hi2c);
}

HAL_StatusTypeDef FDC_InitWithRetry(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t started;

  if (hi2c == NULL) return HAL_ERROR;

  /*
   * Both devices can now power up together.  Do not touch the bus while the
   * NXP3110 firmware is still configuring its TWI slave.  After the guard
   * time, use address-only probes and start register writes only after ACK.
   */
  fdc_soc_ready_seen = 0U;
  fdc_soc_ready_status = HAL_ERROR;
  fdc_soc_ready_error = 0U;
  fdc_soc_ready_attempts = 0U;
  fdc_soc_ready_elapsed_ms = 0U;
  started = HAL_GetTick();
  HAL_Delay(FDC_SOC_STARTUP_DELAY_MS);

  do
  {
    if (fdc_soc_ready_attempts < 0xFFU) fdc_soc_ready_attempts++;
    status = HAL_I2C_IsDeviceReady(hi2c, FDC_I2C_ADDR, 1U,
                                   FDC_I2C_TIMEOUT_MS);
    fdc_soc_ready_status = (uint8_t)status;
    fdc_soc_ready_error = (uint8_t)HAL_I2C_GetError(hi2c);
    fdc_soc_ready_elapsed_ms =
        (uint16_t)(HAL_GetTick() - started);
    if (status == HAL_OK)
    {
      fdc_soc_ready_seen = 1U;
      fdc_soc_ready_error = 0U;
      FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_DEVICE_READY, 0U, status);
      HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
      break;
    }
    HAL_Delay(FDC_SOC_READY_POLL_MS);
  }
  while ((HAL_GetTick() - started) <
         (FDC_SOC_STARTUP_DELAY_MS + FDC_SOC_READY_TIMEOUT_MS));

  if (status != HAL_OK)
  {
    FDC_RecordDiagnostic(hi2c, FDC_DIAG_OP_DEVICE_READY, 0U, status);
    return status;
  }

  for (uint32_t attempt = 0U; attempt < FDC_INIT_RETRY_COUNT; ++attempt)
  {
    if (attempt != 0U)
    {
      status = FDC_ClearI2C3Bus(hi2c);
      if (status != HAL_OK)
      {
        HAL_Delay(FDC_INIT_RETRY_DELAY_MS);
        continue;
      }
    }
    status = FDC_Init(hi2c);
    if (status == HAL_OK) return HAL_OK;
    HAL_Delay(FDC_INIT_RETRY_DELAY_MS);
  }
  return status;
}

HAL_StatusTypeDef FDC_WarmupAndDiscard(I2C_HandleTypeDef *hi2c)
{
  uint8_t data[FDC_CH0_DATA_SIZE];
  HAL_StatusTypeDef status;

  if (hi2c == NULL) return HAL_ERROR;
  HAL_Delay(FDC_CONVERSION_WARMUP_MS);
  for (uint32_t i = 0U; i < FDC_WARMUP_DISCARD_SAMPLES; ++i)
  {
    status = FDC_ReadCH0Data(hi2c, data);
    if (status != HAL_OK) return status;
    HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
    status = FDC_ReadCH1Data(hi2c, data);
    if (status != HAL_OK) return status;
    HAL_Delay(FDC_WARMUP_SAMPLE_DELAY_MS);
  }
  return HAL_OK;
}

static void FDC_StoreBE32(uint32_t value, uint8_t *at_0, uint8_t *at_2)
{
  at_0[0] = (uint8_t)(value >> 24);
  at_0[1] = (uint8_t)(value >> 16);
  at_2[0] = (uint8_t)(value >> 8);
  at_2[1] = (uint8_t)value;
}

HAL_StatusTypeDef FDC_Raw_ReadBoth(I2C_HandleTypeDef *hi2c,
                                   uint8_t ch0_reg, uint8_t ch1_reg,
                                   uint8_t *ch0_at_0, uint8_t *ch0_at_2,
                                   uint8_t *ch1_at_0, uint8_t *ch1_at_2)
{
  uint32_t ch0, ch1;
  HAL_StatusTypeDef status;

  if ((ch0_at_0 == NULL) || (ch0_at_2 == NULL) ||
      (ch1_at_0 == NULL) || (ch1_at_2 == NULL)) return HAL_ERROR;
  status = FDC_ReadReg32(hi2c, ch0_reg, &ch0);
  if (status != HAL_OK) return status;
  HAL_Delay(FDC_I2C_TRANSACTION_GAP_MS);
  status = FDC_ReadReg32(hi2c, ch1_reg, &ch1);
  if (status != HAL_OK) return status;
  FDC_StoreBE32(ch0 & FDC_DATA_VALUE_Msk, ch0_at_0, ch0_at_2);
  FDC_StoreBE32(ch1 & FDC_DATA_VALUE_Msk, ch1_at_0, ch1_at_2);
  return HAL_OK;
}

HAL_StatusTypeDef FDC_Raw_Read_CH0(I2C_HandleTypeDef *hi2c, uint8_t ch0_reg,
                                   uint8_t ch0_data[4])
{
  uint32_t value;
  HAL_StatusTypeDef status;

  if (ch0_data == NULL) return HAL_ERROR;
  status = FDC_ReadReg32(hi2c, ch0_reg, &value);
  if (status == HAL_OK)
  {
    value &= FDC_DATA_VALUE_Msk;
    ch0_data[0] = (uint8_t)(value >> 24);
    ch0_data[1] = (uint8_t)(value >> 16);
    ch0_data[2] = (uint8_t)(value >> 8);
    ch0_data[3] = (uint8_t)value;
  }
  return status;
}

HAL_StatusTypeDef FDC_Raw_Read_CH1(I2C_HandleTypeDef *hi2c, uint8_t ch1_reg,
                                   uint8_t *ch1_at_0, uint8_t *ch1_at_2)
{
  uint32_t value;
  HAL_StatusTypeDef status;

  if ((ch1_at_0 == NULL) || (ch1_at_2 == NULL)) return HAL_ERROR;
  status = FDC_ReadReg32(hi2c, ch1_reg, &value);
  if (status == HAL_OK)
  {
    FDC_StoreBE32(value & FDC_DATA_VALUE_Msk, ch1_at_0, ch1_at_2);
  }
  return status;
}
