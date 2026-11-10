
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/
#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_config.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "vl53l1_error_codes.h"

uint8_t _I2CBuffer[256];

/* First failed VL53L1X HAL transaction, retained for CAN diagnostics. */
volatile uint8_t vl53l1_diag_phase = 0U; /* 1=register write, 2=read */
volatile uint8_t vl53l1_diag_status = 0U;
volatile uint8_t vl53l1_diag_error = 0U;
volatile uint8_t vl53l1_diag_state = 0U;
volatile uint16_t vl53l1_diag_cr2 = 0U;
volatile uint8_t vl53l1_diag_write_ok = 0U;
volatile uint8_t vl53l1_diag_read_ok = 0U;

int _I2CWrite(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint8_t *pdata, uint32_t count) {
    int status;

    status = HAL_I2C_Master_Transmit(hi2c, Dev, pdata, count, 10);
    if (status == HAL_OK) {
        vl53l1_diag_write_ok++;
    } else if (vl53l1_diag_phase == 0U) {
        vl53l1_diag_phase = 1U;
        vl53l1_diag_status = (uint8_t)status;
        vl53l1_diag_error = (uint8_t)hi2c->ErrorCode;
        vl53l1_diag_state = (uint8_t)hi2c->State;
        vl53l1_diag_cr2 = (uint16_t)hi2c->Instance->CR2;
    }
    if (status) {

    }
    return status;
}

int _I2CRead(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint8_t *pdata, uint32_t count) {
    int status;

    /* HAL sets the read direction; SADD must remain the shifted 7-bit address. */
    status = HAL_I2C_Master_Receive(hi2c, Dev, pdata, count, 10);
    if (status == HAL_OK) {
        vl53l1_diag_read_ok++;
    } else if (vl53l1_diag_phase == 0U) {
        vl53l1_diag_phase = 2U;
        vl53l1_diag_status = (uint8_t)status;
        vl53l1_diag_error = (uint8_t)hi2c->ErrorCode;
        vl53l1_diag_state = (uint8_t)hi2c->State;
        vl53l1_diag_cr2 = (uint16_t)hi2c->Instance->CR2;
    }
    if (status) {

    }
    return status;
}

static int _I2CMemRead(I2C_HandleTypeDef* hi2c, uint16_t Dev,
                       uint16_t index, uint8_t *pdata, uint16_t count) {
    int status = HAL_I2C_Mem_Read(hi2c, Dev, index, I2C_MEMADD_SIZE_16BIT,
                                  pdata, count, 25U);
    if (status == HAL_OK) {
        vl53l1_diag_read_ok++;
    } else if (vl53l1_diag_phase == 0U) {
        vl53l1_diag_phase = 3U; /* combined register-address + repeated-start read */
        vl53l1_diag_status = (uint8_t)status;
        vl53l1_diag_error = (uint8_t)hi2c->ErrorCode;
        vl53l1_diag_state = (uint8_t)hi2c->State;
        vl53l1_diag_cr2 = (uint16_t)hi2c->Instance->CR2;
    }
    return status;
}

int8_t VL53L1_WriteMulti(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 2) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    status_int = _I2CWrite(hi2c, Dev, _I2CBuffer, count + 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_ReadMulti(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count){
    if ((count == 0U) || (count > UINT16_MAX)) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    return (_I2CMemRead(hi2c, Dev, index, pdata, (uint16_t)count) == HAL_OK)
        ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

int8_t VL53L1_WrByte(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint8_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;

    status_int = _I2CWrite(hi2c, Dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_WrWord(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint16_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0x00FF;

    status_int = _I2CWrite(hi2c, Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_WrDWord(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint32_t data) {
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
    _I2CBuffer[3] = (data >> 16) & 0xFF;
    _I2CBuffer[4] = (data >> 8)  & 0xFF;
    _I2CBuffer[5] = (data >> 0 ) & 0xFF;
    status_int = _I2CWrite(hi2c, Dev, _I2CBuffer, 6);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_RdByte(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(hi2c, Dev, index, data, 1U);
}

int8_t VL53L1_RdWord(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint16_t *data) {
    VL53L1_Error Status = VL53L1_ReadMulti(hi2c, Dev, index, _I2CBuffer, 2U);
    if (Status != VL53L1_ERROR_NONE) return Status;
    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
    return Status;
}

int8_t VL53L1_RdDWord(I2C_HandleTypeDef* hi2c, uint16_t Dev, uint16_t index, uint32_t *data) {
    VL53L1_Error Status = VL53L1_ReadMulti(hi2c, Dev, index, _I2CBuffer, 4U);
    if (Status != VL53L1_ERROR_NONE) return Status;
    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];
    return Status;
}

int8_t VL53L1_WaitMs(I2C_HandleTypeDef* hi2c, uint16_t Dev, int32_t wait_ms){

	return 0;
}
