/*
******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Aixcel Co.,Ltd</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Aixcel nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  ******************************************************************************
  */

#ifndef BAROMETER_PROCESS_H
#define BAROMETER_PROCESS_H

#ifdef __cplusplus
  extern "C" {
#endif

#include "common.h"
#include "at32f4xx_i2c.h"
#include "spl06_001_drv.h"

typedef struct __Board_Barameter_Dev_Typedef
{
    SPL06_001_Drv_Typedef SPL06_001_Drv;
    
    float                      Altitude;
    
} Board_Barameter_Dev_Typedef;

extern Board_Barameter_Dev_Typedef BarometerDev;

int32_t Barometer_Dev_Init(void);

int32_t I2C_Barometer_Chip_Select(void);

int32_t I2C_Barometer_Chip_Deselect(void);

int32_t I2C_Barometer_Write_Data(uint8_t dev_id, uint8_t reg_addr, uint8_t* bufp, uint8_t len);

int32_t I2C_Barometer_Read_Data(uint8_t dev_id, uint8_t reg_addr, uint8_t* bufp, uint8_t len);

int32_t I2C_Barometer_Write_D(uint8_t dev_id, uint8_t reg_addr, uint8_t* bufp, uint8_t len);

int32_t I2C_Barometer_Read_D(uint8_t dev_id, uint8_t reg_addr, uint8_t* bufp, uint8_t len);

float Board_Barometer_Calc_Altitude(float barometric_pres);

void Barometer_Process(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_BAROMETER_H */
