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

#include "barometer_process.h"
#include "hwconf.h"
#include "simulate_i2c.h"
#include "spl06_001_drv.h"
    
//#define BOARD_BAROMETER_DEBUG

//#define USE_SIMULATE_I2C1

Simulate_I2C_Bus_Typedef BarometerI2C;

Board_Barameter_Dev_Typedef BarometerDev = {0};

int32_t Barometer_Dev_Init(void)
{
  #ifdef BOARD_BAROMETER_DEBUG
    static uint8_t re06 = 0, re07 = 0;
    static uint8_t re08 = 0, re09 = 0;
  #endif
    
  #ifdef USE_SIMULATE_I2C1
    BarometerI2C.SCL_Port = I2C1_SCL_Port;
    BarometerI2C.SCL_Pin = I2C1_SCL_Pin;
    BarometerI2C.SCL_Pin_No = 6;
    
    BarometerI2C.SDA_Port = I2C1_SDA_Port;
    BarometerI2C.SDA_Pin = I2C1_SDA_Pin;
    BarometerI2C.SDA_Pin_No = 7;
    
    BarometerI2C.Freq = 400000;
    BarometerI2C.Delay = 1;
    
    Simulate_I2C_Init(&BarometerI2C);
  #endif
      
    BarometerDev.SPL06_001_Drv.DevID = SPL06_001_I2C_ADDR_PRIM;
    BarometerDev.SPL06_001_Drv.Mode = SPL06_001_PRS_TMP_CONTINOUS_MODE;
    
    BarometerDev.SPL06_001_Drv.Conf.PRS_ODR = SPL06_001_PRSMR_016_PER_SEC;
    BarometerDev.SPL06_001_Drv.Conf.PRS_OVR = SPL06_001_PRSOR_064_TIMES;
    BarometerDev.SPL06_001_Drv.Conf.TMP_ODR = SPL06_001_TMPMR_008_PER_SEC;
    BarometerDev.SPL06_001_Drv.Conf.TMP_OVR = SPL06_001_TMPOR_008_TIMES;
    BarometerDev.SPL06_001_Drv.Conf.Fifo_EN = SPL06_001_FIFO_DISABLE;
    BarometerDev.SPL06_001_Drv.Conf.Spi3w_En = SPL06_001_SPI3_WIRE_DISABLE;
    
    switch(BarometerDev.SPL06_001_Drv.Conf.PRS_OVR)
    {
        case SPL06_001_PRSOR_001_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 524288;
          break;
        case SPL06_001_PRSOR_002_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 1572864;
          break;
        case SPL06_001_PRSOR_004_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 3670016;
          break;
        case SPL06_001_PRSOR_008_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 7864320;
          break;
        case SPL06_001_PRSOR_016_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 253952;
          break;
        case SPL06_001_PRSOR_032_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 516096;
          break;
        case SPL06_001_PRSOR_064_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 1040384;
          break;
        case SPL06_001_PRSOR_128_TIMES:
          BarometerDev.SPL06_001_Drv.i32kP = 2088960;
          break;
    }
    
    switch(BarometerDev.SPL06_001_Drv.Conf.TMP_OVR)
    {
        case SPL06_001_TMPOR_001_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 524288;
          break;
        case SPL06_001_TMPOR_002_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 1572864;
          break;
        case SPL06_001_TMPOR_004_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 3670016;
          break;
        case SPL06_001_TMPOR_008_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 7864320;
          break;
        case SPL06_001_TMPOR_016_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 253952;
          break;
        case SPL06_001_TMPOR_032_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 516096;
          break;
        case SPL06_001_TMPOR_064_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 1040384;
          break;
        case SPL06_001_TMPOR_128_TIMES:
          BarometerDev.SPL06_001_Drv.i32kT = 2088960;
          break;
    }
    
    BarometerDev.SPL06_001_Drv.DevCtx.chip_select = I2C_Barometer_Chip_Select;
    BarometerDev.SPL06_001_Drv.DevCtx.write_data = I2C_Barometer_Write_Data;
    BarometerDev.SPL06_001_Drv.DevCtx.read_data = I2C_Barometer_Read_Data;
    BarometerDev.SPL06_001_Drv.DevCtx.chip_deselect = I2C_Barometer_Chip_Deselect;  
    BarometerDev.SPL06_001_Drv.DevCtx.handle = (I2C_Type*)I2C1;
    
    SPL06_001_Soft_RST(&BarometerDev.SPL06_001_Drv.DevCtx);
    
    Delay_ms(50);
  
    SPL06_001_Get_ID(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv.ChipID);
    
    SPL06_001_Get_Calib_Param(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv.Param);
    
  #ifdef BOARD_BAROMETER_DEBUG
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_PRS_CFG_REG_ADDR, &re06, 1); 
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_TMP_CFG_REG_ADDR, &re07, 1); 
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_MEAS_CFG_REG_ADDR, &re08, 1);
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_CFG_REG_REG_ADDR, &re09, 1);
  #endif
    SPL06_001_Set_Config(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv.Mode, &BarometerDev.SPL06_001_Drv.Conf);
  #ifdef BOARD_BAROMETER_DEBUG
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_PRS_CFG_REG_ADDR, &re06, 1); 
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_TMP_CFG_REG_ADDR, &re07, 1); 
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_MEAS_CFG_REG_ADDR, &re08, 1);
    SPL06_001_Get_Regs(&BarometerDev.SPL06_001_Drv.DevCtx, SPL06_001_CFG_REG_REG_ADDR, &re09, 1);
  #endif

    return 0;
}

int32_t I2C_Barometer_Chip_Select(void)
{
    return 0;
}

int32_t I2C_Barometer_Chip_Deselect(void)
{
    return 0;
}

int32_t I2C_Barometer_Write_Data(uint8_t dev_id, uint8_t reg_addr, uint8_t* bufp, uint8_t len)
{
    if((len != 0) && (bufp == NULL))
        return -1;
    
    uint8_t buf[64] = {0};
    
    buf[0] = reg_addr;
    
    if(len < 64)
    {
        memcpy(buf+1, bufp, len);
    }
    else
    {
        return -1;
    }
    
  #ifdef USE_SIMULATE_I2C1
    /* Must send reg_addr + bufp together, should not send them independently!!! */
    Simulate_I2C_Master_Transmit(&BarometerI2C, dev_id<<1, buf, len+1);
  #endif
    
  #ifndef USE_SIMULATE_I2C1
    /* Must send reg_addr + bufp together, should not send them independently!!! */
    System_I2C_Master_Transmit(BarometerDev.SPL06_001_Drv.DevCtx.handle, dev_id<<1, buf, len+1);
  #endif
    
    return 0;
}

int32_t I2C_Barometer_Read_Data(uint8_t dev_id, uint8_t reg_addr, uint8_t* bufp, uint8_t len)
{
    if((len != 0) && (bufp == NULL))
        return -1;
    
  #ifdef USE_SIMULATE_I2C1
    Simulate_I2C_Master_Transmit(&BarometerI2C, dev_id<<1, &reg_addr, 1);
    Simulate_I2C_Master_Receive(&BarometerI2C, dev_id<<1, bufp, len);
  #endif

  #ifndef USE_SIMULATE_I2C1    
    System_I2C_Master_Transmit(BarometerDev.SPL06_001_Drv.DevCtx.handle, dev_id<<1, &reg_addr, 1);
    System_I2C_Master_Receive(BarometerDev.SPL06_001_Drv.DevCtx.handle, dev_id<<1, bufp, len);
  #endif
    
    return 0;
}

float Board_Barometer_Calc_Altitude(float barometric_pres)
{
    float Altitude=0;
    
	//Altitude =(44330.0 *(1.0-pow((float)(barometric_pres + 1536) / 101325.0,1.0/5.255)));
    Altitude =(44330.0 *(1.0-pow((float)(barometric_pres - 1536) / 101325.0,1.0/5.255)));
    //Altitude =(44330.0 *(1.0-pow((float)(barometric_pres) / 101325.0,1.0/5.255)));
    
	return Altitude;
}

void Barometer_Process(void* arg)
{
    static uint32_t Systick_Count = 0;
    
    if(Systick_Diff_Get(Systick_Get(), Systick_Count) >= 1000)
    {
        SPL06_001_Get_Raw_Temperature(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv.i32rawTemperature);
        SPL06_001_Get_Raw_Pressure(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv.i32rawPressure);
        
        SPL06_001_Get_Temperature(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv);
        SPL06_001_Get_Pressure(&BarometerDev.SPL06_001_Drv.DevCtx, &BarometerDev.SPL06_001_Drv);
      #ifdef BOARD_BAROMETER_DEBUG
        printf("SPL06 T=%f\nSPL06 P=%f\n", BarometerDev.SPL06_001_Drv.Temperature, BarometerDev.SPL06_001_Drv.Pressure);
      #endif

        BarometerDev.Altitude = Board_Barometer_Calc_Altitude(BarometerDev.SPL06_001_Drv.Pressure);
        
      #ifdef BOARD_BAROMETER_DEBUG
        printf("Altitude=%.2f m\n", BarometerDev.Altitude);
      #endif
        
        Systick_Count = Systick_Get();
    }
}
