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

#ifndef SPL06_001_DRV_H
#define SPL06_001_DRV_H

#ifdef __cplusplus
  extern "C" {
#endif

#include "common.h"

/*! @name Interface selection macros */
#define SPL06_001_SPI_INTF                             UINT8_C(0)
#define SPL06_001_I2C_INTF                             UINT8_C(1)

/*! @name SPI 3-Wire macros */
#define SPL06_001_SPI3_WIRE_ENABLE                     UINT8_C(1)
#define SPL06_001_SPI3_WIRE_DISABLE                    UINT8_C(0)

/*! @name SPI 3-Wire macros */
#define SPL06_001_FIFO_ENABLE                          UINT8_C(1)
#define SPL06_001_FIFO_DISABLE                         UINT8_C(0)

/*! @name I2C addresses */
#define SPL06_001_I2C_ADDR_PRIM                        UINT8_C(0x76)
#define SPL06_001_I2C_ADDR_SEC                         UINT8_C(0x77)

/*! @name register addresses*/
#define SPL06_001_PSR_B2_REG_ADDR                      UINT8_C(0x00)
#define SPL06_001_PSR_B1_REG_ADDR                      UINT8_C(0x01)
#define SPL06_001_PSR_B0_REG_ADDR                      UINT8_C(0x02)
#define SPL06_001_TMP_B2_REG_ADDR                      UINT8_C(0x03)
#define SPL06_001_TMP_B1_REG_ADDR                      UINT8_C(0x04)
#define SPL06_001_TMP_B0_REG_ADDR                      UINT8_C(0x05)
#define SPL06_001_PRS_CFG_REG_ADDR                     UINT8_C(0x06)
#define SPL06_001_TMP_CFG_REG_ADDR                     UINT8_C(0x07)
#define SPL06_001_MEAS_CFG_REG_ADDR                    UINT8_C(0x08)
#define SPL06_001_CFG_REG_REG_ADDR                     UINT8_C(0x09)
#define SPL06_001_INT_STS_REG_ADDR                     UINT8_C(0x0A)
#define SPL06_001_FIFO_STS_REG_ADDR                    UINT8_C(0x0B)
#define SPL06_001_RESET_REG_ADDR                       UINT8_C(0x0C)
#define SPL06_001_ID_REG_ADDR                          UINT8_C(0x0D)

/*! @name Calibration parameter register addresses*/
#define SPL06_001_c0_REG_ADDR                          UINT8_C(0x10)
#define SPL06_001_c0_c1_REG_ADDR                       UINT8_C(0x11)
#define SPL06_001_c1_REG_ADDR                          UINT8_C(0x12)
#define SPL06_001_c00_H_REG_ADDR                       UINT8_C(0x13)
#define SPL06_001_c00_L_REG_ADDR                       UINT8_C(0x14)
#define SPL06_001_c00_c10_REG_ADDR                     UINT8_C(0x15)
#define SPL06_001_c10_H_REG_ADDR                       UINT8_C(0x16)
#define SPL06_001_c10_L_REG_ADDR                       UINT8_C(0x17)
#define SPL06_001_c01_H_REG_ADDR                       UINT8_C(0x18)
#define SPL06_001_c01_L_REG_ADDR                       UINT8_C(0x19)
#define SPL06_001_c11_H_REG_ADDR                       UINT8_C(0x1A)
#define SPL06_001_c11_L_REG_ADDR                       UINT8_C(0x1B)
#define SPL06_001_c20_H_REG_ADDR                       UINT8_C(0x1C)
#define SPL06_001_c20_L_REG_ADDR                       UINT8_C(0x1D)
#define SPL06_001_c21_H_REG_ADDR                       UINT8_C(0x1E)
#define SPL06_001_c21_L_REG_ADDR                       UINT8_C(0x1F)
#define SPL06_001_c30_H_REG_ADDR                       UINT8_C(0x20)
#define SPL06_001_c30_L_REG_ADDR                       UINT8_C(0x21)

/*! @name Power modes */
#define SPL06_001_STANDBY_MODE                         UINT8_C(0x00)
#define SPL06_001_PRS_COMMAND_MODE                     UINT8_C(0x01)
#define SPL06_001_TMP_COMMAND_MODE                     UINT8_C(0x02)
#define SPL06_001_PRS_CONTINOUS_MODE                   UINT8_C(0x05)
#define SPL06_001_TMP_CONTINOUS_MODE                   UINT8_C(0x06)
#define SPL06_001_PRS_TMP_CONTINOUS_MODE               UINT8_C(0x07)

/*! @name Soft reset command */
#define SPL06_001_SOFT_RESET_CMD                       UINT8_C(0x89)

/*! @name Pressure Measurement Rate options */
#define SPL06_001_PRSMR_001_PER_SEC                    UINT8_C(0x00)
#define SPL06_001_PRSMR_002_PER_SEC                    UINT8_C(0x01)
#define SPL06_001_PRSMR_004_PER_SEC                    UINT8_C(0x02)
#define SPL06_001_PRSMR_008_PER_SEC                    UINT8_C(0x03)
#define SPL06_001_PRSMR_016_PER_SEC                    UINT8_C(0x04)
#define SPL06_001_PRSMR_032_PER_SEC                    UINT8_C(0x05)
#define SPL06_001_PRSMR_064_PER_SEC                    UINT8_C(0x06)
#define SPL06_001_PRSMR_128_PER_SEC                    UINT8_C(0x07)

/*! @name Pressure Oversampling Rate options */
#define SPL06_001_PRSOR_001_TIMES                      UINT8_C(0x00)
#define SPL06_001_PRSOR_002_TIMES                      UINT8_C(0x01)
#define SPL06_001_PRSOR_004_TIMES                      UINT8_C(0x02)
#define SPL06_001_PRSOR_008_TIMES                      UINT8_C(0x03)
#define SPL06_001_PRSOR_016_TIMES                      UINT8_C(0x04)
#define SPL06_001_PRSOR_032_TIMES                      UINT8_C(0x05)
#define SPL06_001_PRSOR_064_TIMES                      UINT8_C(0x06)
#define SPL06_001_PRSOR_128_TIMES                      UINT8_C(0x07)

/*! @name Temperature Measurement Rate options */
#define SPL06_001_TMPMR_001_PER_SEC                    UINT8_C(0x00)
#define SPL06_001_TMPMR_002_PER_SEC                    UINT8_C(0x01)
#define SPL06_001_TMPMR_004_PER_SEC                    UINT8_C(0x02)
#define SPL06_001_TMPMR_008_PER_SEC                    UINT8_C(0x03)
#define SPL06_001_TMPMR_016_PER_SEC                    UINT8_C(0x04)
#define SPL06_001_TMPMR_032_PER_SEC                    UINT8_C(0x05)
#define SPL06_001_TMPMR_064_PER_SEC                    UINT8_C(0x06)
#define SPL06_001_TMPMR_128_PER_SEC                    UINT8_C(0x07)

/*! @name Temperature Oversampling Rate options */
#define SPL06_001_TMPOR_001_TIMES                      UINT8_C(0x00)
#define SPL06_001_TMPOR_002_TIMES                      UINT8_C(0x01)
#define SPL06_001_TMPOR_004_TIMES                      UINT8_C(0x02)
#define SPL06_001_TMPOR_008_TIMES                      UINT8_C(0x03)
#define SPL06_001_TMPOR_016_TIMES                      UINT8_C(0x04)
#define SPL06_001_TMPOR_032_TIMES                      UINT8_C(0x05)
#define SPL06_001_TMPOR_064_TIMES                      UINT8_C(0x06)
#define SPL06_001_TMPOR_128_TIMES                      UINT8_C(0x07)

typedef __packed union __SPL06_001_PSR_B2_Typedef
{
    struct
    {
        uint8_t  PSR23_16;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_PSR_B2_Typedef;

typedef __packed union __SPL06_001_PSR_B1_Typedef
{
    struct
    {
        uint8_t  PSR15_08;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_PSR_B1_Typedef;

typedef __packed union __SPL06_001_PSR_B0_Typedef
{
    struct
    {
        uint8_t  PSR07_00;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_PSR_B0_Typedef;

typedef __packed union __SPL06_001_TMP_B2_Typedef
{
    struct
    {
        uint8_t  TMP23_16;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_TMP_B2_Typedef;

typedef __packed union __SPL06_001_TMP_B1_Typedef
{
    struct
    {
        uint8_t  TMP15_08;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_TMP_B1_Typedef;

typedef __packed union __SPL06_001_TMP_B0_Typedef
{
    struct
    {
        uint8_t  TMP07_00;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_TMP_B0_Typedef;

typedef __packed union __SPL06_001_PRS_CFG_Typedef
{
    struct
    {
        uint8_t  PM_PRC              : 4;
        uint8_t  PM_RATE             : 3;
        uint8_t  RESERVE             : 1;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_PRS_CFG_Typedef;

typedef __packed union __SPL06_001_TMP_CFG_Typedef
{
    struct
    {
        uint8_t  TM_PRC              : 3;
        uint8_t  RESERVE             : 1;
        uint8_t  TM_RATE             : 3;
        uint8_t  TMP_EXT             : 1;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_TMP_CFG_Typedef;

typedef __packed union __SPL06_001_MEAS_CFG_Typedef
{
    struct
    {
        uint8_t  MEAS_CRTL           : 3;
        uint8_t  RESERVE             : 1;
        uint8_t  PRS_RDY             : 1;
        uint8_t  TMP_RDY             : 1;
        uint8_t  SENSOR_RDY          : 1;
        uint8_t  COEF_RDY            : 1;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_MEAS_CFG_Typedef;

typedef __packed union __SPL06_001_CFG_REG_Typedef
{
    struct
    {
        uint8_t  RESERVE             : 1;
        uint8_t  FIFO_EN             : 1;
        uint8_t  PRS_SHIFT_EN        : 1;
        uint8_t  TMP_SHIFT_EN        : 1;
        uint8_t  INT_SEL             : 3;
        uint8_t  INT_HL              : 1;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_CFG_REG_Typedef;

typedef __packed union __SPL06_001_INT_STS_Typedef
{
    struct
    {
        uint8_t  INT_PRS             : 1;
        uint8_t  INT_TMP             : 1;
        uint8_t  INT_FIFO_FULL       : 1;
        uint8_t  RESERVE             : 5;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_INT_STS_Typedef;

typedef __packed union __SPL06_001_FIFO_STS_Typedef
{
    struct
    {
        uint8_t  FIFO_EMPTY          : 1;
        uint8_t  FIFO_FULL           : 1;
        uint8_t  RESERVE             : 6;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_FIFO_STS_Typedef;

typedef __packed union __SPL06_001_RESET_Typedef
{
    struct
    {
        uint8_t  SOFT_RST            : 4;
        uint8_t  RESERVE             : 3;
        uint8_t  FIFO_FLUSH          : 1;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_RESET_Typedef;

typedef __packed union __SPL06_001_ID_Typedef
{
    struct
    {
        uint8_t  REV_ID              : 4;
        uint8_t  PROD_ID             : 4;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_ID_Typedef;

typedef __packed union __SPL06_001_c0_Typedef
{
    struct
    {
        uint8_t  c0_11_4;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c0_Typedef;

typedef __packed union __SPL06_001_c0_c1_Typedef
{
    struct
    {
        uint8_t  c1_11_8             : 4;
        uint8_t  c0_3_0              : 4;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c0_c1_Typedef;

typedef __packed union __SPL06_001_c1_Typedef
{
    struct
    {
        uint8_t  c1_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c1_Typedef;

typedef __packed union __SPL06_001_c00_H_Typedef
{
    struct
    {
        uint8_t  c00_19_12;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c00_H_Typedef;

typedef __packed union __SPL06_001_c00_L_Typedef
{
    struct
    {
        uint8_t  c00_11_4;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c00_L_Typedef;

typedef __packed union __SPL06_001_c00_c10_Typedef
{
    struct
    {
        uint8_t  c10_19_16           : 4;
        uint8_t  c00_3_0             : 4;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c00_c10_Typedef;

typedef __packed union __SPL06_001_c10_H_Typedef
{
    struct
    {
        uint8_t  c10_15_8;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c10_H_Typedef;

typedef __packed union __SPL06_001_c10_L_Typedef
{
    struct
    {
        uint8_t  c10_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c10_L_Typedef;

typedef __packed union __SPL06_001_c01_H_Typedef
{
    struct
    {
        uint8_t  c01_15_8;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c01_H_Typedef;

typedef __packed union __SPL06_001_c01_L_Typedef
{
    struct
    {
        uint8_t  c01_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c01_L_Typedef;

typedef __packed union __SPL06_001_c11_H_Typedef
{
    struct
    {
        uint8_t  c11_15_8;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c11_H_Typedef;

typedef __packed union __SPL06_001_c11_L_Typedef
{
    struct
    {
        uint8_t  c11_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c11_L_Typedef;

typedef __packed union __SPL06_001_c20_H_Typedef
{
    struct
    {
        uint8_t  c20_15_8;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c20_H_Typedef;

typedef __packed union __SPL06_001_c20_L_Typedef
{
    struct
    {
        uint8_t  c20_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c20_L_Typedef;

typedef __packed union __SPL06_001_c21_H_Typedef
{
    struct
    {
        uint8_t  c21_15_8;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c21_H_Typedef;

typedef __packed union __SPL06_001_c21_L_Typedef
{
    struct
    {
        uint8_t  c21_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c21_L_Typedef;

typedef __packed union __SPL06_001_c30_H_Typedef
{
    struct
    {
        uint8_t  c30_15_8;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c30_H_Typedef;

typedef __packed union __SPL06_001_c30_L_Typedef
{
    struct
    {
        uint8_t  c30_7_0;
    }reg;
  
    uint8_t byte;
    
} SPL06_001_c30_L_Typedef;

typedef __packed struct __SPL06_001_Reg_Typedef
{
    SPL06_001_PSR_B2_Typedef      PSR_B2;

    SPL06_001_PSR_B1_Typedef      PSR_B1;

    SPL06_001_PSR_B0_Typedef      PSR_B0;

    SPL06_001_TMP_B2_Typedef      TMP_B2;

    SPL06_001_TMP_B1_Typedef      TMP_B1;

    SPL06_001_TMP_B0_Typedef      TMP_B0;

    SPL06_001_PRS_CFG_Typedef     PRS_CFG;

    SPL06_001_TMP_CFG_Typedef     TMP_CFG;

    SPL06_001_MEAS_CFG_Typedef    MEAS_CFG;

    SPL06_001_CFG_REG_Typedef     CFG_REG;

    SPL06_001_INT_STS_Typedef     INT_STS;

    SPL06_001_FIFO_STS_Typedef    FIFO_STS;

    SPL06_001_RESET_Typedef       RESET;

    SPL06_001_ID_Typedef          ID;
    
} SPL06_001_Reg_Typedef;

typedef __packed struct __SPL06_001_COEF_Typedef
{
    SPL06_001_c0_Typedef          c0;

    SPL06_001_c0_c1_Typedef       c0_c1;

    SPL06_001_c1_Typedef          c1;

    SPL06_001_c00_H_Typedef       c00_H;

    SPL06_001_c00_L_Typedef       c00_L;

    SPL06_001_c00_c10_Typedef     c00_c10;

    SPL06_001_c10_H_Typedef       c10_H;

    SPL06_001_c10_L_Typedef       c10_L;

    SPL06_001_c01_H_Typedef       c01_H;

    SPL06_001_c01_L_Typedef       c01_L;

    SPL06_001_c11_H_Typedef       c11_H;

    SPL06_001_c11_L_Typedef       c11_L;

    SPL06_001_c20_H_Typedef       c20_H;

    SPL06_001_c20_L_Typedef       c20_L;

    SPL06_001_c21_H_Typedef       c21_H;

    SPL06_001_c21_L_Typedef       c21_L;

    SPL06_001_c30_H_Typedef       c30_H;

    SPL06_001_c30_L_Typedef       c30_L;
    
} SPL06_001_COEF_Typedef;

/*! @name Calibration parameters' structure */
typedef __packed struct __SPL06_001_Calib_Param_Typedef
{
    int16_t                       c0;
    
    int16_t                       c1;
    
    int32_t                       c00;
    
    int32_t                       c10;
    
    int16_t                       c01;
    
    int16_t                       c11;
    
    int16_t                       c20;
    
    int16_t                       c21;
    
    int16_t                       c30;
    
} SPL06_001_Calib_Param_Typedef;

/*! @name Sensor configuration structure */
typedef __packed struct __SPL06_001_Config_Typedef
{
    uint8_t                       PRS_ODR;
    uint8_t                       PRS_OVR;
    
    uint8_t                       TMP_ODR;
    uint8_t                       TMP_OVR;
    
    uint8_t                       Fifo_EN;
    uint8_t                       Spi3w_En;
    
} SPL06_001_Config_Typedef;

typedef int32_t (*spl06_001_cs_ptr) (void);
typedef int32_t (*spl06_001_wr_ptr) (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
typedef int32_t (*spl06_001_rd_ptr) (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
typedef int32_t (*spl06_001_ds_ptr) (void);

typedef struct __SPL06_001_Ctx_Typedef 
{
    /** Component mandatory fields **/
    spl06_001_cs_ptr              chip_select;
    spl06_001_wr_ptr              write_data;
    spl06_001_rd_ptr              read_data;
    spl06_001_ds_ptr              chip_deselect;
    
    /** Customizable optional pointer **/
    I2C_Type*                     handle;
    
} SPL06_001_Ctx_Typedef;

typedef struct __SPL06_001_Drv_Typedef
{
    uint8_t                       DevID;
    
    uint8_t                       Mode;
    
    SPL06_001_ID_Typedef          ChipID;
    
    SPL06_001_Calib_Param_Typedef Param;
    
    SPL06_001_Config_Typedef      Conf;
    
    int32_t                       i32rawPressure;
    
    int32_t                       i32rawTemperature;
    
    int32_t                       i32kP;    
    
    int32_t                       i32kT;
    
    float                         Pressure;
    
    float                         Temperature;
    
    SPL06_001_Ctx_Typedef         DevCtx;
    
} SPL06_001_Drv_Typedef;

int32_t SPL06_001_Get_Regs(SPL06_001_Ctx_Typedef *ctx, uint8_t reg_addr, uint8_t* p_data, uint8_t len);

int32_t SPL06_001_Set_Regs(SPL06_001_Ctx_Typedef *ctx, uint8_t reg_addr, uint8_t* p_data, uint8_t len);

int32_t SPL06_001_Soft_RST(SPL06_001_Ctx_Typedef *ctx);

int32_t SPL06_001_Get_ID(SPL06_001_Ctx_Typedef *ctx, SPL06_001_ID_Typedef *id);

int32_t SPL06_001_Get_Calib_Param(SPL06_001_Ctx_Typedef *ctx, SPL06_001_Calib_Param_Typedef *param);

int32_t SPL06_001_Get_Config(SPL06_001_Ctx_Typedef *ctx, uint8_t *mode, SPL06_001_Config_Typedef *conf);

int32_t SPL06_001_Set_Config(SPL06_001_Ctx_Typedef *ctx, uint8_t *mode, const SPL06_001_Config_Typedef *conf);

int32_t SPL06_001_Get_Raw_Temperature(SPL06_001_Ctx_Typedef *ctx, int32_t *raw_temp);

int32_t SPL06_001_Get_Raw_Pressure(SPL06_001_Ctx_Typedef *ctx, int32_t *raw_pres);

int32_t SPL06_001_Get_Temperature(SPL06_001_Ctx_Typedef *ctx, SPL06_001_Drv_Typedef *spl06_001);

int32_t SPL06_001_Get_Pressure(SPL06_001_Ctx_Typedef *ctx, SPL06_001_Drv_Typedef *spl06_001);

#ifdef __cplusplus
}
#endif

#endif