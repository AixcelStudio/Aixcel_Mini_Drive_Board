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

#include "spl06_001_drv.h"

int32_t SPL06_001_Get_Regs(SPL06_001_Ctx_Typedef *ctx, uint8_t reg_addr, uint8_t* p_data, uint8_t len)
{
    int32_t ret = 0;
    
    ctx->chip_select();
    
    ret = ctx->read_data(SPL06_001_I2C_ADDR_PRIM, reg_addr, p_data, len);
    
    ctx->chip_deselect();
    
    return ret;
}

int32_t SPL06_001_Set_Regs(SPL06_001_Ctx_Typedef *ctx, uint8_t reg_addr, uint8_t* p_data, uint8_t len)
{
    int32_t ret = 0;
    
    ctx->chip_select();
        
    ret = ctx->write_data(SPL06_001_I2C_ADDR_PRIM, reg_addr, p_data, len);
    
    ctx->chip_deselect();
    
    return ret;
}

int32_t SPL06_001_Soft_RST(SPL06_001_Ctx_Typedef *ctx)
{
    uint8_t reset_cmd = 0x89;
  
    ctx->write_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_RESET_REG_ADDR, &reset_cmd, 1);
    
    return 0;
}

int32_t SPL06_001_Get_ID(SPL06_001_Ctx_Typedef *ctx, SPL06_001_ID_Typedef *id)
{
    if(id == NULL)
        return -1;
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_ID_REG_ADDR, &id->byte, 1);
    
    return 0;
}

int32_t SPL06_001_Get_Calib_Param(SPL06_001_Ctx_Typedef *ctx, SPL06_001_Calib_Param_Typedef *param)
{
    static SPL06_001_COEF_Typedef COEF;
    
    if(param == NULL)
        return -1;
  #if 1
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c0_REG_ADDR, &COEF.c0.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c0_c1_REG_ADDR, &COEF.c0_c1.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c1_REG_ADDR, &COEF.c1.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c00_H_REG_ADDR, &COEF.c00_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c00_L_REG_ADDR, &COEF.c00_L.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c00_c10_REG_ADDR, &COEF.c00_c10.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c10_H_REG_ADDR, &COEF.c10_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c10_L_REG_ADDR, &COEF.c10_L.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c01_H_REG_ADDR, &COEF.c01_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c01_L_REG_ADDR, &COEF.c01_L.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c11_H_REG_ADDR, &COEF.c11_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c11_L_REG_ADDR, &COEF.c11_L.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c20_H_REG_ADDR, &COEF.c20_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c20_L_REG_ADDR, &COEF.c20_L.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c21_H_REG_ADDR, &COEF.c21_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c21_L_REG_ADDR, &COEF.c21_L.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c30_H_REG_ADDR, &COEF.c30_H.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c30_L_REG_ADDR, &COEF.c30_L.byte, 1);
  #endif
    
  #if 0
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_c0_REG_ADDR, &COEF.c0.byte, 18);
  #endif  
    
    param->c0 = ((uint32_t)COEF.c0.reg.c0_11_4 << 4) | COEF.c0_c1.reg.c0_3_0;
    param->c0 = (param->c0 & 0x0800) ? (0xF000 | param->c0) : param->c0;
    
    param->c1 = ((uint32_t)COEF.c0_c1.reg.c1_11_8 << 8) | COEF.c1.reg.c1_7_0;
    param->c1 = (param->c1 & 0x0800) ? (0xF000 | param->c1) : param->c1;
     
    param->c00 = ((int32_t)COEF.c00_H.reg.c00_19_12 << 12) | ((int32_t)COEF.c00_L.reg.c00_11_4 << 4) | COEF.c00_c10.reg.c00_3_0;
    param->c00 = (param->c00 & 0x080000) ? (0xFFF00000 | param->c00) : param->c00;
    
    param->c10 = ((int32_t)COEF.c00_c10.reg.c10_19_16 << 16) | ((int32_t)COEF.c10_H.reg.c10_15_8 << 8) | COEF.c10_L.reg.c10_7_0;
    param->c10 = (param->c10 & 0x080000) ? (0xFFF00000 | param->c10) : param->c10;
    
    param->c01 = ((uint32_t)COEF.c01_H.reg.c01_15_8 << 8) | COEF.c01_L.reg.c01_7_0;
    
    param->c11 = ((uint32_t)COEF.c11_H.reg.c11_15_8 << 8) | COEF.c11_L.reg.c11_7_0;
    
    param->c20 = ((uint32_t)COEF.c20_H.reg.c20_15_8 << 8) | COEF.c20_L.reg.c20_7_0;
    
    param->c21 = ((uint32_t)COEF.c21_H.reg.c21_15_8 << 8) | COEF.c21_L.reg.c21_7_0;
    
    param->c30 = ((uint32_t)COEF.c30_H.reg.c30_15_8 << 8) | COEF.c30_L.reg.c30_7_0;
    
    return 0;
}

int32_t SPL06_001_Get_Config(SPL06_001_Ctx_Typedef *ctx, uint8_t *mode, SPL06_001_Config_Typedef *conf)
{
    SPL06_001_MEAS_CFG_Typedef Meas_Cfg;
    
    SPL06_001_PRS_CFG_Typedef Prs_Cfg;
    
    SPL06_001_TMP_CFG_Typedef Tmp_Cfg;
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_MEAS_CFG_REG_ADDR, &Meas_Cfg.byte, 1);
    
    *mode = Meas_Cfg.reg.MEAS_CRTL;
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PRS_CFG_REG_ADDR, &Prs_Cfg.byte, 1);
    
    conf->PRS_OVR = Prs_Cfg.reg.PM_PRC;
    
    conf->PRS_ODR = Prs_Cfg.reg.PM_RATE;
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_CFG_REG_ADDR, &Tmp_Cfg.byte, 1);
    
    conf->TMP_OVR = Tmp_Cfg.reg.TM_PRC;
    
    conf->TMP_ODR = Tmp_Cfg.reg.TM_RATE;
    
    return 0;
}

int32_t SPL06_001_Set_Config(SPL06_001_Ctx_Typedef *ctx, uint8_t *mode, const SPL06_001_Config_Typedef *conf)
{
    SPL06_001_MEAS_CFG_Typedef Meas_Cfg;
    
    SPL06_001_PRS_CFG_Typedef Prs_Cfg;
    
    SPL06_001_TMP_CFG_Typedef Tmp_Cfg;
    
    SPL06_001_CFG_REG_Typedef Cfg_Reg;
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_MEAS_CFG_REG_ADDR, &Meas_Cfg.byte, 1);
    
    Meas_Cfg.reg.MEAS_CRTL = *mode;
    
    ctx->write_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_MEAS_CFG_REG_ADDR, &Meas_Cfg.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PRS_CFG_REG_ADDR, &Prs_Cfg.byte, 1);
    
    Prs_Cfg.reg.PM_PRC = conf->PRS_OVR;
    
    Prs_Cfg.reg.PM_RATE = conf->PRS_ODR;
    
    ctx->write_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PRS_CFG_REG_ADDR, &Prs_Cfg.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_CFG_REG_ADDR, &Tmp_Cfg.byte, 1);
    
    Tmp_Cfg.reg.TM_PRC = conf->TMP_OVR;
    
    Tmp_Cfg.reg.TM_RATE = conf->TMP_ODR;
    
    Tmp_Cfg.reg.TMP_EXT = 1;
    
    ctx->write_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_CFG_REG_ADDR, &Tmp_Cfg.byte, 1);
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_CFG_REG_REG_ADDR, &Cfg_Reg.byte, 1);
    
    if(conf->PRS_OVR > SPL06_001_PRSOR_008_TIMES)
    {
        Cfg_Reg.reg.PRS_SHIFT_EN = 1;
    }
    else
    {
        Cfg_Reg.reg.PRS_SHIFT_EN = 0;
    }
    
    if(conf->TMP_OVR > SPL06_001_TMPOR_008_TIMES)
    {
        Cfg_Reg.reg.TMP_SHIFT_EN = 1;
    }
    else
    {
        Cfg_Reg.reg.TMP_SHIFT_EN = 0;
    }
    
    ctx->write_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_CFG_REG_REG_ADDR, &Cfg_Reg.byte, 1);
    
    return 0;
}

int32_t SPL06_001_Get_Raw_Temperature(SPL06_001_Ctx_Typedef *ctx, int32_t *raw_temp)
{
    int32_t T;
    uint8_t buf[4] = {0};
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_B2_REG_ADDR, &buf[2], 1);
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_B1_REG_ADDR, &buf[1], 1);
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_B0_REG_ADDR, &buf[0], 1);
    
    *raw_temp = (*(int32_t*)buf & 0x800000) ? (0xFF000000 | *(int32_t*)buf) : (*(int32_t*)buf);
    //ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_TMP_B2_REG_ADDR, buf, 3);
    //T = (int32_t)buf[0]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[2];
    //*raw_temp = (T & 0x800000) ? (0xFF000000 | T) : (T);
    return 0;
}

int32_t SPL06_001_Get_Raw_Pressure(SPL06_001_Ctx_Typedef *ctx, int32_t *raw_pres)
{
    int32_t P;
    uint8_t buf[4] = {0};
    
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PSR_B2_REG_ADDR, &buf[2], 1);
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PSR_B1_REG_ADDR, &buf[1], 1);
    ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PSR_B0_REG_ADDR, &buf[0], 1);
    
    *raw_pres = (*(int32_t*)buf & 0x800000) ? (0xFF000000 | *(int32_t*)buf) : (*(int32_t*)buf);
    //ctx->read_data(SPL06_001_I2C_ADDR_PRIM, SPL06_001_PSR_B2_REG_ADDR, buf, 3);
    //P = (int32_t)buf[0]<<16 | (int32_t)buf[1]<<8 | (int32_t)buf[2];
    //*raw_pres = (P & 0x800000) ? (0xFF000000 | P) : (P);
    return 0;
}

int32_t SPL06_001_Get_Temperature(SPL06_001_Ctx_Typedef *ctx, SPL06_001_Drv_Typedef *spl06_001)
{
    float fTCompensate;
    float fTsc;

    fTsc = spl06_001->i32rawTemperature / (float)spl06_001->i32kT;
    fTCompensate =  spl06_001->Param.c0 * 0.5 + spl06_001->Param.c1 * fTsc;
    spl06_001->Temperature = fTCompensate;
    
    return 0;
}

int32_t SPL06_001_Get_Pressure(SPL06_001_Ctx_Typedef *ctx, SPL06_001_Drv_Typedef *spl06_001)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = spl06_001->i32rawTemperature / (float)spl06_001->i32kT;
    fPsc = spl06_001->i32rawPressure / (float)spl06_001->i32kP;
    qua2 = spl06_001->Param.c10 + fPsc * (spl06_001->Param.c20 + fPsc * spl06_001->Param.c30);
    qua3 = fTsc * fPsc * (spl06_001->Param.c11 + fPsc * spl06_001->Param.c21);

    fPCompensate = spl06_001->Param.c00 + fPsc * qua2 + fTsc * spl06_001->Param.c01 + qua3;
    spl06_001->Pressure = fPCompensate;
    
    return 0;
}