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

#include "qmi8658c_drv.h"

int32_t qmi8658c_read_reg(qmidev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

int32_t qmi8658c_write_reg(qmidev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

/*
 * Get WHO_AM_I
 */
int32_t qmi8658c_who_am_i_get(qmidev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_WHO_AM_I, (uint8_t*)val, 1);
  
  return ret;
}

/*
 * GET REVISION_ID
 */
int32_t qmi8658c_revision_id_get(qmidev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_REVISION_ID, (uint8_t*)val, 1);
  
  return ret;
}

/*
 * Serial Interface Mode
 */
int32_t qmi8658c_sim_set(qmidev_ctx_t *ctx, qmi8658c_sim_t val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
     ctrl1.SIM = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

int32_t qmi8658c_sim_get(qmidev_ctx_t *ctx, qmi8658c_sim_t *val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  switch(ctrl1.SIM)
  {
    case QMI8658C_SIM_4_Wire_SPI:
      *val = QMI8658C_SIM_4_Wire_SPI;
      break;
    case QMI8658C_SIM_3_Wire_SPI:
      *val = QMI8658C_SIM_3_Wire_SPI;
      break;
    default:
      *val = QMI8658C_SIM_4_Wire_SPI;
      break;
  }
  
  return ret;
}

/*
 * Serial Interface Address Auto Increment or Not
 */
int32_t qmi8658c_sip_ai_set(qmidev_ctx_t *ctx, qmi8658c_sip_ai_t val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
     ctrl1.SPI_AI = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

int32_t qmi8658c_sip_ai_get(qmidev_ctx_t *ctx, qmi8658c_sip_ai_t *val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  switch(ctrl1.SPI_AI)
  {
    case QMI8658C_SPI_AI_ON:
      *val = QMI8658C_SPI_AI_ON;
      break;
    case QMI8658C_SPI_AI_OFF:
      *val = QMI8658C_SPI_AI_OFF;
      break;
    default:
      *val = QMI8658C_SPI_AI_ON;
      break;
  }
  
  return ret;
}

/*
 * SPI Read Data Big Endian or Little Endian
 */
int32_t qmi8658c_spi_be_set(qmidev_ctx_t *ctx, qmi8658c_spi_be_t val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
     ctrl1.SPI_BE = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

int32_t qmi8658c_spi_be_get(qmidev_ctx_t *ctx, qmi8658c_spi_be_t *val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  switch(ctrl1.SPI_BE)
  {
    case QMI8658C_SPI_LE:
      *val = QMI8658C_SPI_LE;
      break;
    case QMI8658C_SPI_BE:
      *val = QMI8658C_SPI_BE;
      break;
    default:
      *val = QMI8658C_SPI_LE;
      break;
  }
  
  return ret;
}

/*
 * Sensor Enable or Disable
 */
int32_t qmi8658c_sensor_sta_set(qmidev_ctx_t *ctx, qmi8658c_sensor_sta_t val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
     ctrl1.SensorDisable = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

int32_t qmi8658c_sensor_sta_get(qmidev_ctx_t *ctx, qmi8658c_sensor_sta_t *val)
{
  qmi8658c_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL1, (uint8_t*)&ctrl1, 1);
  switch(ctrl1.SensorDisable)
  {
    case QMI8658C_Sen_Enable:
      *val = QMI8658C_Sen_Enable;
      break;
    case QMI8658C_Sen_Disable:
      *val = QMI8658C_Sen_Disable;
      break;
    default:
      *val = QMI8658C_Sen_Enable;
      break;
  }
  
  return ret;
}

float_t qmi8658c_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.061f);
}

float_t qmi8658c_from_fs4g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.122f);
}

float_t qmi8658c_from_fs8g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.244f);
}

float_t qmi8658c_from_fs16g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.488f);
}

float_t qmi8658c_from_fs16dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 0.488f);
}

float_t qmi8658c_from_fs32dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 0.976f);
}

float_t qmi8658c_from_fs64dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 1.953f);
}

float_t qmi8658c_from_fs128dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 3.906f);
}

float_t qmi8658c_from_fs256dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 7.812f);
}

float_t qmi8658c_from_fs512dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 15.625f);
}

float_t qmi8658c_from_fs1024dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 31.25f);
}

float_t qmi8658c_from_fs2048dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 62.5f);
}

float_t qmi8658c_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

/*
 * Accelerometer Self Test Enable or Disable
 */
int32_t qmi8658c_xl_self_test_set(qmidev_ctx_t *ctx, qmi8658c_xl_self_test_t val)
{
  qmi8658c_ctrl2_t ctrl2;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
     ctrl2.aST = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

int32_t qmi8658c_xl_self_test_get(qmidev_ctx_t *ctx, qmi8658c_xl_self_test_t *val)
{
  qmi8658c_ctrl2_t ctrl2;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  switch(ctrl2.aST)
  {
    case QMI8658C_XL_Self_Test_Disable:
      *val = QMI8658C_XL_Self_Test_Disable;
      break;
    case QMI8658C_XL_Self_Test_Enable:
      *val = QMI8658C_XL_Self_Test_Enable;
      break;
    default:
      *val = QMI8658C_XL_Self_Test_Disable;
      break;
  }
  
  return ret;
}

/*
 * Accelerometer Full Scale
 */
int32_t qmi8658c_xl_full_scale_set(qmidev_ctx_t *ctx, qmi8658c_fs_xl_t val)
{
  qmi8658c_ctrl2_t ctrl2;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
     ctrl2.aFS = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

int32_t qmi8658c_xl_full_scale_get(qmidev_ctx_t *ctx, qmi8658c_fs_xl_t *val)
{
  qmi8658c_ctrl2_t ctrl2;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  switch(ctrl2.aFS)
  {
    case QMI8658C_2g:
      *val = QMI8658C_2g;
      break;
    case QMI8658C_4g:
      *val = QMI8658C_4g;
      break;
    case QMI8658C_8g:
      *val = QMI8658C_8g;
      break;
    case QMI8658C_16g:
      *val = QMI8658C_16g;
      break;
    default:
      *val = QMI8658C_XL_FS_ND;
      break;
  }
  
  return ret;
}

/*
 * Accelerometer Output Data Rate
 */
int32_t qmi8658c_xl_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_xl_t val)
{
  qmi8658c_ctrl2_t ctrl2;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
     ctrl2.aODR = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

int32_t qmi8658c_xl_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_xl_t *val)
{
  qmi8658c_ctrl2_t ctrl2;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL2, (uint8_t*)&ctrl2, 1);
  switch(ctrl2.aODR)
  {
    case QMI8658C_XL_ODR_8kHz:
      *val = QMI8658C_XL_ODR_8kHz;
      break;
    case QMI8658C_XL_ODR_4kHz:
      *val = QMI8658C_XL_ODR_4kHz;
      break;
    case QMI8658C_XL_ODR_2kHz:
      *val = QMI8658C_XL_ODR_2kHz;
      break;
    case QMI8658C_XL_ODR_1kHz:
      *val = QMI8658C_XL_ODR_1kHz;
      break;
    case QMI8658C_XL_ODR_500Hz:
      *val = QMI8658C_XL_ODR_500Hz;
      break;
    case QMI8658C_XL_ODR_250Hz:
      *val = QMI8658C_XL_ODR_250Hz;
      break;
    case QMI8658C_XL_ODR_125Hz:
      *val = QMI8658C_XL_ODR_125Hz;
      break;
    case QMI8658C_XL_ODR_62Hz5:
      *val = QMI8658C_XL_ODR_62Hz5;
      break;
      case QMI8658C_XL_ODR_31Hz25:
      *val = QMI8658C_XL_ODR_31Hz25;
      break;
    case QMI8658C_XL_ODR_128Hz:
      *val = QMI8658C_XL_ODR_128Hz;
      break;
    case QMI8658C_XL_ODR_21Hz:
      *val = QMI8658C_XL_ODR_21Hz;
      break;
    case QMI8658C_XL_ODR_11Hz:
      *val = QMI8658C_XL_ODR_11Hz;
      break;
    case QMI8658C_XL_ODR_3Hz:
      *val = QMI8658C_XL_ODR_3Hz;
      break;
    default:
      *val = QMI8658C_XL_ODR_ND;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope Self Test Enable or Disable
 */
int32_t qmi8658c_gy_self_test_set(qmidev_ctx_t *ctx, qmi8658c_gy_self_test_t val)
{
  qmi8658c_ctrl3_t ctrl3;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
     ctrl3.gST = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_self_test_get(qmidev_ctx_t *ctx, qmi8658c_gy_self_test_t *val)
{
  qmi8658c_ctrl3_t ctrl3;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  switch(ctrl3.gST)
  {
    case QMI8658C_GY_Self_Test_Disable:
      *val = QMI8658C_GY_Self_Test_Disable;
      break;
    case QMI8658C_GY_Self_Test_Enable:
      *val = QMI8658C_GY_Self_Test_Enable;
      break;
    default:
      *val = QMI8658C_GY_Self_Test_Disable;
      break;
  }
  
  return ret;
}


/*
 * Gyroscope Full Scale
 */
int32_t qmi8658c_gy_full_scale_set(qmidev_ctx_t *ctx, qmi8658c_fs_gy_t val)
{
  qmi8658c_ctrl3_t ctrl3;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
     ctrl3.gFS = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_full_scale_get(qmidev_ctx_t *ctx, qmi8658c_fs_gy_t *val)
{
  qmi8658c_ctrl3_t ctrl3;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  switch(ctrl3.gFS)
  {
    case QMI8658C_16dps:
      *val = QMI8658C_16dps;
      break;
    case QMI8658C_32dps:
      *val = QMI8658C_32dps;
      break;
    case QMI8658C_64dps:
      *val = QMI8658C_64dps;
      break;
    case QMI8658C_128dps:
      *val = QMI8658C_128dps;
      break;
    case QMI8658C_256dps:
      *val = QMI8658C_256dps;
      break;
    case QMI8658C_512dps:
      *val = QMI8658C_512dps;
      break;
    case QMI8658C_1024dps:
      *val = QMI8658C_1024dps;
      break;
    case QMI8658C_2048dps:
      *val = QMI8658C_2048dps;
      break;
    default:
      *val = QMI8658C_GY_FS_ND;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope Output Data Rate
 */
int32_t qmi8658c_gy_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_gy_t val)
{
  qmi8658c_ctrl3_t ctrl3;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
     ctrl3.gODR = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_gy_t *val)
{
  qmi8658c_ctrl3_t ctrl3;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL3, (uint8_t*)&ctrl3, 1);
  switch(ctrl3.gODR)
  {
    case QMI8658C_GY_ODR_8kHz:
      *val = QMI8658C_GY_ODR_8kHz;
      break;
    case QMI8658C_GY_ODR_4kHz:
      *val = QMI8658C_GY_ODR_4kHz;
      break;
    case QMI8658C_GY_ODR_2kHz:
      *val = QMI8658C_GY_ODR_2kHz;
      break;
    case QMI8658C_GY_ODR_1kHz:
      *val = QMI8658C_GY_ODR_1kHz;
      break;
    case QMI8658C_GY_ODR_500Hz:
      *val = QMI8658C_GY_ODR_500Hz;
      break;
    case QMI8658C_GY_ODR_250Hz:
      *val = QMI8658C_GY_ODR_250Hz;
      break;
    case QMI8658C_GY_ODR_125Hz:
      *val = QMI8658C_GY_ODR_125Hz;
      break;
    case QMI8658C_GY_ODR_62Hz5:
      *val = QMI8658C_GY_ODR_62Hz5;
      break;
    case QMI8658C_GY_ODR_31Hz25:
      *val = QMI8658C_GY_ODR_31Hz25;
      break;
    default:
      *val = QMI8658C_GY_ODR_ND;
      break;
  }
  
  return ret;
}

/*
 * Magnetometer Output Data Rate
 */
int32_t qmi8658c_mg_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_mg_t val)
{
  qmi8658c_ctrl4_t ctrl4;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
     ctrl4.mODR = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  return ret;
}

int32_t qmi8658c_mg_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_mg_t *val)
{
  qmi8658c_ctrl4_t ctrl4;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL4, (uint8_t*)&ctrl4, 1);
  switch(ctrl4.mODR)
  {
    case QMI8658C_MG_ODR_1kHz:
      *val = QMI8658C_MG_ODR_1kHz;
      break;
    case QMI8658C_MG_ODR_500Hz:
      *val = QMI8658C_MG_ODR_500Hz;
      break;
    case QMI8658C_MG_ODR_250Hz:
      *val = QMI8658C_MG_ODR_250Hz;
      break;
    case QMI8658C_MG_ODR_125Hz:
      *val = QMI8658C_MG_ODR_125Hz;
      break;
    case QMI8658C_MG_ODR_62Hz5:
      *val = QMI8658C_MG_ODR_62Hz5;
      break;
    case QMI8658C_MG_ODR_31Hz25:
      *val = QMI8658C_MG_ODR_31Hz25;
      break;
    default:
      *val = QMI8658C_MG_ODR_ND;
      break;
  }
  
  return ret;
}

/*
 * Accelerometer Low-Pass Filter  Enable or Disable
 */
int32_t qmi8658c_xl_alpf_en_set(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_en_t val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
     ctrl5.aLPF_EN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

int32_t qmi8658c_xl_alpf_en_get(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_en_t *val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  switch(ctrl5.aLPF_EN)
  {
    case QMI8658C_XL_aLPF_Disable:
      *val = QMI8658C_XL_aLPF_Disable;
      break;
    case QMI8658C_XL_aLPF_Enable:
      *val = QMI8658C_XL_aLPF_Enable;
      break;
    default:
      *val = QMI8658C_XL_aLPF_Disable;
      break;
  }
  
  return ret;
}

/*
 * Accelerometer Low-Pass Filter  Mode
 */
int32_t qmi8658c_xl_alpf_mode_set(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_mode_t val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
     ctrl5.aLPF_MODE = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

int32_t qmi8658c_xl_alpf_mode_get(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_mode_t *val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  switch(ctrl5.aLPF_MODE)
  {
    case QMI8658C_XL_aLPF_Mode0:
      *val = QMI8658C_XL_aLPF_Mode0;
      break;
    case QMI8658C_XL_aLPF_Mode1:
      *val = QMI8658C_XL_aLPF_Mode1;
      break;
    case QMI8658C_XL_aLPF_Mode2:
      *val = QMI8658C_XL_aLPF_Mode2;
      break;
    case QMI8658C_XL_aLPF_Mode3:
      *val = QMI8658C_XL_aLPF_Mode3;
      break;
    default:
      *val = QMI8658C_XL_aLPF_Mode0;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope Low-Pass Filter  Enable or Disable
 */
int32_t qmi8658c_gy_alpf_en_set(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_en_t val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
     ctrl5.gLPF_EN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_alpf_en_get(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_en_t *val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  switch(ctrl5.gLPF_EN)
  {
    case QMI8658C_GY_aLPF_Disable:
      *val = QMI8658C_GY_aLPF_Disable;
      break;
    case QMI8658C_GY_aLPF_Enable:
      *val = QMI8658C_GY_aLPF_Enable;
      break;
    default:
      *val = QMI8658C_GY_aLPF_Disable;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope Low-Pass Filter  Mode
 */
int32_t qmi8658c_gy_alpf_mode_set(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_mode_t val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
     ctrl5.gLPF_MODE = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_alpf_mode_get(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_mode_t *val)
{
  qmi8658c_ctrl5_t ctrl5;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL5, (uint8_t*)&ctrl5, 1);
  switch(ctrl5.gLPF_MODE)
  {
    case QMI8658C_GY_aLPF_Mode0:
      *val = QMI8658C_GY_aLPF_Mode0;
      break;
    case QMI8658C_GY_aLPF_Mode1:
      *val = QMI8658C_GY_aLPF_Mode1;
      break;
    case QMI8658C_GY_aLPF_Mode2:
      *val = QMI8658C_GY_aLPF_Mode2;
      break;
    case QMI8658C_GY_aLPF_Mode3:
      *val = QMI8658C_GY_aLPF_Mode3;
      break;
    default:
      *val = QMI8658C_GY_aLPF_Mode0;
      break;
  }
  
  return ret;
}

/*
 * Attitude Engine Output Data Rate
 */
int32_t qmi8658c_ae_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_ae_t val)
{
  qmi8658c_ctrl6_t ctrl6;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL6, (uint8_t*)&ctrl6, 1);
  if(ret == 0){
     ctrl6.sODR = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL6, (uint8_t*)&ctrl6, 1);
  }
  return ret;
}

int32_t qmi8658c_ae_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_ae_t *val)
{
  qmi8658c_ctrl6_t ctrl6;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL6, (uint8_t*)&ctrl6, 1);
  switch(ctrl6.sODR)
  {
    case QMI8658C_AE_ODR_1Hz:
      *val = QMI8658C_AE_ODR_1Hz;
      break;
    case QMI8658C_AE_ODR_2Hz:
      *val = QMI8658C_AE_ODR_2Hz;
      break;
    case QMI8658C_AE_ODR_4Hz:
      *val = QMI8658C_AE_ODR_4Hz;
      break;
    case QMI8658C_AE_ODR_8Hz:
      *val = QMI8658C_AE_ODR_8Hz;
      break;
    case QMI8658C_AE_ODR_16Hz:
      *val = QMI8658C_AE_ODR_16Hz;
      break;
    case QMI8658C_AE_ODR_32Hz:
      *val = QMI8658C_AE_ODR_32Hz;
      break;
    case QMI8658C_AE_ODR_64Hz:
      *val = QMI8658C_AE_ODR_64Hz;
      break;
    default:
      *val = QMI8658C_AE_ODR_ND;
      break;
  }
  
  return ret;
}

/*
 * Motion on Demand Enable or Disable
 */
int32_t qmi8658c_amod_en_set(qmidev_ctx_t *ctx, qmi8658c_amod_en_t val)
{
  qmi8658c_ctrl6_t ctrl6;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL6, (uint8_t*)&ctrl6, 1);
  if(ret == 0){
     ctrl6.sMoD = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL6, (uint8_t*)&ctrl6, 1);
  }
  return ret;
}

int32_t qmi8658c_amod_en_get(qmidev_ctx_t *ctx, qmi8658c_amod_en_t *val)
{
  qmi8658c_ctrl6_t ctrl6;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL6, (uint8_t*)&ctrl6, 1);
  switch(ctrl6.sMoD)
  {
    case QMI8658C_sMoD_Disable:
      *val = QMI8658C_sMoD_Disable;
      break;
    case QMI8658C_sMoD_Enable:
      *val = QMI8658C_sMoD_Enable;
      break;
    default:
      *val = QMI8658C_sMoD_Disable;
      break;
  }
  
  return ret;
}

/*
 * Accelerometer Enable or Disable
 */
int32_t qmi8658c_xl_en_set(qmidev_ctx_t *ctx, qmi8658c_xl_en_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.aEN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  
  return ret;
}

int32_t qmi8658c_xl_en_get(qmidev_ctx_t *ctx, qmi8658c_xl_en_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.aEN)
  {
    case QMI8658C_XL_Disable:
      *val = QMI8658C_XL_Disable;
      break;
    case QMI8658C_XL_Enable:
      *val = QMI8658C_XL_Enable;
      break;
    default:
      *val = QMI8658C_XL_Disable;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope Enable or Disable
 */
int32_t qmi8658c_gy_en_set(qmidev_ctx_t *ctx, qmi8658c_gy_en_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.gEN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_en_get(qmidev_ctx_t *ctx, qmi8658c_gy_en_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.gEN)
  {
    case QMI8658C_GY_Disable:
      *val = QMI8658C_GY_Disable;
      break;
    case QMI8658C_GY_Enable:
      *val = QMI8658C_GY_Enable;
      break;
    default:
      *val = QMI8658C_GY_Disable;
      break;
  }
  
  return ret;
}

/*
 * Magnetometer Enable or Disable
 */
int32_t qmi8658c_mg_en_set(qmidev_ctx_t *ctx, qmi8658c_mg_en_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.mEN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  return ret;
}

int32_t qmi8658c_mg_en_get(qmidev_ctx_t *ctx, qmi8658c_mg_en_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.mEN)
  {
    case QMI8658C_MG_Disable:
      *val = QMI8658C_MG_Disable;
      break;
    case QMI8658C_MG_Enable:
      *val = QMI8658C_MG_Enable;
      break;
    default:
      *val = QMI8658C_MG_Disable;
      break;
  }
  
  return ret;
}

/*
 * AttitudeEngine Orientation And Velocity Increment Computation Enable or Disable
 */
int32_t qmi8658c_aes_en_set(qmidev_ctx_t *ctx, qmi8658c_aes_en_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.sEN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  return ret;
}

int32_t qmi8658c_aes_en_get(qmidev_ctx_t *ctx, qmi8658c_aes_en_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.sEN)
  {
    case QMI8658C_AEs_Disable:
      *val = QMI8658C_AEs_Disable;
      break;
    case QMI8658C_AEs_Enable:
      *val = QMI8658C_AEs_Enable;
      break;
    default:
      *val = QMI8658C_AEs_Disable;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope in Full Mode or Snooze Mode
 */
int32_t qmi8658c_gy_mode_set(qmidev_ctx_t *ctx, qmi8658c_gy_mode_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.gSN = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  return ret;
}

int32_t qmi8658c_gy_mode_get(qmidev_ctx_t *ctx, qmi8658c_gy_mode_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.gSN)
  {
    case QMI8658C_GY_Full_Mode:
      *val = QMI8658C_GY_Full_Mode;
      break;
    case QMI8658C_GY_Snooze_Mode:
      *val = QMI8658C_GY_Snooze_Mode;
      break;
    default:
      *val = QMI8658C_GY_Full_Mode;
      break;
  }
  
  return ret;
}

/*
 * Clock Based On ODR or High Speed Internal Clock
 */
int32_t qmi8658c_sys_hs_set(qmidev_ctx_t *ctx, qmi8658c_sys_hs_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.sys_hs = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  return ret;
}

int32_t qmi8658c_sys_hs_get(qmidev_ctx_t *ctx, qmi8658c_sys_hs_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.sys_hs)
  {
    case QMI8658C_Clock_Based_On_ODR:
      *val = QMI8658C_Clock_Based_On_ODR;
      break;
    case QMI8658C_High_Speed_Internal_Clock:
      *val = QMI8658C_High_Speed_Internal_Clock;
      break;
    default:
      *val = QMI8658C_Clock_Based_On_ODR;
      break;
  }
  
  return ret;
}

/*
 * SyncSmpl Mode Enable or Disable
 */
int32_t qmi8658c_syncsmple_set(qmidev_ctx_t *ctx, qmi8658c_syncsmple_en_t val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  if(ret == 0){
     ctrl7.syncSmpl = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  }
  return ret;
}

int32_t qmi8658c_syncsmple_get(qmidev_ctx_t *ctx, qmi8658c_syncsmple_en_t *val)
{
  qmi8658c_ctrl7_t ctrl7;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_CTRL7, (uint8_t*)&ctrl7, 1);
  switch(ctrl7.syncSmpl)
  {
    case QMI8658C_SyncSmple_Disable:
      *val = QMI8658C_SyncSmple_Disable;
      break;
    case QMI8658C_SyncSmple_Enable:
      *val = QMI8658C_SyncSmple_Enable;
      break;
    default:
      *val = QMI8658C_SyncSmple_Disable;
      break;
  }
  
  return ret;
}

/*
 * FIFO Watermark Set
 */
int32_t qmi8658c_fifo_wtm_set(qmidev_ctx_t *ctx, qmi8658c_reg_t val)
{
  int32_t ret;
  
  ret = qmi8658c_write_reg(ctx, QMI8658C_FIFO_WTM_TH, (uint8_t*)&(val.fifo_wtm_th.FIFO_WTM), 1);
  
  return ret;
}

int32_t qmi8658c_fifo_wtm_get(qmidev_ctx_t *ctx, qmi8658c_reg_t *val)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_WTM_TH, (uint8_t*)&(val->fifo_wtm_th.FIFO_WTM), 1);
  
  return ret;
}

/*
 * FIFO Mode Set
 */
int32_t qmi8658c_fifo_mode_set(qmidev_ctx_t *ctx, qmi8658c_fifo_mode_t val)
{
  qmi8658c_fifo_ctrl_t fifo_ctrl;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
     fifo_ctrl.FIFO_MODE = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  }
  return ret;
}

int32_t qmi8658c_fifo_mode_get(qmidev_ctx_t *ctx, qmi8658c_fifo_mode_t *val)
{
  qmi8658c_fifo_ctrl_t fifo_ctrl;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  switch(fifo_ctrl.FIFO_MODE)
  {
    case QMI8658C_BYPASS_Mode:
      *val = QMI8658C_BYPASS_Mode;
      break;
    case QMI8658C_FIFO_Mode:
      *val = QMI8658C_FIFO_Mode;
      break;
    case QMI8658C_STREAM_Mode:
      *val = QMI8658C_STREAM_Mode;
      break;
    case QMI8658C_STREAM_TO_FIFO_Mode:
      *val = QMI8658C_STREAM_TO_FIFO_Mode;
      break;
    default:
      *val = QMI8658C_BYPASS_Mode;
      break;
  }
  
  return ret;
}

/*
 * FIFO Size Set
 */
int32_t qmi8658c_fifo_size_set(qmidev_ctx_t *ctx, qmi8658c_fifo_size_t val)
{
  qmi8658c_fifo_ctrl_t fifo_ctrl;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
     fifo_ctrl.FIFO_SIZE = (uint8_t) val;
     ret = qmi8658c_write_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  }
  return ret;
}

int32_t qmi8658c_fifo_size_get(qmidev_ctx_t *ctx, qmi8658c_fifo_size_t *val)
{
  qmi8658c_fifo_ctrl_t fifo_ctrl;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  switch(fifo_ctrl.FIFO_SIZE)
  {
    case QMI8658C_FIFO_16Samples:
      *val = QMI8658C_FIFO_16Samples;
      break;
    case QMI8658C_FIFO_32Samples:
      *val = QMI8658C_FIFO_32Samples;
      break;
    case QMI8658C_FIFO_64Samples:
      *val = QMI8658C_FIFO_64Samples;
      break;
    case QMI8658C_FIFO_128Samples:
      *val = QMI8658C_FIFO_128Samples;
      break;
    default:
      *val = QMI8658C_FIFO_16Samples;
      break;
  }
  
  return ret;
}

/*
 * FIFO RD Mode Get
 */
int32_t qmi8658c_fifo_rd_mode_get(qmidev_ctx_t *ctx, qmi8658c_reg_t *val)
{
  qmi8658c_fifo_ctrl_t fifo_ctrl;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
     val->fifo_ctrl.FIFO_RD_MODE = fifo_ctrl.FIFO_RD_MODE;
  }
  return ret;
}

/*
 * FIFO Sample Cnt Get
 */
int32_t qmi8658c_fifo_sample_cnt_get(qmidev_ctx_t *ctx, uint16_t *val)
{
  qmi8658c_fifo_smpl_cnt_t fifo_smpl_cnt_lsb;
  qmi8658c_fifo_status_t fifo_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_SMPL_CNT, (uint8_t*)&fifo_smpl_cnt_lsb, 1);
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_STATUS, (uint8_t*)&fifo_status, 1);
  
  *val = (fifo_status.FIFO_SMPL_CNT_MSB & 0x03)<<8 + fifo_smpl_cnt_lsb.FIFO_SMPL_CNT_LSB;
  
  return ret;
}

/*
 * FIFO Empty Sta Get
 */
int32_t qmi8658c_fifo_empty_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_empty_sta_t *val)
{
  qmi8658c_fifo_status_t fifo_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_STATUS, (uint8_t*)&fifo_status, 1);
  switch(fifo_status.FIFO_NOT_EMPTY)
  {
    case QMI8658C_FIFO_IS_EMPTY:
      *val = QMI8658C_FIFO_IS_EMPTY;
      break;
    case QMI8658C_FIFO_NOT_EMPTY:
      *val = QMI8658C_FIFO_NOT_EMPTY;
      break;
    default:
      *val = QMI8658C_FIFO_IS_EMPTY;
      break;
  }
  
  return ret;
}

/*
 * FIFO Overflow Sta Get
 */
int32_t qmi8658c_fifo_overflow_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_overflow_sta_t *val)
{
  qmi8658c_fifo_status_t fifo_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_STATUS, (uint8_t*)&fifo_status, 1);
  switch(fifo_status.FIFO_OVFLOW)
  {
    case QMI8658C_FIFO_NOT_HAPPENED:
      *val = QMI8658C_FIFO_NOT_HAPPENED;
      break;
    case QMI8658C_FIFO_HAS_HAPPENED:
      *val = QMI8658C_FIFO_HAS_HAPPENED;
      break;
    default:
      *val = QMI8658C_FIFO_NOT_HAPPENED;
      break;
  }
  
  return ret;
}

/*
 * FIFO Watermark Hit Flag Get
 */
int32_t qmi8658c_fifo_watermark_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_watermark_sta_t *val)
{
  qmi8658c_fifo_status_t fifo_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_STATUS, (uint8_t*)&fifo_status, 1);
  switch(fifo_status.FIFO_WTM)
  {
    case QMI8658C_FIFO_Watermark_NOT_HIT:
      *val = QMI8658C_FIFO_Watermark_NOT_HIT;
      break;
    case QMI8658C_FIFO_Watermark_HAS_HIT:
      *val = QMI8658C_FIFO_Watermark_HAS_HIT;
      break;
    default:
      *val = QMI8658C_FIFO_Watermark_NOT_HIT;
      break;
  }
  
  return ret;
}

/*
 * FIFO Full Sta Get
 */
int32_t qmi8658c_fifo_full_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_full_sta_t *val)
{
  qmi8658c_fifo_status_t fifo_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_STATUS, (uint8_t*)&fifo_status, 1);
  switch(fifo_status.FIFO_FULL)
  {
    case QMI8658C_FIFO_NOT_FULL:
      *val = QMI8658C_FIFO_NOT_FULL;
      break;
    case QMI8658C_FIFO_IS_FULL:
      *val = QMI8658C_FIFO_IS_FULL;
      break;
    default:
      *val = QMI8658C_FIFO_NOT_FULL;
      break;
  }
  
  return ret;
}

/*
 * FIFO_DATA Get
 */
int32_t qmi8658c_fifo_data_get(qmidev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_FIFO_DATA, (uint8_t*)val, 1);
  
  return ret;
}

/*
 * FIFO Pattern Get
 */
int32_t qmi8658c_fifo_pattern_get(qmidev_ctx_t *ctx, uint16_t *val)
{
  qmi8658c_xl_en_t xl_en_sta;
  qmi8658c_gy_en_t gy_en_sta;
  int32_t ret;

  qmi8658c_xl_en_get(ctx, &xl_en_sta);
  qmi8658c_gy_en_get(ctx, &gy_en_sta);
  
  if((xl_en_sta == QMI8658C_XL_Enable) && (gy_en_sta == QMI8658C_GY_Enable))
  {
      *val = 2;
  }
  else if((xl_en_sta == QMI8658C_XL_Enable) || (gy_en_sta == QMI8658C_GY_Enable))
  {
      *val = 1;
  }
  else
  {
      *val = 0;
  }
  
  return ret;
}

/*
 * I2C Master Active Sta Get
 */
int32_t qmi8658c_i2cm_active_sta_get(qmidev_ctx_t *ctx, qmi8658c_i2cm_active_sta_t *val)
{
  qmi8658c_i2cm_status_t i2cm_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_I2CM_STATUS, (uint8_t*)&i2cm_status, 1);
  switch(i2cm_status.I2CM_active)
  {
    case QMI8658C_FIFO_I2C_Master_Transaction_Not_Done:
      *val = QMI8658C_FIFO_I2C_Master_Transaction_Not_Done;
      break;
    case QMI8658C_FIFO_I2C_Master_Transaction_Is_Done:
      *val = QMI8658C_FIFO_I2C_Master_Transaction_Is_Done;
      break;
    default:
      *val = QMI8658C_FIFO_I2C_Master_Transaction_Not_Done;
      break;
  }
  
  return ret;
}

/*
 * Magnetometer Data Valid Sta Get
 */
int32_t qmi8658c_mg_data_sta_get(qmidev_ctx_t *ctx, qmi8658c_mg_data_sta_t *val)
{
  qmi8658c_i2cm_status_t i2cm_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_I2CM_STATUS, (uint8_t*)&i2cm_status, 1);
  switch(i2cm_status.Data_VLD)
  {
    case QMI8658C_FIFO_MG_Data_Not_Valid:
      *val = QMI8658C_FIFO_MG_Data_Not_Valid;
      break;
    case QMI8658C_FIFO_MG_Data_Is_Valid:
      *val = QMI8658C_FIFO_MG_Data_Is_Valid;
      break;
    default:
      *val = QMI8658C_FIFO_MG_Data_Not_Valid;
      break;
  }
  
  return ret;
}

/*
 * I2C Master Data Movement Sta Get
 */
int32_t qmi8658c_i2cm_data_movement_sta_get(qmidev_ctx_t *ctx, qmi8658c_i2cm_data_movement_sta_t *val)
{
  qmi8658c_i2cm_status_t i2cm_status;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_I2CM_STATUS, (uint8_t*)&i2cm_status, 1);
  switch(i2cm_status.I2CM_done)
  {
    case QMI8658C_FIFO_I2C_Master_Data_Movement_Not_Done:
      *val = QMI8658C_FIFO_I2C_Master_Data_Movement_Not_Done;
      break;
    case QMI8658C_FIFO_I2C_Master_Data_Movement_Is_Done:
      *val = QMI8658C_FIFO_I2C_Master_Data_Movement_Is_Done;
      break;
    default:
      *val = QMI8658C_FIFO_I2C_Master_Data_Movement_Not_Done;
      break;
  }
  
  return ret;
}

/*
 * Sensor Data Available Sta Get
 */
int32_t qmi8658c_sen_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_sen_data_available_sta_t *val)
{
  qmi8658c_statusint_t statusint;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUSINT, (uint8_t*)&statusint, 1);
  switch(statusint.Avail)
  {
    case QMI8658C_Sensor_Data_Not_Available:
      *val = QMI8658C_Sensor_Data_Not_Available;
      break;
    case QMI8658C_Sensor_Data_Is_Available:
      *val = QMI8658C_Sensor_Data_Is_Available;
      break;
    default:
      *val = QMI8658C_Sensor_Data_Not_Available;
      break;
  }
  
  return ret;
}

/*
 * Sensor Data Locked Sta Get
 */
int32_t qmi8658c_sen_data_locked_sta_get(qmidev_ctx_t *ctx, qmi8658c_sen_data_locked_sta_t *val)
{
  qmi8658c_statusint_t statusint;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUSINT, (uint8_t*)&statusint, 1);
  switch(statusint.Locked)
  {
    case QMI8658C_Sensor_Data_Not_Locked:
      *val = QMI8658C_Sensor_Data_Not_Locked;
      break;
    case QMI8658C_Sensor_Data_Is_Locked:
      *val = QMI8658C_Sensor_Data_Is_Locked;
      break;
    default:
      *val = QMI8658C_Sensor_Data_Not_Locked;
      break;
  }
  
  return ret;
}

/*
 * Accelerometer New Data Available Sta Get
 */
int32_t qmi8658c_xl_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_xl_new_data_available_sta_t *val)
{
  qmi8658c_status0_t status0;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUS0, (uint8_t*)&status0, 1);
  switch(status0.aDA)
  {
    case QMI8658C_XL_New_Data_Not_Available:
      *val = QMI8658C_XL_New_Data_Not_Available;
      break;
    case QMI8658C_XL_New_Data_Is_Available:
      *val = QMI8658C_XL_New_Data_Is_Available;
      break;
    default:
      *val = QMI8658C_XL_New_Data_Not_Available;
      break;
  }
  
  return ret;
}

/*
 * Gyroscope New Data Available Sta Get
 */
int32_t qmi8658c_gy_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_gy_new_data_available_sta_t *val)
{
  qmi8658c_status0_t status0;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUS0, (uint8_t*)&status0, 1);
  switch(status0.gDA)
  {
    case QMI8658C_GY_New_Data_Not_Available:
      *val = QMI8658C_GY_New_Data_Not_Available;
      break;
    case QMI8658C_GY_New_Data_Is_Available:
      *val = QMI8658C_GY_New_Data_Is_Available;
      break;
    default:
      *val = QMI8658C_GY_New_Data_Not_Available;
      break;
  }
  
  return ret;
}

/*
 * Magnetometer New Data Available Sta Get
 */
int32_t qmi8658c_mg_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_mg_new_data_available_sta_t *val)
{
  qmi8658c_status0_t status0;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUS0, (uint8_t*)&status0, 1);
  switch(status0.mDA)
  {
    case QMI8658C_MG_New_Data_Not_Available:
      *val = QMI8658C_MG_New_Data_Not_Available;
      break;
    case QMI8658C_MG_New_Data_Is_Available:
      *val = QMI8658C_MG_New_Data_Is_Available;
      break;
    default:
      *val = QMI8658C_MG_New_Data_Not_Available;
      break;
  }
  
  return ret;
}

/*
 * Attitude Engine New Data Available Sta Get
 */
int32_t qmi8658c_ae_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_ae_new_data_available_sta_t *val)
{
  qmi8658c_status0_t status0;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUS0, (uint8_t*)&status0, 1);
  switch(status0.sDA)
  {
    case QMI8658C_AE_New_Data_Not_Available:
      *val = QMI8658C_AE_New_Data_Not_Available;
      break;
    case QMI8658C_AE_New_Data_Is_Available:
      *val = QMI8658C_AE_New_Data_Is_Available;
      break;
    default:
      *val = QMI8658C_AE_New_Data_Not_Available;
      break;
  }
  
  return ret;
}

/*
 * Ctrl9 Command Done Sta Get
 */
int32_t qmi8658c_ctrl9_command_done_sta_get(qmidev_ctx_t *ctx, qmi8658c_ctrl9_command_done_sta_t *val)
{
  qmi8658c_status1_t status1;
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_STATUS1, (uint8_t*)&status1, 1);
  switch(status1.CmdDone)
  {
    case QMI8658C_Ctrl9_Command_Not_Done:
      *val = QMI8658C_Ctrl9_Command_Not_Done;
      break;
    case QMI8658C_Ctrl9_Command_Is_Done:
      *val = QMI8658C_Ctrl9_Command_Is_Done;
      break;
    default:
      *val = QMI8658C_Ctrl9_Command_Not_Done;
      break;
  }
  
  return ret;
}

/*
 * Timestamp Data Get
 */
int32_t qmi8658c_timestamp_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_TIMESTAMP_L, (uint8_t*)buf, 3);
  
  return ret;
}

/*
 * Temperatur Data Get
 */
int32_t qmi8658c_temperature_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_TEMP_L, (uint8_t*)buf, 2);
  
  return ret;
}

/*
 * Accelerometer Data Get
 */
int32_t qmi8658c_accelerometer_raw_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_AX_L, (uint8_t*)buf, 6);
  
  return ret;
}

/*
 * Gyroscope Data Get
 */
int32_t qmi8658c_gyroscope_raw_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_GX_L, (uint8_t*)buf, 6);
  
  return ret;
}

/*
 * Magnetometer Data Get
 */
int32_t qmi8658c_magnetometer_raw_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_MX_L, (uint8_t*)buf, 6);
  
  return ret;
}

/*
 * Quaternion Data Get
 */
int32_t qmi8658c_quaternion_Increment_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_dQW_L, (uint8_t*)buf, 8);
  
  return ret;
}

/*
 * Delta Velocity Data Get
 */
int32_t qmi8658c_delta_velocity_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_dVX_L, (uint8_t*)buf, 6);
  
  return ret;
}

/*
 * AttitudeEngine Register 1 Get
 */
int32_t qmi8658c_ae_reg1_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_AE_REG1, (uint8_t*)buf, 1);
  
  return ret;
}

/*
 * AttitudeEngine Register 2 Get
 */
int32_t qmi8658c_ae_reg2_get(qmidev_ctx_t *ctx, uint8_t *buf)
{
  int32_t ret;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_AE_REG2, (uint8_t*)buf, 1);
  
  return ret;
}

/*
 * Software Reset
 */
int32_t qmi8658c_reset_set(qmidev_ctx_t *ctx, uint8_t val)
{
  int32_t ret = 0;
  
  //ret = qmi8658c_write_reg(ctx, QMI8658C_RESET_REG, (uint8_t*)&val, 1);
  
  return ret;
}

/*
 * Software Reset Get
 */
int32_t qmi8658c_reset_get(qmidev_ctx_t *ctx, uint8_t* val)
{
  int32_t ret = 0;
  
  ret = qmi8658c_read_reg(ctx, QMI8658C_RESET_REG, (uint8_t*)val, 1);
  
  return ret;
}

