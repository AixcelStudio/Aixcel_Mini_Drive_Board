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

#ifndef qmi8658c_drv_H
#define qmi8658c_drv_H

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdint.h>
#include <math.h>
#include "common.h"

typedef int32_t (*qmidev_write_ptr)(SPI_Type*, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*qmidev_read_ptr) (SPI_Type*, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  qmidev_write_ptr  write_reg;
  qmidev_read_ptr   read_reg;
  
  /** Customizable optional pointer **/
  SPI_Type* handle;
} qmidev_ctx_t;    

#define PROPERTY_DISABLE                        (0U)
#define PROPERTY_ENABLE                         (1U)

/** I2C Device Address 8 bit format  if SA0=0 -> 6A if SA0=1 -> 6B **/
#define QMI8658C_I2C_ADD_L                     0x6AU
#define QMI8658C_I2C_ADD_H                     0x6BU

/** Device Identification (WHO_AM_I) **/
#define QMI8658C_ID                            0x05U

/** Device RevisionID (REVISION_ID) **/
//#define QMI8658C_REVISION                      0x79U
#define QMI8658C_REVISION                      0x7BU

#define QMI8658C_WHO_AM_I                      0x00U
typedef struct {
  uint8_t                 WHO_AM_I;
} qmi8658c_who_am_i_t;

#define QMI8658C_REVISION_ID                   0x01U
typedef struct {
  uint8_t              REVISION_ID;
} qmi8658c_revision_id_t;

#define QMI8658C_CTRL1                         0x02U
typedef struct {
  uint8_t SensorDisable        : 1;
  uint8_t Reserved             : 4;
  uint8_t SPI_BE               : 1;
  uint8_t SPI_AI               : 1;
  uint8_t SIM                  : 1;
} qmi8658c_ctrl1_t;

#define QMI8658C_CTRL2                         0x03U
typedef struct {
  uint8_t aODR                 : 4;
  uint8_t aFS                  : 3;
  uint8_t aST                  : 1;
} qmi8658c_ctrl2_t;

#define QMI8658C_CTRL3                         0x04U
typedef struct {
  uint8_t gODR                 : 4;
  uint8_t gFS                  : 3;
  uint8_t gST                  : 1;
} qmi8658c_ctrl3_t;

#define QMI8658C_CTRL4                         0x05U
typedef struct {
  uint8_t mODR                 : 3;
  uint8_t mDEV                 : 4;
  uint8_t Reserved             : 1;
} qmi8658c_ctrl4_t;

#define QMI8658C_CTRL5                         0x06U
typedef struct {
  uint8_t aLPF_EN              : 1;
  uint8_t aLPF_MODE            : 2;
  uint8_t Reserved1            : 1;
  uint8_t gLPF_EN              : 1;
  uint8_t gLPF_MODE            : 2;
  uint8_t Reserved2            : 1;
} qmi8658c_ctrl5_t;

#define QMI8658C_CTRL6                         0x07U
typedef struct {
  uint8_t sODR                 : 3;
  uint8_t Reserved             : 4;
  uint8_t sMoD                 : 1;
} qmi8658c_ctrl6_t;

#define QMI8658C_CTRL7                         0x08U
typedef struct {
  uint8_t aEN                  : 1;
  uint8_t gEN                  : 1;
  uint8_t mEN                  : 1;
  uint8_t sEN                  : 1;
  uint8_t gSN                  : 1;
  uint8_t Reserved             : 1;
  uint8_t sys_hs               : 1;
  uint8_t syncSmpl             : 1;
} qmi8658c_ctrl7_t;

#define QMI8658C_CTRL8                         0x09U
typedef struct {
  uint8_t                 Reserved;
} qmi8658c_ctrl8_t;

#define QMI8658C_CTRL9                         0x0AU
typedef struct {
  uint8_t                 Reserved;
} qmi8658c_ctrl9_t;

#define QMI8658C_FIFO_WTM_TH                   0x13U
typedef struct {
  uint8_t                 FIFO_WTM;
} qmi8658c_fifo_wtm_th_t;

#define QMI8658C_FIFO_CTRL                     0x14U
typedef struct {
  uint8_t FIFO_MODE            : 2;
  uint8_t FIFO_SIZE            : 2;
  uint8_t Reserved             : 3;
  uint8_t FIFO_RD_MODE         : 1;
} qmi8658c_fifo_ctrl_t;

#define QMI8658C_FIFO_SMPL_CNT                 0x15U
typedef struct {
  uint8_t               FIFO_SMPL_CNT_LSB;
} qmi8658c_fifo_smpl_cnt_t;

#define QMI8658C_FIFO_STATUS                   0x16U
typedef struct {
  uint8_t FIFO_SMPL_CNT_MSB    : 2;
  uint8_t Reserved             : 2;
  uint8_t FIFO_NOT_EMPTY       : 1;
  uint8_t FIFO_OVFLOW          : 1;
  uint8_t FIFO_WTM             : 1;
  uint8_t FIFO_FULL            : 1;
} qmi8658c_fifo_status_t;

#define QMI8658C_FIFO_DATA                     0x17U
typedef struct {
  uint8_t                 FIFO_DATA;
} qmi8658c_fifo_data_t;

#define QMI8658C_I2CM_STATUS                   0x2CU
typedef struct {
  uint8_t I2CM_active          : 1;
  uint8_t Data_VLD             : 1;
  uint8_t I2CM_done            : 1;
  uint8_t Reserved             : 5;
} qmi8658c_i2cm_status_t;

#define QMI8658C_STATUSINT                     0x2DU
typedef struct {
  uint8_t Avail                : 1;
  uint8_t Locked               : 1;
  uint8_t Reserved             : 6;
} qmi8658c_statusint_t;

#define QMI8658C_STATUS0                       0x2EU
typedef struct {
  uint8_t aDA                  : 1;
  uint8_t gDA                  : 1;
  uint8_t mDA                  : 1;
  uint8_t sDA                  : 1;
  uint8_t Reserved             : 4;
} qmi8658c_status0_t;

#define QMI8658C_STATUS1                       0x2FU
typedef struct {
  uint8_t CmdDone              : 1;
  uint8_t Reserved             : 7;
} qmi8658c_status1_t;

#define QMI8658C_TIMESTAMP_L                   0x30U
typedef struct {
  uint8_t              TIMESTAMP_L;
} qmi8658c_timestamp_l_t;

#define QMI8658C_TIMESTAMP_M                   0x31U
typedef struct {
  uint8_t              TIMESTAMP_M;
} qmi8658c_timestamp_m_t;

#define QMI8658C_TIMESTAMP_H                   0x32U
typedef struct {
  uint8_t              TIMESTAMP_H;
} qmi8658c_timestamp_h_t;

#define QMI8658C_TEMP_L                        0x33U
typedef struct {
  uint8_t              TEMP_L;
} qmi8658c_temp_l_t;

#define QMI8658C_TEMP_H                        0x34U
typedef struct {
  uint8_t              TEMP_H;
} qmi8658c_temp_h_t;

#define QMI8658C_AX_L                          0x35U
typedef struct {
  uint8_t               AX_L;
} qmi8658c_ax_l_t;

#define QMI8658C_AX_H                          0x36U
typedef struct {
  uint8_t               AX_H;
} qmi8658c_ax_h_t;

#define QMI8658C_AY_L                          0x37U
typedef struct {
  uint8_t               AY_L;
} qmi8658c_ay_l_t;

#define QMI8658C_AY_H                          0x38U
typedef struct {
  uint8_t               AY_H;
} qmi8658c_ay_h_t;

#define QMI8658C_AZ_L                          0x39U
typedef struct {
  uint8_t               AZ_L;
} qmi8658c_az_l_t;

#define QMI8658C_AZ_H                          0x3AU
typedef struct {
  uint8_t               AZ_H;
} qmi8658c_az_h_t;

#define QMI8658C_GX_L                          0x3BU
typedef struct {
  uint8_t               GX_L;
} qmi8658c_gx_l_t;

#define QMI8658C_GX_H                          0x3CU
typedef struct {
  uint8_t               GX_H;
} qmi8658c_gx_h_t;

#define QMI8658C_GY_L                          0x3DU
typedef struct {
  uint8_t               GY_L;
} qmi8658c_gy_l_t;

#define QMI8658C_GY_H                          0x3EU
typedef struct {
  uint8_t               GY_H;
} qmi8658c_gy_h_t;

#define QMI8658C_GZ_L                          0x3FU
typedef struct {
  uint8_t               GZ_L;
} qmi8658c_gz_l_t;

#define QMI8658C_GZ_H                          0x40U
typedef struct {
  uint8_t               GZ_H;
} qmi8658c_gz_h_t;

#define QMI8658C_MX_L                          0x41U
typedef struct {
  uint8_t               MX_L;
} qmi8658c_mx_l_t;

#define QMI8658C_MX_H                          0x42U
typedef struct {
  uint8_t               MX_H;
} qmi8658c_mx_h_t;

#define QMI8658C_MY_L                          0x43U
typedef struct {
  uint8_t               MY_L;
} qmi8658c_my_l_t;

#define QMI8658C_MY_H                          0x44U
typedef struct {
  uint8_t               MY_H;
} qmi8658c_my_h_t;

#define QMI8658C_MZ_L                          0x45U
typedef struct {
  uint8_t               MZ_L;
} qmi8658c_mz_l_t;

#define QMI8658C_MZ_H                          0x46U
typedef struct {
  uint8_t               MZ_H;
} qmi8658c_mz_h_t;

#define QMI8658C_dQW_L                         0x49U
typedef struct {
  uint8_t               dQW_L;
} qmi8658c_dqw_l_t;

#define QMI8658C_dQW_H                         0x4AU
typedef struct {
  uint8_t               dQW_H;
} qmi8658c_dqw_h_t;

#define QMI8658C_dQX_L                         0x4BU
typedef struct {
  uint8_t               dQX_L;
} qmi8658c_dqx_l_t;

#define QMI8658C_dQX_H                         0x4CU
typedef struct {
  uint8_t               dQX_H;
} qmi8658c_dqx_h_t;

#define QMI8658C_dQY_L                         0x4DU
typedef struct {
  uint8_t               dQY_L;
} qmi8658c_dqy_l_t;

#define QMI8658C_dQY_H                         0x4EU
typedef struct {
  uint8_t               dQY_H;
} qmi8658c_dqy_h_t;

#define QMI8658C_dQZ_L                         0x4FU
typedef struct {
  uint8_t               dQZ_L;
} qmi8658c_dqz_l_t;

#define QMI8658C_dQZ_H                         0x50U
typedef struct {
  uint8_t               dQZ_H;
} qmi8658c_dqz_h_t;

#define QMI8658C_dVX_L                         0x51U
typedef struct {
  uint8_t               dVX_L;
} qmi8658c_dvx_l_t;

#define QMI8658C_dVX_H                         0x52U
typedef struct {
  uint8_t               dVX_H;
} qmi8658c_dvx_h_t;

#define QMI8658C_dVY_L                         0x53U
typedef struct {
  uint8_t               dVY_L;
} qmi8658c_dvy_l_t;

#define QMI8658C_dVY_H                         0x54U
typedef struct {
  uint8_t               dVY_H;
} qmi8658c_dvy_h_t;

#define QMI8658C_dVZ_L                         0x55U
typedef struct {
  uint8_t               dVZ_L;
} qmi8658c_dvz_l_t;

#define QMI8658C_dVZ_H                         0x56U
typedef struct {
  uint8_t               dVZ_H;
} qmi8658c_dvz_h_t;

#define QMI8658C_AE_REG1                       0x57U
typedef struct {
  uint8_t ax_clip              : 1;
  uint8_t ay_clip              : 1;
  uint8_t az_clip              : 1;
  uint8_t wx_clip              : 1;
  uint8_t wy_clip              : 1;
  uint8_t wz_clip              : 1;
  uint8_t GyroBiasAck          : 1;
  uint8_t Reserved             : 1;
} qmi8658c_ae_reg1_t;

#define QMI8658C_AE_REG2                       0x58U
typedef struct {
  uint8_t dvx_of               : 1;
  uint8_t dvy_of               : 1;
  uint8_t dvz_of               : 1;
  uint8_t Reserved             : 5;
} qmi8658c_ae_reg2_t;

#define QMI8658C_RESET_REG                     0x60U

typedef union{
  qmi8658c_who_am_i_t                        who_am_i;
  qmi8658c_revision_id_t                     revision_id;
  qmi8658c_ctrl1_t                           ctrl1;
  qmi8658c_ctrl2_t                           ctrl2;  
  qmi8658c_ctrl3_t                           ctrl3; 
  qmi8658c_ctrl4_t                           ctrl4;
  qmi8658c_ctrl5_t                           ctrl5;
  qmi8658c_ctrl6_t                           ctrl6;
  qmi8658c_ctrl7_t                           ctrl7;
  qmi8658c_ctrl8_t                           ctrl8;
  qmi8658c_ctrl9_t                           ctrl9;
  qmi8658c_fifo_wtm_th_t                     fifo_wtm_th;
  qmi8658c_fifo_ctrl_t                       fifo_ctrl;
  qmi8658c_fifo_smpl_cnt_t                   fifo_smpl_cnt;
  qmi8658c_fifo_status_t                     fifo_status;
  qmi8658c_fifo_data_t                       fifo_data;
  qmi8658c_i2cm_status_t                     i2cm_status;
  qmi8658c_statusint_t                       statusint;
  qmi8658c_status0_t                         status0;
  qmi8658c_status1_t                         status1;
  qmi8658c_timestamp_l_t                     timestamp_l;
  qmi8658c_timestamp_m_t                     timestamp_m;
  qmi8658c_timestamp_h_t                     timestamp_h;
  qmi8658c_temp_l_t                          temp_l;
  qmi8658c_temp_h_t                          temp_h;
  qmi8658c_ax_l_t                            ax_l;
  qmi8658c_ax_h_t                            ax_h; 
  qmi8658c_ay_l_t                            ay_l;
  qmi8658c_ay_h_t                            ay_h;
  qmi8658c_az_l_t                            az_l;
  qmi8658c_az_h_t                            az_h;
  qmi8658c_gx_l_t                            gx_l;
  qmi8658c_gx_h_t                            gx_h;
  qmi8658c_gy_l_t                            gy_l;
  qmi8658c_gy_h_t                            gy_h;
  qmi8658c_gz_l_t                            gz_l;
  qmi8658c_gz_h_t                            gz_h;
  qmi8658c_mx_l_t                            mx_l;
  qmi8658c_mx_h_t                            mx_h;
  qmi8658c_my_l_t                            my_l;
  qmi8658c_my_h_t                            my_h;
  qmi8658c_mz_l_t                            mz_l;
  qmi8658c_mz_h_t                            mz_h;
  qmi8658c_dqw_l_t                           dqw_l;
  qmi8658c_dqw_h_t                           dqw_h;
  qmi8658c_dqx_l_t                           dqx_l;
  qmi8658c_dqx_h_t                           dqx_h;
  qmi8658c_dqy_l_t                           dqy_l;
  qmi8658c_dqy_h_t                           dqy_h;
  qmi8658c_dqz_l_t                           dqz_l;
  qmi8658c_dqz_h_t                           dqz_h;
  qmi8658c_dvx_l_t                           dvx_l;
  qmi8658c_dvx_h_t                           dvx_h;
  qmi8658c_dvy_l_t                           dvy_l;
  qmi8658c_dvy_h_t                           dvy_h;
  qmi8658c_dvz_l_t                           dvz_l;
  qmi8658c_dvz_h_t                           dvz_h; 
  qmi8658c_ae_reg1_t                         ae_reg1;
  qmi8658c_ae_reg2_t                         ae_reg2;
  uint8_t                                    byte;
} qmi8658c_reg_t;

int32_t qmi8658c_read_reg(qmidev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t qmi8658c_write_reg(qmidev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

/*
 * Serial Interface Mode
 */
typedef enum {
  QMI8658C_SIM_4_Wire_SPI  = 0,
  QMI8658C_SIM_3_Wire_SPI  = 1,
} qmi8658c_sim_t;
int32_t qmi8658c_sim_set(qmidev_ctx_t *ctx, qmi8658c_sim_t val);
int32_t qmi8658c_sim_get(qmidev_ctx_t *ctx, qmi8658c_sim_t *val);

/*
 * Serial Interface Address Auto Increment or Not
 */
typedef enum {
  QMI8658C_SPI_AI_ON       = 0,
  QMI8658C_SPI_AI_OFF      = 1,
} qmi8658c_sip_ai_t;
int32_t qmi8658c_sip_ai_set(qmidev_ctx_t *ctx, qmi8658c_sip_ai_t val);
int32_t qmi8658c_sip_ai_get(qmidev_ctx_t *ctx, qmi8658c_sip_ai_t *val);

/*
 * SPI Read Data Big Endian or Little Endian
 */
typedef enum {
  QMI8658C_SPI_LE          = 0,
  QMI8658C_SPI_BE          = 1,
} qmi8658c_spi_be_t;
int32_t qmi8658c_spi_be_set(qmidev_ctx_t *ctx, qmi8658c_spi_be_t val);
int32_t qmi8658c_spi_be_get(qmidev_ctx_t *ctx, qmi8658c_spi_be_t *val);

/*
 * Sensor Enable or Disable
 */
typedef enum {
  QMI8658C_Sen_Enable      = 0,
  QMI8658C_Sen_Disable     = 1,
} qmi8658c_sensor_sta_t;
int32_t qmi8658c_sensor_sta_set(qmidev_ctx_t *ctx, qmi8658c_sensor_sta_t val);
int32_t qmi8658c_sensor_sta_get(qmidev_ctx_t *ctx, qmi8658c_sensor_sta_t *val);

extern float_t qmi8658c_from_fs2g_to_mg(int16_t lsb);
extern float_t qmi8658c_from_fs4g_to_mg(int16_t lsb);
extern float_t qmi8658c_from_fs8g_to_mg(int16_t lsb);
extern float_t qmi8658c_from_fs16g_to_mg(int16_t lsb);

extern float_t qmi8658c_from_fs16dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs32dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs64dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs128dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs256dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs512dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs1024dps_to_mdps(int16_t lsb);
extern float_t qmi8658c_from_fs2048dps_to_mdps(int16_t lsb);

extern float_t qmi8658c_from_lsb_to_celsius(int16_t lsb);

/*
 * Accelerometer Self Test Enable or Disable
 */
typedef enum {
  QMI8658C_XL_Self_Test_Disable      = 0,
  QMI8658C_XL_Self_Test_Enable       = 1,
} qmi8658c_xl_self_test_t;
int32_t qmi8658c_xl_self_test_set(qmidev_ctx_t *ctx, qmi8658c_xl_self_test_t val);
int32_t qmi8658c_xl_self_test_get(qmidev_ctx_t *ctx, qmi8658c_xl_self_test_t *val);

/*
 * Accelerometer Full Scale
 */
typedef enum {
  QMI8658C_2g              =  0,
  QMI8658C_4g              =  1,
  QMI8658C_8g              =  2,
  QMI8658C_16g             =  3,
  QMI8658C_XL_FS_ND        =  4,   /* ERROR CODE */
} qmi8658c_fs_xl_t;
int32_t qmi8658c_xl_full_scale_set(qmidev_ctx_t *ctx, qmi8658c_fs_xl_t val);
int32_t qmi8658c_xl_full_scale_get(qmidev_ctx_t *ctx, qmi8658c_fs_xl_t *val);

/*
 * Accelerometer Output Data Rate
 */
typedef enum {
  QMI8658C_XL_ODR_8kHz     =  0,
  QMI8658C_XL_ODR_4kHz     =  1,
  QMI8658C_XL_ODR_2kHz     =  2,
  QMI8658C_XL_ODR_1kHz     =  3,
  QMI8658C_XL_ODR_500Hz    =  4,
  QMI8658C_XL_ODR_250Hz    =  5,
  QMI8658C_XL_ODR_125Hz    =  6,
  QMI8658C_XL_ODR_62Hz5    =  7,
  QMI8658C_XL_ODR_31Hz25   =  8,
  QMI8658C_XL_ODR_128Hz    = 12,
  QMI8658C_XL_ODR_21Hz     = 13,
  QMI8658C_XL_ODR_11Hz     = 14,
  QMI8658C_XL_ODR_3Hz      = 15,
  QMI8658C_XL_ODR_ND       = 16,
} qmi8658c_odr_xl_t;
int32_t qmi8658c_xl_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_xl_t val);
int32_t qmi8658c_xl_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_xl_t *val);

/*
 * Gyroscope Self Test Enable or Disable
 */
typedef enum {
  QMI8658C_GY_Self_Test_Disable      =  0,
  QMI8658C_GY_Self_Test_Enable       =  1,
} qmi8658c_gy_self_test_t;
int32_t qmi8658c_gy_self_test_set(qmidev_ctx_t *ctx, qmi8658c_gy_self_test_t val);
int32_t qmi8658c_gy_self_test_get(qmidev_ctx_t *ctx, qmi8658c_gy_self_test_t *val);

/*
 * Gyroscope Full Scale
 */
typedef enum {
  QMI8658C_16dps           =  0,
  QMI8658C_32dps           =  1,
  QMI8658C_64dps           =  2,
  QMI8658C_128dps          =  3,
  QMI8658C_256dps          =  4,
  QMI8658C_512dps          =  5,
  QMI8658C_1024dps         =  6,
  QMI8658C_2048dps         =  7,
  QMI8658C_GY_FS_ND        =  8,    /* ERROR CODE */
} qmi8658c_fs_gy_t;
int32_t qmi8658c_gy_full_scale_set(qmidev_ctx_t *ctx, qmi8658c_fs_gy_t val);
int32_t qmi8658c_gy_full_scale_get(qmidev_ctx_t *ctx, qmi8658c_fs_gy_t *val);

/*
 * Gyroscope Output Data Rate
 */
typedef enum {
  QMI8658C_GY_ODR_8kHz     =  0,
  QMI8658C_GY_ODR_4kHz     =  1,
  QMI8658C_GY_ODR_2kHz     =  2,
  QMI8658C_GY_ODR_1kHz     =  3,
  QMI8658C_GY_ODR_500Hz    =  4,
  QMI8658C_GY_ODR_250Hz    =  5,
  QMI8658C_GY_ODR_125Hz    =  6,
  QMI8658C_GY_ODR_62Hz5    =  7,
  QMI8658C_GY_ODR_31Hz25   =  8,
  QMI8658C_GY_ODR_ND       =  9,    /* ERROR CODE */
} qmi8658c_odr_gy_t;
int32_t qmi8658c_gy_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_gy_t val);
int32_t qmi8658c_gy_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_gy_t *val);

/*
 * Magnetometer Output Data Rate
 */
typedef enum {
  QMI8658C_MG_ODR_1kHz     =  0,
  QMI8658C_MG_ODR_500Hz    =  1,
  QMI8658C_MG_ODR_250Hz    =  2,
  QMI8658C_MG_ODR_125Hz    =  3,
  QMI8658C_MG_ODR_62Hz5    =  4,
  QMI8658C_MG_ODR_31Hz25   =  5,
  QMI8658C_MG_ODR_ND       =  6,    /* ERROR CODE */
} qmi8658c_odr_mg_t;
int32_t qmi8658c_mg_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_mg_t val);
int32_t qmi8658c_mg_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_mg_t *val);


/*
 * Accelerometer Low-Pass Filter  Enable or Disable
 */
typedef enum {
  QMI8658C_XL_aLPF_Disable    =  0,
  QMI8658C_XL_aLPF_Enable     =  1,
} qmi8658c_xl_alpf_en_t;
int32_t qmi8658c_xl_alpf_en_set(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_en_t val);
int32_t qmi8658c_xl_alpf_en_get(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_en_t *val);

/*
 * Accelerometer Low-Pass Filter  Mode
 */
typedef enum {
  QMI8658C_XL_aLPF_Mode0      =  0,
  QMI8658C_XL_aLPF_Mode1      =  1,
  QMI8658C_XL_aLPF_Mode2      =  3,
  QMI8658C_XL_aLPF_Mode3      =  4,
} qmi8658c_xl_alpf_mode_t;
int32_t qmi8658c_xl_alpf_mode_set(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_mode_t val);
int32_t qmi8658c_xl_alpf_mode_get(qmidev_ctx_t *ctx, qmi8658c_xl_alpf_mode_t *val);

/*
 * Gyroscope Low-Pass Filter  Enable or Disable
 */
typedef enum {
  QMI8658C_GY_aLPF_Disable    =  0,
  QMI8658C_GY_aLPF_Enable     =  1,
} qmi8658c_gy_alpf_en_t;
int32_t qmi8658c_gy_alpf_en_set(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_en_t val);
int32_t qmi8658c_gy_alpf_en_get(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_en_t *val);

/*
 * Gyroscope Low-Pass Filter  Mode
 */
typedef enum {
  QMI8658C_GY_aLPF_Mode0      =  0,
  QMI8658C_GY_aLPF_Mode1      =  1,
  QMI8658C_GY_aLPF_Mode2      =  3,
  QMI8658C_GY_aLPF_Mode3      =  4,
} qmi8658c_gy_alpf_mode_t;
int32_t qmi8658c_gy_alpf_mode_set(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_mode_t val);
int32_t qmi8658c_gy_alpf_mode_get(qmidev_ctx_t *ctx, qmi8658c_gy_alpf_mode_t *val);

/*
 * Attitude Engine Output Data Rate
 */
typedef enum {
  QMI8658C_AE_ODR_1Hz         =  0,
  QMI8658C_AE_ODR_2Hz         =  1,
  QMI8658C_AE_ODR_4Hz         =  2,
  QMI8658C_AE_ODR_8Hz         =  3,
  QMI8658C_AE_ODR_16Hz        =  4,
  QMI8658C_AE_ODR_32Hz        =  5,
  QMI8658C_AE_ODR_64Hz        =  6,
  QMI8658C_AE_ODR_ND          =  7,    /* ERROR CODE */
} qmi8658c_odr_ae_t;
int32_t qmi8658c_ae_data_rate_set(qmidev_ctx_t *ctx, qmi8658c_odr_ae_t val);
int32_t qmi8658c_ae_data_rate_get(qmidev_ctx_t *ctx, qmi8658c_odr_ae_t *val);

/*
 * Motion on Demand Enable or Disable
 */
typedef enum {
  QMI8658C_sMoD_Disable    =  0,
  QMI8658C_sMoD_Enable     =  1,
} qmi8658c_amod_en_t;
int32_t qmi8658c_amod_en_set(qmidev_ctx_t *ctx, qmi8658c_amod_en_t val);
int32_t qmi8658c_amod_en_get(qmidev_ctx_t *ctx, qmi8658c_amod_en_t *val);

/*
 * Accelerometer Enable or Disable
 */
typedef enum {
  QMI8658C_XL_Disable    =  0,
  QMI8658C_XL_Enable     =  1,
} qmi8658c_xl_en_t;
int32_t qmi8658c_xl_en_set(qmidev_ctx_t *ctx, qmi8658c_xl_en_t val);
int32_t qmi8658c_xl_en_get(qmidev_ctx_t *ctx, qmi8658c_xl_en_t *val);

/*
 * Gyroscope Enable or Disable
 */
typedef enum {
  QMI8658C_GY_Disable    =  0,
  QMI8658C_GY_Enable     =  1,
} qmi8658c_gy_en_t;
int32_t qmi8658c_gy_en_set(qmidev_ctx_t *ctx, qmi8658c_gy_en_t val);
int32_t qmi8658c_gy_en_get(qmidev_ctx_t *ctx, qmi8658c_gy_en_t *val);

/*
 * Magnetometer Enable or Disable
 */
typedef enum {
  QMI8658C_MG_Disable    =  0,
  QMI8658C_MG_Enable     =  1,
} qmi8658c_mg_en_t;
int32_t qmi8658c_mg_en_set(qmidev_ctx_t *ctx, qmi8658c_mg_en_t val);
int32_t qmi8658c_mg_en_get(qmidev_ctx_t *ctx, qmi8658c_mg_en_t *val);

/*
 * AttitudeEngine Orientation And Velocity Increment Computation Enable or Disable
 */
typedef enum {
  QMI8658C_AEs_Disable    =  0,
  QMI8658C_AEs_Enable     =  1,
} qmi8658c_aes_en_t;
int32_t qmi8658c_aes_en_set(qmidev_ctx_t *ctx, qmi8658c_aes_en_t val);
int32_t qmi8658c_aes_en_get(qmidev_ctx_t *ctx, qmi8658c_aes_en_t *val);

/*
 * Gyroscope in Full Mode or Snooze Mode
 */
typedef enum {
  QMI8658C_GY_Full_Mode       =  0,
  QMI8658C_GY_Snooze_Mode     =  1,
} qmi8658c_gy_mode_t;
int32_t qmi8658c_gy_mode_set(qmidev_ctx_t *ctx, qmi8658c_gy_mode_t val);
int32_t qmi8658c_gy_mode_get(qmidev_ctx_t *ctx, qmi8658c_gy_mode_t *val);

/*
 * Clock Based On ODR or High Speed Internal Clock
 */
typedef enum {
  QMI8658C_Clock_Based_On_ODR            =  0,
  QMI8658C_High_Speed_Internal_Clock     =  1,
} qmi8658c_sys_hs_t;
int32_t qmi8658c_sys_hs_set(qmidev_ctx_t *ctx, qmi8658c_sys_hs_t val);
int32_t qmi8658c_sys_hs_get(qmidev_ctx_t *ctx, qmi8658c_sys_hs_t *val);

/*
 * SyncSmpl Mode Enable or Disable
 */
typedef enum {
  QMI8658C_SyncSmple_Disable    =  0,
  QMI8658C_SyncSmple_Enable     =  1,
} qmi8658c_syncsmple_en_t;
int32_t qmi8658c_syncsmple_set(qmidev_ctx_t *ctx, qmi8658c_syncsmple_en_t val);
int32_t qmi8658c_syncsmple_get(qmidev_ctx_t *ctx, qmi8658c_syncsmple_en_t *val);

/*
 * FIFO Watermark Set
 */
int32_t qmi8658c_fifo_wtm_set(qmidev_ctx_t *ctx, qmi8658c_reg_t val);
int32_t qmi8658c_fifo_wtm_get(qmidev_ctx_t *ctx, qmi8658c_reg_t *val);

/*
 * FIFO Mode Set
 */
typedef enum {
  QMI8658C_BYPASS_Mode             =  0,
  QMI8658C_FIFO_Mode               =  1,
  QMI8658C_STREAM_Mode             =  2,
  QMI8658C_STREAM_TO_FIFO_Mode     =  3,
} qmi8658c_fifo_mode_t;
int32_t qmi8658c_fifo_mode_set(qmidev_ctx_t *ctx, qmi8658c_fifo_mode_t val);
int32_t qmi8658c_fifo_mode_get(qmidev_ctx_t *ctx, qmi8658c_fifo_mode_t *val);

/*
 * FIFO Size Set
 */
typedef enum {
  QMI8658C_FIFO_16Samples          =  0,
  QMI8658C_FIFO_32Samples          =  1,
  QMI8658C_FIFO_64Samples          =  2,
  QMI8658C_FIFO_128Samples         =  3,
} qmi8658c_fifo_size_t;
int32_t qmi8658c_fifo_size_set(qmidev_ctx_t *ctx, qmi8658c_fifo_size_t val);
int32_t qmi8658c_fifo_size_get(qmidev_ctx_t *ctx, qmi8658c_fifo_size_t *val);

/*
 * FIFO RD Mode Get
 */
int32_t qmi8658c_fifo_rd_mode_get(qmidev_ctx_t *ctx, qmi8658c_reg_t *val);

/*
 * FIFO Sample Cnt Get
 */
int32_t qmi8658c_fifo_sample_cnt_get(qmidev_ctx_t *ctx, uint16_t *val);

/*
 * FIFO Empty Sta Get
 */
typedef enum {
  QMI8658C_FIFO_IS_EMPTY        =  0,
  QMI8658C_FIFO_NOT_EMPTY       =  1,
} qmi8658c_fifo_empty_sta_t;
int32_t qmi8658c_fifo_empty_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_empty_sta_t *val);

/*
 * FIFO Overflow Sta Get
 */
typedef enum {
  QMI8658C_FIFO_NOT_HAPPENED    =  0,
  QMI8658C_FIFO_HAS_HAPPENED    =  1,
} qmi8658c_fifo_overflow_sta_t;
int32_t qmi8658c_fifo_overflow_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_overflow_sta_t *val);

/*
 * FIFO Watermark Hit Flag Get
 */
typedef enum {
  QMI8658C_FIFO_Watermark_NOT_HIT    =  0,
  QMI8658C_FIFO_Watermark_HAS_HIT    =  1,
} qmi8658c_fifo_watermark_sta_t;
int32_t qmi8658c_fifo_watermark_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_watermark_sta_t *val);

/*
 * FIFO Full Sta Get
 */
typedef enum {
  QMI8658C_FIFO_NOT_FULL    =  0,
  QMI8658C_FIFO_IS_FULL     =  1,
} qmi8658c_fifo_full_sta_t;
int32_t qmi8658c_fifo_full_sta_get(qmidev_ctx_t *ctx, qmi8658c_fifo_full_sta_t *val);

/*
 * FIFO Pattern Get
 */
int32_t qmi8658c_fifo_pattern_get(qmidev_ctx_t *ctx, uint16_t *val);

/*
 * FIFO_DATA Get
 */
int32_t qmi8658c_fifo_data_get(qmidev_ctx_t *ctx, uint8_t *val);

/*
 * I2C Master Active Sta Get
 */
typedef enum {
  QMI8658C_FIFO_I2C_Master_Transaction_Not_Done    =  0,
  QMI8658C_FIFO_I2C_Master_Transaction_Is_Done     =  1,
} qmi8658c_i2cm_active_sta_t;
int32_t qmi8658c_i2cm_active_sta_get(qmidev_ctx_t *ctx, qmi8658c_i2cm_active_sta_t *val);

/*
 * Magnetometer Data Valid Sta Get
 */
typedef enum {
  QMI8658C_FIFO_MG_Data_Not_Valid    =  0,
  QMI8658C_FIFO_MG_Data_Is_Valid     =  1,
} qmi8658c_mg_data_sta_t;
int32_t qmi8658c_mg_data_sta_get(qmidev_ctx_t *ctx, qmi8658c_mg_data_sta_t *val);

/*
 * I2C Master Data Movement Sta Get
 */
typedef enum {
  QMI8658C_FIFO_I2C_Master_Data_Movement_Not_Done    =  0,
  QMI8658C_FIFO_I2C_Master_Data_Movement_Is_Done     =  1,
} qmi8658c_i2cm_data_movement_sta_t;
int32_t qmi8658c_i2cm_data_movement_sta_get(qmidev_ctx_t *ctx, qmi8658c_i2cm_data_movement_sta_t *val);

/*
 * Sensor Data Available Sta Get
 */
typedef enum {
  QMI8658C_Sensor_Data_Not_Available    =  0,
  QMI8658C_Sensor_Data_Is_Available     =  1,
} qmi8658c_sen_data_available_sta_t;
int32_t qmi8658c_sen_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_sen_data_available_sta_t *val);

/*
 * Sensor Data Locked Sta Get
 */
typedef enum {
  QMI8658C_Sensor_Data_Not_Locked    =  0,
  QMI8658C_Sensor_Data_Is_Locked     =  1,
} qmi8658c_sen_data_locked_sta_t;
int32_t qmi8658c_sen_data_locked_sta_get(qmidev_ctx_t *ctx, qmi8658c_sen_data_locked_sta_t *val);

/*
 * Accelerometer New Data Available Sta Get
 */
typedef enum {
  QMI8658C_XL_New_Data_Not_Available    =  0,
  QMI8658C_XL_New_Data_Is_Available     =  1,
} qmi8658c_xl_new_data_available_sta_t;
int32_t qmi8658c_xl_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_xl_new_data_available_sta_t *val);

/*
 * Gyroscope New Data Available Sta Get
 */
typedef enum {
  QMI8658C_GY_New_Data_Not_Available    =  0,
  QMI8658C_GY_New_Data_Is_Available     =  1,
} qmi8658c_gy_new_data_available_sta_t;
int32_t qmi8658c_gy_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_gy_new_data_available_sta_t *val);

/*
 * Magnetometer New Data Available Sta Get
 */
typedef enum {
  QMI8658C_MG_New_Data_Not_Available    =  0,
  QMI8658C_MG_New_Data_Is_Available     =  1,
} qmi8658c_mg_new_data_available_sta_t;
int32_t qmi8658c_mg_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_mg_new_data_available_sta_t *val);

/*
 * Attitude Engine New Data Available Sta Get
 */
typedef enum {
  QMI8658C_AE_New_Data_Not_Available    =  0,
  QMI8658C_AE_New_Data_Is_Available     =  1,
} qmi8658c_ae_new_data_available_sta_t;
int32_t qmi8658c_ae_new_data_available_sta_get(qmidev_ctx_t *ctx, qmi8658c_ae_new_data_available_sta_t *val);

/*
 * Ctrl9 Command Done Sta Get
 */
typedef enum {
  QMI8658C_Ctrl9_Command_Not_Done    =  0,
  QMI8658C_Ctrl9_Command_Is_Done     =  1,
} qmi8658c_ctrl9_command_done_sta_t;
int32_t qmi8658c_ctrl9_command_done_sta_get(qmidev_ctx_t *ctx, qmi8658c_ctrl9_command_done_sta_t *val);

/*
 * Timestamp Data Get
 */
int32_t qmi8658c_timestamp_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Temperatur Data Get
 */
int32_t qmi8658c_temperature_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Accelerometer Data Get
 */
int32_t qmi8658c_accelerometer_raw_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Gyroscope Data Get
 */
int32_t qmi8658c_gyroscope_raw_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Magnetometer Data Get
 */
int32_t qmi8658c_magnetometer_raw_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Quaternion Data Get
 */
int32_t qmi8658c_quaternion_Increment_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Delta Velocity Data Get
 */
int32_t qmi8658c_delta_velocity_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * AttitudeEngine Register 1 Get
 */
int32_t qmi8658c_ae_reg1_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * AttitudeEngine Register 2 Get
 */
int32_t qmi8658c_ae_reg2_get(qmidev_ctx_t *ctx, uint8_t *buf);

/*
 * Software Reset
 */
int32_t qmi8658c_reset_set(qmidev_ctx_t *ctx, uint8_t val);

/*
 * Software Reset Get
 */
int32_t qmi8658c_reset_get(qmidev_ctx_t *ctx, uint8_t* val);

/*
 * Get WHO_AM_I
 */

int32_t qmi8658c_who_am_i_get(qmidev_ctx_t *ctx, uint8_t *val);
/*
 * GET REVISION_ID
 */
int32_t qmi8658c_revision_id_get(qmidev_ctx_t *ctx, uint8_t *val);


#ifdef __cplusplus
}
#endif

#endif