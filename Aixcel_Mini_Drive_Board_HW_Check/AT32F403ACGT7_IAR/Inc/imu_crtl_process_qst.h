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

#ifndef IMU_CTRL_PROCESS_QST_H
#define IMU_CTRL_PROCESS_QST_H

#ifdef __cplusplus
  extern "C" {
#endif

#if (IMU_MANUFACTURE_ID == MANUFACTURE_ID_QST)
    
#include "common.h"
#include "qmi8658c_drv.h"

#define    DEFAULT_IMU_REG_VAL                   0x00

#define    IMU_RX_BUF_CELL_CNT                   36
#define    IMU_RX_BUF_CELL_SIZE                  sizeof(IMU_Data_Typedef)    


/*枚举IMU当前所处状态*/
typedef enum
{
    IMU_IS_STOPPED = 0,
    IMU_IS_RESET,
    IMU_IS_WORKING,  
    IMU_IS_ERROR,
} IMU_State_Typedef;

/*枚举IMU buffer的状态*/
typedef enum
{
    BUF_IS_EMPTY = 0,
    BUF_IS_WRITING,
    BUF_IS_READING,
    BUF_IS_READY, 
} IMU_Buf_State_Typedef;

typedef enum
{
    NORMAL_MODE = 0,
    FIFO_MODE = 1,
} IMU_DATA_PATCH_MODE_Typedef;

typedef __packed struct __IMU_Accl_Work_Pattern_Typedef
{
    uint8_t              AcclFullScale;
    
    uint8_t              AcclDataRate;
    
    uint8_t              DataPatchMode;

} IMU_Accl_Work_Pattern_Typedef;

typedef __packed struct __IMU_Gyro_Work_Pattern_Typedef
{
    uint8_t              GyroFullScale;
    
    uint8_t              GyroDataRate;
    
    uint8_t              DataPatchMode;

} IMU_Gyro_Work_Pattern_Typedef;

typedef __packed struct __IMU_Status_Typedef
{
    IMU_State_Typedef    AcclState;
    
    IMU_State_Typedef    GyroState;

} IMU_Status_Typedef;

typedef __packed struct __IMU_Info_Typedef
{
    uint8_t    MFR;
    
    uint8_t    Type;
    
    uint8_t    Reserve;
    
    uint8_t    WhoAmI;

} IMU_Info_Typedef;

typedef __packed struct __IMU_Accl_Data_Set_Typedef
{
    int8_t     AcclData[6];

    int8_t     TimeData[4];

    int8_t     TempData[2];

} IMU_Accl_Data_Set_Typedef;

typedef __packed struct __IMU_Gyro_Data_Set_Typedef
{
    int8_t     GyroData[6];

    int8_t     TimeData[4];

    int8_t     TempData[2];

} IMU_Gyro_Data_Set_Typedef;

typedef __packed struct __IMU_Combine_Data_Set_Typedef
{
    int8_t     AcclData[6];

    int8_t     GyroData[6];

    int8_t     TimeData[4];

    int8_t     TempData[2];

} IMU_Combine_Data_Set_Typedef;

typedef __packed struct __IMU_Data_Typedef
{
    float      AcclX;
    float      AcclY;
    float      AcclZ;

    float      GyroX;
    float      GyroY;
    float      GyroZ;

    float      TemperatureDeg;
    
    uint64_t   TimeStampUs;

} IMU_Data_Typedef;

typedef __packed struct __Triple_Index_Ring_Buffer_Typedef
{
    IMU_Data_Typedef  *pBuffer;

    uint32_t          Head; 
    uint32_t          Tail;

    uint32_t          Fake_Head; 
    uint32_t          Fake_Tail;

    uint32_t          View_Window_Head; 
    uint32_t          View_Window_Tail; 

    uint32_t          Buffer_Size;  

} Triple_Index_Ring_Buffer_Typedef;

typedef __packed struct __QMI8658C_Reg_TypeDef 
{
  uint8_t                               WHO_AM_I;
  uint8_t                            REVISION_ID;
  qmi8658c_reg_t                           CTRL1;
  qmi8658c_reg_t                           CTRL2;  
  qmi8658c_reg_t                           CTRL3; 
  qmi8658c_reg_t                           CTRL4;
  qmi8658c_reg_t                           CTRL5;
  qmi8658c_reg_t                           CTRL6;
  qmi8658c_reg_t                           CTRL7;
  qmi8658c_reg_t                           CTRL8;
  qmi8658c_reg_t                           CTRL9;
  uint8_t                                 CAL1_L;
  uint8_t                                 CAL1_H;
  uint8_t                                 CAL2_L;
  uint8_t                                 CAL2_H;
  uint8_t                                 CAL3_L;
  uint8_t                                 CAL3_H;
  uint8_t                                 CAL4_L;
  uint8_t                                 CAL4_H;
  qmi8658c_reg_t                     FIFO_WTM_th;
  qmi8658c_reg_t                       FIFO_CTRL;
  qmi8658c_reg_t                   FIFO_SMPL_cnt;
  qmi8658c_reg_t                     FIFO_STATUS;
  qmi8658c_reg_t                       FIFO_DATA;
  uint8_t                            RESERVED_24;
  uint8_t                            RESERVED_25;
  uint8_t                            RESERVED_26;
  uint8_t                            RESERVED_27;
  uint8_t                            RESERVED_28;
  uint8_t                            RESERVED_29;
  uint8_t                            RESERVED_30;
  uint8_t                            RESERVED_31;
  uint8_t                            RESERVED_32;
  uint8_t                            RESERVED_33;
  uint8_t                            RESERVED_34;
  uint8_t                            RESERVED_35;
  uint8_t                            RESERVED_36;
  uint8_t                            RESERVED_37;
  uint8_t                            RESERVED_38;
  uint8_t                            RESERVED_39;
  uint8_t                            RESERVED_40;
  uint8_t                            RESERVED_41;
  uint8_t                            RESERVED_42;
  uint8_t                            RESERVED_43;
  qmi8658c_reg_t                     I2CM_STATUS;
  qmi8658c_reg_t                       STATUSINT;
  qmi8658c_reg_t                         STATUS0;
  qmi8658c_reg_t                         STATUS1;
  qmi8658c_reg_t                     TIMESTAMP_L;
  qmi8658c_reg_t                     TIMESTAMP_M;
  qmi8658c_reg_t                     TIMESTAMP_H;
  qmi8658c_reg_t                          TEMP_L;
  qmi8658c_reg_t                          TEMP_H;
  qmi8658c_reg_t                            AX_L;
  qmi8658c_reg_t                            AX_H; 
  qmi8658c_reg_t                            AY_L;
  qmi8658c_reg_t                            AY_H;
  qmi8658c_reg_t                            AZ_L;
  qmi8658c_reg_t                            AZ_H;
  qmi8658c_reg_t                            GX_L;
  qmi8658c_reg_t                            GX_H;
  qmi8658c_reg_t                            GY_L;
  qmi8658c_reg_t                            GY_H;
  qmi8658c_reg_t                            GZ_L;
  qmi8658c_reg_t                            GZ_H;
  qmi8658c_reg_t                            MX_L;
  qmi8658c_reg_t                            MX_H;
  qmi8658c_reg_t                            MY_L;
  qmi8658c_reg_t                            MY_H;
  qmi8658c_reg_t                            MZ_L;
  qmi8658c_reg_t                            MZ_H;
  uint8_t                            RESERVED_71;
  uint8_t                            RESERVED_72;
  qmi8658c_reg_t                           DQW_L;
  qmi8658c_reg_t                           DQW_H;
  qmi8658c_reg_t                           DQX_L;
  qmi8658c_reg_t                           DQX_H;
  qmi8658c_reg_t                           DQY_L;
  qmi8658c_reg_t                           DQY_H;
  qmi8658c_reg_t                           DQZ_L;
  qmi8658c_reg_t                           DQZ_H;
  qmi8658c_reg_t                           DVX_L;
  qmi8658c_reg_t                           DVX_H;
  qmi8658c_reg_t                           DVY_L;
  qmi8658c_reg_t                           DVY_H;
  qmi8658c_reg_t                           DVZ_L;
  qmi8658c_reg_t                           DVZ_H; 
  qmi8658c_reg_t                         AE_REG1;
  qmi8658c_reg_t                         AE_REG2;
  
} QMI8658C_Reg_TypeDef;

/*IMU驱动结构体*/
typedef struct __IMU_Drv_Typedef 
{
    IMU_Status_Typedef   IMUStatus;

    IMU_Info_Typedef     IMUInfo;
    
    qmidev_ctx_t         Dev_Ctx;

    uint8_t              CurrentAccelerometerFullScale;
    uint8_t              CurrentAccelerometerDataRate;

    uint8_t              CurrentGyroscopeSleepMode;
    uint8_t              CurrentGyroscopeFullScale;
    uint8_t              CurrentGyroscopeDataRate; 
    uint8_t              CurrentGyroscopeHighPerformentMode;                                

    float                Accl_Sensitivity;
    float                Gyro_Sensitivity;
    
    IMU_DATA_PATCH_MODE_Typedef  IMUDataPatchMode;
    
    QMI8658C_Reg_TypeDef Reg;
    
    IMU_Data_Typedef     *p_Rx_Mem;
  
    uint16_t             Rx_Buf_Cell_Cnt;
    uint8_t              Rx_Buf_Cell_Size;
    uint32_t             Rx_Buf_Byte_Size;
  
    Triple_Index_Ring_Buffer_Typedef  Rx_Ring_Buffer;

    bool(*Get_IMU_View_Data)(void);

    bool(*Start_IMU_Accl)(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode);

    bool(*Stop_IMU_Accl)(void);

    bool(*Start_IMU_Gyro)(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode);

    bool(*Stop_IMU_Gyro)(void);

    bool(*Reset_IMU)(void);

    bool(*Read_IMU_Register)(uint8_t reg_addr, uint32_t time_out_ms);

    bool(*Write_IMU_Register)(uint8_t reg_addr, uint8_t reg_data);

    bool(*Get_IMU_Info)(uint32_t time_out_ms);

} IMU_Drv_Typedef;

/**************************************************************************************************************************************************/

extern IMU_Drv_Typedef IMUDrv;

extern IMU_Data_Typedef IMU_Data;

void IMU_Drv_Init(void);

void IMU_Info_Init(void);

void IMU_Func_Init(void);

void Triple_Index_Ring_Buffer_Init(void);

static bool Triple_Index_Ring_Enqueue(IMU_Data_Typedef *p_data, Triple_Index_Ring_Buffer_Typedef *p_ring);

static IMU_Data_Typedef *Triple_Index_Ring_Dequeue(Triple_Index_Ring_Buffer_Typedef *p_ring);

static IMU_Data_Typedef *Triple_Index_Ring_Fake_Dequeue(Triple_Index_Ring_Buffer_Typedef *p_ring);

IMU_Data_Typedef *Triple_Index_Ring_View_Window_Dequeue(Triple_Index_Ring_Buffer_Typedef *p_ring);

static bool __Get_IMU_View_Data (void);

static bool __Start_IMU_Accl(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode);

static bool __Stop_IMU_Accl(void);

static bool __Start_IMU_Gyro(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode);

static bool __Stop_IMU_Gyro(void);

static bool __Reset_IMU(void);

static bool __Read_IMU_Register(uint8_t reg_addr, uint32_t time_out_ms);

static bool __Write_IMU_Register(uint8_t reg_addr, uint8_t reg_data);

static bool __Get_IMU_Info(uint32_t time_out_ms);

static void Cache_IMU_Accl_Data_Set(IMU_Accl_Data_Set_Typedef *data_set);

static void Cache_IMU_Gyro_Data_Set(IMU_Gyro_Data_Set_Typedef *data_set);

static void Cache_IMU_Combine_Data_Set(IMU_Combine_Data_Set_Typedef *data_set);

void QMI8658C_Accl_Data_Process_On_FIFO(void);

void QMI8658C_Gyro_Data_Process_On_FIFO(void);

void QMI8658C_Combine_Data_Process_On_FIFO(void);

static int32_t platform_write(SPI_Type* handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(SPI_Type* handle, uint8_t reg, uint8_t *bufp, uint16_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif /* IMU_CTRL_PROCESS_QST_H */
