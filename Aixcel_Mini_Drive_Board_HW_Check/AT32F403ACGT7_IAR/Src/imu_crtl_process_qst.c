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

#include "imu_crtl_process_qst.h"
#include "hwconf.h"

#if (IMU_MANUFACTURE_ID == MANUFACTURE_ID_QST)

#define BOARD_IMU_DEBUG

IMU_Drv_Typedef IMUDrv;

IMU_Data_Typedef IMU_Data = {0};

static IMU_Data_Typedef  Rx_Mem_Region[IMU_RX_BUF_CELL_CNT];

void IMU_Drv_Init(void)
{
    IMU_Info_Init();
    IMU_Func_Init();
}

void IMU_Info_Init(void)
{
    IMUDrv.IMUStatus.AcclState = IMU_IS_STOPPED;
    IMUDrv.IMUStatus.GyroState = IMU_IS_STOPPED;

    memset((char*)&(IMUDrv.IMUInfo), 0, sizeof(IMUDrv.IMUInfo));

    IMUDrv.IMUInfo.MFR = MANUFACTURE_ID_QST;
    IMUDrv.IMUInfo.Type = TRIAXIAL_ACCELEROMETER + TRIAXIAL_GYROSCOPE;
    
    IMUDrv.Dev_Ctx.write_reg = platform_write;
    IMUDrv.Dev_Ctx.read_reg = platform_read;
    IMUDrv.Dev_Ctx.handle = (SPI_Type*)SPI2;

    IMUDrv.CurrentAccelerometerFullScale = QMI8658C_2g;
    IMUDrv.CurrentAccelerometerDataRate = QMI8658C_XL_ODR_8kHz;

    IMUDrv.CurrentGyroscopeSleepMode = 0x00;
    IMUDrv.CurrentGyroscopeFullScale = QMI8658C_16dps;
    IMUDrv.CurrentGyroscopeDataRate = QMI8658C_GY_ODR_8kHz;
    IMUDrv.CurrentGyroscopeHighPerformentMode = 0x01;

    IMUDrv.Accl_Sensitivity = 0.0;
    IMUDrv.Gyro_Sensitivity = 0.0;
    
    IMUDrv.IMUDataPatchMode = FIFO_MODE;
    
    IMUDrv.p_Rx_Mem = (IMU_Data_Typedef*)Rx_Mem_Region;
  
    IMUDrv.Rx_Buf_Cell_Cnt = IMU_RX_BUF_CELL_CNT;
    IMUDrv.Rx_Buf_Cell_Size = IMU_RX_BUF_CELL_SIZE;
    IMUDrv.Rx_Buf_Byte_Size = IMU_RX_BUF_CELL_CNT * IMU_RX_BUF_CELL_SIZE;

    Triple_Index_Ring_Buffer_Init();
}

void IMU_Func_Init(void)
{
    IMUDrv.Get_IMU_View_Data = __Get_IMU_View_Data;

    IMUDrv.Start_IMU_Accl = __Start_IMU_Accl;

    IMUDrv.Stop_IMU_Accl = __Stop_IMU_Accl;

    IMUDrv.Start_IMU_Gyro = __Start_IMU_Gyro;

    IMUDrv.Stop_IMU_Gyro = __Stop_IMU_Gyro;

    IMUDrv.Reset_IMU = __Reset_IMU;

    IMUDrv.Read_IMU_Register = __Read_IMU_Register;

    IMUDrv.Write_IMU_Register = __Write_IMU_Register;

    IMUDrv.Get_IMU_Info = __Get_IMU_Info;
}

void Triple_Index_Ring_Buffer_Init(void)
{
    memset((char*)IMUDrv.p_Rx_Mem, 0, sizeof(IMUDrv.p_Rx_Mem));	

    memset((char*)&(IMUDrv.Rx_Ring_Buffer), 0, sizeof(IMUDrv.Rx_Ring_Buffer));

    IMUDrv.Rx_Ring_Buffer.pBuffer = IMUDrv.p_Rx_Mem;
    IMUDrv.Rx_Ring_Buffer.Head = 0;
    IMUDrv.Rx_Ring_Buffer.Tail = 0;
    IMUDrv.Rx_Ring_Buffer.Fake_Head = 0;
    IMUDrv.Rx_Ring_Buffer.Fake_Tail = 0;
    IMUDrv.Rx_Ring_Buffer.View_Window_Head = 0;
    IMUDrv.Rx_Ring_Buffer.View_Window_Tail = 0;
    IMUDrv.Rx_Ring_Buffer.Buffer_Size = IMUDrv.Rx_Buf_Cell_Cnt - 1;
}

static bool Triple_Index_Ring_Enqueue(IMU_Data_Typedef *p_data, Triple_Index_Ring_Buffer_Typedef *p_ring)
{
    if(((p_ring->Tail+1) == p_ring->Head) || ((p_ring->Tail == p_ring->Buffer_Size) && (p_ring->Head==0)))	
    {
        return false;
    }
    else
    {
        p_ring->pBuffer[p_ring->Tail] = *p_data;
        p_ring->Tail++;					                
        if(p_ring->Tail > p_ring->Buffer_Size)			 			                       
            p_ring->Tail = 0;
        p_ring->Fake_Tail = p_ring->Tail;                //伪索引的tail随同真索引的tail同步更新，注意：反向不成立！！！                            
        return true;
    }
}

static IMU_Data_Typedef *Triple_Index_Ring_Dequeue(Triple_Index_Ring_Buffer_Typedef *p_ring)
{
    IMU_Data_Typedef *p_data;
    if(p_ring->Head == p_ring->Tail)
    {
  	    return NULL;                                        	 
    }
    else
    {
        p_data = &p_ring->pBuffer[p_ring->Head];
        p_ring->Head++;							            	
        if(p_ring->Head > p_ring->Buffer_Size)		
            p_ring->Head = 0;  
        p_ring->View_Window_Head = p_ring->Head;         //滑动窗口的head随同真索引的head同步更新，注意：反向不成立！！！                        
        return p_data;
    }   
}

static IMU_Data_Typedef *Triple_Index_Ring_Fake_Dequeue(Triple_Index_Ring_Buffer_Typedef *p_ring)
{
    IMU_Data_Typedef *p_data;
    if(p_ring->Fake_Head == p_ring->Fake_Tail)
    {
  	    return NULL;                                        	 
    }
    else
    {
        p_data = &p_ring->pBuffer[p_ring->Fake_Head];
        p_ring->Fake_Head++;							            	
        if(p_ring->Fake_Head > p_ring->Buffer_Size)		
            p_ring->Fake_Head = 0;  
        p_ring->View_Window_Tail = p_ring->Fake_Head;    //滑动窗口的tail随同伪索引的head同步更新，注意：反向不成立！！！                        
        return p_data;
    }   
}

IMU_Data_Typedef *Triple_Index_Ring_View_Window_Dequeue(Triple_Index_Ring_Buffer_Typedef *p_ring)
{
    IMU_Data_Typedef *p_data;
    if(p_ring->View_Window_Head == p_ring->View_Window_Tail)
    {
  	    return NULL;                                        	 
    }
    else
    {
        p_data = &p_ring->pBuffer[p_ring->View_Window_Head];
        p_ring->View_Window_Head++;							            	
        if(p_ring->View_Window_Head > p_ring->Buffer_Size)		
            p_ring->View_Window_Head = 0;                        
        return p_data;
    }   
} 

static bool __Get_IMU_View_Data(void)
{
    return true;
}

static bool __Start_IMU_Accl(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode)
{    
    if(IMUDrv.IMUStatus.GyroState == IMU_IS_WORKING)
    {
        /* Restore default configuration */
        static uint8_t rst;
        
        qmi8658c_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            qmi8658c_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
        
        __Start_IMU_Gyro(IMUDrv.CurrentGyroscopeFullScale, IMUDrv.CurrentGyroscopeDataRate, patch_mode);
    }
  
    if(patch_mode == NORMAL_MODE)
    {
        //Set FIFO mode to Bypass mode(Disable Mode)
        qmi8658c_fifo_mode_set(&IMUDrv.Dev_Ctx, QMI8658C_BYPASS_Mode);
        
        //Set full scale
        qmi8658c_xl_full_scale_set(&IMUDrv.Dev_Ctx, (qmi8658c_fs_xl_t)full_scale);
        
        //Set Output Data Rate
        qmi8658c_xl_data_rate_set(&IMUDrv.Dev_Ctx, (qmi8658c_odr_xl_t)date_rate);
       
        /*
         * Configure filtering chain(No aux interface)
         */ 
        /* Accelerometer - analog filter */
        qmi8658c_xl_alpf_en_set(&IMUDrv.Dev_Ctx, QMI8658C_XL_aLPF_Enable);
        
        qmi8658c_xl_en_set(&IMUDrv.Dev_Ctx, QMI8658C_XL_Enable);
        
        qmi8658c_sensor_sta_set(&IMUDrv.Dev_Ctx, QMI8658C_Sen_Enable);
    }    
    else if(patch_mode == FIFO_MODE)
    {
        //Set full scale
        qmi8658c_xl_full_scale_set(&IMUDrv.Dev_Ctx, (qmi8658c_fs_xl_t)full_scale); 
        
        //qmi8658c_reg_t watermark;
        //watermark.fifo_wtm_th.FIFO_WTM = 1;
        
        //因为第二个参数为"1", 因此QMI8658C的FIFO中存储"1份"测量样本就达到了FIFO的WaterMark!!!
        //qmi8658c_fifo_wtm_set(&IMUDrv.Dev_Ctx, watermark);
        
        //qmi8658c_fifo_size_set(&IMUDrv.Dev_Ctx, QMI8658C_FIFO_16Samples);
        
        //Set FIFO mode to Stream mode(Continous Mode)
        //qmi8658c_fifo_mode_set(&IMUDrv.Dev_Ctx, QMI8658C_STREAM_Mode);

        //Set Output Data Rate
        qmi8658c_xl_data_rate_set(&IMUDrv.Dev_Ctx, (qmi8658c_odr_xl_t)date_rate);
        
        /*
         * Configure filtering chain(No aux interface)
         */ 
        /* Accelerometer - analog filter */
        qmi8658c_xl_alpf_en_set(&IMUDrv.Dev_Ctx, QMI8658C_XL_aLPF_Enable);
        
        qmi8658c_xl_en_set(&IMUDrv.Dev_Ctx, QMI8658C_XL_Enable);
        
        qmi8658c_sensor_sta_set(&IMUDrv.Dev_Ctx, QMI8658C_Sen_Enable);
    }
    
    IMUDrv.IMUStatus.AcclState = IMU_IS_WORKING;
    
    IMUDrv.CurrentAccelerometerFullScale = full_scale;
    IMUDrv.CurrentAccelerometerDataRate = date_rate;
    
    IMUDrv.IMUDataPatchMode = patch_mode;
    
    switch(IMUDrv.CurrentAccelerometerFullScale)
    {
        case QMI8658C_2g:
            IMUDrv.Accl_Sensitivity = (1<<14);
            break;
        case QMI8658C_4g:
            IMUDrv.Accl_Sensitivity = (1<<13);
            break;
        case QMI8658C_8g:
            IMUDrv.Accl_Sensitivity = (1<<12);
            break;
        case QMI8658C_16g:
            IMUDrv.Accl_Sensitivity = (1<<11);
            break;
        default:
            IMUDrv.Accl_Sensitivity = (1<<14);
            break;
    } 
    
    return true;
}

static bool __Stop_IMU_Accl(void)
{
    if(IMUDrv.IMUStatus.GyroState != IMU_IS_WORKING)
    {
        /* Restore default configuration */
        static uint8_t rst;
        
        qmi8658c_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            qmi8658c_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
    }
    
    qmi8658c_xl_en_set(&IMUDrv.Dev_Ctx, QMI8658C_XL_Disable);

    IMUDrv.IMUStatus.AcclState = IMU_IS_STOPPED;

    IMUDrv.Accl_Sensitivity = 0.0;

    return true;
}

static bool __Start_IMU_Gyro(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode)
{
    if(IMUDrv.IMUStatus.AcclState == IMU_IS_WORKING)
    {
        /* Restore default configuration */
        static uint8_t rst;
        
        qmi8658c_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            qmi8658c_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
    
        __Start_IMU_Accl(IMUDrv.CurrentAccelerometerFullScale, IMUDrv.CurrentAccelerometerDataRate, patch_mode);
    }
  
    if(patch_mode == NORMAL_MODE)
    {
        //Set FIFO mode to Bypass mode(Disable Mode)
        qmi8658c_fifo_mode_set(&IMUDrv.Dev_Ctx, QMI8658C_BYPASS_Mode);
        
        //Set full scale
        qmi8658c_gy_full_scale_set(&IMUDrv.Dev_Ctx, (qmi8658c_fs_gy_t)full_scale);
        
        //Set Output Data Rate
        qmi8658c_gy_data_rate_set(&IMUDrv.Dev_Ctx, (qmi8658c_odr_gy_t)date_rate);
       
        /*
         * Configure filtering chain(No aux interface)
         */   
        /* Gyroscope - filtering chain */
        qmi8658c_gy_alpf_en_set(&IMUDrv.Dev_Ctx, QMI8658C_GY_aLPF_Enable);
        
        qmi8658c_gy_en_set(&IMUDrv.Dev_Ctx, QMI8658C_GY_Enable);
        
        qmi8658c_sensor_sta_set(&IMUDrv.Dev_Ctx, QMI8658C_Sen_Enable);
    }    
    else if(patch_mode == FIFO_MODE)
    {
        //Set full scale
        qmi8658c_gy_full_scale_set(&IMUDrv.Dev_Ctx, (qmi8658c_fs_gy_t)full_scale);
        
        //qmi8658c_reg_t watermark;
        //watermark.fifo_wtm_th.FIFO_WTM = 1;
        
        //因为第二个参数为"1", 因此QMI8658C的FIFO中存储"1份"测量样本就达到了FIFO的WaterMark!!!
        //qmi8658c_fifo_wtm_set(&IMUDrv.Dev_Ctx, watermark);
        
        //qmi8658c_fifo_size_set(&IMUDrv.Dev_Ctx, QMI8658C_FIFO_16Samples);
        
        //Set FIFO mode to Stream mode(Continous Mode)
        //qmi8658c_fifo_mode_set(&IMUDrv.Dev_Ctx, QMI8658C_STREAM_Mode);

        //Set Output Data Rate
        qmi8658c_gy_data_rate_set(&IMUDrv.Dev_Ctx, (qmi8658c_odr_gy_t)date_rate);
        
        /*
         * Configure filtering chain(No aux interface)
         */       
        /* Gyroscope - filtering chain */
        qmi8658c_gy_alpf_en_set(&IMUDrv.Dev_Ctx, QMI8658C_GY_aLPF_Enable);
        
        qmi8658c_gy_en_set(&IMUDrv.Dev_Ctx, QMI8658C_GY_Enable);
        
        qmi8658c_sensor_sta_set(&IMUDrv.Dev_Ctx, QMI8658C_Sen_Enable);
    }
    
    IMUDrv.IMUStatus.GyroState = IMU_IS_WORKING;
    
    IMUDrv.CurrentGyroscopeFullScale = full_scale;
    IMUDrv.CurrentGyroscopeDataRate = date_rate;
    
    IMUDrv.IMUDataPatchMode = patch_mode;
    
    switch(IMUDrv.CurrentGyroscopeFullScale)
    {
        case QMI8658C_16dps:
            IMUDrv.Gyro_Sensitivity = 2048;
            break;
        case QMI8658C_32dps:
            IMUDrv.Gyro_Sensitivity = 1024;
            break;
        case QMI8658C_64dps:
            IMUDrv.Gyro_Sensitivity = 512;
            break;
        case QMI8658C_128dps:
            IMUDrv.Gyro_Sensitivity = 256;
            break;
        case QMI8658C_256dps:
            IMUDrv.Gyro_Sensitivity = 128;
            break;
        case QMI8658C_512dps:
            IMUDrv.Gyro_Sensitivity = 64;
            break;
        case QMI8658C_1024dps:
            IMUDrv.Gyro_Sensitivity = 32;
            break;
        case QMI8658C_2048dps:
            IMUDrv.Gyro_Sensitivity = 16;
            break;
        default:
            IMUDrv.Gyro_Sensitivity = 2048;
            break;
    }
    
    return true;
}

static bool __Stop_IMU_Gyro(void)
{
    if(IMUDrv.IMUStatus.AcclState != IMU_IS_WORKING)
    {
        /* Restore default configuration */
        static uint8_t rst;
        
        qmi8658c_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            qmi8658c_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
    }
    
    qmi8658c_xl_en_set(&IMUDrv.Dev_Ctx, QMI8658C_XL_Disable);
    
    IMUDrv.IMUStatus.GyroState = IMU_IS_STOPPED;
    
    IMUDrv.Gyro_Sensitivity = 0.0;

    return true;
}

static bool __Reset_IMU(void)
{
    /* Restore default configuration */
    static uint8_t rst;
    
    qmi8658c_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
    
    Delay_ms(20);
    
    do {
        qmi8658c_reset_get(&IMUDrv.Dev_Ctx, &rst);
    } while (rst);
    
    qmi8658c_sim_set(&IMUDrv.Dev_Ctx, QMI8658C_SIM_4_Wire_SPI);
    qmi8658c_sip_ai_set(&IMUDrv.Dev_Ctx, QMI8658C_SPI_AI_OFF);
    qmi8658c_spi_be_set(&IMUDrv.Dev_Ctx, QMI8658C_SPI_BE);
    qmi8658c_sensor_sta_set(&IMUDrv.Dev_Ctx, QMI8658C_Sen_Enable);
    
    IMUDrv.IMUStatus.AcclState = IMU_IS_STOPPED;
    IMUDrv.IMUStatus.GyroState = IMU_IS_STOPPED;
    
    IMUDrv.CurrentAccelerometerFullScale = QMI8658C_2g;
    IMUDrv.CurrentAccelerometerDataRate = QMI8658C_XL_ODR_8kHz;

    IMUDrv.CurrentGyroscopeSleepMode = 0x00;
    IMUDrv.CurrentGyroscopeFullScale = QMI8658C_16dps;
    IMUDrv.CurrentGyroscopeDataRate = QMI8658C_GY_ODR_8kHz;
    IMUDrv.CurrentGyroscopeHighPerformentMode = 0x00;

    IMUDrv.IMUDataPatchMode = FIFO_MODE;
    
    IMUDrv.Accl_Sensitivity = 0.0;
    IMUDrv.Gyro_Sensitivity = 0.0;

    return true;
}

static bool __Read_IMU_Register(uint8_t reg_addr, uint32_t time_out_ms)
{
    qmi8658c_read_reg(&IMUDrv.Dev_Ctx, reg_addr, &((uint8_t*)&IMUDrv.Reg)[reg_addr], 1);

    return true;
}

static bool __Write_IMU_Register(uint8_t reg_addr, uint8_t reg_data)
{
    qmi8658c_write_reg(&IMUDrv.Dev_Ctx, reg_addr, (uint8_t*)&reg_data, 1);

    return true;
}

static bool __Get_IMU_Info(uint32_t time_out_ms)
{
    /* Check device ID */
    do {
        qmi8658c_who_am_i_get(&IMUDrv.Dev_Ctx, &IMUDrv.Reg.WHO_AM_I);  
    } while(IMUDrv.Reg.WHO_AM_I != QMI8658C_ID);//QMI8658C_ID 0x05
    
    do {
        qmi8658c_revision_id_get(&IMUDrv.Dev_Ctx, &IMUDrv.Reg.REVISION_ID);  
    } while(0);//QMI8658C_ID 0x79 or 0x7B ?
    
    IMUDrv.IMUInfo.WhoAmI = IMUDrv.Reg.WHO_AM_I;

    printf("IMUDrv.Reg.WHO_AM_I = 0x%X\n", IMUDrv.Reg.WHO_AM_I);
    printf("IMUDrv.Reg.REVISION_ID = 0x%X\n", IMUDrv.Reg.REVISION_ID);
    
    return true;
}

static void Cache_IMU_Accl_Data_Set(IMU_Accl_Data_Set_Typedef *data_set)
{
    if(IMUDrv.IMUStatus.AcclState == IMU_IS_WORKING)
    {
        IMU_Data_Typedef imu_data = {0};
        
        imu_data.AcclX = (*((int16_t*)&(data_set->AcclData[0])))*ONE_G / IMUDrv.Accl_Sensitivity;
        imu_data.AcclY = (*((int16_t*)&(data_set->AcclData[2])))*ONE_G / IMUDrv.Accl_Sensitivity;
        imu_data.AcclZ = (*((int16_t*)&(data_set->AcclData[4])))*ONE_G / IMUDrv.Accl_Sensitivity;

        imu_data.TimeStampUs = *((uint32_t*)(data_set->TimeData));
        
        imu_data.TemperatureDeg = (float)(*((uint16_t*)(data_set->TempData))) / 256.0f;

        Triple_Index_Ring_Enqueue(&imu_data, &(IMUDrv.Rx_Ring_Buffer));
        
        IMU_Data = imu_data;
        
      #ifdef BOARD_IMU_DEBUG  
        static uint32_t print_imu_tick_record = 0;
        if((Systick_Get() - print_imu_tick_record) >= 100)
        { 
            print_imu_tick_record = Systick_Get();  
            printf("A_x=%4.2f\tA_y=%4.2f\tA_z=%4.2f\tTime=%ld us\tTemp=%6.2f\r\n", imu_data.AcclX, imu_data.AcclY, imu_data.AcclZ, imu_data.TimeStampUs, imu_data.TemperatureDeg);
        }
      #endif
    }
}

static void Cache_IMU_Gyro_Data_Set(IMU_Gyro_Data_Set_Typedef *data_set)
{
    if(IMUDrv.IMUStatus.GyroState == IMU_IS_WORKING)
    {
        IMU_Data_Typedef imu_data = {0};

        imu_data.GyroX = (*((int16_t*)&(data_set->GyroData[0])))*1.0f / IMUDrv.Gyro_Sensitivity;
        imu_data.GyroY = (*((int16_t*)&(data_set->GyroData[2])))*1.0f / IMUDrv.Gyro_Sensitivity;
        imu_data.GyroZ = (*((int16_t*)&(data_set->GyroData[4])))*1.0f / IMUDrv.Gyro_Sensitivity;

        imu_data.TimeStampUs = *((uint32_t*)(data_set->TimeData));
        
        imu_data.TemperatureDeg = (float)(*((uint16_t*)(data_set->TempData))) / 256.0f;

        Triple_Index_Ring_Enqueue(&imu_data, &(IMUDrv.Rx_Ring_Buffer));
        
        IMU_Data = imu_data;
        
      #ifdef BOARD_IMU_DEBUG  
        static uint32_t print_imu_tick_record = 0;
        if((Systick_Get() - print_imu_tick_record) >= 100)
        { 
            print_imu_tick_record = Systick_Get();
            printf("G_x=%4.2f\tG_y=%4.2f\tG_z=%4.2f\tTime=%ld us\tTemp=%6.2f\r\n", imu_data.GyroX, imu_data.GyroY, imu_data.GyroZ, imu_data.TimeStampUs, imu_data.TemperatureDeg);
        }
      #endif
    }
}

static void Cache_IMU_Combine_Data_Set(IMU_Combine_Data_Set_Typedef *data_set)
{
    if((IMUDrv.IMUStatus.AcclState == IMU_IS_WORKING) && (IMUDrv.IMUStatus.GyroState == IMU_IS_WORKING))
    {
        IMU_Data_Typedef imu_data = {0};
        
        imu_data.AcclX = (float)(*((int16_t*)&(data_set->AcclData[0])))*ONE_G / IMUDrv.Accl_Sensitivity;
        imu_data.AcclY = (float)(*((int16_t*)&(data_set->AcclData[2])))*ONE_G / IMUDrv.Accl_Sensitivity;
        imu_data.AcclZ = (float)(*((int16_t*)&(data_set->AcclData[4])))*ONE_G / IMUDrv.Accl_Sensitivity;

        imu_data.GyroX = (float)(*((int16_t*)&(data_set->GyroData[0])))*1.0f / IMUDrv.Gyro_Sensitivity;
        imu_data.GyroY = (float)(*((int16_t*)&(data_set->GyroData[2])))*1.0f / IMUDrv.Gyro_Sensitivity;
        imu_data.GyroZ = (float)(*((int16_t*)&(data_set->GyroData[4])))*1.0f / IMUDrv.Gyro_Sensitivity;
        
        imu_data.TimeStampUs = *((uint32_t*)(data_set->TimeData));
        
        imu_data.TemperatureDeg = (float)(*((uint16_t*)(data_set->TempData))) / 256.0f;

        Triple_Index_Ring_Enqueue(&imu_data, &(IMUDrv.Rx_Ring_Buffer));
        
        IMU_Data = imu_data;
        
      #ifdef BOARD_IMU_DEBUG  
        static uint32_t print_imu_tick_record = 0;
        if((Systick_Get() - print_imu_tick_record) >= 100)
        { 
            print_imu_tick_record = Systick_Get();
            printf("G_x=%4.2f\tG_y=%4.2f\tG_z=%4.2f\tTime=%ld us\tTemp=%6.2f\r\n", imu_data.GyroX, imu_data.GyroY, imu_data.GyroZ, imu_data.TimeStampUs, imu_data.TemperatureDeg);  
            printf("A_x=%4.2f\tA_y=%4.2f\tA_z=%4.2f\tTime=%ld us\tTemp=%6.2f\r\n", imu_data.AcclX, imu_data.AcclY, imu_data.AcclZ, imu_data.TimeStampUs, imu_data.TemperatureDeg);
        }
      #endif
    } 
}

void QMI8658C_Accl_Data_Process_On_FIFO(void)
{
    qmi8658c_sen_data_available_sta_t data_sta;
    
    IMU_Accl_Data_Set_Typedef accl_data_set = {0};
    
    //Read QMI8658C sensor data available flag
    qmi8658c_sen_data_available_sta_get(&IMUDrv.Dev_Ctx, &data_sta);
    if(data_sta == QMI8658C_Sensor_Data_Is_Available)
    {
        //Read timestamp
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_TIMESTAMP_L, (uint8_t*)&accl_data_set.TimeData, 3);
      
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_AX_L, (uint8_t*)&accl_data_set.AcclData[0], 6);
        
        //Read temperature data
        qmi8658c_temperature_get(&IMUDrv.Dev_Ctx, (uint8_t*)accl_data_set.TempData);
        
        Cache_IMU_Accl_Data_Set(&accl_data_set);
    }
}

void QMI8658C_Gyro_Data_Process_On_FIFO(void)
{
    qmi8658c_sen_data_available_sta_t data_sta;
    
    IMU_Gyro_Data_Set_Typedef gyro_data_set = {0};
    
    //Read QMI8658C sensor data available flag
    qmi8658c_sen_data_available_sta_get(&IMUDrv.Dev_Ctx, &data_sta);
    if(data_sta == QMI8658C_Sensor_Data_Is_Available)
    {
        //Read timestamp
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_TIMESTAMP_L, (uint8_t*)&gyro_data_set.TimeData, 3);
      
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_GX_L, (uint8_t*)&gyro_data_set.GyroData[0], 6);
      
        //Read temperature data
        qmi8658c_temperature_get(&IMUDrv.Dev_Ctx, (uint8_t*)gyro_data_set.TempData);
        
        Cache_IMU_Gyro_Data_Set(&gyro_data_set);
    }
}

void QMI8658C_Combine_Data_Process_On_FIFO(void)
{
    qmi8658c_sen_data_available_sta_t data_sta;
    
    IMU_Combine_Data_Set_Typedef combine_data_set = {0};
    
    //Read QMI8658C sensor data available flag
    qmi8658c_sen_data_available_sta_get(&IMUDrv.Dev_Ctx, &data_sta);
    if(data_sta == QMI8658C_Sensor_Data_Is_Available)
    {
        //Read timestamp
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_TIMESTAMP_L, (uint8_t*)&combine_data_set.TimeData, 3);
              
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_AX_L, (uint8_t*)&combine_data_set.AcclData[0], 6);
            
        qmi8658c_read_reg(&IMUDrv.Dev_Ctx, QMI8658C_GX_L, (uint8_t*)&combine_data_set.GyroData[0], 6);
      
        //Read temperature data
        qmi8658c_temperature_get(&IMUDrv.Dev_Ctx, (uint8_t*)combine_data_set.TempData);
        
        Cache_IMU_Combine_Data_Set(&combine_data_set);
    }
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. Is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(SPI_Type* handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  if(handle == SPI2) 
  {
      IMU_SELECT();
      SPI_Transmmit_Data(handle, &reg, 1, 1000);
      SPI_Transmmit_Data(handle, bufp, len, 1000);
      IMU_DESELECT();
  }

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. Is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(SPI_Type* handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  if(handle == SPI2) 
  {
      /* Read command */
      reg |= 0x80;
      IMU_SELECT();
      SPI_Transmmit_Data(handle, &reg, 1, 1000);
      SPI_Receive_Data(handle, bufp, len, 1000);
      IMU_DESELECT();
  }

  return 0;
}

#endif
