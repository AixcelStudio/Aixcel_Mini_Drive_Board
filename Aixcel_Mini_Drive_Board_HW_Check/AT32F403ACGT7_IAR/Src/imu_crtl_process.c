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

#include "imu_ctrl_process.h"
#include "hwconf.h"

#if (IMU_MANUFACTURE_ID == MANUFACTURE_ID_ST)

//#define BOARD_IMU_DEBUG

IMU_Drv_Typedef IMUDrv;

IMU_Data_Typedef IMU_Data = {0};

static IMU_Data_Typedef  Rx_Mem_Region[IMU_RX_BUF_CELL_CNT];

void IMU_Drv_Init(void)
{
    IMU_Info_Init();
    
    IMU_Func_Init();
    
    IMUDrv.Get_IMU_Info(0x01);
    
    IMUDrv.Reset_IMU();
}

void IMU_Info_Init(void)
{
    IMUDrv.IMUStatus.AcclState = IMU_IS_STOPPED;
    IMUDrv.IMUStatus.GyroState = IMU_IS_STOPPED;

    memset((char*)&(IMUDrv.IMUInfo), 0, sizeof(IMUDrv.IMUInfo));

    IMUDrv.IMUInfo.MFR = MANUFACTURE_ID_ST;
    IMUDrv.IMUInfo.Type = TRIAXIAL_ACCELEROMETER + TRIAXIAL_GYROSCOPE;
    
    IMUDrv.Dev_Ctx.write_reg = platform_write;
    IMUDrv.Dev_Ctx.read_reg = platform_read;
    IMUDrv.Dev_Ctx.handle = (SPI_Type*)SPI1;

    IMUDrv.CurrentAccelerometerFullScale = LSM6DSL_2g;
    IMUDrv.CurrentAccelerometerDataRate = LSM6DSL_XL_ODR_OFF;

    IMUDrv.CurrentGyroscopeSleepMode = 0x00;
    IMUDrv.CurrentGyroscopeFullScale = LSM6DSL_125dps;
    IMUDrv.CurrentGyroscopeDataRate = LSM6DSL_GY_ODR_OFF;
    IMUDrv.CurrentGyroscopeHighPerformentMode = 0x00;

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
        
        lsm6dsl_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            lsm6dsl_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
        
        __Start_IMU_Gyro(IMUDrv.CurrentGyroscopeFullScale, IMUDrv.CurrentGyroscopeDataRate, patch_mode);
    }
  
    if(patch_mode == NORMAL_MODE)
    {
        //Enable Block Data Update
        lsm6dsl_block_data_update_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        
        //Set High Resolution Timestamp (25 us tick)
        lsm6dsl_timestamp_res_set(&IMUDrv.Dev_Ctx, LSM6DSL_LSB_25us);

        //Enable timestamp in HW
        lsm6dsl_timestamp_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        
        //Set full scale
        lsm6dsl_xl_full_scale_set(&IMUDrv.Dev_Ctx, (lsm6dsl_fs_xl_t)full_scale);
        
        //Set Output Data Rate
        lsm6dsl_xl_data_rate_set(&IMUDrv.Dev_Ctx, (lsm6dsl_odr_xl_t)date_rate);
       
        /*
         * Configure filtering chain(No aux interface)
         */ 
        /* Accelerometer - analog filter */
        lsm6dsl_xl_filter_analog_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_ANA_BW_400Hz);
       
        /* Accelerometer - LPF1 path ( LPF2 not used )*/
        //lsm6dsl_xl_lp1_bandwidth_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
       
        /* Accelerometer - LPF1 + LPF2 path */  
        lsm6dsl_xl_lp2_bandwidth_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
       
        /* Accelerometer - High Pass / Slope path */
        //lsm6dsl_xl_reference_mode_set(&IMUDrv.Dev_Ctx, PROPERTY_DISABLE);
        //lsm6dsl_xl_hp_bandwidth_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_HP_ODR_DIV_100);
    }    
    else if(patch_mode == FIFO_MODE)
    {
        lsm6dsl_int1_route_t int_1_reg;
        
        //Enable Block Data Update
        lsm6dsl_block_data_update_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);

        //Set full scale
        lsm6dsl_xl_full_scale_set(&IMUDrv.Dev_Ctx, (lsm6dsl_fs_xl_t)full_scale);        
        
        uint8_t pattern_len = 0;
        
        if(IMUDrv.IMUStatus.GyroState == IMU_IS_WORKING)
        {
            pattern_len = 6;
        }
        else
        {
            pattern_len = 3;
        }
        
        //因为第二个参数为"1 * pattern_len", 因此LSM6DSL的FIFO中存储"1份"测量样本就达到了FIFO的WaterMark!!!
        //(如果只启动A传感器或者G传感器 那么"一份"就代表3*2个字节,如果同时启动A传感器和G传感器 那么"一份"就代表3*2*2个字节)!!!
        lsm6dsl_fifo_watermark_set(&IMUDrv.Dev_Ctx, 1 * pattern_len);
        
        //Set FIFO mode to Stream mode(Continous Mode)
        lsm6dsl_fifo_mode_set(&IMUDrv.Dev_Ctx, LSM6DSL_STREAM_MODE);      

        //Enable FIFO watermark interrupt generation on INT1 pin
        lsm6dsl_pin_int1_route_get(&IMUDrv.Dev_Ctx, &int_1_reg);
        int_1_reg.int1_fth = PROPERTY_ENABLE;
        lsm6dsl_pin_int1_route_set(&IMUDrv.Dev_Ctx, int_1_reg);
        
        //Set FIFO sensor decimator
        lsm6dsl_fifo_xl_batch_set(&IMUDrv.Dev_Ctx, LSM6DSL_FIFO_XL_NO_DEC);

        //Set ODR FIFO
        lsm6dsl_odr_fifo_t odr_fifo = LSM6DSL_FIFO_DISABLE;
        
        switch((lsm6dsl_odr_xl_t)date_rate)
        {
            case LSM6DSL_XL_ODR_OFF:
              odr_fifo = LSM6DSL_FIFO_DISABLE;
              break;
            case LSM6DSL_XL_ODR_12Hz5:
              odr_fifo = LSM6DSL_FIFO_12Hz5;
              break;
            case LSM6DSL_XL_ODR_26Hz:
              odr_fifo = LSM6DSL_FIFO_26Hz;
              break;
            case LSM6DSL_XL_ODR_52Hz:
              odr_fifo = LSM6DSL_FIFO_52Hz;
              break;
            case LSM6DSL_XL_ODR_104Hz:
              odr_fifo = LSM6DSL_FIFO_104Hz;
              break;
            case LSM6DSL_XL_ODR_208Hz:
              odr_fifo = LSM6DSL_FIFO_208Hz;
              break;
            case LSM6DSL_XL_ODR_416Hz:
              odr_fifo = LSM6DSL_FIFO_416Hz;
              break;
            case LSM6DSL_XL_ODR_833Hz:
              odr_fifo = LSM6DSL_FIFO_833Hz;
              break;
            case LSM6DSL_XL_ODR_1k66Hz:
              odr_fifo = LSM6DSL_FIFO_1k66Hz;
              break;
            case LSM6DSL_XL_ODR_3k33Hz:
              odr_fifo = LSM6DSL_FIFO_3k33Hz;
              break;
            case LSM6DSL_XL_ODR_6k66Hz:
              odr_fifo = LSM6DSL_FIFO_6k66Hz;
              break;
            default:
              odr_fifo = LSM6DSL_FIFO_DISABLE;
              break;
        }
        
        lsm6dsl_fifo_data_rate_set(&IMUDrv.Dev_Ctx, odr_fifo);

        //Set High Resolution Timestamp (25 us tick)
        lsm6dsl_timestamp_res_set(&IMUDrv.Dev_Ctx, LSM6DSL_LSB_25us);

        //Enable timestamp in HW
        lsm6dsl_timestamp_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);

        //Set Output Data Rate for acc/gyro
        lsm6dsl_xl_data_rate_set(&IMUDrv.Dev_Ctx, (lsm6dsl_odr_xl_t)date_rate);
        
        /*
         * Configure filtering chain(No aux interface)
         */ 
        /* Accelerometer - analog filter */
        lsm6dsl_xl_filter_analog_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_ANA_BW_400Hz);
       
        /* Accelerometer - LPF1 path ( LPF2 not used )*/
        //lsm6dsl_xl_lp1_bandwidth_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
       
        /* Accelerometer - LPF1 + LPF2 path */  
        lsm6dsl_xl_lp2_bandwidth_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
       
        /* Accelerometer - High Pass / Slope path */
        //lsm6dsl_xl_reference_mode_set(&IMUDrv.Dev_Ctx, PROPERTY_DISABLE);
        //lsm6dsl_xl_hp_bandwidth_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_HP_ODR_DIV_100);
    }
    
    IMUDrv.IMUStatus.AcclState = IMU_IS_WORKING;
    
    IMUDrv.CurrentAccelerometerFullScale = full_scale;
    IMUDrv.CurrentAccelerometerDataRate = date_rate;
    
    IMUDrv.IMUDataPatchMode = patch_mode;
    
    switch(IMUDrv.CurrentAccelerometerFullScale)
    {
        case LSM6DSL_2g:
            IMUDrv.Accl_Sensitivity = 0.061;
            break;
        case LSM6DSL_4g:
            IMUDrv.Accl_Sensitivity = 0.122;
            break;
        case LSM6DSL_8g:
            IMUDrv.Accl_Sensitivity = 0.244;
            break;
        case LSM6DSL_16g:
            IMUDrv.Accl_Sensitivity = 0.488;
            break;
        default:
            IMUDrv.Accl_Sensitivity = 0.0;
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
        
        lsm6dsl_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            lsm6dsl_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
        
        //Disable timestamp in HW
        lsm6dsl_timestamp_set(&IMUDrv.Dev_Ctx, PROPERTY_DISABLE);
        lsm6dsl_fifo_data_rate_set(&IMUDrv.Dev_Ctx, LSM6DSL_FIFO_DISABLE);
    }

    //Set Output Data Rate for acc/gyro to off
    lsm6dsl_xl_data_rate_set(&IMUDrv.Dev_Ctx, LSM6DSL_XL_ODR_OFF);

    IMUDrv.IMUStatus.AcclState = IMU_IS_STOPPED;
    
    IMUDrv.CurrentAccelerometerDataRate = LSM6DSL_XL_ODR_OFF;

    IMUDrv.Accl_Sensitivity = 0.0;

    return true;
}

static bool __Start_IMU_Gyro(uint8_t full_scale, uint8_t date_rate, IMU_DATA_PATCH_MODE_Typedef patch_mode)
{
    if(IMUDrv.IMUStatus.AcclState == IMU_IS_WORKING)
    {
        /* Restore default configuration */
        static uint8_t rst;
        
        lsm6dsl_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            lsm6dsl_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
    
        __Start_IMU_Accl(IMUDrv.CurrentAccelerometerFullScale, IMUDrv.CurrentAccelerometerDataRate, patch_mode);
    }
  
    if(patch_mode == NORMAL_MODE)
    {
        //Enable Block Data Update
        lsm6dsl_block_data_update_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        
        //Set High Resolution Timestamp (25 us tick)
        lsm6dsl_timestamp_res_set(&IMUDrv.Dev_Ctx, LSM6DSL_LSB_25us);

        //Enable timestamp in HW
        lsm6dsl_timestamp_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        
        //Set full scale
        lsm6dsl_gy_full_scale_set(&IMUDrv.Dev_Ctx, (lsm6dsl_fs_g_t)full_scale);
        
        //Set Output Data Rate
        lsm6dsl_gy_data_rate_set(&IMUDrv.Dev_Ctx, (lsm6dsl_odr_g_t)date_rate);
       
        /*
         * Configure filtering chain(No aux interface)
         */   
        /* Gyroscope - filtering chain */
        lsm6dsl_gy_band_pass_set(&IMUDrv.Dev_Ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
    }    
    else if(patch_mode == FIFO_MODE)
    {
        lsm6dsl_int1_route_t int_1_reg;
        
        //Enable Block Data Update
        lsm6dsl_block_data_update_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);

        //Set full scale
        lsm6dsl_gy_full_scale_set(&IMUDrv.Dev_Ctx, (lsm6dsl_fs_g_t)full_scale);
        
        uint8_t pattern_len = 0;
        
        if(IMUDrv.IMUStatus.AcclState == IMU_IS_WORKING)
        {
            pattern_len = 6;
        }
        else
        {
            pattern_len = 3;
        }
        
        //因为第二个参数为"1 * pattern_len", 因此LSM6DSL的FIFO中存储"1份"测量样本就达到了FIFO的WaterMark!!!
        //(如果只启动A传感器或者G传感器 那么"一份"就代表3*2个字节,如果同时启动A传感器和G传感器 那么"一份"就代表3*2*2个字节)!!!
        lsm6dsl_fifo_watermark_set(&IMUDrv.Dev_Ctx, 1 * pattern_len);
        
        //Set FIFO mode to Stream mode(Continous Mode)
        lsm6dsl_fifo_mode_set(&IMUDrv.Dev_Ctx, LSM6DSL_STREAM_MODE);

        //Enable FIFO watermark interrupt generation on INT1 pin
        lsm6dsl_pin_int1_route_get(&IMUDrv.Dev_Ctx, &int_1_reg);
        int_1_reg.int1_fth = PROPERTY_ENABLE;
        lsm6dsl_pin_int1_route_set(&IMUDrv.Dev_Ctx, int_1_reg);
        
        //Set FIFO sensor decimator
        lsm6dsl_fifo_gy_batch_set(&IMUDrv.Dev_Ctx, LSM6DSL_FIFO_GY_NO_DEC);

        //Set ODR FIFO
        lsm6dsl_odr_fifo_t odr_fifo = LSM6DSL_FIFO_DISABLE;
        
        switch((lsm6dsl_odr_g_t)date_rate)
        {
            case LSM6DSL_GY_ODR_OFF:
              odr_fifo = LSM6DSL_FIFO_DISABLE;
              break;
            case LSM6DSL_GY_ODR_12Hz5:
              odr_fifo = LSM6DSL_FIFO_12Hz5;
              break;
            case LSM6DSL_GY_ODR_26Hz:
              odr_fifo = LSM6DSL_FIFO_26Hz;
              break;
            case LSM6DSL_GY_ODR_52Hz:
              odr_fifo = LSM6DSL_FIFO_52Hz;
              break;
            case LSM6DSL_GY_ODR_104Hz:
              odr_fifo = LSM6DSL_FIFO_104Hz;
              break;
            case LSM6DSL_GY_ODR_208Hz:
              odr_fifo = LSM6DSL_FIFO_208Hz;
              break;
            case LSM6DSL_GY_ODR_416Hz:
              odr_fifo = LSM6DSL_FIFO_416Hz;
              break;
            case LSM6DSL_GY_ODR_833Hz:
              odr_fifo = LSM6DSL_FIFO_833Hz;
              break;
            case LSM6DSL_GY_ODR_1k66Hz:
              odr_fifo = LSM6DSL_FIFO_1k66Hz;
              break;
            case LSM6DSL_GY_ODR_3k33Hz:
              odr_fifo = LSM6DSL_FIFO_3k33Hz;
              break;
            case LSM6DSL_GY_ODR_6k66Hz:
              odr_fifo = LSM6DSL_FIFO_6k66Hz;
              break;
            default:
              odr_fifo = LSM6DSL_FIFO_DISABLE;
              break;
        }
        
        lsm6dsl_fifo_data_rate_set(&IMUDrv.Dev_Ctx, odr_fifo);

        //Set High Resolution Timestamp (25 us tick)
        lsm6dsl_timestamp_res_set(&IMUDrv.Dev_Ctx, LSM6DSL_LSB_25us);

        //Enable timestamp in HW
        lsm6dsl_timestamp_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);

        //Set Output Data Rate for acc/gyro to 1666 Hz
        lsm6dsl_gy_data_rate_set(&IMUDrv.Dev_Ctx, (lsm6dsl_odr_g_t)date_rate);
        
        /*
         * Configure filtering chain(No aux interface)
         */       
        /* Gyroscope - filtering chain */
        lsm6dsl_gy_band_pass_set(&IMUDrv.Dev_Ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
    }
    
    IMUDrv.IMUStatus.GyroState = IMU_IS_WORKING;
    
    IMUDrv.CurrentGyroscopeFullScale = full_scale;
    IMUDrv.CurrentGyroscopeDataRate = date_rate;
    
    IMUDrv.IMUDataPatchMode = patch_mode;
    
    switch(IMUDrv.CurrentGyroscopeFullScale)
    {
        case LSM6DSL_125dps:
            IMUDrv.Gyro_Sensitivity = 4.375;
            break;
        case LSM6DSL_250dps:
            IMUDrv.Gyro_Sensitivity = 8.75;
            break;
        case LSM6DSL_500dps:
            IMUDrv.Gyro_Sensitivity = 17.5;
            break;
        case LSM6DSL_1000dps:
            IMUDrv.Gyro_Sensitivity = 35.0;
            break;
        case LSM6DSL_2000dps:
            IMUDrv.Gyro_Sensitivity = 70.0;
            break;
        default:
            IMUDrv.Gyro_Sensitivity = 0.0;
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
        
        lsm6dsl_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
        do {
            lsm6dsl_reset_get(&IMUDrv.Dev_Ctx, &rst);
        } while (rst);
        
        //Disable timestamp in HW
        lsm6dsl_timestamp_set(&IMUDrv.Dev_Ctx, PROPERTY_DISABLE);
        lsm6dsl_fifo_data_rate_set(&IMUDrv.Dev_Ctx, LSM6DSL_FIFO_DISABLE);
    }

    //Set Output Data Rate for acc/gyro to off
    lsm6dsl_gy_data_rate_set(&IMUDrv.Dev_Ctx, LSM6DSL_GY_ODR_OFF);
    
    IMUDrv.IMUStatus.GyroState = IMU_IS_STOPPED;
    
    IMUDrv.CurrentGyroscopeDataRate = LSM6DSL_GY_ODR_OFF;
    
    IMUDrv.Gyro_Sensitivity = 0.0;

    return true;
}

static bool __Reset_IMU(void)
{
    /* Restore default configuration */
    static uint8_t rst;
    
    lsm6dsl_reset_set(&IMUDrv.Dev_Ctx, PROPERTY_ENABLE);
    do {
        lsm6dsl_reset_get(&IMUDrv.Dev_Ctx, &rst);
    } while (rst);
    
    IMUDrv.IMUStatus.AcclState = IMU_IS_STOPPED;
    IMUDrv.IMUStatus.GyroState = IMU_IS_STOPPED;
    
    IMUDrv.CurrentAccelerometerFullScale = LSM6DSL_2g;
    IMUDrv.CurrentAccelerometerDataRate = LSM6DSL_XL_ODR_OFF;

    IMUDrv.CurrentGyroscopeSleepMode = 0x00;
    IMUDrv.CurrentGyroscopeFullScale = LSM6DSL_125dps;
    IMUDrv.CurrentGyroscopeDataRate = LSM6DSL_GY_ODR_OFF;
    IMUDrv.CurrentGyroscopeHighPerformentMode = 0x00;

    IMUDrv.IMUDataPatchMode = FIFO_MODE;
    
    IMUDrv.Accl_Sensitivity = 0.0;
    IMUDrv.Gyro_Sensitivity = 0.0;

    return true;
}

static bool __Read_IMU_Register(uint8_t reg_addr, uint32_t time_out_ms)
{
    lsm6dsl_read_reg(&IMUDrv.Dev_Ctx, reg_addr, &((uint8_t*)&IMUDrv.Reg)[reg_addr], 1);

    return true;
}

static bool __Write_IMU_Register(uint8_t reg_addr, uint8_t reg_data)
{
    lsm6dsl_write_reg(&IMUDrv.Dev_Ctx, reg_addr, (uint8_t*)&reg_data, 1);

    return true;
}

static bool __Get_IMU_Info(uint32_t time_out_ms)
{
    /* Check device ID */
    do {
        lsm6dsl_device_id_get(&IMUDrv.Dev_Ctx, &IMUDrv.Reg.WHO_AM_I);  
    } while(IMUDrv.Reg.WHO_AM_I != LSM6DSL_ID);//while(IMUDrv.Reg.WHO_AM_I != LSM6DSL_ID); /*manage here device not found */LSM6DSL id Ox6A, LSM6DS3 id Ox69,
    
    IMUDrv.IMUInfo.WhoAmI = IMUDrv.Reg.WHO_AM_I;

   #ifdef BOARD_IMU_DEBUG  
     printf("IMUDrv.IMUInfo.WhoAmI = 0x%X\n", IMUDrv.IMUInfo.WhoAmI);
   #endif
     
    return true;
}

static void Cache_IMU_Accl_Data_Set(IMU_Accl_Data_Set_Typedef *data_set)
{
    if(IMUDrv.IMUStatus.AcclState == IMU_IS_WORKING)
    {
        IMU_Data_Typedef imu_data = {0};
        
        imu_data.AcclX = *((int16_t*)&(data_set->AcclData[0])) * IMUDrv.Accl_Sensitivity;
        imu_data.AcclY = *((int16_t*)&(data_set->AcclData[2])) * IMUDrv.Accl_Sensitivity;
        imu_data.AcclZ = *((int16_t*)&(data_set->AcclData[4])) * IMUDrv.Accl_Sensitivity;

        imu_data.TimeStampUs = *((uint32_t*)(data_set->TimeData)) * 25;
        imu_data.TemperatureDeg = (float)(*((uint16_t*)(data_set->TempData))) / 256.0f + 25.0f;

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

        imu_data.GyroX = *((int16_t*)&(data_set->GyroData[0])) * IMUDrv.Gyro_Sensitivity / 1000.0;
        imu_data.GyroY = *((int16_t*)&(data_set->GyroData[2])) * IMUDrv.Gyro_Sensitivity / 1000.0;
        imu_data.GyroZ = *((int16_t*)&(data_set->GyroData[4])) * IMUDrv.Gyro_Sensitivity / 1000.0;

        imu_data.TimeStampUs = *((uint32_t*)(data_set->TimeData)) * 25;
        imu_data.TemperatureDeg = (float)(*((uint16_t*)(data_set->TempData))) / 256.0f + 25.0f;

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
        
        imu_data.AcclX = (float)(*((int16_t*)&(data_set->AcclData[0]))) * IMUDrv.Accl_Sensitivity;
        imu_data.AcclY = (float)(*((int16_t*)&(data_set->AcclData[2]))) * IMUDrv.Accl_Sensitivity;
        imu_data.AcclZ = (float)(*((int16_t*)&(data_set->AcclData[4]))) * IMUDrv.Accl_Sensitivity;

        imu_data.GyroX = (float)(*((int16_t*)&(data_set->GyroData[0]))) * IMUDrv.Gyro_Sensitivity / 1000.0;
        imu_data.GyroY = (float)(*((int16_t*)&(data_set->GyroData[2]))) * IMUDrv.Gyro_Sensitivity / 1000.0;
        imu_data.GyroZ = (float)(*((int16_t*)&(data_set->GyroData[4]))) * IMUDrv.Gyro_Sensitivity / 1000.0;

        imu_data.TimeStampUs = *((uint32_t*)(data_set->TimeData)) * 25;
        imu_data.TemperatureDeg = (float)(*((uint16_t*)(data_set->TempData))) / 256.0f + 25.0f;

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

void LSM6DSL_Accl_Data_Process_On_FIFO(void)
{
    static uint8_t waterm = 0;
    static uint16_t num = 0;
    static uint16_t pattern = 0; 
    static uint8_t temperature[2] = {0};
  
    waterm = 0;
    num = 0;
    pattern = 0;   
    
    IMU_Accl_Data_Set_Typedef accl_data_set = {0};
    
    //Read LSM6DSL watermark flag
    lsm6dsl_fifo_wtm_flag_get(&IMUDrv.Dev_Ctx, &waterm);
    if (waterm)
    {
        //Read timestamp
        lsm6dsl_read_reg(&IMUDrv.Dev_Ctx,
                         LSM6DSL_TIMESTAMP0_REG,
                         (uint8_t*)&accl_data_set.TimeData, 3);
      
        //Read number of word in FIFO
        lsm6dsl_fifo_data_level_get(&IMUDrv.Dev_Ctx, &num);
      
        while (num-- > 0)
        {
            lsm6dsl_fifo_pattern_get(&IMUDrv.Dev_Ctx, &pattern);
          
            switch(pattern)
            {
              case 0:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&accl_data_set.AcclData[0], 2);
                break;
              case 1:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&accl_data_set.AcclData[2], 2);
                break;
              case 2:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&accl_data_set.AcclData[4], 2);
                break;
              default:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, &IMUDrv.Reg.RESERVED_43, 2);
                break;         
            }
        }
      
        lsm6dsl_reg_t reg;
        lsm6dsl_status_reg_get(&IMUDrv.Dev_Ctx, &reg.status_reg);
        
        if(reg.status_reg.tda)
        {  
            //Read temperature data
            lsm6dsl_temperature_raw_get(&IMUDrv.Dev_Ctx, temperature);
        }
        accl_data_set.TempData[0] = temperature[0];
        accl_data_set.TempData[1] = temperature[1];
        
        Cache_IMU_Accl_Data_Set(&accl_data_set);
    }
}

void LSM6DSL_Gyro_Data_Process_On_FIFO(void)
{
    static uint8_t waterm = 0;
    static uint16_t num = 0;
    static uint16_t pattern = 0; 
    static uint8_t temperature[2] = {0};
    
    waterm = 0;
    num = 0;
    pattern = 0;   
    
    IMU_Gyro_Data_Set_Typedef gyro_data_set = {0};
    
    //Read LSM6DSL watermark flag
    lsm6dsl_fifo_wtm_flag_get(&IMUDrv.Dev_Ctx, &waterm);
    if (waterm)
    {
        //Read timestamp
        lsm6dsl_read_reg(&IMUDrv.Dev_Ctx,
                         LSM6DSL_TIMESTAMP0_REG,
                         (uint8_t*)&gyro_data_set.TimeData, 3);
      
        //Read number of word in FIFO
        lsm6dsl_fifo_data_level_get(&IMUDrv.Dev_Ctx, &num);
      
        while (num-- > 0)
        {
            lsm6dsl_fifo_pattern_get(&IMUDrv.Dev_Ctx, &pattern);
          
            switch(pattern)
            {
              case 0:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&gyro_data_set.GyroData[0], 2);
                break;
              case 1:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&gyro_data_set.GyroData[2], 2);
                break;
             case 2:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&gyro_data_set.GyroData[4], 2);
                break;
              default:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, &IMUDrv.Reg.RESERVED_43, 2);
                break;         
            }
        }
      
        lsm6dsl_reg_t reg;
        lsm6dsl_status_reg_get(&IMUDrv.Dev_Ctx, &reg.status_reg);
        
        if(reg.status_reg.tda)
        {  
            //Read temperature data
            lsm6dsl_temperature_raw_get(&IMUDrv.Dev_Ctx, temperature);
        }
        gyro_data_set.TempData[0] = temperature[0];
        gyro_data_set.TempData[1] = temperature[1];
        
        Cache_IMU_Gyro_Data_Set(&gyro_data_set);
    }
}

void LSM6DSL_Combine_Data_Process_On_FIFO(void)
{
    static uint8_t waterm = 0;
    static uint16_t num = 0;
    static uint16_t pattern = 0; 
    static uint8_t temperature[2] = {0};
    
    waterm = 0;
    num = 0;
    pattern = 0;   
    
    IMU_Combine_Data_Set_Typedef combine_data_set = {0};
    
    //Read LSM6DSL watermark flag
    lsm6dsl_fifo_wtm_flag_get(&IMUDrv.Dev_Ctx, &waterm);
    if(waterm)
    {
        //Read timestamp
        lsm6dsl_read_reg(&IMUDrv.Dev_Ctx,
                         LSM6DSL_TIMESTAMP0_REG,
                         (uint8_t*)&combine_data_set.TimeData, 3);
      
        //Read number of word in FIFO
        lsm6dsl_fifo_data_level_get(&IMUDrv.Dev_Ctx, &num);
      
        while(num-- > 0)
        {
            lsm6dsl_fifo_pattern_get(&IMUDrv.Dev_Ctx, &pattern);
          
            switch(pattern)
            {
              case 0:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&combine_data_set.GyroData[0], 2);
                break;
              case 1:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&combine_data_set.GyroData[2], 2);
                break;
             case 2:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&combine_data_set.GyroData[4], 2);
                break;
              case 3:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&combine_data_set.AcclData[0], 2);
                break;
              case 4:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&combine_data_set.AcclData[2], 2);
                break;
              case 5:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, (uint8_t*)&combine_data_set.AcclData[4], 2);
                break;
              default:
                lsm6dsl_fifo_raw_data_get(&IMUDrv.Dev_Ctx, &IMUDrv.Reg.RESERVED_43, 2);
                break;         
            }
        }
      
        lsm6dsl_reg_t reg;
        lsm6dsl_status_reg_get(&IMUDrv.Dev_Ctx, &reg.status_reg);
        
        if(reg.status_reg.tda)
        {  
            //Read temperature data
            lsm6dsl_temperature_raw_get(&IMUDrv.Dev_Ctx, temperature);
        }
        combine_data_set.TempData[0] = temperature[0];
        combine_data_set.TempData[1] = temperature[1];
        
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
  if(handle == SPI1) 
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
  if(handle == SPI1) 
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
