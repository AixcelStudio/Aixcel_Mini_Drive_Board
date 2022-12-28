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

#include "dac_ctrl_process.h"

#define MAX_SINE_POINT_NUM    1000

const double PI = 3.1415926535897932;

static uint32_t Sine_Wave_Point = 0;

static uint16_t Sine_Wave_Array[MAX_SINE_POINT_NUM];

bool Set_DAC_Out1_Param(uint32_t freq, uint32_t peak_ampl, uint32_t trough_ampl, uint32_t horizon_phase_shift)  //freq: 15k~500k, ampl: 0~4095
{
    if((freq == 0) || (peak_ampl > 4095) || (trough_ampl > 4095) || (peak_ampl < trough_ampl))
        return false;
  
    float relative_ampl = (peak_ampl - trough_ampl)/2;
    
    uint32_t vertical_phase_shift = peak_ampl - ((peak_ampl - trough_ampl)/2) + 1;
    
    Sine_Wave_Point = 120000000/8/freq;  //TMR6's frep is 120MHz, but TMR_SetAutoreload(TMR6, 7); so, trigger freq 120MHz / 8 !!!
    
    if(Sine_Wave_Point > MAX_SINE_POINT_NUM)
        return false;
    
    for(uint32_t i = 0; i < Sine_Wave_Point; i++)
    {
        Sine_Wave_Array[i] = (uint16_t)(relative_ampl*sin(PI*2*i/Sine_Wave_Point) + vertical_phase_shift);
    }
    
    /* DAC OUT1 DMA Config */
    DMA_InitType DMA_InitStruct;
    
    /* clear all the interrupt flags */
    DMA_ClearFlag(DMA2_FLAG_GL3);
    DMA_ClearFlag(DMA2_FLAG_TC3);
    DMA_ClearFlag(DMA2_FLAG_HT3);
    DMA_ClearFlag(DMA2_FLAG_ERR3);
    
    /* configure the DMA1 channel 2 */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)Sine_Wave_Array;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = Sine_Wave_Point;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->HDR12R1);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_CIRCULAR;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA2_Channel3, &DMA_InitStruct);
    
    return true;
}

void Set_DAC_Out1_State(uint8_t state)
{
    if(state)
    {
        DAC_Ctrl(DAC_Channel_1, ENABLE);
        DAC_DMACtrl(DAC_Channel_1, ENABLE);
        DMA_ChannelEnable(DMA2_Channel3, ENABLE);
    }
    else
    {
        DMA_ChannelEnable(DMA2_Channel3, DISABLE);
        DAC_DMACtrl(DAC_Channel_1, DISABLE);
        DAC_Ctrl(DAC_Channel_1, DISABLE);
    }
}
