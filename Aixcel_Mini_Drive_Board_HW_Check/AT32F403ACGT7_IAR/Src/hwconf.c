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

#include "hwconf.h"
#include "systick.h"

#ifdef USART1_ENABLE
uint8_t  USART1_Rx_Mem_Region[USART1_RX_BUF_CELL_CNT * USART1_RX_BUF_CELL_SIZE];
uint8_t  USART1_Tx_Mem_Region[USART1_TX_BUF_CELL_CNT * USART1_TX_BUF_CELL_SIZE];
#endif

#ifdef USART2_ENABLE
uint8_t  USART2_Rx_Mem_Region[USART2_RX_BUF_CELL_CNT * USART2_RX_BUF_CELL_SIZE];
uint8_t  USART2_Tx_Mem_Region[USART2_TX_BUF_CELL_CNT * USART2_TX_BUF_CELL_SIZE];
#endif

#ifdef USART3_ENABLE
uint8_t  USART3_Rx_Mem_Region[USART3_RX_BUF_CELL_CNT * USART3_RX_BUF_CELL_SIZE];
uint8_t  USART3_Tx_Mem_Region[USART3_TX_BUF_CELL_CNT * USART3_TX_BUF_CELL_SIZE];
#endif

#ifdef UART4_ENABLE
uint8_t  UART4_Rx_Mem_Region[UART4_RX_BUF_CELL_CNT * UART4_RX_BUF_CELL_SIZE];
uint8_t  UART4_Tx_Mem_Region[UART4_TX_BUF_CELL_CNT * UART4_TX_BUF_CELL_SIZE];
#endif

#ifdef UART5_ENABLE
uint8_t  UART5_Rx_Mem_Region[UART5_RX_BUF_CELL_CNT * UART5_RX_BUF_CELL_SIZE];
uint8_t  UART5_Tx_Mem_Region[UART5_TX_BUF_CELL_CNT * UART5_TX_BUF_CELL_SIZE];
#endif

static bool     System_Power_Flag            =    false;
static bool     System_Key_Down_Flag         =    false;
static uint32_t System_Key_Down_Record_Time  =    0;

void gpio_init(GPIO_Type* GPIOx, GPIOMode_Type GPIO_Mode, GPIOMaxSpeed_Type GPIO_MaxSpeed, uint16_t GPIO_Pins)
{
    GPIO_InitType GPIO_InitStructure;
    
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pins  = GPIO_Pins;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
    GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed;
    
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/*
 * STM32F103/GD32F103??PC13 PC14 PC15??????????????????IO??????????????????????, AT32F403A??GD32F303??????????????
 */
void System_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOC, ENABLE);
    
    /*????????RTC????????????*/
    //PWR_BackupAccessCtrl(ENABLE);

    /*??????????????????????????????PC14 PC15????????????IO??*/
    //RCC_LSEConfig(RCC_LSE_DISABLE);

    /*??????????????????PC13????????IO????*/
    //BKP_TamperPinCmd(DISABLE);

    /*??????????????????*/
    //PWR_BackupAccessCtrl(DISABLE);

    /* NOJTAG: JTAG-DP Disabled and SW-DP Enabled */
    GPIO_PinsRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //??????????SWD??????????????JTAG????????????JTAG??????????PA15??PB3??PB4??????????????IO
    //GPIO_PinsRemapConfig(GPIO_Remap_SWJ_AllDisable, ENABLE); //??????????PA13??PA14??PA15??PB3??PB4????????????IO????????????SWD??????????????????????????????????????????????????????SWD????????????????
    //GPIO_PinsRemapConfig(GPIO_Remap_SWJ_NoJNTRST, ENABLE);   //PB4(USB_RENUM)??????NJTRST????????????????IO??????????????????PB4????????IO????????????SWD??JTAG????????
    
    /* Configure GPIO pin Output Level */
    GPIO_ResetBits(MCU_TIO1_GPIO_Port, MCU_TIO1_Pin);
    
    GPIO_ResetBits(MCU_TIO2_GPIO_Port, MCU_TIO2_Pin);
    
    GPIO_ResetBits(MCU_AIO1_GPIO_Port, MCU_AIO1_Pin);
    
    GPIO_ResetBits(MCU_AIO2_GPIO_Port, MCU_AIO2_Pin);
    
    GPIO_SetBits(EMG_SIG_GPIO_Port, EMG_SIG_Pin);
    
    GPIO_SetBits(BKP_SIG_GPIO_Port, BKP_SIG_Pin);
    
    GPIO_SetBits(PWR_SIG_GPIO_Port, PWR_SIG_Pin);
    
    GPIO_ResetBits(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    
    GPIO_ResetBits(SPI2_CS_GPIO_Port, SPI2_CS_Pin);
    
    GPIO_ResetBits(M485_DE_GPIO_Port, M485_DE_Pin);
    
    GPIO_ResetBits(ESP32_EN_GPIO_Port, ESP32_EN_Pin);
    
    GPIO_ResetBits(IMU_INT1_GPIO_Port, IMU_INT1_Pin);
    
    GPIO_SetBits(SYS_LED_GPIO_Port, SYS_LED_Pin);

    /* Configure GPIO pin */
    gpio_init(MCU_TIO1_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, MCU_TIO1_Pin);
    
    gpio_init(MCU_TIO2_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, MCU_TIO2_Pin);
    
    gpio_init(MCU_AIO1_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, MCU_AIO1_Pin);
    
    gpio_init(MCU_AIO2_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, MCU_AIO2_Pin);
    
    gpio_init(EMG_SIG_GPIO_Port, GPIO_Mode_OUT_PP/*GPIO_Mode_IN_FLOATING*/, GPIO_MaxSpeed_50MHz, EMG_SIG_Pin);
    
    gpio_init(BKP_SIG_GPIO_Port, GPIO_Mode_OUT_PP/*GPIO_Mode_IN_FLOATING*/, GPIO_MaxSpeed_50MHz, BKP_SIG_Pin);
    
    gpio_init(PWR_SIG_GPIO_Port, GPIO_Mode_OUT_PP/*GPIO_Mode_IN_FLOATING*/, GPIO_MaxSpeed_50MHz, PWR_SIG_Pin);
    
    gpio_init(SPI1_CS_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, SPI1_CS_Pin);
    
    gpio_init(SPI2_CS_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, SPI2_CS_Pin);
    
    gpio_init(M485_DE_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, M485_DE_Pin);
    
    gpio_init(ESP32_EN_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, ESP32_EN_Pin);
    
    gpio_init(IMU_INT1_GPIO_Port, GPIO_Mode_IN_FLOATING, GPIO_MaxSpeed_50MHz, IMU_INT1_Pin);
    
    gpio_init(SYS_LED_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, SYS_LED_Pin);
}

/**
  * @brief  USB GPIO initialize
  *         USB use DP->PA12, DM->PA11    
  * @param  None
  * @retval None
  */
void System_USB_GPIO_init()
{
    GPIO_InitType GPIO_InitStructure;
    /* Enable the USB Clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);

    /*Configure DP, DM pin as GPIO_Mode_OUT_PP*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pins  = USB_DP_PIN | USB_DM_PIN;
  #if !defined (AT32F421xx)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
  #else
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
  #endif
    GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_Init(USB_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(USB_GPIO, USB_DP_PIN);
}

void System_PWR_ON_Beep(void)
{
    System_WWDG_Disable();
    
    TMR_SetCompare1(TMR1, 161);
    
    Delay_ms(600);
    
    TMR_SetCompare1(TMR1, 0);
    
    Delay_ms(600);
    
    TMR_SetCompare1(TMR1, 161);
    
    Delay_ms(600);
    
    TMR_SetCompare1(TMR1, 0);
    
    Delay_ms(600);
    
    TMR_SetCompare1(TMR1, 161);
    
    Delay_ms(600);
    
    TMR_SetCompare1(TMR1, 0);
    
    Delay_ms(600);
    
    System_WWDG_Init();
}

void System_NVIC_Init(void)
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
     
    /* USART1 interrupt Init */
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);
    
    /* USART2 interrupt Init */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART2_IRQn);
      
    /* USART3 interrupt Init */
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);
    
    /* SPI1 interrupt Init */
    NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(SPI1_IRQn);
    
    /* SPI2 interrupt Init */
    NVIC_SetPriority(SPI2_I2S2EXT_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(SPI2_I2S2EXT_IRQn);
    
    /* CAN1 interrupt Init */
  #ifdef CAN1_ENABLE
    NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_SetPriority(CAN1_RX1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CAN1_RX1_IRQn);
  #else
    NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  #endif
    
    /* CAN2 interrupt Init */
  #ifdef CAN2_ENABLE
    NVIC_SetPriority(CAN2_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CAN2_TX_IRQn);
    NVIC_SetPriority(CAN2_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    NVIC_SetPriority(CAN2_RX1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CAN2_RX1_IRQn);
  #endif
    
    /* EXTI3 interrupt Init */
    NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI3_IRQn);
    
    /* EXTI4 interrupt Init */
    NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI4_IRQn);
    
    /* EXTI9_5 interrupt Init */
    NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    /* EXTI15_10 interrupt Init */
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void System_Misc_Init(void)
{
    ;
}

void exti_init(uint32_t EXTI_Line, EXTIMode_Type EXTI_Mode, EXTITrigger_Type EXTI_Trigger, FunctionalState EXTI_LineEnable)
{
    EXTI_InitType EXTI_InitStruct;
    
    EXTI_StructInit(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line = EXTI_Line;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger;
    EXTI_InitStruct.EXTI_LineEnable = EXTI_LineEnable;
    
    EXTI_Init(&EXTI_InitStruct);
}

void System_EXTI_Init(void)
{
    /* connect key EXTI line to key GPIO pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinsSource3);

    /* configure key EXTI line */
    exti_init(EXTI_Line3, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE);
    
    EXTI_ClearFlag(EXTI_Line3);
    
    
    /* connect key EXTI line to key GPIO pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinsSource4);

    /* configure key EXTI line */
    exti_init(EXTI_Line4, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE);
    
    EXTI_ClearFlag(EXTI_Line4);
    
    
    /* connect key EXTI line to key GPIO pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinsSource7);

    /* configure key EXTI line */
    exti_init(EXTI_Line7, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE);
    
    EXTI_ClearFlag(EXTI_Line7);
    
    
    /* connect key EXTI line to key GPIO pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinsSource14);

    /* configure key EXTI line */
    exti_init(EXTI_Line14, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE);
    
    EXTI_ClearFlag(EXTI_Line14);
}

/* AT32F4xxx DMA1 Channel Request List
 ************************************************************************************************************
 *Periphrial     Channel1      Channel2      Channel3      Channel4      Channel5      Channel6      Channel7
 *************************************************************************************************************
 *                                                        TIMER1_CH4
 *TIMER1                      TIMER1_CH1    TIMER1_CH2    TIMER1_TRIG   TIMER1_UP     TIMER1_CH3
 *                                                        TIMER1_COM
 *************************************************************************************************************
 *TIMER2        TIMER2_CH3    TIMER2_UP                                 TIMER2_CH1                  TIMER2_CH2
 *                                                                                                  TIMER2_CH3
 *************************************************************************************************************
 *TIMER3                      TIMER3_CH3    TIMER3_CH4                                TIMER3_CH1
 *                                          TIMER3_UP                                 TIMER3_TRIG
 *************************************************************************************************************
 *TIMER4        TIMER4_CH1                                TIMER4_CH2    TIMER4_CH3                  TIMER4_UP
 *************************************************************************************************************
 *ADC1          ADC1
 *************************************************************************************************************
 *SPI/I2S                 SPI1_RX/I2S1_RX SPI1_TX/I2S1_TX SPI2/I2S2_RX SPI2/I2S2_TX
 *************************************************************************************************************
 *USART                       USART3_TX     USART3_RX     USART1_TX     USART1_RX     USART2_RX     USART2_TX
 *************************************************************************************************************
 *I2C                         I2C3_TX       I2C3_RX       I2C2_TX       I2C2_RX       I2C1_TX       I2C1_RX
 *************************************************************************************************************
 */

/* AT32F4xxx DMA2 Channel Request List
 ************************************************************************************************************
 *Periphrial     Channel1      Channel2      Channel3      Channel4      Channel5      
 *************************************************************************************************************
 *TIMER5        TIMER5_CH4    TIMER5_CH3                  TIMER5_CH2    TIMER5_CH1
 *              TIMER5_TRIG   TIMER5_UP
 *************************************************************************************************************
 *TIMER6                                    TIMER6_UP                                
 *************************************************************************************************************
 *TIMER7                                                  TIMER6_UP                             
 *************************************************************************************************************
 *              TIMER8_CH3    TIMER8_CH4
 *TIMER8        TIMER8_UP     TIMER8_TRIG   TIMER8_CH1                  TIMER8_CH2
 *                            TIMER8_COM   
 *************************************************************************************************************
 *ADC2                                                                    ADC2
 *************************************************************************************************************
 *DAC                                        DAC_CH0       DAC_CH1
 *************************************************************************************************************
 *SPI/I2S     SPI2/I2S2_RX   SPI2/I2S2_TX     
 *************************************************************************************************************
 *UART4                                      UART4_RX                    UART4_TX       
 *************************************************************************************************************
 *SDIO                                                      SDIO  
 *************************************************************************************************************
 *SDIO2                                                                   SDIO2 
 *************************************************************************************************************
 */

void System_DMA_Init(void)
{
    /* DMA controller clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);
    
    /* DMA interrupt init */
    /* DMA1_Channel2_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel2_IRQn); 
    
    /* DMA1_Channel3_IRQn interrupt configuration */ 
    NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    
    /* DMA1_Channel4_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    
    /* DMA1_Channel5_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    
    /* DMA1_Channel6_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    
    /* DMA1_Channel7_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    
    
    /* DMA controller clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA2, ENABLE);
    
    /* DMA2_Channel3_IRQn interrupt configuration */ 
    NVIC_SetPriority(DMA2_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA2_Channel3_IRQn);
}

#ifdef USART1_ENABLE
void System_USART1_Init(uint32_t baud_rate)
{
    /* enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);

    /* enable USART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE);

    /* connect port to USARTx_Tx */
    gpio_init(USART1_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, USART1_Tx_Pin);

    /* connect port to USARTx_Rx */
    gpio_init(USART1_GPIO_Port, GPIO_Mode_IN_FLOATING, GPIO_MaxSpeed_50MHz, USART1_Rx_Pin);

    /* Deinitializes the USARTx peripheral */
    USART_Reset(USART1);

    /* USART configure */
    USART_InitType USART_InitStruct;
    
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = baud_rate;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    USART_Init(USART1, &USART_InitStruct);
    
    USART_Cmd(USART1, ENABLE);
    
    USART_INTConfig(USART1, USART_INT_RDNE, ENABLE);
    
    /**************************************************************************/
  #if 0    
    /* USART1 DMA Config */
    DMA_InitType DMA_InitStruct;
    
    /* initialize DMA1 channel5 --> USART1 Rx */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)USART1_Rx_Mem_Region;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = USART1_BASE + 0x04U;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);

    /* USART DMA enable for reception */
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel5, DMA_INT_TC, ENABLE);

    /* disable DMA1 channel5 */
    DMA_ChannelEnable(DMA1_Channel5, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
        
    /* initialize DMA1 channel4 --> USART1 Tx */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)USART1_Tx_Mem_Region;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = USART1_BASE + 0x04U;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);

    /* USART DMA enable for transmission */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel4, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel4 */
    DMA_ChannelEnable(DMA1_Channel4, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
  #endif
}
#endif

#ifdef USART2_ENABLE
void System_USART2_Init(uint32_t baud_rate)
{
    /* enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);

    /* enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART2, ENABLE);

    /* connect port to USARTx_Tx */
    gpio_init(USART2_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, USART2_Tx_Pin);

    /* connect port to USARTx_Rx */
    gpio_init(USART2_GPIO_Port, GPIO_Mode_IN_FLOATING, GPIO_MaxSpeed_50MHz, USART2_Rx_Pin);

    /* Deinitializes the USARTx peripheral */
    USART_Reset(USART2);

    /* USART configure */
    USART_InitType USART_InitStruct;
    
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = baud_rate;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    USART_Init(USART2, &USART_InitStruct);
    
    USART_Cmd(USART2, ENABLE);
    
    USART_INTConfig(USART2, USART_INT_RDNE, ENABLE);
    
    /**************************************************************************/
  #if 0    
    /* USART2 DMA Config */
    DMA_InitType DMA_InitStruct;
    
    /* initialize DMA1 channel6 --> USART2 Rx */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)USART2_Rx_Mem_Region;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = USART2_BASE + 0x04U;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel6, &DMA_InitStruct);

    /* USART DMA enable for reception */
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel6, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel6 */
    DMA_ChannelEnable(DMA1_Channel6, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
    
    /* initialize DMA1 channel7 --> USART2 Tx */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)USART2_Tx_Mem_Region;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = USART2_BASE + 0x04U;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel7, &DMA_InitStruct);

    /* USART DMA enable for transmission */
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel7, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel7 */
    DMA_ChannelEnable(DMA1_Channel7, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
  #endif
}
#endif

#ifdef USART3_ENABLE
void System_USART3_Init(uint32_t baud_rate)
{
    /* enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);

    /* enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART3, ENABLE);

    /* connect port to USARTx_Tx */
    gpio_init(USART3_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, USART3_Tx_Pin);

    /* connect port to USARTx_Rx */
    gpio_init(USART3_GPIO_Port, GPIO_Mode_IN_FLOATING, GPIO_MaxSpeed_50MHz, USART3_Rx_Pin);

    /* Deinitializes the USARTx peripheral */
    USART_Reset(USART3);
    
    /* USART configure */
    USART_InitType USART_InitStruct;
    
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = baud_rate;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    USART_Init(USART3, &USART_InitStruct);
    
    USART_Cmd(USART3, ENABLE);
    
    USART_INTConfig(USART3, USART_INT_RDNE, ENABLE);
    
    /**************************************************************************/
  #if 0    
    /* USART3 DMA Config */
    DMA_InitType DMA_InitStruct;
    
    /* initialize DMA1 channel3 --> USART3 Rx */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)USART3_Rx_Mem_Region;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = USART3_BASE + 0x04U;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel3, &DMA_InitStruct);

    /* USART DMA enable for reception */
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel3, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel3 */
    DMA_ChannelEnable(DMA1_Channel3, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
    
    /* initialize DMA1 channel2 --> USART3 Tx */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)USART3_Tx_Mem_Region;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = USART3_BASE + 0x04U;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel2, &DMA_InitStruct);

    /* USART DMA enable for transmission */
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel2, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel2 */
    DMA_ChannelEnable(DMA1_Channel2, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
  #endif
}
#endif

void USART2_EN_M485_Tx(void)
{
    M485_DE_Pin_PULL_UP();
}

void USART2_EN_M485_Rx(void)
{
    M485_DE_Pin_PULL_DOWN();
}

void USB_CDC_USART_Init(uint32_t baud_rate)
{
    ;
}

void System_TMR1_Init(void)
{
    /* For AT32F4xx TIMER1,8,9,10,11 is at APB2(APB2_Fmax = 120MHz while CK_AHB = 240MHz & APB2 prescale = 2), 
     * but if(APB2 prescale = 1) CK_TIMERx = APB2, else CK_TIMERx = APB2*2, while APB2 = 120MHZ then CK_TIMERx = 240MHz,
     * if TIMER1's prescaler = 239, then TIMER1's clock is 240MHz/(1+239)=1MHz, 1MHz / 322 = 3.1KHz,
     * while APB2_Fmax = 100MHz & CK_AHB = 200MHz,
     * CK_TIMERx = 200MHz, if TIMER1's prescaler = 199, then TIMER1's clock is 200MHz/(1+199)=1MHz, 1MHz / 322 = 3.1KHz
     */
    TMR_OCInitType TMR_OCInitStruct;
    TMR_TimerBaseInitType TMR_TimeBaseInitStruct;	
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR1, ENABLE);
    
    gpio_init(BEEP_CTRL_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, BEEP_CTRL_Pin);
    
    TMR_Reset(TMR1);
    
    /* TIMER1 configuration */
    TMR_TimeBaseInitStruct.TMR_DIV               = 239;
    TMR_TimeBaseInitStruct.TMR_Period            = 322-1;
    TMR_TimeBaseInitStruct.TMR_ClockDivision     = TMR_CKD_DIV1;
    TMR_TimeBaseInitStruct.TMR_RepetitionCounter = 0;
    
    TMR_TimeBaseInit(TMR1, &TMR_TimeBaseInitStruct);
    
    TMR_CounterModeConfig(TMR1, TMR_CounterDIR_Up);
    
    /* CH1 configuration in PWM mode1, duty cycle (pwm)/322 */
    TMR_OCInitStruct.TMR_OCMode       = TMR_OCMode_PWM1;
    TMR_OCInitStruct.TMR_OutputState  = TMR_OutputState_Enable;
    TMR_OCInitStruct.TMR_OutputNState = TMR_OutputNState_Disable;
    TMR_OCInitStruct.TMR_Pulse        = 0;
    TMR_OCInitStruct.TMR_OCPolarity   = TMR_OCPolarity_High;
    TMR_OCInitStruct.TMR_OCNPolarity  = TMR_OCNPolarity_High;
    TMR_OCInitStruct.TMR_OCIdleState  = TMR_OCIdleState_Reset;
    TMR_OCInitStruct.TMR_OCNIdleState = TMR_OCNIdleState_Reset;
    
    TMR_OC1Init(TMR1, &TMR_OCInitStruct);
    
    TMR_CtrlPWMOutputs(TMR1, ENABLE);
    
    TMR_OC1PreloadConfig(TMR1, TMR_OCPreload_Enable);

    /* auto-reload preload enable */
    TMR_ARPreloadConfig(TMR1, ENABLE);
    
    /* tmr enable */
    TMR_Cmd(TMR1, ENABLE);
}

/* 
 * STM32F103/GD32F103/GD32F303/AT32F403A????MCU??????????2/3/4/5??????/??????????????????????16????????????????16??????????32??????????
 * STM32F429????????2/3/4/5??????/??????????????????????32??????????16?????? TIM2 ?? TIM5????????DMA??STM32F429??????????/4??????/??????????
 * ??????????????????????16??????????32????????????????DMA??STM32F429????????2/5??????/????????????????????????????????32??????????????????
 * ????????????????????????32????????????????????????????????????????????16??????????????2/5??????/??????????????????????????????????32????
 * ????????????????????????32??????????????????????????PWM????????????????????????????
 */
void System_TMR2_Init(void)
{
    /* For AT32F4xx TIMER2,3,4,5,6,7,12,13,14 is at APB1(APB1_Fmax = 120MHz while CK_AHB = 240MHz & APB1 prescale = 2), 
     * but if(APB1 prescale = 1) CK_TIMERx = APB1, else CK_TIMERx = APB1*2, while APB1 = 120MHZ then CK_TIMERx = 240MHz,
     * if TIMER2's prescaler = 1, then TIMER2's clock is 240MHz/(1+1)=120MHz, 120MHz / 150 = 800KHz,
     * while APB1_Fmax = 100MHz & CK_AHB = 200MHz,
     * CK_TIMERx = 200MHz, if TIMER2's prescaler = 1, then TIMER2's clock is 200MHz/(1+1)=100MHz, 100MHz / 125 = 800KHz
     */
    TMR_OCInitType TMR_OCInitStruct;
    TMR_TimerBaseInitType TMR_TimeBaseInitStruct;	
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR2, ENABLE);
    
    gpio_init(MCU_TIO1_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, MCU_TIO1_Pin);
    gpio_init(MCU_TIO2_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, MCU_TIO2_Pin);
    
    TMR_Reset(TMR2);
    
    /* TIMER2 configuration */
    TMR_TimeBaseInitStruct.TMR_DIV               = 1;
    TMR_TimeBaseInitStruct.TMR_Period            = 150-1;
    TMR_TimeBaseInitStruct.TMR_ClockDivision     = TMR_CKD_DIV1;
    TMR_TimeBaseInitStruct.TMR_RepetitionCounter = 0;
    
    TMR_TimeBaseInit(TMR2, &TMR_TimeBaseInitStruct);
    
    TMR_CounterModeConfig(TMR2, TMR_CounterDIR_Up);
    
    /* CH configuration in PWM mode1, duty cycle (pwm)/150 */
    TMR_OCInitStruct.TMR_OCMode       = TMR_OCMode_PWM1;
    TMR_OCInitStruct.TMR_OutputState  = TMR_OutputState_Enable;
    TMR_OCInitStruct.TMR_OutputNState = TMR_OutputNState_Disable;
    TMR_OCInitStruct.TMR_Pulse        = 75;
    TMR_OCInitStruct.TMR_OCPolarity   = TMR_OCPolarity_High;
    TMR_OCInitStruct.TMR_OCNPolarity  = TMR_OCNPolarity_High;
    TMR_OCInitStruct.TMR_OCIdleState  = TMR_OCIdleState_Reset;
    TMR_OCInitStruct.TMR_OCNIdleState = TMR_OCNIdleState_Reset;
    
    TMR_OC1Init(TMR2, &TMR_OCInitStruct);
    TMR_OC2Init(TMR2, &TMR_OCInitStruct);
    
    TMR_CtrlPWMOutputs(TMR2, ENABLE);
    
    TMR_OC1PreloadConfig(TMR2, TMR_OCPreload_Enable);
    TMR_OC2PreloadConfig(TMR2, TMR_OCPreload_Enable);

    /* auto-reload preload enable */
    TMR_ARPreloadConfig(TMR2, ENABLE);
    
    /* tmr enable */
    TMR_Cmd(TMR2, ENABLE);
    
    TMR_DMACmd(TMR2, TMR_DMA_CC1, ENABLE);
    TMR_DMACmd(TMR2, TMR_DMA_CC2, ENABLE);
    
    /**************************************************************************/
    
    DMA_InitType DMA_InitStruct;
    
    /* initialize DMA1 channel5 --> TMR2_CH1 */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = NULL;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(TMR2->CC1);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel5, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel5 */
    DMA_ChannelEnable(DMA1_Channel5, DISABLE);
    
    /* initialize DMA1 channel7 --> TMR2_CH2 */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = NULL;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(TMR2->CC2);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel7, &DMA_InitStruct);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel7, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel7 */
    DMA_ChannelEnable(DMA1_Channel7, DISABLE);
}

void System_TMR3_Init(void)
{
    /* For AT32F4xx TIMER2,3,4,5,6,7,12,13,14 is at APB1(APB1_Fmax = 120MHz while CK_AHB = 240MHz & APB1 prescale = 2), 
     * but if(APB1 prescale = 1) CK_TIMERx = APB1, else CK_TIMERx = APB1*2, while APB1 = 120MHZ then CK_TIMERx = 240MHz,
     * if TIMER3's prescaler = 1, then TIMER3's clock is 240MHz/(1+1)=120MHz, 120MHz / 150 = 800KHz,
     */
    TMR_OCInitType TMR_OCInitStruct;
    TMR_TimerBaseInitType TMR_TimeBaseInitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);
    
    gpio_init(MCU_AIO1_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, MCU_AIO1_Pin);
    
    gpio_init(MCU_AIO2_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, MCU_AIO2_Pin);
    
    TMR_Reset(TMR3);
    
    /* TIMER3 configuration */
    TMR_TimeBaseInitStruct.TMR_DIV               = 1;
    TMR_TimeBaseInitStruct.TMR_Period            = 150-1;
    TMR_TimeBaseInitStruct.TMR_ClockDivision     = TMR_CKD_DIV1;
    TMR_TimeBaseInitStruct.TMR_RepetitionCounter = 0;
    
    TMR_TimeBaseInit(TMR3, &TMR_TimeBaseInitStruct);
    
    TMR_CounterModeConfig(TMR3, TMR_CounterDIR_Up);

    /* CH4 configuration in PWM mode1, duty cycle (pwm)/150 */
    TMR_OCInitStruct.TMR_OCMode       = TMR_OCMode_PWM1;
    TMR_OCInitStruct.TMR_OutputState  = TMR_OutputState_Enable;
    TMR_OCInitStruct.TMR_OutputNState = TMR_OutputNState_Disable;
    TMR_OCInitStruct.TMR_Pulse        = 75;
    TMR_OCInitStruct.TMR_OCPolarity   = TMR_OCPolarity_High;
    TMR_OCInitStruct.TMR_OCNPolarity  = TMR_OCNPolarity_High;
    TMR_OCInitStruct.TMR_OCIdleState  = TMR_OCIdleState_Reset;
    TMR_OCInitStruct.TMR_OCNIdleState = TMR_OCNIdleState_Reset;
    
    TMR_OC3Init(TMR3, &TMR_OCInitStruct);
    TMR_OC4Init(TMR3, &TMR_OCInitStruct);
    
    TMR_OC3PreloadConfig(TMR3, TMR_OCPreload_Enable);
    TMR_OC4PreloadConfig(TMR3, TMR_OCPreload_Enable);

    /* auto-reload preload enable */
    TMR_ARPreloadConfig(TMR3, ENABLE);
    /* TMR3 enable */
    TMR_Cmd(TMR3, ENABLE);
    
    TMR_DMACmd(TMR3, TMR_DMA_CC3, ENABLE);
    TMR_DMACmd(TMR3, TMR_DMA_CC4, ENABLE);
    
    /**************************************************************************/
    
    DMA_InitType DMA_InitStruct;
    
    /* initialize DMA1 channel2 --> TMR3_CH3 */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = NULL;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(TMR3->CC3);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel2, &DMA_InitStruct);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel2, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel2 */
    DMA_ChannelEnable(DMA1_Channel2, DISABLE);
    
    /**************************************************************************/
    
    /* initialize DMA1 channel3 --> TMR3_CH4 */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = NULL;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(TMR3->CC4);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel3, &DMA_InitStruct);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA1_Channel3, DMA_INT_TC, ENABLE);
    
    /* disable DMA1 channel3 */
    DMA_ChannelEnable(DMA1_Channel3, DISABLE);
}

void System_TMR6_Init(void)
{
    /* For AT32F4xx TIMER2,3,4,5,6,7,12,13,14 is at APB1(APB1_Fmax = 120MHz while CK_AHB = 240MHz & APB1 prescale = 2), 
     * but if(APB1 prescale = 1) CK_TIMERx = APB1, else CK_TIMERx = APB1*2, while APB1 = 120MHZ then CK_TIMERx = 240MHz,
     * if TIMER6's prescaler = 0x01, then TIMER6's clock is 240MHz/(0x01+1)=120MHz
     */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6, ENABLE);
    
    TMR_Reset(TMR6);
    
    TMR_DIVConfig(TMR6, 0x01, TMR_DIVReloadMode_Update); 
    
    TMR_SetAutoreload(TMR6, 7);
    
    TMR_SelectOutputTrigger(TMR6, TMR_TRGOSource_Update);
    
    TMR_Cmd(TMR6, ENABLE);
}

#ifdef DAC_OUT1_ENABLE

/* Sine Wave Data Number */
#define SINE_POINT_NUM 32

/******************************Sine Wave Data***********************************/
const uint16_t SineWave12bit[SINE_POINT_NUM] = {
        2048,    2460,    2856,    3218,    3532,    3786,    3969,    4072,
        4093,    4031,    3887,    3668,    3382,    3042,    2661,    2255, 
        1841,    1435,    1054,     714,     428,     209,      65,       3,
          24,     127,     310,     564,     878,    1240,    1636,    2048
};

void System_DAC_OUT1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_DAC, ENABLE);
    
    /* once enabled the DAC, the corresponding GPIO pin is connected to the DAC converter automatically */
    gpio_init(GPIOA, GPIO_Mode_IN_ANALOG, GPIO_MaxSpeed_50MHz, GPIO_Pins_4);
    
    DAC_Reset();
    
    /* configure the DAC0 */
    DAC_InitType DAC_InitStruct;
    
    DAC_InitStruct.DAC_Trigger = DAC_Trigger_TMR6_TRGO;
    DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
      
    DAC_Init(DAC_Channel_1, &DAC_InitStruct);
    
    /* enable DAC1 and DMA for DAC1 */    
    DAC_Ctrl(DAC_Channel_1, ENABLE);
    DAC_DMACtrl(DAC_Channel_1, ENABLE);
    
    /**************************************************************************/
    
    /* DAC OUT1 DMA Config */
    DMA_InitType DMA_InitStruct;
    
    /* clear all the interrupt flags */
    DMA_ClearFlag(DMA2_FLAG_GL3);
    DMA_ClearFlag(DMA2_FLAG_TC3);
    DMA_ClearFlag(DMA2_FLAG_HT3);
    DMA_ClearFlag(DMA2_FLAG_ERR3);
    
    /* configure the DMA1 channel 3 */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)SineWave12bit;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = SINE_POINT_NUM-1;         //The one phrase's end is connected with the next's start, so minus 1!!!
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->HDR12R1);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_CIRCULAR;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA2_Channel3, &DMA_InitStruct);
    
    /* enable DMA transfer complete interrupt */
    DMA_INTConfig(DMA2_Channel3, DMA_INT_TC, ENABLE);
    
    DMA_ChannelEnable(DMA2_Channel3, DISABLE);               //Caution: Not enable DMA_Channel during init process, Only enable DMA_Channel while receive/transmmit data using DMA !!!
}
#endif

#ifdef ADC1_ENABLE
typedef enum
{
    ADC_Convertion_AIO1,
    ADC_Convertion_AIO2,
    ADC_Convertion_CNT,
}ADC_Convertion_Rank_Typedef;

uint16_t ADC_Convertion_Value[ADC_Convertion_CNT] = {0x00};

void System_ADC1_Init(void)
{
    /* enable GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
    
    /* enable ADC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC1, ENABLE);
    
    /* config ADC clock */
    RCC_ADCCLKConfig(RCC_APB2CLK_Div8);
    
    /* config the GPIO as analog mode */
    gpio_init(MCU_AIO1_GPIO_Port, GPIO_Mode_IN_ANALOG, GPIO_MaxSpeed_50MHz, MCU_AIO1_Pin);
    gpio_init(MCU_AIO2_GPIO_Port, GPIO_Mode_IN_ANALOG, GPIO_MaxSpeed_50MHz, MCU_AIO2_Pin);
    
    ADC_InitType ADC_InitStruct;
    
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;           /* ADC mode config */
    ADC_InitStruct.ADC_ContinuousMode = ENABLE;               /* ADC continuous function enable */
    ADC_InitStruct.ADC_ScanMode = ENABLE;                     /* ADC scan function enable */
    ADC_InitStruct.ADC_ExternalTrig = ADC_ExternalTrig_None;  /* ADC trigger config */
    ADC_InitStruct.ADC_NumOfChannel = ADC_Convertion_CNT;                      /* ADC channel length config */
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;       /* ADC data alignment config */
    
    ADC_Init(ADC1, &ADC_InitStruct);
    
    /* ADC regular channel config */ 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 0, ADC_SampleTime_239_5);  
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_239_5);   
    
    ADC_ExternalTrigConvCtrl(ADC1, ENABLE);

    /* ADC DMA function enable */
    ADC_DMACtrl(ADC1, ENABLE);
    /* enable ADC interface */
    ADC_Ctrl(ADC1, ENABLE);
    Delay_ms(1);
    /* ADC calibration and reset calibration */
    ADC_RstCalibration(ADC1);
    ADC_StartCalibration(ADC1);

    /* ADC software trigger enable */
    ADC_SoftwareStartConvCtrl(ADC1, ENABLE);
    
    /**************************************************************************/
    
    /* ADC_DMA_channel configuration */
    DMA_InitType DMA_InitStruct;
    
    /* ADC DMA_channel configuration */
    DMA_Reset(DMA1_Channel1);
    
    /* initialize DMA single data mode */
    DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(ADC_Convertion_Value);
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_BufferSize = ADC_Convertion_CNT;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->RDOR);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_HIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_CIRCULAR;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
  
    /* enable DMA channel */
    DMA_ChannelEnable(DMA1_Channel1, ENABLE);
}
#endif

#ifdef SPI1_ENABLE
void System_SPI1_Init(uint32_t baud_rate)
{
    SPI_InitType SPI_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_SPI1, ENABLE);
    
    /* SPI1 GPIO config: NSS/PA4, SCK/PA5, MOSI/PA7 */
    gpio_init(SPI1_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, SPI1_CS_Pin);
    gpio_init(SPI1_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, SPI1_SCK_Pin);
    gpio_init(SPI1_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, SPI1_MOSI_Pin);
    /* SPI1 GPIO config: MISO/PA6 */
    gpio_init(SPI1_GPIO_Port, GPIO_Mode_IN_FLOATING, GPIO_MaxSpeed_50MHz, SPI1_MISO_Pin);
    
    /* Deinitializes the SPIx peripheral */
    SPI_I2S_Reset(SPI1);
    
    /* SPI1 parameter config */
    SPI_DefaultInitParaConfig(&SPI_InitStruct);
    
    SPI_InitStruct.SPI_TransMode        = SPI_TRANSMODE_FULLDUPLEX;
    SPI_InitStruct.SPI_Mode             = SPI_MODE_MASTER;
    SPI_InitStruct.SPI_FrameSize        = SPI_FRAMESIZE_8BIT;
    SPI_InitStruct.SPI_CPOL             = SPI_CPOL_HIGH;
    SPI_InitStruct.SPI_CPHA             = SPI_CPHA_2EDGE;
    SPI_InitStruct.SPI_NSSSEL           = SPI_NSSSEL_SOFT;
    SPI_InitStruct.SPI_MCLKP            = SPI_MCLKP_16;     //SPI1????APB2??, AT32F403A??APB2????????????120MHz, 16??????????7.5MHz, lsm6dsl??SPI??????????10MHz
    SPI_InitStruct.SPI_FirstBit         = SPI_FIRSTBIT_MSB;
    SPI_Init(SPI1, &SPI_InitStruct);
    
    SPI_Enable(SPI1, ENABLE);
}
#endif

#ifdef SPI2_ENABLE
void System_SPI2_Init(uint32_t baud_rate)
{
    SPI_InitType SPI_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_SPI2, ENABLE);
    
    /* SPI2 GPIO config: NSS/PB12, SCK/PB13, MOSI/PB15 */
    gpio_init(SPI2_GPIO_Port, GPIO_Mode_OUT_PP, GPIO_MaxSpeed_50MHz, SPI2_CS_Pin);
    gpio_init(SPI2_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, SPI2_SCK_Pin);
    gpio_init(SPI2_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, SPI2_MOSI_Pin);
    /* SPI2 GPIO config: MISO/PB14 */
    gpio_init(SPI2_GPIO_Port, GPIO_Mode_IN_FLOATING, GPIO_MaxSpeed_50MHz, SPI2_MISO_Pin);

    /* Deinitializes the SPIx peripheral */
    SPI_I2S_Reset(SPI2);
    
    /* SPI2 parameter config */
    SPI_DefaultInitParaConfig(&SPI_InitStruct);
    
    SPI_InitStruct.SPI_TransMode        = SPI_TRANSMODE_FULLDUPLEX;
    SPI_InitStruct.SPI_Mode             = SPI_MODE_MASTER;
    SPI_InitStruct.SPI_FrameSize        = SPI_FRAMESIZE_8BIT;
    SPI_InitStruct.SPI_CPOL             = SPI_CPOL_HIGH;
    SPI_InitStruct.SPI_CPHA             = SPI_CPHA_2EDGE;
    SPI_InitStruct.SPI_NSSSEL           = SPI_NSSSEL_SOFT;
    SPI_InitStruct.SPI_MCLKP            = SPI_MCLKP_4;     //SPI2????APB1??, AT32F403A??APB2????????????120MHz, 4??????????30MHz, GD25Q127CSIG??SPI????????????????80MHz
    SPI_InitStruct.SPI_FirstBit         = SPI_FIRSTBIT_MSB;
    SPI_Init(SPI2, &SPI_InitStruct);
    
    SPI_Enable(SPI2, ENABLE);
}
#endif

#ifdef I2C1_ENABLE
void System_I2C1_Init(uint32_t baud_rate)
{
    /* enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);

    /* enable I2C clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_I2C1, ENABLE);
    
    /* connect PB8 to I2C1_SCL, PB9 to I2C1_SDA */
    gpio_init(I2C1_SCL_Port, GPIO_Mode_AF_OD, GPIO_MaxSpeed_50MHz, I2C1_SCL_Pin);
    gpio_init(I2C1_SDA_Port, GPIO_Mode_AF_OD, GPIO_MaxSpeed_50MHz, I2C1_SDA_Pin);
    
    GPIO_PinsRemapConfig(AFIO_MAP5_I2C1_0001, ENABLE);
    
    I2C_DeInit(I2C1);
    
    I2C_InitType I2C_InitStruct;
    
    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_BitRate = baud_rate;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2CDevice;
    I2C_InitStruct.I2C_FmDutyCycle = I2C_FmDutyCycle_2_1;
    I2C_InitStruct.I2C_OwnAddr1 = I2C1_OWN_ADDRESS0;
    I2C_InitStruct.I2C_AddrMode = I2C_AddrMode_7bit;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    
    I2C_Init(I2C1, &I2C_InitStruct);
    
    /* enable I2C1 */
    I2C_Cmd(I2C1, ENABLE);
}
#endif

#ifdef CAN1_ENABLE
void System_CAN1_Init(CAN_Baudrate_Typedef baudrate)
{
    CAN_InitType CAN_InitStruct;
    
    /* enable CAN clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_CAN1, ENABLE);
    
    gpio_init(CAN1_GPIO_Port, GPIO_Mode_IN_PU, GPIO_MaxSpeed_50MHz, CAN1_RX_Pin);
    gpio_init(CAN1_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, CAN1_TX_Pin);
    
    /* 
     * CAN1_REMAP [1:0] 
     * 00: ??????????(CAN_RX/PA11, CAN_TX/PA12), 
     * 01: ????????, 
     * 10: ??????????(CAN_RX/PB8,  CAN_TX/PB9), 
     * 11: ??????????(CAN_RX/PD0,  CAN_TX/PD1),
     */
    //GPIO_PinsRemapConfig(GPIO_Remap1_CAN1, ENABLE);
    
    CAN_StructInit(&CAN_InitStruct);
        
    CAN_Reset(CAN1);
    
    Delay_ms(2);
    
    /* initialize CAN parameters */
    CAN_InitStruct.CAN_TTC = DISABLE;
    CAN_InitStruct.CAN_ABO = DISABLE;
    CAN_InitStruct.CAN_AWU = DISABLE;
    CAN_InitStruct.CAN_NART = DISABLE;
    CAN_InitStruct.CAN_RFL = DISABLE;
    CAN_InitStruct.CAN_TFP = DISABLE;
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
    
    switch(baudrate)
    {
        case CAN_BAUDRATE_5K:
            CAN_InitStruct.CAN_Prescaler = 2400;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_10K:
            CAN_InitStruct.CAN_Prescaler = 1200;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_20K:
            CAN_InitStruct.CAN_Prescaler = 600;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
            break;
        case CAN_BAUDRATE_50K:
            CAN_InitStruct.CAN_Prescaler = 240;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_100K:
            CAN_InitStruct.CAN_Prescaler = 120;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_125K:
            CAN_InitStruct.CAN_Prescaler = 96;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_250K:
            CAN_InitStruct.CAN_Prescaler = 48;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_500K:
            CAN_InitStruct.CAN_Prescaler = 24;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_600K:
            CAN_InitStruct.CAN_Prescaler = 20;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_750K:
            CAN_InitStruct.CAN_Prescaler = 16;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_800K:
            CAN_InitStruct.CAN_Prescaler = 12;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;
            break;
        case CAN_BAUDRATE_1M:        
            CAN_InitStruct.CAN_Prescaler = 12;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        default:
            CAN_InitStruct.CAN_Prescaler = 12;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
    }
    
    /* initialize CAN */
    CAN_Init(CAN1, &CAN_InitStruct);
}
#endif

#ifdef CAN2_ENABLE
void System_CAN2_Init(CAN_Baudrate_Typedef baudrate)
{
    CAN_InitType CAN_InitStruct;
    
    /* enable CAN clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_CAN2, ENABLE);
    
    gpio_init(CAN2_GPIO_Port, GPIO_Mode_IN_PU, GPIO_MaxSpeed_50MHz, CAN2_RX_Pin);
    gpio_init(CAN2_GPIO_Port, GPIO_Mode_AF_PP, GPIO_MaxSpeed_50MHz, CAN2_TX_Pin);
    
    /* 
     * CAN2_REMAP 
     * 0: ??????????(CAN_RX/PB12, CAN_TX/PB13), 
     * 1: ??????(CAN_RX/PB5,  CAN_TX/PB6),
     */
    GPIO_PinsRemapConfig(GPIO_Remap_CAN2, ENABLE);
    
    CAN_StructInit(&CAN_InitStruct);
    
    CAN_Reset(CAN2);
    
    Delay_ms(2);
    
    /* initialize CAN parameters */
    CAN_InitStruct.CAN_TTC = DISABLE;
    CAN_InitStruct.CAN_ABO = DISABLE;
    CAN_InitStruct.CAN_AWU = DISABLE;
    CAN_InitStruct.CAN_NART = DISABLE;
    CAN_InitStruct.CAN_RFL = DISABLE;
    CAN_InitStruct.CAN_TFP = DISABLE;
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
    
    switch(baudrate)
    {
        case CAN_BAUDRATE_5K:
            CAN_InitStruct.CAN_Prescaler = 2400;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_10K:
            CAN_InitStruct.CAN_Prescaler = 1200;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_20K:
            CAN_InitStruct.CAN_Prescaler = 600;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
            break;
        case CAN_BAUDRATE_50K:
            CAN_InitStruct.CAN_Prescaler = 240;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_100K:
            CAN_InitStruct.CAN_Prescaler = 120;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_125K:
            CAN_InitStruct.CAN_Prescaler = 96;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_250K:
            CAN_InitStruct.CAN_Prescaler = 48;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_500K:
            CAN_InitStruct.CAN_Prescaler = 24;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_600K:
            CAN_InitStruct.CAN_Prescaler = 20;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_750K:
            CAN_InitStruct.CAN_Prescaler = 16;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        case CAN_BAUDRATE_800K:
            CAN_InitStruct.CAN_Prescaler = 12;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;
            break;
        case CAN_BAUDRATE_1M:        
            CAN_InitStruct.CAN_Prescaler = 12;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
        default:
            CAN_InitStruct.CAN_Prescaler = 12;
            CAN_InitStruct.CAN_SJW = CAN_SJW_2tq;
            CAN_InitStruct.CAN_BS1 = CAN_BS1_5tq;
            CAN_InitStruct.CAN_BS2 = CAN_BS2_4tq;
            break;
    }
    
    /* initialize CAN */
    CAN_Init(CAN2, &CAN_InitStruct);
}
#endif

uint8_t System_CAN_Start(CAN_Type* can_periph)
{
  #ifdef CAN1_ENABLE 
    CAN_INTConfig(CAN1, CAN_INT_RFP0, ENABLE);        /*!< Enable FIFO 0 message pending interrupt */
    CAN_INTConfig(CAN1, CAN_INT_RFFU0, ENABLE);       /*!< Enable FIFO 0 full interrupt            */
    CAN_INTConfig(CAN1, CAN_INT_RFOV0, ENABLE);       /*!< Enable FIFO 0 overrun interrupt         */
    CAN_INTConfig(CAN1, CAN_INT_RFP1, ENABLE);        /*!< Enable FIFO 1 message pending interrupt */
    CAN_INTConfig(CAN1, CAN_INT_RFFU1, ENABLE);       /*!< Enable FIFO 1 full interrupt            */
    CAN_INTConfig(CAN1, CAN_INT_RFOV1, ENABLE);       /*!< Enable FIFO 1 overrun interrupt         */
  #endif
    
  #ifdef CAN2_ENABLE                                //Caution: STM32F10x has no CAN2!!!
    CAN_INTConfig(CAN2, CAN_INT_RFP0, ENABLE);        /*!< Enable FIFO 0 message pending interrupt */
    CAN_INTConfig(CAN2, CAN_INT_RFFU0, ENABLE);       /*!< Enable FIFO 0 full interrupt            */
    CAN_INTConfig(CAN2, CAN_INT_RFOV0, ENABLE);       /*!< Enable FIFO 0 overrun interrupt         */
    CAN_INTConfig(CAN2, CAN_INT_RFP1, ENABLE);        /*!< Enable FIFO 1 message pending interrupt */
    CAN_INTConfig(CAN2, CAN_INT_RFFU1, ENABLE);       /*!< Enable FIFO 1 full interrupt            */
    CAN_INTConfig(CAN2, CAN_INT_RFOV1, ENABLE);       /*!< Enable FIFO 1 overrun interrupt         */
  #endif
    
    return CAN_OperatingModeRequest(can_periph, CAN_OperatingMode_Normal);
}

uint8_t System_CAN_Stop(CAN_Type* can_periph)
{
    return CAN_OperatingModeRequest(can_periph, CAN_OperatingMode_Initialization);
}

#ifdef WWDG_ENABLE
void System_WWDG_Init(void)
{
    /* enable WWDGT clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_WWDG, ENABLE);
    
    /*
     *  set WWDGT clock = (PCLK1 (100MHz)/4096)/8 = 3051Hz (~327 us)
     *  set counter value to 127
     *  set window value to 127
     *  refresh window is: ~327 * (127-127)= 0ms < refresh window < ~327 * (127-63) =20.9ms.
     */
    WWDG_SetPrescaler(WWDG_Psc_8);
    WWDG_SetCounter(127);
    WWDG_SetWindowCounter(127);
    WWDG_Enable(127);
}
#else
void System_WWDG_Init(void)
{
    ;
}
#endif

void System_WWDG_Disable(void)
{
    WWDG_SetCounter(127);
    
    /* disable WWDG clock */
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_WWDG, DISABLE);
}

void EXTI3_IRQ_Callback(void)
{
    ;
}

void EXTI4_IRQ_Callback(void)
{
    ;
}

void EXTI7_IRQ_Callback(void)
{
  #if 0
    if(System_Key_Down_Flag == false)
    {
        System_Key_Down_Flag = true;
        
        System_Key_Down_Record_Time = Systick_Get();
        
        /* configure key EXTI line */
        exti_init(EXTI_Line7, EXTI_Mode_Interrupt, EXTI_Trigger_Rising, ENABLE);
    }
    else
    {
        System_Key_Down_Flag = false;
        
        if(Systick_Get() - System_Key_Down_Record_Time > 1000)
        {
            if(System_Power_Flag == false)
            {
                System_Power_Flag = true;
            }
            else
            {
                System_Power_Flag = false;
            }
        }
        
        /* configure key EXTI line */
        exti_init(EXTI_Line7, EXTI_Mode_Interrupt, EXTI_Trigger_Falling, ENABLE);
    }
  #endif
}

void EXTI15_10_IRQ_Callback(void)
{
    ;
}

bool System_Power_Flag_Get(void)
{
    return System_Power_Flag;
}
