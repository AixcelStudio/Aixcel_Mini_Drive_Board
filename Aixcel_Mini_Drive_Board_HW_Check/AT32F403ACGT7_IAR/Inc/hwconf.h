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

#ifndef HWCONF_H
#define HWCONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f4xx.h"
#include <stdio.h>
#include "common.h"

//#define  WWDG_ENABLE

#define  USART1_ENABLE
#define  USART2_ENABLE
#define  USART3_ENABLE
#define  SPI1_ENABLE
#define  SPI2_ENABLE
#define  I2C1_ENABLE
#define  CAN2_ENABLE

#ifdef USART1_ENABLE
#define  USART1_RX_BUF_CELL_CNT           32U
#define  USART1_RX_BUF_CELL_SIZE          16U
 
#define  USART1_TX_BUF_CELL_CNT           32U
#define  USART1_TX_BUF_CELL_SIZE          16U
#endif

#ifdef USART2_ENABLE
#define  USART2_RX_BUF_CELL_CNT           32U
#define  USART2_RX_BUF_CELL_SIZE          16U
    
#define  USART2_TX_BUF_CELL_CNT           32U
#define  USART2_TX_BUF_CELL_SIZE          16U
#endif

#ifdef USART3_ENABLE
#define  USART3_RX_BUF_CELL_CNT           32U
#define  USART3_RX_BUF_CELL_SIZE          16U

#define  USART3_TX_BUF_CELL_CNT           32U
#define  USART3_TX_BUF_CELL_SIZE          16U
#endif

#ifdef UART4_ENABLE
#define  UART4_RX_BUF_CELL_CNT            32U
#define  UART4_RX_BUF_CELL_SIZE           16U

#define  UART4_TX_BUF_CELL_CNT            64U
#define  UART4_TX_BUF_CELL_SIZE           16U
#endif

#ifdef UART5_ENABLE
#define  UART5_RX_BUF_CELL_CNT            32U
#define  UART5_RX_BUF_CELL_SIZE           16U

#define  UART5_TX_BUF_CELL_CNT            32U
#define  UART5_TX_BUF_CELL_SIZE           16U
#endif
    
#define  NVIC_PRIORITYGROUP_0     0x00000007U /*!< 0 bits for pre-emption priority
                                                   4 bits for subpriority */
#define  NVIC_PRIORITYGROUP_1     0x00000006U /*!< 1 bits for pre-emption priority
                                                   3 bits for subpriority */
#define  NVIC_PRIORITYGROUP_2     0x00000005U /*!< 2 bits for pre-emption priority
                                                   2 bits for subpriority */
#define  NVIC_PRIORITYGROUP_3     0x00000004U /*!< 3 bits for pre-emption priority
                                                   1 bits for subpriority */
#define  NVIC_PRIORITYGROUP_4     0x00000003U /*!< 4 bits for pre-emption priority
                                                   0 bits for subpriority */

#define  FREQ_1MHz                   1000000U

#define  FREQ_8MHz                   8000000U

#define  FREQ_12MHz                 12000000U

#define  FREQ_20MHz                 20000000U

#define  FREQ_100KHz                  100000U

#define  FREQ_400KHz                  400000U


//#define  USE_DMA_TX_USART1_DATA

/* USE_DMA_RX_USART1_DATA 和 USE_PWM_CTRL_RGB_STRIP_1ST 只能二选一*/
//#define  USE_DMA_RX_USART1_DATA

/* USE_DMA_TX_USART2_DATA 和 USE_PWM_CTRL_RGB_STRIP_2ND 只能二选一*/
//#define  USE_DMA_TX_USART2_DATA
                                                       
/* USE_DMA_TX_USART3_DATA 和 USE_PWM_CTRL_RGB_STRIP_3RD 只能二选一*/
//#define  USE_DMA_TX_USART3_DATA
                                                       
/* USE_DMA_RX_USART3_DATA 和 USE_PWM_CTRL_RGB_STRIP_4TH 只能二选一*/
//#define  USE_DMA_RX_USART3_DATA

  
/* USART Pin defines ---------------------------------------------------------*/
#define  USART1_Tx_Pin            GPIO_Pins_9
#define  USART1_Rx_Pin            GPIO_Pins_10
#define  USART1_GPIO_Port         GPIOA

#define  USART2_Tx_Pin            GPIO_Pins_2
#define  USART2_Rx_Pin            GPIO_Pins_3
#define  USART2_GPIO_Port         GPIOA

#define  USART3_Tx_Pin            GPIO_Pins_10
#define  USART3_Rx_Pin            GPIO_Pins_11
#define  USART3_GPIO_Port         GPIOB

/* SPI Pin defines -----------------------------------------------------------*/
#define  SPI1_SCK_Pin             GPIO_Pins_5
#define  SPI1_MISO_Pin            GPIO_Pins_6
#define  SPI1_MOSI_Pin            GPIO_Pins_7
#define  SPI1_GPIO_Port           GPIOA

#define  SPI2_SCK_Pin             GPIO_Pins_13
#define  SPI2_MISO_Pin            GPIO_Pins_14
#define  SPI2_MOSI_Pin            GPIO_Pins_15
#define  SPI2_GPIO_Port           GPIOB

/* I2C Pin defines -----------------------------------------------------------*/
#define  I2C1_SCL_Pin             GPIO_Pins_8
#define  I2C1_SDA_Pin             GPIO_Pins_9
#define  I2C1_SCL_Port            GPIOB
#define  I2C1_SDA_Port            GPIOB

#define  I2C1_OWN_ADDRESS0        0x45
#define  I2C1_OWN_ADDRESS1        0x46

/* CAN Pin defines -----------------------------------------------------------*/
#define  CAN2_RX_Pin              GPIO_Pins_5
#define  CAN2_TX_Pin              GPIO_Pins_6
#define  CAN2_GPIO_Port           GPIOB
  
/* CAN Pin defines -----------------------------------------------------------*/
#define USB_DP_PIN               GPIO_Pins_12
#define USB_DM_PIN               GPIO_Pins_11
#define USB_GPIO                 GPIOA

/* Pin defines ---------------------------------------------------------------*/
#define  MCU_TIO1_Pin             GPIO_Pins_0
#define  MCU_TIO1_GPIO_Port       GPIOA

#define  MCU_TIO2_Pin             GPIO_Pins_1
#define  MCU_TIO2_GPIO_Port       GPIOA

#define  MCU_AIO1_Pin             GPIO_Pins_0
#define  MCU_AIO1_GPIO_Port       GPIOB

#define  MCU_AIO2_Pin             GPIO_Pins_1
#define  MCU_AIO2_GPIO_Port       GPIOB

#define  EMG_SIG_Pin              GPIO_Pins_3
#define  EMG_SIG_GPIO_Port        GPIOB

#define  BKP_SIG_Pin              GPIO_Pins_4
#define  BKP_SIG_GPIO_Port        GPIOB

#define  PWR_SIG_Pin              GPIO_Pins_7
#define  PWR_SIG_GPIO_Port        GPIOB

#define  SPI1_CS_Pin              GPIO_Pins_4
#define  SPI1_CS_GPIO_Port        GPIOA

#define  SPI2_CS_Pin              GPIO_Pins_12
#define  SPI2_CS_GPIO_Port        GPIOB

#define  M485_DE_Pin              GPIO_Pins_15
#define  M485_DE_GPIO_Port        GPIOC

#define  ESP32_EN_Pin             GPIO_Pins_13
#define  ESP32_EN_GPIO_Port       GPIOC

#define  IMU_INT1_Pin             GPIO_Pins_14
#define  IMU_INT1_GPIO_Port       GPIOC

#define  BEEP_CTRL_Pin            GPIO_Pins_8
#define  BEEP_CTRL_GPIO_Port      GPIOA

#define  SYS_LED_Pin              GPIO_Pins_15
#define  SYS_LED_GPIO_Port        GPIOA


/* GPIO Pin function defines -------------------------------------------------*/
#define  SYS_LED_ON()             (SYS_LED_GPIO_Port->BRE = SYS_LED_Pin)
#define  SYS_LED_OFF()            (SYS_LED_GPIO_Port->BSRE = SYS_LED_Pin)
#define  SYS_LED_TOGGLE()         (SYS_LED_GPIO_Port->OPTDT ^= SYS_LED_Pin)

#define  M485_DE_Pin_PULL_DOWN()  (M485_DE_GPIO_Port->BRE = M485_DE_Pin)
#define  M485_DE_Pin_PULL_UP()    (M485_DE_GPIO_Port->BSRE = M485_DE_Pin)
#define  M485_DE_Pin_TOGGLE()     (M485_DE_GPIO_Port->OPTDT ^= M485_DE_Pin)

#define  ESP32_ENABLE()           (ESP32_EN_GPIO_Port->BSRE = ESP32_EN_Pin)
#define  ESP32_DISABLE()          (ESP32_EN_GPIO_Port->BRE = ESP32_EN_Pin)
#define  ESP32_TOGGLE()           (ESP32_EN_GPIO_Port->OPTDT ^= ESP32_EN_Pin)

#define  IMU_SELECT()             (SPI1_CS_GPIO_Port->BRE = SPI1_CS_Pin)
#define  IMU_DESELECT()           (SPI1_CS_GPIO_Port->BSRE = SPI1_CS_Pin)
#define  IMU_TOGGLE()             (SPI1_CS_GPIO_Port->OPTDT ^= SPI1_CS_Pin)

#define  SPI_Flash_SELECT()       (SPI2_CS_GPIO_Port->BRE = SPI2_CS_Pin)
#define  SPI_Flash_DESELECT()     (SPI2_CS_GPIO_Port->BSRE = SPI2_CS_Pin)
#define  SPI_Flash_TOGGLE()       (SPI2_CS_GPIO_Port->OPTDT ^= SPI2_CS_Pin)

#define  MCU_TIO1_PULL_DOWN()     (MCU_TIO1_GPIO_Port->BRE = MCU_TIO1_Pin)
#define  MCU_TIO1_PULL_UP()       (MCU_TIO1_GPIO_Port->BSRE = MCU_TIO1_Pin)
#define  MCU_TIO1_TOGGLE()        (MCU_TIO1_GPIO_Port->OPTDT ^= MCU_TIO1_Pin)

#define  MCU_TIO2_PULL_DOWN()     (MCU_TIO2_GPIO_Port->BRE = MCU_TIO2_Pin)
#define  MCU_TIO2_PULL_UP()       (MCU_TIO2_GPIO_Port->BSRE = MCU_TIO2_Pin)
#define  MCU_TIO2_TOGGLE()        (MCU_TIO2_GPIO_Port->OPTDT ^= MCU_TIO2_Pin)

#define  MCU_AIO1_PULL_DOWN()     (MCU_AIO1_GPIO_Port->BRE = MCU_AIO1_Pin)
#define  MCU_AIO1_PULL_UP()       (MCU_AIO1_GPIO_Port->BSRE = MCU_AIO1_Pin)
#define  MCU_AIO1_TOGGLE()        (MCU_AIO1_GPIO_Port->OPTDT ^= MCU_AIO1_Pin)

#define  MCU_AIO2_PULL_DOWN()     (MCU_AIO2_GPIO_Port->BRE = MCU_AIO2_Pin)
#define  MCU_AIO2_PULL_UP()       (MCU_AIO2_GPIO_Port->BSRE = MCU_AIO2_Pin)
#define  MCU_AIO2_TOGGLE()        (MCU_AIO2_GPIO_Port->OPTDT ^= MCU_AIO2_Pin)

#define  EMG_SIG_PULL_DOWN()      (EMG_SIG_GPIO_Port->BRE = EMG_SIG_Pin)
#define  EMG_SIG_PULL_UP()        (EMG_SIG_GPIO_Port->BSRE = EMG_SIG_Pin)
#define  EMG_SIG_TOGGLE()         (EMG_SIG_GPIO_Port->OPTDT ^= EMG_SIG_Pin)

#define  BKP_SIG_PULL_DOWN()      (BKP_SIG_GPIO_Port->BRE = BKP_SIG_Pin)
#define  BKP_SIG_PULL_UP()        (BKP_SIG_GPIO_Port->BSRE = BKP_SIG_Pin)
#define  BKP_SIG_TOGGLE()         (BKP_SIG_GPIO_Port->OPTDT ^= BKP_SIG_Pin)

#define  PWR_SIG_PULL_DOWN()      (PWR_SIG_GPIO_Port->BRE = PWR_SIG_Pin)
#define  PWR_SIG_PULL_UP()        (PWR_SIG_GPIO_Port->BSRE = PWR_SIG_Pin)
#define  PWR_SIG_TOGGLE()         (PWR_SIG_GPIO_Port->OPTDT ^= PWR_SIG_Pin)

#define  BEEP_CTRL_PULL_DOWN()    (BEEP_CTRL_GPIO_Port->BRE = BEEP_CTRL_Pin)
#define  BEEP_CTRL_PULL_UP()      (BEEP_CTRL_GPIO_Port->BSRE = BEEP_CTRL_Pin)
#define  BEEP_CTRL_TOGGLE()       (BEEP_CTRL_GPIO_Port->OPTDT ^= BEEP_CTRL_Pin)


void gpio_init(GPIO_Type* GPIOx, GPIOMode_Type GPIO_Mode, GPIOMaxSpeed_Type GPIO_MaxSpeed, uint16_t GPIO_Pins);

void System_GPIO_Init(void);

void System_USB_GPIO_init(void);

void System_PWR_ON_Beep(void);

void System_NVIC_Init(void);

void System_Misc_Init(void);

void System_EXTI_Init(void);

void System_DMA_Init(void);

void System_USART1_Init(uint32_t baud_rate);

void System_USART2_Init(uint32_t baud_rate);

void System_USART3_Init(uint32_t baud_rate);

void USART2_EN_M485_Tx(void);

void USART2_EN_M485_Rx(void);

void USB_CDC_USART_Init(uint32_t baud_rate);

void System_TMR1_Init(void);

void System_TMR2_Init(void);

void System_TMR3_Init(void);

void System_TMR6_Init(void);

void System_DAC_OUT1_Init(void);

void System_ADC1_IN0_Init(void);

void System_SPI1_Init(uint32_t baud_rate);

void System_SPI2_Init(uint32_t baud_rate);

void System_I2C1_Init(uint32_t baud_rate);

void System_CAN1_Init(CAN_Baudrate_Typedef baudrate);

void System_CAN2_Init(CAN_Baudrate_Typedef baudrate);

uint8_t System_CAN_Start(CAN_Type* can_periph);

uint8_t System_CAN_Stop(CAN_Type* can_periph);

void System_WWDG_Init(void);

void System_WWDG_Disable(void);

void EXTI3_IRQ_Callback(void);

void EXTI4_IRQ_Callback(void);

void EXTI7_IRQ_Callback(void);

void EXTI15_10_IRQ_Callback(void);

bool System_Start_Flag_Get(void);


#ifdef __cplusplus
}
#endif

#endif /* HWCONF_H */
