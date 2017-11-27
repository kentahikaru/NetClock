/**
  ******************************************************************************
  * @file    stm32746g_discovery.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file contains definitions for STM32746G_DISCOVERY's LEDs,
  *          push-buttons and COM ports hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32746G_DISCOVERY_H
#define __STM32746G_DISCOVERY_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
#include "stm32f7xx_usart.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32746G_DISCOVERY
  * @{
  */
      
/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Exported_Types STM32746G_DISCOVERY_LOW_LEVEL Exported Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED_GREEN = LED1,
}Led_TypeDef;

typedef enum 
{  
  BUTTON_WAKEUP = 0,
  BUTTON_TAMPER = 1,
  BUTTON_KEY = 2
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

typedef enum 
{
  COM1 = 0,
  COM2 = 1
}COM_TypeDef;
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Exported_Constants STM32746G_DISCOVERY_LOW_LEVEL Exported Constants
  * @{
  */ 

/** 
  * @brief  Define for STM32746G_DISCOVERY board
  */ 
#if !defined (USE_STM32746G_DISCO)
 #define USE_STM32746G_DISCO
#endif

/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL_LED
  * @{
  */

#define LEDn                             ((uint8_t)1)

#define LED1_GPIO_PORT                   GPIOI
#define LED1_GPIO_CLK_ENABLE()           RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE)
#define LED1_GPIO_CLK_DISABLE()          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, DISABLE)
#define LED1_PIN                         GPIO_Pin_1

/**
  * @}
  */

/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL_BUTTON
  * @{
  */ 
#define BUTTONn                             ((uint8_t)3) 

/**
  * @brief Wakeup push-button
  */
#define WAKEUP_BUTTON_PIN                   GPIO_Pin_11
#define WAKEUP_BUTTON_GPIO_PORT             GPIOI
#define WAKEUP_BUTTON_GPIO_CLK_ENABLE()     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE)
#define WAKEUP_BUTTON_GPIO_CLK_DISABLE()    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, DISABLE)
#define WAKEUP_BUTTON_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOI
#define WAKEUP_BUTTON_EXTI_PIN_SOURCE       EXTI_PinSource11
#define WAKEUP_BUTTON_EXTI_LINE             EXTI_Line11
#define WAKEUP_BUTTON_EXTI_IRQn             EXTI15_10_IRQn 

/**
  * @brief Tamper push-button
  */
#define TAMPER_BUTTON_PIN                    GPIO_Pin_11
#define TAMPER_BUTTON_GPIO_PORT              GPIOI
#define TAMPER_BUTTON_GPIO_CLK_ENABLE()      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE)
#define TAMPER_BUTTON_GPIO_CLK_DISABLE()     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, DISABLE)
#define TAMPER_BUTTON_EXTI_PORT_SOURCE       EXTI_PortSourceGPIOI
#define TAMPER_BUTTON_EXTI_PIN_SOURCE        EXTI_PinSource11
#define TAMPER_BUTTON_EXTI_LINE              EXTI_Line11
#define TAMPER_BUTTON_EXTI_IRQn              EXTI15_10_IRQn

/**
  * @brief Key push-button
  */
#define KEY_BUTTON_PIN                       GPIO_Pin_11
#define KEY_BUTTON_GPIO_PORT                 GPIOI
#define KEY_BUTTON_GPIO_CLK_ENABLE()         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE)
#define KEY_BUTTON_GPIO_CLK_DISABLE()        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, DISABLE)
#define KEY_BUTTON_EXTI_PORT_SOURCE          EXTI_PortSourceGPIOI
#define KEY_BUTTON_EXTI_PIN_SOURCE           EXTI_PinSource11
#define KEY_BUTTON_EXTI_LINE                 EXTI_Line11
#define KEY_BUTTON_EXTI_IRQn                 EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) WAKEUP_BUTTON_GPIO_CLK_ENABLE(); else\
                                                   if((__INDEX__) == 1) TAMPER_BUTTON_GPIO_CLK_ENABLE(); else\
                                                   KEY_BUTTON_GPIO_CLK_ENABLE(); } while(0)											   

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    (((__INDEX__) == 0) ? WAKEUP_BUTTON_GPIO_CLK_DISABLE() :\
                                                ((__INDEX__) == 1) ? TAMPER_BUTTON_GPIO_CLK_DISABLE() : KEY_BUTTON_GPIO_CLK_DISABLE())

/**
  * @}
  */

/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL_SIGNAL
  * @{
  */
#define SIGNALn                             ((uint8_t)1)

/**
  * @brief SD-detect signal
  */
#define SD_DETECT_PIN                        GPIO_Pin_13
#define SD_DETECT_GPIO_PORT                  GPIOC
#define SD_DETECT_GPIO_CLK_ENABLE()          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define SD_DETECT_GPIO_CLK_DISABLE()         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, DISABLE)
#define SD_DETECT_EXTI_IRQn                  EXTI15_10_IRQn

//#define SDMMC_FIFO_ADDRESS                ((uint32_t)0x40012C80)

#define SD_SDMMC_DMA                   DMA2
#define SD_SDMMC_DMA_CLK               RCC_AHB1Periph_DMA2
 
#define SD_SDMMC_DMA_STREAM3	          3
//#define SD_SDMMC_DMA_STREAM6           6

#ifdef SD_SDMMC_DMA_STREAM3
 #define SD_SDMMC_DMA_STREAM            DMA2_Stream3
 #define SD_SDMMC_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDMMC_DMA_FLAG_FEIF         DMA_FLAG_FEIF3
 #define SD_SDMMC_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF3
 #define SD_SDMMC_DMA_FLAG_TEIF         DMA_FLAG_TEIF3
 #define SD_SDMMC_DMA_FLAG_HTIF         DMA_FLAG_HTIF3
 #define SD_SDMMC_DMA_FLAG_TCIF         DMA_FLAG_TCIF3 
 #define SD_SDMMC_DMA_IRQn              DMA2_Stream3_IRQn
 #define SD_SDMMC_DMA_IRQHANDLER        DMA2_Stream3_IRQHandler 
#elif defined SD_SDMMC_DMA_STREAM6
 #define SD_SDMMC_DMA_STREAM            DMA2_Stream6
 #define SD_SDMMC_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDMMC_DMA_FLAG_FEIF         DMA_FLAG_FEIF6
 #define SD_SDMMC_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF6
 #define SD_SDMMC_DMA_FLAG_TEIF         DMA_FLAG_TEIF6
 #define SD_SDMMC_DMA_FLAG_HTIF         DMA_FLAG_HTIF6
 #define SD_SDMMC_DMA_FLAG_TCIF         DMA_FLAG_TCIF6 
 #define SD_SDMMC_DMA_IRQn              DMA2_Stream6_IRQn
 #define SD_SDMMC_DMA_IRQHANDLER        DMA2_Stream6_IRQHandler
#endif /* SD_SDMMC_DMA_STREAM3 */

/**
  * @brief Touch screen interrupt signal
  */
#define TS_INT_PIN                           GPIO_Pin_13
#define TS_INT_GPIO_PORT                     GPIOI
#define TS_INT_GPIO_CLK_ENABLE()             RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE)
#define TS_INT_GPIO_CLK_DISABLE()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, DISABLE)

#define TS_INT_EXTI_LINE                     EXTI_Line13
#define TS_INT_EXTI_PORT_SOURCE              EXTI_PortSourceGPIOI
#define TS_INT_EXTI_PIN_SOURCE               EXTI_PinSource13
#define TS_INT_EXTI_IRQn                     EXTI15_10_IRQn

#define TS_INT_EXTI_IRQHandler               EXTI15_10_IRQHandler

/**
  * @}
  */ 

/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL_COM
  * @{
  */
#define COMn                             ((uint8_t)1)

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define DISCOVERY_COM1                          USART1
#define DISCOVERY_COM1_CLK_ENABLE()             RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define DISCOVERY_COM1_CLK_DISABLE()            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE)

#define DISCOVERY_COM1_TX_PIN                   GPIO_Pin_9
#define DISCOVERY_COM1_TX_GPIO_PORT             GPIOA
#define DISCOVERY_COM1_TX_GPIO_CLK_ENABLE()     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define DISCOVERY_COM1_TX_GPIO_CLK_DISABLE()    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE)
#define DISCOVERY_COM1_TX_AF                    GPIO_AF7_USART1
#define DISCOVERY_COM1_TX_PIN_SOURCE            GPIO_PinSource9

#define DISCOVERY_COM1_RX_PIN                   GPIO_Pin_7
#define DISCOVERY_COM1_RX_GPIO_PORT             GPIOB
#define DISCOVERY_COM1_RX_GPIO_CLK_ENABLE()     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define DISCOVERY_COM1_RX_GPIO_CLK_DISABLE()    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE)
#define DISCOVERY_COM1_RX_AF                    GPIO_AF7_USART1
#define DISCOVERY_COM1_RX_PIN_SOURCE            GPIO_PinSource7

#define DISCOVERY_COM1_IRQn                     USART1_IRQn

#define DISCOVERY_COMx_CLK_ENABLE(__INDEX__)            do { if((__INDEX__) == COM1) DISCOVERY_COM1_CLK_ENABLE(); } while(0)
#define DISCOVERY_COMx_CLK_DISABLE(__INDEX__)           do { if((__INDEX__) == COM1) DISCOVERY_COM1_CLK_DISABLE(); } while(0)

#define DISCOVERY_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) DISCOVERY_COM1_TX_GPIO_CLK_ENABLE(); } while(0)
#define DISCOVERY_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) DISCOVERY_COM1_TX_GPIO_CLK_DISABLE(); } while(0)

#define DISCOVERY_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) DISCOVERY_COM1_RX_GPIO_CLK_ENABLE(); } while(0)
#define DISCOVERY_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) DISCOVERY_COM1_RX_GPIO_CLK_DISABLE(); } while(0)

/* Exported constant IO ------------------------------------------------------*/

#define LCD_I2C_ADDRESS                  ((uint16_t)0x70)
#define CAMERA_I2C_ADDRESS               ((uint16_t)0x60)
#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)
#define EEPROM_I2C_ADDRESS_A01           ((uint16_t)0xA0)
#define EEPROM_I2C_ADDRESS_A02           ((uint16_t)0xA6)
#define TS_I2C_ADDRESS                   ((uint16_t)0x70)

/* I2C clock speed configuration (in Hz) 
   WARNING: 
   Make sure that this define is not already declared in other files (ie. 
   stm32746g_discovery.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                       ((uint32_t)100000)
#endif /* I2C_SPEED */

/* User can use this section to tailor I2Cx/I2Cx instance used and associated 
   resources */
/* Definition for AUDIO and LCD I2Cx resources */
#define DISCOVERY_AUDIO_I2Cx                             I2C3
#define DISCOVERY_AUDIO_I2Cx_CLK_ENABLE()                RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE)
#define DISCOVERY_AUDIO_DMAx_CLK_ENABLE()                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE)
#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE)

#define DISCOVERY_AUDIO_I2Cx_FORCE_RESET()               RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE)
#define DISCOVERY_AUDIO_I2Cx_RELEASE_RESET()             RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, DISABLE)

/* Definition for I2Cx Pins */
#define DISCOVERY_AUDIO_I2Cx_SCL_PIN                     GPIO_Pin_7
#define DISCOVERY_AUDIO_I2Cx_SCL_GPIO_PORT               GPIOH
#define DISCOVERY_AUDIO_I2Cx_SCL_AF                      GPIO_AF4_I2C3
#define DISCOVERY_AUDIO_I2Cx_SCL_ENABLE()                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE)
#define DISCOVERY_AUDIO_I2Cx_SCL_PIN_SOURCE              GPIO_PinSource7

#define DISCOVERY_AUDIO_I2Cx_SDA_PIN                     GPIO_Pin_8
#define DISCOVERY_AUDIO_I2Cx_SDA_GPIO_PORT               GPIOH
#define DISCOVERY_AUDIO_I2Cx_SDA_AF                      GPIO_AF4_I2C3
#define DISCOVERY_AUDIO_I2Cx_SDA_ENABLE()                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE)
#define DISCOVERY_AUDIO_I2Cx_SDA_PIN_SOURCE              GPIO_PinSource8
   
/* I2C interrupt requests */
#define DISCOVERY_AUDIO_I2Cx_EV_IRQn                     I2C3_EV_IRQn
#define DISCOVERY_AUDIO_I2Cx_ER_IRQn                     I2C3_ER_IRQn

/* Definition for external, camera and Arduino connector I2Cx resources */
#define DISCOVERY_EXT_I2Cx                               I2C1
#define DISCOVERY_EXT_I2Cx_CLK_ENABLE()                  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE)
#define DISCOVERY_EXT_DMAx_CLK_ENABLE()                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE)
#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)

#define DISCOVERY_EXT_I2Cx_FORCE_RESET()                 RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE)
#define DISCOVERY_EXT_I2Cx_RELEASE_RESET()               RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE)

/* Definition for I2Cx Pins */
#define DISCOVERY_EXT_I2Cx_SCL_PIN                       GPIO_Pin_8
#define DISCOVERY_EXT_I2Cx_SCL_GPIO_PORT                 GPIOB
#define DISCOVERY_EXT_I2Cx_SCL_AF                        GPIO_AF4_I2C1 
#define DISCOVERY_EXT_I2Cx_SCL_ENABLE()                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define DISCOVERY_EXT_I2Cx_SCL_PIN_SOURCE                GPIO_PinSource8

#define DISCOVERY_EXT_I2Cx_SDA_PIN                       GPIO_Pin_9
#define DISCOVERY_EXT_I2Cx_SDA_GPIO_PORT                 GPIOB
#define DISCOVERY_EXT_I2Cx_SDA_AF                        GPIO_AF4_I2C1
#define DISCOVERY_EXT_I2Cx_SDA_ENABLE()                  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define DISCOVERY_EXT_I2Cx_SDA_PIN_SOURCE                GPIO_PinSource9
     
/* I2C interrupt requests */
#define DISCOVERY_EXT_I2Cx_EV_IRQn                       I2C1_EV_IRQn
#define DISCOVERY_EXT_I2Cx_ER_IRQn                       I2C1_ER_IRQn

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated from APB1 source clock = 50 MHz */
/* Due to the big MOFSET capacity for adapting the camera level the rising time is very large (>1us) */
/* 0x40912732 takes in account the big rising and aims a clock of 100khz */
/* this value might be adapted when next Rev Birdie board is available */
#ifndef DISCOVERY_I2Cx_TIMING  
#define DISCOVERY_I2Cx_TIMING                      ((uint32_t)0x40912732)  
#endif /* DISCOVERY_I2Cx_TIMING */

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Exported_Macros STM32746G_DISCOVERY_LOW_LEVEL Exported Macros
  * @{
  */  
/**
  * @}
  */ 

/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL_Exported_Functions
  * @{
  */
uint32_t  BSP_GetVersion(void);
void      BSP_LED_Init(Led_TypeDef Led);
void      BSP_LED_DeInit(Led_TypeDef Led);
void      BSP_LED_On(Led_TypeDef Led);
void      BSP_LED_Off(Led_TypeDef Led);
void      BSP_LED_Toggle(Led_TypeDef Led);
void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void      BSP_PB_DeInit(Button_TypeDef Button);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);
void      BSP_COM_Init(COM_TypeDef COM, USART_InitTypeDef * USART_InitStruct);
void      BSP_COM_DeInit(COM_TypeDef COM);

void      BSP_SD_LowLevel_DeInit(void);
void      BSP_SD_LowLevel_Init(void);
void      BSP_SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
void      BSP_SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);

void      STD_Delay(uint32_t Delay);



/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32746G_DISCOVERY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
