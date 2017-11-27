/**
  ******************************************************************************
  * @file    stm32746g_discovery.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file provides a set of firmware functions to manage LEDs, 
  *          push-buttons and COM ports available on STM32746G-Discovery
  *          board(MB1191) from STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32746g_discovery.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32746G_DISCOVERY
  * @{
  */

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL STM32746G_DISCOVERY_LOW_LEVEL
  * @{
  */

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Private_TypesDefinitions STM32746G_DISCOVERY_LOW_LEVEL Private Types Definitions
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Private_Defines STM32746G_DISCOVERY_LOW_LEVEL Private Defines
  * @{
  */
/**
 * @brief STM32746G DISCOVERY BSP Driver version number V1.0.0
   */
#define __STM32746G_DISCO_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32746G_DISCO_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32746G_DISCO_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32746G_DISCO_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32746G_DISCO_BSP_VERSION         ((__STM32746G_DISCO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32746G_DISCO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32746G_DISCO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32746G_DISCO_BSP_VERSION_RC))
/**
  * @}
  */

#define I2C_REGADDR_SIZE_8BIT           1
#define I2C_REGADDR_SIZE_16BIT          2

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Private_Macros STM32746G_DISCOVERY_LOW_LEVEL Private Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Private_Variables STM32746G_DISCOVERY_LOW_LEVEL Private Variables
  * @{
  */

const uint32_t GPIO_PIN[LEDn] = {LED1_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {WAKEUP_BUTTON_GPIO_PORT,
                                      TAMPER_BUTTON_GPIO_PORT,
                                      KEY_BUTTON_GPIO_PORT};

const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN,
                                      TAMPER_BUTTON_PIN,
                                      KEY_BUTTON_PIN};

const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {WAKEUP_BUTTON_EXTI_PORT_SOURCE,
                                             TAMPER_BUTTON_EXTI_PORT_SOURCE,
                                             KEY_BUTTON_EXTI_PORT_SOURCE};
								 
const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {WAKEUP_BUTTON_EXTI_PIN_SOURCE,
                                            TAMPER_BUTTON_EXTI_PIN_SOURCE,
                                            KEY_BUTTON_EXTI_PIN_SOURCE};

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {WAKEUP_BUTTON_EXTI_LINE,
                                            TAMPER_BUTTON_EXTI_LINE,
                                            KEY_BUTTON_EXTI_LINE};

const uint16_t BUTTON_IRQn[BUTTONn] = {WAKEUP_BUTTON_EXTI_IRQn,
                                       TAMPER_BUTTON_EXTI_IRQn,
                                       KEY_BUTTON_EXTI_IRQn};

USART_TypeDef* COM_USART[COMn] = {DISCOVERY_COM1};

GPIO_TypeDef* COM_TX_PORT[COMn] = {DISCOVERY_COM1_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {DISCOVERY_COM1_RX_GPIO_PORT};

const uint16_t COM_TX_PIN[COMn] = {DISCOVERY_COM1_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {DISCOVERY_COM1_RX_PIN};

const uint16_t COM_TX_AF[COMn] = {DISCOVERY_COM1_TX_AF};

const uint16_t COM_RX_AF[COMn] = {DISCOVERY_COM1_RX_AF};

const uint16_t COM_TX_PIN_SOURCE[COMn] = {DISCOVERY_COM1_TX_PIN_SOURCE};

const uint16_t COM_RX_PIN_SOURCE[COMn] = {DISCOVERY_COM1_RX_PIN_SOURCE};

static I2C_TypeDef * hI2cAudioHandler = DISCOVERY_AUDIO_I2Cx;
static I2C_TypeDef * hI2cExtHandler = DISCOVERY_EXT_I2Cx;

/**
  * @}
  */

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes STM32746G_DISCOVERY_LOW_LEVEL Private Function Prototypes
  * @{
  */
static void     I2Cx_MspInit(I2C_TypeDef *i2c_handler);
static void     I2Cx_Init(I2C_TypeDef *i2c_handler);

static ErrorStatus I2Cx_Write(I2C_TypeDef *I2C_Handler, uint8_t DeviceAddr, uint16_t RegAddr, uint8_t RegAddrSize, uint8_t* pBuffer, uint16_t NumByteToRead, uint32_t Timeout);
static ErrorStatus I2Cx_Read(I2C_TypeDef *I2C_Handler, uint8_t DeviceAddr, uint16_t RegAddr, uint8_t RegAddrSize, uint8_t* pBuffer, uint16_t NumByteToRead, uint32_t Timeout);
static ErrorStatus I2Cx_ReadMultiple(I2C_TypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
static ErrorStatus I2Cx_WriteMultiple(I2C_TypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
static ErrorStatus I2Cx_IsDeviceReady(I2C_TypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials);
static ErrorStatus I2Cx_IsDeviceReadyTimeout(I2C_TypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
static void        I2Cx_Error(I2C_TypeDef *i2c_handler, uint8_t Addr);
static ErrorStatus I2Cx_TIMEOUT_UserCallback(void);

/* AUDIO IO functions */
void            AUDIO_IO_Init(void);
void            AUDIO_IO_DeInit(void);
void            AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint16_t        AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void            AUDIO_IO_Delay(uint32_t Delay);

/* TOUCHSCREEN IO functions */
void            TS_IO_Init(void);
void            TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         TS_IO_Read(uint8_t Addr, uint8_t Reg);
void            TS_IO_Delay(uint32_t Delay);

/* CAMERA IO functions */
void            CAMERA_IO_Init(void);
void            CAMERA_Delay(uint32_t Delay);
void            CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         CAMERA_IO_Read(uint8_t Addr, uint8_t Reg);

/* I2C EEPROM IO function */
void                EEPROM_IO_Init(void);
ErrorStatus   EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
ErrorStatus   EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
ErrorStatus   EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
/**
  * @}
  */

/** @defgroup STM32746G_DISCOVERY_LOW_LEVEL_Exported_Functions STM32746G_DISCOVERY_LOW_LEVELSTM32746G_DISCOVERY_LOW_LEVEL Exported Functions
  * @{
  */ 

  /**
  * @brief  This method returns the STM32746G DISCOVERY BSP Driver revision
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32746G_DISCO_BSP_VERSION;
}

/**
  * @brief  Configures LED on GPIO.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_TypeDef*     gpio_led;

  if (Led == LED1)
  {
    gpio_led = LED1_GPIO_PORT;
    /* Enable the GPIO_LED clock */
    LED1_GPIO_CLK_ENABLE();

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  
    GPIO_Init(gpio_led, &GPIO_InitStructure);
    
    /* By default, turn off LED */
    GPIO_WriteBit(gpio_led, GPIO_PIN[Led], Bit_RESET);
  }
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  * @note Led DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_TypeDef*     gpio_led;

  if (Led == LED1)
  {
    gpio_led = LED1_GPIO_PORT;
    
    /* Turn off LED */
    GPIO_WriteBit(gpio_led, GPIO_PIN[Led], Bit_RESET);
    
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    
    GPIO_Init(gpio_led, &GPIO_InitStructure);
  }
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  GPIO_TypeDef*     gpio_led;

  if (Led == LED1)	/* Switch On LED connected to GPIO */
  {
    gpio_led = LED1_GPIO_PORT;
    GPIO_WriteBit(gpio_led, GPIO_PIN[Led], Bit_SET);
  }
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  GPIO_TypeDef*     gpio_led;

  if (Led == LED1) /* Switch Off LED connected to GPIO */
  {
    gpio_led = LED1_GPIO_PORT;
    GPIO_WriteBit(gpio_led, GPIO_PIN[Led], Bit_RESET);
  }
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  GPIO_TypeDef*     gpio_led;

  if (Led == LED1)	/* Toggle LED connected to GPIO */
  {
    gpio_led = LED1_GPIO_PORT;
    GPIO_ToggleBits(gpio_led, GPIO_PIN[Led]);
  }
}

/**
  * @brief  Configures button GPIO and EXTI Line.
  * @param  Button: Button to be configured
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_WAKEUP: Wakeup Push Button 
  *            @arg  BUTTON_TAMPER: Tamper Push Button  
  *            @arg  BUTTON_KEY: Key Push Button
  * @param  ButtonMode: Button mode
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *            @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line 
  *                                    with interrupt generation capability
  * @note On STM32746G-Discovery board, the three buttons (Wakeup, Tamper and key buttons)
  *       are mapped on the same push button named "User"
  *       on the board serigraphy.
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the BUTTON clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);
  
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);    
    
    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    
    if(Button != BUTTON_WAKEUP)
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    }
    else
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    }
    
    EXTI_Init(&EXTI_InitStructure);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Push Button DeInit.
  * @param  Button: Button to be configured
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_WAKEUP: Wakeup Push Button 
  *            @arg  BUTTON_TAMPER: Tamper Push Button  
  *            @arg  BUTTON_KEY: Key Push Button
  * @note On STM32746G-Discovery board, the three buttons (Wakeup, Tamper and key buttons) 
  *       are mapped on the same push button named "User"
  *       on the board serigraphy.
  * @note PB DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
  
  /* Disable Button EXTI Interrupt and NVIC */
  EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);
}


/**
  * @brief  Returns the selected button state.
  * @param  Button: Button to be checked
  *          This parameter can be one of the following values:
  *            @arg  BUTTON_WAKEUP: Wakeup Push Button 
  *            @arg  BUTTON_TAMPER: Tamper Push Button 
  *            @arg  BUTTON_KEY: Key Push Button
  * @note On STM32746G-Discovery board, the three buttons (Wakeup, Tamper and key buttons) 
  *       are mapped on the same push button named "User"
  *       on the board serigraphy.
  * @retval The Button GPIO pin value
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Configures COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1 
  *            @arg  COM2 
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_Init(COM_TypeDef COM, USART_InitTypeDef * USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  DISCOVERY_COMx_TX_GPIO_CLK_ENABLE(COM);
  DISCOVERY_COMx_RX_GPIO_CLK_ENABLE(COM);

  /* Enable USART clock */
  DISCOVERY_COMx_CLK_ENABLE(COM);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function */
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* Connect COM_TX_PIN to UART_TX*/
  GPIO_PinAFConfig(COM_TX_PORT[COM],
                   COM_TX_PIN_SOURCE[COM],
                   COM_TX_AF[COM]);
  
  /* Connect COM_RX_PIN to UART_RX*/
  GPIO_PinAFConfig(COM_RX_PORT[COM],
                   COM_RX_PIN_SOURCE[COM],
                   COM_RX_AF[COM]);
  
  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);
  
  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}

/**
  * @brief  DeInit COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1 
  *            @arg  COM2 
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_DeInit(COM_TypeDef COM)
{
  /* USART configuration */
  USART_DeInit(COM_USART[COM]);

  /* Enable USART clock */
  DISCOVERY_COMx_CLK_DISABLE(COM);

  /* DeInit GPIO pins can be done in the application 
     (by surcharging this __weak function) */

  /* GPIO pins clock, DMA clock can be shut down in the application 
     by surcharging this __weak function */
}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* I2C Routines *********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
static void I2Cx_MspInit(I2C_TypeDef *i2c_handler)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  if (i2c_handler == DISCOVERY_AUDIO_I2Cx)
  {
    /* AUDIO and LCD I2C MSP init */

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    DISCOVERY_AUDIO_I2Cx_SCL_ENABLE();
    DISCOVERY_AUDIO_I2Cx_SDA_ENABLE();
    
    /* Connect PXx to I2C_SCL*/
    GPIO_PinAFConfig(DISCOVERY_AUDIO_I2Cx_SCL_GPIO_PORT,
                     DISCOVERY_AUDIO_I2Cx_SCL_PIN_SOURCE,
                     DISCOVERY_AUDIO_I2Cx_SCL_AF);
    
    /* Connect PXx to I2C_SDA*/
    GPIO_PinAFConfig(DISCOVERY_AUDIO_I2Cx_SDA_GPIO_PORT,
                     DISCOVERY_AUDIO_I2Cx_SDA_PIN_SOURCE,
                     DISCOVERY_AUDIO_I2Cx_SDA_AF); 
  
    /* Configure I2C Tx as alternate function */
    GPIO_InitStructure.GPIO_Pin = DISCOVERY_AUDIO_I2Cx_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(DISCOVERY_AUDIO_I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

    /* Configure I2C Rx as alternate function */
    GPIO_InitStructure.GPIO_Pin = DISCOVERY_AUDIO_I2Cx_SDA_PIN;
    GPIO_Init(DISCOVERY_AUDIO_I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
    
    /*** Configure the I2C peripheral ***/
    /* Enable I2C clock */
    DISCOVERY_AUDIO_I2Cx_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    //DISCOVERY_AUDIO_I2Cx_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    //DISCOVERY_AUDIO_I2Cx_RELEASE_RESET();

    /* Enable and set I2Cx Interrupt to a lower priority */
    NVIC_InitStructure.NVIC_IRQChannel = DISCOVERY_AUDIO_I2Cx_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable and set I2Cx Interrupt to a lower priority */
    NVIC_InitStructure.NVIC_IRQChannel = DISCOVERY_AUDIO_I2Cx_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);
  }
  else
  {
    /* External, camera and Arduino connector I2C MSP init */

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

    /* Connect PXx to I2C_SCL*/
    GPIO_PinAFConfig(DISCOVERY_EXT_I2Cx_SCL_GPIO_PORT,
                     DISCOVERY_EXT_I2Cx_SCL_PIN_SOURCE,
                     DISCOVERY_EXT_I2Cx_SCL_AF);
    
    /* Connect PXx to I2C_SDA*/
    GPIO_PinAFConfig(DISCOVERY_EXT_I2Cx_SDA_GPIO_PORT,
                     DISCOVERY_EXT_I2Cx_SDA_PIN_SOURCE,
                     DISCOVERY_EXT_I2Cx_SDA_AF);
    
    /* Configure I2C Tx as alternate function */
    GPIO_InitStructure.GPIO_Pin = DISCOVERY_EXT_I2Cx_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(DISCOVERY_EXT_I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

    /* Configure I2C Rx as alternate function */
    GPIO_InitStructure.GPIO_Pin = DISCOVERY_EXT_I2Cx_SDA_PIN;
    GPIO_Init(DISCOVERY_EXT_I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
    
    /*** Configure the I2C peripheral ***/
    /* Enable I2C clock */
    DISCOVERY_EXT_I2Cx_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    //DISCOVERY_EXT_I2Cx_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    //DISCOVERY_EXT_I2Cx_RELEASE_RESET();
    
    /* Enable and set I2Cx Interrupt to a lower priority */
    NVIC_InitStructure.NVIC_IRQChannel = DISCOVERY_EXT_I2Cx_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable and set I2Cx Interrupt to a lower priority */
    NVIC_InitStructure.NVIC_IRQChannel = DISCOVERY_EXT_I2Cx_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);
  }
}

/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
static void I2Cx_Init(I2C_TypeDef *i2c_handler)
{
  I2C_InitTypeDef I2C_InitStructure;
  I2C_TypeDef * I2Cx;
  
  if((i2c_handler->CR1 & I2C_CR1_PE) == 0)
  {
    if (i2c_handler == DISCOVERY_AUDIO_I2Cx)
    {
      /* Audio and LCD I2C configuration */
      I2Cx = DISCOVERY_AUDIO_I2Cx;
    }
    else
    {
      /* External, camera and Arduino connector  I2C configuration */
      I2Cx = DISCOVERY_EXT_I2Cx;
    }
    I2C_InitStructure.I2C_Timing = DISCOVERY_I2Cx_TIMING;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    
    RCC_I2CClockSourceConfig(I2Cx, RCC_I2CCLKSource_APB);
    
    /* Init the I2C */
    I2Cx_MspInit(I2Cx);
    
    /* Initialize the I2C peripheral */
    I2C_Init(I2Cx, &I2C_InitStructure);
    
    /* Enable the I2C peripheral */
    I2C_Cmd(I2Cx, ENABLE);
  }
}

/**
  * @brief  Writes one byte to the I2Cx.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the I2Cx register to be written.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the LSM303DLH.
  * @retval I2Cx Status
  */
static ErrorStatus I2Cx_Write(I2C_TypeDef *I2C_Handler, uint8_t DeviceAddr, uint16_t RegAddr, uint8_t RegAddrSize, uint8_t* pBuffer, uint16_t NumByteToRead, uint32_t Timeout)
{
  /* Test on BUSY Flag */
  uint8_t RegAddrCount = 0;
  uint32_t I2Cx_Timeout = Timeout;
  while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_BUSY) != RESET)
  {
    if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C_Handler, DeviceAddr, RegAddrSize, I2C_Reload_Mode, I2C_Generate_Start_Write);
    
  while ( RegAddrSize > RegAddrCount )
  {
    /* Wait until TXIS flag is set */
    I2Cx_Timeout = Timeout;  
    while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_TXIS) == RESET)   
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
    
    /* Send Register address */
    if( RegAddrSize == I2C_REGADDR_SIZE_16BIT )
    {
      if( RegAddrCount == 0 )
      {
        I2C_SendData(I2C_Handler, ((uint8_t)((uint16_t)((RegAddr) & (uint16_t)(0x00FF)))) );
      }
      else if( RegAddrCount == 1 )
      {
        I2C_SendData(I2C_Handler, ((uint8_t)((uint16_t)(((uint16_t)((RegAddr) & (uint16_t)(0xFF00))) >> 8))) );
      }
    }
    else if( RegAddrSize == I2C_REGADDR_SIZE_8BIT )
    {
      I2C_SendData(I2C_Handler, ((uint8_t)((uint16_t)((RegAddr) & (uint16_t)(0x00FF)))) );
    }
    
    /* Wait until TCR flag is set */
    I2Cx_Timeout = Timeout;
    while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_TCR) == RESET)
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
    
    RegAddrCount++;
  } 
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C_Handler, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_No_StartStop);

  while (NumByteToRead)
  {
    /* Wait until TXIS flag is set */
    I2Cx_Timeout = Timeout;
    while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_TXIS) == RESET)
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
    
    /* Write data to TXDR */
    I2C_SendData(I2C_Handler, *pBuffer);
  
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
    
    /* Decrement the read bytes counter */
    NumByteToRead--;
  } 
  
  /* Wait until STOPF flag is set */
  I2Cx_Timeout = Timeout;
  while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_STOPF) == RESET)
  {
    if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
  }
  
  /* Clear STOPF flag */
  I2C_ClearFlag(I2C_Handler, I2C_ICR_STOPCF);
  
  return SUCCESS;
}

/**
  * @brief  Reads a block of data from the I2Cx.
  * @param  DeviceAddr : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
  * @param  RegAddr : specifies the I2Cx internal address register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LSM303DLH.
  * @param  NumByteToRead : number of bytes to read from the LSM303DLH ( NumByteToRead >1  only for the Mgnetometer readinf).
  * @retval I2Cx register value
  */
static ErrorStatus I2Cx_Read(I2C_TypeDef *I2C_Handler, uint8_t DeviceAddr, uint16_t RegAddr, uint8_t RegAddrSize, uint8_t* pBuffer, uint16_t NumByteToRead, uint32_t Timeout)
{    
  /* Test on BUSY Flag */
  uint32_t I2Cx_Timeout = Timeout;
  uint8_t RegAddrCount = 0;
  while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_BUSY) != RESET)
  {
    if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C_Handler, DeviceAddr, RegAddrSize, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  
  while ( RegAddrSize > RegAddrCount )
  {
    /* Wait until TXIS flag is set */
    I2Cx_Timeout = Timeout;  
    while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_TXIS) == RESET)   
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
    
    /* Send Register address */
    if( RegAddrSize == I2C_REGADDR_SIZE_16BIT )
    {
      if( RegAddrCount == 0 )
      {
        I2C_SendData(I2C_Handler, ((uint8_t)((uint16_t)((RegAddr) & (uint16_t)(0x00FF)))) );
      }
      else if( RegAddrCount == 1 )
      {
        I2C_SendData(I2C_Handler, ((uint8_t)((uint16_t)(((uint16_t)((RegAddr) & (uint16_t)(0xFF00))) >> 8))) );
      }
    }
    else if( RegAddrSize == I2C_REGADDR_SIZE_8BIT )
    {
      I2C_SendData(I2C_Handler, ((uint8_t)((uint16_t)((RegAddr) & (uint16_t)(0x00FF)))) );
    }
    
    /* Wait until TC flag is set */
    I2Cx_Timeout = Timeout;
    while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_TC) == RESET)
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
    
    RegAddrCount++;
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C_Handler, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  
  /* Wait until all data are received */
  while (NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    I2Cx_Timeout = Timeout;
    while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_RXNE) == RESET)    
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
    
    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(I2C_Handler);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
    
    /* Decrement the read bytes counter */
    NumByteToRead--;
  } 
  
  /* Wait until STOPF flag is set */
  I2Cx_Timeout = Timeout;
  while(I2C_GetFlagStatus(I2C_Handler, I2C_ISR_STOPF) == RESET)   
  {
    if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
  }
  
  /* Clear STOPF flag */
  I2C_ClearFlag(I2C_Handler, I2C_ICR_STOPCF);
  
  /* If all operations OK */
  return SUCCESS;  
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static ErrorStatus I2Cx_IsDeviceReadyTimeout(I2C_TypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  __IO uint32_t I2C_Trials = 0;
  
  /* Test on BUSY Flag */
  uint32_t I2Cx_Timeout = Timeout;
  while(I2C_GetFlagStatus(hi2c, I2C_ISR_BUSY) != RESET)
  {
    if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
  }
  
  do
  {
    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(hi2c, DevAddress, 0, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
      
    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set or a NACK flag is set*/
    uint32_t I2Cx_Timeout = Timeout;
    while((I2C_GetFlagStatus(hi2c, I2C_FLAG_STOPF) == RESET) && (I2C_GetFlagStatus(hi2c, I2C_FLAG_NACKF) == RESET) )
    {
      if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
    }
      
    /* Check if the NACKF flag has not been set */
    if ( I2C_GetFlagStatus(hi2c, I2C_FLAG_NACKF) == RESET )
    {
      /* Wait until STOPF flag is reset */ 
      uint32_t I2Cx_Timeout = Timeout;
      while((I2C_GetFlagStatus(hi2c, I2C_FLAG_STOPF) == RESET))
      {
        if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
      }
      
      /* Clear STOP Flag */
      I2C_ClearFlag(hi2c, I2C_FLAG_STOPF);
        
      return SUCCESS;
    }
    else
    {
      /* Wait until STOPF flag is reset */ 
      uint32_t I2Cx_Timeout = Timeout;
      while((I2C_GetFlagStatus(hi2c, I2C_FLAG_STOPF) == RESET))
      {
        if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
      }

      /* Generate Stop */
      I2C_GenerateSTOP(hi2c, ENABLE);
      
      /* Clear NACK Flag */
      I2C_ClearFlag(hi2c, I2C_FLAG_NACKF);
      /* Clear STOP Flag, auto generated with autoend*/
      I2C_ClearFlag(hi2c, I2C_FLAG_STOPF);
    }
    
    /* Check if the maximum allowed number of trials has been reached */
    if (I2C_Trials++ == Trials)
    {
      /* Generate Stop */
      I2C_GenerateSTOP(hi2c, ENABLE);
      
      /* Wait until STOPF flag is reset */ 
      uint32_t I2Cx_Timeout = Timeout;
      while((I2C_GetFlagStatus(hi2c, I2C_FLAG_STOPF) == RESET))
      {
        if((I2Cx_Timeout--) == 0) return I2Cx_TIMEOUT_UserCallback();
      }
        
      /* Clear STOP Flag */
      I2C_ClearFlag(hi2c, I2C_FLAG_STOPF);
    }
  }while(I2C_Trials < Trials);
  
  return ERROR;
}

/**
  * @brief  Reads multiple data.
  * @param  i2c_handler : I2C handler
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  MemAddress: Memory address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static ErrorStatus I2Cx_ReadMultiple(I2C_TypeDef *i2c_handler,
                                           uint8_t Addr,
                                           uint16_t Reg,
                                           uint16_t MemAddress,
                                           uint8_t *Buffer,
                                           uint16_t Length)
{
  ErrorStatus status = SUCCESS;

  status = I2Cx_Read(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

  /* Check the communication status */
  if(status != SUCCESS)
  {
    /* I2C error occurred */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Writes a value in a register of the device through BUS in using DMA mode.
  * @param  i2c_handler : I2C handler
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address 
  * @param  Buffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
static ErrorStatus I2Cx_WriteMultiple(I2C_TypeDef *i2c_handler,
                                            uint8_t Addr,
                                            uint16_t Reg,
                                            uint16_t MemAddress,
                                            uint8_t *Buffer,
                                            uint16_t Length)
{
  ErrorStatus status = SUCCESS;
  
  status = I2Cx_Write(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != SUCCESS)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  i2c_handler : I2C handler
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
static ErrorStatus I2Cx_IsDeviceReady(I2C_TypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials)
{ 
  return (I2Cx_IsDeviceReadyTimeout(i2c_handler, DevAddress, Trials, 1000));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  i2c_handler : I2C handler
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(I2C_TypeDef *i2c_handler, uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  I2C_DeInit(i2c_handler);
  
  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
}

static ErrorStatus I2Cx_TIMEOUT_UserCallback(void)
{
  return ERROR;
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/**
  * @brief  DeInitializes the SDMMC interface.
  * @param  None
  * @retval None
  */
void BSP_SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*!< Disable SDMMC Clock */
  SDMMC_ClockCmd(SDMMC1,DISABLE);
  
  /*!< Set Power State to OFF */
  SDMMC_SetPowerState(SDMMC1,SDMMC_PowerState_OFF);

  /*!< DeInitializes the SDMMC peripheral */
  SDMMC_DeInit(SDMMC1);
  
  /* Disable the SDMMC APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDMMC1, DISABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF0_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF0_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF0_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF0_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF0_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF0_MCO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void BSP_SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD , ENABLE);
  SD_DETECT_GPIO_CLK_ENABLE();

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF12_SDMMC1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF12_SDMMC1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF12_SDMMC1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF12_SDMMC1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF12_SDMMC1);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF12_SDMMC1);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the SDMMC APB2 Clock */
  //RCC_SDMMCClockSourceConfig(RCC_SDMMCCLKSource_48MHZ);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDMMC1, ENABLE);
  
  /* Enable the DMA2 Clock */
  RCC_AHB1PeriphClockCmd(SD_SDMMC_DMA_CLK, ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDMMC Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void BSP_SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDMMC_DMA_STREAM, SD_SDMMC_DMA_FLAG_FEIF | SD_SDMMC_DMA_FLAG_DMEIF | SD_SDMMC_DMA_FLAG_TEIF | SD_SDMMC_DMA_FLAG_HTIF | SD_SDMMC_DMA_FLAG_TCIF);
  
  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDMMC_DMA_STREAM, DISABLE);

  /* DMA2 Stream3  or Stream6 Config */
  DMA_DeInit(SD_SDMMC_DMA_STREAM);
  
  SDDMA_InitStructure.DMA_Channel = SD_SDMMC_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDMMC1->FIFO;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = 0;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDMMC_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDMMC_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDMMC_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3  or Stream6 enable */
  DMA_Cmd(SD_SDMMC_DMA_STREAM, ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDMMC Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void BSP_SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;
  
  DMA_DoubleBufferModeCmd(SD_SDMMC_DMA_STREAM, DISABLE);
  DMA_ClearFlag(SD_SDMMC_DMA_STREAM, SD_SDMMC_DMA_FLAG_FEIF | SD_SDMMC_DMA_FLAG_DMEIF | SD_SDMMC_DMA_FLAG_TEIF | SD_SDMMC_DMA_FLAG_HTIF | SD_SDMMC_DMA_FLAG_TCIF);
  
  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDMMC_DMA_STREAM, DISABLE);
  
  /* DMA2 Stream3 or Stream6 Config */
  DMA_DeInit(SD_SDMMC_DMA_STREAM);
  
  SDDMA_InitStructure.DMA_Channel = SD_SDMMC_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDMMC1->FIFO;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDMMC_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDMMC_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDMMC_DMA_STREAM, DMA_FlowCtrl_Peripheral);
  
  /* DMA2 Stream3 or Stream6 enable */
  DMA_Cmd(SD_SDMMC_DMA_STREAM, ENABLE);
}

/********************************* LINK AUDIO *********************************/

/**
  * @brief  Initializes Audio low level.
  * @retval None
  */
void AUDIO_IO_Init(void) 
{
  I2Cx_Init(hI2cAudioHandler);
}

/**
  * @brief  Deinitializes Audio low level.
  * @retval None
  */
void AUDIO_IO_DeInit(void)
{
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value)
{
  uint16_t tmp = Value;
  
  Value = ((uint16_t)(tmp >> 8) & 0x00FF);
  
  Value |= ((uint16_t)(tmp << 8)& 0xFF00);
  
  I2Cx_WriteMultiple(hI2cAudioHandler, Addr, Reg, I2C_REGADDR_SIZE_16BIT,(uint8_t*)&Value, 2);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Data to be read
  */
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg)
{
  uint16_t read_value = 0, tmp = 0;
  
  I2Cx_ReadMultiple(hI2cAudioHandler, Addr, Reg, I2C_REGADDR_SIZE_16BIT, (uint8_t*)&read_value, 2);
  
  tmp = ((uint16_t)(read_value >> 8) & 0x00FF);
  
  tmp |= ((uint16_t)(read_value << 8)& 0xFF00);
  
  read_value = tmp;
  
  return read_value;
}

/**
  * @brief  AUDIO Codec delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void AUDIO_IO_Delay(uint32_t Delay)
{
  STD_Delay(Delay);
}

/********************************* LINK CAMERA ********************************/

/**
  * @brief  Initializes Camera low level.
  * @retval None
  */
void CAMERA_IO_Init(void) 
{
  I2Cx_Init(hI2cExtHandler);
}

/**
  * @brief  Camera writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
void CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(hI2cExtHandler, Addr, (uint16_t)Reg, I2C_REGADDR_SIZE_8BIT,(uint8_t*)&Value, 1);
}

/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
uint8_t CAMERA_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(hI2cExtHandler, Addr, Reg, I2C_REGADDR_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
}

/**
  * @brief  Camera delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void CAMERA_Delay(uint32_t Delay)
{
  STD_Delay(Delay);
}

/******************************** LINK I2C EEPROM *****************************/

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @retval None
  */
void EEPROM_IO_Init(void)
{
  I2Cx_Init(hI2cExtHandler);
}

/**
  * @brief  Write data to I2C EEPROM driver in using DMA channel.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be sent
  * @retval HAL status
  */
ErrorStatus EEPROM_IO_WriteData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_WriteMultiple(hI2cExtHandler, DevAddress, MemAddress, I2C_REGADDR_SIZE_16BIT, pBuffer, BufferSize));
}

/**
  * @brief  Read data from I2C EEPROM driver in using DMA channel.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  pBuffer: Pointer to data buffer
  * @param  BufferSize: Amount of data to be read
  * @retval HAL status
  */
ErrorStatus EEPROM_IO_ReadData(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
  return (I2Cx_ReadMultiple(hI2cExtHandler, DevAddress, MemAddress, I2C_REGADDR_SIZE_16BIT, pBuffer, BufferSize));
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
ErrorStatus EEPROM_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
  return (I2Cx_IsDeviceReady(hI2cExtHandler, DevAddress, Trials));
}

/********************************* LINK TOUCHSCREEN *********************************/

/**
  * @brief  Initializes Touchscreen low level.
  * @retval None
  */
void TS_IO_Init(void)
{
  I2Cx_Init(hI2cAudioHandler);
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(hI2cAudioHandler, Addr, Reg, 1, (uint8_t*)&Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(hI2cAudioHandler, Addr, Reg, 1, (uint8_t*)&read_value, 1);

  return read_value;
}

/**
  * @brief  TS delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void TS_IO_Delay(uint32_t Delay)
{
  STD_Delay(Delay);
}

/**
  * @brief  TS delay
  * @param  Delay: Delay in ms
  * @retval None
  */
__weak void STD_Delay(uint32_t Delay)
{
  volatile uint32_t Loop;
  
  while( Delay-- )
  {
    Loop = 4320;
    while( Loop-- );
  }
}

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
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
