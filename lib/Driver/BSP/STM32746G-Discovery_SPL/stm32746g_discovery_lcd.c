/**
  ******************************************************************************
  * @file    stm32746g_discovery_lcd.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32746G-Discovery board.
  @verbatim
  1. How To use this driver:
  --------------------------
     - This driver is used to drive directly an LCD TFT using the LTDC controller.
     - This driver uses timing and setting for RK043FN48H LCD.
  
  2. Driver description:
  ---------------------
    + Initialization steps:
       o Initialize the LCD using the BSP_LCD_Init() function.
       o Apply the Layer configuration using the BSP_LCD_LayerDefaultInit() function.    
       o Select the LCD layer to be used using the BSP_LCD_SelectLayer() function.
       o Enable the LCD display using the BSP_LCD_DisplayOn() function.
  
    + Options
       o Configure and enable the color keying functionality using the 
         BSP_LCD_SetColorKeying() function.
       o Modify in the fly the transparency and/or the frame buffer address
         using the following functions:
         - BSP_LCD_SetTransparency()
         - BSP_LCD_SetLayerAddress() 
    
    + Display on LCD
       o Clear the hole LCD using BSP_LCD_Clear() function or only one specified string
         line using the BSP_LCD_ClearStringLine() function.
       o Display a character on the specified line and column using the BSP_LCD_DisplayChar()
         function or a complete string line using the BSP_LCD_DisplayStringAtLine() function.
       o Display a string line on the specified position (x,y in pixel) and align mode
         using the BSP_LCD_DisplayStringAtLine() function.          
       o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap) 
         on LCD using the available set of functions.       
  @endverbatim
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
#include "stm32746g_discovery_lcd.h"
#include "../../../Utilities/Fonts/fonts.h"
#include "../../../Utilities/Fonts/font24.c"
#include "../../../Utilities/Fonts/font20.c"
#include "../../../Utilities/Fonts/font16.c"
#include "../../../Utilities/Fonts/font12.c"
#include "../../../Utilities/Fonts/font8.c"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32746G_DISCOVERY
  * @{
  */
    
/** @addtogroup STM32746G_DISCOVERY_LCD
  * @{
  */ 

/** @defgroup STM32746G_DISCOVERY_LCD_Private_TypesDefinitions STM32746G_DISCOVERY_LCD Private Types Definitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_LCD_Private_Defines STM32746G_DISCOVERY LCD Private Defines
  * @{
  */
#define POLY_X(Z)              ((int32_t)((Points + Z)->X))
#define POLY_Y(Z)              ((int32_t)((Points + Z)->Y))      
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_LCD_Private_Macros STM32746G_DISCOVERY_LCD Private Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))      
/**
  * @}
  */ 
    
/** @defgroup STM32746G_DISCOVERY_LCD_Private_Variables STM32746G_DISCOVERY_LCD Private Variables
  * @{
  */ 
static LTDC_HandleTypeDef  hLtdcHandler;

/* Default LCD configuration with LCD Layer 1 */
static uint32_t            ActiveLayer = 0;
static LCD_DrawPropTypeDef DrawProp[MAX_LAYER_NUMBER];
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_LCD_Private_FunctionPrototypes STM32746G_DISCOVERY_LCD Private Function Prototypes
  * @{
  */ 
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex);
static void LL_ConvertLineToARGB8888(void * pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode);
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_LCD_Exported_Functions STM32746G_DISCOVERY_LCD Exported Functions
  * @{
  */

/**
  * @brief  Initializes the LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_Init(void)
{
  int i;
  
  /* Enable the LTDC and DMA2D clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_LTDC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2D, ENABLE); 
  
  /* Select the used LCD */

  /* The RK043FN48H LCD 480x272 is selected */
  /* Timing configuration */
  /* Configure horizontal synchronization width */
  hLtdcHandler.LTDC_InitStruct.LTDC_HorizontalSync = (RK043FN48H_HSYNC - 1);
  /* Configure vertical synchronization height */
  hLtdcHandler.LTDC_InitStruct.LTDC_VerticalSync = (RK043FN48H_VSYNC - 1);
  /* Configure accumulated horizontal back porch */
  hLtdcHandler.LTDC_InitStruct.LTDC_AccumulatedHBP = (RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
  /* Configure accumulated vertical back porch */
  hLtdcHandler.LTDC_InitStruct.LTDC_AccumulatedVBP = (RK043FN48H_VSYNC + RK043FN48H_VBP - 1); 
  /* Configure accumulated active width */
  hLtdcHandler.LTDC_InitStruct.LTDC_AccumulatedActiveW = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
  /* Configure accumulated active height */
  hLtdcHandler.LTDC_InitStruct.LTDC_AccumulatedActiveH = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
  /* Configure total width */
  hLtdcHandler.LTDC_InitStruct.LTDC_TotalWidth = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP + RK043FN48H_HFP - 1);
  /* Configure total height */
  hLtdcHandler.LTDC_InitStruct.LTDC_TotalHeigh = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP + RK043FN48H_VFP - 1);
  
  /* LCD clock configuration */
  BSP_LCD_ClockConfig(&hLtdcHandler, 0);

  /* Initialize the LCD pixel width and pixel height */
  for( i = 0; i < MAX_LAYER_NUMBER; i++ )
  {
    hLtdcHandler.ImageWidth[i]  = RK043FN48H_WIDTH;
    hLtdcHandler.ImageHeight[i] = RK043FN48H_HEIGHT;
  }
  
  /* Background value */
  hLtdcHandler.LTDC_InitStruct.LTDC_BackgroundRedValue = 0;            
  hLtdcHandler.LTDC_InitStruct.LTDC_BackgroundGreenValue = 0;          
  hLtdcHandler.LTDC_InitStruct.LTDC_BackgroundBlueValue = 0; 
  
  /* Polarity */
  /* Initialize the horizontal synchronization polarity as active low*/
  hLtdcHandler.LTDC_InitStruct.LTDC_HSPolarity = LTDC_HSPolarity_AL;     
  /* Initialize the vertical synchronization polarity as active low */  
  hLtdcHandler.LTDC_InitStruct.LTDC_VSPolarity = LTDC_VSPolarity_AL;     
  /* Initialize the data enable polarity as active low */ 
  hLtdcHandler.LTDC_InitStruct.LTDC_DEPolarity = LTDC_DEPolarity_AL;     
  /* Initialize the pixel clock polarity as input pixel clock */
  hLtdcHandler.LTDC_InitStruct.LTDC_PCPolarity = LTDC_PCPolarity_IPC;
  
  hLtdcHandler.Instance = LTDC;
  
  /* Initialize the LCD Msp: this __weak function can be rewritten by the application */
  BSP_LCD_MspInit(&hLtdcHandler, 0);
  
  LTDC_Init(&hLtdcHandler.LTDC_InitStruct);

  /* Assert display enable LCD_DISP pin */
  GPIO_WriteBit(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, Bit_SET);

  /* Assert backlight LCD_BL_CTRL pin */
  GPIO_WriteBit(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, Bit_RESET);
    
  /* Initialize the font */
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
  return LCD_OK;
}

/**
  * @brief  DeInitializes the LCD.
  * @retval LCD state
  */
uint8_t BSP_LCD_DeInit(void)
{ 
  /* Initialize the hLtdcHandler Instance parameter */
  hLtdcHandler.Instance = LTDC;

 /* Disable LTDC block */
  LTDC_Cmd(DISABLE);

  /* DeInit the LTDC */
  LTDC_DeInit();

  /* DeInit the LTDC MSP : this __weak function can be rewritten by the application */
  BSP_LCD_MspDeInit(&hLtdcHandler, 0);

  return LCD_OK;
}

/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  //return hLtdcHandler.LayerCfg[ActiveLayer].ImageWidth;
  return hLtdcHandler.ImageWidth[ActiveLayer];
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  //return hLtdcHandler.LayerCfg[ActiveLayer].ImageHeight;
  return hLtdcHandler.ImageHeight[ActiveLayer];
}

/**
  * @brief  Set the LCD X size.
  * @param  imageWidthPixels : image width in pixels unit
  * @retval None
  */
void BSP_LCD_SetXSize(uint32_t imageWidthPixels)
{
  //hLtdcHandler.LayerCfg[ActiveLayer].ImageWidth = imageWidthPixels;
  hLtdcHandler.ImageWidth[ActiveLayer] = imageWidthPixels;
}

/**
  * @brief  Set the LCD Y size.
  * @param  imageHeightPixels : image height in lines unit
  * @retval None
  */
void BSP_LCD_SetYSize(uint32_t imageHeightPixels)
{
  //hLtdcHandler.LayerCfg[ActiveLayer].ImageHeight = imageHeightPixels;
  hLtdcHandler.ImageHeight[ActiveLayer] = imageHeightPixels;
}

/**
  * @brief  Initializes the LCD layer in ARGB8888 format (32 bits per pixel).
  * @param  LayerIndex: Layer foreground or background
  * @param  FB_Address: Layer frame buffer
  * @retval None
  */
void BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address)
{
  /* Windowing configuration */ 
  /* In this case all the active display area is used to display a picture then:
  Horizontal start = horizontal synchronization + Horizontal back porch = 30 
  Horizontal stop = Horizontal start + window width -1 = 30 + 240 -1
  Vertical start   = vertical synchronization + vertical back porch     = 4
  Vertical stop   = Vertical start + window height -1  = 4 + 160 -1      */ 
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_HorizontalStart = RK043FN48H_HSYNC + RK043FN48H_HBP;
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_HorizontalStop = RK043FN48H_HSYNC + RK043FN48H_HBP + BSP_LCD_GetXSize() - 1; 
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_VerticalStart = RK043FN48H_VSYNC + RK043FN48H_VBP;
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_VerticalStop = RK043FN48H_VSYNC + RK043FN48H_VBP + BSP_LCD_GetYSize() - 1; 
  
  /* Pixel Format configuration*/           
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_PixelFormat = LTDC_Pixelformat_ARGB8888;
  
  /* Alpha constant (255 totally opaque) */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_ConstantAlpha = 255;
  
  /* Configure blending factors */       
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_PAxCA;
  
  /* Default Color configuration (configure A,R,G,B component values) */          
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorBlue = 0;        
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorGreen = 0;       
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorRed = 0;         
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorAlpha = 0;  
  
  /* Input Address configuration */    
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBStartAdress = FB_Address;
  
  /* the length of one line of pixels in bytes + 3 then :
  Line Lenth = Active high width x number of bytes per pixel + 3 
  Active high width         = 240 
  number of bytes per pixel = 2    (pixel_format : RGB565) 
  */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBLineLength = ((BSP_LCD_GetXSize() * 4) + 3);
  
  /*  the pitch is the increment from the start of one line of pixels to the 
  start of the next line in bytes, then :
  Pitch = Active high width x number of bytes per pixel     
  */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBPitch = (BSP_LCD_GetXSize() * 4);
  
  /* configure the number of lines */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBLineNumber = BSP_LCD_GetYSize();
  
  if( LayerIndex == 0 )
  {
    LTDC_LayerInit(LTDC_Layer1, &hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex]);
    LTDC_LayerCmd(LTDC_Layer1, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
    LTDC_DitherCmd(ENABLE);
    
    hLtdcHandler.LayerInstance[LayerIndex] = LTDC_Layer1;
  }
  else
  {
    LTDC_LayerInit(LTDC_Layer2, &hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex]);
    LTDC_LayerCmd(LTDC_Layer2, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
    LTDC_DitherCmd(ENABLE);
    
    hLtdcHandler.LayerInstance[LayerIndex] = LTDC_Layer2;
  }
  
  DrawProp[LayerIndex].BackColor = LCD_COLOR_WHITE;
  DrawProp[LayerIndex].pFont     = &Font24;
  DrawProp[LayerIndex].TextColor = LCD_COLOR_BLACK; 
}

/**
  * @brief  Initializes the LCD layer in RGB565 format (16 bits per pixel).
  * @param  LayerIndex: Layer foreground or background
  * @param  FB_Address: Layer frame buffer
  * @retval None
  */
void BSP_LCD_LayerRgb565Init(uint16_t LayerIndex, uint32_t FB_Address)
{     
  /* Windowing configuration */ 
  /* In this case all the active display area is used to display a picture then:
  Horizontal start = horizontal synchronization + Horizontal back porch = 30 
  Horizontal stop = Horizontal start + window width -1 = 30 + 240 -1
  Vertical start   = vertical synchronization + vertical back porch     = 4
  Vertical stop   = Vertical start + window height -1  = 4 + 160 -1      */ 
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_HorizontalStart = RK043FN48H_HSYNC + RK043FN48H_HBP;
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_HorizontalStop = RK043FN48H_HSYNC + RK043FN48H_HBP + BSP_LCD_GetXSize() - 1; 
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_VerticalStart = RK043FN48H_VSYNC + RK043FN48H_VBP;
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_VerticalStop = RK043FN48H_VSYNC + RK043FN48H_VBP + BSP_LCD_GetYSize() - 1; 
  
  /* Pixel Format configuration*/           
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_PixelFormat = LTDC_Pixelformat_RGB565;
  
  /* Alpha constant (255 totally opaque) */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_ConstantAlpha = 255;
  
  /* Configure blending factors */       
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_PAxCA;
  
  /* Default Color configuration (configure A,R,G,B component values) */          
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorBlue = 0;        
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorGreen = 0;       
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorRed = 0;         
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_DefaultColorAlpha = 0;  
  
  /* Input Address configuration */    
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBStartAdress = FB_Address;
  
  /* the length of one line of pixels in bytes + 3 then :
  Line Lenth = Active high width x number of bytes per pixel + 3 
  Active high width         = 240 
  number of bytes per pixel = 2    (pixel_format : RGB565) 
  */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBLineLength = ((BSP_LCD_GetXSize() * 2) + 3);
  
  /*  the pitch is the increment from the start of one line of pixels to the 
  start of the next line in bytes, then :
  Pitch = Active high width x number of bytes per pixel     
  */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBPitch = (BSP_LCD_GetXSize() * 2);
  
  /* configure the number of lines */
  hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex].LTDC_CFBLineNumber = BSP_LCD_GetYSize();
  
  if( LayerIndex == 0 )
  {
    LTDC_LayerInit(LTDC_Layer1, &hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex]);
    LTDC_LayerCmd(LTDC_Layer1, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
    LTDC_DitherCmd(ENABLE);
  }
  else
  {
    LTDC_LayerInit(LTDC_Layer2, &hLtdcHandler.LTDC_Layer_InitStruct[LayerIndex]);
    LTDC_LayerCmd(LTDC_Layer2, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
    LTDC_DitherCmd(ENABLE);
  }

  DrawProp[LayerIndex].BackColor = LCD_COLOR_WHITE;
  DrawProp[LayerIndex].pFont     = &Font24;
  DrawProp[LayerIndex].TextColor = LCD_COLOR_BLACK; 
}

/**
  * @brief  Selects the LCD Layer.
  * @param  LayerIndex: Layer foreground or background
  * @retval None
  */
void BSP_LCD_SelectLayer(uint32_t LayerIndex)
{
  ActiveLayer = LayerIndex;
} 

/**
  * @brief  Sets an LCD Layer visible
  * @param  LayerIndex: Visible Layer
  * @param  State: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE 
  * @retval None
  */
void BSP_LCD_SetLayerVisible(uint32_t LayerIndex, FunctionalState State)
{
  if( LayerIndex == 0 )
  {
    LTDC_LayerCmd(LTDC_Layer1, State);
  }
  else
  {
    LTDC_LayerCmd(LTDC_Layer2, State);
  }
  
  /* Reload LTDC configuration  */
  LTDC_ReloadConfig(LTDC_IMReload);
} 

/**
  * @brief  Configures the transparency.
  * @param  LayerIndex: Layer foreground or background.
  * @param  Transparency: Transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF 
  * @retval None
  */
void BSP_LCD_SetTransparency(uint32_t LayerIndex, uint8_t Transparency)
{
  if( LayerIndex == 0 )
  {
    LTDC_LayerAlpha(LTDC_Layer1, Transparency);
  }
  else
  {
    LTDC_LayerAlpha(LTDC_Layer2, Transparency);
  }
  
  /* Reload LTDC configuration  */
  LTDC_ReloadConfig(LTDC_IMReload);
}

/**
  * @brief  Sets an LCD layer frame buffer address.
  * @param  LayerIndex: Layer foreground or background
  * @param  Address: New LCD frame buffer value      
  * @retval None
  */
void BSP_LCD_SetLayerAddress(uint32_t LayerIndex, uint32_t Address)
{
  if( LayerIndex == 0 )
  {
    LTDC_LayerAddress(LTDC_Layer1, Address);
  }
  else
  {
    LTDC_LayerAddress(LTDC_Layer2, Address);
  }
  
  /* Reload LTDC configuration  */
  LTDC_ReloadConfig(LTDC_IMReload);
}

/**
  * @brief  Sets display window.
  * @param  LayerIndex: Layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height  
  * @retval None
  */
void BSP_LCD_SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  if( LayerIndex == 0 )
  {
    /* Reconfigure the layer size */
    LTDC_LayerSize(LTDC_Layer1, Width, Height);
    
    /* Reconfigure the layer position */
    LTDC_LayerPosition(LTDC_Layer1, Xpos, Ypos);
  }
  else
  {
    /* Reconfigure the layer size */
    LTDC_LayerSize(LTDC_Layer2, Width, Height);
    
    /* Reconfigure the layer position */
    LTDC_LayerPosition(LTDC_Layer2, Xpos, Ypos);
  }
  
  /* Reload LTDC configuration  */
  LTDC_ReloadConfig(LTDC_IMReload);
}

/**
  * @brief  Configures and sets the color keying.
  * @param  LayerIndex: Layer foreground or background
  * @param  RGBValue: Color reference
  * @retval None
  */
void BSP_LCD_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue)
{  
  LTDC_ColorKeying_InitTypeDef   LTDC_colorkeying_InitStruct;
  
  /* Configure and Enable the color Keying for LCD Layer */
  LTDC_colorkeying_InitStruct.LTDC_ColorKeyBlue = 0x0000FF & RGBValue;
  LTDC_colorkeying_InitStruct.LTDC_ColorKeyGreen = (0x00FF00 & RGBValue) >> 8;
  LTDC_colorkeying_InitStruct.LTDC_ColorKeyRed = (0xFF0000 & RGBValue) >> 16;  

  if (LayerIndex == 0)
  {   
    /* Enable the color Keying for Layer1 */
    LTDC_ColorKeyingConfig(LTDC_Layer1, &LTDC_colorkeying_InitStruct, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
  }
  else
  {
    /* Enable the color Keying for Layer2 */
    LTDC_ColorKeyingConfig(LTDC_Layer2, &LTDC_colorkeying_InitStruct, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
  }
}

/**
  * @brief  Disables the color keying.
  * @param  LayerIndex: Layer foreground or background
  * @retval None
  */
void BSP_LCD_ResetColorKeying(uint32_t LayerIndex)
{   
  /* Disable the color Keying for LCD Layer */
  LTDC_ColorKeying_InitTypeDef   LTDC_colorkeying_InitStruct;
  
  if (LayerIndex == 0)
  {   
    /* Disable the color Keying for Layer1 */
    LTDC_ColorKeyingConfig(LTDC_Layer1, &LTDC_colorkeying_InitStruct, DISABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
  }
  else
  {
    /* Disable the color Keying for Layer2 */
    LTDC_ColorKeyingConfig(LTDC_Layer2, &LTDC_colorkeying_InitStruct, DISABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
  }
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code ARGB(8-8-8-8)
  * @retval None
  */
void BSP_LCD_SetTextColor(uint32_t Color)
{
  DrawProp[ActiveLayer].TextColor = Color;
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint32_t BSP_LCD_GetTextColor(void)
{
  return DrawProp[ActiveLayer].TextColor;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Layer background color code ARGB(8-8-8-8)
  * @retval None
  */
void BSP_LCD_SetBackColor(uint32_t Color)
{
  DrawProp[ActiveLayer].BackColor = Color;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background colour
  */
uint32_t BSP_LCD_GetBackColor(void)
{
  return DrawProp[ActiveLayer].BackColor;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Layer font to be used
  * @retval None
  */
void BSP_LCD_SetFont(sFONT *fonts)
{
  DrawProp[ActiveLayer].pFont = fonts;
}

/**
  * @brief  Gets the LCD text font.
  * @retval Used layer font
  */
sFONT *BSP_LCD_GetFont(void)
{
  return DrawProp[ActiveLayer].pFont;
}

/**
  * @brief  Reads an LCD pixel.
  * @param  Xpos: X position 
  * @param  Ypos: Y position 
  * @retval RGB pixel color
  */
uint32_t BSP_LCD_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint32_t ret = 0;
  
  if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_ARGB8888)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint32_t*) (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));
  }
  else if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB888)
  {
    /* Read data value from SDRAM memory */
    ret = (*(__IO uint32_t*) (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos))) & 0x00FFFFFF);
  }
  else if((hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB565) || \
          (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_ARGB4444) || \
          (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_AL88))  
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint16_t*) (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));    
  }
  else
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint8_t*) (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos)));    
  }
  
  return ret;
}

/**
  * @brief  Clears the hole LCD.
  * @param  Color: Color of the background
  * @retval None
  */
void BSP_LCD_Clear(uint32_t Color)
{ 
  /* Clear the LCD */ 
  LL_FillBuffer(ActiveLayer, (uint32_t *)(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress), BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), 0, Color);
}

/**
  * @brief  Clears the selected line.
  * @param  Line: Line to be cleared
  * @retval None
  */
void BSP_LCD_ClearStringLine(uint32_t Line)
{
  uint32_t color_backup = DrawProp[ActiveLayer].TextColor;
  DrawProp[ActiveLayer].TextColor = DrawProp[ActiveLayer].BackColor;
  
  /* Draw rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp[ActiveLayer].pFont->Height), BSP_LCD_GetXSize(), DrawProp[ActiveLayer].pFont->Height);
  
  DrawProp[ActiveLayer].TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);  
}

/**
  * @brief  Displays one character.
  * @param  Xpos: Start column address
  * @param  Ypos: Line where to display the character shape.
  * @param  Ascii: Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E 
  * @retval None
  */
void BSP_LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  DrawChar(Xpos, Ypos, &DrawProp[ActiveLayer].pFont->table[(Ascii-' ') *\
    DrawProp[ActiveLayer].pFont->Height * ((DrawProp[ActiveLayer].pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays characters on the LCD.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)   
  * @param  Text: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE   
  * @retval None
  */
void BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t ref_column = 1, i = 0;
  uint32_t size = 0, xsize = 0; 
  uint8_t  *ptr = Text;
  
  /* Get the text size */
  while (*ptr++) size ++ ;
  
  /* Characters number per line */
  xsize = (BSP_LCD_GetXSize()/DrawProp[ActiveLayer].pFont->Width);
  
  switch (Mode)
  {
  case CENTER_MODE:
    {
      ref_column = Xpos + ((xsize - size)* DrawProp[ActiveLayer].pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      ref_column = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      ref_column = - Xpos + ((xsize - size)*DrawProp[ActiveLayer].pFont->Width);
      break;
    }    
  default:
    {
      ref_column = Xpos;
      break;
    }
  }
  
  /* Check that the Start column is located in the screen */
  if ((ref_column < 1) || (ref_column >= 0x8000))
  {
    ref_column = 1;
  }

  /* Send the string character by character on LCD */
  while ((*Text != 0) & (((BSP_LCD_GetXSize() - (i*DrawProp[ActiveLayer].pFont->Width)) & 0xFFFF) >= DrawProp[ActiveLayer].pFont->Width))
  {
    /* Display one character on LCD */
    BSP_LCD_DisplayChar(ref_column, Ypos, *Text);
    /* Decrement the column position by 16 */
    ref_column += DrawProp[ActiveLayer].pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }  
}

/**
  * @brief  Displays a maximum of 60 characters on the LCD.
  * @param  Line: Line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{  
  BSP_LCD_DisplayStringAt(0, LINE(Line), ptr, LEFT_MODE);
}

/**
  * @brief  Draws an horizontal line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t  Xaddress = 0;
  
  /* Get the line address */
  if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB565)
  { /* RGB565 format */
    Xaddress = (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress) + 2*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  else
  { /* ARGB8888 format */
    Xaddress = (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  
  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, Length, 1, 0, DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a vertical line.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t  Xaddress = 0;
  
  /* Get the line address */
  if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB565)
  { /* RGB565 format */
    Xaddress = (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress) + 2*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  else
  { /* ARGB8888 format */
    Xaddress = (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  
  /* Write line */
  LL_FillBuffer(ActiveLayer, (uint32_t *)Xaddress, 1, Length, (BSP_LCD_GetXSize() - 1), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws an uni-line (between two points).
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @retval None
  */
void BSP_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, num_add = 0, num_pixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    num_add = deltay;
    num_pixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    num_add = deltax;
    num_pixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= num_pixels; curpixel++)
  {
    BSP_LCD_DrawPixel(x, y, DrawProp[ActiveLayer].TextColor);   /* Draw the current pixel */
    num += num_add;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

/**
  * @brief  Draws a rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width  
  * @param  Height: Rectangle height
  * @retval None
  */
void BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Draw horizontal lines */
  BSP_LCD_DrawHLine(Xpos, Ypos, Width);
  BSP_LCD_DrawHLine(Xpos, (Ypos+ Height), Width);
  
  /* Draw vertical lines */
  BSP_LCD_DrawVLine(Xpos, Ypos, Height);
  BSP_LCD_DrawVLine((Xpos + Width), Ypos, Height);
}

/**
  * @brief  Draws a circle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  * @retval None
  */
void BSP_LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t   decision;    /* Decision Variable */ 
  uint32_t  current_x;   /* Current X Value */
  uint32_t  current_y;   /* Current Y Value */
  
  decision = 3 - (Radius << 1);
  current_x = 0;
  current_y = Radius;
  
  while (current_x <= current_y)
  {
    BSP_LCD_DrawPixel((Xpos + current_x), (Ypos - current_y), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos - current_x), (Ypos - current_y), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos + current_y), (Ypos - current_x), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos - current_y), (Ypos - current_x), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos + current_x), (Ypos + current_y), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos - current_x), (Ypos + current_y), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos + current_y), (Ypos + current_x), DrawProp[ActiveLayer].TextColor);
    
    BSP_LCD_DrawPixel((Xpos - current_y), (Ypos + current_x), DrawProp[ActiveLayer].TextColor);
    
    if (decision < 0)
    { 
      decision += (current_x << 2) + 6;
    }
    else
    {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
    }
    current_x++;
  } 
}

/**
  * @brief  Draws an poly-line (between many points).
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_DrawPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t x = 0, y = 0;
  
  if(PointCount < 2)
  {
    return;
  }
  
  BSP_LCD_DrawLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);
  
  while(--PointCount)
  {
    x = Points->X;
    y = Points->Y;
    Points++;
    BSP_LCD_DrawLine(x, y, Points->X, Points->Y);
  }
}

/**
  * @brief  Draws an ellipse on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius
  * @retval None
  */
void BSP_LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;
  
  rad1 = XRadius;
  rad2 = YRadius;
  
  k = (float)(rad2/rad1);  
  
  do { 
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/k)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/k)), (Ypos+y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos+(uint16_t)(x/k)), (Ypos-y), DrawProp[ActiveLayer].TextColor);
    BSP_LCD_DrawPixel((Xpos-(uint16_t)(x/k)), (Ypos-y), DrawProp[ActiveLayer].TextColor);      
    
    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;     
  }
  while (y <= 0);
}

/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in ARGB mode (8-8-8-8)
  * @retval None
  */
void BSP_LCD_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  /* Write data value to all SDRAM memory */
  if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB565)
  { /* RGB565 format */
    *(__IO uint16_t*) (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (2*(Ypos*BSP_LCD_GetXSize() + Xpos))) = (uint16_t)RGB_Code;
  }
  else
  { /* ARGB8888 format */
    *(__IO uint32_t*) (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (4*(Ypos*BSP_LCD_GetXSize() + Xpos))) = RGB_Code;
  }
}

/**
  * @brief  Draws a bitmap picture loaded in the internal Flash in ARGB888 format (32 bits per pixel).
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address in the internal Flash
  * @retval None
  */
void BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
  uint32_t address;
  uint32_t input_color_mode = 0;
  
  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (pbmp + 10);
  index |= (*(__IO uint16_t *) (pbmp + 12)) << 16;
  
  /* Read bitmap width */
  width = *(uint16_t *) (pbmp + 18);
  width |= (*(uint16_t *) (pbmp + 20)) << 16;
  
  /* Read bitmap height */
  height = *(uint16_t *) (pbmp + 22);
  height |= (*(uint16_t *) (pbmp + 24)) << 16; 
  
  /* Read bit/pixel */
  bit_pixel = *(uint16_t *) (pbmp + 28);   
  
  /* Set the address */
  address = hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress + (((BSP_LCD_GetXSize()*Ypos) + Xpos)*(4));
  
  /* Get the layer pixel format */    
  if ((bit_pixel/8) == 4)
  {
    input_color_mode = CM_ARGB8888;
  }
  else if ((bit_pixel/8) == 2)
  {
    input_color_mode = CM_RGB565;   
  }
  else 
  {
    input_color_mode = CM_RGB888;
  }
  
  /* Bypass the bitmap header */
  pbmp += (index + (width * (height - 1) * (bit_pixel/8)));  
  
  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {
    /* Pixel format conversion */
    LL_ConvertLineToARGB8888((uint32_t *)pbmp, (uint32_t *)address, width, input_color_mode);
    
    /* Increment the source and destination buffers */
    address+=  (BSP_LCD_GetXSize()*4);
    pbmp -= width*(bit_pixel/8);
  } 
}

/**
  * @brief  Draws a full rectangle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Width: Rectangle width  
  * @param  Height: Rectangle height
  * @retval None
  */
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint32_t  x_address = 0;
  
  /* Set the text color */
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
  
  /* Get the rectangle start address */
  if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB565)
  { /* RGB565 format */
    x_address = (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress) + 2*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  else
  { /* ARGB8888 format */
    x_address = (hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_CFBStartAdress) + 4*(BSP_LCD_GetXSize()*Ypos + Xpos);
  }
  /* Fill the rectangle */
  LL_FillBuffer(ActiveLayer, (uint32_t *)x_address, Width, Height, (BSP_LCD_GetXSize() - Width), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a full circle.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Radius: Circle radius
  * @retval None
  */
void BSP_LCD_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  decision;     /* Decision Variable */ 
  uint32_t  current_x;   /* Current X Value */
  uint32_t  current_y;   /* Current Y Value */
  
  decision = 3 - (Radius << 1);
  
  current_x = 0;
  current_y = Radius;
  
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
  
  while (current_x <= current_y)
  {
    if(current_y > 0) 
    {
      BSP_LCD_DrawHLine(Xpos - current_y, Ypos + current_x, 2*current_y);
      BSP_LCD_DrawHLine(Xpos - current_y, Ypos - current_x, 2*current_y);
    }
    
    if(current_x > 0) 
    {
      BSP_LCD_DrawHLine(Xpos - current_x, Ypos - current_y, 2*current_x);
      BSP_LCD_DrawHLine(Xpos - current_x, Ypos + current_y, 2*current_x);
    }
    if (decision < 0)
    { 
      decision += (current_x << 2) + 6;
    }
    else
    {
      decision += ((current_x - current_y) << 2) + 10;
      current_y--;
    }
    current_x++;
  }
  
  BSP_LCD_SetTextColor(DrawProp[ActiveLayer].TextColor);
  BSP_LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Draws a full poly-line (between many points).
  * @param  Points: Pointer to the points array
  * @param  PointCount: Number of points
  * @retval None
  */
void BSP_LCD_FillPolygon(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  image_left = 0, image_right = 0, image_top = 0, image_bottom = 0;
  
  image_left = image_right = Points->X;
  image_top= image_bottom = Points->Y;
  
  for(counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < image_left)
    {
      image_left = pixelX;
    }
    if(pixelX > image_right)
    {
      image_right = pixelX;
    }
    
    pixelY = POLY_Y(counter);
    if(pixelY < image_top)
    { 
      image_top = pixelY;
    }
    if(pixelY > image_bottom)
    {
      image_bottom = pixelY;
    }
  }  
  
  if(PointCount < 2)
  {
    return;
  }
  
  X_center = (image_left + image_right)/2;
  Y_center = (image_bottom + image_top)/2;
  
  X_first = Points->X;
  Y_first = Points->Y;
  
  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;    
    
    FillTriangle(X, X2, X_center, Y, Y2, Y_center);
    FillTriangle(X, X_center, X2, Y, Y_center, Y2);
    FillTriangle(X_center, X2, X, Y_center, Y2, Y);   
  }
  
  FillTriangle(X_first, X2, X_center, Y_first, Y2, Y_center);
  FillTriangle(X_first, X_center, X2, Y_first, Y_center, Y2);
  FillTriangle(X_center, X2, X_first, Y_center, Y2, Y_first);   
}

/**
  * @brief  Draws a full ellipse.
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  XRadius: Ellipse X radius
  * @param  YRadius: Ellipse Y radius  
  * @retval None
  */
void BSP_LCD_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;
  
  rad1 = XRadius;
  rad2 = YRadius;
  
  k = (float)(rad2/rad1);
  
  do 
  {       
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/k)), (Ypos+y), (2*(uint16_t)(x/k) + 1));
    BSP_LCD_DrawHLine((Xpos-(uint16_t)(x/k)), (Ypos-y), (2*(uint16_t)(x/k) + 1));
    
    e2 = err;
    if (e2 <= x) 
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Enables the display.
  * @retval None
  */
void BSP_LCD_DisplayOn(void)
{
  /* Display On */
  LTDC_ReloadConfig(LTDC_IMReload);
  LTDC_Cmd(ENABLE);
  GPIO_WriteBit(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, Bit_SET);        /* Assert LCD_DISP pin */
  GPIO_WriteBit(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, Bit_SET);  /* Assert LCD_BL_CTRL pin */
}

/**
  * @brief  Disables the display.
  * @retval None
  */
void BSP_LCD_DisplayOff(void)
{
  /* Display Off */
  LTDC_Cmd(DISABLE);
  GPIO_WriteBit(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, Bit_RESET);      /* De-assert LCD_DISP pin */
  GPIO_WriteBit(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, Bit_RESET);/* De-assert LCD_BL_CTRL pin */
}

/**
  * @brief  Initializes the LTDC MSP.
  * @param  hltdc: LTDC handle
  * @param  Params
  * @retval None
  */
__weak void BSP_LCD_MspInit(LTDC_HandleTypeDef *hltdc, void *Params)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable GPIOI, GPIOJ, GPIOK, GPIOE, GPIOG AHB Clocks */
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG | \
                          RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOJ | \
                          RCC_AHB1Periph_GPIOK, ENABLE);
  LCD_DISP_GPIO_CLK_ENABLE();
  LCD_BL_CTRL_GPIO_CLK_ENABLE();
  /* GPIOs Configuration */
  /*
  +------------------------+-----------------------+----------------------------+
  +                       LCD pins assignment                                   +
  +------------------------+-----------------------+----------------------------+
  |  LCD_TFT R0 <-> PI.15  |  LCD_TFT G0 <-> PJ.07 |  LCD_TFT B0 <-> PE.04      |
  |  LCD_TFT R1 <-> PJ.00  |  LCD_TFT G1 <-> PJ.08 |  LCD_TFT B1 <-> PJ.13      |
  |  LCD_TFT R2 <-> PJ.01  |  LCD_TFT G2 <-> PJ.09 |  LCD_TFT B2 <-> PJ.14      |
  |  LCD_TFT R3 <-> PJ.02  |  LCD_TFT G3 <-> PJ.10 |  LCD_TFT B3 <-> PJ.15      |
  |  LCD_TFT R4 <-> PJ.03  |  LCD_TFT G4 <-> PJ.11 |  LCD_TFT B4 <-> PG.12      |
  |  LCD_TFT R5 <-> PJ.04  |  LCD_TFT G5 <-> PK.00 |  LCD_TFT B5 <-> PK.04      |
  |  LCD_TFT R6 <-> PJ.05  |  LCD_TFT G6 <-> PK.01 |  LCD_TFT B6 <-> PK.05      |
  |  LCD_TFT R7 <-> PJ.06  |  LCD_TFT G7 <-> PK.02 |  LCD_TFT B7 <-> PK.06      |
  -------------------------------------------------------------------------------
  |  LCD_TFT HSYNC <-> PI.10  | LCDTFT VSYNC <->  PI.09 |
  |  LCD_TFT CLK   <-> PI.14  | LCD_TFT DE   <->  PK.07 |
  -----------------------------------------------------
  |  LCD_TFT DISP  <-> PI.12  | LCD_TFT BL   <->  PK.03 |
  -----------------------------------------------------
  
  */
  
  /* GPIOE configuration */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF14_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4; 
  GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /* GPIOG configuration */
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF9_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12; 
  GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  /* GPIOI configuration */
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource8, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource9, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource10, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource13, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource14, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOI, GPIO_PinSource15, GPIO_AF14_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | \
                              GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
  GPIO_Init(GPIOI, &GPIO_InitStruct);
  
  /* GPIOJ configuration */
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource0, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource1, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource2, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource3, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource4, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource5, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource6, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource7, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource8, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource9, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource10, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource11, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource13, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource14, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource15, GPIO_AF14_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
                              GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | \
                              GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | \
                              GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
  GPIO_Init(GPIOJ, &GPIO_InitStruct);
  
  /* GPIOK configuration */
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource0, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource1, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource2, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource4, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource5, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource6, GPIO_AF14_LTDC);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource7, GPIO_AF14_LTDC);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | \
                              GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; 
  GPIO_Init(GPIOK, &GPIO_InitStruct);
  
  /* LCD_DISP configuration */
  GPIO_InitStruct.GPIO_Pin = LCD_DISP_PIN; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_DISP_GPIO_PORT, &GPIO_InitStruct);
  
  /* LCD_BL configuration */
  GPIO_InitStruct.GPIO_Pin = LCD_BL_CTRL_PIN; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  DeInitializes BSP_LCD MSP.
  * @param  hltdc: LTDC handle
  * @param  Params
  * @retval None
  */
__weak void BSP_LCD_MspDeInit(LTDC_HandleTypeDef *hltdc, void *Params)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Disable LTDC block */
  LTDC_Cmd(DISABLE);

  /* LTDC Pins deactivation */

  /* GPIOE deactivation */
  GPIO_InitStruct.GPIO_Pin       = GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode      = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType     = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd      = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed     = GPIO_Low_Speed;
  GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* GPIOG deactivation */
  GPIO_InitStruct.GPIO_Pin       = GPIO_Pin_12;
  GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* GPIOI deactivation */
  GPIO_InitStruct.GPIO_Pin       = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12 | \
                                  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* GPIOJ deactivation */
  GPIO_InitStruct.GPIO_Pin       = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | \
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | \
                                  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /* GPIOK deactivation */
  GPIO_InitStruct.GPIO_Pin       = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | \
                                  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOK, &GPIO_InitStruct);

  /* Disable LTDC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_LTDC, DISABLE);

  /* GPIO pins clock can be shut down in the application
     by surcharging this __weak function */
}

/**
  * @brief  Clock Config.
  * @param  hltdc: LTDC handle
  * @param  Params
  * @note   This API is called by BSP_LCD_Init()
  *         Being __weak it can be overwritten by the application
  * @retval None
  */
__weak void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params)
{
  uint32_t tmpreg0, tmpreg1;
  
  /* Configure PLLSAI prescalers for LCD */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAI_N = 192 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAI_R = 192/5 = 38.4 Mhz */
  /* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDivR = 38.4/4 = 9.6 Mhz */
  
  /* Read PLLSAIP and PLLSAIQ value from PLLSAICFGR register (these value are not needed for LTDC configuration) */
  tmpreg0 = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIQ) >> POSITION_VAL(RCC_PLLSAICFGR_PLLSAIQ));
  tmpreg1 = ((RCC->PLLSAICFGR & RCC_PLLSAICFGR_PLLSAIP) >> POSITION_VAL(RCC_PLLSAICFGR_PLLSAIP));
  
  RCC_PLLSAIConfig( 192, tmpreg1, tmpreg0, RK043FN48H_FREQUENCY_DIVIDER );
  RCC_LTDCCLKDivConfig(RCC_PLLSAIDivR_Div4);
  
  /* Enable PLLSAI Clock */
  RCC_PLLSAICmd(ENABLE);
  
  /* Wait for PLLSAI activation */
  while(RCC_GetFlagStatus(RCC_FLAG_PLLSAIRDY) == RESET)
  {
  }
}


/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  c: Pointer to the character data
  * @retval None
  */
static void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t  offset;
  uint8_t  *pchar;
  uint32_t line;
  
  height = DrawProp[ActiveLayer].pFont->Height;
  width  = DrawProp[ActiveLayer].pFont->Width;
  
  offset =  8 *((width + 7)/8) -  width ;
  
  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c + (width + 7)/8 * i);
    
    switch(((width + 7)/8))
    {
      
    case 1:
      line =  pchar[0];      
      break;
      
    case 2:
      line =  (pchar[0]<< 8) | pchar[1];      
      break;
      
    case 3:
    default:
      line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];      
      break;
    } 
    
    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1))) 
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].TextColor);
      }
      else
      {
        BSP_LCD_DrawPixel((Xpos + j), Ypos, DrawProp[ActiveLayer].BackColor);
      } 
    }
    Ypos++;
  }
}

/**
  * @brief  Fills a triangle (between 3 points).
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  x3: Point 3 X position
  * @param  y3: Point 3 Y position
  * @retval None
  */
static void FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3)
{ 
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, num_add = 0, num_pixels = 0,
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    num_add = deltay;
    num_pixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    num_add = deltax;
    num_pixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= num_pixels; curpixel++)
  {
    BSP_LCD_DrawLine(x, y, x3, y3);
    
    num += num_add;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  } 
}

/**
  * @brief  Fills a buffer.
  * @param  LayerIndex: Layer index
  * @param  pDst: Pointer to destination buffer
  * @param  xSize: Buffer width
  * @param  ySize: Buffer height
  * @param  OffLine: Offset
  * @param  ColorIndex: Color index
  * @retval None
  */
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex) 
{
  DMA2D_InitTypeDef      DMA2D_InitStruct;
  
  uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0, Alpha_Value = 0;
  
  DMA2D_DeInit();
  DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;
  
  if(hLtdcHandler.LTDC_Layer_InitStruct[ActiveLayer].LTDC_PixelFormat == LTDC_Pixelformat_RGB565)
  { /* RGB565 format */ 
    DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;
    Red_Value = (0xF800 & ColorIndex) >> 11;
    Blue_Value = 0x001F & ColorIndex;
    Green_Value = (0x07E0 & ColorIndex) >> 5;
  }
  else
  { /* ARGB8888 format */
    DMA2D_InitStruct.DMA2D_CMode = DMA2D_ARGB8888;
    Red_Value = (0xFF0000 & ColorIndex) >> 16;
    Blue_Value = 0x0000FF & ColorIndex;
    Green_Value = (0x00FF00 & ColorIndex) >> 8;
    Alpha_Value = (0xFF000000 & ColorIndex) >> 24;
  }
  
  DMA2D_InitStruct.DMA2D_OutputGreen = Green_Value;
  DMA2D_InitStruct.DMA2D_OutputBlue = Blue_Value;
  DMA2D_InitStruct.DMA2D_OutputRed = Red_Value;
  DMA2D_InitStruct.DMA2D_OutputAlpha = Alpha_Value;
  DMA2D_InitStruct.DMA2D_OutputMemoryAdd = (uint32_t)pDst;
  DMA2D_InitStruct.DMA2D_OutputOffset = OffLine;
  DMA2D_InitStruct.DMA2D_NumberOfLine = ySize;
  DMA2D_InitStruct.DMA2D_PixelPerLine = xSize;
  DMA2D_Init(&DMA2D_InitStruct);
  
  /* Start Transfer */ 
  DMA2D_StartTransfer(); 
  
  /* Wait for CTC Flag activation */
  while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET)
  {
  }
}

/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  xSize: Buffer width
  * @param  ColorMode: Input color mode   
  * @retval None
  */
static void LL_ConvertLineToARGB8888(void *pSrc, void *pDst, uint32_t xSize, uint32_t ColorMode)
{    
  DMA2D_InitTypeDef      DMA2D_InitStruct;
  DMA2D_FG_InitTypeDef   DMA2D_FG_InitStruct;
  
  /* Configure the DMA2D Mode, Color Mode and output offset */
  DMA2D_DeInit();
  DMA2D_InitStruct.DMA2D_Mode = DMA2D_M2M_PFC;
  DMA2D_InitStruct.DMA2D_CMode = DMA2D_ARGB8888;
  DMA2D_InitStruct.DMA2D_OutputGreen = 0;
  DMA2D_InitStruct.DMA2D_OutputBlue = 0;
  DMA2D_InitStruct.DMA2D_OutputRed = 0;
  DMA2D_InitStruct.DMA2D_OutputAlpha = 0;
  DMA2D_InitStruct.DMA2D_OutputMemoryAdd = (uint32_t)pDst;
  DMA2D_InitStruct.DMA2D_OutputOffset = 0;
  DMA2D_InitStruct.DMA2D_NumberOfLine = 1;
  DMA2D_InitStruct.DMA2D_PixelPerLine = xSize;
  
  /* DMA2D Initialization */
  DMA2D_Init(&DMA2D_InitStruct);
  
  /* Foreground Configuration */
  DMA2D_FG_StructInit(&DMA2D_FG_InitStruct);
  DMA2D_FG_InitStruct.DMA2D_FGMA = (uint32_t)pSrc;
  DMA2D_FG_InitStruct.DMA2D_FGO = 0;
  DMA2D_FG_InitStruct.DMA2D_FGCM = ColorMode;
  DMA2D_FG_InitStruct.DMA2D_FGPFC_ALPHA_MODE = NO_MODIF_ALPHA_VALUE;
  DMA2D_FG_InitStruct.DMA2D_FGPFC_ALPHA_VALUE = 0xFF;
  DMA2D_FGConfig (&DMA2D_FG_InitStruct);
  
  DMA2D_FGStart(ENABLE);
  
  /* Start Transfer */ 
  DMA2D_StartTransfer(); 
  
  /* Wait for CTC Flag activation */
  while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET)
  {
  }
  
  DMA2D_FGStart(DISABLE);
}


LTDC_HandleTypeDef BSP_LCD_GetConfig( void )
{
  return hLtdcHandler;
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
