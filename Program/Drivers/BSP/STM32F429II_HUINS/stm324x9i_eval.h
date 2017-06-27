/**
  ******************************************************************************
  * @file    stm324x9i_eval.h
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    26-February-2014
  * @brief   This file contains definitions for STM324x9I_EVAL's LEDs,
  *          push-buttons and COM ports hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#ifndef __STM324X9I_EVAL_H
#define __STM324X9I_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
  * @{
  */

/** @addtogroup STM324x9I_EVAL
  * @{
  */

/** @addtogroup STM324x9I_EVAL_LOW_LEVEL
  * @{
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3

}Led_TypeDef;

typedef enum
{
  COM1 = 0,
  COM2 = 1

}COM_TypeDef;

typedef enum
{
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5

}JOYState_TypeDef;

/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Constants
  * @{
  */

/**
  * @brief  Define for STM32429II_HUINS board
  */
#if !defined (USE_STM32F429II_HUINS)
 #define USE_STM32F429II_HUINS
#endif

/** @addtogroup STM324x9I_EVAL_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             4

#define LED1_PIN                         GPIO_PIN_8
#define LED1_GPIO_PORT                   GPIOI
#define LED1_GPIO_CLK_ENABLE()           __GPIOI_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOI_CLK_DISABLE()


#define LED2_PIN                         GPIO_PIN_9
#define LED2_GPIO_PORT                   GPIOI
#define LED2_GPIO_CLK_ENABLE()           __GPIOI_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __GPIOI_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_10
#define LED3_GPIO_PORT                   GPIOI
#define LED3_GPIO_CLK_ENABLE()           __GPIOI_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOI_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_11
#define LED4_GPIO_PORT                   GPIOI
#define LED4_GPIO_CLK_ENABLE()           __GPIOI_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOI_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LED1_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_ENABLE() : LED4_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED1_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_DISABLE() : LED4_GPIO_CLK_DISABLE())

/**
  * @}
  */

/** @addtogroup STM324x9I_EVAL_LOW_LEVEL_COM
  * @{
  */
#define COMn                             2

extern UART_HandleTypeDef UartHandle1, UartHandle2;
extern UART_HandleTypeDef WIFI_UartHandle;

/**
 * @brief Definition for WIFI COM port, connected to USART2
 */
#define WIFI_UART                          USART2
#define WIFI_UART_CLK_ENABLE()             __USART2_CLK_ENABLE()
#define WIFI_UART_CLK_DISABLE()            __USART2_CLK_DISABLE()

#define WIFI_UART_TX_PIN                   GPIO_PIN_2
#define WIFI_UART_TX_GPIO_PORT             GPIOA
#define WIFI_UART_TX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()
#define WIFI_UART_TX_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()
#define WIFI_UART_TX_AF                    GPIO_AF7_USART2

#define WIFI_UART_RX_PIN                   GPIO_PIN_3
#define WIFI_UART_RX_GPIO_PORT             GPIOA
#define WIFI_UART_RX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()
#define WIFI_UART_RX_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()
#define WIFI_UART_RX_AF                    GPIO_AF7_USART2

#define WIFI_UART_CTS_PIN                   GPIO_PIN_3
#define WIFI_UART_CTS_GPIO_PORT             GPIOD
#define WIFI_UART_CTS_GPIO_CLK_ENABLE()     __GPIOD_CLK_ENABLE()
#define WIFI_UART_CTS_GPIO_CLK_DISABLE()    __GPIOD_CLK_DISABLE()
#define WIFI_UART_CTS_AF                    GPIO_AF7_USART2

#define WIFI_UART_RTS_PIN                   GPIO_PIN_4
#define WIFI_UART_RTS_GPIO_PORT             GPIOD
#define WIFI_UART_RTS_GPIO_CLK_ENABLE()     __GPIOD_CLK_ENABLE()
#define WIFI_UART_RTS_GPIO_CLK_DISABLE()    __GPIOD_CLK_DISABLE()
#define WIFI_UART_RTS_AF                    GPIO_AF7_USART2

#define WIFI_UART_IRQn                     USART2_IRQn

#define	WiFlySerial_IRQHandler	USART2_IRQHandler
/**
 * @brief Definition for COM port1, connected to USART1
 */
#define EVAL_COM1                          USART3
#define EVAL_COM1_CLK_ENABLE()             __USART3_CLK_ENABLE()
#define EVAL_COM1_CLK_DISABLE()            __USART3_CLK_DISABLE()

#define EVAL_COM1_TX_PIN                   GPIO_PIN_10
#define EVAL_COM1_TX_GPIO_PORT             GPIOB
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE()
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()    __GPIOB_CLK_DISABLE()
#define EVAL_COM1_TX_AF                    GPIO_AF7_USART3

#define EVAL_COM1_RX_PIN                   GPIO_PIN_11
#define EVAL_COM1_RX_GPIO_PORT             GPIOB
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE()
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()    __GPIOB_CLK_DISABLE()
#define EVAL_COM1_RX_AF                    GPIO_AF7_USART3

#define EVAL_COM1_IRQn                     USART3_IRQn

#define EVAL_COM2                          USART6
#define EVAL_COM2_CLK_ENABLE()             __USART6_CLK_ENABLE()
#define EVAL_COM2_CLK_DISABLE()            __USART6_CLK_DISABLE()

#define EVAL_COM2_TX_PIN                   GPIO_PIN_14
#define EVAL_COM2_TX_GPIO_PORT             GPIOG
#define EVAL_COM2_TX_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE()
#define EVAL_COM2_TX_GPIO_CLK_DISABLE()    __GPIOG_CLK_DISABLE()
#define EVAL_COM2_TX_AF                    GPIO_AF8_USART6

#define EVAL_COM2_RX_PIN                   GPIO_PIN_9
#define EVAL_COM2_RX_GPIO_PORT             GPIOG
#define EVAL_COM2_RX_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE()
#define EVAL_COM2_RX_GPIO_CLK_DISABLE()    __GPIOG_CLK_DISABLE()
#define EVAL_COM2_RX_AF                    GPIO_AF8_USART6

#define EVAL_COM2_IRQn                     USART6_IRQn

#define EVAL_COMx_CLK_ENABLE(__INDEX__)            (((__INDEX__) == 0) ? EVAL_COM1_CLK_ENABLE() : EVAL_COM2_CLK_ENABLE())
#define EVAL_COMx_CLK_DISABLE(__INDEX__)           (((__INDEX__) == 0) ? EVAL_COM1_CLK_DISABLE() : EVAL_COM2_CLK_DISABLE())

#define EVAL_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_ENABLE() : EVAL_COM2_TX_GPIO_CLK_ENABLE())
#define EVAL_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_TX_GPIO_CLK_DISABLE() : EVAL_COM2_TX_GPIO_CLK_DISABLE())

#define EVAL_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_ENABLE() : EVAL_COM2_RX_GPIO_CLK_ENABLE())
#define EVAL_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_RX_GPIO_CLK_DISABLE() : EVAL_COM2_RX_GPIO_CLK_DISABLE())

#define EVAL_COMx_INT_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? EVAL_COM1_IRQn : EVAL_COM2_IRQn)


/**
  * @brief Eval Pins definition
  */

/* Exported constant IO ------------------------------------------------------*/
#define IO_I2C_ADDRESS                   0x84
#define TS_I2C_ADDRESS                   0x82
#define CAMERA_I2C_ADDRESS               0x60
#define AUDIO_I2C_ADDRESS                0x34
/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   stm324x9I_eval.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                        100000
#endif /* I2C_SPEED */

/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for I2Cx clock resources */
#define EVAL_I2Cx                             I2C2
#define EVAL_I2Cx_CLK_ENABLE()                __I2C2_CLK_ENABLE()
#define EVAL_DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define EVAL_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __GPIOH_CLK_ENABLE()

#define EVAL_I2Cx_FORCE_RESET()               __I2C2_FORCE_RESET()
#define EVAL_I2Cx_RELEASE_RESET()             __I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define EVAL_I2Cx_SCL_PIN                     GPIO_PIN_4
#define EVAL_I2Cx_SCL_SDA_GPIO_PORT           GPIOH
#define EVAL_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C2
#define EVAL_I2Cx_SDA_PIN                     GPIO_PIN_5

/* I2C interrupt requests */
#define EVAL_I2Cx_EV_IRQn                     I2C2_EV_IRQn
#define EVAL_I2Cx_ER_IRQn                     I2C2_ER_IRQn

#define I2C_ADDR_MAX11646	(0x6C)
#define I2C_ADDR_MAX11606	(0x68)
#define I2C_ADDR_MAX11608	(0x66) // uBrain Core V2.0  ADC 8 Channel 

#define  I2C_ADC_ADDR	(I2C_ADDR_MAX11608)

#define _swap(a, b)	{ a^=b; b^=a; a^=b; }


// HyunKH
// GY-291 ¡Æ¢®¨ùO¥ì¥ì ¨ù¨ú¨ù¡©
#define GY291_ADDRESS 					0x3B
#define POWER_CTL  0x2D	//Power Control Register 
#define DATA_FORMAT  0x31
#define DATAX0  0x32	//X-Axis Data 0 
#define DATAX1  0x33	//X-Axis Data 1 
#define DATAY0  0x34	//Y-Axis Data 0 
#define DATAY1  0x35	//Y-Axis Data 1 
#define DATAZ0  0x36	//Z-Axis Data 0 
#define DATAZ1  0x37	//Z-Axis Data 1 

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Exported_Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);

void             BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *husart);
void BSP_COM1_Init(void);
void BSP_COM2_Init(void);

void BSP_WIFI_UART_Init(uint32_t BaudRate);

/* IOExpander IO functions */
void IOE_WriteNoReg(uint8_t Addr, uint8_t Value);
uint8_t IOE_ReadNoReg(uint8_t Addr);
uint16_t IOE_ReadMultipleNoReg(uint8_t Addr, uint8_t *Buffer, uint16_t Length);
void IOE_WriteMultipleNoReg(uint8_t Addr, uint8_t *Buffer, uint16_t Length);

/* MAX11646 functions */
void ADC_Init(void);
uint16_t ADC_Get_Data_ALL(uint16_t *adc_data);
uint16_t ADC_Get_Data(uint8_t channel, uint16_t *adc_data);

uint8_t BSP_UART_GetChar(uint8_t COM);
void BSP_UART_PutChar(uint8_t COM, uint8_t ch);
void BSP_UART_Printf(uint8_t COM, const char *fmt,...);

void GY291_Init(void);
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM324X9I_EVAL_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
