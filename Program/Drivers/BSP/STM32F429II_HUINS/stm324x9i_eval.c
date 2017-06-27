/**
  ******************************************************************************
  * @file    stm324x9i_eval.c
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    26-February-2014
  * @brief   This file provides a set of firmware functions to manage LEDs,
  *          push-buttons and COM ports available on STM324x9I-EVAL evaluation
  *          board(MB1045) RevB from STMicroelectronics.
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

/* File Info: ------------------------------------------------------------------
                                   User NOTE

   This driver requires the stm324x9i_eval_io to manage the joystick

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm324x9i_eval.h"
#include "stm324x9i_eval_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
  * @{
  */

/** @addtogroup STM324x9I_EVAL
  * @{
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL
  * @{
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Defines
  * @{
  */

/**
 * @brief STM324x9I EVAL BSP Driver version number V2.0.1
   */
#define __STM324x9I_EVAL_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM324x9I_EVAL_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM324x9I_EVAL_BSP_VERSION_SUB2   (0x01) /*!< [15:8]  sub2 version */
#define __STM324x9I_EVAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM324x9I_EVAL_BSP_VERSION         ((__STM324x9I_EVAL_BSP_VERSION_MAIN << 24)\
                                             |(__STM324x9I_EVAL_BSP_VERSION_SUB1 << 16)\
                                             |(__STM324x9I_EVAL_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM324x9I_EVAL_BSP_VERSION_RC))
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Variables
  * @{
  */
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT,
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN};

/* UART handler declaration */
UART_HandleTypeDef UartHandle1, UartHandle2;
UART_HandleTypeDef WIFI_UartHandle;
USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2};
GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};
const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};
const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF, EVAL_COM2_TX_AF};
const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF, EVAL_COM2_RX_AF};
uint8_t GetChBuf[2];

static I2C_HandleTypeDef heval_I2c;

/**
  * @}
  */

/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */
static void     I2Cx_MspInit(void);
static void     I2Cx_Init(void);
static void     I2Cx_ITConfig(void);
static void     I2Cx_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  I2Cx_Read(uint8_t Addr, uint8_t Reg);
static HAL_StatusTypeDef I2Cx_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_WriteMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
//static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
static void     I2Cx_Error(uint8_t Addr);
static void I2Cx_WriteNoReg(uint16_t Addr, uint8_t Value);
static uint8_t I2Cx_ReadNoReg(uint16_t Addr);
static HAL_StatusTypeDef I2Cx_WriteMultipleNoReg(uint16_t Addr, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef I2Cx_ReadMultipleNoReg(uint16_t Addr, uint8_t *Buffer, uint16_t Length);

/* IOExpander IO functions */
void            IOE_Init(void);
void            IOE_ITConfig(void);
void            IOE_Delay(uint32_t Delay);
void            IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t        IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void            IOE_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

/* AUDIO IO functions */
void            AUDIO_IO_Init(void);
void            AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint16_t        AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void            AUDIO_IO_Delay(uint32_t Delay);

/* CAMERA IO functions */
void            CAMERA_IO_Init(void);
void            CAMERA_Delay(uint32_t Delay);
void            CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t         CAMERA_IO_Read(uint8_t Addr, uint8_t Reg);

/**
  * @}
  */


/** @defgroup STM324x9I_EVAL_LOW_LEVEL_Private_Functions
  * @{
  */

/**
  * @brief  This method returns the STM324x9I EVAL BSP Driver revision
  * @param  None
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
	return __STM324x9I_EVAL_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured.
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO_LED clock */
	LEDx_GPIO_CLK_ENABLE(Led);

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = GPIO_PIN[Led];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
	HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
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
void BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable GPIO clock */
	EVAL_COMx_TX_GPIO_CLK_ENABLE(COM);
	EVAL_COMx_RX_GPIO_CLK_ENABLE(COM);

	/* Enable USART clock */
	EVAL_COMx_CLK_ENABLE(COM);

	/* Configure USART Tx as alternate function */
	GPIO_InitStruct.Pin = COM_TX_PIN[COM];
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = COM_TX_AF[COM];
	HAL_GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStruct);

	/* Configure USART Rx as alternate function */
	GPIO_InitStruct.Pin = COM_RX_PIN[COM];
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate = COM_RX_AF[COM];
	HAL_GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStruct);

	/* NVIC for USART */
	HAL_NVIC_SetPriority(EVAL_COMx_INT_ENABLE(COM), 5, 0);
	HAL_NVIC_EnableIRQ(EVAL_COMx_INT_ENABLE(COM));

	/* USART configuration */
	huart->Instance = COM_USART[COM];
	HAL_UART_Init(huart);
}

void BSP_COM1_Init(void)
{
	/* Configure the USART3 peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART3 configured as follow:
	  - Word Length = 8 Bits
	  - Stop Bit = One Stop bit
	  - Parity = NONE parity
	  - BaudRate = 115200 baud
	  - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle1.Instance = EVAL_COM1;

	UartHandle1.Init.BaudRate = 115200; //(460800); //115200;
	UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle1.Init.StopBits = UART_STOPBITS_1;
	UartHandle1.Init.Parity = UART_PARITY_NONE;
	UartHandle1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle1.Init.Mode = UART_MODE_TX | UART_MODE_RX;

	BSP_COM_Init(COM1, &UartHandle1);
}

void BSP_COM2_Init(void)
{
	/* Configure the USART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART6 configured as follow:
	  - Word Length = 8 Bits
	  - Stop Bit = One Stop bit
	  - Parity = NONE parity
	  - BaudRate = 115200 baud
	  - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle2.Instance = EVAL_COM2;

	UartHandle2.Init.BaudRate = 115200;
	UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle2.Init.StopBits = UART_STOPBITS_1;
	UartHandle2.Init.Parity = UART_PARITY_NONE;
	UartHandle2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle2.Init.Mode = UART_MODE_TX | UART_MODE_RX;

	BSP_COM_Init(COM2, &UartHandle2);
}

void BSP_WIFI_UART_Init(uint32_t BaudRate)
{
	 /* UART 설정과 UART 핀, Interrupt 설정을 한다. */
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Configure the USART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	  - Word Length = 8 Bits
	  - Stop Bit = One Stop bit
	  - Parity = NONE parity
	  - BaudRate = 9600 baud (WiFi Default)
	  - Hardware flow control disabled (RTS and CTS signals) */
	
	/* WIFI_UART == UART2 */
	WIFI_UartHandle.Instance = WIFI_UART;

  /* BaudRate는 WiFi 기본 설정이 9600으로 해주고 
    WordLength_8B 로 설정
    STOPBIT는 1 / Parity는 사용하지 않음
    HwFlowCtl CTS와 RTS 사용
    TX와 RX 모드로 설정 */	
	WIFI_UartHandle.Init.BaudRate = BaudRate; //9600;
	WIFI_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	WIFI_UartHandle.Init.StopBits = UART_STOPBITS_1;
	WIFI_UartHandle.Init.Parity = UART_PARITY_NONE;
	WIFI_UartHandle.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; //UART_HWCONTROL_NONE;
	WIFI_UartHandle.Init.Mode = UART_MODE_TX | UART_MODE_RX;


	/* Enable GPIO clock */
	WIFI_UART_TX_GPIO_CLK_ENABLE();
	WIFI_UART_RX_GPIO_CLK_ENABLE();

	/* Enable USART clock */
	WIFI_UART_CLK_ENABLE();

	/* Configure USART Tx as alternate function */
	GPIO_InitStruct.Pin = WIFI_UART_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = WIFI_UART_TX_AF;
	HAL_GPIO_Init(WIFI_UART_TX_GPIO_PORT, &GPIO_InitStruct);

	/* Configure USART Rx as alternate function */
	GPIO_InitStruct.Pin = WIFI_UART_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate = WIFI_UART_RX_AF;
	HAL_GPIO_Init(WIFI_UART_RX_GPIO_PORT, &GPIO_InitStruct);

	if(WIFI_UartHandle.Init.HwFlowCtl & UART_HWCONTROL_CTS)
	{
		/* Configure USART Rx as alternate function */
		GPIO_InitStruct.Pin = WIFI_UART_CTS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = WIFI_UART_CTS_AF;
		HAL_GPIO_Init(WIFI_UART_CTS_GPIO_PORT, &GPIO_InitStruct);
	}

	if(WIFI_UartHandle.Init.HwFlowCtl & UART_HWCONTROL_RTS)
	{
		/* Configure USART Rx as alternate function */
		GPIO_InitStruct.Pin = WIFI_UART_RTS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = WIFI_UART_RTS_AF;
		HAL_GPIO_Init(WIFI_UART_RTS_GPIO_PORT, &GPIO_InitStruct);
	}

	/* NVIC for USART */
	HAL_NVIC_SetPriority(WIFI_UART_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(WIFI_UART_IRQn);

	/* USART configuration */
	HAL_UART_Init(&WIFI_UartHandle);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&WIFI_UartHandle, UART_IT_RXNE);

}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* I2C Routines**********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  None
  * @retval None
  */
static void I2Cx_MspInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*** Configure the GPIOs ***/
	/* Enable GPIO clock */
	EVAL_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

	/* Configure I2C Tx as alternate function */
	GPIO_InitStruct.Pin = EVAL_I2Cx_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = EVAL_I2Cx_SCL_SDA_AF;
	HAL_GPIO_Init(EVAL_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);

	/* Configure I2C Rx as alternate function */
	GPIO_InitStruct.Pin = EVAL_I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = EVAL_I2Cx_SCL_SDA_AF;
	HAL_GPIO_Init(EVAL_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);

	/*** Configure the I2C peripheral ***/
	/* Enable I2C clock */
	EVAL_I2Cx_CLK_ENABLE();

	/* Force the I2C peripheral clock reset */
	EVAL_I2Cx_FORCE_RESET();

	/* Release the I2C peripheral clock reset */
	EVAL_I2Cx_RELEASE_RESET();

	/* Enable and set I2Cx Interrupt to the highest priority */
	HAL_NVIC_SetPriority(EVAL_I2Cx_EV_IRQn, 0x05, 0);
	HAL_NVIC_EnableIRQ(EVAL_I2Cx_EV_IRQn);

	/* Enable and set I2Cx Interrupt to the highest priority */
	HAL_NVIC_SetPriority(EVAL_I2Cx_ER_IRQn, 0x05, 0);
	HAL_NVIC_EnableIRQ(EVAL_I2Cx_ER_IRQn);
}

/**
  * @brief  Initializes I2C HAL.
  * @param  None
  * @retval None
  */
static void I2Cx_Init(void)
{
	if(HAL_I2C_GetState(&heval_I2c) == HAL_I2C_STATE_RESET)
	{
		heval_I2c.Instance = I2C2;
		heval_I2c.Init.ClockSpeed      = I2C_SPEED;
		heval_I2c.Init.DutyCycle       = I2C_DUTYCYCLE_2;
		heval_I2c.Init.OwnAddress1     = 0;
		heval_I2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
		heval_I2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		heval_I2c.Init.OwnAddress2     = 0;
		heval_I2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		heval_I2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;

		/* Init the I2C */
		I2Cx_MspInit();
		HAL_I2C_Init(&heval_I2c);
	}
}

/**
  * @brief  Configures I2C Interrupt.
  * @param  None
  * @retval None
  */
static void I2Cx_ITConfig(void)
{
	static uint8_t I2C_IT_Enabled = 0;
	GPIO_InitTypeDef  GPIO_InitStruct;

	if(I2C_IT_Enabled == 0)
	{
		I2C_IT_Enabled = 1;
		/* Enable the GPIO EXTI clock */
		__GPIOE_CLK_ENABLE();
		__GPIOI_CLK_ENABLE();
		__SYSCFG_CLK_ENABLE();

		GPIO_InitStruct.Pin   = GPIO_PIN_2;
		GPIO_InitStruct.Pull  = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Pin   = GPIO_PIN_3;
		HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

		/* Enable and set GPIO EXTI Interrupt to the lowest priority */
		HAL_NVIC_SetPriority((IRQn_Type)(EXTI2_IRQn), 0x0F, 0x0F);
		HAL_NVIC_SetPriority((IRQn_Type)(EXTI3_IRQn), 0x0F, 0x0F);
		HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI2_IRQn));
		HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI3_IRQn));
	}
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
static void I2Cx_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&heval_I2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 100);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error(Addr);
	}
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
static uint8_t I2Cx_Read(uint8_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t Value = 0;

	status = HAL_I2C_Mem_Read(&heval_I2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error(Addr);
	}

	return Value;
}

/**
  * @brief  Reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(&heval_I2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* I2C error occured */
		I2Cx_Error(Addr);
	}
	return status;
}

/**
  * @brief  Write a value in a register of the device through BUS in using DMA mode
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  pBuffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_WriteMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&heval_I2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Re-Initiaize the I2C Bus */
		I2Cx_Error(Addr);
	}
	return status;
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
/*static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
  return (HAL_I2C_IsDeviceReady(&heval_I2c, DevAddress, Trials, 1000));
}*/

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(uint8_t Addr)
{
	/* De-initialize the I2C comunication bus */
	HAL_I2C_DeInit(&heval_I2c);

	/* Re-Initiaize the I2C comunication bus */
	I2Cx_Init();
}

/**
  * @brief  Writes a single data for no register inde.
  * @param  Addr: I2C address
  * @param  Value: Data to be written
  * @retval None
  */
static void I2Cx_WriteNoReg(uint16_t Addr, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Master_Transmit(&heval_I2c, Addr, &Value, 1, 100);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error(Addr);
	}
}

/**
  * @brief  Reads a single data for no register index.
  * @param  Addr: I2C address
  * @retval Read data
  */
static uint8_t I2Cx_ReadNoReg(uint16_t Addr)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t Value = 0;

	status = HAL_I2C_Master_Receive(&heval_I2c, Addr, &Value, 1, 1000);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error(Addr);
	}

	return Value;
}

/**
  * @brief  Write a value in a register of the device through BUS in using DMA mode for no register index.
  * @param  Addr: Device address on BUS Bus.
  * @param  pBuffer: The target register value to be written
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_WriteMultipleNoReg(uint16_t Addr, uint8_t *Buffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Master_Transmit(&heval_I2c, Addr, Buffer, Length, 1000);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Re-Initiaize the I2C Bus */
		I2Cx_Error(Addr);
	}
	return status;
}

/**
  * @brief  Reads multiple data for no register index.
  * @param  Addr: I2C address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultipleNoReg(uint16_t Addr, uint8_t *Buffer, uint16_t Length)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Master_Receive(&heval_I2c, Addr, Buffer, Length, 1000);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* I2C error occured */
		I2Cx_Error(Addr);
	}
	return status;
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK IOE ***********************************/

/**
  * @brief  Initializes IOE low level.
  * @param  None
  * @retval None
  */
void IOE_Init(void)
{
	I2Cx_Init();
}

/**
  * @brief  Configures IOE low level interrupt.
  * @param  None
  * @retval None
  */
void IOE_ITConfig(void)
{
	I2Cx_ITConfig();
}

/**
  * @brief  IOE writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	I2Cx_Write(Addr, Reg, Value);
}

/**
  * @brief  IOE reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
	return I2Cx_Read(Addr, Reg);
}

/**
  * @brief  IOE reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
	return I2Cx_ReadMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE writes multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */
void IOE_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
	I2Cx_WriteMultiple(Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void IOE_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

/********************************* LINK AUDIO *********************************/
/**
  * @brief  Initializes Audio low level.
  * @param  None
  * @retval None
  */
void AUDIO_IO_Init(void)
{
	I2Cx_Init();
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

	I2Cx_WriteMultiple(Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg)
{
	uint16_t Read_Value = 0, tmp = 0;

	I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Read_Value, 2); 

	tmp = ((uint16_t)(Read_Value >> 8) & 0x00FF);

	tmp |= ((uint16_t)(Read_Value << 8)& 0xFF00);

	Read_Value = tmp;

	return Read_Value;
}

/**
  * @brief  AUDIO Codec delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void AUDIO_IO_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

/********************************* LINK CAMERA ********************************/

/**
  * @brief  Initializes Camera low level.
  * @param  None
  * @retval None
  */
void CAMERA_IO_Init(void)
{
	I2Cx_Init();
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
	I2Cx_Write(Addr, Reg, Value);
}

/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t CAMERA_IO_Read(uint8_t Addr, uint8_t Reg)
{
	return I2Cx_Read(Addr, Reg);
}

/**
  * @brief  Camera delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void CAMERA_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

void ADC_Init(void)
{
	IOE_Init();

	IOE_WriteNoReg(I2C_ADC_ADDR, 0x8A); // Setup byte, CLK(1=external clock), nRST(1=no action)
	IOE_Delay(1);
	IOE_WriteNoReg(I2C_ADC_ADDR, 0x88); // Setup byte, CLK(1=external clock), nRST(0=reset)
	IOE_Delay(1);
	IOE_WriteNoReg(I2C_ADC_ADDR, 0x8A); // Setup byte, CLK(1=external clock), nRST(1=no action)
	IOE_Delay(1);
}

uint16_t ADC_Get_Data_ALL(uint16_t *adc_data)
{
	uint16_t result;

	result = ADC_Get_Data(0, &adc_data[0]);
	result = ADC_Get_Data(1, &adc_data[1]);
	result = ADC_Get_Data(2, &adc_data[2]);
	result = ADC_Get_Data(3, &adc_data[3]);

	return result;
}

uint16_t ADC_Get_Data(uint8_t channel, uint16_t *adc_data)
{
	uint16_t result;
	uint16_t all_data[4];
	uint8_t *value = (uint8_t *) all_data;

	IOE_WriteNoReg(I2C_ADC_ADDR, 0x61 | ((channel & 0xF) << 1)); // Configuration byte, scan(00), cs0(1), SGL(1=single-ended)
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);	
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);
	IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // dummy
	HAL_Delay(1);

	result = IOE_ReadMultipleNoReg(I2C_ADC_ADDR, value, 2); // get channel Data

	if(result == HAL_OK && channel<8) // all channel Data Read
	{
		_swap(value[0], value[1]); // exchange LSB to MSB
		adc_data[0] = all_data[0] & 0x3FF;
	}

	return result;
}

void GY291_Init(void)
{
	IOE_Write(GY291_ADDRESS,DATA_FORMAT,0x01);
	IOE_Delay(1);
	IOE_Write(GY291_ADDRESS,POWER_CTL,0x08);
	IOE_Delay(1);
}
void GY291_Get_Data(void)
{
	uint16_t _buff[6];
	uint8_t *value = (uint8_t *) _buff;
	
	IOE_ReadMultiple(GY291_ADDRESS,DATAX0,value,12);
	
//	int x = (((int)_buff[1]) << 8) | _buff[0];    
//	int y = (((int)_buff[3]) << 8) | _buff[2]; 
//	int z = (((int)_buff[5]) << 8) | _buff[4]; 
}
/**
  * @brief  IOE writes single data.
  * @param  Addr: I2C address
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_WriteNoReg(uint8_t Addr, uint8_t Value)
{
	I2Cx_WriteNoReg(Addr, Value);
}

/**
  * @brief  IOE reads single data.
  * @param  Addr: I2C address
  * @retval Read data
  */
uint8_t IOE_ReadNoReg(uint8_t Addr)
{
	return I2Cx_ReadNoReg(Addr);
}

/**
  * @brief  IOE reads multiple data.
  * @param  Addr: I2C address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_ReadMultipleNoReg(uint8_t Addr, uint8_t *Buffer, uint16_t Length)
{
	return I2Cx_ReadMultipleNoReg(Addr, Buffer, Length);
}

/**
  * @brief  IOE writes multiple data.
  * @param  Addr: I2C address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */
void IOE_WriteMultipleNoReg(uint8_t Addr, uint8_t *Buffer, uint16_t Length)
{
	I2Cx_WriteMultipleNoReg(Addr, Buffer, Length);
}

uint8_t BSP_UART_GetChar(uint8_t COM)
{
	uint8_t ch = 0;

	if(COM == COM1)
	{
		if( (UartHandle1.Instance->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE)
		{
			ch = UartHandle1.Instance->DR & 0xFF;
		}
	}
	else if(COM == COM2)
	{
		if( (UartHandle2.Instance->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE)
		{
			ch = UartHandle2.Instance->DR & 0xFF;
		}
	}

	return ch;
}

void BSP_UART_PutChar(uint8_t COM, uint8_t ch)
{
	if(COM == COM1)
	{
		HAL_UART_Transmit(&UartHandle1, &ch, 1, 10);
	}
	else if(COM == COM2)
	{
		HAL_UART_Transmit(&UartHandle2, &ch, 1, 10);
	}
}

void BSP_UART_Printf(uint8_t COM, const char *fmt,...)
{
	va_list ap;
	char string[1024];
	unsigned int length, i;

	va_start(ap, fmt);
	vsprintf(string, fmt, ap);
	length = strlen(string);
	for(i=0; i<length; i++)
	{
		if(string[i] == '\n') putchar('\r');
		BSP_UART_PutChar(COM, string[i]);
	}
	va_end(ap);
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
