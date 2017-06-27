/**
  ******************************************************************************
  * @file    stm324x9i_eval_wifi.c
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    26-February-2014
  * @brief   This file provides a set of functions needed to manage the IO pins
  *          on STM324x9I-EVAL evaluation board.
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

/* File Info : -----------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive the IO module of the STM324x9I-EVAL evaluation 
     board.
   - The STMPE1600 IO expander device component driver must be included with this 
     driver in order to run the IO functionalities commanded by the IO expander 
     device mounted on the evaluation board.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the IO module using the BSP_IO_Init() function. This 
       function includes the MSP layer hardware resources initialization and the
       communication layer configuration to start the IO functionalities use.    
  
  + IO functionalities use
     o The IO pin mode is configured when calling the function BSP_IO_ConfigPin(), you 
       must specify the desired IO mode by choosing the "IO_ModeTypedef" parameter 
       predefined value.
     o If an IO pin is used in interrupt mode, the function BSP_IO_ITGetStatus() is 
       needed to get the interrupt status. To clear the IT pending bits, you should 
       call the function BSP_IO_ITClear() with specifying the IO pending bit to clear.
     o The IT is handled using the corresponding external interrupt IRQ handler,
       the user IT callback treatment is implemented on the same external interrupt
       callback.
     o To get/set an IO pin combination state you can use the functions 
       BSP_IO_ReadPin()/BSP_IO_WritePin() or the function BSP_IO_TogglePin() to toggle the pin 
       state.
 
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm324x9i_eval_wifi.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM324x9I_EVAL
  * @{
  */ 
  
/** @defgroup STM324x9I_EVAL_WIFI
  * @{
  */   
  
/* Private typedef -----------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WIFI_Private_Types_Definitions
  * @{
  */ 
  
/* Private define ------------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WIFI_Private_Defines
  * @{
  */
  
/**
  * @}
  */ 

/* Private macro -------------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WIFI_Private_Macros
  * @{
  */ 
  
/* Private variables ---------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WIFI_Private_Variables
  * @{
  */ 
static WiFi_DrvTypeDef *wifi_driver;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

uint32_t srcIP  = INADDR_NONE;
uint16_t srcPort  = PORT_NONE;
uint32_t destIP = INADDR_NONE;
uint32_t destPort = PORT_NONE;
uint8_t mysock = 0;
uint32_t selfIP = 0;
uint8_t *RXBuf = NULL;
osMessageQId WiFiRXEvent;
osPoolId mpool2;

extern osMessageQId WiFiTXEvent;
extern osMessageQId SPITXEvent;

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WIFI_Private_Function_Prototypes
  * @{
  */ 
static void SPI_Init(void);
static void SPI_DeInit(void);
    
/* Private functions ---------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WIFI_Private_Functions
  * @{
  */
void BSP_WiFi_Init(void)
{
	RXBuf = (uint8_t *)malloc(MAX_RX_BUFFER_SIZE);
	SPI_Init();
}

void BSP_WiFi_DeInit(void)
{
	SN8200_OFF();
	SPI_DeInit();
	if(RXBuf != NULL) {
		free(RXBuf);
		RXBuf = NULL;
	}
}

void BSP_WiFi_Start(void) 
{
	/* Initialize the WIFI driver structure */
	wifi_driver = &sn8200_wifi_drv;
	
	wifi_driver->Start();
}

uint16_t BSP_WiFi_RX(uint8_t *data)
{
  return wifi_driver->RX(data);
}

void BSP_WiFi_On() 
{
	wifi_driver->On();
}

void BSP_WiFi_Scan(void)
{
	wifi_driver->Scan();
}

void BSP_WiFi_Join(void)
{
	wifi_driver->Disconn();
	
	HAL_Delay(500);
	
	wifi_driver->Join();
}
	
void BSP_SNIC_GetIP()
{
	wifi_driver->SnicInit();

	HAL_Delay(10);
	
	wifi_driver->SnicIP();
	
	HAL_Delay(10);
	
	wifi_driver->SnicDhcp();
}

void BSP_SNIC_TCPClient(uint16_t bufsize, uint8_t timeout) 
{
	wifi_driver->tcpCreateSock(0, 0xFF, 0xFF);
	
	HAL_Delay(1000);
	
	if(mysock)
	{
		if(wifi_driver->getTCP() == CMD_ERROR)
		{
			printf("\r\nInvalid Server");
			return;
		}
		wifi_driver->tcpConnectToServer(mysock, destIP, destPort, timeout);
	}
}

void BSP_SNIC_TCPServer(uint8_t maxClient) 
{
	if(wifi_driver->setTCP() == CMD_ERROR) 
	{
		printf("\r\nInvalid Server to create");
		return;
	}
	else printf("\r\nIP = %d, port = %d", srcIP, srcPort);
	
	wifi_driver->tcpCreateSock(1, 0, (uint16_t)11000);
	
	HAL_Delay(330);
	
	if(mysock)
	{
		wifi_driver->tcpCreateConnection(mysock, maxClient);
	}
}

void BSP_SNIC_UDPServer(void) 
{
	/*if(wifi_driver->setTCP() == CMD_ERROR) 
	{
		printf("\r\nInvalid Server to create");
		return;
	}
	else printf("\r\nIP = %d, port = %d", srcIP, srcPort);*/
	
	wifi_driver->udpCreateSock(1, 0, (uint16_t)11000);
	
	HAL_Delay(10);
	
	wifi_driver->udpStartReceive(mysock);
}

void BSP_SNIC_TCPSend(uint8_t sock, uint8_t *buf, uint16_t len, uint16_t index) 
{
	wifi_driver->tcpSend(sock, buf, len, index);
}

void BSP_SNIC_UDPSend(uint32_t ip, uint16_t port, uint8_t *buf, uint16_t len, uint16_t index, uint8_t mode) 
{
	wifi_driver->udpSend(mysock, ip, port, buf, len, index, mode);
}

void BSP_WiFi_GetStatus(void) 
{
	wifi_driver->GetStatus();
}

void BSP_WiFi_Leave(void) 
{
	wifi_driver->Leave();
}

void BSP_WiFi_Off(void) 
{
	if(mysock) wifi_driver->closeSock(mysock);
	HAL_Delay(500);
	wifi_driver->SnicCleanup();
	HAL_Delay(500);
	wifi_driver->Disconn();
	HAL_Delay(500);
	wifi_driver->Off();
	selfIP = 0;
}

/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
  /* Enable SPI1 clock */
  __SPI1_CLK_ENABLE();
	/* Enable DMA2 clock */
  __DMA2_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/
	/* Configure PC13 pin as input floating */
	GPIO_InitStruct.Pin       = GPIO_PIN_13;
  GPIO_InitStruct.Mode      = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_5;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  /* SPI MISO, MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = DMA2_Stream5;
  
  hdma_tx.Init.Channel             = DMA_CHANNEL_3;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  HAL_DMA_Init(&hdma_tx);
  
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmatx, hdma_tx);
    
  /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = DMA2_Stream2;
  
  hdma_rx.Init.Channel             = DMA_CHANNEL_3;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4; 

  HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmarx, hdma_rx);
	
	/*##-4- Configure the NVIC for DMA #########################################*/ 
  /* NVIC configuration for DMA transfer complete interrupt (SPI1_TX) */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

#ifdef EXAMPLE
	/* NVIC configuration for DMA transfer complete interrupt (SPI1_RX) */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);   
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
}

/**
  * @brief SPI MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
	static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
	
  /*##-1- Reset peripherals ##################################################*/
  __SPI1_FORCE_RESET();
  __SPI1_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure SPI SCK as alternate function  */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
  /* Configure SPI MISO, MOSI as alternate function  */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);
	
	/*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  HAL_DMA_DeInit(&hdma_tx); 
  /* De-Initialize the DMA Stream associate to reception process */
  HAL_DMA_DeInit(&hdma_rx);
	
	/*##-4- Disable the NVIC for DMA ###########################################*/
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
}

/**
  * @brief  TxRx Transfer completed callback
  * @param  hspi: SPI handle. 
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8_t *TmpBuf = (uint8_t *)osPoolCAlloc(mpool2);
	memcpy(TmpBuf, RXBuf, MAX_RX_BUFFER_SIZE);
	osMessagePut(WiFiRXEvent, (uint32_t)TmpBuf, 0);
}

/**
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	printf("\r\nerr");
}

static void SPI_Init(void)
{
  /*!< SPI configuration */
  SpiHandle.Instance               = SPI1;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
  SpiHandle.Init.Mode = SPI_MODE_MASTER;

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    printf("\r\nSPI1 Initialization Error");
  }
}

static void SPI_DeInit(void)
{
  if(HAL_SPI_DeInit(&SpiHandle) != HAL_OK)
  {
    /* DeInitialization Error */
    printf("\r\nSPI1 DeInitialization Error");
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
