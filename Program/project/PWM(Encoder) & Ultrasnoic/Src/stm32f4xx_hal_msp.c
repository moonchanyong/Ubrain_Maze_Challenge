/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   HAL MSP module.    
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
	
	if(htim->Instance == TIM8)
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIM8 Peripheral clock enable */
		__TIM8_CLK_ENABLE();
		/* Enable GPIOC Channels Clock */
		__GPIOC_CLK_ENABLE();
		
		/*##-2- Configure I/Os #############	########################################*/
		/* Configure PC.6, 7 (TIM3_Channel1,2) in output, push-pull, alternate function mode*/ 
		/* Common configuration for all channels */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		/* Channel 3 configuration */
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 ;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);	
	}
	else	if(htim->Instance == TIM4)	
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIM4 Peripheral clock enable */
		__TIM4_CLK_ENABLE();
		/* Enable GPIOD Channels Clock */
		__GPIOD_CLK_ENABLE();
		
		/*##-2- Configure I/Os #############	########################################*/
		/* Configure PD.12, 13 (TIM4_Channel1,2) in output, push-pull, alternate function mode*/ 
		/* Common configuration for all channels */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		/* Channel 3 configuration */
		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}	
	else	if(htim->Instance == TIM10)	
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIMx Peripheral clock enable */
		__TIM10_CLK_ENABLE();
		/* Enable GPIO Channels Clock */
		__GPIOF_CLK_ENABLE();
	 		
		/*##-2- Configure I/Os #############	########################################*/
				GPIO_InitStruct.Pin = GPIO_PIN_6 ;
		/* Common configuration for all channels */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
		/* Channel 3 configuration */
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	}
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx Peripheral clock enable */
	__TIM3_CLK_ENABLE();

	/* Enable GPIO channels Clock */
	__GPIOB_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0 ;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7 ;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*##-2- Configure the NVIC for TIMx #########################################*/
	HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);

	/* Enable the TIM3 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
