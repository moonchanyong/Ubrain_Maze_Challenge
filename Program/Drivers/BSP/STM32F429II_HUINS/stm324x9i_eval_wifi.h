/**
  ******************************************************************************
  * @file    stm324x9i_eval_wifi.h
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    26-February-2014
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm324x9i_eval_wifi.c driver.
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
#ifndef __STM324x9I_EVAL_WIFI_H
#define __STM324x9I_EVAL_WIFI_H

#ifdef __cplusplus
 extern "C" {
#endif   
   
/* Includes ------------------------------------------------------------------*/
#include "stm324x9i_eval.h"
#include "..\Components\sn8200\sn8200.h"  
   
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM324x9I_EVAL
  * @{
  */
    
/** @defgroup STM324x9I_EVAL_WiFi
  * @{
  */    

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  WiFi_OK       = 0,
  WiFi_ERROR    = 1,
  WiFi_TIMEOUT  = 2,
}WiFi_StatusTypeDef;

/** @defgroup STM324x9I_EVAL_WiFi_Exported_Types
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WiFi_Exported_Constants
  * @{
  */

/* Exported macro ------------------------------------------------------------*/
  
/** @defgroup STM324x9I_EVAL_WiFi_Exported_Macro
  * @{
  */ 

/* Exported functions --------------------------------------------------------*/

/** @defgroup STM324x9I_EVAL_WiFi_Exported_Functions
  * @{
  */

void BSP_WiFi_Init(void);
void BSP_WiFi_DeInit(void);
void BSP_WiFi_Start(void);
uint16_t BSP_WiFi_RX(uint8_t *data);
void BSP_WiFi_On(void);
void BSP_WiFi_Scan(void);
void BSP_WiFi_Join(void);
void BSP_SNIC_GetIP(void);
void BSP_SNIC_TCPClient(uint16_t bufsize, uint8_t timeout);
void BSP_SNIC_TCPServer(uint8_t maxClient);
void BSP_SNIC_UDPServer(void);
void BSP_SNIC_TCPSend(uint8_t sock, uint8_t * buf, uint16_t len, uint16_t index);
void BSP_SNIC_UDPSend(uint32_t ip, uint16_t port, uint8_t *buf, uint16_t len, uint16_t index, uint8_t mode);
void BSP_WiFi_GetStatus(void);
void BSP_WiFi_Leave(void);
void BSP_WiFi_Off(void);

#ifdef __cplusplus
}
#endif
#endif /* __STM324x9I_EVAL_WiFi_H */

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
