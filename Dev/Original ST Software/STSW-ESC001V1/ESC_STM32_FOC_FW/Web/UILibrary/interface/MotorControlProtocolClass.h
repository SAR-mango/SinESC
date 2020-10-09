/**
  ******************************************************************************
  * @file    MotorControlProtocolClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MotorControlProtocol class      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCP_MCUIC_H
#define __MCP_MCUIC_H

#include "MC_type.h"

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */

/** @addtogroup MotorControlProtocol
  * @{
  */
  
/** @defgroup MotorControlProtocol_class_exported_types Motor Control Protocol class exported types
* @{
*/

///*Class name*/
typedef struct CMCP_UI_t *CMCP_UI;

#include "FrameCommunicationProtocolClass.h"

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  MotorControlProtocol class parameters definition  
  */
typedef const struct
{
  // Parameters 
  pFCPParams_t pFCPParams; // FCP
} MCPParams_t, *pMCPParams_t;

/**
* @}
*/

/** @defgroup MotorControlProtocol_class_exported_methods Motor Control Protocol class exported methods
  * @{
  */

/*Methods*/
CMCP_UI MCP_NewObject(pUserInterfaceParams_t pUserInterfaceParams,
                      pMCPParams_t pMCPParams);
void MCP_Init(CMCP_UI this, CFCP oFCP, CUI oDAC, const char* s_fwVer);
void MCP_OnTimeOut(CMCP_UI this);
void MCP_ReceivedFrame(CMCP_UI this, uint8_t Code, uint8_t *buffer, uint8_t Size);
void MCP_SentFrame(CMCP_UI this, uint8_t Code, uint8_t *buffer, uint8_t Size);
void MCP_WaitNextFrame(CMCP_UI this);
void MCP_SendOverrunMeassage(CMCP_UI this);
void MCP_SendTimeoutMeassage(CMCP_UI this);
void MCP_SendATRMeassage(CMCP_UI this);
void MCP_SendBadCRCMessage(CMCP_UI this);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __MCP_MCUIC_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
