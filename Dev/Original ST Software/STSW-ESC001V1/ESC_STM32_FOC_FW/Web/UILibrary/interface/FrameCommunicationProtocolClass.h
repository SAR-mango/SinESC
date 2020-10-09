/**
  ******************************************************************************
  * @file    FrameCommunicationProtocolClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of FrameCommunicationProtocol class      
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
#ifndef __FCPC_H
#define __FCPC_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */

/** @addtogroup FrameCommunicationProtocol
  * @{
  */
  
/** @defgroup FrameCommunicationProtocol_class_exported_types Frame Communication Protocol class exported types
* @{
*/

/*Class name*/
typedef struct CFCP_t *CFCP;

#include "MC_type.h"

#include "PhysicalLayerCommunication_Class.h"

/* Exported constants --------------------------------------------------------*/
#define FRAME_ERROR_NONE              0x00
#define FRAME_ERROR_TRANSFER_ONGOING  0x01
#define FRAME_ERROR_WAITING_TRANSFER  0x02 
#define FRAME_ERROR_INVALID_PARAMETER 0x03 
#define FRAME_ERROR_TIME_OUT          0x04
#define FRAME_ERROR_INVALID_FRAME     0x05


#define FRAME_MAX_BUFFER_LENGTH  0x80
#define FRAME_HEADER_SIZE        0x02
#define FRAME_CRC_SIZE           0x01
#define FRAME_MAX_SIZE           (FRAME_HEADER_SIZE + FRAME_MAX_BUFFER_LENGTH + FRAME_CRC_SIZE)

#define FRAME_ACK_SIZE           0x01
#define FRAME_ACK_CODE           0xFF

/* Exported types ------------------------------------------------------------*/
typedef void (*PFN_TIMEOUT)();

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  FrameCommunicationProtocol class parameters definition  
  */
typedef const struct
{
  // Parameters
  uint32_t TimeOutRXByte;
  uint32_t TimeOutTXByte;
  PFN_TIMEOUT RXTimeOutEvent;
  PFN_TIMEOUT TXTimeOutEvent;
} FCPParams_t, *pFCPParams_t;

#include "UserInterfaceClass.h"
#include "MotorControlProtocolClass.h"

/**
* @}
*/

/** @defgroup FrameCommunicationProtocol_class_exported_methods Frame Communication Protocol class exported methods
  * @{
  */

/*Methods*/
CFCP FCP_NewObject(pFCPParams_t pFCPParam);
void FCP_Init(CFCP this, CCOM oCOM);
void FCP_SetParent(CFCP this, CMCP_UI oMCP);
void Frame_ResetTX(CFCP this);                 
void Frame_ResetRX(CFCP this);                 
uint8_t Frame_Receive(CFCP this, uint8_t *code, uint8_t *buffer, uint8_t *size);
uint8_t Frame_Send(CFCP this, uint8_t code, uint8_t *buffer, uint8_t size);
void SendingFrame(CFCP this, uint8_t *buffer, uint16_t bytes);
void* ReceivingFrame(CFCP this, uint8_t *buffer, uint16_t size);
void FCPTimeOut(CFCP this);
void FCP_SendOverrunMeassage(CFCP this);
void FCP_SendTimeoutMeassage(CFCP this);
void FCP_SendATRMeassage(CFCP this);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __FCPC_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
