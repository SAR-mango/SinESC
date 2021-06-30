/**
  ******************************************************************************
  * @file    FrameCommunicationProtocolPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private definition for FrameCommunicationProtocol
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
#ifndef __FCP_H
#define __FCP_H

/* Includes ------------------------------------------------------------------*/
#include "FrameCommunicationProtocolClass.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum FrameTransmitionState_e
{
  FCP_IDLE,
  TRANSFERING_HEADER,
  TRANSFERING_BUFFER,
  TRANSFERING_CRC,
  TRANSFER_COMPLETE,
} FrameTransmitionState_t;

typedef struct Framedata_s
{ 
    uint8_t Code;
    uint8_t Size;  
    uint8_t Buffer[FRAME_MAX_BUFFER_LENGTH];
    uint8_t nCRC;
} FrameData_t, *PFrameData_t;

typedef struct FrameTransmition_s
{
    FrameData_t Frame;
    FrameTransmitionState_t State;
    PFN_TIMEOUT OnTimeOutByte;
    uint16_t FrameTransferError;
} FrameTransmition_t;

typedef struct
{
  // Class variables
  CMCP_UI parent;
  CCOM oCOM; // COM
  FrameTransmition_t frameTransmitionTX;
  FrameTransmition_t frameTransmitionRX;
} Vars_t,*pVars_t;

/* Params re-definition ------------------------------------------------------*/
typedef FCPParams_t Params_t, *pParams_t; 

/* Virtual Methods -----------------------------------------------------------*/
typedef struct
{
  // Class virtual methods TBD
  void (*pVReset)(CFCP this);
}Methods_t,*pMethods_t;

typedef struct
{
  Methods_t Methods_str; /*execution of virtual methods*/
  void *DerivedClass;
  Vars_t Vars_str;
  pParams_t pParams_str;
}_CFCP_t, *_CFCP;

#endif /* __FCP_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
