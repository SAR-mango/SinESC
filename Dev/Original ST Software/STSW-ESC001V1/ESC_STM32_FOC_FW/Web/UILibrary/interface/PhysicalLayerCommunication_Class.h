/**
  ******************************************************************************
  * @file    PhysicalLayerCommunication_Class.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of Physical Layer Communication class      
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
#ifndef __PLC_H
#define __PLC_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */

/** @addtogroup PhysicalLayerCommunication
  * @{
  */
  
/** @defgroup PhysicalLayerCommunication_class_exported_types Physical Layer Communication class exported types
* @{
*/

/*Class name*/
typedef struct CCOM_t *CCOM;

#include "MC_type.h"
#include "FrameCommunicationProtocolClass.h"

/* Exported types ------------------------------------------------------------*/

/**
* @}
*/

/** @defgroup PhysicalLayerCommunication_class_exported_methods Physical Layer Communication class exported methods
  * @{
  */

/*Methods*/
CCOM COM_NewObject(void);
void COM_SetParent(CCOM this, CFCP oFCP); // FCP
void COM_ResetTX(CCOM this);
void COM_ResetRX(CCOM this);
void COM_SendBuffer(CCOM this,uint8_t *pBuffer, uint16_t sizeToSend);
void COM_ReceiveBuffer(CCOM this,uint8_t *pBuffer, uint16_t sizeToReceive);
void COM_StartReceive(CCOM this); /* executed by derived classes */
void COM_StartTransmit(CCOM this); /* executed by derived classes */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __PLC_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
