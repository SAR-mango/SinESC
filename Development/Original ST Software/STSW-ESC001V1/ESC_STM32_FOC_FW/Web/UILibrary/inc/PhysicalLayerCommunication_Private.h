/**
  ******************************************************************************
  * @file    PhysicalLayerCommunication_Private.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private definition for Physical Layer Communication
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
#ifndef __COM_H
#define __COM_H

#include "PhysicalLayerCommunication_Class.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct DataSend_s
{
  uint8_t *Buffer;
  uint16_t BufferCount;
  uint16_t BufferTransfer;
} DataSend_t;

typedef struct DataCommunication_s
{
  DataSend_t RX;
  DataSend_t TX;
} DataCommunication_t;

typedef struct
{
  DataCommunication_t PL_Data;
  CFCP parent;
}Vars_t,*pVars_t;

typedef struct
{
  void* (*pIRQ_Handler)(void *this, unsigned char flag, unsigned short rx_data);
  void (*pStartReceive)(CCOM this);
  void (*pStartTransmit)(CCOM this);
}Methods_t,*pMethods_t;

typedef struct
{
  Methods_t Methods_str; /*execution of virtual methods*/
  void *DerivedClass;
  Vars_t Vars_str;
}_CCOM_t, *_CCOM;

#endif /* __COM_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
