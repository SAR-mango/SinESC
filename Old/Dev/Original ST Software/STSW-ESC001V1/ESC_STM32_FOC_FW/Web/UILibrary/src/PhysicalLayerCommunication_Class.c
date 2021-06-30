/**
  ******************************************************************************
  * @file    PhysicalLayerCommunication_Class.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private implementation for Physical Layer Communication
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

/* Includes ------------------------------------------------------------------*/

#include "PhysicalLayerCommunication_Class.h"
#include "PhysicalLayerCommunication_Private.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  
  #define MAX_COM_NUM 1

  _CCOM_t COMpool[MAX_COM_NUM];
  uint8_t COM_Allocated = 0;
#endif

/**
  * @brief  Creates an object of the class "Physical Layer Communication"
  * @param  pSensorParam: Physical Layer parameters
  * @retval oCOM: new Physical Layer object
  */
CCOM COM_NewObject(void)
{
  _CCOM _oCOM;
  
  #ifdef MC_CLASS_DYNAMIC
    _oCOM = (_CCOM)calloc(1,sizeof(_CCOM_t));
  #else
    if (COM_Allocated  < MAX_COM_NUM)
    {
      _oCOM = &COMpool[COM_Allocated++];
    }
    else
    {
      _oCOM = MC_NULL;
    }
  #endif
  
  return ((CCOM)_oCOM);
}

// Sets the parent member
void COM_SetParent(CCOM this, CFCP oFCP)
{
  ((_CCOM)this)->Vars_str.parent = oFCP;
}

/*******************************************************************************
* Function Name  : COM_ResetTX
* Description    : Reset Transfer communication
* Input          : none 
* Return         : none
*******************************************************************************/
void COM_ResetTX(CCOM this)
{
  ((_CCOM)this)->Vars_str.PL_Data.TX.Buffer = MC_NULL;
  ((_CCOM)this)->Vars_str.PL_Data.TX.BufferCount = 0;
  ((_CCOM)this)->Vars_str.PL_Data.TX.BufferTransfer = 0;
}

/*******************************************************************************
* Function Name  : COM_ResetRX
* Description    : Reset Receive communication
* Input          : none 
* Return         : none
*******************************************************************************/
void COM_ResetRX(CCOM this)
{
  ((_CCOM)this)->Vars_str.PL_Data.RX.Buffer = MC_NULL;
  ((_CCOM)this)->Vars_str.PL_Data.RX.BufferCount = 0;
  ((_CCOM)this)->Vars_str.PL_Data.RX.BufferTransfer = 0;
}

/*******************************************************************************
* Function Name  : COM_SendBuffer
* Description    : Send a buffer in com
* Input          : pBuffer data to send, 
*                  sizeToSend sizeof of pBuffer 
*                  pfDataSend  Callback when transmition completed
* Return         : none
*******************************************************************************/
void COM_SendBuffer(CCOM this, uint8_t *pBuffer, uint16_t sizeToSend)
{
    if (pBuffer != 0 && sizeToSend != 0)
    {
      ((_CCOM)this)->Vars_str.PL_Data.TX.Buffer = pBuffer;
      ((_CCOM)this)->Vars_str.PL_Data.TX.BufferCount = sizeToSend;
      ((_CCOM)this)->Vars_str.PL_Data.TX.BufferTransfer = 0;
      
      COM_StartTransmit(this);
    }
}

/*******************************************************************************
* Function Name  : COM_ReceiveBuffer
* Description    : Receive a buffer from 
* Input          : pBuffer allowed buffer where receive data 
*                  sizeToReceive sizeof of data to receive in byte
*                  pfDataReceived  Callback when transmition completed
* Return         : none
*******************************************************************************/
void COM_ReceiveBuffer(CCOM this, uint8_t *pBuffer, uint16_t sizeToReceive)
{    
  if (pBuffer != 0 && sizeToReceive != 0)
  {
    ((_CCOM)this)->Vars_str.PL_Data.RX.Buffer = pBuffer;
    ((_CCOM)this)->Vars_str.PL_Data.RX.BufferCount = sizeToReceive;
    ((_CCOM)this)->Vars_str.PL_Data.RX.BufferTransfer = 0;
    
    COM_StartReceive(this);
  }  
}

/**
  * @brief  Start receive from the channel (IRQ enabling)
  * @param  this: COM object 
  * @retval None
  */
void COM_StartReceive(CCOM this)
{
  ((_CCOM)this)->Methods_str.pStartReceive(this);
}

/**
  * @brief  Start transmit to the channel (IRQ enabling)
  * @param  this: COM object 
  * @retval None
  */
void COM_StartTransmit(CCOM this)
{
  ((_CCOM)this)->Methods_str.pStartTransmit(this);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
