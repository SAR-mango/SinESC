/**
  ******************************************************************************
  * @file    FrameCommunicationProtocolClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private implementation for FrameCommunicationProtocol class
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
#include "UserInterfaceClass.h"
#include "MotorControlProtocolClass.h"

#include "FrameCommunicationProtocolClass.h"
#include "FrameCommunicationProtocolPrivate.h"

/* Private function prototypes -----------------------------------------------*/
uint8_t CalcCRC(PFrameData_t pFrame);
uint8_t IsFrameValid(PFrameData_t pFrame);
void FrameTimeOutConfiguration(_CFCP _oFCP);

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  
  #define MAX_FCP_NUM 1

  _CFCP_t FCPpool[MAX_FCP_NUM];
  uint8_t FCP_Allocated = 0;
#endif

/**
  * @brief  Creates an object of the class FrameCommunicationProtocol
  * @param  pSensorParam: Physical Layer parameters
  * @retval oFCP: new Physical Layer object
  */
CFCP FCP_NewObject(pFCPParams_t pFCPParam)
{
  _CFCP _oFCP;
  
  #ifdef MC_CLASS_DYNAMIC
    _oFCP = (_CFCP)calloc(1,sizeof(_CFCP_t));
  #else
    if (FCP_Allocated  < MAX_FCP_NUM)
    {
      _oFCP = &FCPpool[FCP_Allocated++];
    }
    else
    {
      _oFCP = MC_NULL;
    }
  #endif

  FrameTimeOutConfiguration(_oFCP);
  _oFCP->Vars_str.frameTransmitionRX.OnTimeOutByte = pFCPParam->RXTimeOutEvent;
  _oFCP->Vars_str.frameTransmitionTX.OnTimeOutByte = pFCPParam->TXTimeOutEvent;
    
  return ((CFCP)_oFCP);
}

void FCP_Init(CFCP this, CCOM oCOM)
{
  _CFCP _oFCP = ((_CFCP)this);
  _oFCP->Vars_str.oCOM = oCOM;
  COM_SetParent(oCOM, (CFCP)_oFCP);
  
  Frame_ResetTX((CFCP)_oFCP);
  Frame_ResetRX((CFCP)_oFCP);
}

void FCP_SetParent(CFCP this, CMCP_UI oMCP)
{
  ((_CFCP)this)->Vars_str.parent = oMCP;
}

void Frame_ResetTX(CFCP this)
{
  ((_CFCP)this)->Vars_str.frameTransmitionTX.State = FCP_IDLE;
  COM_ResetTX((CCOM)(((_CFCP)this)->Vars_str.oCOM));
}

void Frame_ResetRX(CFCP this)
{
  COM_ResetRX((CCOM)(((_CFCP)this)->Vars_str.oCOM));
  ((_CFCP)this)->Vars_str.frameTransmitionRX.State = FCP_IDLE;
}

uint8_t Frame_Receive(CFCP this, uint8_t *code, uint8_t *buffer, uint8_t *size)
{
  uint8_t nRet;
  uint8_t i;

  ((_CFCP)this)->Vars_str.frameTransmitionRX.FrameTransferError = FRAME_ERROR_NONE;

  nRet = FRAME_ERROR_TRANSFER_ONGOING;
  if (((_CFCP)this)->Vars_str.frameTransmitionRX.State == FCP_IDLE)
  {
    ((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Code = 0x00;
    ((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Size = 0x00;

    for (i=0; i < FRAME_MAX_BUFFER_LENGTH; i++)
      ((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Buffer[i] = 0x00;

    ((_CFCP)this)->Vars_str.frameTransmitionRX.State = TRANSFERING_HEADER;

    COM_ReceiveBuffer((CCOM)(((_CFCP)this)->Vars_str.oCOM),(uint8_t*)&(((_CFCP)this)->Vars_str.frameTransmitionRX).Frame, FRAME_MAX_SIZE); // ReceivingFrame
    
    nRet = FRAME_ERROR_WAITING_TRANSFER;    
  }
  return nRet;
}

uint8_t Frame_Send(CFCP this, uint8_t code, uint8_t *buffer, uint8_t size)
{
  uint8_t nRet, idx;

  nRet = FRAME_ERROR_TRANSFER_ONGOING;
  if (((_CFCP)this)->Vars_str.frameTransmitionTX.State == FCP_IDLE)
  {
    ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Code = code;
    ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Size = size;
    
    for (idx=0; idx < size; idx++)
      ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Buffer[idx] = buffer[idx];
    
    ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Buffer[size] = CalcCRC((PFrameData_t)&(((_CFCP)this)->Vars_str.frameTransmitionTX.Frame));

    ((_CFCP)this)->Vars_str.frameTransmitionTX.State = TRANSFERING_BUFFER;

    COM_SendBuffer((CCOM)(((_CFCP)this)->Vars_str.oCOM),(uint8_t *)&(((_CFCP)this)->Vars_str.frameTransmitionTX.Frame), ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Size + FRAME_HEADER_SIZE + FRAME_CRC_SIZE); // SendingFrame
    
    nRet = FRAME_ERROR_WAITING_TRANSFER;
  }
  
  return nRet;
}

/*******************************************************************************
* Function Name  : FrameTimeOutConfiguration
* Description    : This function Configuration for timer7 to manage the timeout
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FrameTimeOutConfiguration(_CFCP _oFCP)
{  
  _oFCP->Vars_str.frameTransmitionRX.FrameTransferError = FRAME_ERROR_NONE;
  _oFCP->Vars_str.frameTransmitionTX.FrameTransferError = FRAME_ERROR_NONE;
}

// Private functions

uint8_t CalcCRC(PFrameData_t pFrame)
{
  uint8_t nCRC = 0;
  uint16_t nSum = 0;
  uint8_t idx;

  if(MC_NULL != pFrame)
  { 
    nSum += pFrame->Code;
    nSum += pFrame->Size;
    for(idx = 0; idx < (pFrame->Size); idx++)
    {
      nSum += pFrame->Buffer[idx];
    }
    nCRC = (uint8_t)(nSum & 0xFF) ; // Low Byte of nSum
    nCRC += (uint8_t) (nSum >> 8) ; // High Byte of nSum
  }

  return nCRC ;
}

const uint16_t Usart_Timeout_none = 0;
const uint16_t Usart_Timeout_start = 1;
const uint16_t Usart_Timeout_stop = 2;

void* ReceivingFrame(CFCP this, uint8_t *buffer, uint16_t size)
{
  void* pRetVal = (void *)(&Usart_Timeout_none);
  if (size == 1)  //First Byte received
  {
    pRetVal = (void *)(&Usart_Timeout_start);
  }
  switch(((_CFCP)this)->Vars_str.frameTransmitionRX.State)
  {
    case TRANSFERING_HEADER:
    {
      if (size == FRAME_HEADER_SIZE)
      {
        //Receive DATA BUFFER
        if (((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Size > 0 )
        {
          ((_CFCP)this)->Vars_str.frameTransmitionRX.State = TRANSFERING_BUFFER;
        }
        else
        {
          ((_CFCP)this)->Vars_str.frameTransmitionRX.State = TRANSFERING_CRC;
        }
      }
      break;
    }
    case TRANSFERING_BUFFER:
    {
      //Receive CRC
      if ((((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Size + FRAME_HEADER_SIZE) == size)
        ((_CFCP)this)->Vars_str.frameTransmitionRX.State = TRANSFERING_CRC;

      break;
    }
    case TRANSFERING_CRC:
    {
        if (size == (((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Size + FRAME_HEADER_SIZE + FRAME_CRC_SIZE))
        {
          pRetVal = (void *)(&Usart_Timeout_stop);
          ((_CFCP)this)->Vars_str.frameTransmitionRX.State = FCP_IDLE;
          USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
          
         
          ((_CFCP)this)->Vars_str.frameTransmitionRX.FrameTransferError = FRAME_ERROR_INVALID_FRAME;
          if (IsFrameValid((PFrameData_t)&(((_CFCP)this)->Vars_str.frameTransmitionRX.Frame)))
          {
            ((_CFCP)this)->Vars_str.frameTransmitionRX.FrameTransferError = FRAME_ERROR_NONE;

            MCP_ReceivedFrame(((_CFCP)this)->Vars_str.parent,(uint8_t)((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Code, (uint8_t *)((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Buffer, (uint8_t)((_CFCP)this)->Vars_str.frameTransmitionRX.Frame.Size);
          }
          else
          {
            MCP_SendBadCRCMessage(((_CFCP)this)->Vars_str.parent);
          }
        }
        break;
    }
  }
  return pRetVal;
}

void SendingFrame(CFCP this, uint8_t *buffer, uint16_t size)
{
    if (size == ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Size + FRAME_HEADER_SIZE + FRAME_CRC_SIZE)
      ((_CFCP)this)->Vars_str.frameTransmitionTX.State = FCP_IDLE;

    MCP_SentFrame(((_CFCP)this)->Vars_str.parent, ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Code, 
                                         (uint8_t *)((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Buffer, 
                                         ((_CFCP)this)->Vars_str.frameTransmitionTX.Frame.Size);
        
    //    Frame_ResetTX();
}

uint8_t IsFrameValid(PFrameData_t pFrame)
{
  if (pFrame)
    return CalcCRC(pFrame) == pFrame->Buffer[pFrame->Size];
  else
    return 0;
}

void FCPTimeOut(CFCP this)
{
  _CFCP _oFCP = ((_CFCP)this);
  _oFCP->Vars_str.frameTransmitionRX.FrameTransferError = FRAME_ERROR_TIME_OUT;
  Frame_ResetRX((CFCP)_oFCP);
}

void FCP_SendOverrunMeassage(CFCP this)
{
  MCP_SendOverrunMeassage(((_CFCP)this)->Vars_str.parent);
}

void FCP_SendTimeoutMeassage(CFCP this)
{
  MCP_SendTimeoutMeassage(((_CFCP)this)->Vars_str.parent);
}

void FCP_SendATRMeassage(CFCP this)
{
  MCP_SendATRMeassage(((_CFCP)this)->Vars_str.parent);
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
