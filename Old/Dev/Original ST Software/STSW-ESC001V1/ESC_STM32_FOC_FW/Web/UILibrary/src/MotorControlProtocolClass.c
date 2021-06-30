/**
  ******************************************************************************
  * @file    MotorControlProtocolClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   Private implementation for MotorControlProtocol class
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
#include "UserInterfacePrivate.h"
#include "MotorControlProtocolClass.h"
#include "MotorControlProtocolPrivate.h"

/* Private define ------------------------------------------------------------*/
#define RET_BUFFER 0x02

#define ACK_NOERROR 0xF0
#define ACK_ERROR   0xFF
#define ATR_FRAME_START 0xE0

#define MC_PROTOCOL_CODE_NONE        0x00

typedef enum ERROR_CODE_e
{
	ERROR_NONE = 0,
	ERROR_BAD_FRAME_ID,         /* 0x01 */
	ERROR_CODE_SET_READ_ONLY,   /* 0x02 */ 
	ERROR_CODE_GET_WRITE_ONLY,  /* 0x03 */
	ERROR_CODE_NO_TARGET_DRIVE, /* 0x04 */
	ERROR_CODE_WRONG_SET,       /* 0x05 */
	ERROR_CODE_CMD_ID,          /* 0x06 */
	ERROR_CODE_WRONG_CMD,       /* 0x07 */
	ERROR_CODE_OVERRUN,         /* 0x08 */
	ERROR_CODE_TIMEOUT,         /* 0x09 */
	ERROR_CODE_BAD_CRC,         /* 0x0A */
	ERROR_BAD_MOTOR_SELECTED,   /* 0x0B */
	ERROR_MP_NOT_ENABLED        /* 0x0C */
} ERROR_CODE;

MPInfo_t MPInfo = {0, 0};

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  
  #define MAX_MCP_MCUI_NUM 1

  _DCMCP_t MCP_MCUIpool[MAX_MCP_MCUI_NUM];
  uint8_t MCP_MCUI_Allocated = 0;
#endif

/**
  * @brief  Creates an object of the class MotorControlProtocol
  * @param  pMCPParam: MotorControlProtocol parameters
  * @retval oMCP_MCUI: new MotorControlProtocol object
  */
CMCP_UI MCP_NewObject(pUserInterfaceParams_t pUserInterfaceParams, 
                      pMCPParams_t pMCPParams)
{
  _CUI _oUI;
  _DCMCP _oMCP;
  
  _oUI = (_CUI)UI_NewObject(pUserInterfaceParams);

  #ifdef MC_CLASS_DYNAMIC
    _oMCP = (_DCMCP)calloc(1,sizeof(_DCMCP_t));
  #else
    if (MCP_MCUI_Allocated  < MAX_MCP_MCUI_NUM)
    {
      _oMCP = &MCP_MCUIpool[MCP_MCUI_Allocated++];
    }
    else
    {
      _oMCP = MC_NULL;
    }
  #endif
  
  _oMCP->pDParams_str = pMCPParams;
  _oUI->DerivedClass = (void*)_oMCP;
  
  return ((CMCP_UI)_oUI);
}

void MCP_Init(CMCP_UI this, CFCP oFCP, CUI oDAC, const char* s_fwVer)
{
  _DCMCP _oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  _oMCP->DVars_str.oFCP = oFCP;
  _oMCP->DVars_str.oDAC = oDAC;
  _oMCP->DVars_str.s_fwVer = s_fwVer;
  FCP_SetParent(oFCP, (CMCP_UI) this);

  MCP_WaitNextFrame(this);
}

void MCP_OnTimeOut(CMCP_UI this)
{
     MCP_WaitNextFrame(this);
}

void MCP_WaitNextFrame(CMCP_UI this)
{
  _DCMCP oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  FCPTimeOut(oMCP->DVars_str.oFCP);
  oMCP->DVars_str.BufferSize = FRAME_MAX_BUFFER_LENGTH;
  Frame_Receive(oMCP->DVars_str.oFCP,MC_NULL, oMCP->DVars_str.BufferFrame, &(oMCP->DVars_str.BufferSize));
}

void MCP_SentFrame(CMCP_UI this, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
    MCP_WaitNextFrame(this);
}

void MCP_ReceivedFrame(CMCP_UI this, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
  _DCMCP oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  bool RequireAck = TRUE;
  bool bNoError = FALSE; // Default is error
  uint8_t bErrorCode;
  
  /* Protocol version >3.3 motor selection inside Frame ID */
  uint8_t bMotorSelection = (Code & 0xE0) >> 5; /* Mask: 1110|0000 */
  if (bMotorSelection != 0)
  {
    if (UI_SetReg((CUI)this, MC_PROTOCOL_REG_TARGET_MOTOR, bMotorSelection - 1))
    {
      Code &= 0x1F; /* Mask: 0001|1111 */
      
      /* Change also the DAC selected motor */
      if (oMCP->DVars_str.oDAC)
      {
        UI_SetReg((CUI)(oMCP->DVars_str.oDAC), MC_PROTOCOL_REG_TARGET_MOTOR, bMotorSelection - 1);
      }
    }
    else
    {
      Code = MC_PROTOCOL_CODE_NONE; /* Error */
      bErrorCode = ERROR_BAD_MOTOR_SELECTED;
    }
  }
  
  switch (Code)
  {
  case MC_PROTOCOL_CODE_SET_REG:
    {
      MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
      bErrorCode = ERROR_CODE_WRONG_SET;
      
      switch (bRegID)
      {
      case MC_PROTOCOL_REG_TARGET_MOTOR:
        {
          /* Deprecated */
          int32_t wValue = (int32_t)(buffer[1]);
          
          UI_SetReg((CUI)(oMCP->DVars_str.oDAC), bRegID, wValue);
          bNoError = UI_SetReg((CUI)this, bRegID, wValue);
        }
        break;
      case MC_PROTOCOL_REG_CONTROL_MODE:
      case MC_PROTOCOL_REG_SC_PP:
        {
          /* 8bit variables */
          bNoError = UI_SetReg((CUI)this, bRegID, (int32_t)(buffer[1]));
        }
        break;
        
      case MC_PROTOCOL_REG_DAC_OUT1:
        {
          UI_SetDAC(oMCP->DVars_str.oDAC, DAC_CH0, (MC_Protocol_REG_t)(buffer[1]));
          bNoError = TRUE; /* No check inside class return always true*/
        }
        break;
        
      case MC_PROTOCOL_REG_DAC_OUT2:
        {
          UI_SetDAC(oMCP->DVars_str.oDAC, DAC_CH1, (MC_Protocol_REG_t)(buffer[1]));
          bNoError = TRUE; /* No check inside class return always true*/
        }
        break;
        
      case MC_PROTOCOL_REG_TORQUE_REF:
      case MC_PROTOCOL_REG_FLUX_REF: 
      case MC_PROTOCOL_REG_SPEED_KP:
      case MC_PROTOCOL_REG_SPEED_KI:
      case MC_PROTOCOL_REG_SPEED_KD:
      case MC_PROTOCOL_REG_TORQUE_KP:
      case MC_PROTOCOL_REG_TORQUE_KI:
      case MC_PROTOCOL_REG_TORQUE_KD:
      case MC_PROTOCOL_REG_FLUX_KP:
      case MC_PROTOCOL_REG_FLUX_KI:
      case MC_PROTOCOL_REG_FLUX_KD:
      case MC_PROTOCOL_REG_PLL_KI:
      case MC_PROTOCOL_REG_PLL_KP:
      case MC_PROTOCOL_REG_FLUXWK_KP:
      case MC_PROTOCOL_REG_FLUXWK_KI:
      case MC_PROTOCOL_REG_FLUXWK_BUS:
      case MC_PROTOCOL_REG_IQ_SPEEDMODE:
      case MC_PROTOCOL_REG_PFC_DCBUS_REF:
      case MC_PROTOCOL_REG_PFC_I_KP:
      case MC_PROTOCOL_REG_PFC_I_KI:
      case MC_PROTOCOL_REG_PFC_I_KD:
      case MC_PROTOCOL_REG_PFC_V_KP:
      case MC_PROTOCOL_REG_PFC_V_KI:
      case MC_PROTOCOL_REG_PFC_V_KD:
      case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
      case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
        {
          /* 16bit variables */
          int32_t wValue = buffer[1] + (buffer[2] << 8);
          bNoError = UI_SetReg((CUI)this, bRegID, wValue);
        }
        break;
        
      case MC_PROTOCOL_REG_OBSERVER_C1:
      case MC_PROTOCOL_REG_OBSERVER_C2:
      case MC_PROTOCOL_REG_FF_1Q:
      case MC_PROTOCOL_REG_FF_1D:
      case MC_PROTOCOL_REG_FF_2:
      case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
      case MC_PROTOCOL_REG_SC_CURRENT:
      case MC_PROTOCOL_REG_SC_SPDBANDWIDTH:
      case MC_PROTOCOL_REG_SC_LDLQRATIO:
      case MC_PROTOCOL_REG_SC_NOMINAL_SPEED:
      case MC_PROTOCOL_REG_SC_CURRBANDWIDTH:
      case MC_PROTOCOL_REG_SC_STARTUP_SPEED:
      case MC_PROTOCOL_REG_SC_STARTUP_ACC:
        {
          /* 32bit variables */
          int32_t wValue = buffer[1] + (buffer[2] << 8) + (buffer[3] << 16) + (buffer[4] << 24);
          bNoError = UI_SetReg((CUI)this, bRegID, wValue);
        }
        break;
        
      default:
        {
          bErrorCode = ERROR_CODE_SET_READ_ONLY;
        }
        break;
      }      
    }
    break;
  case MC_PROTOCOL_CODE_GET_REG:
    {
      MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
      bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
      
      switch (bRegID)
      {
      case MC_PROTOCOL_REG_TARGET_MOTOR:
      case MC_PROTOCOL_REG_STATUS:
      case MC_PROTOCOL_REG_CONTROL_MODE:
      case MC_PROTOCOL_REG_RUC_STAGE_NBR:
      case MC_PROTOCOL_REG_PFC_STATUS:
      case MC_PROTOCOL_REG_PFC_ENABLED:
      case MC_PROTOCOL_REG_SC_CHECK:
      case MC_PROTOCOL_REG_SC_STATE:
      case MC_PROTOCOL_REG_SC_STEPS:
      case MC_PROTOCOL_REG_SC_PP:
      case MC_PROTOCOL_REG_SC_FOC_REP_RATE:
      case MC_PROTOCOL_REG_SC_COMPLETED:
        {
          /* 8bit variables */
          int32_t value = UI_GetReg((CUI)this, bRegID);
          if (value != (int32_t)(GUI_ERROR_CODE))
          {
            Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, (uint8_t*)(&value), 1);
            bNoError = TRUE;
            RequireAck = FALSE;
          }
        }
        break;
        
      case MC_PROTOCOL_REG_DAC_OUT1:
        {
          if (oMCP->DVars_str.oDAC)
          {
            MC_Protocol_REG_t value = UI_GetDAC(oMCP->DVars_str.oDAC, DAC_CH0);
            Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, (uint8_t*)(&value), 1);
            bNoError = TRUE;
            RequireAck = FALSE;
          }
        }
        break;
        
      case MC_PROTOCOL_REG_DAC_OUT2:
        {
          if (oMCP->DVars_str.oDAC)
          {
            MC_Protocol_REG_t value = UI_GetDAC(oMCP->DVars_str.oDAC, DAC_CH1);
            Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, (uint8_t*)(&value), 1);
            bNoError = TRUE;
          }
        }
        break;
        
      case MC_PROTOCOL_REG_SPEED_KP:
      case MC_PROTOCOL_REG_SPEED_KP_DIV:
      case MC_PROTOCOL_REG_SPEED_KI:
      case MC_PROTOCOL_REG_SPEED_KI_DIV:
      case MC_PROTOCOL_REG_SPEED_KD:
      case MC_PROTOCOL_REG_TORQUE_REF:
      case MC_PROTOCOL_REG_TORQUE_KP:
      case MC_PROTOCOL_REG_TORQUE_KI:
      case MC_PROTOCOL_REG_TORQUE_KD:
      case MC_PROTOCOL_REG_FLUX_REF:
      case MC_PROTOCOL_REG_FLUX_KP:
      case MC_PROTOCOL_REG_FLUX_KI:
      case MC_PROTOCOL_REG_FLUX_KD:
      case MC_PROTOCOL_REG_OBSERVER_C1:
      case MC_PROTOCOL_REG_OBSERVER_C2:
      case MC_PROTOCOL_REG_OBSERVER_CR_C1:
      case MC_PROTOCOL_REG_OBSERVER_CR_C2:
      case MC_PROTOCOL_REG_PLL_KP:
      case MC_PROTOCOL_REG_PLL_KI:
      case MC_PROTOCOL_REG_FLUXWK_KP:
      case MC_PROTOCOL_REG_FLUXWK_KI:
      case MC_PROTOCOL_REG_FLUXWK_BUS:
      case MC_PROTOCOL_REG_BUS_VOLTAGE:
      case MC_PROTOCOL_REG_HEATS_TEMP:
      case MC_PROTOCOL_REG_MOTOR_POWER:
      case MC_PROTOCOL_REG_TORQUE_MEAS:
      case MC_PROTOCOL_REG_FLUX_MEAS:
      case MC_PROTOCOL_REG_FLUXWK_BUS_MEAS:
      case MC_PROTOCOL_REG_IQ_SPEEDMODE:
      case MC_PROTOCOL_REG_FF_VQ:
      case MC_PROTOCOL_REG_FF_VD:
      case MC_PROTOCOL_REG_FF_VQ_PIOUT:
      case MC_PROTOCOL_REG_FF_VD_PIOUT:
      case MC_PROTOCOL_REG_PFC_DCBUS_REF:
      case MC_PROTOCOL_REG_PFC_DCBUS_MEAS:
      case MC_PROTOCOL_REG_PFC_ACBUS_FREQ:
      case MC_PROTOCOL_REG_PFC_ACBUS_RMS:
      case MC_PROTOCOL_REG_PFC_I_KP:
      case MC_PROTOCOL_REG_PFC_I_KI:
      case MC_PROTOCOL_REG_PFC_I_KD:
      case MC_PROTOCOL_REG_PFC_V_KP:
      case MC_PROTOCOL_REG_PFC_V_KI:
      case MC_PROTOCOL_REG_PFC_V_KD:
      case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
      case MC_PROTOCOL_REG_HFI_EL_ANGLE:
      case MC_PROTOCOL_REG_HFI_ROT_SPEED:
      case MC_PROTOCOL_REG_HFI_CURRENT:
      case MC_PROTOCOL_REG_HFI_INIT_ANG_PLL:
      case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
      case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
      case MC_PROTOCOL_REG_CTRBDID:
      case MC_PROTOCOL_REG_PWBDID:
        {
          int32_t value = UI_GetReg((CUI)this, bRegID);
          if (value != (int32_t)(GUI_ERROR_CODE))
          {
            /* 16bit variables */
            Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, (uint8_t*)(&value), 2);
            bNoError = TRUE;
            RequireAck = FALSE;
          }
        }
        break;
        
      case MC_PROTOCOL_REG_FLAGS:
      case MC_PROTOCOL_REG_SPEED_REF:
      case MC_PROTOCOL_REG_SPEED_MEAS:
      case MC_PROTOCOL_REG_FF_1Q:
      case MC_PROTOCOL_REG_FF_1D:
      case MC_PROTOCOL_REG_FF_2:  
      case MC_PROTOCOL_REG_PFC_FAULTS:
      case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
      case MC_PROTOCOL_REG_SC_RS:
      case MC_PROTOCOL_REG_SC_LS:
      case MC_PROTOCOL_REG_SC_KE:
      case MC_PROTOCOL_REG_SC_VBUS:
      case MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED:
      case MC_PROTOCOL_REG_SC_CURRENT:
      case MC_PROTOCOL_REG_SC_SPDBANDWIDTH:
      case MC_PROTOCOL_REG_SC_LDLQRATIO:
      case MC_PROTOCOL_REG_SC_NOMINAL_SPEED:
      case MC_PROTOCOL_REG_SC_CURRBANDWIDTH:
      case MC_PROTOCOL_REG_SC_J:
      case MC_PROTOCOL_REG_SC_F:
      case MC_PROTOCOL_REG_SC_MAX_CURRENT:
      case MC_PROTOCOL_REG_SC_STARTUP_SPEED:
      case MC_PROTOCOL_REG_SC_STARTUP_ACC:
      case MC_PROTOCOL_REG_SC_PWM_FREQUENCY:
      case MC_PROTOCOL_REG_UID:
        {
          int32_t value = UI_GetReg((CUI)this, bRegID);
          if (value != (int32_t)(GUI_ERROR_CODE))
          {
            /* 32bit variables */
            Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, (uint8_t*)(&value), 4);
            bNoError = TRUE;
            RequireAck = FALSE;
          }
        }
        break;
        
      default:
        bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
        break;
      }
    }
    break;
  case MC_PROTOCOL_CODE_EXECUTE_CMD:
    {
      uint8_t bCmdID = buffer[0];
      bErrorCode = ERROR_CODE_WRONG_CMD;
      bNoError = UI_ExecCmd((CUI)this,bCmdID);
    }
    break;
  case MC_PROTOCOL_CODE_GET_BOARD_INFO:
    {
      /* GetBoardInfo */
      unsigned char i;
      uint8_t outBuff[32];
      for (i = 0; i < 32; i++)
      {
        outBuff[i] = 0;
      }
      for (i = 0; (i<29) && (oMCP->DVars_str.s_fwVer[i]!=0); i++)
      {
        outBuff[3+i] = oMCP->DVars_str.s_fwVer[i];
      }
      outBuff[0] = oMCP->DVars_str.s_fwVer[i+5];
      outBuff[1] = oMCP->DVars_str.s_fwVer[i+7];
      outBuff[2] = oMCP->DVars_str.s_fwVer[i+9];
      Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, outBuff, 32);
      bNoError = TRUE;
    }
    break;
  case MC_PROTOCOL_CODE_SET_RAMP:
    {
      uint16_t duration = buffer[4] + (buffer[5] << 8);
      int32_t rpm = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
      bNoError = UI_ExecSpeedRamp((CUI)this, rpm,duration);
    }
    break;
  case MC_PROTOCOL_CODE_GET_REVUP_DATA:
    {
      uint8_t outBuff[8];
      uint16_t Durationms;
      int16_t FinalMecSpeed01Hz;
      int16_t FinalTorque;
      int32_t rpm;
      UI_GetRevupData((CUI)this, buffer[0], &Durationms, &FinalMecSpeed01Hz, &FinalTorque);
      rpm = FinalMecSpeed01Hz * 6;
      outBuff[0] = (uint8_t)(rpm);
      outBuff[1] = (uint8_t)(rpm >> 8);
      outBuff[2] = (uint8_t)(rpm >> 16);
      outBuff[3] = (uint8_t)(rpm >> 24);
      outBuff[4] = (uint8_t)(FinalTorque);
      outBuff[5] = (uint8_t)(FinalTorque >> 8);
      outBuff[6] = (uint8_t)(Durationms);
      outBuff[7] = (uint8_t)(Durationms >> 8);
      Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, outBuff, 8);
    }
    break;
  case MC_PROTOCOL_CODE_SET_REVUP_DATA:
    {
      uint8_t bStage;
      uint16_t hDurationms;
      int16_t hFinalMecSpeed01Hz;
      int16_t hFinalTorque;
      int32_t rpm;
      bStage = buffer[0];
      hDurationms = buffer[7] + (buffer[8] << 8);
      rpm = buffer[1] + (buffer[2] << 8) + (buffer[3] << 16) + (buffer[4] << 24);
      hFinalMecSpeed01Hz = rpm / 6;
      hFinalTorque = buffer[5] + (buffer[6] << 8);
      bNoError = UI_SetRevupData((CUI)this, bStage, hDurationms, hFinalMecSpeed01Hz, hFinalTorque); 
    }
    break;
  case MC_PROTOCOL_CODE_SET_CURRENT_REF:
    {
      int16_t hIqRef;
      int16_t hIdRef;
      hIqRef = buffer[0] + (buffer[1] << 8);
      hIdRef = buffer[2] + (buffer[3] << 8);
      UI_SetCurrentReferences((CUI)this, hIqRef, hIdRef);
      bNoError = TRUE;
    }
    break;
  case MC_PROTOCOL_CODE_GET_MP_INFO:
    {
      MPInfo_t stepList;
      stepList.data = buffer;
      stepList.len = Size;
      bErrorCode = ERROR_MP_NOT_ENABLED;
      bNoError = UI_GetMPInfo(&stepList, &MPInfo);
      
      if (bNoError)
      {
        Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, MPInfo.data, MPInfo.len);
        RequireAck = FALSE;
      }
    }
    break;
  case MC_PROTOCOL_CODE_NONE:
    {
    }
    break;
  default:
    {
      bErrorCode = ERROR_BAD_FRAME_ID;
    }
    break;
  }
  
  if (RequireAck)
  {
    if (bNoError)
    {
      Frame_Send(oMCP->DVars_str.oFCP,ACK_NOERROR, MC_NULL, 0);
    }
    else
    {
      Frame_Send(oMCP->DVars_str.oFCP,ACK_ERROR, &bErrorCode, 1);
    }
  }
}

void MCP_SendOverrunMeassage(CMCP_UI this)
{
  _DCMCP oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  uint8_t bErrorCode = ERROR_CODE_OVERRUN;
  Frame_Send(oMCP->DVars_str.oFCP,ACK_ERROR, &bErrorCode, 1);
}

void MCP_SendTimeoutMeassage(CMCP_UI this)
{
  _DCMCP oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  uint8_t bErrorCode = ERROR_CODE_TIMEOUT;
  Frame_Send(oMCP->DVars_str.oFCP,ACK_ERROR, &bErrorCode, 1);
}

void MCP_SendATRMeassage(CMCP_UI this)
{
  _DCMCP oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  uint32_t wUID = UI_GetReg((CUI)this, MC_PROTOCOL_REG_UID);
  unsigned char i;
  uint8_t bFWX;
  uint8_t bFWY;
  uint8_t bFWZ;
  uint16_t hCBDID = UI_GetReg((CUI)this, MC_PROTOCOL_REG_CTRBDID);
  uint16_t hPBDID = UI_GetReg((CUI)this, MC_PROTOCOL_REG_PWBDID);
  uint16_t hPBDID2 = UI_GetReg((CUI)this, MC_PROTOCOL_REG_PWBDID2);
  uint8_t buff[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
  
  for (i = 0; (i<29) && (oMCP->DVars_str.s_fwVer[i]!=0); i++);
  
  bFWX = oMCP->DVars_str.s_fwVer[i+5];
  bFWY = oMCP->DVars_str.s_fwVer[i+7];
  bFWZ = oMCP->DVars_str.s_fwVer[i+9];
  
  *(uint32_t*)(&buff[0]) = wUID;
  buff[4] = bFWX;
  buff[5] = bFWY;
  buff[6] = bFWZ;
  buff[7] = (uint8_t)(hCBDID);
  buff[8] = (uint8_t)(hCBDID>>8);
  buff[9] = (uint8_t)(hPBDID);
  buff[10]= (uint8_t)(hPBDID>>8);
  buff[11] = (uint8_t)(hPBDID2);
  buff[12]= (uint8_t)(hPBDID2>>8);

  Frame_Send(oMCP->DVars_str.oFCP,ATR_FRAME_START, buff, 13);
}

void MCP_SendBadCRCMessage(CMCP_UI this)
{
  _DCMCP oMCP = (_DCMCP)(((_CUI)this)->DerivedClass);
  uint8_t bErrorCode = ERROR_CODE_BAD_CRC;
  Frame_Send(oMCP->DVars_str.oFCP,ACK_ERROR, &bErrorCode, 1);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
