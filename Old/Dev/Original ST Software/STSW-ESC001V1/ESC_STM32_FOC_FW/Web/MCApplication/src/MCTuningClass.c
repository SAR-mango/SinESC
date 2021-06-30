/**
  ******************************************************************************
  * @file    MCTuningClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MCTuning class
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
#include "MCTuningClass.h"
#include "MCTuningClassPrivate.h"
#include "MCTuningPrivate.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #include "MC_type.h"
  #define MAX_MCT_NUM 2u

  _CMCT_t MCTpool[MAX_MCT_NUM];
  unsigned char MCT_Allocated = 0u;
#endif

/**
  * @brief  Creates an object of the class MCTuning
  * @param  pMCTuningParams pointer to an MCTuning parameters structure
  * @retval CMCT new instance of MCTuning object
  */
CMCT MCT_NewObject(pMCTuningParams_t pMCTuningParams)
{
  _CMCT _oMCT;
  
  #ifdef MC_CLASS_DYNAMIC
    _oMCT = (_CMCT)calloc(1u,sizeof(_CMCT_t));
  #else
    if (MCT_Allocated  < MAX_MCT_NUM)
    {
      _oMCT = &MCTpool[MCT_Allocated++];
    }
    else
    {
      _oMCT = MC_NULL;
    }
  #endif
  
  _oMCT->pParams_str = (pParams_t)pMCTuningParams;
  
  return ((CMCT)_oMCT);
}

/**
  * @brief  Example of public method of the class MCTuning
  * @param  this related object of class CMCT
  * @retval none
  */
void MCT_Init(CMCT this, MCTuningInitStruct_t MCTuningInitStruct)
{
  ((_CMCT)this)->Vars_str.MCTuningInitStruct = MCTuningInitStruct;
}

/**
  * @brief  It returns the FluxWeakeningCtrl object
  * @param  this related object of class CMCT
  * @retval CFW, instance of FluxWeakeningCtrl Class
  */
CFW MCT_GetFluxWeakeningCtrl(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oFW);
}

/**
  * @brief  It returns the FeedForwardCtrl object
  * @param  this related object of class CMCT
  * @retval CFF, instance of FeedForwardCtrl Class
  */
CFF MCT_GetFeedForwardCtrl(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oFF);
}

/**
  * @brief  It returns the HFICtrl object
  * \n\link HFI_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CHFI_FP, instance of HFICtrl Class
  */
CHFI_FP MCT_GetHFICtrl(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oHFI);
}

/**
  * @brief  It returns the speed control loop PI(D) object
  * @param  this related object of class CMCT
  * @retval none
  */
CPI MCT_GetSpeedLoopPID(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oPIDSpeed);
}

/**
  * @brief  It returns the Iq current control loop PI(D) object
  * @param  this related object of class CMCT
  * @retval none
  */
CPI MCT_GetIqLoopPID(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oPIDIq);
}

/**
  * @brief  It returns the Id current control loop PI(D) object
  * @param  this related object of class CMCT
  * @retval none
  */
CPI MCT_GetIdLoopPID(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oPIDId);
}

/**
  * @brief  It returns the Flux Weakening control loop PI(D) object
  * @param  this related object of class CMCT
  * @retval none
  */
CPI MCT_GetFluxWeakeningLoopPID(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oPIDFluxWeakening);
}

/**
  * @brief  It returns the PWMnCurrFdbk object
  * @param  this related object of class CMCT
  * @retval CPWMC, instance of PWMnCurrFdbkClass
  */
CPWMC MCT_GetPWMnCurrFdbk(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oPWMnCurrFdbk);
}

/**
  * @brief  It returns the RevupCtrl object
  * @param  this related object of class CMCT
  * @retval CRUC, instance of RevupCtrlClass
  */
CRUC MCT_GetRevupCtrl(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oRevupCtrl);
}

/**
  * @brief  It returns the Main Speed'n Position sensor object.
  * Main position sensor is considered the one used to execute FOC
  * @param  this related object of class CMCT
  * @retval CSPD, instance of SpeednPosFdbkClass
  */
CSPD MCT_GetSpeednPosSensorMain(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oSpeedSensorMain);
}

/**
  * @brief  It returns the Auxiliary Speed'n Position sensor object.
  *         Auxiliary position sensor is considered the one used to backup/tune
  *         the main one
  * @param  this related object of class CMCT
  * @retval CSPD, instance of SpeednPosFdbkClass; MC_NULL if no auxiliary sensor
  *         is active
  */
CSPD MCT_GetSpeednPosSensorAuxiliary(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oSpeedSensorAux);
}

/**
  * @brief  It returns the Virtual Speed'n Position sensor object.
  *         Virtual position sensor is considered the one used to rev-up the
  *         motor during the start-up procedure required by the state-observer
  *         sensorless algorithm
  * @param  this related object of class CMCT
  * @retval CSPD, instance of SpeednPosFdbkClass; MC_NULL if no auxiliary sensor
  *         is active
  */
CSPD MCT_GetSpeednPosSensorVirtual(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oSpeedSensorVirtual);
}

/**
  * @brief  It returns the Speed'n Torque Controller object.
  * @param  this related object of class CMCT
  * @retval CSTC, instance of SpeednTorqCtrlClass
  */
CSTC MCT_GetSpeednTorqueController(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oSpeednTorqueCtrl);
}

/**
  * @brief  It returns the State Machine object.
  * @param  this related object of class CMCT
  * @retval CSTM, instance of StateMachineClass
  */
CSTM MCT_GetStateMachine(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oStateMachine);
}

/**
  * @brief  It returns the Temperature sensor object.
  * @param  this related object of class CMCT
  * @retval CTSNS, instance of TemperatureSensorClass
  */
CTSNS MCT_GetTemperatureSensor(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oTemperatureSensor);
}

/**
  * @brief  It returns the Bus Voltage sensor object
  * @param  this related object of class CMCT
  * @retval CVBS, instance of BusVoltageSensorClass
  */
CVBS MCT_GetBusVoltageSensor(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oBusVoltageSensor);
}

/**
  * @brief  It returns the Brake resistor object
  * @param  this related object of class CMCT
  * @retval CDOUT, instance of DigitalOutput Class; MC_NULL if no Brake resistor
  *         is active
  */
CDOUT MCT_GetBrakeResistor(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oBrakeDigitalOutput);
}

/**
  * @brief  It returns the NTC Relay object
  * @param  this related object of class CMCT
  * @retval CDOUT, instance of DigitalOutput Class; MC_NULL if no Relay
  *         is active
  */
CDOUT MCT_GetNTCRelay(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oNTCRelay);
}

/**
  * @brief  It returns the MotorPowerMeasurement object
  * @param  this related object of class CMCT
  * @retval CMPM, instance of MotorPowerMeasurement Class; MC_NULL if no MPM
  *         is active
  */
CMPM MCT_GetMotorPowerMeasurement(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oMPM);
}

/**
  * @brief  It returns the Selfcommissioning object
  * \n\link Selfcommisioning_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval CSCC, instance of Selfcommissioning Class; MC_NULL if no SCC
  *         is active
  */
CSCC MCT_GetSelfCommissioning(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oSCC);
}

/**
  * @brief  It returns the One touch tuning object
  * \n\link OneTouchTuning_class_exported_methodsT Methods listed here can be applied \endlink
  * @param  this related object of class CMCT
  * @retval COTT, instance of One touch tuning Class; MC_NULL if no OTT
  *         is active
  */
COTT MCT_GetOneTouchTuning(CMCT this)
{
  return (((_CMCT)this)->Vars_str.MCTuningInitStruct.oOTT);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
