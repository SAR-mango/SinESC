/**
  ******************************************************************************
  * @file    SpeednTorqCtrlClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of SpeednTorqCtrl class
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
#include "SpeednTorqCtrlClass.h"
#include "SpeednTorqCtrlPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CSTC_t STCpool[MAX_STC_NUM];
  unsigned char STC_Allocated = 0u;
#endif
  
#define CLASS_VARS   &((_CSTC)this)->Vars_str
#define CLASS_PARAMS  ((_CSTC)this)->pParams_str
  
#define CHECK_BOUNDARY

/**
  * @brief  Creates an object of the class SpeednTorqCtrl
  * @param  pSpeednTorqCtrlParams pointer to an SpeednTorqCtrl parameters structure
  * @retval CSTC new instance of SpeednTorqCtrl object
  */
CSTC STC_NewObject(pSpeednTorqCtrlParams_t pSpeednTorqCtrlParams)
{
  _CSTC _oSTC;
  
  #ifdef MC_CLASS_DYNAMIC
    _oSTC = (_CSTC)calloc(1u,sizeof(_CSTC_t));
  #else
    if (STC_Allocated  < MAX_STC_NUM)
    {
      _oSTC = &STCpool[STC_Allocated++];
    }
    else
    {
      _oSTC = MC_NULL;
    }
  #endif
  
  _oSTC->pParams_str = (pParams_t)pSpeednTorqCtrlParams;
  
  return ((CSTC)_oSTC);
}

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class CSTC.
  * @param  oPI the PI object used as controller for the speed regulation.
  *         It can be equal to MC_NULL if the STC is initialized in torque mode
  *         and it will never be configured in speed mode.
  * @param  oSPD the speed sensor used to perform the speed regulation.
  *         It can be equal to MC_NULL if the STC is used only in torque 
  *         mode.
  * @retval none.
  */
void STC_Init(CSTC this, CPI oPI, CSPD oSPD)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  
  pVars->oPISpeed = oPI;
  pVars->oSPD = oSPD;
  pVars->bMode = pParams->bModeDefault;
  pVars->wSpeedRef01HzExt = (int32_t)pParams->hMecSpeedRef01HzDefault * 65536;
  pVars->wTorqueRef = (int32_t)pParams->hTorqueRefDefault * 65536;
  pVars->hTargetFinal = 0;
  pVars->wRampRemainingStep = 0u;
  pVars->wIncDecAmount = 0;
  pVars->hMaxPositiveTorque = pParams->hMaxPositiveTorque;
  pVars->hMinNegativeTorque = pParams->hMinNegativeTorque;
}

/**
  * @brief It sets in real time the speed sensor utilized by the STC. 
  * @param this related object of class CSTC
  * @param oSPD Speed sensor object to be set.
  * @retval none
  */
void STC_SetSpeedSensor(CSTC this, CSPD oSPD)
{
  pVars_t pVars = CLASS_VARS;  
  pVars->oSPD = oSPD;
}

/**
  * @brief It returns the speed sensor utilized by the FOC. 
  * @param this related object of class CSTC
  * @retval CSPD speed sensor utilized by the FOC.
  */
CSPD STC_GetSpeedSensor(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  return (pVars->oSPD);
}

/**
  * @brief  It should be called before each motor restart. If STC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param this related object of class CSTC.
  * @retval none.
  */
void STC_Clear(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  if (pVars->bMode == STC_SPEED_MODE)
  {
    PI_SetIntegralTerm(((_CSTC)this)->Vars_str.oPISpeed,0);
  }
}

/**
  * @brief  Get the current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  * @param  this related object of class CSTC.
  * @retval int16_t current mechanical rotor speed reference expressed in tenths
  *         of HZ. 
  */
int16_t STC_GetMecSpeedRef01Hz(CSTC this)
{
  return ((int16_t)(((_CSTC)this)->Vars_str.wSpeedRef01HzExt/65536));
}

/**
  * @brief  Get the current motor torque reference. This value represents 
  *         actually the Iq current reference expressed in digit.
  *         To convert current expressed in digit to current expressed in Amps
  *         is possible to use the formula: 
  *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
  * @param  this related object of class CSTC.
  * @retval int16_t current motor torque reference. This value represents 
  *         actually the Iq current expressed in digit.
  */
int16_t STC_GetTorqueRef(CSTC this)
{
  return ((int16_t)(((_CSTC)this)->Vars_str.wTorqueRef/65536));
}

/**
  * @brief  Set the modality of the speed and torque controller. Two modality 
  *         are available Torque mode and Speed mode.
  *         In Torque mode is possible to set directly the motor torque 
  *         reference or execute a motor torque ramp. This value represents 
  *         actually the Iq current reference expressed in digit.
  *         In Speed mode is possible to set the mechanical rotor speed
  *         reference or execute a speed ramp. The required motor torque is
  *         automatically calculated by the STC.
  *         This command interrupts the execution of any previous ramp command 
  *         maintaining the last value of Iq.
  * @param  this related object of class CSTC.
  * @param  bMode modality of STC. It can be one of these two settings: 
  *         STC_TORQUE_MODE to enable the Torque mode or STC_SPEED_MODE to
  *         enable the Speed mode.
  * @retval none
  */
void STC_SetControlMode(CSTC this, STC_Modality_t bMode)
{
  pVars_t pVars = CLASS_VARS;
  pVars->bMode = bMode;
  pVars->wRampRemainingStep = 0u; /* Interrupts previous ramp. */
}

/**
  * @brief  Get the modality of the speed and torque controller. 
  * @param  this related object of class CSTC.
  * @retval STC_Modality_t It returns the modality of STC. It can be one of 
  *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
  */
STC_Modality_t STC_GetControlMode(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  return pVars->bMode;
}

/**
  * @brief  Starts the execution of a ramp using new target and duration. This
  *         command interrupts the execution of any previous ramp command.
  *         The generated ramp will be in the modality previously set by
  *         STC_SetControlMode method.
  * @param  this related object of class CSTC  
  * @param  hTargetFinal final value of command. This is different accordingly
  *         the STC modality. 
  *         If STC is in Torque mode hTargetFinal is the value of motor torque
  *         reference at the end of the ramp. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  *         If STC is in Speed mode hTargetFinal is the value of mechanical
  *         rotor speed reference at the end of the ramp expressed in tenths of
  *         HZ.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It return false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or max
  *         application torque or below min application speed depending on
  *         current modality of TSC) in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true. 
  */
bool STC_ExecRamp(CSTC this, int16_t hTargetFinal, uint32_t hDurationms)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  bool AllowedRange = TRUE;
  uint32_t wAux;
  int32_t wAux1;
  int16_t hCurrentReference;
  
  /* Check if the hTargetFinal is out of the bound of application. */
  if (pVars->bMode == STC_TORQUE_MODE)
  {
    hCurrentReference = STC_GetTorqueRef(this);
    #ifdef CHECK_BOUNDARY
    if ((int32_t)hTargetFinal > (int32_t)pVars->hMaxPositiveTorque)
    {
      AllowedRange = FALSE;
    }
    if ((int32_t)hTargetFinal < (int32_t)pVars->hMinNegativeTorque)
    {
      AllowedRange = FALSE;
    }
    #endif
  }
  else
  {
    hCurrentReference = (int16_t)(pVars->wSpeedRef01HzExt >> 16);
    
    #ifdef CHECK_BOUNDARY
    if ((int32_t)hTargetFinal > (int32_t)pParams->hMaxAppPositiveMecSpeed01Hz)
    {
      AllowedRange = FALSE;
    } else
      if (hTargetFinal < pParams->hMinAppNegativeMecSpeed01Hz)
      {
        AllowedRange = FALSE;
      } else
        if ((int32_t)hTargetFinal < (int32_t)pParams->hMinAppPositiveMecSpeed01Hz)
        {
          if (hTargetFinal > pParams->hMaxAppNegativeMecSpeed01Hz)
          {
            AllowedRange = FALSE;
          }
        }
        else{}
    #endif
  }
  
  if (AllowedRange == TRUE)
  {
    /* Interrupts the execution of any previous ramp command */
    if (hDurationms == 0u)
    {
      if (pVars->bMode == STC_SPEED_MODE)
      {
        pVars->wSpeedRef01HzExt = (int32_t)hTargetFinal * 65536;
      }
      else
      {
        pVars->wTorqueRef = (int32_t)hTargetFinal * 65536;
      }
      pVars->wRampRemainingStep = 0u;
      pVars->wIncDecAmount = 0;
    }
    else
    {
      /* Store the hTargetFinal to be applied in the last step */
      pVars->hTargetFinal = hTargetFinal;
      
      /* Compute the (wRampRemainingStep) number of steps remaining to complete 
      the ramp. */
      wAux = (uint32_t)hDurationms * (uint32_t)pParams->hSTCFrequencyHz;
      wAux /= 1000u;
      pVars->wRampRemainingStep = wAux;
      pVars->wRampRemainingStep++;
      
      /* Compute the increment/decrement amount (wIncDecAmount) to be applied to 
      the reference value at each CalcTorqueReference. */
      wAux1 = ((int32_t)hTargetFinal - (int32_t)hCurrentReference) * 65536;
      wAux1 /= (int32_t)pVars->wRampRemainingStep;
      pVars->wIncDecAmount = wAux1;
    }
  }

  return AllowedRange;
}

/**
  * @brief  This command interrupts the execution of any previous ramp command.
  *         If STC has been set in Torque mode the last value of Iq is
  *         maintained.
  *         If STC has been set in Speed mode the last value of mechanical
  *         rotor speed reference is maintained.
  * @param  this related object of class CSTC.
  * @retval none
  */
void STC_StopRamp(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->wRampRemainingStep = 0u;
	pVars->wIncDecAmount = 0;
}

/**
  * @brief  It is used to compute the new value of motor torque reference. It
  *         must be called at fixed time equal to hSTCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  this related object of class CSTC.
  * @retval int16_t motor torque reference. This value represents actually the
  *         Iq current expressed in digit.
  *         To convert current expressed in Amps to current expressed in digit
  *         is possible to use the formula:
  *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
  */
int16_t STC_CalcTorqueReference(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  int32_t wCurrentReference;
  int16_t hTorqueReference = 0;
  int16_t hMeasuredSpeed;
  int16_t hTargetSpeed;
  int16_t hError;
  
  if (pVars->bMode == STC_TORQUE_MODE)
  {
    wCurrentReference = pVars->wTorqueRef;
  }
  else
  {
    wCurrentReference = pVars->wSpeedRef01HzExt;
  }
  
  /* Update the speed reference or the torque reference according to the mode 
     and terminates the ramp if needed. */
  if (pVars->wRampRemainingStep > 1u)
  {
    /* Increment/decrement the reference value. */
    wCurrentReference += pVars->wIncDecAmount;
    
    /* Decrement the number of remaining steps */
    pVars->wRampRemainingStep--;
  }
  else if (pVars->wRampRemainingStep == 1u)
  {
    /* Set the backup value of hTargetFinal. */
    wCurrentReference = (int32_t)pVars->hTargetFinal * 65536;
    pVars->wRampRemainingStep = 0u;
  }
  else
  {
    /* Do nothing. */
  }
  
  if (pVars->bMode == STC_SPEED_MODE)
  {
    /* Run the speed control loop */
    
    /* Compute speed error */
    hTargetSpeed = (int16_t)(wCurrentReference / 65536);
    hMeasuredSpeed = SPD_GetAvrgMecSpeed01Hz(pVars->oSPD);
    hError = hTargetSpeed - hMeasuredSpeed;
    hTorqueReference = PI_Controller(pVars->oPISpeed, (int32_t)hError);
    
    pVars->wSpeedRef01HzExt = wCurrentReference;
    pVars->wTorqueRef = (int32_t)hTorqueReference * 65536;
  }
  else
  {
    pVars->wTorqueRef = wCurrentReference;
    hTorqueReference = (int16_t)(wCurrentReference / 65536);
  }
  
  return hTorqueReference;
}

/**
  * @brief  Get the Default mechanical rotor speed reference expressed in tenths
  *         of HZ.
  * @param  this related object of class CSTC.
  * @retval int16_t It returns the Default mechanical rotor speed. reference 
  *         expressed in tenths of HZ.
  */
int16_t STC_GetMecSpeedRef01HzDefault(CSTC this)
{
  pParams_t pParams = CLASS_PARAMS;

  return pParams->hMecSpeedRef01HzDefault;
}

/**
  * @brief  Get the Application maximum positive value of rotor speed. It's 
            expressed in tenth of mechanical Hertz.
  * @param  this related object of class CSTC.
  * @retval uint16_t It returns the application maximum positive value of rotor
            speed expressed in tenth of mechanical Hertz.
  */
uint16_t STC_GetMaxAppPositiveMecSpeed01Hz(CSTC this)
{
  pParams_t pParams = CLASS_PARAMS;

  return pParams->hMaxAppPositiveMecSpeed01Hz;
}

/**
  * @brief  Get the Application minimum negative value of rotor speed. It's 
            expressed in tenth of mechanical Hertz.
  * @param  this related object of class CSTC.
  * @retval uint16_t It returns the application minimum negative value of rotor
            speed expressed in tenth of mechanical Hertz.
  */
int16_t STC_GetMinAppNegativeMecSpeed01Hz(CSTC this)
{
  pParams_t pParams = CLASS_PARAMS;

  return pParams->hMinAppNegativeMecSpeed01Hz;
}

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  this related object of class CSTC.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool STC_RampCompleted(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  bool retVal = FALSE;
  if (pVars->wRampRemainingStep == 0u)
  {
    retVal = TRUE;
  }
  return retVal;
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  this related object of class CSTC.
  * @retval bool It returns TRUE if the command is executed, FALSE otherwise.
  */
bool STC_StopSpeedRamp(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  bool retVal = FALSE;
  if (pVars->bMode == STC_SPEED_MODE)
  {
    pVars->wRampRemainingStep = 0u;
    retVal = TRUE;
  }
  return retVal;
}
                                             
/**
  * @brief It returns the default values of Iqdref. 
  * @param this related object of class CSTC
  * @retval default values of Iqdref.
  */
Curr_Components STC_GetDefaultIqdref(CSTC this)
{
  pParams_t pParams = CLASS_PARAMS;
  Curr_Components IqdRefDefault;
  IqdRefDefault.qI_Component1 = pParams->hTorqueRefDefault;
  IqdRefDefault.qI_Component2 = pParams->hIdrefDefault;
  return IqdRefDefault;
}

/**
  * @brief  Change the nominal current .
  * @param  this related object of class CSTC.
  * @param  hNominalCurrent This value represents actually the maximum Iq current 
            expressed in digit.
  * @retval none
  */
void STC_SetNominalCurrent(CSTC this, uint16_t hNominalCurrent)
{
  pVars_t pVars = CLASS_VARS;
  pVars->hMaxPositiveTorque = hNominalCurrent;
  pVars->hMinNegativeTorque = -hNominalCurrent;
}

/**
  * @brief  Force the speed reference to the curren speed. It is used
  *         at the START_RUN state to initialize the speed reference.
  * @param  this related object of class CSTC  
  * @retval none
  */
void STC_ForceSpeedReferenceToCurrentSpeed(CSTC this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->wSpeedRef01HzExt = (int32_t)SPD_GetAvrgMecSpeed01Hz(pVars->oSPD) * (int32_t)65536;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
