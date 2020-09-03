/**
  ******************************************************************************
  * @file    RampMngrClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of RampMngr class
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
#include "RampMngrClass.h"
#include "RampMngrPrivate.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #define MAX_RMNG_NUM 4u

  _CRMNG_t RMNGpool[MAX_RMNG_NUM];
  unsigned char RMNG_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CRMNG)this)->Vars_str
#define CLASS_PARAMS  ((_CRMNG)this)->pParams_str

/**
  * @brief  Creates an object of the class RampMngr
  * @param  pRampMngrParams pointer to an RampMngr parameters structure
  * @retval CRMNG new instance of RampMngr object
  */
CRMNG RMNG_NewObject(pRampMngrParams_t pRampMngrParams)
{
  _CRMNG _oRMNG;
  
  #ifdef MC_CLASS_DYNAMIC
    _oRMNG = (_CRMNG)calloc(1u,sizeof(_CRMNG_t));
  #else
    if (RMNG_Allocated  < MAX_RMNG_NUM)
    {
      _oRMNG = &RMNGpool[RMNG_Allocated++];
    }
    else
    {
      _oRMNG = MC_NULL;
    }
  #endif
  
  _oRMNG->pParams_str = (pParams_t)pRampMngrParams;
  
  RMNG_Init((CRMNG)_oRMNG);
  
  return ((CRMNG)_oRMNG);
}

/**
  * @brief  It reset the state variable to zero.
  * @param  this related object of class CRMNG
  * @retval none.
  */
void RMNG_Init(CRMNG this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->wExt = 0;
  pVars->hTargetFinal = 0;
  pVars->wRampRemainingStep = 0u;
  pVars->wIncDecAmount = 0;
}

/**
  * @brief  Exec the ramp calculations and returns the current value of the 
            state variable. 
            It must be called at fixed interval defined in the hExecFreq.
  * @param  this related object of class CRMNG
  * @retval int16_t value of the state variable
  */
int16_t RMNG_Calc(CRMNG this)
{
  int16_t hRetVal;
  pVars_t pVars = CLASS_VARS;
  int32_t wCurrentReference;
  
  wCurrentReference = pVars->wExt;
  
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
    wCurrentReference = (int32_t)pVars->hTargetFinal * 32768;
    pVars->wRampRemainingStep = 0u;
  }
  else
  {
    /* Do nothing. */
  }
  
  pVars->wExt = wCurrentReference;
  
  hRetVal = (int16_t)(pVars->wExt / 32768);
  return hRetVal;
}

/**
  * @brief  Setup the ramp to be executed
  * @param  this related object of class CRMNG
  * @param  hTargetFinal final value of state variable at the end of the ramp.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval none
  */
void RMNG_ExecRamp(CRMNG this, int16_t hTargetFinal, uint16_t hDurationms)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  uint32_t wAux;
  int32_t wAux1;
  int16_t hCurrentReference;
  
  hCurrentReference = (int16_t)(pVars->wExt / 32768);
  
  if (hDurationms == 0u)
  {
    pVars->wExt = (int32_t)hTargetFinal * 32768;
    pVars->wRampRemainingStep = 0u;
    pVars->wIncDecAmount = 0;
  }
  else
  {
    /* Store the hTargetFinal to be applied in the last step */
    pVars->hTargetFinal = hTargetFinal;
    
    /* Compute the (wRampRemainingStep) number of steps remaining to complete 
    the ramp. */
    wAux = (uint32_t)hDurationms * (uint32_t)pParams->hFrequencyHz;
    wAux /= 1000u;
    pVars->wRampRemainingStep = wAux;
    pVars->wRampRemainingStep++;
    
    /* Compute the increment/decrement amount (wIncDecAmount) to be applied to 
    the reference value at each CalcTorqueReference. */
    wAux1 = ((int32_t)hTargetFinal - (int32_t)hCurrentReference) * 32768;
    wAux1 /= (int32_t)pVars->wRampRemainingStep;
    pVars->wIncDecAmount = wAux1;
  }
}

/**
  * @brief  Returns the current value of the state variable.
  * @param  this related object of class CRMNG
  * @retval int16_t value of the state variable
  */
int16_t RMNG_GetValue(CRMNG this)
{
  int16_t hRetVal;
  pVars_t pVars = CLASS_VARS;
  hRetVal = (int16_t)(pVars->wExt / 32768);
  return hRetVal;
}

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  this related object of class CRMNG.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool RMNG_RampCompleted(CRMNG this)
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
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  this related object of class CRMNG.
  * @retval none
  */
void RMNG_StopRamp(CRMNG this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->wRampRemainingStep = 0u;
  pVars->wIncDecAmount = 0;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
