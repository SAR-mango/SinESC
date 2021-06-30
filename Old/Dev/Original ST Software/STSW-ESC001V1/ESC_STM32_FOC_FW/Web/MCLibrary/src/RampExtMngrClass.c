/**
  ******************************************************************************
  * @file    RampExtMngr.c
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
#include "RampExtMngrClass.h"
#include "RampExtMngrPrivate.h"
#include "MC_type.h"
#ifdef STM32F0XX
#include "FastDivClass.h"
#endif

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #define MAX_REMNG_NUM 4u

  _CREMNG_t REMNGpool[MAX_REMNG_NUM];
  unsigned char REMNG_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CREMNG)this)->Vars_str
#define CLASS_PARAMS  ((_CREMNG)this)->pParams_str
  
static uint32_t getScalingFactor(int32_t wTarget);

/**
  * @brief  Creates an object of the class RampMngr
  * @param  pRampMngrParams pointer to an RampMngr parameters structure
  * @retval CREMNG new instance of RampMngr object
  */
CREMNG REMNG_NewObject(pRampExtMngrParams_t pRampExtMngrParams)
{
  _CREMNG _oREMNG;
  
  #ifdef MC_CLASS_DYNAMIC
    _oREMNG = (_CREMNG)calloc(1u,sizeof(_CREMNG_t));
  #else
    if (REMNG_Allocated  < MAX_REMNG_NUM)
    {
      _oREMNG = &REMNGpool[REMNG_Allocated++];
    }
    else
    {
      _oREMNG = MC_NULL;
    }
  #endif
  
  _oREMNG->pParams_str = (pParams_t)pRampExtMngrParams;
  
  REMNG_Init((CREMNG)_oREMNG);
  
#ifdef STM32F0XX
  _oREMNG->Vars_str.fd = FD_NewObject();
#endif
  
  return ((CREMNG)_oREMNG);
}

/**
  * @brief  It reset the state variable to zero.
  * @param  this related object of class CREMNG
  * @retval none.
  */
void REMNG_Init(CREMNG this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->wExt = 0;
  pVars->wTargetFinal = 0;
  pVars->wRampRemainingStep = 0u;
  pVars->wIncDecAmount = 0;
  pVars->wScalingFactor = 1u;
}

/**
  * @brief  Exec the ramp calculations and returns the current value of the 
            state variable. 
            It must be called at fixed interval defined in the hExecFreq.
  * @param  this related object of class CREMNG
  * @retval int32_t value of the state variable
  */
int32_t REMNG_Calc(CREMNG this)
{
  int32_t wRetVal;
  pVars_t pVars = CLASS_VARS;
  int32_t wCurrentReference;
  
  wCurrentReference = pVars->wExt;
  
  /* Update the variable and terminates the ramp if needed. */
  if (pVars->wRampRemainingStep > 1u)
  {
    /* Increment/decrement the reference value. */
    wCurrentReference += pVars->wIncDecAmount;
    
    /* Decrement the number of remaining steps */
    pVars->wRampRemainingStep--;
  }
  else if (pVars->wRampRemainingStep == 1u)
  {
    /* Set the backup value of wTargetFinal. */
    wCurrentReference = pVars->wTargetFinal * (int32_t)(pVars->wScalingFactor);
    pVars->wRampRemainingStep = 0u;
  }
  else
  {
    /* Do nothing. */
  }
  
  pVars->wExt = wCurrentReference;
  
#ifdef STM32F0XX
  wRetVal = FD_FastDiv(pVars->fd, pVars->wExt, (int32_t)(pVars->wScalingFactor));
#else
  wRetVal = pVars->wExt / (int32_t)(pVars->wScalingFactor);
#endif
  
  return wRetVal;
}

/**
  * @brief  Setup the ramp to be executed
  * @param  this related object of class CREMNG
  * @param  hTargetFinal (signed 32bit) final value of state variable at the end
  *         of the ramp.
  * @param  hDurationms (unsigned 32bit) the duration of the ramp expressed in 
  *         milliseconds. It is possible to set 0 to perform an instantaneous 
  *         change in the value.
  * @retval bool It returns TRUE is command is valid, FALSE otherwise
  */
bool REMNG_ExecRamp(CREMNG this, int32_t wTargetFinal, uint32_t wDurationms)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  uint32_t wAux;
  int32_t wAux1;
  int32_t wCurrentReference;
  bool retVal = TRUE;
  
  /* Get current state */
#ifdef STM32F0XX
  wCurrentReference = FD_FastDiv(pVars->fd, pVars->wExt, (int32_t)(pVars->wScalingFactor));
#else
  wCurrentReference = pVars->wExt / (int32_t)(pVars->wScalingFactor);
#endif
  
  if (wDurationms == 0u)
  {
    pVars->wScalingFactor = getScalingFactor(wTargetFinal);
    pVars->wExt = wTargetFinal * (int32_t)(pVars->wScalingFactor);
    pVars->wRampRemainingStep = 0u;
    pVars->wIncDecAmount = 0;
  }
  else
  {
    uint32_t wScalingFactor = getScalingFactor(wTargetFinal - wCurrentReference);
    uint32_t wScalingFactor2 = getScalingFactor(wCurrentReference);
    uint32_t wScalingFactor3 = getScalingFactor(wTargetFinal);
    uint32_t wScalingFactorMin;
    
    if (wScalingFactor <  wScalingFactor2)
    {
      if (wScalingFactor < wScalingFactor3)
      {
        wScalingFactorMin = wScalingFactor;
      }
      else
      {
         wScalingFactorMin = wScalingFactor3;
      }
    }
    else
    {
      if (wScalingFactor2 < wScalingFactor3)
      {
        wScalingFactorMin = wScalingFactor2;
      }
      else
      {
         wScalingFactorMin = wScalingFactor3;
      } 
    }
    
    pVars->wScalingFactor = wScalingFactorMin;
    pVars->wExt = wCurrentReference * (int32_t)(pVars->wScalingFactor);
    
    /* Store the wTargetFinal to be applied in the last step */
    pVars->wTargetFinal = wTargetFinal;
    
    /* Compute the (wRampRemainingStep) number of steps remaining to complete 
    the ramp. */
    wAux = wDurationms * (uint32_t)pParams->hFrequencyHz; /* Check for overflow and use prescaler */
    wAux /= 1000u;
    pVars->wRampRemainingStep = wAux;
    pVars->wRampRemainingStep++;
    
    /* Compute the increment/decrement amount (wIncDecAmount) to be applied to 
    the reference value at each CalcTorqueReference. */
    wAux1 = (wTargetFinal - wCurrentReference) * (int32_t)(pVars->wScalingFactor);
    wAux1 /= (int32_t)(pVars->wRampRemainingStep);
    pVars->wIncDecAmount = wAux1;
  }
  
  return retVal;
}

/**
  * @brief  Returns the current value of the state variable.
  * @param  this related object of class CREMNG
  * @retval int32_t value of the state variable
  */
int32_t REMNG_GetValue(CREMNG this)
{
  int32_t wRetVal;
  pVars_t pVars = CLASS_VARS;
  wRetVal = pVars->wExt / (int32_t)(pVars->wScalingFactor);
  return wRetVal;
}

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  this related object of class CREMNG.
  * @retval bool It returns TRUE if the ramp is completed, FALSE otherwise.
  */
bool REMNG_RampCompleted(CREMNG this)
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
  * @param  this related object of class CREMNG.
  * @retval none
  */
void REMNG_StopRamp(CREMNG this)
{
  pVars_t pVars = CLASS_VARS;
  pVars->wRampRemainingStep = 0u;
  pVars->wIncDecAmount = 0;
}

/**
  * @brief  Calculating the scaling factor to maximixe the resolution. It
  *         perform the 2^int(31-log2(wTarget)) with an iterative approach.
  *         It allows to keep wTarget * Scaling factor inside s32 type.
  * @param  wTarget Input data.
  * @retval uint32_t It returns the optimized scaling factor.
  */
static uint32_t getScalingFactor(int32_t wTarget)
{
  uint8_t i;
  uint32_t wTargetAbs;
  if (wTarget < 0)
  {
    int32_t wAux;
    wAux = -wTarget;
    wTargetAbs = (uint32_t)(wAux);
  }
  else
  {
    wTargetAbs = (uint32_t)(wTarget);
  }
  for (i = 1u; i < 32u; i++)
  {
    uint32_t limit = ((uint32_t)(1) << (31u-i));
    if (wTargetAbs > limit)
    {
      break;
    }
  }
  return ((uint32_t)(1u) << (i-1u));
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
