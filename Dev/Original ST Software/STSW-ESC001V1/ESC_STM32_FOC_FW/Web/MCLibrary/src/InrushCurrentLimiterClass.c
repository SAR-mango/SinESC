/**
  ******************************************************************************
  * @file    InrushCurrentLimiterClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of InrushCurrentLimiter class
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
#include "InrushCurrentLimiterClass.h"
#include "InrushCurrentLimiterPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CICL_t ICLpool[MAX_ICL_NUM];
  unsigned char ICL_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CICL)this)->Vars_str
#define CLASS_PARAMS  ((_CICL)this)->pParams_str

/**
  * @brief  Creates an object of the class InrushCurrentLimiter
  * @param  pInrushCurrentLimiterParams pointer to an InrushCurrentLimiter parameters structure
  * @retval CICL new instance of InrushCurrentLimiter object
  */
CICL ICL_NewObject(pInrushCurrentLimiterParams_t pInrushCurrentLimiterParams)
{
  _CICL _oICL;
  
  #ifdef MC_CLASS_DYNAMIC
    _oICL = (_CICL)calloc(1u,sizeof(_CICL_t));
  #else
    if (ICL_Allocated  < MAX_ICL_NUM)
    {
      _oICL = &ICLpool[ICL_Allocated++];
    }
    else
    {
      _oICL = MC_NULL;
    }
  #endif
  
  _oICL->pParams_str = (pParams_t)pInrushCurrentLimiterParams;
  
  return ((CICL)_oICL);
}

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation. It is also used to assign the 
  *         bus voltage sensor and digital output to be used by the ICL.
  * @param  this related object of class CICL.
  * @param  oVBS the bus voltage sensor used by the ICL.
  * @param  oDOUT the digital output used by the ICL.
  * @retval none.
  */
void ICL_Init(CICL this, CVBS oVBS, CDOUT oDOUT)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  int32_t wAux;
  
  pVars->oVBS = oVBS;
  pVars->oDOUT = oDOUT;
  pVars->ICLState = ICL_ACTIVE;
  DOUT_SetOutputState(oDOUT, ACTIVE);
  pVars->hRemainingTicks = 0u;
  wAux = (int32_t)(pParams->hDurationms);
  wAux *= (int32_t)(pParams->hICLFrequencyHz);
  wAux /= 1000;
  wAux -= 1;
  if (wAux > (int32_t)(U16_MAX))
  {
    wAux = (int32_t)(U16_MAX);
  }
  if (wAux < 1)
  {
    wAux = 1;
  }
  pVars->hTotalTicks = (uint16_t)(wAux);
}

/**
  * @brief  It clocks the inrush current limiter and must be called with a 
  *         frequency equal to the one settled in the parameters
  *         hEACFrequencyHz. 
  * @param  this related object of class CICL.
  * @retval ICLState_t returns the ICL state see 
  * \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink.
  */
ICLState_t ICL_Exec(CICL this)
{
  pVars_t pVars = CLASS_VARS;
  
  /* ICL actions.*/
  switch (pVars->ICLState)
  {
  case ICL_ACTIVATION:
    {
      /* ICL activation: counting the step before pass in ICL_ACTIVE.*/
      if (pVars->hRemainingTicks == 0u)
      {
        pVars->ICLState = ICL_ACTIVE;
      }
      else
      {
        pVars->hRemainingTicks--;
      }
    }
    break;
  case ICL_DEACTIVATION:
    {
      /* ICL deactivation: counting the step before pass in ICL_INACTIVE.*/
      if (pVars->hRemainingTicks == 0u)
      {
        pVars->ICLState = ICL_INACTIVE;
      }
      else
      {
        pVars->hRemainingTicks--;
      }
    }
    break;
  case ICL_ACTIVE:
    {
      /* ICL is active: if bus is present deactivate the ICL */
      if (VBS_CheckVbus(pVars->oVBS) != MC_UNDER_VOLT)
      {
        DOUT_SetOutputState(pVars->oDOUT, INACTIVE);
        pVars->ICLState = ICL_DEACTIVATION;
        pVars->hRemainingTicks = pVars->hTotalTicks;
      }
    }
    break;
  case ICL_INACTIVE:
    {
      /* ICL is inactive: if bus is not present activate the ICL */
      if (VBS_CheckVbus(pVars->oVBS) == MC_UNDER_VOLT)
      {
        DOUT_SetOutputState(pVars->oDOUT, ACTIVE);
        pVars->ICLState = ICL_ACTIVATION;
        pVars->hRemainingTicks = pVars->hTotalTicks;
      }
    }
    break;
  case ICL_IDLE:
  default:
    {
    }
    break;
  }
  
  return pVars->ICLState;
}

/**
  * @brief It returns the state of the ICL. See 
  * \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink.
  * @param this related object of class CICL.
  * @retval ICLState_t returns the ICL state see 
  * \link InrushCurrentLimiter_class_exported_types ICLState_t\endlink.
  */
ICLState_t ICL_GetState(CICL this)
{
  pVars_t pVars = CLASS_VARS;
  
  return pVars->ICLState;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
