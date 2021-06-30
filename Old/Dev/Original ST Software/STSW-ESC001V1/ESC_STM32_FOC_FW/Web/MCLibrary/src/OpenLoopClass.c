/**
  ******************************************************************************
  * @file    OpenLoopClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of OpenLoop class
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
#include "OpenLoopClass.h"
#include "OpenLoopPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _COL_t OLpool[MAX_OL_NUM];
  unsigned char OL_Allocated = 0u;
#endif

#define CLASS_VARS   &((_COL)this)->Vars_str
#define CLASS_PARAMS  ((_COL)this)->pParams_str

/**
  * @brief  Creates an object of the class OpenLoop.
  * @param  pOpenLoopParams pointer to an OpenLoop parameters structure.
  * @retval COL new instance of OpenLoop object.
  */
COL OL_NewObject(pOpenLoopParams_t pOpenLoopParams)
{
  _COL _oOL;
  
  #ifdef MC_CLASS_DYNAMIC
    _oOL = (_COL)calloc(1u,sizeof(_COL_t));
  #else
    if (OL_Allocated  < MAX_OL_NUM)
    {
      _oOL = &OLpool[OL_Allocated++];
    }
    else
    {
      _oOL = MC_NULL;
    }
  #endif
  
  _oOL->pParams_str = (pParams_t)pOpenLoopParams;
  
  return ((COL)_oOL);
}

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation.
  * @param  this related object of class COL.
  * @param  oVSS Related VSS object used.
  * @retval none
  */
void OL_Init(COL this, CSPD oVSS)
{
  pParams_t pParams = CLASS_PARAMS;
  pVars_t pVars = CLASS_VARS;
  pVars->hVoltage = pParams->hDefaultVoltage;
  pVars->oVSS = oVSS;
  pVars->VFMode = pParams->VFMode;
}

/**
  * @brief  It return the open loop Vqd.
  * @param  this related object of class COL.
  * @retval Volt_Components Vqd conditioned values.
  */
Volt_Components OL_VqdConditioning(COL this)
{
  pVars_t pVars = CLASS_VARS;
  Volt_Components Vqd;
  
  Vqd.qV_Component1 = pVars->hVoltage;
  Vqd.qV_Component2 = 0;
  
 return(Vqd); 
}

/**
  * @brief  It allows changing applied open loop phase voltage.
  * @param  this related object of class COL
  * @param  hNewVoltage New voltage value to be applied by the open loop.
  * @retval None
  */
void OL_UpdateVoltage(COL this, int16_t hNewVoltage)
{
  pVars_t pVars = CLASS_VARS;
  pVars->hVoltage = hNewVoltage;
}

/**
  * @brief  It have to be called regularly to update the applied voltage in case
  *         V/F is enabled. If V/F is disabled nothing is done.
  * @param  this related object of class COL
  * @retval None
  */
void OL_Calc(COL this)
{
  pParams_t pParams = CLASS_PARAMS;
  pVars_t pVars = CLASS_VARS;
  if (pVars->VFMode == TRUE) 
  {
    /* V/F mode TRUE means enabled */
    int16_t hMecSpeed01Hz = SPD_GetAvrgMecSpeed01Hz(pVars->oVSS);
    int16_t hOLVoltage = pParams->hVFOffset + (pParams->hVFSlope*hMecSpeed01Hz);
    pVars->hVoltage = hOLVoltage;
  }
}

/**
  * @brief  It is used to enable or disable the V/F mode.
  * @param  this related object of class COL
  * @param  VFEnabling TRUE to enable V/F mode, FALSE to disable. 
  * @retval None
  */
void OL_VF(COL this, bool VFEnabling)
{
  pVars_t pVars = CLASS_VARS;
  pVars->VFMode = VFEnabling;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
