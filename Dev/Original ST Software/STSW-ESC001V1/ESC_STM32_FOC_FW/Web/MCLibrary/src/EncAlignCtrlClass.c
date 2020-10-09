/**
  ******************************************************************************
  * @file    EncAlignCtrlClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of EncAlignCtrl class
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
#include "EncAlignCtrlClass.h"
#include "EncAlignCtrlPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CEAC_t EACpool[MAX_EAC_NUM];
  unsigned char EAC_Allocated = 0u;
#endif
  
#define CLASS_VARS   &((_CEAC)this)->Vars_str
#define CLASS_PARAMS  ((_CEAC)this)->pParams_str
  
/**
  * @brief  Creates an object of the class EncAlignCtrl
  * @param  pEncAlignCtrlParams pointer to an EncAlignCtrl parameters structure
  * @retval CEAC new instance of EncAlignCtrl object
  */
CEAC EAC_NewObject(pEncAlignCtrlParams_t pEncAlignCtrlParams)
{
  _CEAC _oEAC;
  
  #ifdef MC_CLASS_DYNAMIC
    _oEAC = (_CEAC)calloc(1u,sizeof(_CEAC_t));
  #else
    if (EAC_Allocated  < MAX_EAC_NUM)
    {
      _oEAC = &EACpool[EAC_Allocated++];
    }
    else
    {
      _oEAC = MC_NULL;
    }
  #endif
  
  _oEAC->pParams_str = (pParams_t)pEncAlignCtrlParams;
  
  return ((CEAC)_oEAC);
}

/**
  * @brief  Initializes all the object variables, usually it has to be called 
  *         once right after object creation. It is also used to assign the 
  *         speed and torque controller, the virtual speed sensor objects and 
  *         the encoder object to be used by encoder alignment controller.
  * @param  this related object of class CEAC.
  * @param  oSTC the speed and torque controller used by the EAC.
  * @param  oVSS the virtual speed sensor used by the EAC.
  * @param  oENC the encoder object used by the EAC.
  * @retval none.
  */
void EAC_Init(CEAC this, CSTC oSTC, CVSS_SPD oVSS, CENC_SPD oENC )
{
  pVars_t pVars = CLASS_VARS;
  pVars->oSTC = oSTC;
  pVars->oVSS = oVSS;
  pVars->oENC = oENC;
  pVars->EncAligned = FALSE;
  pVars->EncRestart = FALSE;
}

/**
  * @brief  It is called to start the encoder alignment procedure.
  *	       	It configure the VSS with the required angle and sets the STC to 
  *         execute the required torque ramp.
  * @param  this related object of class CEAC.
  * @retval none.
  */
void EAC_StartAlignment(CEAC this)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  
  uint32_t wAux;
  
  /* Set oVSS mechanical speed to zero.*/
  VSPD_SetMecAcceleration((CSPD)pVars->oVSS, 0, 0u);
  
  /* Set oVSS mechanical angle.*/
  SPD_SetMecAngle((CSPD)pVars->oVSS, pParams->hElAngle);
  
  /* Set oSTC in STC_TORQUE_MODE.*/
  STC_SetControlMode(pVars->oSTC, STC_TORQUE_MODE);
  
  /* Set starting torque to Zero */
  STC_ExecRamp(pVars->oSTC, 0, 0u);
  
  /* Execute the torque ramp.*/
  STC_ExecRamp(pVars->oSTC, pParams->hFinalTorque, (uint32_t)(pParams->hDurationms));
  
  /* Compute hRemainingTicks, the number of thick of alignment phase.*/
  wAux = (uint32_t)pParams->hDurationms * (uint32_t)pParams->hEACFrequencyHz;
  wAux /= 1000u;
  pVars->hRemainingTicks = (uint16_t)(wAux);
  pVars->hRemainingTicks++;  
}

/**
  * @brief  It clocks the encoder alignment controller and must be called with a 
  *         frequency equal to the one settled in the parameters 
  *         hEACFrequencyHz. Calling this method the EAC is possible to verify 
  *         if the alignment duration has been finished. At the end of alignment 
  *         the encoder object it is set to the defined mechanical angle.
  *         Note: STC, VSS, ENC are not clocked by EAC_Exec.
  * @param  this related object of class CEAC.
  * @retval bool It returns TRUE when the programmed alignment has been 
  *         completed. 
  */
bool EAC_Exec(CEAC this)
{
  pVars_t pVars = CLASS_VARS;
  pParams_t pParams = CLASS_PARAMS;
  
  bool retVal = TRUE;

  if (pVars->hRemainingTicks > 0u)
  {
    pVars->hRemainingTicks--;
    
    if (pVars->hRemainingTicks == 0u)
    {
      /* Set oVSS mechanical angle.*/
      SPD_SetMecAngle((CSPD)pVars->oENC, pParams->hElAngle / (int16_t)(pParams->bElToMecRatio));
      pVars->EncAligned = TRUE;
      retVal = TRUE;
    }
    else
    {
      retVal = FALSE;
    }
  }
  
  return retVal;
}

/**
  * @brief  This function returns TRUE if the encoder has been aligned at least 
  *         one time, FALSE if hasn't been never aligned.
  * @param  this related object of class CEAC.
  * @retval bool It returns TRUE if the encoder has been aligned at least 
  *         one time, FALSE if hasn't been never aligned. 
  */
bool EAC_IsAligned(CEAC this)
{
  pVars_t pVars = CLASS_VARS;
  return pVars->EncAligned;
}

/**
  * @brief  This function is used to program a restart after an encoder 
  *         alignment.
  * @param  this related object of class CEAC.
  * @param  restart Set to TRUE if a restart is programmed else FALSE
  * @retval none. 
  */
void EAC_SetRestartState(CEAC this, bool restart)
{
  pVars_t pVars = CLASS_VARS;
  pVars->EncRestart = restart;
}

/**
  * @brief  This function is used to verify if a restart after an encoder 
  *         alignment has been requested.
  * @param  this related object of class CEAC.
  * @retval bool It is TRUE if a restart is programmed else FALSE.
  */
bool EAC_GetRestartState(CEAC this)
{
  pVars_t pVars = CLASS_VARS;
  return pVars->EncRestart;
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
