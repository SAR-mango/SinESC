/**
  ******************************************************************************
  * @file    MotorPowerMeasurementClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MotorPowerMeasurement class
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
#include "MotorPowerMeasurementClass.h"
#include "MotorPowerMeasurementPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CMPM_t MPMpool[MAX_MPM_NUM];
  unsigned char MPM_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CMPM)this)->Vars_str
#define CLASS_PARAMS  ((_CMPM)this)->pParams_str

/**
  * @brief  Creates an object of the class MotorPowerMeasurement
  * @param  pMotorPowerMeasurementParams pointer to an MotorPowerMeasurement 
  *         parameters structure
  * @retval CMPM new instance of MotorPowerMeasurement object
  */
CMPM MPM_NewObject(void)
{
  _CMPM _oMPM;
  
  #ifdef MC_CLASS_DYNAMIC
    _oMPM = (_CMPM)calloc(1u,sizeof(_CMPM_t));
  #else
    if (MPM_Allocated  < MAX_MPM_NUM)
    {
      _oMPM = &MPMpool[MPM_Allocated++];
    }
    else
    {
      _oMPM = MC_NULL;
    }
  #endif
    
  return ((CMPM)_oMPM);
}

/**
  * @brief Initializes all the object variables, usually it has to be called 
  *        once right after object creation.
  * @param this related object of class CMPM.
  * @param pMPMInitStruct the pointer of the init structure, required by derived
  *       class, up-casted to pMPMInitStruct_t.
  * @retval none.
  */
void MPM_Init(CMPM this, pMPMInitStruct_t pMPMInitStruct)
{
  ((_CMPM)this)->Methods_str.pPQD_Init(this, pMPMInitStruct);
  MPM_Clear(this);
}

/**
  * @brief  It should be called before each motor restart. It clears the 
  *         measurement buffer and initialize the index.
  * @param this related object of class CMPM.
  * @retval none.
  */
void MPM_Clear(CMPM this)
{
  uint16_t i;
  pVars_t pVars = CLASS_VARS;
  for (i = 0u; i < MPM_BUFFER_LENGHT; i++)
  {
    pVars->hMeasBuffer[i] = 0;
  }
  pVars->hNextMeasBufferIndex = 0u;
  pVars->hLastMeasBufferIndex = 0u;
  ((_CMPM)this)->Methods_str.pPQD_Clear(this);
}

/**
  * @brief  This method should be called with periodicity. It computes and 
  *         returns the measured motor power expressed in watt. It is also used
  *         to fill, with that measure, the buffer used to compute the average
  *         motor power. 
  * @param this related object of class CMPM.
  * @retval int16_t The measured motor power expressed in watt.
  */
int16_t MPM_CalcElMotorPower(CMPM this)
{
  uint16_t i;
  int32_t wAux = 0;
  pVars_t pVars = CLASS_VARS;
  int16_t hMotorPower = 0;
  hMotorPower = ((_CMPM)this)->Methods_str.pPQD_CalcElMotorPower(this);
  /* Store the measured values in the buffer.*/
  pVars->hMeasBuffer[pVars->hNextMeasBufferIndex] = hMotorPower;
  pVars->hLastMeasBufferIndex = pVars->hNextMeasBufferIndex;
  pVars->hNextMeasBufferIndex++;
  if (pVars->hNextMeasBufferIndex >= MPM_BUFFER_LENGHT)
  {
    pVars->hNextMeasBufferIndex = 0u;
  }
  /* Compute the average measured motor power */
  for (i = 0u; i < MPM_BUFFER_LENGHT; i++)
  {
    wAux += (int32_t)(pVars->hMeasBuffer[i]);
  }
  wAux /= (int32_t)MPM_BUFFER_LENGHT;
  pVars->hAvrgElMotorPowerW = (int16_t)(wAux);
  /* Return the last measured motor power */
  return hMotorPower;
}

/**
  * @brief  This method is used to get the last measured motor power 
  *         (instantaneous value) expressed in watt.
  * @param this related object of class CMPM.
  * @retval int16_t The last measured motor power (instantaneous value) 
  *         expressed in watt.
  */
int16_t MPM_GetElMotorPowerW(CMPM this)
{
  pVars_t pVars = CLASS_VARS;
  return pVars->hMeasBuffer[pVars->hLastMeasBufferIndex];
}

/**
  * @brief  This method is used to get the average measured motor power 
  *         expressed in watt.
  * @param this related object of class CMPM.
  * @retval int16_t The average measured motor power expressed in watt.
  */
int16_t MPM_GetAvrgElMotorPowerW(CMPM this)
{
  pVars_t pVars = CLASS_VARS;
  return pVars->hAvrgElMotorPowerW;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
