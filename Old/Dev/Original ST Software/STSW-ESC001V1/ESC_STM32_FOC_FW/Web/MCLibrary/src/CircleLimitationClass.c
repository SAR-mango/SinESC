/**
  ******************************************************************************
  * @file    CircleLimitationClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of CircleLimitation class
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
#include "CircleLimitationClass.h"
#include "CircleLimitationPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CCLM_t CLMpool[MAX_CLM_NUM];
  unsigned char CLM_Allocated = 0u;
#endif

#define CLASS_VARS   &((_CCLM)this)->Vars_str
#define CLASS_PARAMS  ((_CCLM)this)->pParams_str

/**
  * @brief  Creates an object of the class CircleLimitation
  * @param  pCircleLimitationParams pointer to an CircleLimitation parameters structure
  * @retval CCLM new instance of CircleLimitation object
  */
CCLM CLM_NewObject(pCircleLimitationParams_t pCircleLimitationParams)
{
  _CCLM _oCLM;
  
  #ifdef MC_CLASS_DYNAMIC
    _oCLM = (_CCLM)calloc(1u,sizeof(_CCLM_t));
  #else
    if (CLM_Allocated  < MAX_CLM_NUM)
    {
      _oCLM = &CLMpool[CLM_Allocated++];
    }
    else
    {
      _oCLM = MC_NULL;
    }
  #endif
  
  _oCLM->pParams_str = (pParams_t)pCircleLimitationParams;
  
  return ((CCLM)_oCLM);
}

/**
  * @brief Check whether Vqd.qV_Component1^2 + Vqd.qV_Component2^2 <= 32767^2 
  *        and if not it applies a limitation keeping constant ratio 
  *        Vqd.qV_Component1 / Vqd.qV_Component2
  * @param  this related object of class CCLM
  * @param  Vqd Voltage in qd reference frame  
  * @retval Volt_Components Limited Vqd vector
  */
Volt_Components Circle_Limitation(CCLM this, Volt_Components Vqd)
{
  uint32_t uw_temp;
  int32_t sw_temp; 
  Volt_Components Local_Vqd=Vqd;
  pParams_t pParams = CLASS_PARAMS;
  
  sw_temp =(int32_t)(Vqd.qV_Component1) * Vqd.qV_Component1 + 
    (int32_t)(Vqd.qV_Component2) * Vqd.qV_Component2;  
  uw_temp =(uint32_t) sw_temp;
  /* uw_temp min value 0, max value 2*32767*32767 */
  
  if (uw_temp > (uint32_t)(pParams->hMaxModule) * pParams->hMaxModule) 
  {
    uint16_t hTable_Element;
    
    uw_temp /= (uint32_t)(16777216); 
    /* wtemp min value pParams->bStart_index, max value 127 */
    uw_temp -= pParams->bStart_index;
    
    /* uw_temp min value 0, max value 127 - pParams->bStart_index */   
    hTable_Element = pParams->hCircle_limit_table[(uint8_t)uw_temp];
    
    sw_temp = Vqd.qV_Component1 * (int32_t)hTable_Element; 
    Local_Vqd.qV_Component1 = (int16_t)(sw_temp/32768);  
    
    sw_temp = Vqd.qV_Component2 * (int32_t)(hTable_Element); 
    Local_Vqd.qV_Component2 = (int16_t)(sw_temp/32768);  
  }
  
  return(Local_Vqd);
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
