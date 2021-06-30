/**
  ******************************************************************************
  * @file    PIRegulatorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of PI class      
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
#include "PIRegulatorClass.h"
#include "PIRegulatorPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

/*#define MC_CLASS_DYNAMIC*/
#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  _CPI_t PIpool[MAX_PI_NUM];
  uint8_t PI_Allocated = 0u;
#endif

/**
  * @brief  Creates an object of the class PI regulator
  * @param  pPIParams_t pointer to a PI parameters structure
  * @retval CPI new instance of PI object
  */
CPI PI_NewObject(pPIParams_t pPIParams)
{
  _CPI _oPI;
  
  #ifdef MC_CLASS_DYNAMIC
  _oPI = (_CPI)calloc(1u, sizeof(_CPI_t));
  #else
    if (PI_Allocated  < MAX_PI_NUM)
    {
      _oPI = &PIpool[PI_Allocated++];
    }
    else
    {
      _oPI = MC_NULL;
    }
  #endif

  
  _oPI->pParams_str = (pParams_t)pPIParams;
    
  _oPI->DerivedClass = MC_NULL; 
  
  return ((CPI)_oPI);
}


/**
  * @brief  Initiliaze all the object variables, usually it has to be called 
  *         once right after object creation
  * @param  CPI PI regulator object
  * @retval None
  */
void PI_ObjectInit(CPI this)
{
 ((_CPI)this)->Vars_str.hKpGain = ((_CPI)this)->pParams_str->hDefKpGain;
 ((_CPI)this)->Vars_str.hKiGain = ((_CPI)this)->pParams_str->hDefKiGain;
 ((_CPI)this)->Vars_str.wIntegralTerm = 0;
 ((_CPI)this)->Vars_str.wUpperIntegralLimit = ((_CPI)this)->pParams_str->wDefMaxIntegralTerm; 
 ((_CPI)this)->Vars_str.wLowerIntegralLimit = ((_CPI)this)->pParams_str->wDefMinIntegralTerm; 
 ((_CPI)this)->Vars_str.hUpperOutputLimit = ((_CPI)this)->pParams_str->hDefMaxOutput;
 ((_CPI)this)->Vars_str.hLowerOutputLimit = ((_CPI)this)->pParams_str->hDefMinOutput;
 ((_CPI)this)->Vars_str.hKpDivisor = ((_CPI)this)->pParams_str->hKpDivisor;
 ((_CPI)this)->Vars_str.hKiDivisor = ((_CPI)this)->pParams_str->hKiDivisor;
 ((_CPI)this)->Vars_str.hKpDivisorPOW2 = ((_CPI)this)->pParams_str->hKpDivisorPOW2;
 ((_CPI)this)->Vars_str.hKiDivisorPOW2 = ((_CPI)this)->pParams_str->hKiDivisorPOW2;
  return;
}

/**
  * @brief  It updates the Kp gain 
  * @param  CPI PI object
  * @param  int16_t New Kp gain
  * @retval None
  */
void PI_SetKP(CPI this, int16_t hKpGain)
{
 ((_CPI)this)->Vars_str.hKpGain = hKpGain;
}

/**
  * @brief  It updates the Ki gain 
  * @param  CPI PI object
  * @param  int16_t New Ki gain
  * @retval None
  */
void PI_SetKI(CPI this, int16_t hKiGain)
{
 ((_CPI)this)->Vars_str.hKiGain = hKiGain;
}

/**
  * @brief  It returns the Kp gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
int16_t PI_GetKP(CPI this)
{
  return(((_CPI)this)->Vars_str.hKpGain);
}

/**
  * @brief  It returns the Ki gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
int16_t PI_GetKI(CPI this)
{
  return(((_CPI)this)->Vars_str.hKiGain);
}

/**
  * @brief  It returns the Default Kp gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
int16_t PI_GetDefaultKP(CPI this)
{
   return(((_CPI)this)->pParams_str->hDefKpGain);
}

/**
  * @brief  It returns the Default Ki gain of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
int16_t PI_GetDefaultKI(CPI this)
{
   return(((_CPI)this)->pParams_str->hDefKiGain);
}

/**
  * @brief  It set a new value into the PI integral term 
  * @param  CPI PI regulator object
  * @param  int32_t New integral term value
  * @retval None
  */
void PI_SetIntegralTerm(CPI this, int32_t wIntegralTermValue)
{
 ((_CPI)this)->Vars_str.wIntegralTerm = wIntegralTermValue;  

 return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
  * @brief  This function compute the output of a PI regulator sum of its 
  *         proportional and integral terms
  * @param  CPI PI regulator object
  * @param  int32_t Present process variable error, intended as the reference 
  *         value minus the present process variable value
  * @retval int16_t PI output
  */
int16_t PI_Controller(CPI this, int32_t wProcessVarError)
{
  int32_t wProportional_Term, wIntegral_Term, wOutput_32,wIntegral_sum_temp;
  int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = ((_CPI)this)->Vars_str.hUpperOutputLimit;
  int16_t hLowerOutputLimit = ((_CPI)this)->Vars_str.hLowerOutputLimit;
  
  /* Proportional term computation*/
  wProportional_Term = ((_CPI)this)->Vars_str.hKpGain * wProcessVarError;
  
  /* Integral term computation */
  if (((_CPI)this)->Vars_str.hKiGain == 0)
  {
    ((_CPI)this)->Vars_str.wIntegralTerm = 0;
  }
  else
  { 
    wIntegral_Term = ((_CPI)this)->Vars_str.hKiGain * wProcessVarError;
    wIntegral_sum_temp = ((_CPI)this)->Vars_str.wIntegralTerm + wIntegral_Term;
    
    if (wIntegral_sum_temp < 0)
    {
      if (((_CPI)this)->Vars_str.wIntegralTerm > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = S32_MAX;
        }
      }
    }
    else
    {
      if (((_CPI)this)->Vars_str.wIntegralTerm < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = -S32_MAX;
        }
      }
    }		

    if (wIntegral_sum_temp > ((_CPI)this)->Vars_str.wUpperIntegralLimit)
    {
      ((_CPI)this)->Vars_str.wIntegralTerm = ((_CPI)this)->Vars_str.
                                                            wUpperIntegralLimit;
    }
    else if (wIntegral_sum_temp < ((_CPI)this)->Vars_str.wLowerIntegralLimit)
    { 
      ((_CPI)this)->Vars_str.wIntegralTerm = ((_CPI)this)->Vars_str.
                                                            wLowerIntegralLimit;
    }
    else
    {
      ((_CPI)this)->Vars_str.wIntegralTerm = wIntegral_sum_temp;
    }
  }
  
#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = (wProportional_Term/(int32_t)((_CPI)this)->Vars_str.
                hKpDivisor)+(((_CPI)this)->Vars_str.wIntegralTerm/(int32_t)
                ((_CPI)this)->Vars_str.hKiDivisor);
#else    
 /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right) 
             is used by the compiler to perform the shifts (instead of LSR 
             logical shift right)*/ 
  wOutput_32 = (wProportional_Term >>((_CPI)this)->Vars_str.hKpDivisorPOW2) + 
  (((_CPI)this)->Vars_str.wIntegralTerm >> ((_CPI)this)->Vars_str.hKiDivisorPOW2);
#endif

  if (wOutput_32 > hUpperOutputLimit)
  {
    
    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;		  			 	
  }
  else if (wOutput_32 < hLowerOutputLimit)
  {
    
    wDischarge = hLowerOutputLimit - wOutput_32; 
    wOutput_32 = hLowerOutputLimit;
  }
  else
  {}
  
  ((_CPI)this)->Vars_str.wIntegralTerm += wDischarge;
  
  return((int16_t)(wOutput_32)); 	
}
/**
  * @brief  It returns the Kp gain divisor of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Kp gain 
  */
uint16_t PI_GetKPDivisor(CPI this)
{
  return (((_CPI)this)->Vars_str.hKpDivisor);
}

/**
  * @brief  It updates the Kp divisor
  * @param  CPI PI regulator object
  * @param  uint16_t New Kp divisor expressed as power of 2.
            E.g. if gain divisor is 512 the value 
            must be 9 because 2^9 = 512
  * @retval None
  */
void PI_SetKPDivisorPOW2(CPI this, uint16_t hKpDivisorPOW2)
{
  ((_CPI)this)->Vars_str.hKpDivisorPOW2 = hKpDivisorPOW2;
  ((_CPI)this)->Vars_str.hKpDivisor = ((uint16_t)(1u) << hKpDivisorPOW2);
}

/**
  * @brief  It returns the Ki gain divisor of the passed PI object
  * @param  CPI PI regulator object
  * @retval int16_t Ki gain 
  */
uint16_t PI_GetKIDivisor(CPI this)
{
  return (((_CPI)this)->Vars_str.hKiDivisor);  
}

/**
  * @brief  It updates the Ki divisor
  * @param  CPI PI regulator object
  * @param  uint16_t New Ki divisor expressed as power of 2.
            E.g. if gain divisor is 512 the value 
            must be 9 because 2^9 = 512
  * @retval None
  */
void PI_SetKIDivisorPOW2(CPI this, uint16_t hKiDivisorPOW2)
{
  int32_t wKiDiv = ((int32_t)(1u) << hKiDivisorPOW2);
  ((_CPI)this)->Vars_str.hKiDivisorPOW2 = hKiDivisorPOW2;
  ((_CPI)this)->Vars_str.hKiDivisor = (uint16_t)(wKiDiv);
  PI_SetUpperIntegralTermLimit(this, (int32_t)S16_MAX * wKiDiv);
  PI_SetLowerIntegralTermLimit(this, (int32_t)S16_MIN * wKiDiv);
}

/**
  * @brief  It set a new value for lower integral term limit  
  * @param  CPI PI regulator object
  * @param  int32_t New lower integral term limit value
  * @retval None
  */
void PI_SetLowerIntegralTermLimit(CPI this, int32_t wLowerLimit)
{
  ((_CPI)this)->Vars_str.wLowerIntegralLimit = wLowerLimit;
}

/**
  * @brief  It set a new value for upper integral term limit  
  * @param  CPI PI regulator object
  * @param  int32_t New upper integral term limit value
  * @retval None
  */
void PI_SetUpperIntegralTermLimit(CPI this, int32_t wUpperLimit)
{
  ((_CPI)this)->Vars_str.wUpperIntegralLimit = wUpperLimit;
}

/**
  * @brief  It set a new value for lower output limit  
  * @param  CPI PI regulator object
  * @param  int32_t New lower output limit value
  * @retval None
  */
void PI_SetLowerOutputLimit(CPI this, int16_t hLowerLimit)
{
  ((_CPI)this)->Vars_str.hLowerOutputLimit = hLowerLimit;
}

/**
  * @brief  It set a new value for upper output limit  
  * @param  CPI PI regulator object
  * @param  int32_t New upper output limit value
  * @retval None
  */
void PI_SetUpperOutputLimit(CPI this, int16_t hUpperLimit)
{
  ((_CPI)this)->Vars_str.hUpperOutputLimit = hUpperLimit;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
