/**
  ******************************************************************************
  * @file    PID_PIRegulatorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private implementation of PID_PI Class     
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
#include "PID_PIRegulatorClass.h"
#include "PID_PIRegulatorPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define DCLASS_VARS  ((_DCPID)(((_CPI) this)->DerivedClass))->DVars_str
#define DCLASS_PARAMS ((_DCPID)(((_CPI) this)->DerivedClass))->pDParams_str

/*#define MC_CLASS_DYNAMIC*/
#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCPID_t PID_PIpool[MAX_PID_PI_NUM];
	uint8_t PID_PI_Allocated = 0u;
#endif

/*#define DEBUG*/

#ifdef DEBUG
static volatile _CPI oPIdbg;
static volatile _DCPID oPIDdbg;
#endif

/**
  * @brief  Creates an object of the derived class PID
  * @param  pPIParams_t pointer to PI regulator class parameters structure
  * @param  pPIDParams_t pointer to a PID Derived class parameters structure
  * @retval CPID_PI new instance of derived class object
  */
CPID_PI PID_NewObject(pPIParams_t pPIParam,
                                  pPIDParams_t pPIDParam)
{
  _CPI _oPI;
  _DCPID _oPID;

  _oPI = (_CPI)PI_NewObject(pPIParam);
	#ifdef MC_CLASS_DYNAMIC
		_oPID = (_DCPID)calloc(1u,sizeof(_DCPID_t));
	#else
		if (PID_PI_Allocated  < MAX_PID_PI_NUM)
		{
			_oPID = &PID_PIpool[PID_PI_Allocated++];
		}
		else
		{
			_oPID = MC_NULL;
		}
	#endif
    
  _oPID->pDParams_str = pPIDParam;
  _oPI->DerivedClass = (void*)_oPID;
  
  return ((CPID_PI)_oPI);
}

/**
  * @brief  It initiliazes all the object variables, usually it has to be called 
  *         once right after object creation
  * @param  CPID_PI PID regulator object
  * @retval None
  */
void PID_ObjectInit(CPID_PI this)
{
    /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  
  
  ((_CPI)this)->Vars_str.hKpGain = ((_CPI)this)->pParams_str->hDefKpGain;
  
  ((_CPI)this)->Vars_str.hKiGain = ((_CPI)this)->pParams_str->hDefKiGain;
  ((_CPI)this)->Vars_str.wIntegralTerm = 0;
  ((_CPI)this)->Vars_str.wUpperIntegralLimit = ((_CPI)this)->pParams_str->
                                                            wDefMaxIntegralTerm; 
  ((_CPI)this)->Vars_str.wLowerIntegralLimit = ((_CPI)this)->pParams_str->
                                                            wDefMinIntegralTerm; 
  ((_CPI)this)->Vars_str.hUpperOutputLimit = ((_CPI)this)->pParams_str->
                                                                  hDefMaxOutput;
  ((_CPI)this)->Vars_str.hLowerOutputLimit = ((_CPI)this)->pParams_str->
                                                                  hDefMinOutput;  
  pDVars_str->hKdGain = DCLASS_PARAMS->hDefKdGain;
                      
  pDVars_str->wPrevProcessVarError = 0;                  
    
  return;
}


    
/**
  * @brief  It set a new value into the PID Previous error variable required to 
  *         compute derivative term
  * @param  CPID_PI PID regulator object
  * @param  int32_t New integral term value
  * @retval None
  */
void PID_SetPrevError(CPID_PI this, int32_t wPrevProcessVarError)
{
 ((_DCPID)(((_CPI)this)->DerivedClass))->DVars_str.wPrevProcessVarError = 
                                                           wPrevProcessVarError;
  return;
}


/**
  * @brief  It updates the Kd gain 
  * @param  CPID_PI PID regulator object
  * @param  int16_t New Kd gain
  * @retval None
*/
void PID_SetKD(CPID_PI  this, int16_t hKdGain)
{
  if(((_DCPID)(((_CPI)this)->DerivedClass)) != MC_NULL)
  {
  ((_DCPID)(((_CPI)this)->DerivedClass))->DVars_str.hKdGain = hKdGain;
  }
}

/**
  * @brief  It returns the Kd gain structure of the PID object passed
  * @param  CPID_PI PI regulator object
  * @retval int16_t Kd gain
  */
int16_t PID_GetKD(CPID_PI this)
{ 
  int16_t haux;
  if(((_DCPID)(((_CPI)this)->DerivedClass)) != MC_NULL)
  {
    haux = ((_DCPID)(((_CPI)this)->DerivedClass))->DVars_str.hKdGain;
  }
  else
  {
    haux= (int16_t) 0;
  }
  return (haux);
}


/**
  * @brief  It returns the Kd gain divisor of the PID object passed 
  * @param  CPID_PI PID regulator object
  * @retval int16_t Kd gain
  */
uint16_t PID_GetKDDivisor(CPID_PI this)
{
  uint16_t hAux = 0u;
  
  if(((_DCPID)(((_CPI)this)->DerivedClass)) != MC_NULL)
  {
    hAux = ((_DCPID)(((_CPI)this)->DerivedClass))->pDParams_str->hKdDivisor;
  }   
  return (hAux); 
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  This function compute the output of a PID regulator sum of its 
*         proportional, integral and derivative terms
* @param  CPID_PI PID regulator object
* @param  int32_t Present process variable error, intended as the 
*         reference value minus the present process variable value
* @retval int16_t PI output
*/
int16_t PID_Controller(CPID_PI  this, int32_t wProcessVarError)
{
  int32_t wDifferential_Term, wDeltaError, wTemp_output;
  
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  
  
  wDeltaError = wProcessVarError - pDVars_str->wPrevProcessVarError;
  
  wDifferential_Term = pDVars_str->hKdGain * wDeltaError; 
  
#ifdef FULL_MISRA_C_COMPLIANCY
  wDifferential_Term /= (int32_t)((_DCPID)(((_CPI)this)->DerivedClass))->
    pDParams_str->hKdDivisor;
#else  
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) 
  is used by the compiler to perform the shifts (instead of LSR 
  logical shift right)*/ 
  wDifferential_Term >>= 
    (int32_t)((_DCPID)(((_CPI)this)->DerivedClass))->
      pDParams_str->hKdDivisorPOW2;
#endif
  
  pDVars_str->wPrevProcessVarError = wProcessVarError; 
  
  wTemp_output = PI_Controller((CPI)this, wProcessVarError) + wDifferential_Term;
  
  if (wTemp_output > ((_CPI)this)->Vars_str.hUpperOutputLimit)
  {
    wTemp_output = ((_CPI)this)->Vars_str.hUpperOutputLimit;		  			 	
  }
  else if (wTemp_output < ((_CPI)this)->Vars_str.hLowerOutputLimit)
  {
    wTemp_output = ((_CPI)this)->Vars_str.hLowerOutputLimit;
  }
  else
  {}
  
  return((int16_t) wTemp_output); 		
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
