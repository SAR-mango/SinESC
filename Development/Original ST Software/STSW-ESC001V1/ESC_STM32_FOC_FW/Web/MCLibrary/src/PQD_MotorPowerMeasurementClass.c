/**
  ******************************************************************************
  * @file    PQD_MotorPowerMeasurementClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of PQD class      
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
#include "PQD_MotorPowerMeasurementClass.h"
#include "PQD_MotorPowerMeasurementPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_DCPQD_MPM_t PQD_MPMpool[MAX_PQD_MPM_NUM];
	unsigned char PQD_MPM_Allocated = 0u;
#endif

#define DCLASS_PARAM ((_DCPQD_MPM)(((_CMPM) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCPQD_MPM)(((_CMPM) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CMPM)this)->Vars_str)
#define  CLASS_PARAM (((_CMPM)this)->pParams_str)

/**
  * @brief  Implementation of derived class init method.
  * @param this related object of class CMPM.
  * @param pMPMInitStruct the pointer of the init structure, required by derived class, up-casted to pMPMInitStruct_t.
  * @retval none.
  */
static void PQD_Init(CMPM this, pMPMInitStruct_t pMPMInitStruct);

/**
  * @brief Implementation of derived class init method. It should be called before each motor restart.
  * @param this related object of class CMPM.
  * @retval none.
  */
static void PQD_Clear(CMPM this);

/**
  * @brief Implementation of derived class CalcElMotorPower.
  * @param this related object of class CMPM.
  * @retval int16_t The measured motor power expressed in watt.
  */
static int16_t PQD_CalcElMotorPower(CMPM this);

/**
  * @brief  Creates an object of the class PQD
  * @param  pMotorPowerMeasurementParams pointer to an MotorPowerMeasurement parameters structure
  * @param  pPQDParams pointer to an PQD parameters structure
  * @retval CPQD_MPM new instance of PQD object
  */
CPQD_MPM PQD_NewObject(pPQDParams_t pPQDParams)
{
	_CMPM _oMotorPowerMeasurement;
	_DCPQD_MPM _oPQD;

	_oMotorPowerMeasurement = (_CMPM)MPM_NewObject();

	#ifdef MC_CLASS_DYNAMIC
		_oPQD = (_DCPQD_MPM)calloc(1u,sizeof(_DCPQD_MPM_t));
	#else
		if (PQD_MPM_Allocated  < MAX_PQD_MPM_NUM)
		{
			_oPQD = &PQD_MPMpool[PQD_MPM_Allocated++];
		}
		else
		{
			_oPQD = MC_NULL;
		}
	#endif
  
	_oPQD->pDParams_str = pPQDParams;
	_oMotorPowerMeasurement->DerivedClass = (void*)_oPQD;
  
	_oMotorPowerMeasurement->Methods_str.pPQD_Init = &PQD_Init;
        _oMotorPowerMeasurement->Methods_str.pPQD_Clear = &PQD_Clear;
        _oMotorPowerMeasurement->Methods_str.pPQD_CalcElMotorPower = &PQD_CalcElMotorPower;

	return ((CPQD_MPM)_oMotorPowerMeasurement);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup MotorPowerMeasurement_PQD
  * @{
  */

/** @defgroup PQD_class_private_methods PQD class private methods
* @{
*/

/**
  * @brief  Implementation of derived class init method.
  * @param this related object of class CMPM.
  * @param pMPMInitStruct the pointer of the init structure, required by derived class, up-casted to pMPMInitStruct_t.
  * @retval none.
  */
static void PQD_Init(CMPM this, pMPMInitStruct_t pMPMInitStruct)
{
  pDVars_t pDVars = DCLASS_VARS;
  pPQD_MPMInitStruct_t pPQD_MPMInitStruct = (pPQD_MPMInitStruct_t)pMPMInitStruct;
  
  pDVars->pFOCVars = pPQD_MPMInitStruct->pFOCVars;
  pDVars->oVBS = pPQD_MPMInitStruct->oVBS;
}

/**
  * @brief Implementation of derived class init method. It should be called before each motor restart.
  * @param this related object of class CMPM.
  * @retval none.
  */
static void PQD_Clear(CMPM this)
{
}

/**
  * @brief Implementation of derived class CalcElMotorPower.
  * @param this related object of class CMPM.
  * @retval int16_t The measured motor power expressed in watt.
  */
static int16_t PQD_CalcElMotorPower(CMPM this)
{
  int32_t wAux,wAux2,wAux3;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  Curr_Components Iqd = pDVars->pFOCVars->Iqd;
  Volt_Components Vqd = pDVars->pFOCVars->Vqd;
  wAux = ((int32_t)Iqd.qI_Component1 * (int32_t)Vqd.qV_Component1) + 
    ((int32_t)Iqd.qI_Component2 * (int32_t)Vqd.qV_Component2);
  wAux /= 65536;
    
  wAux2 = pDParams->wConvFact * (int32_t)VBS_GetAvBusVoltage_V(pDVars->oVBS);
  wAux2 /= 600; /* 600 is max bus voltage expressed in volt.*/
  
  wAux3 = wAux * wAux2;
  wAux3 *= 6; /* 6 is max bus voltage expressed in thousend of volt.*/
  wAux3 /= 10;
  wAux3 /= 65536;
    
  return (int16_t)(wAux3);
}

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
