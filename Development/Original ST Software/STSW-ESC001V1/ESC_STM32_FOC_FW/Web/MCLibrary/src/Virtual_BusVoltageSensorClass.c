/**
  ******************************************************************************
  * @file    Virtual_BusVoltageSensorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of Virtual Vbus sensor 
  *          class      
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
#include "BusVoltageSensorClass.h"
#include "BusVoltageSensorPrivate.h"
#include "Virtual_BusVoltageSensorClass.h"
#include "Virtual_BusVoltageSensorPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define DCLASS_PARAMS ((_CVVBS_VBS)(((_CVBS) this)->DerivedClass))->pDParams_str
#define BCLASS_VARS   ((_CVBS)this)->Vars_str

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_CVVBS_VBS_t VVBS_VBSpool[MAX_VVBS_VBS_NUM];
	unsigned char VVBS_VBS_Allocated = 0u;
#endif


static void VVBS_Init(CVBS this, CPWMC oPWMnCurrentSensor);
static void VVBS_Clear(CVBS this);
static uint16_t VVBS_NoErrors(CVBS this);

/**
  * @brief  Creates an object of the class Virtual Vbus sensor
  * @param  pBusVoltageSensorParams pointer to an BusVoltageSensor parameters 
  *         structure
  * @param  pVirtualParams pointer to an Virtual Vbus sensor parameters structure
  * @retval CVVBS_VBS new instance of Virtual Vbus sensor object
  */
CVVBS_VBS VVBS_NewObject(pBusVoltageSensorParams_t pBusVoltageSensorParams, pVirtualParams_t pVirtualParams)
{
	_CVBS _oBusVoltageSensor;
	_CVVBS_VBS _oVirtual;

	_oBusVoltageSensor = (_CVBS)VBS_NewObject(pBusVoltageSensorParams);

	#ifdef MC_CLASS_DYNAMIC
		_oVirtual = (_CVVBS_VBS)calloc(1u,sizeof(_CVVBS_VBS_t));
	#else
		if (VVBS_VBS_Allocated  < MAX_VVBS_VBS_NUM)
		{
			_oVirtual = &VVBS_VBSpool[VVBS_VBS_Allocated++];
		}
		else
		{
			_oVirtual = MC_NULL;
		}
	#endif
  
	_oVirtual->pDParams_str = pVirtualParams;
	_oBusVoltageSensor->DerivedClass = (void*)_oVirtual;
        
	_oBusVoltageSensor->Methods_str.pVBS_Init = &VVBS_Init;
	_oBusVoltageSensor->Methods_str.pVBS_Clear = &VVBS_Clear;
        _oBusVoltageSensor->Methods_str.pVBS_CalcAvVbus = &VVBS_NoErrors;
	return ((CVVBS_VBS)_oBusVoltageSensor);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup BusVoltageSensor_Virtual
  * @{
  */

/** @defgroup Virtual_class_private_methods Virtual Vbus sensor class private methods
* @{
*/

/**
  * @brief  It initializes bus voltage conversion for virtual bus voltage sensor
  * @param  this related object of class CVBS
  * @retval none
  */
static void VVBS_Init(CVBS this, CPWMC oPWMnCurrentSensor)
{
  pVars_t pVars_str = &BCLASS_VARS;
  
  pVars_str->hFaultState = MC_NO_ERROR;
  pVars_str->hLatestConv = DCLASS_PARAMS->hExpectedVbus_d;
  pVars_str->hAvBusVoltage_d = DCLASS_PARAMS->hExpectedVbus_d;
}

/**
  * @brief  It simply returns in virtual Vbus sensor implementation
  * @param  this related object of class CVBS
  * @retval none
  */
static void VVBS_Clear(CVBS this)
{
  return;
}

/**
  * @brief  It returns MC_NO_ERROR
  * @param  this related object of class CVBS
* @retval uint16_t Fault code error: MC_NO_ERROR
  */
static uint16_t VVBS_NoErrors(CVBS this)
{
  return(MC_NO_ERROR);
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
