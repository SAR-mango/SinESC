/**
  ******************************************************************************
  * @file    Virtual_TemperatureSensorClass.c
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
#include "TemperatureSensorClass.h"
#include "TemperatureSensorPrivate.h"
#include "Virtual_TemperatureSensorClass.h"
#include "Virtual_TemperatureSensorPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define DCLASS_PARAMS ((_CVTS_TSNS)(((_CTSNS) this)->DerivedClass))->pDParams_str
#define BCLASS_VARS   ((_CTSNS)this)->Vars_str
#define DCLASS_VARS   ((_CVTS_TSNS)(((_CTSNS) this)->DerivedClass))->DVars_str

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_CVTS_TSNS_t VTS_TSNSpool[MAX_VTS_TSNS_NUM];
	unsigned char VTS_TSNS_Allocated = 0u;
#endif


static void VTS_Init(CTSNS this, CPWMC oPWMnCurrentSensor);
static void VTS_Clear(CTSNS this);
static uint16_t VTS_NoErrors(CTSNS this);
static int16_t VTS_GetAvTemp_C(CTSNS this);
/**
  * @brief  Creates an object of the class Virtual Temperature sensor
  * @param  pTempSensorParams pointer to an TemperatureSensor parameters 
  *         structure
  * @param  pVirtualParams pointer to an Virtual Temperature sensor parameters structure
  * @retval CVTS_TSNS new instance of Virtual Temperature sensor object
  */
CVTS_TSNS VTS_NewObject(pTempSensorParams_t pTempSensorParams, pVirtualTParams_t pVirtualParams)
{
	_CTSNS _oTempSensor;
	_CVTS_TSNS _oVirtual;

	_oTempSensor = (_CTSNS)TSNS_NewObject(pTempSensorParams);

	#ifdef MC_CLASS_DYNAMIC
		_oVirtual = (_CVTS_TSNS)calloc(1u,sizeof(_CVTS_TSNS_t));
	#else
		if (VTS_TSNS_Allocated  < MAX_VTS_TSNS_NUM)
		{
			_oVirtual = &VTS_TSNSpool[VTS_TSNS_Allocated++];
		}
		else
		{
			_oVirtual = MC_NULL;
		}
	#endif
  
	_oVirtual->pDParams_str = pVirtualParams;
	_oTempSensor->DerivedClass = (void*)_oVirtual;
        
	_oTempSensor->Methods_str.pTSNS_Init = &VTS_Init;
	_oTempSensor->Methods_str.pTSNS_Clear = &VTS_Clear;
        _oTempSensor->Methods_str.pTSNS_CalcAvTemp = &VTS_NoErrors;
        _oTempSensor->Methods_str.pTSNS_GetAvTemp_C = &VTS_GetAvTemp_C;
	return ((CVTS_TSNS)_oTempSensor);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup TemperatureSensor_Virtual
  * @{
  */

/** @defgroup Virtual_class_private_methods Virtual Temperature sensor class private methods
* @{
*/

/**
  * @brief  It initializes virtual temperature sensor
  * @param  this related object of class CTSNS
  * @retval none
  */
static void VTS_Init(CTSNS this, CPWMC oPWMnCurrentSensor)
{
  pVars_t pVars_str = &BCLASS_VARS;
  
  pVars_str->hFaultState = MC_NO_ERROR;
  pVars_str->hAvTemp_d = DCLASS_PARAMS->hExpectedTemp_d;
}

/**
  * @brief  It simply returns in virtual temperature sensor implementation
  * @param  this related object of class CTSNS
  * @retval none
  */
static void VTS_Clear(CTSNS this)
{
  return;
}

/**
  * @brief  It returns MC_NO_ERROR in virtual temperature implementation
  * @param  this related object of class CTSNS
* @retval uint16_t Fault code error: MC_NO_ERROR
  */
static uint16_t VTS_NoErrors(CTSNS this)
{
  return(MC_NO_ERROR);
}

/**
  * @brief  It returns expected temperature measurement expressed in 
  *         Celsius degrees
  * @param  this related object of class CTSNS
  * @retval int16_t Expected temperature measurement expressed in Celsius degrees
  */
static int16_t VTS_GetAvTemp_C(CTSNS this)
  {
  return(DCLASS_PARAMS->hExpectedTemp_C);
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
