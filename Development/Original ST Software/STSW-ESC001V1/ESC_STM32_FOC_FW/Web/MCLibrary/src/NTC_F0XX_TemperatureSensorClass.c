/**
  ******************************************************************************
  * @file    NTC_F0XX_TemperatureSensorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of NTC class for F0XX
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
#include "NTC_TemperatureSensorClass.h"
#include "NTC_TemperatureSensorPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define DCLASS_PARAMS ((_CNTC_TSNS)(((_CTSNS) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_CNTC_TSNS)(((_CTSNS) this)->DerivedClass))->DVars_str

#define BCLASS_PARAMS ((_CTSNS)this)->pParams_str
#define BCLASS_VARS   ((_CTSNS)this)->Vars_str

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else

	

	_CNTC_TSNS_t NTC_TSNSpool[MAX_NTC_TSNS_NUM];
	unsigned char NTC_TSNS_Allocated = 0u;
#endif


static void NTC_Init(CTSNS this, CPWMC oPWMnCurrentSensor);
static void NTC_Clear(CTSNS this);
static uint16_t NTC_CalcAvVbus(CTSNS this);
static uint16_t NTC_SetFaultState(CTSNS this);
static int16_t NTC_GetAvTemp_C(CTSNS this);

/**
  * @brief  Creates an object of the class NTC
  * @param  pTempSensorParams pointer to an TempSensor parameters structure
  * @param  pNTCParams pointer to an NTC parameters structure
  * @retval CNTC_TSNS new instance of NTC object
  */
CNTC_TSNS NTC_NewObject(pTempSensorParams_t pTempSensorParams, pNTCParams_t pNTCParams)
{
	_CTSNS _oTempSensor;
	_CNTC_TSNS _oNTC;

	_oTempSensor = (_CTSNS)TSNS_NewObject(pTempSensorParams);

	#ifdef MC_CLASS_DYNAMIC
		_oNTC = (_CNTC_TSNS)calloc(1u,sizeof(_CNTC_TSNS_t));
	#else
		if (NTC_TSNS_Allocated  < MAX_NTC_TSNS_NUM)
		{
			_oNTC = &NTC_TSNSpool[NTC_TSNS_Allocated++];
		}
		else
		{
			_oNTC = MC_NULL;
		}
	#endif
  
	_oNTC->pDParams_str = pNTCParams;
	_oTempSensor->DerivedClass = (void*)_oNTC;
        
	_oTempSensor->Methods_str.pTSNS_Init = &NTC_Init;
	_oTempSensor->Methods_str.pTSNS_Clear = &NTC_Clear;
        _oTempSensor->Methods_str.pTSNS_CalcAvTemp = &NTC_CalcAvVbus;
        _oTempSensor->Methods_str.pTSNS_GetAvTemp_C = &NTC_GetAvTemp_C;
	return ((CNTC_TSNS)_oTempSensor);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup TemperatureSensor_NTC
  * @{
  */

/** @defgroup NTC_class_private_methods NTC class private methods
* @{
*/

/**
  * @brief  It initializes temperature conversion (ADC channel, conversion time, 
  *         GPIO port and pin). It must be called only after PWMC_Init. 
  * @param  this related object of class TSNS
  * @retval none
  */
static void NTC_Init(CTSNS this, CPWMC oPWMnCurrentSensor)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADConv_t ADConv_struct; 
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
    /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hTsensPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(pDParams_str->hTsensPort, &GPIO_InitStructure);  
  
  DCLASS_VARS.oPWMnCurrentSensor = oPWMnCurrentSensor;
  /* Configure AD chaneel sampling time */
  ADConv_struct.Channel = pDParams_str->bTsensADChannel;
  ADConv_struct.SamplTime = pDParams_str->bTsensSamplingTime;
  PWMC_ADC_SetSamplingTime(oPWMnCurrentSensor, ADConv_struct);
  NTC_Clear(this);
}


/**
  * @brief  It clears temperature average value
  * @param  this related object of class CTSNS
  * @retval none
  */
static void NTC_Clear(CTSNS this)
{
  BCLASS_VARS.hAvTemp_d = 0u;
}

/**
  * @brief  It actually performes the temperature sensing ADC conversion and 
  *         updates average value
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
static uint16_t NTC_CalcAvVbus(CTSNS this)
{
  uint32_t wtemp;
  uint16_t hAux;
  pDParams_t pDParams_str = DCLASS_PARAMS; 
  pVars_t pVars_str = &BCLASS_VARS;
    
  hAux = PWMC_ExecRegularConv(DCLASS_VARS.oPWMnCurrentSensor,
                                                pDParams_str->bTsensADChannel);
  
  if(hAux != 0xFFFFu)
  {
    wtemp =  (uint32_t)(pDParams_str->hLowPassFilterBW)-1u;
    wtemp *= (uint32_t) (pVars_str->hAvTemp_d);
    wtemp += hAux;
    wtemp /= (uint32_t)(pDParams_str->hLowPassFilterBW);
    
    pVars_str->hAvTemp_d = (uint16_t) wtemp;
  }
  
  pVars_str->hFaultState = NTC_SetFaultState(this);
  
  return(pVars_str->hFaultState);
}

/**
  * @brief  It returns MC_OVER_TEMP or MC_NO_ERROR depending on
  *         temperature sensor output and protection threshold values
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
static uint16_t NTC_SetFaultState(CTSNS this)
{
  uint16_t hFault;
  pDParams_t pDParams_str = DCLASS_PARAMS;  
  pVars_t pVars_str = &BCLASS_VARS;

  if(pVars_str->hAvTemp_d > pDParams_str->hOverTempThreshold)
    {
      hFault = MC_OVER_TEMP;
    }
    else if (pVars_str->hAvTemp_d < pDParams_str->hOverTempDeactThreshold)
    {
      hFault = MC_NO_ERROR;    
    }
    else
    {
      
      hFault = pVars_str->hFaultState;    
    }  
  return hFault;
}

/**
  * @brief  It returns latest averaged temperature measurement expressed in 
  *         Celsius degrees
  * @param  this related object of class CTSNS
  * @retval uint16_t Latest averaged temperature measurement in Celsius degrees
  */
static int16_t NTC_GetAvTemp_C(CTSNS this)
  {
  int32_t wTemp;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
  wTemp = (int32_t)(BCLASS_VARS.hAvTemp_d);
  wTemp -= (int32_t)(pDParams_str->wV0);
  wTemp *= pDParams_str->hSensitivity;
  wTemp = wTemp/65536 + (int32_t)(pDParams_str->hT0);
  
  return((int16_t)wTemp);
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
