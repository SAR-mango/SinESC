/**
  ******************************************************************************
  * @file    Rdivider_BusVoltageSensorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains private impelementation of Rdivider class      
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
#include "Rdivider_BusVoltageSensorClass.h"
#include "Rdivider_F30X_BusVoltageSensorPrivate.h"
#include "MCLibraryConf.h"
#include "MC_type.h"

#define DCLASS_PARAMS ((_CRVBS_VBS)(((_CVBS) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_CRVBS_VBS)(((_CVBS) this)->DerivedClass))->DVars_str

#define CLASS_PARAMS ((_CVBS)this)->pParams_str
#define CLASS_VARS   ((_CVBS)this)->Vars_str

#ifdef MC_CLASS_DYNAMIC
	#include "stdlib.h" /* Used for dynamic allocation */
#else
	_CRVBS_VBS_t RVBS_VBSpool[MAX_RVBS_VBS_NUM];
	unsigned char RVBS_VBS_Allocated = 0u;
#endif

static void RVBS_Init(CVBS this, CPWMC oPWMnCurrentSensor);
static void RVBS_Clear(CVBS this);
static uint16_t RVBS_CalcAvVbus(CVBS this);
static uint16_t RVBS_SetFaultState(CVBS this);

/**
  * @brief  Creates an object of the class Rdivider
  * @param  pBusVoltageSensorParams pointer to an BusVoltageSensor parameters structure
  * @param  pRdividerParams pointer to an Rdivider parameters structure
  * @retval CRVBS_VBS new instance of Rdivider object
  */
CRVBS_VBS RVBS_NewObject(pBusVoltageSensorParams_t pBusVoltageSensorParams, pRdividerParams_t pRdividerParams)
{
	_CVBS _oBusVoltageSensor;
	_CRVBS_VBS _oRdivider;

	_oBusVoltageSensor = (_CVBS)VBS_NewObject(pBusVoltageSensorParams);

	#ifdef MC_CLASS_DYNAMIC
		_oRdivider = (_CRVBS_VBS)calloc(1u,sizeof(_CRVBS_VBS_t));
	#else
		if (RVBS_VBS_Allocated  < MAX_RVBS_VBS_NUM)
		{
			_oRdivider = &RVBS_VBSpool[RVBS_VBS_Allocated++];
		}
		else
		{
			_oRdivider = MC_NULL;
		}
	#endif
  
	_oRdivider->pDParams_str = pRdividerParams;
	_oBusVoltageSensor->DerivedClass = (void*)_oRdivider;
        
	_oBusVoltageSensor->Methods_str.pVBS_Init = &RVBS_Init;
	_oBusVoltageSensor->Methods_str.pVBS_Clear = &RVBS_Clear;
        _oBusVoltageSensor->Methods_str.pVBS_CalcAvVbus = &RVBS_CalcAvVbus;

	return ((CRVBS_VBS)_oBusVoltageSensor);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup BusVoltageSensor_Rdivider
  * @{
  */

/** @defgroup Rdivider_class_private_methods Rdivider class private methods
* @{
*/

/**
  * @brief  It initializes bus voltage conversion (ADC channel, conversion time, 
  *         GPIO port and pin). It must be called only after PWMC_Init. 
  * @param  this related object of class CVBS
  * @retval none
  */
static void RVBS_Init(CVBS this, CPWMC oPWMnCurrentSensor)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADConv_t ADConv_struct; 
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
    /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hVbusPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(pDParams_str->hVbusPort, &GPIO_InitStructure);  
  
  DCLASS_VARS.oPWMnCurrentSensor = oPWMnCurrentSensor;
  /* Configure AD chaneel sampling time */
  ADConv_struct.Channel = pDParams_str->bVbusADChannel;
  ADConv_struct.SamplTime = pDParams_str->bVbusSamplingTime;
  PWMC_ADC_SetSamplingTime(oPWMnCurrentSensor, ADConv_struct);
  RVBS_Clear(this);
  
  if (pDParams_str->hLowPassFilterBW > BUFF_MAX)
  {
    while (1); /* Error VBUS_SW_FILTER_BW_FACTOR > BUFF_MAX */
  }
}


/**
  * @brief  It clears bus voltage FW variable containing average bus voltage 
  *         value
  * @param  this related object of class CVBS
  * @retval none
  */
static void RVBS_Clear(CVBS this)
{
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  
  pVars_str->hAvBusVoltage_d = ((pDParams_str->hOverVoltageThreshold) + 
                                    (pDParams_str->hUnderVoltageThreshold))/2u;
  pDVars_str->vindex = 0u;
  pDVars_str->nelem = 0u;
}

static uint16_t RVBS_ConvertVbusFiltrered(CVBS this)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  uint16_t hAux;
  uint8_t vindex;
  uint16_t max,min;
  uint32_t tot = 0u;
  uint16_t hTotElement = pDParams_str->hLowPassFilterBW;
  
  for (vindex = 0; vindex < hTotElement; )
  {
    hAux = PWMC_ExecRegularConv(pDVars_str->oPWMnCurrentSensor,
                              pDParams_str->bVbusADChannel);
    
    if(hAux != 0xFFFFu)
    {
      if (vindex == 0)
      {
        min = hAux;
        max = hAux;
      }
      else
      {
        if (hAux < min)
        {
          min = hAux;
        }
        if (hAux > max)
        {
          max = hAux;
        }
      }
      vindex++;
      
      tot += hAux;
    }
  }
  
  tot -= max;
  tot -= min;
  return (uint16_t)(tot / (hTotElement-2u));
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
static uint16_t RVBS_CalcAvVbus(CVBS this)
{
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  uint8_t i;
  uint16_t max,min,mean;
  uint32_t tot = 0u;
  uint16_t hAux;
  uint16_t hTotElement = pDParams_str->hLowPassFilterBW;
  
  hAux = RVBS_ConvertVbusFiltrered(this);
  pVars_str->hLatestConv = hAux;
  
  pDVars_str->vbuffer[pDVars_str->vindex] = hAux;
  mean = hAux;
  pDVars_str->vindex++;
  if (pDVars_str->vindex >= hTotElement)
  {
    pDVars_str->vindex = 0u;
  }
  if (pDVars_str->nelem < hTotElement)
  {
    pDVars_str->nelem++;
  }
  if (pDVars_str->nelem > 2u)
  {
    min = pDVars_str->vbuffer[0];
    max = pDVars_str->vbuffer[0];
    for (i = 0u; i < pDVars_str->nelem; i++)
    {
      uint16_t hElem = pDVars_str->vbuffer[i];
      tot += hElem;
      if (hElem < min)
      {
        min = hElem;
      }
      if (hElem > max)
      {
        max = hElem;
      }
    }
    tot -= max;
    tot -= min;
    mean = (uint16_t)(tot / (pDVars_str->nelem-2u));
  }
  pVars_str->hAvBusVoltage_d = mean;
  
  pVars_str->hFaultState = RVBS_SetFaultState(this);
  
  return(pVars_str->hFaultState);
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
static uint16_t RVBS_SetFaultState(CVBS this)
{
  uint16_t hFault;
  pDParams_t pDParams_str = DCLASS_PARAMS;  
  pVars_t pVars_str = &CLASS_VARS;

  if(pVars_str->hAvBusVoltage_d > pDParams_str->hOverVoltageThreshold)
    {
      hFault = MC_OVER_VOLT;
    }
    else if (pVars_str->hAvBusVoltage_d < pDParams_str->hUnderVoltageThreshold)
    {
      hFault = MC_UNDER_VOLT;    
    }
    else
    {
      hFault = MC_NO_ERROR;    
    }  
  return hFault;
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
