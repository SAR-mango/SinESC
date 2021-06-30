/**
  ******************************************************************************
  * @file    BusVoltageSensorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of Bus Voltage Sensor class      
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
#include "MCLibraryConf.h"
#include "MC_type.h"

#define BCLASS_PARAMS ((_CVBS)this)->pParams_str
#define BCLASS_VARS   ((_CVBS)this)->Vars_str

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_CVBS_t VBSpool[MAX_VBS_NUM];
unsigned char VBS_Allocated = 0u;
#endif

/**
* @brief  Creates an object of the class BusVoltageSensor
* @param  pBusVoltageSensorParams pointer to an BusVoltageSensor parameters structure
* @retval CVBS new instance of BusVoltageSensor object
*/
  CVBS VBS_NewObject(pBusVoltageSensorParams_t pBusVoltageSensorParams)
  {
    _CVBS _oVBS;
    
#ifdef MC_CLASS_DYNAMIC
    _oVBS = (_CVBS)calloc(1u,sizeof(_CVBS_t));
#else
    if (VBS_Allocated  < MAX_VBS_NUM)
    {
      _oVBS = &VBSpool[VBS_Allocated++];
    }
    else
    {
      _oVBS = MC_NULL;
    }
#endif
    
    _oVBS->pParams_str = (pParams_t)pBusVoltageSensorParams;
    
    return ((CVBS)_oVBS);
  }
  
/**
  * @brief  It initializes bus voltage conversion. It must be called only after 
  *         current sensor initialization (PWMC_Init)
  * @param  this related object of class CVBS
  * @retval none
  */
  void VBS_Init(CVBS this, CPWMC oPWMnCurrentSensor)
  { 
    (((_CVBS)this)->Methods_str.pVBS_Init)(this, oPWMnCurrentSensor);
  }
  
  
/**
  * @brief  It clears bus voltage FW variable containing average bus voltage 
  *         value
  * @param  this related object of class CVBS
  * @retval none
  */
  void VBS_Clear(CVBS this)
  {
    (((_CVBS)this)->Methods_str.pVBS_Clear)(this);
  }
  
/**
  * @brief  It clocks the bus voltage reading, performes Vbus conversion 
  *         and updates the average 
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
  uint16_t VBS_CalcAvVbus(CVBS this)
  {
    return((((_CVBS)this)->Methods_str.pVBS_CalcAvVbus)(this));
  }
  
/**
  * @brief  It return latest Vbus conversion result expressed in u16Volt
  * @param  this related object of class CVBS
  * @retval uint16_t Latest Vbus conversion result in digit
  */
  uint16_t VBS_GetBusVoltage_d(CVBS this)
  {
    return(BCLASS_VARS.hLatestConv);
  }

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#endif
#endif  
/**
  * @brief  It return latest averaged Vbus measurement expressed in u16Volt
  * @param  this related object of class CVBS
  * @retval uint16_t Latest averaged Vbus measurement in digit
  */
  uint16_t VBS_GetAvBusVoltage_d(CVBS this)
  {
    return(BCLASS_VARS.hAvBusVoltage_d);
  }
  
/**
  * @brief  It return latest averaged Vbus measurement expressed in Volts
  * @param  this related object of class CVBS
  * @retval uint16_t Latest averaged Vbus measurement in Volts
  */
  uint16_t VBS_GetAvBusVoltage_V(CVBS this)
  {
  uint32_t wTemp;
  
  wTemp = (uint32_t)(BCLASS_VARS.hAvBusVoltage_d);
  wTemp *= BCLASS_PARAMS->hConversionFactor;
  wTemp /= 65536u;
  
  return((uint16_t)wTemp);
  }
  
/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
  uint16_t VBS_CheckVbus(CVBS this)
  {
    return(BCLASS_VARS.hFaultState);
  }
  
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
