/**
  ******************************************************************************
  * @file    TemperatureSensorClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of PI Regulator class      
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
#include "MCLibraryConf.h"
#include "MC_type.h"

#define BCLASS_PARAMS ((_CTSNS)this)->pParams_str
#define BCLASS_VARS   ((_CTSNS)this)->Vars_str

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_CTSNS_t TSNSpool[MAX_TSNS_NUM];
unsigned char TSNS_Allocated = 0u;
#endif

/**
* @brief  Creates an object of the class TemperatureSensor
* @param  pTempSensorParams pointer to an TempSensor parameters structure
* @retval CTSNS new instance of TempSensor object
*/
  CTSNS TSNS_NewObject(pTempSensorParams_t pTempSensorParams)
  {
    _CTSNS _oTSNS;
    
#ifdef MC_CLASS_DYNAMIC
    _oTSNS = (_CTSNS)calloc(1u,sizeof(_CTSNS_t));
#else
    if (TSNS_Allocated  < MAX_TSNS_NUM)
    {
      _oTSNS = &TSNSpool[TSNS_Allocated++];
    }
    else
    {
      _oTSNS = MC_NULL;
    }
#endif
    
    _oTSNS->pParams_str = (pParams_t)pTempSensorParams;
    
    return ((CTSNS)_oTSNS);
  }
  
/**
  * @brief  It initializes temperature sensing conversions. It must be called 
  *         only after current sensor initialization (PWMC_Init)
  * @param  this related object of class CTSNS
  * @param  oPWMnCurrentSensor CPWMC object to be used for regular conversions
  * @retval none
  */
  void TSNS_Init(CTSNS this, CPWMC oPWMnCurrentSensor)
  { 
    (((_CTSNS)this)->Methods_str.pTSNS_Init)(this, oPWMnCurrentSensor);
  }
  
  
/**
  * @brief  It clears FW variable containing average temperature measurement 
  *         value
  * @param  this related object of class CTSNS
  * @retval none
  */
  void TSNS_Clear(CTSNS this)
  {
    (((_CTSNS)this)->Methods_str.pTSNS_Clear)(this);
  }
  
/**
  * @brief  It clock the temperature sensing. It performes ADC conversion and 
  *         updates the average
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
  uint16_t TSNS_CalcAvTemp(CTSNS this)
  {
    return((((_CTSNS)this)->Methods_str.pTSNS_CalcAvTemp)(this));
  }
   
/**
  * @brief  It return latest averaged temperature measurement expressed in 
  *         u16Celsius
  * @param  this related object of class CTSNS
  * @retval uint16_t Latest averaged temperature measurement in u16Celsius
  */
  uint16_t TSNS_GetAvTemp_d(CTSNS this)
  {
    return(BCLASS_VARS.hAvTemp_d);
  }
  
/**
  * @brief  It returns latest averaged temperature measurement expressed in 
  *         Celsius degrees
  * @param  this related object of class CTSNS
  * @retval uint16_t Latest averaged temperature measurement in Celsius degrees
  */
  int16_t TSNS_GetAvTemp_C(CTSNS this)
  {
    return((((_CTSNS)this)->Methods_str.pTSNS_GetAvTemp_C)(this));
  }
  
/**
  * @brief  It returns MC_OVER_TEMP or MC_NO_ERROR depending on
  *         temperature sensor output and protection threshold values
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
  uint16_t TSNS_CheckTemp(CTSNS this)
  {
    return(BCLASS_VARS.hFaultState);
  }
  
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
