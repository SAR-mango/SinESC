/**
  ******************************************************************************
  * @file    TemperatureSensorClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of Temperature Sensor class      
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TEMPERATURESENSORCLASS_H
#define __TEMPERATURESENSORCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PWMnCurrFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup TemperatureSensor
  * @{
  */
  
/** @defgroup Temperature_Sensor_class_exported_types Temperature Sensor class exported types
* @{
*/

/** 
  * @brief  Public temperature sensor class definition 
  */
typedef struct CTSNS_t *CTSNS;

/** 
  * @brief  Temperature sensor class parameters definition  
  */
typedef const struct
{
  SensorType_t  bSensorType;   /*!< It contains the information about the type 
                                   of instanced temperature sensor object.  
                                   It can be equal to REAL_SENSOR or 
                                   VIRTUAL_SENSOR */
}TempSensorParams_t, *pTempSensorParams_t;
  
/**
* @}
*/

/** @defgroup Temperature_sensor_class_exported_methods Temperature sensor class exported methods
  * @{
  */

/**
  * @brief  It initializes temperature sensing conversions. It must be called 
  *         only after current sensor initialization (PWMC_Init)
  * @param  this related object of class CTSNS
  * @param  oPWMnCurrentSensor CPWMC object to be used for regular conversions
  * @retval none
  */
void TSNS_Init(CTSNS this, CPWMC oPWMnCurrentSensor);

/**
  * @brief  It clears FW variable containing average temperature measurement 
  *         value
  * @param  this related object of class CTSNS
  * @retval none
  */
void TSNS_Clear(CTSNS this);

/**
  * @brief  It clock the temperature sensing. It performes ADC conversion and 
  *         updates the average. It returns MC_OVER_TEMP or 
  *         MC_NO_ERROR depending on temperature average value measurement 
  *         and protection threshold values
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
uint16_t TSNS_CalcAvTemp(CTSNS this);

/**
  * @brief  It return latest averaged temperature measurement expressed in 
  *         u16Celsius
  * @param  this related object of class CTSNS
  * @retval uint16_t Latest averaged temperature measurement in u16Celsius
  */
uint16_t TSNS_GetAvTemp_d(CTSNS this);

/**
  * @brief  It returns latest averaged temperature measurement expressed in 
  *         Celsius degrees
  * @param  this related object of class CTSNS
  * @retval int16_t Latest averaged temperature measurement in Celsius degrees
  */
int16_t TSNS_GetAvTemp_C(CTSNS this);


/**
  * @brief  It returns MC_OVER_TEMP or MC_NO_ERROR depending on
  *         temperature measurement and protection threshold values
  * @param  this related object of class CTSNS
  * @retval uint16_t Fault code error
  */
uint16_t TSNS_CheckTemp(CTSNS this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __TEMPERATURESENSORCLASS_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
