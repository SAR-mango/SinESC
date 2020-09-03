/**
  ******************************************************************************
  * @file    BusVoltageSensorClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of BusVoltageSensor class      
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
#ifndef __BUSVOLTAGESENSORCLASS_H
#define __BUSVOLTAGESENSORCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PWMnCurrFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */
  
/** @defgroup BusVoltageSensor_class_exported_types BusVoltageSensor class exported types
* @{
*/

/** 
  * @brief  Public BusVoltageSensor class definition 
  */
typedef struct CVBS_t *CVBS;

/** 
  * @brief  BusVoltageSensor class parameters definition  
  */
typedef const struct
{
  SensorType_t  bSensorType;   /*!< It contains the information about the type 
                                   of instanced bus voltage sensor object.  
                                   It can be equal to REAL_SENSOR or 
                                   VIRTUAL_SENSOR */
  uint16_t hConversionFactor;  /*!< It is used to convert bus voltage from 
                                   u16Volts into real Volts (V). 
                                   1 u16Volt = 65536/hConversionFactor Volts
                                   For real sensors hConversionFactor it's 
                                   equal to the product between the expected MCU
                                   voltage and the voltage sensing network 
                                   attenuation. For virtual sensors it must
                                   be equal to 500 */
}BusVoltageSensorParams_t, *pBusVoltageSensorParams_t;
  
/**
* @}
*/

/** @defgroup BusVoltageSensor_class_exported_methods BusVoltageSensor class exported methods
  * @{
  */
/**
  * @brief  It initializes bus voltage conversion. It must be called only after 
  *         current sensor initialization (PWMC_Init)
  * @param  this related object of class CVBS
  * @param  oPWMnCurrentSensor CPWMC object to be used for regular conversions
  * @retval none
  */
void VBS_Init(CVBS this, CPWMC oPWMnCurrentSensor);

/**
  * @brief  It clears bus voltage FW variable containing average bus voltage 
  *         value
  * @param  this related object of class CVBS
  * @retval none
  */
void VBS_Clear(CVBS this);

/**
  * @brief  It clocks the bus voltage reading, performes Vbus conversion 
  *         and updates the average 
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
uint16_t VBS_CalcAvVbus(CVBS this);

/**
  * @brief  It returns latest Vbus conversion result expressed in u16Volt
  * @param  this related object of class CVBS
  * @retval uint16_t Latest Vbus conversion result 
  */
uint16_t VBS_GetBusVoltage_d(CVBS this);

/**
  * @brief  It returns latest averaged Vbus measurement expressed in u16Volt
  * @param  this related object of class CVBS
  * @retval uint16_t Latest averaged Vbus measurement in digital value
  */
uint16_t VBS_GetAvBusVoltage_d(CVBS this);

/**
  * @brief  It returns latest averaged Vbus measurement expressed in Volts
  * @param  this related object of class CVBS
  * @retval uint16_t Latest averaged Vbus measurement in Volts
  */
uint16_t VBS_GetAvBusVoltage_V(CVBS this);


/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage measurement and protection threshold values
  * @param  this related object of class CVBS
  * @retval uint16_t Fault code error
  */
uint16_t VBS_CheckVbus(CVBS this);
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __BUSVOLTAGESENSORCLASS_H */
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
