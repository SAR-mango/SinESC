/**
  ******************************************************************************
  * @file    Virtual_TemperatureSensorClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of Virtual Vbus sensor class      
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
#ifndef __VIRTUAL_TEMPERATURESENSORCLASS_H
#define __VIRTUAL_TEMPERATURESENSORCLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup TemperatureSensor_Virtual
  * @{
  */

/** @defgroup VirtualTemp_class_exported_types Virtual Temperature sensor class exported types
* @{
*/

/** 
  * @brief  Public Virtual Temperature sensor class definition
  */
typedef struct CVTS_TSNS_t *CVTS_TSNS;

/** 
  * @brief  Virtual temperature sensor class parameters definition
  */
typedef const struct
{ 
  uint16_t hExpectedTemp_d;          /*!< Value returned by base class method
										  TSNS_GetAvTemp_d */
  int16_t  hExpectedTemp_C;          /*!< Expected temperature in degrees 
										  (value returned by base class method
										   TSNS_GetAvTemp_C) */
}VirtualTParams_t, *pVirtualTParams_t;
/**
  * @}
  */

/** @defgroup VirtualTemp_class_exported_methods Virtual Temperature sensor class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class Virtual temperature sensor
  * @param  pTempSensorParams pointer to an TempSensor parameters 
  *         structure
  * @param  pVirtualParams pointer to a virtual temperature sensor parameters structure
  * @retval CVTS_TSNS new instance of virtual temperature sensor object
  */
CVTS_TSNS VTS_NewObject(pTempSensorParams_t pTempSensorParams, 
                                               pVirtualTParams_t pVirtualParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__VIRTUAL_TEMPERATURESENSORCLASS_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
