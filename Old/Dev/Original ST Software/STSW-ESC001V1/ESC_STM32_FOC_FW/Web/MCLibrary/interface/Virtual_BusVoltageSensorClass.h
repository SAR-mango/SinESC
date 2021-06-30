/**
  ******************************************************************************
  * @file    Virtual_BusVoltageSensorClass.h
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
#ifndef __VIRTUAL_BUSVOLTAGESENSORCLASS_H
#define __VIRTUAL_BUSVOLTAGESENSORCLASS_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup BusVoltageSensor_Virtual
  * @{
  */

/** @defgroup Virtual_class_exported_types Virtual Vbus sensor class exported types
* @{
*/

/** 
  * @brief  Public Virtual Vbus sensor class definition
  */
typedef struct CVVBS_VBS_t *CVVBS_VBS;

/** 
  * @brief  Virtual Vbus sensor class parameters definition
  */
typedef const struct
{
  uint16_t hExpectedVbus_d;           /*!< Expected Vbus voltage expressed in 
                                           digital value 
                                          hOverVoltageThreshold(digital value)= 
                                          Over Voltage Threshold (V) * 65536 
                                                          / 500 */                                                                        
}VirtualParams_t, *pVirtualParams_t;
/**
  * @}
  */

/** @defgroup Virtual_class_exported_methods Virtual Vbus sensor class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class Virtual Vbus sensor
  * @param  pBusVoltageSensorParams pointer to an BusVoltageSensor parameters 
  *         structure
  * @param  pVirtualParams pointer to an Virtual Vbus sensor parameters structure
  * @retval CVVBS_VBS new instance of Virtual Vbus sensor object
  */
CVVBS_VBS VVBS_NewObject(pBusVoltageSensorParams_t pBusVoltageSensorParams, 
                                               pVirtualParams_t pVirtualParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__VIRTUAL_BUSVOLTAGESENSORCLASS_H*/
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
