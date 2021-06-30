/**
  ******************************************************************************
  * @file    MotorPowerMeasurementClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of MotorPowerMeasurement class      
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
#ifndef __MOTORPOWERMEASUREMENTCLASS_H
#define __MOTORPOWERMEASUREMENTCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup MotorPowerMeasurement
  * @{
  */
  
/** @defgroup MotorPowerMeasurement_class_exported_types MotorPowerMeasurement class exported types
* @{
*/

/** 
  * @brief  Public MotorPowerMeasurement class definition 
  */
typedef struct CMPM_t *CMPM;

/** 
  * @brief  MotorPowerMeasurement class init struct definition  
  */
typedef void* pMPMInitStruct_t;
  
/**
* @}
*/

/** @defgroup MotorPowerMeasurement_class_exported_methods MotorPowerMeasurement class exported methods
  * @{
  */

/**
  * @brief Initializes all the object variables, usually it has to be called 
  *        once right after object creation.
  * @param this related object of class CMPM.
  * @param pMPMInitStruct the pointer of the init structure, required by derived
  *       class, up-casted to pMPMInitStruct_t.
  * @retval none.
  */
void MPM_Init(CMPM this, pMPMInitStruct_t pMPMInitStruct);

/**
  * @brief  It should be called before each motor restart. It clears the 
  *         measurement buffer and initialize the index.
  * @param this related object of class CMPM.
  * @retval none.
  */
void MPM_Clear(CMPM this);

/**
  * @brief  This method should be called with periodicity. It computes and 
  *         returns the measured motor power expressed in watt. It is also used
  *         to fill, with that measure, the buffer used to compute the average
  *         motor power. 
  * @param this related object of class CMPM.
  * @retval int16_t The measured motor power expressed in watt.
  */
int16_t MPM_CalcElMotorPower(CMPM this);

/**
  * @brief  This method is used to get the last measured motor power 
  *         (instantaneous value) expressed in watt.
  * @param this related object of class CMPM.
  * @retval int16_t The last measured motor power (instantaneous value) 
  *         expressed in watt.
  */
int16_t MPM_GetElMotorPowerW(CMPM this);

/**
  * @brief  This method is used to get the average measured motor power 
  *         expressed in watt.
  * @param this related object of class CMPM.
  * @retval int16_t The average measured motor power expressed in watt.
  */
int16_t MPM_GetAvrgElMotorPowerW(CMPM this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __MOTORPOWERMEASUREMENTCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
