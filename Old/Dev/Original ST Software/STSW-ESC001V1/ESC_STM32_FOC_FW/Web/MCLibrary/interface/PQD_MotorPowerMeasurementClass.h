/**
  ******************************************************************************
  * @file    PQD_MotorPowerMeasurementClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of PQD class      
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
#ifndef __PQD_MOTORPOWERMEASUREMENTCLASS_H
#define __PQD_MOTORPOWERMEASUREMENTCLASS_H

#include "BusVoltageSensorClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup MotorPowerMeasurement_PQD
  * @{
  */

/** @defgroup PQD_class_exported_types PQD class exported types
* @{
*/

/** 
  * @brief  Public PQD class definition
  */
typedef struct CPQD_MPM_t *CPQD_MPM;

/** 
  * @brief  PQD class parameters definition
  */
typedef const struct
{
  int32_t wConvFact; /* It is the conversion factor used to convert the 
                         variables expressed in digit into variables expressed 
                         in physical measurement unit. It is used to convert the
                         power in watts. It must be equal to 
                         (1000 * 3 * Vddµ)/(sqrt(3) * Rshunt * Aop) */
}PQDParams_t, *pPQDParams_t;

/** 
  * @brief  PQD class init struct definition  
  */
typedef struct {
	pFOCVars_t pFOCVars;    /*!< Pointer to FOC vars used by MPM.*/
	CVBS oVBS;              /*!< Bus voltage sensor object used by MPM.*/
} PQD_MPMInitStruct_t, *pPQD_MPMInitStruct_t;


/**
  * @}
  */

/** @defgroup PQD_class_exported_methods PQD class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class PQD
  * @param  pMotorPowerMeasurementParams pointer to an MotorPowerMeasurement parameters structure
  * @param  pPQDParams pointer to an PQD parameters structure
  * @retval CPQD_MPM new instance of PQD object
  */
CPQD_MPM PQD_NewObject(pPQDParams_t pPQDParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__PQD_MOTORPOWERMEASUREMENTCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
