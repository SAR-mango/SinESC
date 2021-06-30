/**
  ******************************************************************************
  * @file    CircleLimitationClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of CircleLimitation class      
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
#ifndef __CIRCLELIMITATIONCLASS_H
#define __CIRCLELIMITATIONCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup CircleLimitation
  * @{
  */
  
/** @defgroup CircleLimitation_class_exported_types CircleLimitation class exported types
* @{
*/

/** 
  * @brief  Public CircleLimitation class definition 
  */
typedef struct CCLM_t *CCLM;

/** 
  * @brief  CircleLimitation class parameters definition  
  */
typedef const struct
{
  uint16_t hMaxModule;                /*!< Circle limitation maximum allowed 
                                          module */
  uint16_t hCircle_limit_table[87];   /*!< Circle limitation table */
  uint8_t bStart_index;               /*!< Circle limitation table indexing 
                                            start */
} CircleLimitationParams_t, *pCircleLimitationParams_t;
  
/**
* @}
*/

/** @defgroup CircleLimitation_class_exported_methods CircleLimitation class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class CircleLimitation
  * @param  pCircleLimitationParams pointer to an CircleLimitation parameters structure
  * @retval CCLM new instance of CircleLimitation object
  */
CCLM CLM_NewObject(pCircleLimitationParams_t pCircleLimitationParams);

/**
  * @brief Check whether Vqd.qV_Component1^2 + Vqd.qV_Component2^2 <= 32767^2 
  *        and if not it applies a limitation keeping constant ratio 
  *        Vqd.qV_Component1 / Vqd.qV_Component2
  * @param  this related object of class CCLM
  * @param  Vqd Voltage in qd reference frame  
  * @retval Volt_Components Limited Vqd vector
  */
Volt_Components Circle_Limitation(CCLM this, Volt_Components Vqd);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __CIRCLELIMITATIONCLASS_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
