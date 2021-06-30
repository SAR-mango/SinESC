/**
  ******************************************************************************
  * @file    R1_F0XX_PWMnCurrFdbkClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.3.0
  * @date    22-Sep-2016 15:29
  * @brief   This file contains interface of R1_F0XX class      
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
#ifndef __R1_F0XX_PWMNCURRFDBKCLASS_H
#define __R1_F0XX_PWMNCURRFDBKCLASS_H

#include "R1_SD_PWMnCurrFdbkClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup PWMnCurrFdbk_R1_F0XX
  * @{
  */

/** @defgroup R1_F0XX_class_exported_types R1_F0XX class exported types
* @{
*/

/** 
  * @brief  Public R1_F0XX class definition
  */
typedef struct CR1F0XX_PWMC_t *CR1F0XX_PWMC;

/** @defgroup R1_F0XX_class_exported_methods R1_F0XX class exported methods
  * @{
  */
  
/**
  * @brief  Creates an object of the class R1_F0XX
  * @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
  * @param  pR1_F0XXParams pointer to an R1_F0XX parameters structure
  * @retval CR1F0XX_PWMC new instance of R1_F0XX object
  */
CR1F0XX_PWMC R1F0XX_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, pR1_SDParams_t pR1_SDParams);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__R1_F0XX_PWMNCURRFDBKCLASS_H*/

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
